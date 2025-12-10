---
description: "Explicación detallada de la sintaxis de separación por etapas que resuelve el 'infierno de one-liner' de RxJS. Al separar claramente la definición de streams, transformación y suscripción, y dar nombre a cada etapa, puedes escribir código reactivo fácil de depurar, probar y leer. Con ejemplos prácticos de refactorización."
---

# Infierno de one-liner y sintaxis de separación por etapas

La razón principal por la que el código RxJS parece un "infierno de one-liner" es porque **"definición de stream", "transformación" y "suscripción (efectos secundarios)" están mezclados**. Esto reduce significativamente la legibilidad y facilidad de depuración.

## Por qué ocurre el "infierno de one-liner"

### ❌ Código problemático común

```ts
import { fromEvent } from 'rxjs';
import { map, filter, debounceTime, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

fromEvent(document, 'click')
  .pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    debounceTime(300),
    switchMap(x => ajax(`/api?x=${x}`))
  )
  .subscribe(res => {
    if (res.status === 200) {
      console.log('OK');
    } else {
      handleError(res);
    }
  });

function handleError(res: any) {
  console.error('Error:', res);
}
```

### Problemas

| Problema | Impacto |
|---|---|
| **Líneas largas** | Los lectores se pierden |
| **Difícil de depurar** | Difícil verificar estados intermedios |
| **Difícil de probar** | Solo se puede probar el stream completo |
| **Estructura de procesamiento anidada** | Las bifurcaciones condicionales dentro de subscribe tienden a profundizarse |
| **No reutilizable** | El procesamiento del pipeline no se puede usar en otro lugar |


## Solución: Sintaxis de separación por etapas (Functional Style)

Organiza el código RxJS en una "composición de 3 etapas con relaciones claras".

1. **Definición de stream (source)** - Fuente de generación de datos
2. **Transformación de stream (pipeline)** - Procesamiento de transformación de datos
3. **Suscripción y efectos secundarios (subscription)** - Efectos secundarios como actualización de UI o logs


## Patrón recomendado: Sintaxis de separación por etapas

```ts
import { fromEvent } from 'rxjs';
import { map, filter, throttleTime } from 'rxjs';

// 1. Definición de Observable (fuente del stream)
const clicks$ = fromEvent(document, 'click');

// 2. Definición de pipeline (procesamiento de transformación de datos)
const processed$ = clicks$.pipe(
  map(event => (event as MouseEvent).clientX),
  filter(x => x > 100),
  throttleTime(200)
);

// 3. Procesamiento de suscripción (ejecución de efectos secundarios)
const subscription = processed$.subscribe({
  next: x => console.log('Posición del clic:', x),
  error: err => console.error(err),
  complete: () => console.log('Completado')
});
```

### Beneficios

| Beneficio | Detalle |
|---|---|
| **Significado claro por paso** | La responsabilidad de cada etapa es evidente de un vistazo |
| **Fácil de depurar** | Puedes verificar streams intermedios con `console.log` o `tap` |
| **Fácil de probar** | Puedes probar streams intermedios como `processed$` de forma unitaria |
| **Anidamiento superficial** | El procesamiento dentro de subscribe se simplifica |
| **Reutilizable** | Puedes extraer el procesamiento del pipeline como función |


## Variación: Separación de funciones (Modularización)

Cuando el procesamiento de transformación se alarga, **separa el pipeline como función**.

```ts
import { Observable } from 'rxjs';
import { map, filter, distinctUntilChanged } from 'rxjs';
import { fromEvent } from 'rxjs';

// Extraer procesamiento de pipeline como función
function transformClicks(source$: Observable<Event>): Observable<number> {
  return source$.pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    distinctUntilChanged()
  );
}

// Lado de uso
const clicks$ = fromEvent(document, 'click');
const xPosition$ = transformClicks(clicks$);
const subscription = xPosition$.subscribe(x => console.log(x));
```

**Punto clave:** Cuando extraes "cómo transformar" como función pura, **la facilidad de prueba explota**.


## Reglas de nomenclatura (Naming Rule)

Aclara la intención del código con nomenclatura apropiada.

| Etapa | Ejemplo de nomenclatura | Significado |
|---|---|---|
| **Source** | `clicks$`, `input$`, `routeParams$` | Fuente de eventos o datos |
| **Pipe** | `processed$`, `validInput$`, `apiResponse$` | Stream procesado |
| **Subscription** | `subscription`, `uiSubscription` | Efectos secundarios realmente ejecutados |

Añadiendo el **sufijo `$`** queda claro de un vistazo que "es un Observable".


## Escribir más declarativamente (RxJS 7 en adelante)

Extrae `pipe` como función y hazlo reutilizable.

```ts
import { pipe, fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Definir pipeline como función (reutilizable)
const processClicks = pipe(
  map((ev: MouseEvent) => ev.clientX),
  filter(x => x > 100)
);

const clicks$ = fromEvent(document, 'click');
const processed$ = clicks$.pipe(processClicks);
processed$.subscribe(x => console.log(x));
```

**Beneficio:** La lógica de procesamiento (`processClicks`) se puede reutilizar en otros streams.


## Before/After: Refactorización por patrón típico

Presentamos ejemplos de mejora en casos de uso reales.

### A. Evento UI → API → Actualización UI

#### ❌ Before (infierno de one-liner)

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, switchMap, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

interface ApiRes {
  items: string[];
  error?: string;
}

const button = document.getElementById('btn') as HTMLButtonElement;
const list = document.getElementById('list') as HTMLElement;

fromEvent(button, 'click').pipe(
  throttleTime(500),
  switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
  catchError(err => of({ items: [], error: err.message }))
).subscribe(res => {
  list.innerHTML = res.items.map(item => `<li>${item}</li>`).join('');
  if (res.error) alert(res.error);
});
```

#### ✅ After (separación por etapas + funcionalización)

```ts
import { fromEvent, pipe, of } from 'rxjs';
import { throttleTime, switchMap, map, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface ApiRes {
  items: string[];
}

interface Result {
  items: string[];
  error: string | null;
}

const button = document.getElementById('btn') as HTMLButtonElement;
const list = document.getElementById('list') as HTMLElement;

// 1) source
const clicks$ = fromEvent(button, 'click');

// 2) pipeline (extraído a función pura)
const loadItems = () =>
  pipe(
    throttleTime(500),
    switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
    map((res: ApiRes) => ({ items: res.items, error: null as string | null })),
    catchError(err => of({ items: [] as string[], error: String(err?.message ?? err) }))
  );

const result$ = clicks$.pipe(loadItems());

// 3) subscription (solo efectos secundarios)
const subscription = result$.subscribe(({ items, error }) => {
  renderList(items);
  if (error) toast(error);
});

function renderList(items: string[]) {
  list.innerHTML = items.map(item => `<li>${item}</li>`).join('');
}

function toast(message: string) {
  alert(message);
}
```

**Mejoras:**
- Procesamiento de pipeline `loadItems()` convertido en función pura
- Efectos secundarios (`renderList`, `toast`) concentrados en el lado de subscribe
- Fácil de probar y depurar


### B. Valor de formulario → Validación → Guardado API (guardado automático)

#### ❌ Before

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

fromEvent(input, 'input')
  .pipe(
    map((e: Event) => (e.target as HTMLInputElement).value),
    debounceTime(400),
    distinctUntilChanged(),
    filter(v => v.length >= 3),
    switchMap(v => ajax.post('/api/save', { v }))
  )
  .subscribe(
    () => console.log('OK'),
    err => alert(err.message)
  );
```

#### ✅ After (separación de responsabilidades + nomenclatura)

```ts
import { fromEvent, pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

// 1) source
const value$ = fromEvent<Event>(input, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value)
);

// 2) pipeline (validación)
const validate = () =>
  pipe(
    debounceTime(400),
    distinctUntilChanged(),
    filter((v: string) => v.length >= 3)
  );

// 2) pipeline (guardado automático)
const autosave = () =>
  pipe(
    switchMap((v: string) => ajax.post('/api/save', { v }))
  );

const save$ = value$.pipe(validate(), autosave());

// 3) subscription
const subscription = save$.subscribe({
  next: () => showSuccess(),
  error: (err) => showError(String(err?.message ?? err))
});

function showSuccess() {
  console.log('Guardado');
}

function showError(message: string) {
  alert(message);
}
```

**Mejoras:**
- Validación (`validate`) y guardado (`autosave`) separados
- Cada pipeline se vuelve reutilizable
- Pruebas fáciles (se puede probar validación y guardado individualmente)


### C. Caché + Actualización manual

```ts
import { merge, of, Subject } from 'rxjs';
import { switchMap, shareReplay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Item {
  id: number;
  name: string;
}

const refreshBtn = document.getElementById('refresh-btn') as HTMLButtonElement;

// 1) sources
const refresh$ = new Subject<void>();
const initial$ = of(void 0);

// 2) pipeline
const fetchItems$ = merge(initial$, refresh$).pipe(
  switchMap(() => ajax.getJSON<Item[]>('/api/items')),
  shareReplay({ bufferSize: 1, refCount: true }) // Memoización
);

// 3) subscription
const subscription = fetchItems$.subscribe(items => renderList(items));

// Recargar desde UI
refreshBtn?.addEventListener('click', () => refresh$.next());

function renderList(items: Item[]) {
  console.log('Items:', items);
}
```

**Puntos clave:**
- Carga automática inicial (`initial$`) y actualización manual (`refresh$`) separadas
- Último valor en caché con `shareReplay`
- Múltiples suscriptores comparten el mismo resultado


## Avanzado: Cuando quieres incorporar logs intermedios

Puedes observar cada etapa con `tap()`.

```ts
import { fromEvent } from 'rxjs';
import { map, tap } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

const processed$ = clicks$.pipe(
  tap(() => console.log('Clic ocurrido')),
  map(e => (e as MouseEvent).clientX),
  tap(x => console.log('Coordenada X:', x))
);

processed$.subscribe(x => console.log('Valor final:', x));
```

**Puntos clave:**
- `tap` es operador exclusivo para efectos secundarios
- Puedes verificar valores en cada etapa durante depuración
- Deberías eliminarlo en entorno de producción


## Demostración de facilidad de prueba

Con la separación por etapas, puedes **probar el procesamiento del pipeline de forma unitaria**.

### Ejemplo: Prueba de validación de entrada

```ts
// validate.ts
import { pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter } from 'rxjs';

export const validateQuery = () =>
  pipe(
    map((s: string) => s.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((s) => s.length >= 3)
  );
```

```ts
// validate.spec.ts
import { TestScheduler } from 'rxjs/testing';
import { validateQuery } from './validate';

describe('validateQuery', () => {
  it('trims, debounces, distincts, filters length>=3', () => {
    const scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });

    scheduler.run(({ hot, expectObservable }) => {
      // Entrada: " a ", "ab", "abc", "abc ", "abcd"
      const input = hot<string>('-a-b-c--d-e----|', {
        a: ' a ',
        b: 'ab',
        c: 'abc',
        d: 'abc ',
        e: 'abcd'
      });

      const output$ = input.pipe(validateQuery());

      // Esperado: solo 'abc' y 'abcd' pasan
      expectObservable(output$).toBe('--------c-----e-|', {
        c: 'abc',
        e: 'abcd'
      });
    });
  });
});
```

**Beneficios:**
- Procesamiento de pipeline se puede probar **individualmente**
- No depende de DOM/HTTP = **rápido y estable**
- Control del eje temporal con pruebas marble

Consulta [Métodos de prueba](/es/guide/testing/unit-tests) para detalles.


## Plantillas de instrucciones para GitHub Copilot

Colección de prompts utilizables en refactorizaciones reales.

### 1. Descomposición en composición de tres etapas

```
Refactoriza este código RxJS descomponiéndolo en 3 etapas "source / pipeline / subscription".
Requisitos:
- Nombrar Observables con sufijo $
- Extraer pipeline como función que retorna pipe(...) (ejemplo: validate(), loadItems())
- Concentrar efectos secundarios (actualización UI, console, toast) dentro de subscribe
- Insertar tap() en lugares apropiados para poder observar estados intermedios (con comentarios)
- Nombres de variables y funciones que transmitan el dominio
```

### 2. Clarificación de selección de operador

```
Quiero prevenir múltiples llamadas API por múltiples clics.
Propón cuál de switchMap/mergeMap/concatMap/exhaustMap usar actualmente,
y reemplaza con el operador correcto. Escribe la justificación en comentarios.

Directrices:
- Guardado de formularios es procesamiento secuencial (concatMap)
- Sugerencias de búsqueda descartan solicitudes antiguas (switchMap)
- Clics repetidos en botón prohíben ejecución doble (exhaustMap)
```

### 3. Patrón de guardado automático

```
Refactoriza el siguiente código a patrón de guardado automático:
- Entrada con debounceTime y distinctUntilChanged
- Guardado serializado con concatMap
- Efectos secundarios de notificación de éxito/fallo concentrados en lado de subscribe
- Funcionalizar transformación para facilitar pruebas
- Si es posible, cachear último estado con shareReplay
```

### 4. Caché + Actualización manual

```
Cambiar a patrón "carga automática inicial + actualización manual":
- Introducir refresh$ Subject
- merge(initial$, refresh$) → switchMap(fetch)
- Cachear último valor con shareReplay({bufferSize:1, refCount:true})
- Extraer pipe de fetch como función para reutilización
```


## Conclusión: Guía resumen para escribir legiblemente

| Ítem | Contenido recomendado |
|---|---|
| ✅ 1 | **Escribir separadamente** Observable, pipe y subscribe |
| ✅ 2 | Streams intermedios **mostrar significado con nombre de variable** |
| ✅ 3 | Pipes complejos **funcionalizarlos** |
| ✅ 4 | Hacer posible **verificación intermedia con tap()** |
| ✅ 5 | Hacer reutilizable con `processSomething = pipe(...)` |


## Resumen

- El **infierno de one-liner** ocurre cuando definición de stream, transformación y suscripción se mezclan
- **Sintaxis de separación por etapas** (Source → Pipeline → Subscription) clarifica responsabilidades
- **Funcionalizar pipeline** mejora facilidad de prueba y reutilización
- **Nomenclatura apropiada** (sufijo `$`, nombres de variables significativos) mejora legibilidad

## Secciones relacionadas

- **[Errores comunes y soluciones](/es/guide/anti-patterns/common-mistakes#13-過度な複雑化)** - Anti-patrón de excesiva complejidad
- **[Métodos de prueba](/es/guide/testing/unit-tests)** - Cómo probar código RxJS
- **[Comprensión de operadores](/es/guide/operators/)** - Cómo usar cada operador

## Próximos pasos

1. Buscar lugares en código existente que sean "infierno de one-liner"
2. Refactorizar con sintaxis de separación por etapas
3. Funcionalizar procesamiento de pipeline y escribir pruebas unitarias
4. Unificar en todo el equipo utilizando plantillas de instrucciones Copilot


> [!NOTE]
> Una guía más completa sobre "cómo escribir RxJS legible" se planea tratar en el futuro **Chapter 12: Patrones prácticos**.
