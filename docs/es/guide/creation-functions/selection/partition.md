---
description: "La Función de Creación partition divide un Observable en dos Observables basándose en una condición. Es conveniente para distribuir valores por condiciones como éxito/fallo, válido/inválido, impar/par. Explicamos la implementación segura de tipos en TypeScript y la diferencia con filter()."
---

# partition - Dividir en dos streams por condición

`partition` es una Función de Creación que **divide un Observable en dos Observables** basándose en una condición.
Especificas la condición con una función predicado (predicate), y puedes obtener los valores que cumplen la condición y los que no la cumplen como streams separados.

## Sintaxis básica y uso

```ts
import { partition, of } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Dividir en pares e impares
const [evens$, odds$] = partition(source$, (value) => value % 2 === 0);

evens$.subscribe((value) => console.log('Par:', value));
// Salida: Par: 2, Par: 4, Par: 6

odds$.subscribe((value) => console.log('Impar:', value));
// Salida: Impar: 1, Impar: 3, Impar: 5
```

- `partition` devuelve **un array que contiene dos Observables**
- `[0]`: Stream de valores que cumplen la condición
- `[1]`: Stream de valores que no cumplen la condición

[🌐 Documentación Oficial RxJS - `partition`](https://rxjs.dev/api/index/function/partition)

## Patrones de uso típicos

- **Procesamiento de bifurcación éxito/fallo** (distribución por código de estado HTTP)
- **Distribución de eventos** (clic izquierdo/clic derecho)
- **Clasificación de datos** (válido/inválido, adulto/menor, etc.)
- **División de stream basada en condiciones**

## Ejemplo de código práctico (con UI)

Al hacer clic en un botón, el procesamiento se bifurca según si las coordenadas del clic están en la mitad izquierda o derecha de la pantalla.

```ts
import { partition, fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Crear áreas de salida
const leftArea = document.createElement('div');
leftArea.innerHTML = '<h3>Clic Izquierdo</h3><ul id="left-list"></ul>';
leftArea.style.float = 'left';
leftArea.style.width = '45%';
leftArea.style.padding = '10px';
leftArea.style.background = '#e3f2fd';
document.body.appendChild(leftArea);

const rightArea = document.createElement('div');
rightArea.innerHTML = '<h3>Clic Derecho</h3><ul id="right-list"></ul>';
rightArea.style.float = 'right';
rightArea.style.width = '45%';
rightArea.style.padding = '10px';
rightArea.style.background = '#fce4ec';
document.body.appendChild(rightArea);

// Eventos de clic
const clicks$ = fromEvent<MouseEvent>(document, 'click');

// Coordenada X central de la pantalla
const centerX = window.innerWidth / 2;

// Dividir en mitad izquierda y mitad derecha
const [leftClicks$, rightClicks$] = partition(
  clicks$,
  (event) => event.clientX < centerX
);

// Procesar clics izquierdos
leftClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const leftList = document.getElementById('left-list')!;
  const li = document.createElement('li');
  li.textContent = `Coordenadas: (${pos.x}, ${pos.y})`;
  leftList.appendChild(li);
});

// Procesar clics derechos
rightClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const rightList = document.getElementById('right-list')!;
  const li = document.createElement('li');
  li.textContent = `Coordenadas: (${pos.x}, ${pos.y})`;
  rightList.appendChild(li);
});
```

- Al hacer clic en la pantalla, se registra en la lista izquierda o derecha según la posición del clic.
- Puedes crear dos streams independientes a partir de una fuente.

## Ejemplo práctico: Procesamiento de bifurcación de respuesta API

Ejemplo de dividir éxito y fallo por código de estado HTTP

```ts
import { partition, from, of } from 'rxjs';
import { mergeMap, map, catchError, share } from 'rxjs';

interface ApiResponse {
  status: number;
  data?: any;
  error?: string;
}

// Llamadas API ficticias
const apiCalls$ = from([
  fetch('/api/users/1'),
  fetch('/api/users/999'), // Usuario que no existe
  fetch('/api/users/2'),
]);

// Procesar Response y convertir a ApiResponse
const responses$ = apiCalls$.pipe(
  mergeMap(fetchPromise => from(fetchPromise)),
  mergeMap(response =>
    from(response.json()).pipe(
      map(data => ({
        status: response.status,
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : (data.message || 'Error')
      } as ApiResponse)),
      catchError(err => of({
        status: response.status,
        data: undefined,
        error: err.message || 'Error al parsear respuesta'
      } as ApiResponse))
    )
  ),
  share() // Manejar las 2 suscripciones de partition
);

// Dividir en éxito (rango 200) y fallo (otros)
const [success$, failure$] = partition(
  responses$,
  (response: ApiResponse) => response.status >= 200 && response.status < 300
);

// Procesar respuestas exitosas
success$.subscribe((response) => {
  console.log('✅ Éxito:', response.data);
  // Mostrar datos de éxito en UI
});

// Procesar respuestas fallidas
failure$.subscribe((response) => {
  console.error('❌ Fallo:', response.error);
  // Mostrar mensaje de error
});
```

## Comparación con filter

### Diferencias básicas

| Método | Descripción | Salida | Caso de uso |
|------|------|------|--------------|
| `partition` | Divide una fuente en dos streams | 2 Observables | Cuando quieres usar ambos streams **simultáneamente** |
| `filter` | Deja pasar solo los valores que cumplen la condición | 1 Observable | Cuando solo necesitas un stream |

### Ejemplos específicos de uso

**Usa partition cuando procesas ambos streams simultáneamente**

```ts
import { partition, interval } from 'rxjs';
import { map, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Éxito</h4><ul id="success-list"></ul>';
successArea.style.float = 'left';
successArea.style.width = '45%';
output.appendChild(successArea);

const failureArea = document.createElement('div');
failureArea.innerHTML = '<h4 style="color: red;">❌ Fallo</h4><ul id="failure-list"></ul>';
failureArea.style.float = 'right';
failureArea.style.width = '45%';
output.appendChild(failureArea);

// Stream aleatorio de éxito/fallo
const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Tarea${i + 1}`
  }))
);

// ✅ partition - Procesar éxito y fallo simultáneamente
const [success$, failure$] = partition(tasks$, task => task.success);

success$.subscribe(task => {
  const successList = document.getElementById('success-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  successList.appendChild(li);
});

failure$.subscribe(task => {
  const failureList = document.getElementById('failure-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  failureList.appendChild(li);
});
```

**Usa filter cuando solo necesitas un stream**

```ts
import { interval } from 'rxjs';
import { map, take, filter } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Mostrar solo éxitos</h4><ul id="success-only"></ul>';
output.appendChild(successArea);

const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Tarea${i + 1}`
  }))
);

// ✅ filter - Procesar solo éxitos (ignorar fallos)
tasks$
  .pipe(filter(task => task.success))
  .subscribe(task => {
    const successList = document.getElementById('success-only')!;
    const li = document.createElement('li');
    li.textContent = task.message;
    successList.appendChild(li);
  });
```

**Comparación de usar filter dos veces vs partition**

```ts
import { of } from 'rxjs';
import { filter } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// ❌ Usar filter dos veces - La fuente puede ejecutarse dos veces
const evens1$ = numbers$.pipe(filter(n => n % 2 === 0));
const odds1$ = numbers$.pipe(filter(n => n % 2 !== 0));

evens1$.subscribe(n => console.log('Par:', n));
odds1$.subscribe(n => console.log('Impar:', n));
// Problema: Si numbers$ es un cold observable, se ejecuta dos veces

// ✅ Usar partition - Crear ambos streams con una ejecución
const [evens2$, odds2$] = partition(numbers$, n => n % 2 === 0);

evens2$.subscribe(n => console.log('Par:', n));
odds2$.subscribe(n => console.log('Impar:', n));
// Ventaja: Crear eficientemente dos streams desde una fuente
```

**Usa filter cuando quieres bifurcar dentro de un pipeline**

```ts
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

interface User {
  id: number;
  name: string;
  age: number;
  isActive: boolean;
}

const users$ = from([
  { id: 1, name: 'Alice', age: 25, isActive: true },
  { id: 2, name: 'Bob', age: 30, isActive: false },
  { id: 3, name: 'Carol', age: 35, isActive: true }
]);

// ❌ partition es una Función de Creación, no se puede usar dentro del pipeline
// users$.pipe(
//   map(user => user.name),
//   partition(name => name.startsWith('A')) // Error
// );

// ✅ Usar filter - Se puede usar dentro del pipeline
users$
  .pipe(
    filter(user => user.isActive),  // Solo usuarios activos
    map(user => user.name)           // Extraer nombres
  )
  .subscribe(console.log);
// Salida: Alice, Carol
```

### Resumen

| Situación | Método recomendado | Razón |
|------|--------------|------|
| Quieres procesar **ambos** éxito y fallo | `partition` | Puedes crear dos streams con una ejecución de la fuente |
| Quieres procesar **solo** éxitos | `filter` | Simple y claro |
| Quieres bifurcar condicionalmente dentro de un pipeline | `filter` | `partition` no se puede usar porque es una Función de Creación |
| Quieres bifurcar en 3 o más con condiciones complejas | `groupBy` | Puede dividir en múltiples grupos |

## Precauciones

### 1. Suscribirse a ambos streams

Los dos Observables creados con `partition` **comparten la fuente original**.
Si no te suscribes a ambos, el stream original puede no procesarse completamente.

```ts
const [success$, failure$] = partition(source$, predicate);

// Suscribirse a ambos
success$.subscribe(handleSuccess);
failure$.subscribe(handleFailure);
```

### 2. La fuente se ejecuta dos veces

`partition` se suscribe internamente a la fuente original dos veces.
Ten cuidado si hay efectos secundarios.

```ts
let callCount = 0;
const source$ = new Observable(observer => {
  callCount++;
  console.log(`Recuento de suscripciones: ${callCount}`);
  observer.next(1);
  observer.complete();
});

const [a$, b$] = partition(source$, n => n > 0);
a$.subscribe(); // Recuento de suscripciones: 1
b$.subscribe(); // Recuento de suscripciones: 2
```

Usa `share()` para evitar efectos secundarios.

```ts
import { share } from 'rxjs';

const shared$ = source$.pipe(share());
const [a$, b$] = partition(shared$, n => n > 0);
```

### 3. No se proporciona como Pipeable Operator

Desde RxJS 7, `partition` se proporciona **solo como Función de Creación**.
No se puede usar dentro de un pipeline.

```ts
// ❌ No posible
source$.pipe(
  partition(n => n % 2 === 0) // Error
);

// ✅ Uso correcto
const [evens$, odds$] = partition(source$, n => n % 2 === 0);
```

## Patrones Alternativos

Si quieres bifurcar dentro de un pipeline, usa `filter`.

```ts
const source$ = of(1, 2, 3, 4, 5, 6);

const evens$ = source$.pipe(filter(n => n % 2 === 0));
const odds$ = source$.pipe(filter(n => n % 2 !== 0));

// O compartir la fuente con share
const shared$ = source$.pipe(share());
const evens$ = shared$.pipe(filter(n => n % 2 === 0));
const odds$ = shared$.pipe(filter(n => n % 2 !== 0));
```

## Operadores Relacionados

- [`filter`](../../operators/filtering/filter.md) - Pasar solo valores que cumplen la condición
- [`groupBy`](../../operators/transformation/groupBy.md) - Dividir en múltiples grupos
- [`share`](../../operators/multicasting/share.md) - Compartir la fuente

## Resumen

`partition` es una herramienta poderosa que divide un Observable en dos basándose en una condición.

- ✅ Ideal para procesamiento de bifurcación éxito/fallo
- ✅ Crea dos streams independientes
- ⚠️ La fuente se suscribe dos veces (cuidado con efectos secundarios)
- ⚠️ No se proporciona como Pipeable Operator
