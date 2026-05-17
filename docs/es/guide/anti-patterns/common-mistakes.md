---
description: Esta página detalla 15 anti-patrones comunes al usar RxJS con TypeScript y sus respectivas soluciones. Aprenda anti-patrones prácticos como publicación externa de Subject, subscribe anidado, fugas de memoria y mal uso de shareReplay.
---

# Errores Comunes y Cómo Tratarlos

Esta página detalla 15 anti-patrones comunes al usar RxJS con TypeScript y sus respectivas soluciones.

## Tabla de Contenidos

1. [Publicación externa de Subject](#1-publicacion-externa-de-subject)
2. [Subscribe anidado (callback hell)](#2-subscribe-anidado-callback-hell)
3. [Olvido de unsubscribe (fuga de memoria)](#3-olvido-de-unsubscribe-fuga-de-memoria)
4. [Mal uso de shareReplay](#4-mal-uso-de-sharereplay)
5. [Efectos secundarios en map](#5-efectos-secundarios-en-map)
6. [Ignorar diferencias entre Observable Cold/Hot](#6-ignorar-diferencias-entre-observable-cold-hot)
7. [Mezcla impropia de Promise y Observable](#7-mezcla-impropia-de-promise-y-observable)
8. [Ignorar backpressure](#8-ignorar-backpressure)
9. [Supresión de errores](#9-supresion-de-errores)
10. [Fugas de suscripción de eventos DOM](#10-fugas-de-suscripcion-de-eventos-dom)
11. [Falta de seguridad de tipos (uso excesivo de any)](#11-falta-de-seguridad-de-tipos-uso-excesivo-de-any)
12. [Selección impropia de operadores](#12-seleccion-impropia-de-operadores)
13. [Sobrecomplicación](#13-sobrecomplicacion)
14. [Cambios de estado en subscribe](#14-cambios-de-estado-en-subscribe)
15. [Falta de pruebas](#15-falta-de-pruebas)


## 1. Publicación externa de Subject

### Problema

Si Subject se expone tal cual, se llamará `next()` desde el exterior y la gestión del estado será impredecible.

### ❌ Ejemplo malo

```ts
import { Subject } from 'rxjs';

// Exportar Subject tal cual
export const cartChanged$ = new Subject<void>();

// Cualquiera puede llamar next() desde otro archivo
cartChanged$.next(); // Puede ser llamado en momentos inesperados
```

### ✅ Ejemplo bueno

```ts
import { BehaviorSubject, Observable } from 'rxjs';

class CartStore {
  private readonly _items$ = new BehaviorSubject<string[]>([]);

  // Publicar como Observable de solo lectura
  readonly items$: Observable<string[]> = this._items$.asObservable();

  // Los cambios de estado se controlan mediante métodos dedicados
  add(item: string): void {
    this._items$.next([...this._items$.value, item]);
  }

  remove(item: string): void {
    this._items$.next(
      this._items$.value.filter(i => i !== item)
    );
  }
}

export const cartStore = new CartStore();
```

### Explicación

- Convertir a Observable de solo lectura con `asObservable()`
- Permitir cambios de estado solo mediante métodos dedicados
- Mejora la trazabilidad de los cambios y facilita la depuración


## 2. Subscribe anidado (callback hell)

### Problema

Llamar más `subscribe` en un `subscribe` causa callback hell, lo que complica el manejo de errores y el procesamiento de cancelación.

### ❌ Ejemplo malo

```ts
import { of } from 'rxjs';

// Simulación de llamada API
function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
}

// Subscribe anidado
apiA().subscribe(a => {
  apiB(a.id).subscribe(b => {
    apiC(b.token).subscribe(result => {
      console.log('done', result);
    });
  });
});
```

### ✅ Ejemplo bueno

```ts
import { of } from 'rxjs';
import { switchMap } from 'rxjs';

function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
};


// Aplanar usando operadores de orden superior
apiA().pipe(
  switchMap(a => apiB(a.id)),
  switchMap(b => apiC(b.token))
).subscribe(result => {
  console.log('done', result);
});
```

### Explicación

- Usar operadores de orden superior como `switchMap`, `mergeMap`, y `concatMap`
- El manejo de errores se puede hacer en un solo lugar
- Solo una vez para desuscribirse
- Mejora de la legibilidad del código


## 3. Olvido de unsubscribe (fuga de memoria)

### Problema

No desuscribirse de un stream infinito (por ejemplo, un event listener) causa una fuga de memoria.

### ❌ Ejemplo malo

```ts
import { fromEvent } from 'rxjs';

// Durante la inicialización del componente
function setupResizeHandler() {
  fromEvent(window, 'resize').subscribe(() => {
    console.log('resized');
  });
  // ¡No se desuscribe!
}

// El event listener permanece incluso después de que el componente se destruye
```

### ✅ Ejemplo bueno

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeUntil, finalize } from 'rxjs';

class MyComponent {
  private readonly destroy$ = new Subject<void>();

  ngOnInit(): void {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$),
      finalize(() => console.log('cleanup'))
    ).subscribe(() => {
      console.log('resized');
    });
  }

  ngOnDestroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### ✅ Otro ejemplo bueno (cómo usar Subscription)

```ts
import { fromEvent, Subscription } from 'rxjs';

class MyComponent {
  private subscription = new Subscription();

  ngOnInit(): void {
    this.subscription.add(
      fromEvent(window, 'resize').subscribe(() => {
        console.log('resized');
      })
    );
  }

  ngOnDestroy(): void {
    this.subscription.unsubscribe();
  }
}
```

### Explicación

- Se recomienda el patrón `takeUntil` (declarativo y sin ambigüedades)
- La gestión manual con Subscription también es efectiva
- Siempre desuscribirse al destruir componentes


## 4. Mal uso de shareReplay

### Problema

Usar `shareReplay` sin comprender cómo funciona puede resultar en la reproducción de datos antiguos y fugas de memoria.

### ❌ Ejemplo malo

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Hacer el tamaño del buffer ilimitado
const shared$ = interval(1000).pipe(
  shareReplay() // Por defecto es buffer ilimitado
);

// Los valores permanecen en memoria incluso cuando no hay suscriptores
```

### ✅ Ejemplo bueno

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Especificar explícitamente el tamaño del buffer y el conteo de referencias
const shared$ = interval(1000).pipe(
  take(10),
  shareReplay({
    bufferSize: 1,
    refCount: true // Liberar recursos cuando no hay más suscriptores
  })
);
```

### Explicación

- Especificar explícitamente `bufferSize` (generalmente 1)
- `refCount: true` para liberación automática cuando no hay más suscriptores
- `shareReplay({ bufferSize: 1, refCount: true })` es seguro para streams que se completan, como solicitudes HTTP


## 5. Efectos secundarios en map

### Problema

Cambiar el estado en el operador `map` causa un comportamiento impredecible.

### ❌ Ejemplo malo

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

let counter = 0;

const source$ = of(1, 2, 3).pipe(
  map(value => {
    counter++; // ¡Efecto secundario!
    return value * 2;
  })
);

source$.subscribe(console.log);
source$.subscribe(console.log); // counter aumenta inesperadamente
```

### ✅ Ejemplo bueno

```ts
import { of } from 'rxjs';
import { map, tap, scan } from 'rxjs';

// Solo transformación pura
const source$ = of(1, 2, 3).pipe(
  map(value => value * 2)
);

// Separar efectos secundarios con tap
const withLogging$ = source$.pipe(
  tap(value => console.log('Processing:', value))
);

// Usar scan para acumular estado
const withCounter$ = of(1, 2, 3).pipe(
  scan((acc, value) => ({ count: acc.count + 1, value }), { count: 0, value: 0 })
);
```

### Explicación

- `map` se usa como función pura
- Los efectos secundarios (logs, llamadas API, etc.) se separan en `tap`
- Usar `scan` o `reduce` para acumular estado


## 6. Ignorar diferencias entre Observable Cold/Hot

### Problema

Usar un Observable sin comprender si es Cold o Hot puede llevar a ejecuciones duplicadas y comportamiento inesperado.

### ❌ Ejemplo malo

```ts
import { ajax } from 'rxjs/ajax';

// Cold Observable - La solicitud HTTP se ejecuta por suscripción
const data$ = ajax.getJSON('https://api.example.com/data');

data$.subscribe(console.log); // Solicitud 1
data$.subscribe(console.log); // Solicitud 2 (duplicación innecesaria)
```

### ✅ Ejemplo bueno

```ts
import { ajax } from 'rxjs/ajax';
import { shareReplay } from 'rxjs';

// Convertir a Hot Observable y compartir
const data$ = ajax.getJSON('https://api.example.com/data').pipe(
  shareReplay({ bufferSize: 1, refCount: true })
);

data$.subscribe(console.log); // Solicitud 1
data$.subscribe(console.log); // Usar resultado en caché
```

### Explicación

- Cold Observable: ejecutado por suscripción (`of`, `from`, `fromEvent`, `ajax`, etc.)
- Hot Observable: ejecutado independientemente de la suscripción (Subject, Observable multicast, etc.)
- Cold se puede convertir a Hot con `share` / `shareReplay`


## 7. Mezcla impropia de Promise y Observable

### Problema

Mezclar Promise y Observable sin conversión adecuada conduce a un manejo incompleto de errores y cancelación.

### ❌ Ejemplo malo

```ts
import { from } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Usar Promise tal cual
from(fetchData()).subscribe(data => {
  fetchData().then(moreData => { // Promise anidado
    console.log(data, moreData);
  });
});
```

### ✅ Ejemplo bueno

```ts
import { from } from 'rxjs';
import { switchMap } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Convertir Promise a Observable y unificar
from(fetchData()).pipe(
  switchMap(() => from(fetchData()))
).subscribe(moreData => {
  console.log(moreData);
});
```

### Explicación

- Convertir Promise a Observable con `from`
- Procesamiento uniforme en el pipeline de Observable
- Manejo de errores y cancelación más fáciles


## 8. Ignorar backpressure

### Problema

El manejo no controlado de eventos de alta frecuencia resulta en un bajo rendimiento.

### ❌ Ejemplo malo

```ts
import { fromEvent } from 'rxjs';

// Procesar eventos de entrada tal cual
fromEvent(document.getElementById('search'), 'input').subscribe(event => {
  // Llamada API en cada entrada (sobrecarga)
  searchAPI((event.target as HTMLInputElement).value);
});

function searchAPI(query: string): void {
  console.log('Searching for:', query);
}
```

### ✅ Ejemplo bueno

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, distinctUntilChanged, map, switchMap } from 'rxjs';

// Aplicar debounce y cancelación
fromEvent(document.getElementById('search'), 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // Esperar 300ms
  distinctUntilChanged(), // Solo cuando el valor cambia
  switchMap(query => searchAPI(query)) // Cancelar solicitudes antiguas
).subscribe(results => {
  console.log('Results:', results);
});
```

### Explicación

- `debounceTime` para esperar un cierto período de tiempo
- `throttleTime` limita la frecuencia máxima
- `distinctUntilChanged` para excluir duplicados
- Cancelar solicitudes antiguas con `switchMap`


## 9. Supresión de errores

### Problema

No manejar los errores correctamente dificulta la depuración y degrada la experiencia del usuario.

### ❌ Ejemplo malo

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

// Ignorar error
ajax.getJSON('https://api.example.com/data').pipe(
  catchError(() => of(null)) // La información del error se pierde
).subscribe(data => {
  console.log(data); // Causa desconocida incluso si llega null
});
```

### ✅ Ejemplo bueno

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

interface ApiResponse {
  data: unknown;
  error?: string;
}

ajax.getJSON<ApiResponse>('https://api.example.com/data').pipe(
  catchError((error: unknown) => {
    console.error('API Error:', error);
    // Notificar al usuario
    showErrorToast('No se pudieron recuperar los datos');
    // Devolver valor alternativo con información del error
    return of({ data: null, error: (error instanceof Error ? error.message : String(error)) } as ApiResponse);
  })
).subscribe((response) => {
  if (response.error) {
    console.log('Modo alternativo debido a:', response.error);
  }
});

function showErrorToast(message: string): void {
  console.log('Toast:', message);
}
```

### Explicación

- Registrar errores
- Proporcionar retroalimentación al usuario
- Devolver valores alternativos con información del error
- Considerar estrategias de reintento (`retry`, `retryWhen`)


## 10. Fugas de suscripción de eventos DOM

### Problema

No liberar correctamente los event listeners del DOM resulta en fugas de memoria.

### ❌ Ejemplo malo

```ts
import { fromEvent } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;

  constructor() {
    this.button = document.createElement('button');

    // Registrar event listener
    fromEvent(this.button, 'click').subscribe(() => {
      console.log('clicked');
    });

    // No se desuscribe
  }

  destroy(): void {
    this.button.remove();
    // El listener permanece
  }
}
```

### ✅ Ejemplo bueno

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;
  private readonly destroy$ = new Subject<void>();

  constructor() {
    this.button = document.createElement('button');

    fromEvent(this.button, 'click').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => {
      console.log('clicked');
    });
  }

  destroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
    this.button.remove();
  }
}
```

### Explicación

- Desuscribirse de forma confiable con el patrón `takeUntil`
- Activar `destroy$` cuando el componente se destruye
- Liberar listeners antes de eliminar elementos DOM


## 11. Falta de seguridad de tipos (uso excesivo de any)

### Problema

El uso excesivo de `any` desactiva la verificación de tipos de TypeScript y es propenso a errores en tiempo de ejecución.

### ❌ Ejemplo malo

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

function fetchUser(): Observable<any> {
  return new Observable(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// La verificación de tipos no funciona
fetchUser().pipe(
  map(user => user.naem) // ¡Error tipográfico! No se notará hasta el tiempo de ejecución
).subscribe(console.log);
```

### ✅ Ejemplo bueno

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

interface User {
  name: string;
  age: number;
}

function fetchUser(): Observable<User> {
  return new Observable<User>(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// La verificación de tipos funciona
fetchUser().pipe(
  map(user => user.name) // Detección de errores en tiempo de compilación
).subscribe(console.log);
```

### Explicación

- Definir interfaces y alias de tipos
- Parámetros de tipo explícitos para `Observable<T>`
- Aprovechar al máximo la inferencia de tipos de TypeScript


## 12. Selección impropia de operadores

### Problema

Usar un operador que no es adecuado para el propósito conduce a un comportamiento ineficiente o inesperado.

### ❌ Ejemplo malo

```ts
import { fromEvent } from 'rxjs';
import { mergeMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Buscar en cada clic del botón (las solicitudes antiguas no se cancelan)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  mergeMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### ✅ Ejemplo bueno

```ts
import { fromEvent } from 'rxjs';
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Procesar solo la solicitud más reciente (las solicitudes antiguas se cancelan automáticamente)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  switchMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### Distinción entre los principales operadores de orden superior

| Operador | Uso |
|---|---|
| `switchMap` | Procesar solo el stream más reciente (búsqueda, autocompletado) |
| `mergeMap` | Procesamiento concurrente (cualquier orden) |
| `concatMap` | Procesamiento secuencial (el orden es importante) |
| `exhaustMap` | Ignorar nueva entrada durante la ejecución (prevenir pulsaciones consecutivas de botones) |

### Explicación

- Comprender el comportamiento de cada operador
- Seleccionar el adecuado para su caso de uso
- Consulte [Operadores de Transformación](/es/guide/operators/transformation/) para más detalles


## 13. Sobrecomplicación

### Problema

Un caso en el que RxJS sobrecomplica un proceso que podría escribirse de forma simple.

### ❌ Ejemplo malo

```ts
import { Observable, of } from 'rxjs';
import { map, mergeMap, toArray } from 'rxjs';

// Complicar la transformación simple de arrays con RxJS
function doubleNumbers(numbers: number[]): Observable<number[]> {
  return of(numbers).pipe(
    mergeMap(arr => of(...arr)),
    map(n => n * 2),
    toArray()
  );
}
```

### ✅ Ejemplo bueno

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// JavaScript regular es suficiente para el procesamiento de arrays
function doubleNumbers(numbers: number[]): number[] {
  return numbers.map(n => n * 2);
}

// Usar RxJS para procesamiento asíncrono y basado en eventos
const button = document.getElementById('calc-btn') as HTMLButtonElement;
const numbers = [1, 2, 3, 4, 5];

fromEvent(button, 'click').pipe(
  map(() => doubleNumbers(numbers))
).subscribe(result => console.log(result));
```

### Explicación

- RxJS se usa para procesamiento asíncrono y streams de eventos
- JavaScript regular es suficiente para procesamiento síncrono de arrays
- Considerar el equilibrio entre complejidad y beneficios


## 14. Cambios de estado en subscribe

### Problema

Cambiar el estado directamente dentro de `subscribe` es difícil de probar y causa errores.

### ❌ Ejemplo malo

```ts
import { interval } from 'rxjs';

class Counter {
  count = 0;

  start(): void {
    interval(1000).subscribe(() => {
      this.count++; // Cambio de estado dentro de subscribe
      this.updateUI();
    });
  }

  updateUI(): void {
    console.log('Count:', this.count);
  }
}
```

### ✅ Ejemplo bueno

```ts
import { interval, BehaviorSubject } from 'rxjs';
import { scan, tap } from 'rxjs';

class Counter {
  private readonly count$ = new BehaviorSubject<number>(0);

  start(): void {
    interval(1000).pipe(
      scan(acc => acc + 1, 0),
      tap(count => this.count$.next(count))
    ).subscribe();

    // La UI se suscribe a count$
    this.count$.subscribe(count => this.updateUI(count));
  }

  updateUI(count: number): void {
    console.log('Count:', count);
  }
}
```

### Explicación

- El estado se gestiona mediante `BehaviorSubject` y `scan`
- `subscribe` se usa como disparador
- Diseño testeable y reactivo


## 15. Falta de pruebas

### Problema

Desplegar código RxJS en producción sin pruebas es propenso a regresiones.

### ❌ Ejemplo malo

```ts
import { interval } from 'rxjs';
import { map, filter } from 'rxjs';

// Desplegar sin pruebas
export function getEvenNumbers() {
  return interval(1000).pipe(
    filter(n => n % 2 === 0),
    map(n => n * 2)
  );
}
```

### ✅ Ejemplo bueno

```ts
import { TestScheduler } from 'rxjs/testing';
import { getEvenNumbers } from './numbers';

describe('getEvenNumbers', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('should emit only even numbers doubled', () => {
    scheduler.run(({ expectObservable }) => {
      const expected = '1s 0 1s 4 1s 8';
      expectObservable(getEvenNumbers()).toBe(expected);
    });
  });
});
```

### Explicación

- Marble Testing con TestScheduler
- El procesamiento asíncrono se puede probar de forma síncrona
- Consulte [Técnicas de Prueba](/es/guide/testing/unit-tests) para más detalles


## Resumen

Al comprender y evitar estos 15 anti-patrones, puede escribir código RxJS más robusto y mantenible.

## Referencias

Esta colección de anti-patrones ha sido preparada con referencia a las siguientes fuentes confiables.

### Repositorio de Documentación Oficial
- **[Documentación Oficial de RxJS](https://rxjs.dev/)** - Referencia oficial de operadores y API
- **[GitHub Issue #5931](https://github.com/ReactiveX/rxjs/issues/5931)** - Discusión sobre el problema de fuga de memoria de shareReplay

### Anti-patrones y mejores prácticas
- **[RxJS in Angular - Antipattern 1: Nested subscriptions](https://www.thinktecture.com/en/angular/rxjs-antipattern-1-nested-subs/)** - Thinktecture AG
- **[RxJS in Angular - Antipattern 2: Stateful Streams](https://www.thinktecture.com/en/angular/rxjs-antipattern-2-state/)** - Thinktecture AG
- **[RxJS Best Practices in Angular 16 (2025)](https://www.infoq.com/articles/rxjs-angular16-best-practices/)** - InfoQ (Mayo 2025)
- **[RxJS: Why memory leaks occur when using a Subject](https://angularindepth.com/posts/1433/rxjs-why-memory-leaks-occur-when-using-a-subject)** - Angular In Depth
- **[RxJS Antipatterns](https://brianflove.com/posts/2017-11-01-ngrx-anti-patterns/)** - Brian Love

### Recursos Adicionales
- **[Learn RxJS](https://www.learnrxjs.io/)** - Guía práctica de operadores y patrones
- **[RxJS Marbles](https://rxmarbles.com/)** - Comprensión visual de operadores

## Utilizado para revisión de código

Verifique su código en busca de anti-patrones.

👉 **[Lista de Verificación para Evitar Anti-patrones](./checklist)** - Revise su código con 15 elementos para verificar

Desde cada elemento de verificación, puede saltar directamente a los detalles del anti-patrón correspondiente en esta página.

## Próximos Pasos

- **[Manejo de Errores](/es/guide/error-handling/strategies)** - Aprenda estrategias de manejo de errores más detalladas
- **[Técnicas de Prueba](/es/guide/testing/unit-tests)** - Aprenda cómo probar eficazmente el código RxJS
- **[Comprensión de Operadores](/es/guide/operators/)** - Aprenda cómo usar cada operador en detalle

¡Incorpore estas mejores prácticas en su codificación diaria para escribir código RxJS de calidad!
