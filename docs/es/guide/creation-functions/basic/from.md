---
description: "from() - Función de Creación que convierte arrays, Promises, iterables y objetos similares a Observable en flujos Observable con integración asíncrona sin interrupciones"
---

# from() - Convertir desde Array, Promise, etc.

`from()` es una Función de Creación que crea un Observable desde arrays, Promises, iterables y objetos similares a Observable.

## Resumen

`from()` convierte estructuras de datos existentes (arrays, Promises, iterables, etc.) en flujos Observable. En particular, se usa frecuentemente para integrar procesamiento asíncrono (Promise) en el mundo RxJS.

**Firma**:
```typescript
function from<T>(input: ObservableInput<T>, scheduler?: SchedulerLike): Observable<T>
```

**Documentación Oficial**: [📘 RxJS Oficial: from()](https://rxjs.dev/api/index/function/from)

## Uso Básico

`from()` acepta una variedad de tipos de entrada.

```typescript
import { from } from 'rxjs';

// Crear desde array
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Valor del array:', value),
  complete: () => console.log('Array completado')
});

// Crear desde Promise
const promise$ = from(Promise.resolve('Resultado de Promise'));
promise$.subscribe({
  next: value => console.log('Resultado de Promise:', value),
  complete: () => console.log('Promise completada')
});

// Crear desde iterable
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Valor del iterable:', value),
  complete: () => console.log('Iterable completado')
});

// Salida:
// Valor del array: 1
// Valor del array: 2
// Valor del array: 3
// Array completado
// Valor del iterable: 1
// Valor del iterable: 2
// Valor del iterable: 3
// Iterable completado
// Resultado de Promise: Resultado de Promise
// Promise completada
```

## Características Importantes

### 1. Emitir Cada Elemento del Array Individualmente

Cuando `from()` recibe un array, emite cada elemento del array individualmente en secuencia.

```typescript
import { from } from 'rxjs';

from([10, 20, 30]).subscribe(value => console.log(value));

// Salida:
// 10
// 20
// 30
```

> [!IMPORTANT]
> **Diferencia con `of()`**:
> - `of([1, 2, 3])` → Emite el array en sí como un único valor
> - `from([1, 2, 3])` → Emite cada elemento `1`, `2`, `3` por separado

### 2. Procesar Promise Automáticamente

Pasar una Promise emitirá el valor resuelto y se completará inmediatamente.

```typescript
import { from } from 'rxjs';

const fetchData = (): Promise<string> => {
  return new Promise(resolve => {
    setTimeout(() => resolve('Obtención de datos completada'), 1000);
  });
};

from(fetchData()).subscribe({
  next: value => console.log(value),
  complete: () => console.log('Completado')
});

// Salida después de 1 segundo:
// Obtención de datos completada
// Completado
```

> [!WARNING]
> Si la Promise es rechazada, Observable emite un error.
> ```typescript
> import { from } from "rxjs";
> from(Promise.reject('Error')).subscribe({
>   error: err => console.error('Ocurrió un error:', err)
> });
> ```

### 3. Soporte para Iterables

Además de arrays, soporta objetos iterables como `Set`, `Map` y `Generator`.

```typescript
import { from } from 'rxjs';

// Set
from(new Set(['A', 'B', 'C'])).subscribe(console.log);
// Salida: A, B, C

// Map (pares clave-valor)
from(new Map([['key1', 'value1'], ['key2', 'value2']])).subscribe(console.log);
// Salida: ['key1', 'value1'], ['key2', 'value2']

// Generator
function* numberGenerator() {
  yield 1;
  yield 2;
  yield 3;
}
from(numberGenerator()).subscribe(console.log);
// Salida: 1, 2, 3
```

### 4. Cold Observable

`from()` es un **Cold Observable**. Cada suscripción inicia una ejecución independiente.

```typescript
import { from } from 'rxjs';

const numbers$ = from([1, 2, 3]);

numbers$.subscribe(val => console.log('Suscriptor A:', val));
numbers$.subscribe(val => console.log('Suscriptor B:', val));

// Cada suscriptor procesa el array independientemente
// Salida:
// Suscriptor A: 1
// Suscriptor A: 2
// Suscriptor A: 3
// Suscriptor B: 1
// Suscriptor B: 2
// Suscriptor B: 3
```

> [!NOTE]
> **Características de Cold Observable**:
> - Se inicia una ejecución independiente para cada suscripción
> - Cada suscriptor recibe su propio flujo de datos
> - Las Promises también se evalúan por suscripción
>
> Ver [Cold Observable y Hot Observable](/es/guide/observables/cold-and-hot-observables) para más información.

## Diferencia Entre from() y of()

La diferencia más importante entre los dos es la forma en que se manejan los arrays.

```typescript
import { from, of } from 'rxjs';

const array = [1, 2, 3];

// of() - emite el array como un único valor
of(array).subscribe(value => {
  console.log('of():', value); // [1, 2, 3]
});

// from() - emite cada elemento del array individualmente
from(array).subscribe(value => {
  console.log('from():', value); // 1, 2, 3
});
```

| Función de Creación | Manejo de Array | Propósito |
|-------------------|-----------|------|
| `of([1, 2, 3])` | Emite el array en sí | Quiere tratar el array como dato |
| `from([1, 2, 3])` | Emite cada elemento individualmente | Quiere procesar elementos del array uno por uno |

## Casos de Uso Prácticos

### 1. Transmitir Llamadas API

Transmitir clientes HTTP basados en Promise como Fetch API y axios.

```typescript
import { from, Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
}

function fetchUser(id: number): Observable<User> {
  return from(
    fetch(`https://jsonplaceholder.typicode.com/users/${id}`)
      .then(response => response.json())
  ).pipe(
    catchError(error => {
      console.error('Error de API:', error);
      return of({ id: 0, name: 'Desconocido', email: '' });
    })
  );
}

fetchUser(1).subscribe(user => console.log('Usuario:', user));
```

### 2. Procesamiento Secuencial de Elementos de Array

Ejecutar procesamiento asíncrono secuencialmente para cada elemento del array.

```typescript
import { from } from 'rxjs';
import { concatMap, delay } from 'rxjs';

const urls = [
  'https://jsonplaceholder.typicode.com/posts/1',
  'https://jsonplaceholder.typicode.com/posts/2',
  'https://jsonplaceholder.typicode.com/posts/3'
];

from(urls).pipe(
  concatMap(url =>
    from(fetch(url).then(res => res.json())).pipe(
      delay(500) // Limitación de tasa
    )
  )
).subscribe(data => console.log('Obtenido:', data));
```

### 3. Procesamiento de Iterador Asíncrono

También se soportan iteradores asíncronos (async generators).

```typescript
import { from } from 'rxjs';

async function* asyncGenerator() {
  yield await Promise.resolve(1);
  yield await Promise.resolve(2);
  yield await Promise.resolve(3);
}

from(asyncGenerator()).subscribe(value => console.log(value));
// Salida: 1, 2, 3
```

### 4. Integración con Event Emitter

Transmitir Node.js EventEmitter y sistemas de eventos personalizados.

```typescript
import { from } from 'rxjs';

// Objeto personalizado iterable
class DataSource {
  *[Symbol.iterator]() {
    yield 'Dato A';
    yield 'Dato B';
    yield 'Dato C';
  }
}

from(new DataSource()).subscribe(console.log);
// Salida: Dato A, Dato B, Dato C
```

## Uso en Pipeline

`from()` es útil cuando se usan datos existentes como punto de partida para procesamiento de pipeline.

```typescript
import { from } from 'rxjs';
import { map, filter, reduce } from 'rxjs';

interface Product {
  id: number;
  name: string;
  price: number;
}

const products: Product[] = [
  { id: 1, name: 'Producto A', price: 1000 },
  { id: 2, name: 'Producto B', price: 2000 },
  { id: 3, name: 'Producto C', price: 500 }
];

from(products).pipe(
  filter(product => product.price >= 1000),
  map(product => product.price),
  reduce((sum, price) => sum + price, 0)
).subscribe(total => console.log('Monto total:', total));
// Salida: Monto total: 3000
```

## Errores Comunes

### 1. Malentender el Momento de Ejecución de Promise

```typescript
// ❌ Incorrecto - Promise comienza a ejecutarse en el momento de creación
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1'); // Ya comenzó
from(promise).subscribe(console.log); // No en el momento de suscripción

// ✅ Correcto - usar defer() si quieres ejecutar en el momento de suscripción
import { defer, from } from 'rxjs';

const deferred$ = defer(() =>
  from(fetch('https://jsonplaceholder.typicode.com/posts/1'))
);
deferred$.subscribe(console.log); // Se ejecuta en el momento de suscripción
```

> [!WARNING]
> **Promise No Tiene Evaluación Perezosa**
>
> Promise comienza a ejecutarse cuando se crea. `from(promise)` solo envuelve una Promise que ya se está ejecutando. Si quieres ejecutar en el momento de suscripción, usa `defer(() => from(promise))`.

### 2. Confundir Array con of()

```typescript
import { from, map, of } from "rxjs";

// ❌ Diferente de la intención - se emite el array completo
of([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Salida: [1, 2, 3] (el array en sí)

// ✅ Correcto - procesar cada elemento individualmente
from([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Salida: 2, 4, 6
```

## Consideraciones de Rendimiento

El rendimiento de `from()` depende del tipo de entrada.

> [!TIP]
> **Consejos de Optimización**:
> - Al procesar grandes cantidades de datos (miles a decenas de miles de elementos), limita el número de operaciones concurrentes al combinar con `concatMap` y `mergeMap`.
> - Al procesar arrays de Promise, considera usar `forkJoin` o `combineLatest`.

```typescript
import { from } from 'rxjs';
import { mergeMap } from 'rxjs';

const urls = [...Array(100)].map((_, i) => `https://jsonplaceholder.typicode.com/posts/${i + 1}`);

from(urls).pipe(
  mergeMap(
    url => from(fetch(url).then(res => res.json())),
    5 // Limitar ejecución concurrente a 5
  )
).subscribe(data => console.log(data));
```

## Funciones de Creación Relacionadas

| Función | Diferencia | Uso |
|----------|------|----------|
| **[of()](/es/guide/creation-functions/basic/of)** | Emitir argumentos en secuencia | Quiere emitir valores tal como están |
| **[fromEvent()](/es/guide/creation-functions/basic/fromEvent)** | Transmitir eventos | Manejar eventos DOM o EventEmitter |
| **[defer()](/es/guide/creation-functions/conditional/defer)** | Diferir generación hasta suscripción | Necesita ejecución perezosa de Promise |
| **ajax()** | Dedicado a comunicación HTTP | Quiere completar solicitudes HTTP dentro de RxJS |

## Resumen

- `from()` crea Observable desde arrays, Promises e iterables
- Emite cada elemento de un array por separado (diferente de `of()`)
- Procesa Promise automáticamente y emite el resultado
- Ideal para integrar procesamiento asíncrono en el mundo RxJS
- Ten en cuenta que Promise se ejecuta en el punto de creación (usa `defer()` para ejecución perezosa)

## Próximos Pasos

- [fromEvent() - Convertir Eventos a Observable](/es/guide/creation-functions/basic/fromEvent)
- [defer() - Diferir Generación Hasta Suscripción](/es/guide/creation-functions/conditional/defer)
- [Volver a Funciones de Creación Básicas](/es/guide/creation-functions/basic/)
