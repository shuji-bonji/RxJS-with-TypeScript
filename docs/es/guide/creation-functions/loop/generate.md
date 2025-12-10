---
description: "generate() - Generación de bucles genérica con control de condiciones flexible: Bucles declarativos tipo while para Fibonacci, paginación y gestión de estado personalizada"
---

# generate() - Generación de Bucles Genérica

`generate()` es una Función de Creación que proporciona procesamiento de bucles flexible como Observable especificando estado inicial, condición de continuación, actualización de estado y selección de resultado.

## Resumen

`generate()` puede describir declarativamente procesamiento de bucles flexible como sentencias while y for. Se usa cuando se requiere condiciones más complejas o gestión de estado que `range()`.

**Firma**:
```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

**Parámetros**:
- `initialState`: El estado inicial del bucle
- `condition`: Función para determinar la condición de continuación (`false` termina el bucle)
- `iterate`: Función para avanzar el estado al siguiente (actualizar estado)
- `resultSelector`: Función para seleccionar un valor a emitir desde el estado (si se omite, se emite el estado mismo)
- `scheduler`: Scheduler que emite valores (omitido: emite valores síncronamente)

**Documentación Oficial**: [📘 RxJS Oficial: generate()](https://rxjs.dev/api/index/function/generate)

## Uso Básico

### Patrón 1: Contador Simple

Este es el uso más básico.

```typescript
import { generate } from 'rxjs';

// Contar del 1 al 5
generate(
  1,              // Estado inicial
  x => x <= 5,    // Condición de continuación
  x => x + 1      // Actualización de estado
).subscribe({
  next: value => console.log('Valor:', value),
  complete: () => console.log('Completado')
});

// Salida:
// Valor: 1
// Valor: 2
// Valor: 3
// Valor: 4
// Valor: 5
// Completado
```

Este código es equivalente a la siguiente sentencia while:

```typescript
let x = 1;
while (x <= 5) {
  console.log('Valor:', x);
  x = x + 1;
}
console.log('Completado');
```

### Patrón 2: Convertir Valores con resultSelector

Puedes separar el estado del valor a emitir.

```typescript
import { generate } from 'rxjs';

// El estado interno es un contador, pero el valor emitido es un valor al cuadrado
generate(
  1,              // Estado inicial: 1
  x => x <= 5,    // Condición de continuación: x <= 5
  x => x + 1,     // Actualización de estado: x + 1
  x => x * x      // Selección de resultado: emitir x^2
).subscribe(console.log);

// Salida: 1, 4, 9, 16, 25
```

### Patrón 3: Objeto de Estado Complejo

Se pueden usar objetos complejos como estados.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

// Calcular suma acumulativa
generate<number, State>(
  { count: 1, sum: 0 },           // Estado inicial
  state => state.count <= 5,      // Condición de continuación
  state => ({                     // Actualización de estado
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => state.sum              // Selección de resultado
).subscribe(console.log);

// Salida: 0, 1, 3, 6, 10
// (0, 0+1, 0+1+2, 0+1+2+3, 0+1+2+3+4)
```

## Características Importantes

### 1. Comportamiento Similar a Sentencia While

`generate()` proporciona control flexible como una sentencia while.

```typescript
import { generate } from "rxjs";

// Sentencia while
let i = 1;
while (i <= 10) {
  console.log(i);
  i = i * 2;
}

// Lo mismo con generate()
generate(
  1,              // let i = 1;
  i => i <= 10,   // while (i <= 10)
  i => i * 2      // i = i * 2;
).subscribe(console.log);

// Salida: 1, 2, 4, 8
```

### 2. Emisión Síncrona

Por defecto, todos los valores se emiten **síncronamente** al suscribirse.

```typescript
import { generate } from 'rxjs';

console.log('Antes de la suscripción');

generate(1, x => x <= 3, x => x + 1).subscribe(val => console.log('Valor:', val));

console.log('Después de la suscripción');

// Salida:
// Antes de la suscripción
// Valor: 1
// Valor: 2
// Valor: 3
// Después de la suscripción
```

### 3. Cuidado con los Bucles Infinitos

Si la condición es siempre `true`, obtendrás un bucle infinito.

```typescript
import { generate, take } from 'rxjs';
// ❌ Peligro: bucle infinito (el navegador se congela)
// generate(0, x => true, x => x + 1).subscribe(console.log);

// ✅ Seguro: usar take() para limitar número
generate(
  0,
  x => true,  // Siempre true
  x => x + 1
).pipe(
  take(10)    // Obtener solo los primeros 10
).subscribe(console.log);

// Salida: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

> [!WARNING]
> **Cuidado con los Bucles Infinitos**:
> - Si la condición es siempre `true`, ocurre un bucle infinito
> - Usar `take()`, `takeWhile()` o `takeUntil()` para limitar el número de emisiones
> - O establecer condiciones de salida apropiadas con funciones condicionales

## Casos de Uso Prácticos

### 1. Secuencia de Fibonacci

Ejemplo de transiciones de estado complejas.

```typescript
import { generate, take } from 'rxjs';
interface FibState {
  current: number;
  next: number;
}

// Primeros 10 términos de la secuencia de Fibonacci
generate<number, FibState>(
  { current: 0, next: 1 },           // Estado inicial: F(0)=0, F(1)=1
  state => true,                     // Generado infinitamente
  state => ({                        // Actualización de estado
    current: state.next,
    next: state.current + state.next
  }),
  state => state.current             // Emitir valor actual
).pipe(
  take(10)                           // Primeros 10 términos
).subscribe(console.log);

// Salida: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### 2. Backoff Exponencial

Esta es la generación de tiempo de espera exponencial usada en el proceso de reintentos.

```typescript
import { generate } from 'rxjs';

interface RetryState {
  attempt: number;
  delay: number;
}

// Generar retraso para backoff exponencial (1, 2, 4, 8, 16 segundos)
generate<number, RetryState>(
  { attempt: 0, delay: 1000 },       // Estado inicial: 1 segundo
  state => state.attempt < 5,        // Máximo 5 intentos
  state => ({                        // Actualización de estado
    attempt: state.attempt + 1,
    delay: state.delay * 2           // Duplicar el tiempo de retraso
  }),
  state => state.delay               // Emitir tiempo de retraso
).subscribe(delay => {
  console.log(`Reintentar ${delay / 1000} segundos después`);
});

// Salida:
// Reintentar 1 segundo después
// Reintentar 2 segundos después
// Reintentar 4 segundos después
// Reintentar 8 segundos después
// Reintentar 16 segundos después
```

### 3. Control de Paginación

Continuar obteniendo mientras exista la siguiente página.

```typescript
import { generate, of, Observable, concatMap, delay } from 'rxjs';
interface PageState {
  page: number;
  hasNext: boolean;
}

interface PageData {
  page: number;
  items: string[];
  hasNext: boolean;
}

// Función para simular obtención de datos de página
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Item${page}-1`, `Item${page}-2`, `Item${page}-3`],
    hasNext: page < 10 // Hasta la página 10
  }).pipe(
    delay(500) // Simular llamada API
  );
}

// Obtener la página mientras exista (realmente obtener hasNext de la respuesta API)
generate<number, PageState>(
  { page: 1, hasNext: true },        // Estado inicial
  state => state.hasNext,            // Continuar mientras haya página siguiente
  state => ({                        // Actualización de estado
    page: state.page + 1,
    hasNext: state.page < 10         // Suponer que hay hasta 10 páginas
  }),
  state => state.page                // Emitir número de página
).pipe(
  concatMap(page => fetchPage(page)) // Obtener cada página en orden
).subscribe(
  data => console.log(`Página ${data.page} obtenida:`, data.items),
  err => console.error('Error:', err),
  () => console.log('Todas las páginas obtenidas')
);

// Salida:
// Página 1 obtenida: ['Item1-1', 'Item1-2', 'Item1-3']
// Página 2 obtenida: ['Item2-1', 'Item2-2', 'Item2-3']
// ...
// Página 10 obtenida: ['Item10-1', 'Item10-2', 'Item10-3']
// Todas las páginas obtenidas
```

### 4. Temporizador Personalizado

Emite eventos a intervalos irregulares.

```typescript
import { generate, of, concatMap, delay } from 'rxjs';
interface TimerState {
  count: number;
  delay: number;
}

// Temporizador con retraso gradualmente creciente
generate<string, TimerState>(
  { count: 0, delay: 1000 },         // Estado inicial: 1 segundo
  state => state.count < 5,          // Hasta 5 veces
  state => ({                        // Actualización de estado
    count: state.count + 1,
    delay: state.delay + 500         // Aumentar retraso en 500 ms
  }),
  state => `Evento${state.count + 1}`
).pipe(
  concatMap((message, index) => {
    const delayTime = 1000 + index * 500;
    console.log(`${delayTime}ms de espera antes de emitir`);
    return of(message).pipe(delay(delayTime));
  })
).subscribe(console.log);

// Salida:
// Emitido después de 1000ms de espera
// Evento 1 (después de 1 segundo)
// Emitido después de 1500ms de espera
// Evento 2 (después de 2.5 segundos)
// Emitido después de 2000ms de espera
// Evento 3 (después de 4.5 segundos)
// ...
```

### 5. Cálculo de Factoriales

Representar cálculos matemáticos como streams.

```typescript
import { generate } from 'rxjs';

interface FactorialState {
  n: number;
  result: number;
}

// Calcular factorial de 5 (5! = 5 × 4 × 3 × 2 × 1 = 120)
generate<number, FactorialState>(
  { n: 5, result: 1 },               // Estado inicial
  state => state.n > 0,              // Continúa para n > 0
  state => ({                        // Actualización de estado
    n: state.n - 1,
    result: state.result * state.n
  }),
  state => state.result              // Emitir resultado intermedio
).subscribe(console.log);

// Salida: 5, 20, 60, 120, 120
// (1*5, 5*4, 20*3, 60*2, 120*1)
```

## Comparación con Otras Funciones de Creación

### generate() vs range()

```typescript
import { generate, range } from 'rxjs';

// range() - numeración secuencial simple
range(1, 5).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5

// generate() - lo mismo, pero más explícito
generate(
  1,
  x => x <= 5,
  x => x + 1
).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5

// Verdadero valor de generate(): pasos complejos
generate(
  1,
  x => x <= 100,
  x => x * 2  // Aumentar por factor de 2
).subscribe(console.log);
// Salida: 1, 2, 4, 8, 16, 32, 64
// (no posible con range())
```

### generate() vs defer()

```typescript
import { generate, defer, of } from 'rxjs';

// generate() - procesamiento de bucle
generate(1, x => x <= 3, x => x + 1).subscribe(console.log);
// Salida: 1, 2, 3

// defer() - generar al suscribirse (no es un bucle)
defer(() => of(1, 2, 3)).subscribe(console.log);
// Salida: 1, 2, 3

// Diferencia: generate() tiene estado, defer solo evaluación perezosa
```

> [!TIP]
> **Criterios de Selección**:
> - **Números secuenciales simples** → `range()`
> - **Condiciones o pasos complejos** → `generate()`
> - **Determinado dinámicamente al suscribirse** → `defer()`
> - **Fibonacci, factorial, etc.** → `generate()`

## Asincronización con Scheduler

Al procesar grandes cantidades de datos, la ejecución asíncrona es posible especificando un scheduler.

```typescript
import { generate, asyncScheduler, observeOn } from 'rxjs';
console.log('Inicio');

// Ejecutar un millón de bucles asíncronamente
generate(
  1,
  x => x <= 1000000,
  x => x + 1
).pipe(
  observeOn(asyncScheduler)
).subscribe({
  next: val => {
    if (val % 100000 === 0) {
      console.log(`Progreso: ${val}`);
    }
  },
  complete: () => console.log('Completado')
});

console.log('Después de suscripción (asíncrono, así que se ejecuta inmediatamente)');

// Salida:
// Inicio
// Después de suscripción (asíncrono, así que se ejecutará inmediatamente)
// Progreso: 100000
// Progreso: 200000
// ...
// Completado
```

## Consideraciones de Rendimiento

Debido a que `generate()` emite valores síncronamente, se debe considerar el rendimiento al generar grandes números de valores o realizar cálculos complejos.

> [!WARNING]
> **Optimización de Rendimiento**:
> ```typescript
> // ❌ Mal ejemplo: cálculo complejo realizado síncronamente (UI bloqueada)
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).subscribe(console.log);
>
> // ✅ Buen ejemplo 1: asíncrono con scheduler
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Buen ejemplo 2: Limitar el número con take()
> generate(
>   1,
>   x => true,  // Bucle infinito
>   x => x + 1
> ).pipe(
>   take(100)   // Solo los primeros 100
> ).subscribe(console.log);
> ```

## Manejo de Errores

Aunque `generate()` en sí no emite errores, pueden ocurrir errores en pipelines y funciones de actualización de estado.

```typescript
import { generate, of, map, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,
  x => x + 1
).pipe(
  map(n => {
    if (n === 5) {
      throw new Error('Error en 5');
    }
    return n * 2;
  }),
  catchError(error => {
    console.error('Error ocurrido:', error.message);
    return of(-1); // Retornar valor predeterminado
  })
).subscribe(console.log);

// Salida: 2, 4, 6, 8, -1
```

### Error en Función de Actualización de Estado

Un error dentro de una función de actualización de estado causará que Observable entre en estado de error.

```typescript
import { generate, EMPTY, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,
  x => {
    if (x === 5) {
      throw new Error('Error en actualización de estado');
    }
    return x + 1;
  }
).pipe(
  catchError(error => {
    console.error('Error:', error.message);
    return EMPTY; // Retornar Observable vacío
  })
).subscribe({
  next: console.log,
  complete: () => console.log('Completado')
});

// Salida: 1, 2, 3, 4, Error: Error en actualización de estado, Completado
```

## Seguridad de Tipos en TypeScript

`generate()` puede separar el tipo del estado del tipo del valor emitido.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

interface Result {
  index: number;
  average: number;
}

// Estado: State, valor emitido: Result
const stats$ = generate<Result, State>(
  { count: 1, sum: 0 },
  state => state.count <= 5,
  state => ({
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => ({
    index: state.count,
    average: state.sum / state.count
  })
);

stats$.subscribe(result => {
  console.log(`[${result.index}] Promedio: ${result.average}`);
});

// Salida:
// [1] Promedio: 0
// [2] Promedio: 0.5
// [3] Promedio: 1
// [4] Promedio: 1.5
// [5] Promedio: 2
```

## Resumen

`generate()` es una Función de Creación poderosa que permite describir declarativamente procesamiento de bucles complejo.

> [!IMPORTANT]
> **Características de generate()**:
> - ✅ Control de bucles flexible como sentencias while/for
> - ✅ Gestión de estado compleja posible
> - ✅ Ideal para cálculos matemáticos como Fibonacci, factorial, etc.
> - ✅ Estado y valores de emisión pueden separarse
> - ⚠️ Cuidado con bucles infinitos (limitado por `take()`)
> - ⚠️ Considerar asíncrono para grandes cantidades de datos
> - ⚠️ Usar `range()` para números secuenciales simples

## Temas Relacionados

- [range()](/es/guide/creation-functions/loop/range) - Generación de números secuenciales simple
- [defer()](/es/guide/creation-functions/conditional/defer) - Generación dinámica al suscribirse
- [expand()](/es/guide/operators/transformation/expand) - Expansión recursiva (operador de orden superior)
- [scan()](/es/guide/operators/transformation/scan) - Cálculo acumulativo

## Referencias

- [RxJS Oficial: generate()](https://rxjs.dev/api/index/function/generate)
- [Learn RxJS: generate](https://www.learnrxjs.io/learn-rxjs/operators/creation/generate)
