---
description: "range() - Función de Creación que genera enteros consecutivos declarativamente: Alternativa eficiente en memoria a bucles for para procesamiento por lotes y paginación"
---

# range() - Genera un rango de números

`range()` es una Función de Creación similar a sentencia for que emite un número especificado de enteros consecutivos desde un valor inicial especificado.

## Resumen

`range()` emite una secuencia de enteros consecutivos como Observable especificando un valor inicial y el número de enteros. Se usa para generación de números secuenciales y procesamiento por lotes como una forma declarativa de reemplazar la sentencia `for` tradicional.

**Firma**:
```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

**Parámetros**:
- `start`: El valor inicial (desde donde empezar a emitir)
- `count`: el número de valores a emitir (omitido, desde 0 hasta menos de `start`)
- `scheduler`: el scheduler para emitir los valores (omitido: emite síncronamente)

**Documentación Oficial**: [📘 RxJS Oficial: range()](https://rxjs.dev/api/index/function/range)

## Uso Básico

### Patrón 1: Especificar valor inicial y conteo

Este es el uso más común.

```typescript
import { range } from 'rxjs';

// Generar 5 números secuenciales desde 1 (1, 2, 3, 4, 5)
range(1, 5).subscribe({
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

### Patrón 2: Números secuenciales comenzando desde 0

Estableciendo el valor inicial en 0, se puede generar un número secuencial como un índice de array.

```typescript
import { range } from 'rxjs';

// 0 a 10 números secuenciales (0, 1, 2, ..., 9)
range(0, 10).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

### Patrón 3: Comenzar con un número negativo

También se pueden generar números negativos.

```typescript
import { range } from 'rxjs';

// 5 números secuenciales desde -3 (-3, -2, -1, 0, 1)
range(-3, 5).subscribe(console.log);
// Salida: -3, -2, -1, 0, 1
```

## Características Importantes

### 1. Emisión Síncrona

Por defecto, `range()` emite todos los valores **síncronamente** al suscribirse.

```typescript
import { range } from 'rxjs';

console.log('Antes de la suscripción');

range(1, 3).subscribe(value => console.log('Valor:', value));

console.log('Después de la suscripción');

// Salida:
// Antes de la suscripción
// Valor: 1
// Valor: 2
// Valor: 3
// Después de la suscripción
```

### 2. Se Completa Inmediatamente

Notifica `complete` inmediatamente después de emitir todos los valores.

```typescript
import { range } from 'rxjs';

range(1, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('¡Completado!')
});

// Salida: 1, 2, 3, ¡Completado!
```

### 3. Equivalencia con sentencia for

`range(start, count)` es equivalente a la siguiente sentencia for.

```typescript
// Sentencia for imperativa
for (let i = start; i < start + count; i++) {
  console.log(i);
}

// range() declarativo
range(start, count).subscribe(console.log);
```

## Casos de Uso Prácticos

### 1. Procesamiento por Lotes

Se usa para ejecutar múltiples tareas secuencialmente.

```typescript
import { range, of, Observable, concatMap, delay, map } from 'rxjs';
// Función para simular procesamiento de datos
function processItem(index: number): Observable<string> {
  return of(index).pipe(
    delay(100), // Simular 100ms de tiempo de procesamiento
    map(i => `Resultado de procesar item ${i}`)
  );
}

// Procesar 10 items de datos secuencialmente (1 segundo de retraso entre cada proceso)
range(1, 10).pipe(
  concatMap(index =>
    processItem(index).pipe(delay(1000))
  )
).subscribe({
  next: result => console.log(`Procesamiento completo: ${result}`),
  complete: () => console.log('Todo el procesamiento completado')
});

// Salida:
// Procesamiento completo: Resultado de procesar item 1 (después de aproximadamente 1.1 segundos)
// Procesamiento completo: Resultado de procesar item 2 (después de aproximadamente 2.1 segundos)
// ...
// Procesamiento completo: Resultado de procesar item 10 (después de aproximadamente 10.1 segundos)
// Todo el procesamiento completado
```

### 2. Paginación

Obtener múltiples páginas de datos secuencialmente.

```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Función para simular obtención de datos de página
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Item${page}-1`, `Item${page}-2`, `Item${page}-3`]
  }).pipe(
    delay(500) // Simular llamada API
  );
}

function fetchAllPages(totalPages: number) {
  return range(1, totalPages).pipe(
    concatMap(page => fetchPage(page))
  );
}

fetchAllPages(5).subscribe({
  next: (data: PageData) => console.log(`Página ${data.page}:`, data.items),
  complete: () => console.log('Todas las páginas obtenidas')
});

// Salida:
// Página 1: ['Item1-1', 'Item1-2', 'Item1-3']
// Página 2: ['Item2-1', 'Item2-2', 'Item2-3']
// Página 3: ['Item3-1', 'Item3-2', 'Item3-3']
// Página 4: ['Item4-1', 'Item4-2', 'Item4-3']
// Página 5: ['Item5-1', 'Item5-2', 'Item5-3']
// Todas las páginas obtenidas
```

### 3. Procesamiento de Índices de Array

Usar como bucle basado en índices al procesar cada elemento de un array.

```typescript
import { range, map } from 'rxjs';
const items = ['Manzana', 'Banana', 'Cereza', 'Dátil', 'Saúco'];

range(0, items.length).pipe(
  map(index => ({ index, item: items[index] }))
).subscribe(({ index, item }) => {
  console.log(`[${index}] ${item}`);
});

// Salida:
// [0] Manzana
// [1] Banana
// [2] Cereza
// [3] Dátil
// [4] Saúco
```

### 4. Generación de Datos de Prueba

Esto es útil para generar datos mock para pruebas unitarias.

```typescript
import { range, map, toArray } from 'rxjs';
// Generar datos de usuario mock
range(1, 100).pipe(
  map(id => ({
    id,
    name: `Usuario${id}`,
    email: `usuario${id}@example.com`
  })),
  toArray()
).subscribe(users => {
  console.log(`${users.length} usuarios generados`);
  // Usar en pruebas
});
```

### 5. Contador para Procesamiento de Reintentos

Controla el número de reintentos en caso de error.

```typescript
import { range, throwError, concat, of, Observable, mergeMap, delay, catchError, map, toArray } from 'rxjs';
// Función para simular obtención de datos (falla aleatoriamente)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.7; // 30% probabilidad de éxito

  return of(shouldFail).pipe(
    delay(300),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Obtención de datos fallida'))
        : of('Obtención de datos exitosa')
    )
  );
}

function fetchWithRetry(maxRetries: number = 3) {
  return concat(
    range(0, maxRetries).pipe(
      map(attempt => {
        console.log(`Intento ${attempt + 1}/${maxRetries}`);
        return fetchData().pipe(
          catchError(error => {
            if (attempt === maxRetries - 1) {
              return throwError(() => new Error('Máximo de reintentos alcanzado'));
            }
            return throwError(() => error);
          }),
          delay(Math.pow(2, attempt) * 1000) // Backoff exponencial
        );
      }),
      toArray()
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Resultado:', result),
  error: err => console.error('Error:', err.message)
});

// Ejemplo de salida:
// Intento 1/3
// Intento 2/3
// Resultado: Obtención de datos exitosa
```

## Asincronización con Scheduler

Al procesar grandes cantidades de datos, la ejecución asíncrona es posible especificando un scheduler.

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
console.log('Inicio');

// Emitir 1,000,000 números asíncronamente
range(1, 1000000).pipe(
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

> [!TIP]
> **Uso del Scheduler**:
> - No bloquear la UI al procesar grandes cantidades de datos
> - Control de tiempo en pruebas (TestScheduler)
> - Control del event loop en entorno Node.js

Para más información, consulta [Tipos de Schedulers y Cómo Usarlos](/es/guide/schedulers/types).

## Comparación con Otras Funciones de Creación

### range() vs of()

```typescript
import { range, of } from 'rxjs';

// range() - enteros consecutivos
range(1, 3).subscribe(console.log);
// Salida: 1, 2, 3

// of() - enumerar valores arbitrarios
of(1, 2, 3).subscribe(console.log);
// Salida: 1, 2, 3

// Diferencia: range() solo acepta números secuenciales, of() acepta valores arbitrarios
of(1, 10, 100).subscribe(console.log);
// Salida: 1, 10, 100 (no posible con range())
```

### range() vs from()

```typescript
import { range, from } from 'rxjs';

// range() - generar números secuenciales
range(1, 5).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5

// from() - generar desde un array (debe crear array previamente)
from([1, 2, 3, 4, 5]).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5

// Ventaja de range(): no hay pre-asignación de arrays en memoria
range(1, 1000000); // Eficiente en memoria
from(Array.from({ length: 1000000 }, (_, i) => i + 1)); // El array va a memoria
```

### range() vs generate()

```typescript
import { range, generate } from 'rxjs';

// range() - numeración secuencial simple
range(1, 5).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5

// generate() - un ejemplo complejo de lo mismo
generate(
  1,                    // Valor inicial
  x => x <= 5,          // Condición de continuación
  x => x + 1            // Iteración
).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5

// Ventajas de generate(): condición compleja y gestión de estado
generate(
  1,
  x => x <= 100,
  x => x * 2  // Incrementa por factor de 2
).subscribe(console.log);
// Salida: 1, 2, 4, 8, 16, 32, 64
// (no posible con range())
```

> [!TIP]
> **Criterios de Selección**:
> - **Requiere números secuenciales** → `range()`
> - **Enumerar cualquier valor** → `of()`
> - **Array/Promise existente** → `from()`
> - **Condición/paso complejo** → `generate()`

## Consideraciones de Rendimiento

Debido a que `range()` emite valores síncronamente, se debe considerar el rendimiento al generar grandes números de valores.

> [!WARNING]
> **Manejo de Grandes Cantidades de Datos**:
> ```typescript
> // ❌ Mal ejemplo: emitir 1 millón de valores síncronamente (la UI se bloqueará)
> range(1, 1000000).subscribe(console.log);
>
> // ✅ Buen ejemplo 1: asíncrono con scheduler
> range(1, 1000000).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Buen ejemplo 2: Dividir por buffering
> range(1, 1000000).pipe(
>   bufferCount(1000)
> ).subscribe(batch => console.log(`${batch.length} casos procesados`));
> ```

## Elegir Entre from() Array

```typescript
import { range, from } from 'rxjs';

// Si necesitas números secuenciales → range() es más conciso
range(0, 10).subscribe(console.log);

// No hay necesidad de crear un array y luego convertirlo (ineficiente)
from(Array.from({ length: 10 }, (_, i) => i)).subscribe(console.log);

// Si hay un array existente → usar from()
const existingArray = [5, 10, 15, 20];
from(existingArray).subscribe(console.log);
```

## Manejo de Errores

Aunque `range()` en sí no emite errores, pueden ocurrir errores en el pipeline.

```typescript
import { range, of, map, catchError } from 'rxjs';
range(1, 10).pipe(
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

## Resumen

`range()` es una Función de Creación simple pero poderosa que produce una secuencia de enteros consecutivos.

> [!IMPORTANT]
> **Características de range()**:
> - ✅ Ideal para generar números consecutivos (alternativa a sentencia for)
> - ✅ Útil para procesamiento por lotes, paginación, generación de datos de prueba
> - ✅ Eficiente en memoria (no hay pre-creación de arrays)
> - ⚠️ Considerar asíncrono para grandes cantidades de datos
> - ⚠️ Usar `generate()` para condiciones complejas

## Temas Relacionados

- [generate()](/es/guide/creation-functions/loop/generate) - Generación de bucles genérica
- [of()](/es/guide/creation-functions/basic/of) - Enumera valores arbitrarios
- [from()](/es/guide/creation-functions/basic/from) - Convertir desde array o Promise
- [interval()](/es/guide/creation-functions/basic/interval) - Emitir valores periódicamente

## Referencias

- [RxJS Oficial: range()](https://rxjs.dev/api/index/function/range)
- [Learn RxJS: range](https://www.learnrxjs.io/learn-rxjs/operators/creation/range)
