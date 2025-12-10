---
description: "Explicamos cómo usar la Función de Creación scheduled() de RxJS para generar Observables especificando un scheduler y controlar el momento de ejecución. Presentamos cómo usar asyncScheduler, queueScheduler, optimización del rendimiento e implementación segura de tipos en TypeScript."
---

# scheduled()

[📘 Documentación Oficial RxJS - scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` es una Función de Creación que puede especificar explícitamente un scheduler al generar un Observable desde fuentes de datos como arrays, Promises e Iterables. Permite un control fino del momento de ejecución (síncrono/asíncrono) y es útil para pruebas y optimización del rendimiento de UI.

## Uso Básico

### Conversión simple de array a Observable

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Emitir array de forma asíncrona
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Inicio de suscripción');
observable$.subscribe({
  next: val => console.log('Valor:', val),
  complete: () => console.log('Completado')
});
console.log('Fin de suscripción');

// Salida:
// Inicio de suscripción
// Fin de suscripción
// Valor: 1
// Valor: 2
// Valor: 3
// Completado
```

> [!IMPORTANT]
> **Diferencia entre síncrono y asíncrono**
>
> Al usar `asyncScheduler`, la emisión de valores se vuelve asíncrona. Por eso, la salida es "Inicio de suscripción" → "Fin de suscripción" → "Valor: 1" en ese orden.

### Comparación con from()

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - Síncrono por defecto
console.log('=== from() ===');
from([1, 2, 3]).subscribe(val => console.log('Valor:', val));
console.log('Fin de suscripción');

// Salida:
// === from() ===
// Valor: 1
// Valor: 2
// Valor: 3
// Fin de suscripción

// scheduled() - Explícitamente asíncrono
console.log('=== scheduled() ===');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Valor:', val));
console.log('Fin de suscripción');

// Salida:
// === scheduled() ===
// Fin de suscripción
// Valor: 1
// Valor: 2
// Valor: 3
```

## Tipos de Scheduler

RxJS proporciona varios schedulers que se usan según el propósito.

| Scheduler | Momento de ejecución | Tecnología base | Usos principales |
|---------------|--------------|-----------|---------|
| `queueScheduler` | Síncrono (método de cola) | Ejecución inmediata | Por defecto, procesamiento síncrono |
| `asyncScheduler` | Asíncrono | `setTimeout` | Optimización de UI, procesamiento largo |
| `asapScheduler` | Asíncrono más rápido | `Promise` (microtask) | Procesamiento asíncrono de alta prioridad |
| `animationFrameScheduler` | Frame de animación | `requestAnimationFrame` | Animación, renderizado de UI |

### queueScheduler (ejecución síncrona)

```typescript
import { scheduled, queueScheduler } from 'rxjs';

console.log('Inicio');
scheduled([1, 2, 3], queueScheduler).subscribe(val => console.log('Valor:', val));
console.log('Fin');

// Salida:
// Inicio
// Valor: 1
// Valor: 2
// Valor: 3
// Fin
```

### asyncScheduler (ejecución asíncrona)

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

console.log('Inicio');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Valor:', val));
console.log('Fin');

// Salida:
// Inicio
// Fin
// Valor: 1
// Valor: 2
// Valor: 3
```

### asapScheduler (microtask)

```typescript
import { scheduled, asapScheduler } from 'rxjs';

console.log('Inicio');
scheduled([1, 2, 3], asapScheduler).subscribe(val => console.log('Valor:', val));
console.log('Fin');

// Salida:
// Inicio
// Fin
// Valor: 1
// Valor: 2
// Valor: 3
```

> [!TIP]
> **asyncScheduler vs asapScheduler**
>
> - `asyncScheduler`: Basado en `setTimeout` (macrotask)
> - `asapScheduler`: Basado en `Promise` (microtask)
>
> `asapScheduler` se ejecuta más rápido, pero ambos son asíncronos.

### animationFrameScheduler (animación)

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// Actualizar valor por cada frame de animación
const positions = [0, 50, 100, 150, 200];
const animation$ = scheduled(positions, animationFrameScheduler).pipe(
  map(pos => `Posición: ${pos}px`)
);

animation$.subscribe(position => {
  console.log(position);
  // Actualizar DOM aquí
});

// Salida: (en cada frame de animación)
// Posición: 0px
// Posición: 50px
// Posición: 100px
// Posición: 150px
// Posición: 200px
```

## Patrones Prácticos

### Procesamiento de grandes datos sin bloquear UI

```typescript
import { scheduled, asyncScheduler, map, bufferCount } from 'rxjs';
// Procesar 1 millón de datos
const largeArray = Array.from({ length: 1000000 }, (_, i) => i);

// ❌ Mal ejemplo: Procesar de forma síncrona (UI se bloquea)
// from(largeArray).subscribe(processData);

// ✅ Buen ejemplo: Procesar de forma asíncrona (UI no se bloquea)
scheduled(largeArray, asyncScheduler).pipe(
  bufferCount(1000), // Procesar en lotes de 1000
  map(batch => batch.reduce((sum, val) => sum + val, 0))
).subscribe({
  next: sum => console.log('Suma del lote:', sum),
  complete: () => console.log('Procesamiento completado')
});

console.log('UI sigue respondiendo');
```

### Combinación con Promise

```typescript
import { scheduled, asyncScheduler, mergeMap } from 'rxjs';
interface User {
  id: number;
  name: string;
}

const userIds = [1, 2, 3, 4, 5];

// Obtener múltiples usuarios de forma asíncrona
scheduled(userIds, asyncScheduler).pipe(
  mergeMap(id =>
    fetch(`https://api.example.com/users/${id}`).then(res => res.json())
  )
).subscribe({
  next: (user: User) => console.log('Usuario:', user),
  error: error => console.error('Error:', error),
  complete: () => console.log('Obtención de todos los usuarios completada')
});
```

### Generación desde Iterable

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Convertir Set con schedule
const uniqueNumbers = new Set([1, 2, 3, 4, 5]);
const observable$ = scheduled(uniqueNumbers, asyncScheduler);

observable$.subscribe({
  next: val => console.log('Valor:', val),
  complete: () => console.log('Completado')
});

// Convertir Map con schedule
const userMap = new Map([
  [1, 'Alice'],
  [2, 'Bob'],
  [3, 'Charlie']
]);

scheduled(userMap, asyncScheduler).subscribe({
  next: ([id, name]) => console.log(`ID: ${id}, Nombre: ${name}`),
  complete: () => console.log('Completado')
});
```

## Uso en Pruebas

`scheduled()` se puede combinar con TestScheduler para escribir pruebas con control de tiempo.

### Prueba Básica

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled } from 'rxjs';

describe('scheduled()', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('emite array en orden', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler);
      const expected = '(abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

### Prueba de procesamiento asíncrono

```typescript
import { scheduled, asyncScheduler, delay } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Prueba de procesamiento asíncrono', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('prueba procesamiento con retraso de forma virtual', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler).pipe(
        delay(1000, testScheduler)
      );

      // Emite después de 1000ms (tiempo virtual)
      const expected = '1000ms (abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

> [!TIP]
> **Ventajas de TestScheduler**
>
> - Puedes probar sin esperar tiempo real
> - Puedes probar procesamiento asíncrono de forma síncrona
> - Reduce significativamente el tiempo de ejecución de pruebas

## Ejemplos de Uso Común

### 1. Obtención de datos con paginación

```typescript
import { scheduled, asyncScheduler, mergeMap, toArray } from 'rxjs';
interface Page {
  page: number;
  data: any[];
}

// Lista de números de página
const pages = [1, 2, 3, 4, 5];

// Obtener cada página de forma asíncrona
const allData$ = scheduled(pages, asyncScheduler).pipe(
  mergeMap(page =>
    fetch(`https://api.example.com/items?page=${page}`)
      .then(res => res.json())
  ),
  toArray() // Combinar datos de todas las páginas
);

allData$.subscribe({
  next: data => console.log('Todos los datos:', data),
  complete: () => console.log('Obtención completada')
});
```

### 2. Procesamiento por lotes

```typescript
import { scheduled, asyncScheduler, bufferCount, mergeMap, delay } from 'rxjs';
// Procesar grandes cantidades de tareas en lotes de 1000
const tasks = Array.from({ length: 10000 }, (_, i) => `Tarea-${i}`);

scheduled(tasks, asyncScheduler).pipe(
  bufferCount(1000), // Crear lotes de 1000
  mergeMap(batch => {
    console.log(`Procesando lote: ${batch.length} elementos`);
    // Ejecutar procesamiento de lote
    return processBatch(batch);
  })
).subscribe({
  complete: () => console.log('Procesamiento de todos los lotes completado')
});

function processBatch(batch: string[]): Promise<void> {
  // Lógica de procesamiento de lote
  return Promise.resolve();
}
```

### 3. Implementación de animación

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// Generar valores de 0 a 100
const frames = Array.from({ length: 100 }, (_, i) => i);

// Ejecutar por cada frame de animación
const animation$ = scheduled(frames, animationFrameScheduler).pipe(
  map(frame => ({
    progress: frame / 100,
    position: frame * 5 // Mover de 0px a 500px
  }))
);

animation$.subscribe({
  next: ({ progress, position }) => {
    const element = document.getElementById('animated-box');
    if (element) {
      element.style.transform = `translateX(${position}px)`;
      console.log(`Progreso: ${(progress * 100).toFixed(0)}%`);
    }
  },
  complete: () => console.log('Animación completada')
});
```

### 4. Procesamiento de tareas con prioridad

```typescript
import { scheduled, asapScheduler, asyncScheduler } from 'rxjs';

// Tareas de alta prioridad (asapScheduler = microtask)
const highPriorityTasks = ['Tarea urgente 1', 'Tarea urgente 2'];
const highPriority$ = scheduled(highPriorityTasks, asapScheduler);

// Tareas de baja prioridad (asyncScheduler = macrotask)
const lowPriorityTasks = ['Tarea normal 1', 'Tarea normal 2'];
const lowPriority$ = scheduled(lowPriorityTasks, asyncScheduler);

console.log('Inicio de tareas');

highPriority$.subscribe(task => console.log('Alta prioridad:', task));
lowPriority$.subscribe(task => console.log('Baja prioridad:', task));

console.log('Registro de tareas completado');

// Salida:
// Inicio de tareas
// Registro de tareas completado
// Alta prioridad: Tarea urgente 1
// Alta prioridad: Tarea urgente 2
// Baja prioridad: Tarea normal 1
// Baja prioridad: Tarea normal 2
```

## Opciones de scheduled()

`scheduled()` tiene la siguiente firma:

```typescript
function scheduled<T>(
  input: ObservableInput<T>,
  scheduler: SchedulerLike
): Observable<T>
```

### Tipos de entrada compatibles

- **Array**: `T[]`
- **Promise**: `Promise<T>`
- **Iterable**: `Iterable<T>` (Set, Map, Generator, etc.)
- **Observable**: `Observable<T>`
- **ArrayLike**: `ArrayLike<T>`

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Array
scheduled([1, 2, 3], asyncScheduler);

// Promise
scheduled(Promise.resolve('resultado'), asyncScheduler);

// Set
scheduled(new Set([1, 2, 3]), asyncScheduler);

// Generator
function* generator() {
  yield 1;
  yield 2;
  yield 3;
}
scheduled(generator(), asyncScheduler);
```

## Errores Comunes y Soluciones

### 1. Olvidar especificar el scheduler

**Ejemplo de error:**
```typescript
// ❌ Error: Se requiere el segundo argumento
const observable$ = scheduled([1, 2, 3]);
```

**Solución:**
```typescript
// ✅ Correcto: Especificar scheduler
const observable$ = scheduled([1, 2, 3], asyncScheduler);
```

### 2. Usar animationFrameScheduler en entorno de navegador

**Problema:**
En entorno Node.js, `requestAnimationFrame` no existe, por lo que ocurre un error.

**Solución:**
```typescript
import { scheduled, animationFrameScheduler, asyncScheduler } from 'rxjs';

// Verificar si es entorno de navegador
const scheduler = typeof window !== 'undefined'
  ? animationFrameScheduler
  : asyncScheduler;

const observable$ = scheduled([1, 2, 3], scheduler);
```

### 3. Confusión entre procesamiento síncrono y asíncrono

**Problema:**
```typescript
// Espera ejecución asíncrona, pero en realidad es síncrono
scheduled([1, 2, 3], queueScheduler).subscribe(val => {
  console.log(val);
});
console.log('Completado'); // ← 1, 2, 3 se muestran antes de esto
```

**Solución:**
```typescript
// Especificar claramente asíncrono
scheduled([1, 2, 3], asyncScheduler).subscribe(val => {
  console.log(val);
});
console.log('Completado'); // ← 1, 2, 3 se muestran después de esto
```

## Comparación con from()

| Característica | from() | scheduled() |
|------|--------|-------------|
| Especificación de scheduler | ❌ No posible (solo por defecto) | ✅ Se puede especificar explícitamente |
| Control síncrono/asíncrono | ❌ No controlable | ✅ Controlable |
| Facilidad de pruebas | Normal | ✅ Control de tiempo posible con TestScheduler |
| Simplicidad | ✅ Simple | Algo complejo |
| Escenarios de uso | Conversión básica | Cuando se necesita control del momento de ejecución |

> [!TIP]
> **Puntos para elegir**
>
> - **Básicamente usa `from()`**: Cuando no se necesita control de scheduler
> - **Casos para usar `scheduled()`**:
>   - Quieres evitar bloqueo de UI
>   - Necesitas control de tiempo en pruebas
>   - Implementación de animación
>   - Procesamiento de tareas con prioridad

## Mejores Prácticas

### 1. Usar asyncScheduler para procesamiento de grandes datos

```typescript
// ✅ Buen ejemplo: No bloquea UI
scheduled(largeArray, asyncScheduler).pipe(
  map(processHeavyTask)
).subscribe();
```

### 2. Usar TestScheduler para pruebas

```typescript
// ✅ Buen ejemplo: Controlar tiempo de forma virtual
testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

### 3. Usar animationFrameScheduler para animaciones

```typescript
// ✅ Buen ejemplo: Sincronizar con el momento de re-renderizado del navegador
scheduled(frames, animationFrameScheduler).subscribe(updateUI);
```

### 4. Selección de scheduler según el entorno

```typescript
// ✅ Buen ejemplo: Cambiar según el entorno
const scheduler = process.env.NODE_ENV === 'test'
  ? queueScheduler
  : asyncScheduler;

const source$ = scheduled(data, scheduler);
```

## Resumen

`scheduled()` es una Función de Creación que genera Observable especificando explícitamente un scheduler.

**Características principales:**
- Control explícito del momento de ejecución (síncrono/asíncrono)
- Selección entre múltiples schedulers
- Facilita pruebas con TestScheduler
- Efectivo para evitar bloqueo de UI

**Escenarios de uso:**
- Procesamiento asíncrono de grandes datos
- Implementación de animación
- Control de tiempo en pruebas
- Procesamiento de tareas con prioridad

**Precauciones:**
- Siempre especificar scheduler
- Seleccionar scheduler apropiado según el entorno
- Entender la diferencia con from()

**Uso recomendado:**
- Optimización de UI: `asyncScheduler`
- Animación: `animationFrameScheduler`
- Pruebas: `TestScheduler`
- Alta prioridad: `asapScheduler`

## Páginas Relacionadas

- [using()](/es/guide/creation-functions/control/using) - Observable con control de recursos
- [Funciones de Creación de Control](/es/guide/creation-functions/control/) - Comparación de scheduled() y using()
- [Tipos de Schedulers](/es/guide/schedulers/types) - Detalles de schedulers
- [from()](/es/guide/creation-functions/basic/from) - Generación básica de Observable

## Recursos de Referencia

- [Documentación Oficial RxJS - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [Documentación Oficial RxJS - Scheduler](https://rxjs.dev/guide/scheduler)
- [Documentación Oficial RxJS - TestScheduler](https://rxjs.dev/api/testing/TestScheduler)
