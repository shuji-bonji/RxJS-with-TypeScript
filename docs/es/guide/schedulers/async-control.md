---
description: Esta página explica cómo usar los schedulers en RxJS y aprender a controlar el procesamiento asíncrono usando observeOn y subscribeOn. Se introducen técnicas prácticas para la optimización del rendimiento y evitar el bloqueo de la UI, como el control del tiempo de ejecución, gestión del contexto de ejecución y priorización de tareas, con ejemplos de código TypeScript.
---
# Control del procesamiento asíncrono

El scheduler en RxJS es un mecanismo importante para controlar el tiempo y el contexto de ejecución del procesamiento asíncrono. Este capítulo explica cómo se utiliza el scheduler para controlar el procesamiento asíncrono.

## Rol del Scheduler

El scheduler desempeña los siguientes tres roles importantes

|Rol|Descripción|
|---|---|
|Controlar el tiempo de ejecución|Decidir cuándo ejecutar tareas|
|Gestionar el contexto de ejecución|Determinar en qué hilos y entorno de ejecución ejecutar las tareas|
|Priorización de tareas|Gestionar el orden de ejecución de múltiples tareas|

## Entender el procesamiento síncrono y asíncrono

### Comportamiento predeterminado (ejecución síncrona)

Por defecto, los operadores de RxJS se ejecutan de forma tan síncrona como sea posible.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

console.log('Inicio de ejecución');

of(1, 2, 3)
  .pipe(
    map((x) => {
      console.log(`map: ${x}`);
      return x * 2;
    })
  )
  .subscribe((x) => console.log(`subscribe: ${x}`));

console.log('Fin de ejecución');

// Salida:
// Inicio de ejecución
// map: 1
// subscribe: 2
// map: 2
// subscribe: 4
// map: 3
// subscribe: 6
// Fin de ejecución
```

### Asincronización con scheduler

El procesamiento puede hacerse asíncrono utilizando el scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Inicio de ejecución');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(x => console.log(`subscribe: ${x}`));

console.log('Fin de ejecución');

// Salida:
// Inicio de ejecución
// Fin de ejecución
// subscribe: 1
// subscribe: 2
// subscribe: 3
```

## Operadores que usan el scheduler

### Operador observeOn

El operador `observeOn` cambia el contexto de ejecución de un stream. Emite valores con el scheduler especificado.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { take, observeOn } from 'rxjs';

// Ejemplo de uso para animación
interval(16)
  .pipe(
    take(10),
    observeOn(animationFrameScheduler)
  )
  .subscribe(() => {
    // Ejecutar en sincronía con los frames de animación
    updateAnimation();
  });

function updateAnimation() {
  // Procesamiento de actualización de animación
}
```

> [!TIP]
> Para explicaciones detalladas, ejemplos prácticos y precauciones sobre el operador `observeOn`, consulte la página del operador [observeOn](../operators/utility/observeOn.md).

### Operador subscribeOn

El operador `subscribeOn` controla cuándo comenzar a suscribirse a un stream.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, tap } from 'rxjs';

console.log('Antes de iniciar suscripción');

of('Ejecución de tarea')
  .pipe(
    tap(() => console.log('Inicio de tarea')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(value => console.log(value));

console.log('Después de iniciar suscripción');

// Salida:
// Antes de iniciar suscripción
// Después de iniciar suscripción
// Inicio de tarea
// Ejecución de tarea
```

> [!TIP]
> Para explicaciones detalladas, ejemplos prácticos y diferencias con `observeOn`, consulte la página del operador [subscribeOn](../operators/utility/subscribeOn.md).

## Ejemplos prácticos de procesamiento asíncrono

### Controlar solicitudes de API

Este es un ejemplo de poner en cola solicitudes y procesarlas en orden.

```ts
import { from, queueScheduler } from 'rxjs';
import { mergeMap, observeOn, tap } from 'rxjs';

interface ApiRequest {
  endpoint: string;
  id: number;
}

const requests: ApiRequest[] = [
  { endpoint: '/users', id: 1 },
  { endpoint: '/posts', id: 1 },
  { endpoint: '/comments', id: 1 },
];

// Poner en cola solicitudes y procesarlas en orden
from(requests)
  .pipe(
    observeOn(queueScheduler),
    tap((req) => console.log(`Agregado a la cola: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // Simular solicitud de API real
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`${req.endpoint}/${req.id} resultado`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`Completado: ${result}`));

// Salida:
// Agregado a la cola: /users
// Agregado a la cola: /posts
// Agregado a la cola: /comments
// Completado: /users/1 resultado
// Completado: /posts/1 resultado
// Completado: /comments/1 resultado
```

### Evitar bloquear hilos de UI

Utilizar el scheduler para evitar bloquear hilos de UI cuando se procesan grandes cantidades de datos.

```ts
import { from, asapScheduler } from 'rxjs';
import { observeOn, bufferCount } from 'rxjs';

const largeDataSet = Array.from({ length: 10000 }, (_, i) => i);

// Tamaño del lote
const batchSize = 100;
// Calcular número total de lotes
const totalBatches = Math.ceil(largeDataSet.length / batchSize);
// Contador de lotes
let batchIndex = 0;

from(largeDataSet)
  .pipe(
    bufferCount(100), // Agrupar 100 elementos a la vez
    observeOn(asapScheduler) // Lo antes posible, pero no bloquea la UI
  )
  .subscribe((batch) => {
    batchIndex++;
    processBatch(batch, batchIndex, totalBatches);
  });

function processBatch(
  batch: number[],
  batchIndex: number,
  totalBatches: number
) {
  // Procesar datos del lote
  const processed = batch.map((n) => n * 2);
  console.log(
    `Lote ${batchIndex} de ${totalBatches} completado: ${processed.length} elementos procesados.`
  );
}

// Salida:
// Lote 1 de 100 completado: 100 elementos procesados.
// Lote 2 de 100 completado: 100 elementos procesados.
// ...
// ...
// Lote 100 de 100 completado: 100 elementos procesados.
```

## Optimización del rendimiento y depuración

### Pruebas con el Scheduler

El siguiente es un ejemplo de prueba de procesamiento asíncrono usando TestScheduler.

```ts
import { TestScheduler } from 'rxjs/testing';
import { delay } from 'rxjs';
import { beforeEach, describe, expect, it } from 'vitest';

describe('Prueba de procesamiento asíncrono', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Prueba del operador delay', () => {
    scheduler.run(({ cold, expectObservable }) => {
      const source = cold('a-b-c|');
      const expected =    '1000ms a-b-(c|)';

      const result = source.pipe(delay(1000, scheduler));

      expectObservable(result).toBe(expected);
    });
  });
});
```

### Salida de logs para depuración

El siguiente es un ejemplo de salida de logs para verificar el funcionamiento del scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { tap, observeOn } from 'rxjs';

console.log('Inicio');

of(1, 2, 3)
  .pipe(
    tap(value => console.log(`[Antes del scheduler - sync] Valor: ${value}`)),
    observeOn(asyncScheduler),  // Usar asyncScheduler
    tap(value => console.log(`[Después del scheduler - async] Valor: ${value}`))
  )
  .subscribe();

console.log('Fin');

// Salida real:
// Inicio
// [Antes del scheduler - sync] Valor: 1
// [Antes del scheduler - sync] Valor: 2
// [Antes del scheduler - sync] Valor: 3
// Fin
// [Después del scheduler - async] Valor: 1
// [Después del scheduler - async] Valor: 2
// [Después del scheduler - async] Valor: 3
```

Usando `asyncScheduler`, puede verificar el comportamiento asíncrono como se esperaba. Mientras que `queueScheduler` usa una cola de microtareas, que se procesa durante la ejecución del código síncrono, `asyncScheduler` usa setTimeout internamente, por lo que se ejecuta completamente de forma asíncrona.

## Ejemplo mostrando diferencias en el comportamiento del scheduler
Este ejemplo muestra la diferencia en el tiempo de ejecución de diferentes schedulers.

```ts
import { of, queueScheduler, asyncScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inicio');

// Procesamiento síncrono
of('sync').subscribe(value => console.log(`2: ${value}`));

// queueScheduler (microtarea)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`3: ${value}`));

// asapScheduler (microtarea)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`4: ${value}`));

// asyncScheduler (macrotarea)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`5: ${value}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Fin');

// Orden de ejecución real:
// 1: Inicio
// 2: sync
// 7: Fin
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```


## Mejores Prácticas

1. **Usar el scheduler solo cuando sea necesario**: Si el comportamiento síncrono predeterminado es suficiente, no forzar el uso del scheduler.

2. **Seleccionar el scheduler apropiado**: Seleccionar el mejor scheduler para su aplicación.
   - Animación: `animationFrameScheduler`
   - Evitar bloqueo de UI: `asapScheduler`
   - Procesamiento en cola: `queueScheduler`
   - Procesamiento asíncrono: `asyncScheduler`

3. **Monitoreo del rendimiento**: monitorear constantemente el impacto en el rendimiento del uso del scheduler

4. **Facilidad de pruebas**: usar `TestScheduler` para escribir pruebas para el procesamiento asíncrono.

## Errores comunes y contramedidas

### Desincronización excesiva

Este es un ejemplo de evitar la asincronización innecesaria y asincronizar solo donde sea necesario.

```ts
// ❌ Asincronización innecesaria
of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Asincronización duplicada
    filter(x => x > 3)
  )
  .subscribe();

// ✅ Asincronizar solo donde sea necesario
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    filter(x => x > 3),
    observeOn(asyncScheduler)  // Asincronizar todo de una vez al final
  )
  .subscribe();
```

### Mal uso del scheduler

Esta es una comparación de uso incorrecto y correcto.

```ts
// ❌ Uso incorrecto
interval(1000)
  .pipe(
    subscribeOn(animationFrameScheduler)  // No afecta al interval
  )
  .subscribe();

// ✅ Uso correcto
interval(1000, animationFrameScheduler)  // Especificar scheduler al momento de creación
  .subscribe();
```

## Resumen

El scheduler es una herramienta poderosa para el control fino del procesamiento asíncrono en RxJS. Usado apropiadamente, puede optimizar el rendimiento, evitar el bloqueo de hilos de UI y facilitar las pruebas. Sin embargo, es importante usarlo solo cuando sea necesario, ya que la asincronización excesiva puede empeorar el rendimiento.

En la siguiente sección, discutiremos en detalle los diferentes tipos de schedulers y cómo usarlos.
