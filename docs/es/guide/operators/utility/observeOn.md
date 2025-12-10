---
description: El operador observeOn controla el momento de emisión de valores de Observable con un scheduler especificado y se utiliza para procesamiento asíncrono y optimización de animaciones.
---

# observeOn - Control de Contexto de Ejecución

El operador `observeOn` controla **el momento de emisión de valores de Observable y el contexto de ejecución** con un scheduler especificado. Las operaciones subsiguientes en un stream pueden ejecutarse en un scheduler específico.

## 🔰 Sintaxis Básica y Operación

Asincroniza el procesamiento subsiguiente especificando un scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Inicio');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(v => console.log('Valor:', v));

console.log('Fin');

// Salida:
// Inicio
// Fin
// Valor: 1
// Valor: 2
// Valor: 3
```

Los procesos anteriores a `observeOn` se ejecutan sincrónicamente, mientras que los procesos posteriores a `observeOn` se ejecutan por el scheduler especificado.

[🌐 Documentación Oficial de RxJS - observeOn](https://rxjs.dev/api/index/function/observeOn)

## 💡 Ejemplos de Uso Típicos

- **Evitar bloqueo de hilo de UI**: Asincronizar procesamiento pesado
- **Optimización de animación**: Renderizado suave con `animationFrameScheduler`
- **Priorizar procesamiento**: Controlar momento de ejecución con diferentes schedulers
- **Control de tarea micro/macro**: Ajustar momento de ejecución

## Tipos de Schedulers

| Scheduler | Características | Casos de Uso |
|:---|:---|:---|
| `asyncScheduler` | Basado en `setTimeout` | Procesamiento asíncrono general |
| `asapScheduler` | Microtareas (Promise.then) | Ejecución asíncrona lo más rápida posible |
| `queueScheduler` | Cola sincrónica | Optimizar procesamiento recursivo |
| `animationFrameScheduler` | `requestAnimationFrame` | Animación, renderizado a 60fps |

> [!TIP]
> Para más información sobre schedulers, consulte [Tipos de Schedulers y Cómo Usarlos](/es/guide/schedulers/types.md).

## 🧪 Ejemplo de Código Práctico 1: Evitar Bloqueo de UI

Este es un ejemplo de ejecución asíncrona de procesamiento de gran cantidad de datos dividido en lotes.

```ts
import { range, asapScheduler } from 'rxjs';
import { observeOn, bufferCount, tap } from 'rxjs';

// Creación de UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'observeOn - Evitar bloqueo de UI';
container.appendChild(title);

const progress = document.createElement('div');
progress.style.marginBottom = '10px';
container.appendChild(progress);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

function addLog(message: string) {
  const logItem = document.createElement('div');
  logItem.style.fontSize = '12px';
  logItem.style.marginBottom = '2px';
  logItem.textContent = message;
  output.appendChild(logItem);
}

const totalItems = 10000;
const batchSize = 100;
const totalBatches = Math.ceil(totalItems / batchSize);
let processedBatches = 0;

addLog('Procesamiento iniciado...');
progress.textContent = 'Progreso: 0%';

range(1, totalItems)
  .pipe(
    bufferCount(batchSize),
    observeOn(asapScheduler),  // Procesar cada lote de forma asíncrona
    tap(batch => {
      // Simular cálculo pesado
      const sum = batch.reduce((acc, n) => acc + n, 0);
      processedBatches++;
      const percent = Math.floor((processedBatches / totalBatches) * 100);
      progress.textContent = `Progreso: ${percent}%`;

      if (processedBatches % 10 === 0 || processedBatches === totalBatches) {
        addLog(`Lote ${processedBatches}/${totalBatches} completado (Total: ${sum})`);
      }
    })
  )
  .subscribe({
    complete: () => {
      addLog('--- Todo el procesamiento completado ---');
      progress.textContent = 'Progreso: 100% ✅';
    }
  });
```

- Procesamiento por lotes de 10,000 elementos de datos, 100 a la vez
- Procesar sin bloquear UI con `asapScheduler`
- Visualización en tiempo real del progreso

## 🧪 Ejemplo de Código Práctico 2: Optimización de Animación

Ejemplo de animación suave usando `animationFrameScheduler`.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { observeOn, take, map } from 'rxjs';

// Creación de UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'observeOn - Animación';
container2.appendChild(title2);

const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = '#4CAF50';
box.style.position = 'relative';
box.style.transition = 'none';
container2.appendChild(box);

let position = 0;

interval(0)
  .pipe(
    observeOn(animationFrameScheduler),  // Ejecutar a 60fps
    take(180),  // 3 segundos (60fps × 3 segundos)
    map(() => {
      position += 2;  // Mover 2px por fotograma
      return position;
    })
  )
  .subscribe({
    next: pos => {
      box.style.left = `${pos}px`;
    },
    complete: () => {
      const message = document.createElement('div');
      message.textContent = 'Animación completada';
      message.style.marginTop = '10px';
      message.style.color = '#4CAF50';
      container2.appendChild(message);
    }
  });
```

- Sincronizar con ciclos de dibujo del navegador con `animationFrameScheduler`
- Animación suave a 60fps
- Pausa automática en pestañas en segundo plano

## 🆚 Diferencias con subscribeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

console.log('=== observeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('Antes de observeOn (sinc)')),
    observeOn(asyncScheduler),
    tap(() => console.log('Después de observeOn (async)'))
  )
  .subscribe();

console.log('=== subscribeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('Después de subscribeOn (async)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Salida:
// === observeOn ===
// Antes de observeOn (sinc)
// Antes de observeOn (sinc)
// Antes de observeOn (sinc)
// === subscribeOn ===
// Después de observeOn (async)
// Después de observeOn (async)
// Después de observeOn (async)
// Después de subscribeOn (async)
// Después de subscribeOn (async)
// Después de subscribeOn (async)
```

| Operador | Alcance de Efectos | Control de Temporización |
|:---|:---|:---|
| `observeOn` | Solo procesos subsiguientes | Momento de emisión del valor |
| `subscribeOn` | Stream completo | Momento de inicio de suscripción |

> [!NOTE]
> Para más información sobre `subscribeOn`, consulte [subscribeOn](./subscribeOn.md).

## ⚠️ Notas Importantes

### 1. La Posición de Colocación es Importante

La ubicación de `observeOn` determina qué procesos se asincronizan.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, map, tap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Proceso 1 (sinc)')),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Async desde aquí
    tap(() => console.log('Proceso 2 (async)')),
    map(x => x + 10)
  )
  .subscribe();

// El Proceso 1 es sincrónico, el Proceso 2 es asíncrono
```

### 2. Múltiples observeOn No Son Acumulativos

```ts
import { of, asyncScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    observeOn(queueScheduler)  // Se aplica el último scheduler
  )
  .subscribe();
```

Se usa el scheduler del último `observeOn` (en este caso `queueScheduler`).

### 3. Impacto en el Rendimiento

El uso frecuente de `observeOn` tiene una sobrecarga.

```ts
import { asyncScheduler, range, map, bufferCount, concatMap, from } from 'rxjs';
import { observeOn } from 'rxjs';

// ❌ Mal ejemplo: Asincronizar para cada valor
range(1, 1000)
  .pipe(
    map(x => x * 2),
    observeOn(asyncScheduler)  // 1000 setTimeouts
  )
  .subscribe();

// ✅ Buen ejemplo: Procesamiento por lotes
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeouts
    concatMap(batch => from(batch).pipe(map(x => x * 2)))
  )
  .subscribe();
```

## Comparación de Momento de Ejecución

```ts
import { of, asyncScheduler, asapScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inicio');

// Procesamiento sincrónico
of('sinc').subscribe(v => console.log(`2: ${v}`));

// queueScheduler
of('cola')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(v => console.log(`5: ${v}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Fin');

// Orden de ejecución:
// 1: Inicio
// 2: sinc
// 7: Fin
// 3: cola
// 4: asap
// 6: Promise
// 5: async
```

## 📚 Operadores Relacionados

- **[subscribeOn](./subscribeOn)** - Controlar momento de inicio de suscripción
- **[delay](./delay)** - Retardo de tiempo fijo
- **[debounceTime](../filtering/debounceTime)** - Retardo después de que se detenga la entrada

## 📖 Documentos Relacionados

- **[Control de Procesamiento Asíncrono](/es/guide/schedulers/async-control.md)** - Fundamentos de Scheduler
- **[Tipos y Uso de Schedulers](/es/guide/schedulers/types.md)** - Detalles de cada scheduler

## ✅ Resumen

El operador `observeOn` controla cuándo se emiten valores y el contexto de ejecución.

- ✅ Ejecutar procesos subsiguientes con el scheduler especificado
- ✅ Útil para evitar bloqueos de UI
- ✅ Utilizado para optimización de animaciones
- ✅ Permite priorización del procesamiento
- ⚠️ La posición de colocación es importante
- ⚠️ Tenga en cuenta la sobrecarga de rendimiento
- ⚠️ Al usar múltiples schedulers, se aplica el último scheduler
