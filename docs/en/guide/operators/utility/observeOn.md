---
description: The observeOn operator controls the timing of issuing Observable values with a specified scheduler and is used for asynchronous processing and animation optimization.
---

# observeOn - Control of Execution Context

The `observeOn` operator controls **the timing of issuing Observable's values and the execution context** with a specified scheduler. Subsequent operations in a stream can be made to execute on a specific scheduler.

## 🔰 Basic Syntax and Operation

Asynchronizes subsequent processing by specifying a scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(v => console.log('Value:', v));

console.log('End');

// Output:
// Start
// End
// Value: 1
// Value: 2
// Value: 3
```

Processes prior to `observeOn` are executed synchronously, while processes after `observeOn` are executed by the specified scheduler.

[🌐 RxJS Official Documentation - observeOn](https://rxjs.dev/api/index/function/observeOn)

## 💡 Typical Usage Examples

- **UI thread block avoidance**: Asynchronize heavy processing
- **Optimization of animation**: Smooth rendering with `animationFrameScheduler`
- **Prioritize processing**: Control execution timing with different schedulers
- **Micro/macro task control**: Fine-tune execution timing

## Types of Schedulers

| Scheduler | Features | Use Cases |
|:---|:---|:---|
| `asyncScheduler` | Based on `setTimeout` | General asynchronous processing |
| `asapScheduler` | Microtasks (Promise.then) | As fast as possible asynchronous execution |
| `queueScheduler` | Synchronous queue | Optimize recursive processing |
| `animationFrameScheduler` | `requestAnimationFrame` | Animation, 60fps rendering |

> [!TIP]
> For more information on schedulers, see [Types of Schedulers and How to Use Them](/en/guide/schedulers/types.md).

## 🧪 Practical Code Example 1: UI Block Avoidance

This is an example of asynchronous execution of a large amount of data processing divided into batches.

```ts
import { range, asapScheduler } from 'rxjs';
import { observeOn, bufferCount, tap } from 'rxjs';

// UI creation
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'observeOn - UI block avoidance';
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

addLog('Processing started...');
progress.textContent = 'Progress: 0%';

range(1, totalItems)
  .pipe(
    bufferCount(batchSize),
    observeOn(asapScheduler),  // Process each batch asynchronously
    tap(batch => {
      // Simulate heavy calculation
      const sum = batch.reduce((acc, n) => acc + n, 0);
      processedBatches++;
      const percent = Math.floor((processedBatches / totalBatches) * 100);
      progress.textContent = `Progress: ${percent}%`;

      if (processedBatches % 10 === 0 || processedBatches === totalBatches) {
        addLog(`Batch ${processedBatches}/${totalBatches} completed (Total: ${sum})`);
      }
    })
  )
  .subscribe({
    complete: () => {
      addLog('--- All processing completed ---');
      progress.textContent = 'Progress: 100% ✅';
    }
  });
```

- Batch processing of 10,000 data items, 100 at a time
- Process without blocking UI with `asapScheduler`
- Real-time display of progress

## 🧪 Practical Code Example 2: Animation Optimization

Smooth animation example using `animationFrameScheduler`.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { observeOn, take, map } from 'rxjs';

// UI creation
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'observeOn - Animation';
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
    observeOn(animationFrameScheduler),  // Execute at 60fps
    take(180),  // 3 seconds (60fps × 3 seconds)
    map(() => {
      position += 2;  // Move 2px per frame
      return position;
    })
  )
  .subscribe({
    next: pos => {
      box.style.left = `${pos}px`;
    },
    complete: () => {
      const message = document.createElement('div');
      message.textContent = 'Animation completed';
      message.style.marginTop = '10px';
      message.style.color = '#4CAF50';
      container2.appendChild(message);
    }
  });
```

- Synchronize with browser drawing cycles with `animationFrameScheduler`
- Smooth 60fps animation
- Automatic pause on background tabs

## 🆚 Differences from subscribeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

console.log('=== observeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('Before observeOn (sync)')),
    observeOn(asyncScheduler),
    tap(() => console.log('After observeOn (async)'))
  )
  .subscribe();

console.log('=== subscribeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('After subscribeOn (async)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Output:
// === observeOn ===
// Before observeOn (sync)
// Before observeOn (sync)
// Before observeOn (sync)
// === subscribeOn ===
// After observeOn (async)
// After observeOn (async)
// After observeOn (async)
// After subscribeOn (async)
// After subscribeOn (async)
// After subscribeOn (async)
```

| Operator | Scope of Effects | Timing Control |
|:---|:---|:---|
| `observeOn` | Subsequent processes only | Timing for issuing value |
| `subscribeOn` | Entire stream | Timing for starting subscription |

> [!NOTE]
> For more information on `subscribeOn`, see [subscribeOn](./subscribeOn.md).

## ⚠️ Important Notes

### 1. Placement Position is Important

The location of `observeOn` determines which processes are asynchronized.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, map, tap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Process 1 (sync)')),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Async from here
    tap(() => console.log('Process 2 (async)')),
    map(x => x + 10)
  )
  .subscribe();

// Process 1 is synchronous, Process 2 is asynchronous
```

### 2. Multiple observeOn Are Not Cumulative

```ts
import { of, asyncScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    observeOn(queueScheduler)  // Last scheduler is applied
  )
  .subscribe();
```

The last `observeOn` scheduler (in this case `queueScheduler`) is used.

### 3. Performance Impact

Frequent use of `observeOn` has an overhead.

```ts
import { asyncScheduler, range, map, bufferCount, concatMap, from } from 'rxjs';
import { observeOn } from 'rxjs';

// ❌ Bad example: Asynchronize for each value
range(1, 1000)
  .pipe(
    map(x => x * 2),
    observeOn(asyncScheduler)  // 1000 setTimeouts
  )
  .subscribe();

// ✅ Good example: Batch processing
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeouts
    concatMap(batch => from(batch).pipe(map(x => x * 2)))
  )
  .subscribe();
```

## Comparison of Execution Timing

```ts
import { of, asyncScheduler, asapScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

// Synchronous processing
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler
of('queue')
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

console.log('7: End');

// Execution order:
// 1: Start
// 2: sync
// 7: End
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```

## 📚 Related Operators

- **[subscribeOn](./subscribeOn)** - Control timing of subscription start
- **[delay](./delay)** - Fixed time delay
- **[debounceTime](../filtering/debounceTime)** - Delay after input stops

## 📖 Related Documents

- **[Asynchronous Processing Control](/en/guide/schedulers/async-control.md)** - Scheduler Basics
- **[Types and Usage of Schedulers](/en/guide/schedulers/types.md)** - Details of each scheduler

## ✅ Summary

The `observeOn` operator controls when values are issued and the execution context.

- ✅ Execute subsequent processes with the specified scheduler
- ✅ Useful for avoiding UI blocks
- ✅ Utilized for animation optimization
- ✅ Allows prioritization of processing
- ⚠️ Placement position is important
- ⚠️ Be aware of performance overhead
- ⚠️ When using multiple schedulers, the last scheduler is applied
