---
description: This page explains how to use schedulers in RxJS and learn how to control asynchronous processing using observeOn and subscribeOn. Practical techniques for performance optimization and UI blocking avoidance, such as execution timing control, execution context management, and task prioritization, are introduced with TypeScript code examples.
---
# Control of asynchronous processing

The scheduler in RxJS is an important mechanism for controlling the timing and execution context of asynchronous processing. This chapter explains how the scheduler is used to control asynchronous processing.

## Role of the Scheduler

The scheduler plays the following three important roles

|Role|Description|
|---|---|
|Control execution timing|decide when to execute tasks|
|Manage execution context|Determine which threads and execution environment to run tasks in|
|Task prioritization|Manage the execution order of multiple tasks|

## Understanding synchronous and asynchronous processing

### Default behavior (synchronous execution)

By default, RxJS operators are executed as synchronously as possible.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

console.log('Start execution');

of(1, 2, 3)
  .pipe(
    map((x) => {
      console.log(`map: ${x}`);
      return x * 2;
    })
  )
  .subscribe((x) => console.log(`subscribe: ${x}`));

console.log('End execution');

// Output:
// Start execution
// map: 1
// subscribe: 2
// map: 2
// subscribe: 4
// map: 3
// subscribe: 6
// End execution
```

### Asynchronization with scheduler

Processing can be asynchronized by using the scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Start execution');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(x => console.log(`subscribe: ${x}`));

console.log('End execution');

// Output:
// Start execution
// End execution
// subscribe: 1
// subscribe: 2
// subscribe: 3
```

## Operators using the scheduler

### observeOn operator

The `observeOn` operator changes the execution context of a stream. It issues values with the specified scheduler.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { take, observeOn } from 'rxjs';

// Example usage for animation
interval(16)
  .pipe(
    take(10),
    observeOn(animationFrameScheduler)
  )
  .subscribe(() => {
    // Execute in sync with animation frames
    updateAnimation();
  });

function updateAnimation() {
  // Animation update processing
}
```

> [!TIP]
> For detailed explanations, practical examples, and precautions regarding the `observeOn` operator, see the [observeOn](../operators/utility/observeOn.md) operator page.

### subscribeOn operator

The `subscribeOn` operator controls when to start subscribing to a stream.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, tap } from 'rxjs';

console.log('Before subscription start');

of('Task execution')
  .pipe(
    tap(() => console.log('Task start')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(value => console.log(value));

console.log('After subscription start');

// Output:
// Before subscription start
// After subscription start
// Task start
// Task execution
```

> [!TIP]
> For detailed explanations, practical examples, and differences from `observeOn`, see the [subscribeOn](../operators/utility/subscribeOn.md) operator page.

## Practical examples of asynchronous processing

### Controlling API requests

This is an example of queuing requests and processing them in order.

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

// Queue requests and process them in order
from(requests)
  .pipe(
    observeOn(queueScheduler),
    tap((req) => console.log(`Added to queue: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // Simulate actual API request
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`${req.endpoint}/${req.id} result`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`Completed: ${result}`));

// Output:
// Added to queue: /users
// Added to queue: /posts
// Added to queue: /comments
// Completed: /users/1 result
// Completed: /posts/1 result
// Completed: /comments/1 result
```

### Avoid blocking UI threads

Utilize the scheduler to avoid blocking UI threads when processing large amounts of data.

```ts
import { from, asapScheduler } from 'rxjs';
import { observeOn, bufferCount } from 'rxjs';

const largeDataSet = Array.from({ length: 10000 }, (_, i) => i);

// Batch size
const batchSize = 100;
// Calculate total number of batches
const totalBatches = Math.ceil(largeDataSet.length / batchSize);
// Batch counter
let batchIndex = 0;

from(largeDataSet)
  .pipe(
    bufferCount(100), // Bundle 100 items at a time
    observeOn(asapScheduler) // As soon as possible, but does not block UI
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
  // Process batch data
  const processed = batch.map((n) => n * 2);
  console.log(
    `Batch ${batchIndex} of ${totalBatches} completed: ${processed.length} items processed.`
  );
}

// Output:
// Batch 1 of 100 completed: 100 items processed.
// Batch 2 of 100 completed: 100 items processed.
// ...
// ...
// Batch 100 of 100 completed: 100 items processed.
```

## Performance optimization and debugging

### Testing with the Scheduler

The following is an example of testing asynchronous processing using TestScheduler.

```ts
import { TestScheduler } from 'rxjs/testing';
import { delay } from 'rxjs';
import { beforeEach, describe, expect, it } from 'vitest';

describe('Asynchronous processing test', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Test delay operator', () => {
    scheduler.run(({ cold, expectObservable }) => {
      const source = cold('a-b-c|');
      const expected =    '1000ms a-b-(c|)';

      const result = source.pipe(delay(1000, scheduler));

      expectObservable(result).toBe(expected);
    });
  });
});
```

### Log output for debugging

The following is an example of log output to check the operation of the scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { tap, observeOn } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    tap(value => console.log(`[Before scheduler - sync] Value: ${value}`)),
    observeOn(asyncScheduler),  // Use asyncScheduler
    tap(value => console.log(`[After scheduler - async] Value: ${value}`))
  )
  .subscribe();

console.log('End');

// Actual output:
// Start
// [Before scheduler - sync] Value: 1
// [Before scheduler - sync] Value: 2
// [Before scheduler - sync] Value: 3
// End
// [After scheduler - async] Value: 1
// [After scheduler - async] Value: 2
// [After scheduler - async] Value: 3
```

Using `asyncScheduler`, you can check the asynchronous behavior as expected. While `queueScheduler` uses a microtask queue, which is processed during the execution of synchronous code, `asyncScheduler` uses setTimeout internally, so it runs completely asynchronously.

## Example showing differences in scheduler behavior
This example shows the difference in execution timing of different schedulers.

```ts
import { of, queueScheduler, asyncScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

// Synchronous processing
of('sync').subscribe(value => console.log(`2: ${value}`));

// queueScheduler (microtask)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`3: ${value}`));

// asapScheduler (microtask)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`4: ${value}`));

// asyncScheduler (macrotask)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`5: ${value}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: End');

// Actual output order:
// 1: Start
// 2: sync
// 7: End
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```


## Best Practices

1. **Use the scheduler only when necessary**: If the default synchronous behavior is sufficient, do not force the use of the scheduler.

2. **Select the appropriate scheduler**: Select the best scheduler for your application.
   - Animation: `animationFrameScheduler`
   - UI block avoidance: `asapScheduler`
   - Queue processing: `queueScheduler`
   - Asynchronous processing: `asyncScheduler`

3. **Performance Monitoring**: constantly monitor performance impact of scheduler usage

4. **Ease of testing**: use `TestScheduler` to write tests for asynchronous processing.

## Common mistakes and countermeasures

### Excessive desynchronization

This is an example of avoiding unnecessary asynchronization and asynchronizing only where necessary.

```ts
// ❌ Unnecessary asynchronization
of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Duplicate asynchronization
    filter(x => x > 3)
  )
  .subscribe();

// ✅ Asynchronize only where necessary
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    filter(x => x > 3),
    observeOn(asyncScheduler)  // Asynchronize all at once at the end
  )
  .subscribe();
```

### Misuse of scheduler

This is a comparison of incorrect and correct usage.

```ts
// ❌ Incorrect usage
interval(1000)
  .pipe(
    subscribeOn(animationFrameScheduler)  // Does not affect interval
  )
  .subscribe();

// ✅ Correct usage
interval(1000, animationFrameScheduler)  // Specify scheduler at creation time
  .subscribe();
```

## Summary

The scheduler is a powerful tool for fine control of asynchronous processing in RxJS. Used properly, it can optimize performance, avoid blocking UI threads, and facilitate testing. However, it is important to use it only when necessary, as excessive asynchronization may actually worsen performance.

In the next section, we will discuss in detail the different types of schedulers and how to use them.
