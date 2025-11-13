---
description: This page explains in detail the characteristics, implementations, and applications of major schedulers in RxJS, such as asyncScheduler and queueScheduler. Understand the differences between macro tasks, micro tasks, and synchronous processing, and learn the execution timing and characteristics of each scheduler. By using them properly, you can optimize the performance and behavior of your application.
---

# Types of Schedulers and How to Use Them

RxJS provides multiple schedulers for different applications. Each scheduler has its own specific execution timing and characteristics, and appropriate use of each can optimize the performance and behavior of your application.

## Classification of Schedulers

RxJS schedulers fall into three main categories.

1. **Macro Task**: executed in the next task queue in the event loop
2. **Micro-task**: executed immediately after the current task is completed and before the next task starts
3. **Synchronous processing**: immediate execution

For more information, please refer to [Task and Scheduler Basics](./task-and-scheduler-basics.md) for details.

## Major schedulers

### asyncScheduler

#### Features
- **Internal implementation**: uses setTimeout
- **Execution timing**: macro tasks
- **Usage**: General asynchronous processing, time-lapse processing

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Asynchronous processing')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: End');

// Output:
// 1: Start
// 2: End
// 3: Asynchronous processing
```

#### Use Cases

This example simulates a heavy computation process.

```ts
import { asyncScheduler, map, observeOn, of } from "rxjs";

function heavyComputation(value: number): number {
  // Simulate heavy computation
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result = Math.sin(result);
  }
  return result;
}

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(value => heavyComputation(value))
  )
  .subscribe(result => {
    console.log(`Calculation result: ${result}`);
  });
```

### queueScheduler

#### Features
- **Internal implementation**: micro task queue
- **Execution timing**: within the current task (appears synchronous)
- **Usage**: Task queuing, recursion optimization

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Queue processing')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: End');

// Output:
// 1: Start
// 2: Queue processing
// 3: End
```

#### Use Cases

This is an example of optimizing a recursive process.

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs';

// Optimization of recursive processing
function fibonacci(n: number): Observable<number> {
  return of([0, 1]).pipe(
    observeOn(queueScheduler),
    expand(([a, b]) => of([b, a + b])),
    map(([a]) => a),
    take(n)
  );
}

fibonacci(10).subscribe(value => console.log(value));
```

### asapScheduler

#### Features
- **Internal implementation**: Promise.resolve().then() or setImmediate
- **Execution timing**: microtasks
- **Use**: For asynchronous execution as soon as possible

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('ASAP processing')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: End');

// Output:
// 1: Start
// 2: End
// 3: ASAP processing
```

#### Use Cases

This is an example of optimizing mouse movement events.

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Mouse movement event optimization
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // UI update processing
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Features
- **Internal implementation**: requestAnimationFrame
- **Execution timing**: before next screen rendering
- **Use**: Animation, drawing process for 60fps

#### Example of a simple rotation animation

This is an example of rotating a circle element in HTML.

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// Create HTML element
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Animation settings
let rotation = 0;

// Animate at 60fps for 2 seconds
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2 seconds = 120 frames
    map(() => {
      rotation += 3;  // Rotate 3 degrees per frame
      return rotation;
    })
  )
  .subscribe(angle => {
    // Actually rotate the DOM element
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Why animationFrameScheduler?

The `animationFrameScheduler` performs synchronously with the browser's drawing cycle, which offers the following advantages

1. **Smooth Animation**: Because processing is performed in sync with the browser's rendering timing (typically 60 fps), smooth animation without any choppiness can be achieved.
2. **Efficient resource use**: When the browser deactivates the tab, the execution of requestAnimationFrame is automatically paused to avoid unnecessary CPU usage.
3. **Anti-flickering**: Ensures computation is completed before the screen is drawn, preventing flickering and displaying incomplete frames.

The following is a comparison of `setInterval` and `animationFrameScheduler`.

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ Inefficient animation using setInterval
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // approx. 60fps

// Problems:
// - Not synchronized with browser rendering timing
// - Continues to run even on background tabs
// - Unable to guarantee accurate 60fps

// ✅ Efficient animation using animationFrameScheduler
interval(0, animationFrameScheduler)
  .pipe(
    map(() => {
      position += 1;
      return position;
    })
  )
  .subscribe(pos => {
    element.style.transform = `translateX(${pos}px)`;
  });

// Benefits
// - Synchronizes with browser rendering timing
// - Automatically pauses on background tabs
// - Achieves stable 60fps
```


#### Example of mouse-following animation

Create a circle animation that follows the mouse cursor.

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Create a following circle
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Let mouse events pass through
document.body.appendChild(circle);

// Current and target positions
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Monitor mouse movement events
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove')
  .pipe(
    map(event => ({
      x: event.clientX,
      y: event.clientY
    }))
  );

// Animation loop
interval(0, animationFrameScheduler)
  .pipe(
    withLatestFrom(mouseMove$),
    map(([_, mousePos]) => mousePos)
  )
  .subscribe(({ x, y }) => {
    // Set mouse position as target
    targetX = x;
    targetY = y;

    // Gradually move from current position to target position (easing)
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;

    // Update DOM element
    circle.style.left = `${currentX - 15}px`;  // Adjust for center position
    circle.style.top = `${currentY - 15}px`;
  });
```

## Guide to using the scheduler

### Comparison by execution timing

The following is an example comparing the execution order of each scheduler.

```ts
import { of, asyncScheduler, queueScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

// Synchronous processing
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler (microtask)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler (microtask)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler (macrotask)
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

### Selection Criteria by Use

The following is a summary of the features and suitable applications of each scheduler.

| Scheduler | Features | Suitable Uses |
|--------------|------|----------|
| asyncScheduler | Uses setTimeout, fully asynchronous | Time-consuming processing, delayed execution |
| queueScheduler | Synchronous but optimizes recursion | Recursive processing, task queue management |
| asapScheduler | Asynchronous execution as fast as possible | Event handling, fast response processing |
| animationFrameScheduler | Synchronized with screen rendering | Animation, UI updates, game development |

## Practical use cases

### Processing large amounts of data

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
```

### WebSocket message handling

This is an example of WebSocket message processing that requires a fast response.

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// Note: This is pseudo-code to illustrate the concept
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // Treat as string
});

socket$
  .pipe(
    // Message processing requiring fast response
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('Message received:', msg);
}
```

### Error retry control

By utilizing the scheduler with the `retry` operator, the timing of retries can be finely controlled.

#### Basic retry control

The `delay` option of the `retry` operator internally uses the `asyncScheduler` to control the retry interval.

```ts
import { throwError, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

// API call simulation
function fetchData(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.7) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Network error'));
    })
  );
}

fetchData(1)
  .pipe(
    retry({
      count: 3,
      delay: 1000  // Wait 1 second with asyncScheduler before retrying
    })
  )
  .subscribe({
    next: result => console.log('✅ Success:', result),
    error: error => console.log('❌ Final error:', error.message)
  });
```

#### Scheduler utilization in exponential back-off

For more advanced control, exponential backoff can be implemented by combining `retryWhen` and `asyncScheduler`.

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, mergeMap, tap } from 'rxjs';

function fetchDataWithBackoff(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.9) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Temporary error'));
    })
  );
}

fetchDataWithBackoff(1)
  .pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;

          // Check maximum retry count
          if (retryCount > 3) {
            console.log('❌ Maximum retry count reached');
            throw error;
          }

          // Exponential backoff: 1 second, 2 seconds, 4 seconds...
          const delayTime = Math.pow(2, index) * 1000;
          console.log(`🔄 Retry ${retryCount} times (after ${delayTime}ms)`);

          // timer internally uses asyncScheduler
          return timer(delayTime);
        })
      )
    )
  )
  .subscribe({
    next: result => console.log('✅ Success:', result),
    error: error => console.log('❌ Final error:', error.message)
  });

// Sample output:
// 🔄 Retry 1 times (after 1000ms)
// 🔄 Retry 2 times (after 2000ms)
// 🔄 Retry 3 times (after 4000ms)
// ❌ Maximum retry count reached
// ❌ Final error: Temporary error
```

#### When asyncScheduler is explicitly specified

Explicitly specifying a specific scheduler allows for more flexible control, such as replacing it with `TestScheduler` during testing.

```ts
import { throwError, asyncScheduler, of } from 'rxjs';
import { retryWhen, mergeMap, delay } from 'rxjs';

function fetchDataWithScheduler(id: number, scheduler = asyncScheduler) {
  return of(id).pipe(
    mergeMap(() => throwError(() => new Error('Error'))),
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          if (index >= 2) throw error;

          // Explicitly specify scheduler
          return of(null).pipe(
            delay(1000, scheduler)
          );
        })
      )
    )
  );
}

// Production environment: use asyncScheduler
fetchDataWithScheduler(1).subscribe({
  error: err => console.log('Error:', err.message)
});

// Test environment: can be replaced with TestScheduler
```

> [!TIP]
> For detailed implementation patterns and debugging methods for retry processing, see the [retry and catchError](/en/guide/error-handling/retry-catch) page.
> - Detailed usage of the retry operator
> - Combination patterns with catchError
> - Retry debugging techniques (tracking the number of attempts, logging, etc.)

## Performance Impact

### Scheduler overhead

This is an example of how to avoid excessive use of the scheduler and optimize for batch processing.

```ts
import { range, asyncScheduler, pipe } from 'rxjs';
import { bufferCount, map, observeOn, tap } from 'rxjs';

// ❌ Excessive scheduler use
range(1, 1000)
  .pipe(
    observeOn(asyncScheduler),  // 1000 setTimeouts
    map(x => x * 2),
    // tap(console.log)
  )
  .subscribe();

// ✅ Optimize with batch processing
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeouts
    map(batch => batch.map(x => x * 2)),
    // tap(console.log)
  )
  .subscribe();
```

## Summary

The choice of scheduler has a significant impact on application performance and responsiveness. Understanding the characteristics of each scheduler and using them in appropriate situations will ensure efficient and smooth operation. As a general guideline,

- For general asynchronous processing, use `asyncScheduler`
- `queueScheduler` for recursive processing and synchronous queuing
- `asapScheduler` for fast response times
- `animationFrameScheduler` for animation

are recommended.
