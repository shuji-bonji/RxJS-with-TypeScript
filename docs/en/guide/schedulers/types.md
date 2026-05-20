---
description: "The features, implementation, and uses of the major schedulers in RxJS, including asyncScheduler and queueScheduler, will be explained in detail. You will understand the difference between macro tasks, micro tasks, and synchronous processing, and learn the execution timing and characteristics of each scheduler. You can optimize the performance and behavior of your application by using the appropriate one."
---

# Types of schedulers and how to use them

RxJS provides multiple schedulers for different applications. Each scheduler has its own execution timing and characteristics and can be used appropriately to optimize application performance and behavior.

## Classification of schedulers

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
    console.log(`Calculation results: ${result}`);
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

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs';

// Optimize recursive processing
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

of('ASAPProcessing')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: End');

// Output:
// 1: Start
// 2: End
// 3: ASAPProcessing
```

#### Use Cases

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Optimization of mouse movement events
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // UIUpdate processing of
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Features
- **Internal implementation**: requestAnimationFrame
- **Execution timing**: before next screen rendering
- **Use**: Animation, drawing process for 60fps

#### Example of a simple rotation animation

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// HTMLCreating elements
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Setting up animations
let rotation = 0;

// 60fpsIn2Seconds of animation
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2Seconds = 120Frames
    map(() => {
      rotation += 3;  // 1Every frame3Degrees of rotation
      return rotation;
    })
  )
  .subscribe(angle => {
    // DOMActual rotation of element
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Why animationFrameScheduler is needed

The `animationFrameScheduler` performs synchronously with the browser's drawing cycle, which offers the following advantages

1. **Smooth Animation**: Because processing is performed in sync with the browser's rendering timing (typically 60 fps), you can achieve smooth animations with no choppiness. 2.
2. **Efficient resource use**: When the browser deactivates the tab, the execution of requestAnimationFrame is automatically paused to avoid unnecessary CPU usage. 3.
3. **Anti-flickering**: Ensures computation is completed before the screen is drawn, preventing screen flickering and displaying incomplete frames.

The following is a comparison of `setInterval` and `animationFrameScheduler`.

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ setIntervalInefficient animation using
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // Approx.60fps

// Problems:
// - Out of sync with browser rendering timing
// - Continues to run even in background tabs
// - Cannot guarantee accurate60fpscannot be guaranteed

// ✅ animationFrameSchedulerEfficient animation using
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

// Advantages
// - Synchronized with browser rendering timing
// - Automatically paused in background tabs
// - Stable60fpsStable animations
```

#### Example of mouse-following animation

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Create circles that follow
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Transparent to mouse events
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
    
    // Gradual movement (easing) from current position to target position々Gradual movement from current position to target position (easing)
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;
    
    // DOMUpdate element
    circle.style.left = `${currentX - 15}px`;  // Adjust to center position
    circle.style.top = `${currentY - 15}px`;
  });
```

## Guide to using the scheduler

### Comparison by execution timing

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

### Selection Criteria by Application


## Practical use examples

### Processing large amounts of data

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
    tap((req) => console.log(`Add to queue: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // Uses actualAPIRequest simulation
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`${req.endpoint}/${req.id} ofresult`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`completed: ${result}`));
```

#### Scheduler utilization with exponential backoff

For more advanced control, exponential backoff can be implemented by combining `retryWhen` and `asyncScheduler`.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

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
    // RxJS 7.3+ Recommended: retry({ count, delay }) Format
    retry({
      count: 3, // Max.3Retry up to 3 times
      delay: (error, retryCount) => {
        // Exponential backoff: 1seconds, 2seconds, 4seconds...
        const delayTime = Math.pow(2, retryCount - 1) * 1000;
        console.log(`🔄 Retries ${retryCount}times (${delayTime}msAfter)`);

        // timer is internally asyncScheduler Use of
        return timer(delayTime);
      }
    })
  )
  .subscribe({
    next: result => console.log('✅ Success:', result),
    error: error => {
      console.log('❌ Max.Number of retries reached');
      console.log('❌ FinalError:', error.message);
    }
  });

// OutputEx.:
// 🔄 Retries 1times (1000msAfter)
// 🔄 Retries 2times (2000msAfter)
// 🔄 Retries 3times (4000msAfter)
// ❌ Max.Number of retries reached
// ❌ FinalError: Temporary error
```

#### When explicitly specifying an asyncScheduler

Explicitly specifying a specific scheduler allows for more flexible control, such as replacing it with `TestScheduler` during testing.


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


> Detailed implementation patterns and debugging methods for retry processing are described in the [retry and catchError](/en/guide/error-handling/retry-catch) page.
> - Detailed usage of the retry operator
> - Combination patterns with catchError
> - Debugging techniques for retries (tracking the number of attempts, logging, etc.)

## Performance Impact

### Scheduler overhead


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

## Summary

The choice of scheduler has a significant impact on the performance and responsiveness of an application. Understanding the characteristics of each scheduler and using them in appropriate situations will ensure efficient and smooth operation. As a general guideline,

- For general asynchronous processing, use `asyncScheduler`.
- queueScheduler` for recursive processing and synchronous queuing
- `asapScheduler` for fast response times
- animationFrameScheduler` for animation

for animation.