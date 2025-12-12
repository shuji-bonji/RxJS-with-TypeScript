---
description: "interval() - Creation Function that continuously emits values at specified intervals: Essential for polling, periodic tasks, countdown timers, and real-time updates"
titleTemplate: ':title | RxJS'
---

# interval() - Continuous Emission at Specified Intervals

`interval()` is a Creation Function that continuously emits values at specified time intervals.

## Overview

`interval()` continuously emits consecutive numbers starting from 0 at specified millisecond intervals. It is frequently used for polling processes and periodic task execution.

**Signature**:
```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

**Official Documentation**: [📘 RxJS Official: interval()](https://rxjs.dev/api/index/function/interval)

## Basic Usage

`interval()` emits numbers that count up at a specified interval.

```typescript
import { interval } from 'rxjs';

// Emit values every 1 second
const interval$ = interval(1000);

interval$.subscribe(value => {
  console.log('Value:', value);
});

// Output (every 1 second):
// Value: 0
// Value: 1
// Value: 2
// Value: 3
// ... (continues infinitely)
```

## Important Characteristics

### 1. Consecutive Numbers Starting from 0

`interval()` always emits integers that start at 0 and increment by 1.

```typescript
import { interval } from 'rxjs';
import { take } from 'rxjs';

interval(500).pipe(
  take(5) // Get only the first 5 values
).subscribe(value => console.log(value));

// Output (every 500ms):
// 0
// 1
// 2
// 3
// 4
```

### 2. Never Completes (Infinite Stream)

`interval()` does not complete automatically and **must be unsubscribed**.

```typescript
import { interval } from 'rxjs';

const subscription = interval(1000).subscribe(value => {
  console.log('Value:', value);
});

// Unsubscribe after 5 seconds
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Stopped');
}, 5000);
```

> [!WARNING]
> **Forgetting to Unsubscribe Causes Memory Leaks**
>
> Because `interval()` continues to emit values indefinitely, forgetting to unsubscribe can cause memory leaks and performance problems. Be sure to call `unsubscribe()` or use operators such as `take()`, `takeUntil()`, or `takeWhile()` to complete automatically.

### 3. Cold Observable

`interval()` is a Cold Observable, which creates an independent timer for each subscription.

```typescript
import { interval } from 'rxjs';

const interval$ = interval(1000);

// Subscription 1
interval$.subscribe(value => console.log('Observer 1:', value));

// Add subscription 2 after 2 seconds
setTimeout(() => {
  interval$.subscribe(value => console.log('Observer 2:', value));
}, 2000);

// Output:
// Observer 1: 0
// Observer 1: 1
// Observer 2: 0  ← Starts from 0 with independent timer
// Observer 1: 2
// Observer 2: 1
```

> [!NOTE]
> **Cold Observable Characteristics**:
> - Independent execution is initiated for each subscription
> - Each subscriber receives its own data stream
> - An independent timer is started for each subscription; use `share()` if you need to share data
>
> See [Cold Observable and Hot Observable](/pt/guide/observables/cold-and-hot-observables) for more information.

## Difference Between interval() and timer()

Although `interval()` and `timer()` are similar, there are some important differences.

```typescript
import { interval, timer } from 'rxjs';
import { take } from 'rxjs';

// interval() - starts immediately, continuous emission
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - starts after delay
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// Output:
// interval: 0  (after 1 second)
// interval: 1  (after 2 seconds)
// timer: 0     (after 2 seconds)
// interval: 2  (after 3 seconds)
// timer: 1     (after 3 seconds)
// timer: 2     (after 4 seconds)
```

| Creation Function | Start Timing | Purpose |
|-------------------|--------------|---------|
| `interval(1000)` | Starts immediately (first value after 1 second) | Periodic execution |
| `timer(2000, 1000)` | Starts after specified time | Periodic execution with delay |
| `timer(2000)` | Emits only once after specified time | Delayed execution |

## Practical Use Cases

### 1. API Polling

Call API at regular intervals to update data.

```typescript
import { from, interval } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

function fetchStatus(): Promise<Status> {
  return fetch('https://jsonplaceholder.typicode.com/users/1')
    .then(res => res.json());
}

// Poll API every 5 seconds
const polling$ = interval(5000).pipe(
  switchMap(() => from(fetchStatus())),
  catchError(error => {
    console.error('API Error:', error);
    return of({ status: 'error', timestamp: Date.now() });
  })
);

const subscription = polling$.subscribe(data => {
  console.log('Status update:', data);
});

// Stop as needed
// subscription.unsubscribe();
```

### 2. Countdown Timer

Implement a countdown for the time limit.

```typescript
import { interval } from 'rxjs';
import { map, takeWhile } from 'rxjs';

const countdown$ = interval(1000).pipe(
  map(count => 10 - count), // Countdown from 10 seconds
  takeWhile(time => time >= 0) // Auto-complete at 0
);

countdown$.subscribe({
  next: time => console.log(`Time remaining: ${time} seconds`),
  complete: () => console.log('Time up!')
});

// Output (every 1 second):
// Time remaining: 10 seconds
// Time remaining: 9 seconds
// ...
// Time remaining: 0 seconds
// Time up!
```

### 3. Auto-save Function

Auto-save form contents periodically.

```typescript
import { fromEvent, from } from 'rxjs';
import { switchMap, debounceTime } from 'rxjs';

// Create form
const form = document.createElement('form');
form.id = 'myForm';
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Enter text';
form.appendChild(input);
document.body.appendChild(form);

const input$ = fromEvent(form, 'input');

// Auto-save 3 seconds after input stops (shortened for demo)
input$.pipe(
  debounceTime(3000), // If there is no input for 3 seconds
  switchMap(() => {
    const formData = new FormData(form);
    // Demo: Simulate with Promise instead of actual API
    return from(
      Promise.resolve({ success: true, data: formData.get('text') })
    );
  })
).subscribe(result => {
  console.log('Auto-saved:', result);
});
```

### 4. Real-time Clock Display

Update current time in real time.

```typescript
import { interval } from 'rxjs';
import { map } from 'rxjs';

// Create element for clock display
const clockElement = document.createElement('div');
clockElement.id = 'clock';
clockElement.style.fontSize = '24px';
clockElement.style.fontFamily = 'monospace';
clockElement.style.padding = '20px';
document.body.appendChild(clockElement);

const clock$ = interval(1000).pipe(
  map(() => new Date().toLocaleTimeString())
);

clock$.subscribe(time => {
  clockElement.textContent = time;
});

// Output: Current time updates every second
```

## Using in Pipeline

`interval()` is used as a starting point for pipelines or as a time control trigger.

```typescript
import { interval } from 'rxjs';
import { map, filter, scan } from 'rxjs';

// Count only even seconds
interval(1000).pipe(
  filter(count => count % 2 === 0),
  scan((sum, count) => sum + count, 0),
  map(sum => `Sum of evens: ${sum}`)
).subscribe(console.log);

// Output (every 1 second):
// Sum of evens: 0
// Sum of evens: 2  (0 + 2)
// Sum of evens: 6  (0 + 2 + 4)
// Sum of evens: 12 (0 + 2 + 4 + 6)
```

## Common Mistakes

### 1. Forgetting to Unsubscribe

```typescript
// ❌ Wrong - runs infinitely without unsubscribe
import { interval } from 'rxjs';

function startPolling() {
  interval(1000).subscribe(value => {
    console.log('Value:', value); // Runs forever
  });
}

startPolling();

// ✅ Correct - hold subscription and unsubscribe as needed
import { interval, Subscription } from 'rxjs';

let subscription: Subscription | null = null;

function startPolling() {
  subscription = interval(1000).subscribe(value => {
    console.log('Value:', value);
  });
}

function stopPolling() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startPolling();
// Call stopPolling() as needed
```

### 2. Multiple Subscriptions Create Independent Timers

```typescript
// ❌ Unintended - two independent timers are created
import { interval } from 'rxjs';

const interval$ = interval(1000);

interval$.subscribe(value => console.log('Observer 1:', value));
interval$.subscribe(value => console.log('Observer 2:', value));
// Two timers run in parallel

// ✅ Correct - share one timer
import { interval } from 'rxjs';
import { share } from 'rxjs';

const interval$ = interval(1000).pipe(share());

interval$.subscribe(value => console.log('Observer 1:', value));
interval$.subscribe(value => console.log('Observer 2:', value));
// One timer is shared
```

## Performance Considerations

Although `interval()` is lightweight, performance should be considered when executing at short intervals.

> [!TIP]
> **Optimization Tips**:
> - Do not perform unnecessary processing (refine with `filter()`)
> - Use short intervals (less than 100ms) with caution
> - Ensure subscriptions are unsubscribed
> - If multiple Observers are needed, share them with `share()`

```typescript
import { interval } from 'rxjs';
import { filter, share } from 'rxjs';

// ❌ Performance problem - heavy processing every 100ms
interval(100).subscribe(() => {
  // Heavy processing
  heavyCalculation();
});

// ✅ Optimization - process only when necessary
interval(100).pipe(
  filter(count => count % 10 === 0), // Once per second (once every 10 times)
  share() // Share among multiple Observers
).subscribe(() => {
  heavyCalculation();
});
```

## Related Creation Functions

| Function | Difference | Usage |
|----------|------|----------|
| **[timer()](/pt/guide/creation-functions/basic/timer)** | Starts after delay, or emits only once | Delayed execution or one-time processing |
| **[fromEvent()](/pt/guide/creation-functions/basic/fromEvent)** | Event-driven | Processing according to user operations |
| **range()** | Emits numbers in specified range immediately | When time control is not needed |

## Summary

- `interval()` continuously emits values at specified intervals
- Emits consecutive integers starting from 0
- Does not auto-complete, must be unsubscribed
- Works as a Cold Observable (independent timer for each subscription)
- Ideal for polling, periodic execution, countdown, etc.

## Next Steps

- [timer() - Start Emitting After Delay](/pt/guide/creation-functions/basic/timer)
- [fromEvent() - Convert Events to Observable](/pt/guide/creation-functions/basic/fromEvent)
- [Return to Basic Creation Functions](/pt/guide/creation-functions/basic/)
