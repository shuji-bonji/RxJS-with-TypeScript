---
description: "timer() - Creation Function that starts emitting after a specified delay: Perfect for delayed execution, polling with delay, and timeout implementations"
---

# timer() - Start Emitting After Delay

`timer()` is a Creation Function that starts emitting values after a specified delay time, supporting both one-time and periodic emission.

## Overview

`timer()` is a flexible Creation Function that allows you to control the timing of the first emission. Its behavior changes depending on the number of arguments, and it can be used for either one-time emission or periodic emission like `interval()`.

**Signature**:
```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

**Official Documentation**: [📘 RxJS Official: timer()](https://rxjs.dev/api/index/function/timer)

## Basic Usage

The behavior of `timer()` depends on the number of arguments.

### One-time Emission

If only the first argument is specified, it emits 0 after the specified time and completes.

```typescript
import { timer } from 'rxjs';

// Emit 0 after 3 seconds and complete
const timer$ = timer(3000);

timer$.subscribe({
  next: value => console.log('Value:', value),
  complete: () => console.log('Complete')
});

// Output after 3 seconds:
// Value: 0
// Complete
```

### Periodic Emission

If an interval is specified for the second argument, it will continue to emit periodically after the initial delay.

```typescript
import { timer } from 'rxjs';

// Start after 3 seconds, then emit values every 1 second
const timer$ = timer(3000, 1000);

timer$.subscribe(value => console.log('Value:', value));

// Output:
// Value: 0  (after 3 seconds)
// Value: 1  (after 4 seconds)
// Value: 2  (after 5 seconds)
// ... (continues infinitely)
```

## Important Characteristics

### 1. Flexible Specification of Delays

The delay can be specified as a number in milliseconds or as a `Date` object.

```typescript
import { timer } from 'rxjs';

// Specify in milliseconds
timer(5000).subscribe(() => console.log('After 5 seconds'));

// Specify with Date object (execute at specific time)
const targetTime = new Date(Date.now() + 10000); // 10 seconds later
timer(targetTime).subscribe(() => console.log('Execute at specified time'));
```

### 2. Behavior Changes Depending on Second Argument

Whether or not the second argument is specified determines whether it completes.

```typescript
import { timer } from 'rxjs';

// Without second argument - emit once and complete
timer(1000).subscribe({
  next: value => console.log('Once:', value),
  complete: () => console.log('Complete')
});

// With second argument - emit infinitely
timer(1000, 1000).subscribe({
  next: value => console.log('Repeat:', value),
  complete: () => console.log('Complete (not displayed)')
});
```

> [!IMPORTANT]
> **With Second Argument, It Does Not Complete**
>
> If you specify the second argument like `timer(1000, 1000)`, it will keep emitting indefinitely, just like `interval()`. Unsubscription is always required.

### 3. Cold Observable

`timer()` is a Cold Observable, which means that an independent timer is created for each subscription.

```typescript
import { timer } from 'rxjs';

const timer$ = timer(1000);

console.log('Start');

// Subscription 1
timer$.subscribe(() => console.log('Observer 1'));

// Add subscription 2 after 500ms
setTimeout(() => {
  timer$.subscribe(() => console.log('Observer 2'));
}, 500);

// Output:
// Start
// Observer 1  (after 1 second)
// Observer 2  (after 1.5 seconds - independent timer)
```

> [!NOTE]
> **Cold Observable Characteristics**:
> - Independent execution is initiated for each subscription
> - Each subscriber receives its own data stream
> - An independent timer is started for each subscription; as with `interval()`, use `share()` if sharing is required
>
> See [Cold Observable and Hot Observable](/en/guide/observables/cold-and-hot-observables) for more information.

## Difference Between timer() and interval()

The main difference between the two is the timing of the first emission.

```typescript
import { timer, interval } from 'rxjs';
import { take } from 'rxjs';

console.log('Start');

// interval() - starts immediately (first value after 1 second)
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - no delay (first value immediately)
timer(0, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// timer() - starts after 2 second delay
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer(delay):', value);
});
```

| Creation Function | First Emission Timing | Purpose |
|-------------------|----------------------|---------|
| `interval(1000)` | After 1 second | Start periodic execution immediately |
| `timer(0, 1000)` | Immediately | Want first execution immediately |
| `timer(2000, 1000)` | After 2 seconds | Periodic execution after delay |
| `timer(2000)` | After 2 seconds (once only) | Delayed execution (one-time) |

## Practical Use Cases

### 1. Delayed Execution

Execute a process only once after a certain period of time.

```typescript
import { from, timer } from 'rxjs';
import { switchMap } from 'rxjs';

function delayedApiCall() {
  return timer(2000).pipe(
    switchMap(() => from(
      fetch('https://jsonplaceholder.typicode.com/posts/1')
        .then(res => res.json())
    ))
  );
}

delayedApiCall().subscribe(data => {
  console.log('Fetch data after 2 seconds:', data);
});
```

### 2. Polling with Delay

Start polling after a certain period of time instead of executing immediately the first time.

```typescript
import { from, timer } from 'rxjs';
import { switchMap, retry } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

// Start polling after 5 seconds, then every 10 seconds
const polling$ = timer(5000, 10000).pipe(
  switchMap(() => from(
    fetch('https://jsonplaceholder.typicode.com/users/1')
      .then(res => res.json() as Promise<Status>)
  )),
  retry(3) // Retry up to 3 times on error
);

const subscription = polling$.subscribe(data => {
  console.log('Status update:', data);
});

// Stop as needed
// subscription.unsubscribe();
```

### 3. Timeout Processing

Timeout occurs when processing is not completed within a certain period of time.

```typescript
import { timer, race, from } from 'rxjs';
import { map } from 'rxjs';

function fetchWithTimeout(url: string, timeoutMs: number) {
  const request$ = from(fetch(url).then(res => res.json()));
  const timeout$ = timer(timeoutMs).pipe(
    map(() => {
      throw new Error('Timeout');
    })
  );

  // Use whichever comes first
  return race(request$, timeout$);
}

fetchWithTimeout('https://jsonplaceholder.typicode.com/posts/1', 3000).subscribe({
  next: data => console.log('Fetch data:', data),
  error: err => console.error('Error:', err.message)
});
```

### 4. Auto-hiding Notifications

Automatically hide notifications after a certain period of time after they are displayed.

```typescript
import { timer, Subject, map } from 'rxjs';
import { switchMap, takeUntil } from 'rxjs';

interface Notification {
  id: number;
  message: string;
}

const notifications$ = new Subject<Notification>();
const dismiss$ = new Subject<number>();

notifications$.pipe(
  switchMap(notification => {
    console.log('Show notification:', notification.message);

    // Auto-hide after 5 seconds
    return timer(5000).pipe(
      takeUntil(dismiss$), // Cancel if manually dismissed
      map(() => notification.id)
    );
  })
).subscribe(id => {
  console.log('Hide notification:', id);
});

// Show notification
notifications$.next({ id: 1, message: 'New message received' });

// To manually dismiss
// dismiss$.next(1);
```

## Using in Pipeline

`timer()` is used as a starting point for delayed processing or periodic execution.

```typescript
import { timer } from 'rxjs';
import { map, take, scan } from 'rxjs';

// Countdown timer (from 10 seconds to 0 seconds)
timer(0, 1000).pipe(
  map(count => 10 - count),
  take(11), // From 0 to 10 (11 values)
  scan((acc, curr) => curr, 0)
).subscribe({
  next: time => console.log(`Remaining: ${time} seconds`),
  complete: () => console.log('Timer finished')
});

// Output:
// Remaining: 10 seconds
// Remaining: 9 seconds
// ...
// Remaining: 0 seconds
// Timer finished
```

## Common Mistakes

### 1. Forgetting to Unsubscribe with Second Argument

```typescript
// ❌ Wrong - runs infinitely with second argument
import { timer } from 'rxjs';

function startTimer() {
  timer(1000, 1000).subscribe(value => {
    console.log('Value:', value); // Runs forever
  });
}

startTimer();

// ✅ Correct - hold subscription and unsubscribe as needed
import { timer, Subscription } from 'rxjs';
import { take } from 'rxjs';

let subscription: Subscription | null = null;

function startTimer() {
  subscription = timer(1000, 1000).pipe(
    take(10) // Auto-complete after 10 times
  ).subscribe(value => {
    console.log('Value:', value);
  });
}

function stopTimer() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startTimer();
```

### 2. Not Understanding Difference from interval()

```typescript
// ❌ Confusion - interval() starts immediately (first value after 1 second)
import { interval } from 'rxjs';

interval(1000).subscribe(value => {
  console.log('interval:', value); // 0 emitted after 1 second
});

// ✅ timer() - when you want to emit first value immediately without delay
import { timer } from 'rxjs';

timer(0, 1000).subscribe(value => {
  console.log('timer:', value); // 0 emitted immediately
});
```

## Performance Considerations

Although `timer()` is lightweight, its usage can affect performance.

> [!TIP]
> **Optimization Tips**:
> - Do not specify second argument for one-time execution
> - Always unsubscribe when no longer needed
> - If multiple Observers are needed, share them with `share()`
> - Use short intervals (less than 100ms) with caution

```typescript
import { timer } from 'rxjs';
import { share } from 'rxjs';

// ❌ Performance problem - multiple independent timers
const timer$ = timer(0, 1000);

timer$.subscribe(value => console.log('Observer 1:', value));
timer$.subscribe(value => console.log('Observer 2:', value));
// Two timers run in parallel

// ✅ Optimization - share one timer
const sharedTimer$ = timer(0, 1000).pipe(share());

sharedTimer$.subscribe(value => console.log('Observer 1:', value));
sharedTimer$.subscribe(value => console.log('Observer 2:', value));
// One timer is shared
```

## Related Creation Functions

| Function | Difference | Usage |
|----------|------|----------|
| **[interval()](/en/guide/creation-functions/basic/interval)** | Starts immediately (no delay) | Periodic execution without delay |
| **[of()](/en/guide/creation-functions/basic/of)** | Emit synchronously and immediately | When asynchronous is not needed |
| **defer()** | Defer processing until subscription | Dynamic value generation |

## Summary

- `timer()` is a Creation Function that starts emitting after a delay
- Without second argument: one-time emission (completes)
- With second argument: periodic emission (does not complete)
- Delay time can be specified in milliseconds or as a `Date` object
- Ideal for delayed execution, polling with delay, timeout processing

## Next Steps

- [interval() - Continuous Emission at Specified Intervals](/en/guide/creation-functions/basic/interval)
- [defer() - Defer Generation Until Subscription](/en/guide/creation-functions/conditional/defer)
- [Return to Basic Creation Functions](/en/guide/creation-functions/basic/)
