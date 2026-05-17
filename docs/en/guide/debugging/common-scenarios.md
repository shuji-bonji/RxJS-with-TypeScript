---
description: This page introduces six common debugging scenarios for RxJS, including values not flowing, different values than expected, subscription not completing, memory leaks, missing errors, and retry tracking, along with practical problems and solutions.
---

# Common Debugging Scenarios

Typical problems encountered in RxJS development and their solutions are described with concrete code examples.

## Scenario 1: Values do not flow

- **Symptom**: I `subscribe` and not a single value is output.


### Cause 1: You forgot to subscribe to Cold Observable.

Cold Observable will not run until it is subscribed to.

```ts
import { interval } from 'rxjs';
import { map } from 'rxjs';

// ❌ Nothing runs because it's not subscribed to
const numbers$ = interval(1000).pipe(
  map(x => {
    console.log('This line is not executed');
    return x * 2;
  })
);

// ✅ Executed by subscribing
numbers$.subscribe(value => console.log('Value:', value));
```

### Cause 2: Completed Subject

Once a Subject is completed, it will not receive values in subsequent subscriptions.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.complete(); // Complete

// ❌ Subscription after completion does not receive value
subject.subscribe(value => console.log('This line is not executed'));

// ✅ Subscribe before completion
const subject2 = new Subject<number>();
subject2.subscribe(value => console.log('Value:', value));
subject2.next(1); // Value: 1
subject2.complete();
```

### Cause 3: Filtering on wrong conditions

Filtering conditions may be too strict and exclude all values.

```ts
import { of } from 'rxjs';
import { filter, tap } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('Before filter:', value)),
    filter(x => x > 10), // All excluded
    tap(value => console.log('After filter:', value)) // This line is not executed
  )
  .subscribe({
    next: value => console.log('Final value:', value),
    complete: () => console.log('Complete (no value)')
  });

// Output:
// Before filter: 1
// Before filter: 2
// Before filter: 3
// Before filter: 4
// Before filter: 5
// Complete (no value)
```

### Debugging Techniques

Use the `tap` operator to see which values are flowing at each step.

```ts
import { of, EMPTY } from 'rxjs';
import { filter, tap, defaultIfEmpty } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('🔵 Input:', value)),
    filter(x => x > 10),
    tap(value => console.log('🟢 Passed filter:', value)),
    defaultIfEmpty('No value') // Default if there is no value
  )
  .subscribe(value => console.log('✅ Output:', value));

// Output:
// 🔵 Input: 1
// 🔵 Input: 2
// 🔵 Input: 3
// 🔵 Input: 4
// 🔵 Input: 5
// ✅ Output: No value
```

## Scenario 2: Different value is output than expected

- **Symptom**: Different value than expected is output.

### Cause 1: Operator is in the wrong order.

The result depends on the order in which the operators are applied.

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// ❌ Different result than expected
of(1, 2, 3, 4, 5)
  .pipe(
    map(x => x * 2),     // 2, 4, 6, 8, 10
    filter(x => x < 5)   // Only 2, 4 pass through
  )
  .subscribe(value => console.log('Result:', value));
// Output: 2, 4

// ✅ Correct order
of(1, 2, 3, 4, 5)
  .pipe(
    filter(x => x < 5),  // Only 1, 2, 3, 4 pass through
    map(x => x * 2)      // 2, 4, 6, 8
  )
  .subscribe(value => console.log('Result:', value));
// Output: 2, 4, 6, 8
```

### Cause 2: Unintended changes due to shared references

Because JavaScript objects are passed by reference, it is possible to modify the original object.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const user: User = { id: 1, name: 'Alice' };

of(user)
  .pipe(
    // ❌ Modifies the original object directly
    map(u => {
      u.name = 'Bob'; // Original object is modified
      return u;
    })
  )
  .subscribe(value => console.log('After change:', value));

console.log('Original object:', user); // { id: 1, name: 'Bob' }

// ✅ Create a new object
of(user)
  .pipe(
    map(u => ({ ...u, name: 'Charlie' })) // New object with spread syntax
  )
  .subscribe(value => console.log('After change:', value));

console.log('Original object:', user); // { id: 1, name: 'Alice' } (not modified)
```

### Cause 3: Timing of asynchronous processing

The order of completion of asynchronous processing may be different than expected.

```ts
import { of, delay } from 'rxjs';
import { mergeMap, tap } from 'rxjs';

// ❌ Does not wait for asynchronous processing to complete
of(1, 2, 3)
  .pipe(
    tap(value => console.log('Start:', value)),
    mergeMap(value =>
      of(value * 2).pipe(
        delay(100 - value * 10) // Larger values complete faster
      )
    )
  )
  .subscribe(value => console.log('Complete:', value));

// Output:
// Start: 1
// Start: 2
// Start: 3
// Complete: 3  ← Shortest delay
// Complete: 2
// Complete: 1  ← Longest delay

// ✅ Guarantee order
import { concatMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(value => console.log('Start:', value)),
    concatMap(value =>  // mergeMap → concatMap
      of(value * 2).pipe(delay(100 - value * 10))
    )
  )
  .subscribe(value => console.log('Complete:', value));

// Output:
// Start: 1
// Complete: 1
// Start: 2
// Complete: 2
// Start: 3
// Complete: 3
```

## Scenario 3: Subscription not completed (infinite stream)

- **Symptom**: `complete` is not called and the stream is not terminated

You need to explicitly complete it, since `interval`, `fromEvent`, etc. keep issuing values indefinitely.

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

// ❌ interval continues to issue values indefinitely
interval(1000)
  .pipe(
    tap(value => console.log('Value:', value))
  )
  .subscribe({
    complete: () => console.log('This line is not executed')
  });

// ✅ Explicitly complete with take
import { take } from 'rxjs';

interval(1000)
  .pipe(
    take(5), // Complete after 5 values
    tap(value => console.log('Value:', value))
  )
  .subscribe({
    complete: () => console.log('Complete')
  });
```

### Debugging Techniques

Set a timeout to stop the infinite stream when debugging.

```ts
import { interval, timer } from 'rxjs';
import { tap, takeUntil } from 'rxjs';

// Set timeout for debugging
const stop$ = timer(5000); // Complete after 5 seconds

interval(1000)
  .pipe(
    tap({
      next: value => console.log('Value:', value),
      complete: () => console.log('Stopped on timeout')
    }),
    takeUntil(stop$)
  )
  .subscribe();
```

## Scenario 4: Memory leak (forgot to unsubscribe)

- **Symptom**: Application gradually becomes slower and slower

### Cause: Unsubscribed subscriptions that are no longer needed

A memory leak occurs when a subscription remains after a component or service is destroyed.

```ts
import { interval } from 'rxjs';

class UserComponent {
  private subscription: any;

  ngOnInit() {
    // ❌ Forgot to unsubscribe
    interval(1000).subscribe(value => {
      console.log('Value:', value); // Continues to execute after component is destroyed
    });
  }

  ngOnDestroy() {
    // No unsubscription
  }
}

// ✅ Manage subscriptions properly
class UserComponentFixed {
  private subscription: any;

  ngOnInit() {
    this.subscription = interval(1000).subscribe(value => {
      console.log('Value:', value);
    });
  }

  ngOnDestroy() {
    // Unsubscribe when component is destroyed
    if (this.subscription) {
      this.subscription.unsubscribe();
    }
  }
}
```

**Recommended pattern: use `takeUntil`**.

The `takeUntil` pattern can be used to automate unsubscriptions.

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class UserComponentBest {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    // ✅ Automatically unsubscribe with takeUntil
    interval(1000)
      .pipe(
        takeUntil(this.destroy$)
      )
      .subscribe(value => console.log('Value:', value));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### Memory leak detection

Track the number of subscriptions with a custom operator.

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

let subscriptionCount = 0;

const trackSubscriptions = <T>() =>
  tap<T>({
    subscribe: () => {
      subscriptionCount++;
      console.log('📈 Subscriptions:', subscriptionCount);
    },
    unsubscribe: () => {
      subscriptionCount--;
      console.log('📉 Subscriptions:', subscriptionCount);
    }
  });

// Usage example
const stream$ = interval(1000).pipe(
  trackSubscriptions()
);

const sub1 = stream$.subscribe();
// Output: 📈 Subscriptions: 1

const sub2 = stream$.subscribe();
// Output: 📈 Subscriptions: 2

setTimeout(() => {
  sub1.unsubscribe();
  // Output: 📉 Subscriptions: 1
}, 3000);
```

## Scenario 5: You don't notice an error

- **Symptom**: Error occurs, but is not displayed and is ignored

Without an error handler, the error may be held in a grip and unnoticed.

```ts
import { of, throwError } from 'rxjs';
import { mergeMap, catchError } from 'rxjs';

// ❌ Error is suppressed because there is no error handling
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Error'));
      }
      return of(value);
    })
  )
  .subscribe(); // No error handler

// ✅ Proper error handling
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Error'));
      }
      return of(value);
    }),
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('🔴 Caught error:', message);
      return of(-1); // Fallback value
    })
  )
  .subscribe({
    next: value => console.log('Value:', value),
    error: error => console.error('🔴 Error on subscribe:', error)
  });

// Output:
// Value: 1
// 🔴 Caught error: Error
// Value: -1
```

### Configure global error handler

A global handler can be configured to catch all outstanding errors.

```ts
import { Observable } from 'rxjs';

// Catch all unhandled errors
const originalCreate = Observable.create;

Observable.create = function(subscribe: any) {
  return originalCreate.call(this, (observer: any) => {
    try {
      return subscribe(observer);
    } catch (error) {
      console.error('🔴 Unhandled error:', error);
      observer.error(error);
    }
  });
};
```

## Scenario 6: I want to track retry attempts

- **Symptom**: I'm using the `retry` operator, but I don't know how many retries I'm getting.

When retrying automatically when an error occurs, tracking how many retries are actually performed would facilitate debugging and logging.

### Basic Retry Debugging

```ts
import { throwError, of, timer } from 'rxjs';
import { retry } from 'rxjs';

throwError(() => new Error('Temporary error'))
  .pipe(
    // RxJS 7.3+ recommended: retry({ count, delay }) form
    retry({
      count: 2, // Retry up to 2 times
      delay: (error, retryCount) => {
        console.log(`🔄 Retry attempt ${retryCount}`);
        return timer(1000);
      }
    })
  )
  .subscribe({
    next: value => console.log('✅ Success:', value),
    error: error => {
      console.log('❌ Maximum retry count reached');
      console.log('🔴 Final error:', error.message);
    }
  });

// Output:
// 🔄 Retry attempt 1
// 🔄 Retry attempt 2
// ❌ Maximum retry count reached
// 🔴 Final error: Temporary error
```

> [!TIP]
> For more detailed implementation patterns on debugging retries, see the "Debugging Retries" section of [retry and catchError](/en/guide/error-handling/retry-catch#debugging-retries).
> - Basic tracking using the tap error callback
> - Detailed logging with retry config
> - Exponential backoff and logging
> - RxJS 7.4+ retry configuration object

## Summary

Solutions to common debugging scenarios

- ✅ **values do not flow** → forgot to subscribe, check filtering conditions
- ✅ **Value different than expected** → beware of operator order, reference sharing
- ✅ **Subscription not completed** → use `take` or `takeUntil` for infinite streams
- ✅ **Memory leak** → auto unsubscribe with `takeUntil` pattern
- ✅ **Missing errors** → implement proper error handling
- ✅ **retry tracking** → logging with `retry` configuration object

## Related Pages

- [Basic Debugging Strategies](/en/guide/debugging/) - How to use tap operator and developer tools
- [Custom Debug Tools](/en/guide/debugging/custom-tools) - Named streams, debug operators
- [Performance Debugging](/en/guide/debugging/performance) - Subscription monitoring, memory usage checking
- [Error Handling](/en/guide/error-handling/strategies) - Error handling strategies
