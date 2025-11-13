---
description: This page explains performance debugging techniques for RxJS applications. It provides practical techniques such as tracking the number of subscriptions, detecting unnecessary reevaluations, monitoring memory usage, setting up the development environment, type-safe debugging, and setting error boundaries.
---

# Performance Debugging and Best Practices

This session will cover techniques for optimizing the performance of RxJS applications and creating an efficient debugging environment.

## Check Subscription Count

Check to see if multiple subscriptions have been unintentionally created.

```ts
import { Observable, defer } from 'rxjs';
import { finalize } from 'rxjs';

let globalSubscriptionId = 0;
let activeSubscriptions = 0;

/**
 * Custom operator to track subscription count
 */
function tracked<T>(label: string) {
  return (source: Observable<T>) =>
    defer(() => {
      const id = ++globalSubscriptionId;
      activeSubscriptions++;
      console.log(`➕ Subscription started [${label}] #${id} (Active: ${activeSubscriptions})`);

      return source.pipe(
        finalize(() => {
          activeSubscriptions--;
          console.log(`➖ Subscription ended [${label}] #${id} (Active: ${activeSubscriptions})`);
        })
      );
    });
}

// Usage example
import { interval } from 'rxjs';
import { take } from 'rxjs';

const stream$ = interval(1000).pipe(
  take(3),
  tracked('Test Stream')
);

const sub1 = stream$.subscribe();
const sub2 = stream$.subscribe();

setTimeout(() => {
  sub1.unsubscribe();
  sub2.unsubscribe();
}, 5000);

// Output:
// ➕ Subscription started [Test Stream] #1 (Active: 1)
// ➕ Subscription started [Test Stream] #2 (Active: 2)
// ➖ Subscription ended [Test Stream] #1 (Active: 1)
// ➖ Subscription ended [Test Stream] #2 (Active: 0)
```

In this implementation,
- ✅ `defer` to generate a new ID each time you subscribe
- ✅ `finalize` to ensure that the unsubscription process is performed reliably
- ✅ Track the number of active subscriptions in real time
- ✅ Type safe and works with RxJS v8

## Detect unnecessary reevaluation

Checks to see if the same value has been calculated more than once.

```ts
import { of } from 'rxjs';
import { map, tap, shareReplay } from 'rxjs';

let computeCount = 0;

function expensiveComputation(value: number): number {
  computeCount++;
  console.log(`💰 Computation executed (${computeCount} times):`, value);
  // Simulate heavy computation
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result += Math.sin(i);
  }
  return result;
}

// ❌ Without shareReplay → Computed for each subscription
console.log('=== Without shareReplay ===');
computeCount = 0;
const withoutShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x))
);

withoutShare$.subscribe(v => console.log('Subscription 1:', v));
withoutShare$.subscribe(v => console.log('Subscription 2:', v));
// Output: Computation runs 6 times (3 values × 2 subscriptions)

// ✅ With shareReplay → Computation results are shared
console.log('\n=== With shareReplay ===');
computeCount = 0;
const withShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x)),
  shareReplay(3)
);

withShare$.subscribe(v => console.log('Subscription 1:', v));
withShare$.subscribe(v => console.log('Subscription 2:', v));
// Output: Computation runs only 3 times
```

## Monitor memory usage

This monitoring method is used to detect memory leaks.

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MemoryMonitor {
  private intervals: ReturnType<typeof setInterval>[] = [];

  start(intervalMs: number = 5000) {
    const id = setInterval(() => {
      if (typeof performance !== 'undefined' && (performance as any).memory) {
        const memory = (performance as any).memory;
        console.log('📊 Memory usage:', {
          Used: `${(memory.usedJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          Total: `${(memory.totalJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          Limit: `${(memory.jsHeapSizeLimit / 1024 / 1024).toFixed(2)} MB`
        });
      }
    }, intervalMs);

    this.intervals.push(id);
  }

  stop() {
    this.intervals.forEach(id => clearInterval(id));
    this.intervals = [];
  }
}

// Usage example
const monitor = new MemoryMonitor();
monitor.start(5000); // Display memory usage every 5 seconds

// Test memory leak
const leakyStreams: any[] = [];

for (let i = 0; i < 100; i++) {
  // ❌ Stream without unsubscription
  const sub = interval(100).subscribe();
  leakyStreams.push(sub);
}

// Unsubscribe after 10 seconds
setTimeout(() => {
  console.log('Unsubscription started');
  leakyStreams.forEach(sub => sub.unsubscribe());
  console.log('Unsubscription completed');

  // Stop monitoring after another 10 seconds
  setTimeout(() => {
    monitor.stop();
  }, 10000);
}, 10000);
```

## Best Practices

### Establishing a Debugging Environment

How to enable debug logging only in the development environment.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Determine debug mode (adjust according to build tool)
const IS_DEVELOPMENT =
  // When using Vite: import.meta.env.DEV
  // When using webpack: process.env.NODE_ENV === 'development'
  // Manual setup: define global variable
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

function devLog<T>(label: string) {
  if (!IS_DEVELOPMENT) {
    return (source: Observable<T>) => source;
  }

  return tap<T>({
    next: value => console.log(`[${label}]`, value),
    error: error => console.error(`[${label}] Error:`, error),
    complete: () => console.log(`[${label}] Complete`)
  });
}

// Usage example
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    devLog('Input'),
    map(x => x * 2),
    devLog('Output')
  )
  .subscribe();
// No logs in production environment
```

### Type safe debugging

This is a debugging method that takes advantage of the TypeScript type system.

```ts
import { tap } from 'rxjs';

type LogLevel = 'debug' | 'info' | 'warn' | 'error';

interface TypedDebugOptions<T> {
  label: string;
  level?: LogLevel;
  transform?: (value: T) => any;
  filter?: (value: T) => boolean;
}

function typedDebug<T>(options: TypedDebugOptions<T>) {
  const { label, level = 'debug', transform, filter } = options;

  const logFn = console[level] || console.log;

  return tap<T>({
    next: value => {
      if (filter && !filter(value)) return;

      const displayValue = transform ? transform(value) : value;
      logFn(`[${label}]`, displayValue);
    }
  });
}

// Usage example
interface User {
  id: number;
  name: string;
  email: string;
}

import { of } from 'rxjs';

of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob', email: 'bob@example.com' },
  { id: 3, name: 'Charlie', email: 'charlie@example.com' }
)
  .pipe(
    typedDebug<User>({
      label: 'User Stream',
      level: 'info',
      transform: user => `${user.name} (${user.email})`,
      filter: user => user.id > 1
    })
  )
  .subscribe();

// Output:
// [User Stream] Bob (bob@example.com)
// [User Stream] Charlie (charlie@example.com)
```

### Setting Error Boundaries

Properly isolate errors for easier debugging.

```ts
import { Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

function errorBoundary<T>(label: string) {
  return (source: Observable<T>) =>
    source.pipe(
      catchError(error => {
        console.error(`🔴 [${label}] Error caught:`, {
          message: error.message,
          stack: error.stack,
          timestamp: new Date().toISOString()
        });

        // Rethrow error or return fallback value
        throw error;
      })
    );
}

// Usage example
import { throwError } from 'rxjs';
import { mergeMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    errorBoundary('Main process'),
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Error at value 2'));
      }
      return of(value);
    }),
    errorBoundary('Async process')
  )
  .subscribe({
    next: value => console.log('Success:', value),
    error: error => console.log('Final error:', error.message)
  });
```

## Summary

Performance Debugging and Best Practices

### Performance Monitoring
- ✅ **Track subscriptions** - manage subscriptions using defer and finalize
- ✅ **Detect re-evaluations** - avoid unnecessary calculations with shareReplay
- ✅ **Memory Monitoring** - track memory usage with performance API

### Optimize your development environment
- ✅ **Environment-specific settings** - enable debug logging only in development environment
- ✅ **Type safe debugging** - leverage TypeScript's type system
- ✅ **Error Boundaries** - properly isolate and debug errors

Together, these techniques optimize the performance of RxJS applications and create an efficient debugging environment.

## Related Pages

- [Basic Debugging Strategies](/en/guide/debugging/) - How to use tap operator and developer tools
- [Common Debugging Scenarios](/en/guide/debugging/common-scenarios) - Problem-specific troubleshooting
- [Custom Debug Tools](/en/guide/debugging/custom-tools) - Named streams, debug operators
- [Operator - shareReplay](/en/guide/operators/multicasting/shareReplay) - Avoid unnecessary reevaluations
