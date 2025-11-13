---
description: This section describes scheduled and using, which are RxJS control Creation Functions. scheduled controls Observable execution timing by specifying a scheduler, and using automatically manages resources such as WebSocket and file handles according to Observable lifecycle. It can also be used for testing and performance optimization.
---

# Control Creation Functions

RxJS provides Creation Functions to control Observable execution timing and resource management in detail. This section describes two functions, `scheduled()` and `using()`, in detail.

## What are Control Creation Functions?

Control Creation Functions are a set of functions for finer control of Observable's behavior. They support advanced use cases such as execution timing control (scheduler) and resource lifecycle management.

### Main Features

- **Execution timing control**: Use scheduler to switch between synchronous and asynchronous execution
- **Resource management**: Automatic resource release according to Observable lifecycle
- **Ease of testing**: Switch between schedulers for ease of testing
- **Performance optimization**: Control execution timing to avoid UI blocking

## List of Control Creation Functions

| Function | Description | Main Uses |
|------|------|---------|
| [scheduled()](/en/guide/creation-functions/control/scheduled) | Generate Observable with specified scheduler | Execution timing control, testing |
| [using()](/en/guide/creation-functions/control/using) | Observable with resource control | Resource management for WebSocket, file handles, etc. |

## scheduled() Basics

`scheduled()` is a function that allows you to explicitly specify a scheduler when generating an Observable from an existing data source (array, Promise, Iterable, etc.).

### Basic Usage

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Emit array asynchronously
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Start subscription');
observable$.subscribe({
  next: val => console.log('Value:', val),
  complete: () => console.log('Complete')
});
console.log('End subscription');

// Output:
// Start subscription
// End subscription
// Value: 1
// Value: 2
// Value: 3
// Complete
```

> [!NOTE]
> With `asyncScheduler`, value emission becomes asynchronous. This allows the subscription process to run without blocking the main thread.

## using() Basics

`using()` is a function that automatically creates and releases resources according to Observable's lifecycle. It creates a resource at the start of a subscription and automatically releases it when the subscription ends (`complete` or `unsubscribe`).

### Basic Usage

```typescript
import { using, interval, Subscription, take } from 'rxjs';

const resource$ = using(
  // Resource factory: executed at subscription start
  () => {
    console.log('Resource created');
    return new Subscription(() => console.log('Resource released'));
  },
  // Observable factory: create Observable using resource
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Value:', value),
  complete: () => console.log('Complete')
});

// Output:
// Resource created
// Value: 0
// Value: 1
// Value: 2
// Complete
// Resource released
```

> [!IMPORTANT]
> `using()` automatically releases resources at the end of a subscription, thus preventing memory leaks.

## Comparison: scheduled() vs using()

| Feature | scheduled() | using() |
|------|-------------|---------|
| Main Purpose | Execution timing control | Resource lifecycle management |
| Scheduler | ✅ Can specify explicitly | ❌ Cannot specify |
| Resource Management | ❌ Manual management required | ✅ Automatic management |
| Use Cases | Testing, UI optimization | WebSocket, file handles |
| Complexity | Simple | Somewhat complex |

## Usage Guidelines

### When to Choose scheduled()

1. **Want to control execution timing**
   - Want to change synchronous processing to asynchronous
   - Want to avoid UI blocking

2. **Need time control for testing**
   - Combine with TestScheduler to control time
   - Want to test asynchronous processing synchronously

3. **Convert existing data sources to Observable**
   - Convert Array, Promise, Iterable to Observable
   - Want to explicitly specify a scheduler

### When to Choose using()

1. **Automatic resource release is required**
   - Managing WebSocket connections
   - File handle management
   - Automatic timer cleanup

2. **Want to prevent memory leaks**
   - Prevent forgetting to release resources
   - Reliable cleanup at end of subscription

3. **Complex resource management**
   - Manage multiple resources at once
   - Manage resource dependencies

## Practical Usage Examples

### scheduled() Usage Example

```typescript
import { scheduled, asyncScheduler, queueScheduler } from 'rxjs';

// Process large amounts of data asynchronously (doesn't block UI)
const largeArray = Array.from({ length: 10000 }, (_, i) => i);
const async$ = scheduled(largeArray, asyncScheduler);

async$.subscribe(value => {
  // Execute heavy processing here
  // UI is not blocked
});

// Execute synchronously in tests
const sync$ = scheduled(largeArray, queueScheduler);
```

### using() Usage Example

```typescript
import { using, timer } from 'rxjs';

// Automatically manage WebSocket connection
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    console.log('WebSocket connection started');
    return {
      unsubscribe: () => {
        ws.close();
        console.log('WebSocket connection ended');
      }
    };
  },
  () => timer(0, 1000) // Receive messages every 1 second
);
```

## Scheduler Types (for scheduled())

| Scheduler | Description | Use Cases |
|---------------|------|---------|
| `queueScheduler` | Synchronous execution (queue method) | Default, synchronous processing |
| `asyncScheduler` | Asynchronous execution (setTimeout) | UI optimization, long-running processing |
| `asapScheduler` | Fastest asynchronous execution (Promise) | High priority asynchronous processing |
| `animationFrameScheduler` | Animation frame | Animation, UI rendering |

> [!TIP]
> For more information on schedulers, see [Scheduler Types](/en/guide/schedulers/types).

## Frequently Asked Questions

### Q1: What is the difference between scheduled() and from()?

**A:** `from()` uses the default scheduler (synchronous) internally. `scheduled()` allows the scheduler to be specified explicitly, thus allowing fine control of execution timing.

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - execute synchronously
const sync$ = from([1, 2, 3]);

// scheduled() - execute asynchronously
const async$ = scheduled([1, 2, 3], asyncScheduler);
```

### Q2: When should I use using()?

**A:** Use when you want to avoid forgetting to release resources. It is especially useful in the following cases:
- Network connections such as WebSocket, EventSource, etc.
- File handles, database connections
- Processes that require manual `clearInterval()` or `clearTimeout()`

### Q3: Why is scheduled() easier to test?

**A:** TestScheduler allows you to virtually control the passage of time. Asynchronous processes can be tested synchronously, greatly reducing test execution time.

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled, asyncScheduler } from 'rxjs';

const testScheduler = new TestScheduler((actual, expected) => {
  expect(actual).toEqual(expected);
});

testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

## Best Practices

### 1. Avoid UI Blocking with scheduled()

```typescript
// ❌ Bad example: Process large amounts of data synchronously
from(largeArray).subscribe(processHeavyTask);

// ✅ Good example: Asynchronous processing with asyncScheduler
scheduled(largeArray, asyncScheduler).subscribe(processHeavyTask);
```

### 2. Ensure Resource Release with using()

```typescript
// ❌ Bad example: Manual resource management
const ws = new WebSocket('wss://example.com');
const source$ = interval(1000);
source$.subscribe(() => ws.send('ping'));
// Resource leak if unsubscribe forgotten

// ✅ Good example: Automatic management with using()
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return { unsubscribe: () => ws.close() };
  },
  () => interval(1000).pipe(tap(() => ws.send('ping')))
);
```

### 3. Use Appropriate Scheduler for Testing

```typescript
// ✅ Good example: TestScheduler for testing
const testScheduler = new TestScheduler(...);
const source$ = scheduled([1, 2, 3], testScheduler);

// ✅ Good example: asyncScheduler for production
const source$ = scheduled([1, 2, 3], asyncScheduler);
```

## Summary

Control Creation Functions are advanced functions for fine-tuning Observable's behavior.

**scheduled():**
- Explicitly controls execution timing (synchronous/asynchronous)
- Useful for time control in testing
- Effective for avoiding UI blocking

**using():**
- Automatic management of resource lifecycle
- Prevents memory leaks
- Ideal for managing connections such as WebSocket

Used appropriately, you can build more robust and performant RxJS applications.

## Next Steps

For detailed usage of each function, please refer to the following pages:

- [scheduled() in detail](/en/guide/creation-functions/control/scheduled) - Generate Observable with scheduler
- [using() in detail](/en/guide/creation-functions/control/using) - Observable with resource control

## Reference Resources

- [RxJS Official Documentation - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [RxJS Official Documentation - using](https://rxjs.dev/api/index/function/using)
- [RxJS Official Documentation - Scheduler](https://rxjs.dev/guide/scheduler)
- [Scheduler Types](/en/guide/schedulers/types)
