---
description: This presentation details how to use RxJS's scheduled() function to specify a scheduler, generate an Observable, and control the timing of execution, with practical code examples.
---

# scheduled()

[📘 RxJS Official Documentation - scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` is a Creation Function that allows you to explicitly specify a scheduler when generating Observables from data sources such as arrays, Promises, and Iterables. This allows fine control of execution timing (synchronous or asynchronous) and is useful for testing and UI performance optimization.

## Basic usage

### Converting a simple array into an Observable

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Issue array asynchronously
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Subscription started');
observable$.subscribe({
  next: val => console.log('Value:', val),
  complete: () => console.log('Complete')
});
console.log('Subscription ended');

// Output:
// Subscription started
// Subscription ended
// Value: 1
// Value: 2
// Value: 3
// Complete
```

> [!IMPORTANT]
> **Synchronous vs. Asynchronous Difference**
>
> Using `asyncScheduler` makes value emission asynchronous. Therefore, the output order is: "Subscription started" → "Subscription ended" → "Value: 1".

### Comparison with from()

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - default is synchronous
console.log('=== from() ===');
from([1, 2, 3]).subscribe(val => console.log('Value:', val));
console.log('Subscription ended');

// Output:
// === from() ===
// Value: 1
// Value: 2
// Value: 3
// Subscription ended

// scheduled() - explicitly asynchronous
console.log('=== scheduled() ===');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Value:', val));
console.log('Subscription ended');

// Output:
// === scheduled() ===
// Subscription ended
// Value: 1
// Value: 2
// Value: 3
```

## Types of schedulers

RxJS provides multiple schedulers, which can be used for different purposes.

| Scheduler | Execution Timing | Base Technology | Main Use |
|-----------|-----------------|----------------|----------|
| `queueScheduler` | Synchronous (queue) | Immediate execution | Default, synchronous processing |
| `asyncScheduler` | Asynchronous | `setTimeout` | UI optimization, long processing |
| `asapScheduler` | Fastest asynchronous | `Promise` (microtask) | High-priority asynchronous processing |
| `animationFrameScheduler` | Animation frame | `requestAnimationFrame` | Animation, UI rendering |

### queueScheduler (synchronous execution)

```typescript
import { scheduled, queueScheduler } from 'rxjs';

console.log('Start');
scheduled([1, 2, 3], queueScheduler).subscribe(val => console.log('Value:', val));
console.log('End');

// Output:
// Start
// Value: 1
// Value: 2
// Value: 3
// End
```

### asyncScheduler (asynchronous execution)

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

console.log('Start');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Value:', val));
console.log('End');

// Output:
// Start
// End
// Value: 1
// Value: 2
// Value: 3
```

### asapScheduler (microtask)

```typescript
import { scheduled, asapScheduler } from 'rxjs';

console.log('Start');
scheduled([1, 2, 3], asapScheduler).subscribe(val => console.log('Value:', val));
console.log('End');

// Output:
// Start
// End
// Value: 1
// Value: 2
// Value: 3
```

> [!TIP]
> **asyncScheduler vs asapScheduler**
>
> - `asyncScheduler`: `setTimeout` based (macrotask)
> - `asapScheduler`: `Promise` based (microtask)
>
> `asapScheduler` executes faster, but both are asynchronous.

### animationFrameScheduler (animation)

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// Update values at each animation frame
const positions = [0, 50, 100, 150, 200];
const animation$ = scheduled(positions, animationFrameScheduler).pipe(
  map(pos => `Position: ${pos}px`)
);

animation$.subscribe(position => {
  console.log(position);
  // Update DOM here
});

// Output: (at each animation frame)
// Position: 0px
// Position: 50px
// Position: 100px
// Position: 150px
// Position: 200px
```

## Practical Patterns

### Mass data processing without blocking UI

```typescript
import { scheduled, asyncScheduler, map, bufferCount } from 'rxjs';
// Process 1 million data items
const largeArray = Array.from({ length: 1000000 }, (_, i) => i);

// ❌ Bad example: synchronous processing (UI will be blocked)
// from(largeArray).subscribe(processData);

// ✅ Good example: asynchronous processing (UI will not be blocked)
scheduled(largeArray, asyncScheduler).pipe(
  bufferCount(1000), // Batch process 1000 at a time
  map(batch => batch.reduce((sum, val) => sum + val, 0))
).subscribe({
  next: sum => console.log('Batch total:', sum),
  complete: () => console.log('Processing complete')
});

console.log('UI remains responsive');
```

### Combination with Promise

```typescript
import { scheduled, asyncScheduler, mergeMap } from 'rxjs';
interface User {
  id: number;
  name: string;
}

const userIds = [1, 2, 3, 4, 5];

// Fetch multiple users asynchronously
scheduled(userIds, asyncScheduler).pipe(
  mergeMap(id =>
    fetch(`https://api.example.com/users/${id}`).then(res => res.json())
  )
).subscribe({
  next: (user: User) => console.log('User:', user),
  error: error => console.error('Error:', error),
  complete: () => console.log('All users fetched')
});
```

### Generation from Iterable

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Convert Set with scheduling
const uniqueNumbers = new Set([1, 2, 3, 4, 5]);
const observable$ = scheduled(uniqueNumbers, asyncScheduler);

observable$.subscribe({
  next: val => console.log('Value:', val),
  complete: () => console.log('Complete')
});

// Convert Map with scheduling
const userMap = new Map([
  [1, 'Alice'],
  [2, 'Bob'],
  [3, 'Charlie']
]);

scheduled(userMap, asyncScheduler).subscribe({
  next: ([id, name]) => console.log(`ID: ${id}, Name: ${name}`),
  complete: () => console.log('Complete')
});
```

## Use in testing

`scheduled()` can be combined with TestScheduler to write tests with time control.

### Basic testing

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled } from 'rxjs';

describe('scheduled()', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('emits array elements in order', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler);
      const expected = '(abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

### Testing asynchronous processing

```typescript
import { scheduled, asyncScheduler, delay } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Testing asynchronous processing', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('virtually tests delayed processing', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler).pipe(
        delay(1000, testScheduler)
      );

      // Emit after 1000ms (virtual time)
      const expected = '1000ms (abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

> [!TIP]
> **Benefits of TestScheduler**
>
> - Test without actually waiting for time
> - Test asynchronous processing synchronously
> - Dramatically shorten test execution time

## Common Usage Examples

### 1. Paginated data retrieval

```typescript
import { scheduled, asyncScheduler, mergeMap, toArray } from 'rxjs';
interface Page {
  page: number;
  data: any[];
}

// List of page numbers
const pages = [1, 2, 3, 4, 5];

// Fetch each page asynchronously
const allData$ = scheduled(pages, asyncScheduler).pipe(
  mergeMap(page =>
    fetch(`https://api.example.com/items?page=${page}`)
      .then(res => res.json())
  ),
  toArray() // Combine all pages of data
);

allData$.subscribe({
  next: data => console.log('All data:', data),
  complete: () => console.log('Fetch complete')
});
```

### 2. Batch processing

```typescript
import { scheduled, asyncScheduler, bufferCount, mergeMap, delay } from 'rxjs';
// Process large number of tasks 1000 at a time
const tasks = Array.from({ length: 10000 }, (_, i) => `Task-${i}`);

scheduled(tasks, asyncScheduler).pipe(
  bufferCount(1000), // Batch 1000 at a time
  mergeMap(batch => {
    console.log(`Processing batch: ${batch.length} items`);
    // Execute batch processing
    return processBatch(batch);
  })
).subscribe({
  complete: () => console.log('All batch processing complete')
});

function processBatch(batch: string[]): Promise<void> {
  // Batch processing logic
  return Promise.resolve();
}
```

### 3. Animation implementation

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// Generate values from 0 to 100
const frames = Array.from({ length: 100 }, (_, i) => i);

// Execute at each animation frame
const animation$ = scheduled(frames, animationFrameScheduler).pipe(
  map(frame => ({
    progress: frame / 100,
    position: frame * 5 // Move from 0px to 500px
  }))
);

animation$.subscribe({
  next: ({ progress, position }) => {
    const element = document.getElementById('animated-box');
    if (element) {
      element.style.transform = `translateX(${position}px)`;
      console.log(`Progress: ${(progress * 100).toFixed(0)}%`);
    }
  },
  complete: () => console.log('Animation complete')
});
```

### 4. Prioritized task processing

```typescript
import { scheduled, asapScheduler, asyncScheduler } from 'rxjs';

// High priority tasks (asapScheduler = microtask)
const highPriorityTasks = ['Urgent task 1', 'Urgent task 2'];
const highPriority$ = scheduled(highPriorityTasks, asapScheduler);

// Low priority tasks (asyncScheduler = macrotask)
const lowPriorityTasks = ['Normal task 1', 'Normal task 2'];
const lowPriority$ = scheduled(lowPriorityTasks, asyncScheduler);

console.log('Task start');

highPriority$.subscribe(task => console.log('High priority:', task));
lowPriority$.subscribe(task => console.log('Low priority:', task));

console.log('Task registration complete');

// Output:
// Task start
// Task registration complete
// High priority: Urgent task 1
// High priority: Urgent task 2
// Low priority: Normal task 1
// Low priority: Normal task 2
```

## scheduled() options

`scheduled()` has the following signature.

```typescript
function scheduled<T>(
  input: ObservableInput<T>,
  scheduler: SchedulerLike
): Observable<T>
```

### Supported input types

- **Array**: `T[]`
- **Promise**: `Promise<T>`
- **Iterable**: `Iterable<T>` (Set, Map, Generator, etc.)
- **Observable**: `Observable<T>`
- **ArrayLike**: `ArrayLike<T>`

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Array
scheduled([1, 2, 3], asyncScheduler);

// Promise
scheduled(Promise.resolve('result'), asyncScheduler);

// Set
scheduled(new Set([1, 2, 3]), asyncScheduler);

// Generator
function* generator() {
  yield 1;
  yield 2;
  yield 3;
}
scheduled(generator(), asyncScheduler);
```

## Common errors and their solutions

### 1. Forgot to specify scheduler

**Error example:**
```typescript
// ❌ Error: 2nd argument required
const observable$ = scheduled([1, 2, 3]);
```

**Solution:**
```typescript
// ✅ Correct: specify scheduler
const observable$ = scheduled([1, 2, 3], asyncScheduler);
```

### 2. Using animationFrameScheduler in a browser environment

**Problem:**
In Node.js environments, `requestAnimationFrame` does not exist, causing errors.

**Solution:**
```typescript
import { scheduled, animationFrameScheduler, asyncScheduler } from 'rxjs';

// Check if browser environment
const scheduler = typeof window !== 'undefined'
  ? animationFrameScheduler
  : asyncScheduler;

const observable$ = scheduled([1, 2, 3], scheduler);
```

### 3. Confusion between synchronous and asynchronous processing

**Problem:**
```typescript
// Expecting asynchronous execution, but actually synchronous
scheduled([1, 2, 3], queueScheduler).subscribe(val => {
  console.log(val);
});
console.log('Complete'); // ← 1, 2, 3 are output before this
```

**Solution:**
```typescript
// Explicitly specify asynchronous
scheduled([1, 2, 3], asyncScheduler).subscribe(val => {
  console.log(val);
});
console.log('Complete'); // ← 1, 2, 3 are output after this
```

## Comparison with from()

| Feature | from() | scheduled() |
|---------|--------|-------------|
| Scheduler specification | ❌ Not possible (default only) | ✅ Explicitly specifiable |
| Synchronous/asynchronous control | ❌ Cannot control | ✅ Controllable |
| Test ease | Normal | ✅ Time controllable with TestScheduler |
| Simplicity | ✅ Simple | Somewhat complex |
| Use case | Basic conversion | When execution timing control is needed |

> [!TIP]
> **Points for choosing**
>
> - **Basically use `from()`**: When scheduler control is not needed
> - **Use `scheduled()` when**:
>   - Want to avoid UI blocking
>   - Need time control in tests
>   - Animation implementation
>   - Prioritized task processing

## Best Practices

### 1. Use asyncScheduler for large data processing

```typescript
// ✅ Good example: doesn't block UI
scheduled(largeArray, asyncScheduler).pipe(
  map(processHeavyTask)
).subscribe();
```

### 2. Use TestScheduler for testing

```typescript
// ✅ Good example: control time virtually
testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

### 3. Use animationFrameScheduler for animation

```typescript
// ✅ Good example: match browser repaint timing
scheduled(frames, animationFrameScheduler).subscribe(updateUI);
```

### 4. Select a scheduler that best suits your environment

```typescript
// ✅ Good example: switch according to environment
const scheduler = process.env.NODE_ENV === 'test'
  ? queueScheduler
  : asyncScheduler;

const source$ = scheduled(data, scheduler);
```

## Summary

`scheduled()` is a Creation Function that creates an Observable by explicitly specifying a scheduler.

**Main Features:**
- Explicit control of execution timing (synchronous or asynchronous)
- Multiple schedulers to choose from
- Easy to test with TestScheduler
- Effective for avoiding UI blocking

**Usage scenarios:**
- Asynchronous processing of large amounts of data
- Implementation of animations
- Time control in testing
- Prioritized task processing

**Notes:**
- Always specify a scheduler
- Select the appropriate scheduler for your environment
- Understand the difference between from() and scheduled()

**Recommended usage:**
- UI optimization: `asyncScheduler`
- Animation: `animationFrameScheduler`
- Testing: `TestScheduler`
- High priority: `asapScheduler`

## Related Pages

- [using()](/pt/guide/creation-functions/control/using) - Observable with resource control
- [Control Creation Functions](/pt/guide/creation-functions/control/) - Comparison of scheduled() and using()
- [Scheduler Types](/pt/guide/schedulers/types) - Scheduler details
- [from()](/pt/guide/creation-functions/basic/from) - Basic Observable generation

## References

- [RxJS Official Documentation - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [RxJS Official Documentation - Scheduler](https://rxjs.dev/guide/scheduler)
- [RxJS Official Documentation - TestScheduler](https://rxjs.dev/api/testing/TestScheduler)
