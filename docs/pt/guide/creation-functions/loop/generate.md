---
description: "generate() - Generic loop generation with flexible condition control: Declarative while-like loops for Fibonacci, pagination, and custom state management"
---

# generate() - Generic Loop Generation

`generate()` is a Creation Function that provides flexible loop processing as Observable by specifying initial state, continuation condition, state update, and result selection.

## Overview

`generate()` can declaratively describe flexible loop processing like while and for statements. It is used when more complex conditions or state management than `range()` is required.

**Signature**:
```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

**Parameters**:
- `initialState`: The initial state of the loop
- `condition`: Function to determine the continuation condition (`false` terminates the loop)
- `iterate`: Function to advance the state to the next (update state)
- `resultSelector`: Function to select a value to issue from the state (if omitted, the state itself is issued)
- `scheduler`: Scheduler that issues values (omitted: issues values synchronously)

**Official Documentation**: [📘 RxJS Official: generate()](https://rxjs.dev/api/index/function/generate)

## Basic Usage

### Pattern 1: Simple Counter

This is the most basic usage.

```typescript
import { generate } from 'rxjs';

// Count from 1 to 5
generate(
  1,              // Initial state
  x => x <= 5,    // Continuation condition
  x => x + 1      // State update
).subscribe({
  next: value => console.log('Value:', value),
  complete: () => console.log('Complete')
});

// Output:
// Value: 1
// Value: 2
// Value: 3
// Value: 4
// Value: 5
// Complete
```

This code is equivalent to the following while statement:

```typescript
let x = 1;
while (x <= 5) {
  console.log('Value:', x);
  x = x + 1;
}
console.log('Complete');
```

### Pattern 2: Convert Values with resultSelector

You can separate the state from the value to be issued.

```typescript
import { generate } from 'rxjs';

// Internal state is a counter, but emitted value is a squared value
generate(
  1,              // Initial state: 1
  x => x <= 5,    // Continuation condition: x <= 5
  x => x + 1,     // State update: x + 1
  x => x * x      // Result selection: issue x^2
).subscribe(console.log);

// Output: 1, 4, 9, 16, 25
```

### Pattern 3: Complex State Object

Complex objects can be used as states.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

// Compute cumulative sum
generate<number, State>(
  { count: 1, sum: 0 },           // Initial state
  state => state.count <= 5,      // Continuation condition
  state => ({                     // State update
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => state.sum              // Result selection
).subscribe(console.log);

// Output: 0, 1, 3, 6, 10
// (0, 0+1, 0+1+2, 0+1+2+3, 0+1+2+3+4)
```

## Important Characteristics

### 1. While Statement-like Behavior

`generate()` provides flexible control like a while statement.

```typescript
import { generate } from "rxjs";

// While statement
let i = 1;
while (i <= 10) {
  console.log(i);
  i = i * 2;
}

// Same thing with generate()
generate(
  1,              // let i = 1;
  i => i <= 10,   // while (i <= 10)
  i => i * 2      // i = i * 2;
).subscribe(console.log);

// Output: 1, 2, 4, 8
```

### 2. Synchronous Emission

By default, all values are published **synchronously** upon subscription.

```typescript
import { generate } from 'rxjs';

console.log('Before subscription');

generate(1, x => x <= 3, x => x + 1).subscribe(val => console.log('Value:', val));

console.log('After subscription');

// Output:
// Before subscription
// Value: 1
// Value: 2
// Value: 3
// After subscription
```

### 3. Beware of Infinite Loops

If the condition is always `true`, you will get an infinite loop.

```typescript
import { generate, take } from 'rxjs';
// ❌ Danger: infinite loop (browser freezes)
// generate(0, x => true, x => x + 1).subscribe(console.log);

// ✅ Safe: use take() to limit number
generate(
  0,
  x => true,  // Always true
  x => x + 1
).pipe(
  take(10)    // Get only the first 10
).subscribe(console.log);

// Output: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

> [!WARNING]
> **Beware of Infinite Loops**:
> - If the condition is always `true`, an infinite loop occurs
> - Use `take()`, `takeWhile()`, or `takeUntil()` to limit the number of issues
> - Or set appropriate exit conditions with conditional functions

## Practical Use Cases

### 1. Fibonacci Sequence

Example of complex state transitions.

```typescript
import { generate, take } from 'rxjs';
interface FibState {
  current: number;
  next: number;
}

// First 10 terms of the Fibonacci sequence
generate<number, FibState>(
  { current: 0, next: 1 },           // Initial state: F(0)=0, F(1)=1
  state => true,                     // Infinitely generated
  state => ({                        // State update
    current: state.next,
    next: state.current + state.next
  }),
  state => state.current             // Issue current value
).pipe(
  take(10)                           // First 10 terms
).subscribe(console.log);

// Output: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### 2. Exponential Backoff

This is the exponential wait time generation used in the retry process.

```typescript
import { generate } from 'rxjs';

interface RetryState {
  attempt: number;
  delay: number;
}

// Generate delay for exponential backoff (1, 2, 4, 8, 16 seconds)
generate<number, RetryState>(
  { attempt: 0, delay: 1000 },       // Initial state: 1 second
  state => state.attempt < 5,        // Maximum 5 attempts
  state => ({                        // State update
    attempt: state.attempt + 1,
    delay: state.delay * 2           // Double the delay time
  }),
  state => state.delay               // Issue delay time
).subscribe(delay => {
  console.log(`Retry ${delay / 1000} seconds later`);
});

// Output:
// Retry 1 second later
// Retry 2 seconds later
// Retry 4 seconds later
// Retry 8 seconds later
// Retry 16 seconds later
```

### 3. Pagination Control

Continue fetching as long as the next page exists.

```typescript
import { generate, of, Observable, concatMap, delay } from 'rxjs';
interface PageState {
  page: number;
  hasNext: boolean;
}

interface PageData {
  page: number;
  items: string[];
  hasNext: boolean;
}

// Function to simulate fetching page data
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Item${page}-1`, `Item${page}-2`, `Item${page}-3`],
    hasNext: page < 10 // Up to page 10
  }).pipe(
    delay(500) // Simulate API call
  );
}

// Get the page as long as it exists (actually get hasNext from the API response)
generate<number, PageState>(
  { page: 1, hasNext: true },        // Initial state
  state => state.hasNext,            // Continue as long as there is a next page
  state => ({                        // State update
    page: state.page + 1,
    hasNext: state.page < 10         // Suppose there are up to 10 pages
  }),
  state => state.page                // Issue page number
).pipe(
  concatMap(page => fetchPage(page)) // Fetch each page in turn
).subscribe(
  data => console.log(`Page ${data.page} fetch:`, data.items),
  err => console.error('Error:', err),
  () => console.log('All pages retrieved')
);

// Output:
// Page 1 fetch: ['Item1-1', 'Item1-2', 'Item1-3']
// Page 2 fetch: ['Item2-1', 'Item2-2', 'Item2-3']
// ...
// Page 10 fetch: ['Item10-1', 'Item10-2', 'Item10-3']
// All pages retrieved
```

### 4. Custom Timer

Emits events at irregular intervals.

```typescript
import { generate, of, concatMap, delay } from 'rxjs';
interface TimerState {
  count: number;
  delay: number;
}

// Timer with gradually increasing delay
generate<string, TimerState>(
  { count: 0, delay: 1000 },         // Initial state: 1 second
  state => state.count < 5,          // Up to 5 times
  state => ({                        // State update
    count: state.count + 1,
    delay: state.delay + 500         // Increase delay by 500 ms
  }),
  state => `Event${state.count + 1}`
).pipe(
  concatMap((message, index) => {
    const delayTime = 1000 + index * 500;
    console.log(`${delayTime}ms wait before issuing`);
    return of(message).pipe(delay(delayTime));
  })
).subscribe(console.log);

// Output:
// Issued after 1000ms wait
// Event 1 (after 1 second)
// Issued after 1500ms wait
// Event 2 (after 2.5 seconds)
// Issued after 2000ms wait
// Event 3 (after 4.5 seconds)
// ...
```

### 5. Computing Factorials

Represent mathematical calculations as streams.

```typescript
import { generate } from 'rxjs';

interface FactorialState {
  n: number;
  result: number;
}

// Compute factorial of 5 (5! = 5 × 4 × 3 × 2 × 1 = 120)
generate<number, FactorialState>(
  { n: 5, result: 1 },               // Initial state
  state => state.n > 0,              // Continues for n > 0
  state => ({                        // State update
    n: state.n - 1,
    result: state.result * state.n
  }),
  state => state.result              // Issue intermediate result
).subscribe(console.log);

// Output: 5, 20, 60, 120, 120
// (1*5, 5*4, 20*3, 60*2, 120*1)
```

## Comparison with Other Creation Functions

### generate() vs range()

```typescript
import { generate, range } from 'rxjs';

// range() - simple sequential numbering
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// generate() - same thing, but more explicit
generate(
  1,
  x => x <= 5,
  x => x + 1
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// True value of generate(): complex steps
generate(
  1,
  x => x <= 100,
  x => x * 2  // Increase by a factor of 2
).subscribe(console.log);
// Output: 1, 2, 4, 8, 16, 32, 64
// (not possible with range())
```

### generate() vs defer()

```typescript
import { generate, defer, of } from 'rxjs';

// generate() - loop processing
generate(1, x => x <= 3, x => x + 1).subscribe(console.log);
// Output: 1, 2, 3

// defer() - generate on subscription (not a loop)
defer(() => of(1, 2, 3)).subscribe(console.log);
// Output: 1, 2, 3

// Difference: generate() has state, defer only lazy evaluation
```

> [!TIP]
> **Selection Criteria**:
> - **Simple sequential numbers** → `range()`
> - **Complex conditions or steps** → `generate()`
> - **Dynamically determined upon subscription** → `defer()`
> - **Fibonacci, factorial, etc.** → `generate()`

## Asynchronization with Scheduler

When processing large amounts of data, asynchronous execution is possible by specifying a scheduler.

```typescript
import { generate, asyncScheduler, observeOn } from 'rxjs';
console.log('Start');

// Run a million loops asynchronously
generate(
  1,
  x => x <= 1000000,
  x => x + 1
).pipe(
  observeOn(asyncScheduler)
).subscribe({
  next: val => {
    if (val % 100000 === 0) {
      console.log(`Progress: ${val}`);
    }
  },
  complete: () => console.log('Complete')
});

console.log('After subscription (asynchronous, so it is executed immediately)');

// Output:
// Start
// After subscription (asynchronous, so it will be executed immediately)
// Progress: 100000
// Progress: 200000
// ...
// Complete
```

## Performance Considerations

Because `generate()` issues values synchronously, performance should be considered when generating large numbers of values or performing complex calculations.

> [!WARNING]
> **Performance Optimization**:
> ```typescript
> // ❌ Bad example: complex computation performed synchronously (UI blocked)
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).subscribe(console.log);
>
> // ✅ Good example 1: asynchronous with scheduler
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Good Example 2: Limit the number in take()
> generate(
>   1,
>   x => true,  // Infinite loop
>   x => x + 1
> ).pipe(
>   take(100)   // Only the first 100
> ).subscribe(console.log);
> ```

## Error Handling

Although `generate()` itself does not issue errors, errors can occur in pipelines and state update functions.

```typescript
import { generate, of, map, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,
  x => x + 1
).pipe(
  map(n => {
    if (n === 5) {
      throw new Error('Error at 5');
    }
    return n * 2;
  }),
  catchError(error => {
    console.error('Error occurred:', error.message);
    return of(-1); // Return default value
  })
).subscribe(console.log);

// Output: 2, 4, 6, 8, -1
```

### Error in State Update Function

An error within a state update function will cause Observable to enter an error state.

```typescript
import { generate, EMPTY, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,
  x => {
    if (x === 5) {
      throw new Error('Error on status update');
    }
    return x + 1;
  }
).pipe(
  catchError(error => {
    console.error('Error:', error.message);
    return EMPTY; // Return empty Observable
  })
).subscribe({
  next: console.log,
  complete: () => console.log('Complete')
});

// Output: 1, 2, 3, 4, Error: Error on status update, Complete
```

## Type Safety in TypeScript

`generate()` can separate the type of the state from the type of the issued value.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

interface Result {
  index: number;
  average: number;
}

// State: State, issued value: Result
const stats$ = generate<Result, State>(
  { count: 1, sum: 0 },
  state => state.count <= 5,
  state => ({
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => ({
    index: state.count,
    average: state.sum / state.count
  })
);

stats$.subscribe(result => {
  console.log(`[${result.index}] Average: ${result.average}`);
});

// Output:
// [1] Average: 0
// [2] Average: 0.5
// [3] Average: 1
// [4] Average: 1.5
// [5] Average: 2
```

## Summary

`generate()` is a powerful Creation Function that allows complex loop processing to be described declaratively.

> [!IMPORTANT]
> **Features of generate()**:
> - ✅ Flexible loop control like while/for statements
> - ✅ Complex state management possible
> - ✅ Ideal for mathematical calculations such as Fibonacci, factorial, etc.
> - ✅ State and issue values can be separated
> - ⚠️ Beware of infinite loops (limited by `take()`)
> - ⚠️ Consider asynchronous for large amounts of data
> - ⚠️ Use `range()` for simple sequential numbers

## Related Topics

- [range()](/pt/guide/creation-functions/loop/range) - Simple sequential number generation
- [defer()](/pt/guide/creation-functions/conditional/defer) - Dynamic generation on subscription
- [expand()](/pt/guide/operators/transformation/expand) - Recursive expansion (higher-order operator)
- [scan()](/pt/guide/operators/transformation/scan) - Cumulative calculation

## References

- [RxJS Official: generate()](https://rxjs.dev/api/index/function/generate)
- [Learn RxJS: generate](https://www.learnrxjs.io/learn-rxjs/operators/creation/generate)
