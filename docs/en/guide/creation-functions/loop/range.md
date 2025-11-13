---
description: "range() - Creation Function that generates consecutive integers declaratively: Memory-efficient alternative to for loops for batch processing and pagination"
---

# range() - Generates a range of numbers

`range()` is a for statement-like Creation Function that issues a specified number of consecutive integers from a specified starting value.

## Overview

`range()` issues a sequence of consecutive integers as Observable by specifying a starting value and the number of integers. It is used for sequential number generation and batch processing as a declarative way to replace the traditional `for` statement.

**Signature**:
```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

**Parameters**:
- `start`: The start value (from which to start issuing)
- `count`: the number of values to publish (omitted, from 0 to less than `start`)
- `scheduler`: the scheduler to issue the values (omitted: issue synchronously)

**Official Documentation**: [📘 RxJS Official: range()](https://rxjs.dev/api/index/function/range)

## Basic Usage

### Pattern 1: Specify starting value and count

This is the most common usage.

```typescript
import { range } from 'rxjs';

// Generate 5 sequential numbers from 1 (1, 2, 3, 4, 5)
range(1, 5).subscribe({
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

### Pattern 2: Sequential numbers starting from 0

By setting the starting value to 0, a sequential number like an array index can be generated.

```typescript
import { range } from 'rxjs';

// 0 to 10 sequential numbers (0, 1, 2, ..., 9)
range(0, 10).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

### Pattern 3: Start with a negative number

Negative numbers can also be generated.

```typescript
import { range } from 'rxjs';

// 5 sequential numbers from -3 (-3, -2, -1, 0, 1)
range(-3, 5).subscribe(console.log);
// Output: -3, -2, -1, 0, 1
```

## Important Characteristics

### 1. Synchronous Emission

By default, `range()` issues all values **synchronously** upon subscription.

```typescript
import { range } from 'rxjs';

console.log('Before subscription');

range(1, 3).subscribe(value => console.log('Value:', value));

console.log('After subscription');

// Output:
// Before subscription
// Value: 1
// Value: 2
// Value: 3
// After subscription
```

### 2. Completes Immediately

Notifies `complete` immediately after publishing all values.

```typescript
import { range } from 'rxjs';

range(1, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Complete!')
});

// Output: 1, 2, 3, Complete!
```

### 3. Equivalence with for statement

`range(start, count)` is equivalent to the following for statement.

```typescript
// Imperative for statement
for (let i = start; i < start + count; i++) {
  console.log(i);
}

// Declarative range()
range(start, count).subscribe(console.log);
```

## Practical Use Cases

### 1. Batch Processing

Used to execute multiple tasks sequentially.

```typescript
import { range, of, Observable, concatMap, delay, map } from 'rxjs';
// Function to simulate data processing
function processItem(index: number): Observable<string> {
  return of(index).pipe(
    delay(100), // Simulate 100ms processing time
    map(i => `Result of processing item ${i}`)
  );
}

// Sequentially process 10 items of data (1 second delay between each process)
range(1, 10).pipe(
  concatMap(index =>
    processItem(index).pipe(delay(1000))
  )
).subscribe({
  next: result => console.log(`Processing complete: ${result}`),
  complete: () => console.log('All processing completed')
});

// Output:
// Processing complete: Result of processing item 1 (after about 1.1 seconds)
// Processing complete: Result of processing item 2 (after about 2.1 seconds)
// ...
// Processing complete: Result of processing item 10 (after approx. 10.1 sec.)
// All processing is complete
```

### 2. Pagination

Retrieve multiple pages of data sequentially.

```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Function to simulate page data fetching
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Item${page}-1`, `Item${page}-2`, `Item${page}-3`]
  }).pipe(
    delay(500) // Simulate API call
  );
}

function fetchAllPages(totalPages: number) {
  return range(1, totalPages).pipe(
    concatMap(page => fetchPage(page))
  );
}

fetchAllPages(5).subscribe({
  next: (data: PageData) => console.log(`Page ${data.page}:`, data.items),
  complete: () => console.log('All pages retrieved')
});

// Output:
// Page 1: ['Item1-1', 'Item1-2', 'Item1-3']
// Page 2: ['Item2-1', 'Item2-2', 'Item2-3']
// Page 3: ['Item3-1', 'Item3-2', 'Item3-3']
// Page 4: ['Item4-1', 'Item4-2', 'Item4-3']
// Page 5: ['Item5-1', 'Item5-2', 'Item5-3']
// All pages retrieved
```

### 3. Processing Array Indexes

Use as an index-based loop when processing each element of an array.

```typescript
import { range, map } from 'rxjs';
const items = ['Apple', 'Banana', 'Cherry', 'Date', 'Elderberry'];

range(0, items.length).pipe(
  map(index => ({ index, item: items[index] }))
).subscribe(({ index, item }) => {
  console.log(`[${index}] ${item}`);
});

// Output:
// [0] Apple
// [1] Banana
// [2] Cherry
// [3] Date
// [4] Elderberry
```

### 4. Test Data Generation

This is useful for generating mock data for unit tests.

```typescript
import { range, map, toArray } from 'rxjs';
// Generate mock user data
range(1, 100).pipe(
  map(id => ({
    id,
    name: `User${id}`,
    email: `user${id}@example.com`
  })),
  toArray()
).subscribe(users => {
  console.log(`${users.length} users generated`);
  // Use in testing
});
```

### 5. Counter for Retry Processing

Controls the number of retries on error.

```typescript
import { range, throwError, concat, of, Observable, mergeMap, delay, catchError, map, toArray } from 'rxjs';
// Function to simulate data fetch (randomly fails)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.7; // 30% chance of success

  return of(shouldFail).pipe(
    delay(300),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Data acquisition failed'))
        : of('Data acquisition succeeded')
    )
  );
}

function fetchWithRetry(maxRetries: number = 3) {
  return concat(
    range(0, maxRetries).pipe(
      map(attempt => {
        console.log(`Attempt ${attempt + 1}/${maxRetries}`);
        return fetchData().pipe(
          catchError(error => {
            if (attempt === maxRetries - 1) {
              return throwError(() => new Error('Maximum retries reached'));
            }
            return throwError(() => error);
          }),
          delay(Math.pow(2, attempt) * 1000) // Exponential backoff
        );
      }),
      toArray()
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Result:', result),
  error: err => console.error('Error:', err.message)
});

// Output example:
// Attempt 1/3
// Attempt 2/3
// Result: Data acquisition succeeded
```

## Asynchronization with Scheduler

When processing large amounts of data, asynchronous execution is possible by specifying a scheduler.

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
console.log('Start');

// Asynchronously issue 1,000,000 numbers
range(1, 1000000).pipe(
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

> [!TIP]
> **Using the Scheduler**:
> - Don't block the UI when processing large amounts of data
> - Time control in testing (TestScheduler)
> - Event loop control in Node.js environment

For more information, please refer to [Types of Schedulers and How to Use Them](/en/guide/schedulers/types).

## Comparison with Other Creation Functions

### range() vs of()

```typescript
import { range, of } from 'rxjs';

// range() - consecutive integers
range(1, 3).subscribe(console.log);
// Output: 1, 2, 3

// of() - enumerate arbitrary values
of(1, 2, 3).subscribe(console.log);
// Output: 1, 2, 3

// Difference: range() only accepts sequential numbers, of() accepts arbitrary values
of(1, 10, 100).subscribe(console.log);
// Output: 1, 10, 100 (not possible with range())
```

### range() vs from()

```typescript
import { range, from } from 'rxjs';

// range() - generate sequential numbers
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// from() - generate from an array (must create array in advance)
from([1, 2, 3, 4, 5]).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// Advantage of range(): no pre-allocation of arrays in memory
range(1, 1000000); // Memory efficient
from(Array.from({ length: 1000000 }, (_, i) => i + 1)); // Array goes into memory
```

### range() vs generate()

```typescript
import { range, generate } from 'rxjs';

// range() - simple sequential numbering
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// generate() - a complex example of the same thing
generate(
  1,                    // Initial value
  x => x <= 5,          // Continuation condition
  x => x + 1            // Iteration
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// Advantages of generate(): complex condition and state management
generate(
  1,
  x => x <= 100,
  x => x * 2  // Increments by a factor of 2
).subscribe(console.log);
// Output: 1, 2, 4, 8, 16, 32, 64
// (not possible with range())
```

> [!TIP]
> **Selection Criteria**:
> - **Requires sequential numbers** → `range()`
> - **Enumerate any value** → `of()`
> - **Existing array/Promise** → `from()`
> - **Complex condition/step** → `generate()`

## Performance Considerations

Because `range()` issues values synchronously, performance should be considered when generating large numbers of values.

> [!WARNING]
> **Handling Large Amounts of Data**:
> ```typescript
> // ❌ Bad example: issue 1 million values synchronously (UI will block)
> range(1, 1000000).subscribe(console.log);
>
> // ✅ Good example 1: asynchronous with scheduler
> range(1, 1000000).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Good Example 2: Split by buffering
> range(1, 1000000).pipe(
>   bufferCount(1000)
> ).subscribe(batch => console.log(`${batch.length} cases processed`));
> ```

## Choosing Between from() Array

```typescript
import { range, from } from 'rxjs';

// If you need sequential numbers → range() is more concise
range(0, 10).subscribe(console.log);

// No need to create an array and then convert it (inefficient)
from(Array.from({ length: 10 }, (_, i) => i)).subscribe(console.log);

// If there is an existing array → use from()
const existingArray = [5, 10, 15, 20];
from(existingArray).subscribe(console.log);
```

## Error Handling

Although `range()` itself does not issue errors, errors may occur in the pipeline.

```typescript
import { range, of, map, catchError } from 'rxjs';
range(1, 10).pipe(
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

## Summary

`range()` is a simple yet powerful Creation Function that produces a sequence of consecutive integers.

> [!IMPORTANT]
> **range() Features**:
> - ✅ Ideal for generating consecutive numbers (alternative to for statement)
> - ✅ Useful for batch processing, pagination, test data generation
> - ✅ Memory efficient (no pre-creation of arrays)
> - ⚠️ Consider asynchronous for large amounts of data
> - ⚠️ Use `generate()` for complex conditions

## Related Topics

- [generate()](/en/guide/creation-functions/loop/generate) - Generic loop generation
- [of()](/en/guide/creation-functions/basic/of) - Enumerates arbitrary values
- [from()](/en/guide/creation-functions/basic/from) - Convert from array or Promise
- [interval()](/en/guide/creation-functions/basic/interval) - Publish values periodically

## References

- [RxJS Official: range()](https://rxjs.dev/api/index/function/range)
- [Learn RxJS: range](https://www.learnrxjs.io/learn-rxjs/operators/creation/range)
