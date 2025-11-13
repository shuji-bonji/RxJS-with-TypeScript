---
description: This section describes Creation Functions that generate values in a loop-like manner, using range and generate to learn how to implement iterative processing such as for and while statements as Observable streams. From sequential number generation to complex state transitions based on custom conditions, you can realize declarative loop processing utilizing TypeScript's type inference.
---

# Loop Generation Creation Functions

Creation Functions for expressing loop processing such as for and while statements as Observable.

## What are Loop Generation Creation Functions?

Loop Generation Creation Functions reactively realize repetitive processing. By replacing conventional imperative loops (`for` and `while` statements) with declarative Observable streams, flexible processing is possible in combination with the RxJS operator chain.

Check the table below to see the characteristics and usage of each Creation Function.

## Major Loop Generation Creation Functions

| Function | Description | Use Cases |
|----------|------|-------------|
| **[range](/en/guide/creation-functions/loop/range)** | Generate a range of numbers (like for statement) | Sequential number generation, batch processing |
| **[generate](/en/guide/creation-functions/loop/generate)** | General-purpose loop generation (like while statement) | Conditional repetition, complex state transitions |

## Usage Criteria

The selection of Loop Generation Creation Functions is determined from the following perspectives.

### 1. Generation Pattern

- **Numeric sequence**: `range()` - Simple sequential number generation with start and end values
- **Complex conditions**: `generate()` - Free control over initial values, conditions, iteration, and result selection

### 2. Loop Types

- **for statement-like loop**: `range()` - `for (let i = start; i <= end; i++)`
- **while statement-like loop**: `generate()` - `while (condition) { ... }`

### 3. Flexibility

- **Simple is sufficient**: `range()` - When you need a sequence of numbers
- **Need advanced control**: `generate()` - Custom state management, conditional branching, step control

## Practical Usage Examples

### range() - Sequential Number Generation

For simple sequential number generation, `range()` is the best choice.

```typescript
import { range, map } from 'rxjs';
// Generate sequential numbers from 1 to 5
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// Use in batch processing
range(0, 10).pipe(
  map(i => `Process ${i + 1}`)
).subscribe(console.log);
// Output: Process 1, Process 2, ..., Process 10
```

### generate() - Conditional Loop

Use `generate()` for complex conditions or custom state management.

```typescript
import { generate } from 'rxjs';

// Generate Fibonacci sequence (first 10 terms)
generate(
  { current: 0, next: 1, count: 0 },  // Initial state
  state => state.count < 10,           // Continue condition
  state => ({                          // State update
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // Result selector
).subscribe(console.log);
// Output: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## Comparison with Imperative Loop

This is a comparison between the conventional imperative loop and RxJS's Loop Generation Creation Functions.

### Imperative for Statement

```typescript
// Conventional for statement
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### Declarative range()

```typescript
import { range, map, toArray } from 'rxjs';
// RxJS range()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

> [!TIP]
> **Advantages of the declarative approach**:
> - Improved readability with pipeline processing
> - Uniform error handling
> - Easy to combine with asynchronous processing
> - Easy to cancel and abort (e.g., `takeUntil()`)

## Converting Cold to Hot

As shown in the table above, **all Loop Generation Creation Functions generate Cold Observables**. Each subscription initiates an independent execution.

However, by using multicast operators (`share()`, `shareReplay()`, etc.), you can **convert a Cold Observable to a Hot Observable**.

### Practical Example: Sharing Calculation Results

```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - Independent calculation for each subscription
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calculating:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('Subscriber 1:', val));
cold$.subscribe(val => console.log('Subscriber 2:', val));
// → Calculation executed twice (2000 calculations)

// 🔥 Hot - Share calculation results among subscribers
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calculating:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('Subscriber 1:', val));
hot$.subscribe(val => console.log('Subscriber 2:', val));
// → Calculation executed only once (1000 calculations)
```

> [!TIP]
> **Cases where Hot conversion is required**:
> - Use high-cost calculations in multiple locations
> - Share batch processing results with multiple components
> - Display pagination results in multiple UI components
>
> For more information, see [Basic Creation - Converting Cold to Hot](/en/guide/creation-functions/basic/#converting-cold-to-hot).

## Combined with Asynchronous Processing

Loop Generation Creation Functions demonstrate powerful functionality when combined with asynchronous processing.

### Sequential Execution of API Calls

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
    items: [`Data${page}-1`, `Data${page}-2`, `Data${page}-3`]
  }).pipe(
    delay(300) // Simulate API call
  );
}

// Sequentially fetch pages 1 to 10 (with 1 second delay between each request)
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe(
  data => console.log(`Page ${data.page} fetched:`, data.items),
  err => console.error('Error:', err)
);
```

### Use in Retry Processing

```typescript
import { range, throwError, of, Observable, mergeMap, retryWhen, delay } from 'rxjs';
// Function to simulate data fetching (randomly fails)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // 40% success rate

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Data fetch failed'))
        : of('Data fetch successful')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          // Retry up to 3 times
          if (index >= 3) {
            return throwError(() => error);
          }
          console.log(`Retry ${index + 1}/3`);
          // Exponential backoff: 1s, 2s, 4s
          return range(0, 1).pipe(delay(Math.pow(2, index) * 1000));
        })
      )
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Result:', result),
  error: err => console.error('Error:', err.message)
});

// Example output:
// Retry 1/3
// Retry 2/3
// Result: Data fetch successful
```

## Relationship with Pipeable Operator

Loop Generation Creation Functions do not have a direct Pipeable Operator counterpart. They are always used as Creation Functions.

However, more advanced processing is possible by combining them with the following operators:

| Operators to Combine | Purpose |
|-------------------|------|
| `map()` | Transform each value |
| `filter()` | Pass only values that match the condition |
| `take()`, `skip()` | Control the number of values |
| `concatMap()`, `mergeMap()` | Execute asynchronous processing for each value |
| `toArray()` | Collect all values into an array |

## Performance Notes

Loop Generation Creation Functions issue values synchronously, so be careful about performance when generating a large number of values.

> [!WARNING]
> **Handling large amounts of data**:
> - Large amounts of data, such as `range(1, 1000000)`, are all issued synchronously and consume memory
> - Buffer with `bufferCount()` or `windowCount()` as needed
> - Or change to asynchronous execution by specifying a scheduler with `scheduled()`

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// Execute with asynchronous scheduler
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## Next Steps

To learn more about the detailed behavior and practical examples of each Creation Function, click on the links from the table above.

You can also understand the whole picture of Creation Functions by learning [Basic Creation Functions](/en/guide/creation-functions/basic/), [Combination Creation Functions](/en/guide/creation-functions/combination/), [Selection/Partition Creation Functions](/en/guide/creation-functions/selection/), and [Conditional Creation Functions](/en/guide/creation-functions/conditional/).
