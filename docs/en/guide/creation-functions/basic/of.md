---
description: "of() - Simplest Creation Function that emits specified values in sequence: Perfect for test data, mocks, default values, and conditional branching"
---

# of() - Sequential Emission of Values

`of()` is the simplest Creation Function that emits the specified values one by one in sequence.

## Overview

`of()` emits the values passed as arguments in sequence as they are subscribed to, and completes immediately after all values are emitted. It is frequently used to create test code or mock data.

**Signature**:
```typescript
function of<T>(...args: T[]): Observable<T>
```

**Official Documentation**: [📘 RxJS Official: of()](https://rxjs.dev/api/index/function/of)

## Basic Usage

`of()` allows multiple values to be passed comma-separated.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Value:', value),
  error: err => console.error('Error:', err),
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

## Important Characteristics

### 1. Synchronous Emission

`of()` emits all values **synchronously** upon subscription.

```typescript
import { of } from 'rxjs';

console.log('Before subscription');

of('A', 'B', 'C').subscribe(value => console.log('Value:', value));

console.log('After subscription');

// Output:
// Before subscription
// Value: A
// Value: B
// Value: C
// After subscription
```

### 2. Immediate Completion

Notifies `complete` immediately after emitting all values.

```typescript
import { of } from 'rxjs';

of(1, 2, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Complete!')
});

// Output: 1, 2, 3, Complete!
```

### 3. Can Emit Any Type of Value

Values of any type can be emitted, from primitive types to objects and arrays.

```typescript
import { of } from 'rxjs';

// Primitive types
of(42, 'hello', true).subscribe(console.log);

// Objects
of(
  { id: 1, name: 'Alice' },
  { id: 2, name: 'Bob' }
).subscribe(console.log);

// Arrays (emits the array itself as a single value)
of([1, 2, 3], [4, 5, 6]).subscribe(console.log);
// Output: [1, 2, 3], [4, 5, 6]
```

### 4. Cold Observable

`of()` is a **Cold Observable**. Each subscription initiates an independent execution.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3);

// First subscription
values$.subscribe(val => console.log('Subscriber A:', val));

// Second subscription (executed independently)
values$.subscribe(val => console.log('Subscriber B:', val));

// Output:
// Subscriber A: 1
// Subscriber A: 2
// Subscriber A: 3
// Subscriber B: 1
// Subscriber B: 2
// Subscriber B: 3
```

> [!NOTE]
> **Cold Observable Characteristics**:
> - Independent execution is initiated for each subscription
> - Each subscriber receives its own data stream
> - If you need to share data, you need to make it Hot with `share()` etc.
>
> See [Cold Observable and Hot Observable](/en/guide/observables/cold-and-hot-observables) for more information.

## Difference Between of() and from()

`of()` and `from()` have different behavior when dealing with arrays. This is a common point of confusion.

```typescript
import { of, from } from 'rxjs';

// of() - emits the array as a single value
of([1, 2, 3]).subscribe(console.log);
// Output: [1, 2, 3]

// from() - emits each element of the array individually
from([1, 2, 3]).subscribe(console.log);
// Output: 1, 2, 3
```

> [!IMPORTANT]
> **Usage Criteria**:
> - To emit the array itself → `of([1, 2, 3])`
> - To emit each element of an array separately → `from([1, 2, 3])`

## Practical Use Cases

### 1. Test Data and Mock Creation

`of()` is most frequently used to create mock data in test code.

```typescript
import { of } from 'rxjs';

// Mock user data
function getMockUser$() {
  return of({
    id: 1,
    name: 'Test User',
    email: 'test@example.com'
  });
}

// Use in tests
getMockUser$().subscribe(user => {
  console.log('User:', user.name); // User: Test User
});
```

### 2. Providing Default Values

Used to provide fallback values in case of errors or default values.

```typescript
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

function fetchData(id: number) {
  if (id < 0) {
    return throwError(() => new Error('Invalid ID'));
  }
  return of({ id, data: 'some data' });
}

fetchData(-1).pipe(
  catchError(err => {
    console.error('Error:', err.message);
    return of({ id: 0, data: 'default data' }); // Default value
  })
).subscribe(result => console.log(result));
// Output: Error: Invalid ID
//         { id: 0, data: 'default data' }
```

### 3. Emit Multiple Values Gradually

Used to execute multiple steps in sequence.

```typescript
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('Loading...', 'Processing...', 'Done!').pipe(
  concatMap(message => of(message).pipe(delay(1000)))
).subscribe(console.log);

// Output (every 1 second):
// Loading...
// Processing...
// Done!
```

### 4. Return Values in Conditional Branching

Used in combination with `iif()` and `switchMap()` to return values according to conditions.

```typescript
import { of, iif } from 'rxjs';

const isAuthenticated = true;

iif(
  () => isAuthenticated,
  of('Welcome back!'),
  of('Please log in')
).subscribe(console.log);
// Output: Welcome back!
```

## Using in Pipeline

`of()` is used as the starting point of a pipeline or to inject data along the way.

```typescript
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

of(1, 2, 3, 4, 5).pipe(
  filter(n => n % 2 === 0),  // Even numbers only
  map(n => n * 10)           // Multiply by 10
).subscribe(console.log);
// Output: 20, 40
```

## Common Mistakes

### 1. Passing an Array Directly

```typescript
// ❌ Wrong - the entire array is emitted as a single value
of([1, 2, 3]).subscribe(console.log);
// Output: [1, 2, 3]

// ✅ Correct - use from() to emit each element separately
from([1, 2, 3]).subscribe(console.log);
// Output: 1, 2, 3

// ✅ Or use spread syntax
of(...[1, 2, 3]).subscribe(console.log);
// Output: 1, 2, 3
```

### 2. Confusion with Asynchronous Processing

Note that `of()` emits synchronously. It is not asynchronous processing.

```typescript
// ❌ This does not become asynchronous
of(fetchDataFromAPI()).subscribe(console.log);
// fetchDataFromAPI() executes immediately and the Promise object is emitted

// ✅ Use from() to stream a Promise
from(fetchDataFromAPI()).subscribe(console.log);
```

## Performance Considerations

`of()` is very lightweight and has little performance overhead. However, when emitting large numbers of values, keep the following in mind.

> [!TIP]
> When emitting a large number of values (thousands or more) sequentially, consider using `from()` or `range()`.

## Related Creation Functions

| Function | Difference | Usage |
|----------|------|----------|
| **[from()](/en/guide/creation-functions/basic/from)** | Convert from array or Promise | Stream iterables or Promises |
| **range()** | Generate a range of numbers | Emit consecutive numbers |
| **EMPTY** | Complete immediately without emitting anything | When an empty stream is needed |

## Summary

- `of()` is the simplest Creation Function that emits the specified values in sequence
- Synchronously emitted upon subscription and completes instantly
- Ideal for test data and mock creation
- If an array is passed, the array itself is emitted (different from `from()`)
- Use `from()` for asynchronous processing

## Next Steps

- [from() - Convert from Array, Promise, etc.](/en/guide/creation-functions/basic/from)
- [Combination Creation Functions](/en/guide/creation-functions/combination/)
- [Return to Basic Creation Functions](/en/guide/creation-functions/basic/)
