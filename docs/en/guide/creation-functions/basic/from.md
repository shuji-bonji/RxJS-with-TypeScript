---
description: "from() - Creation Function that converts arrays, Promises, iterables, and Observable-like objects to Observable streams with seamless async integration"
---

# from() - Convert from Array, Promise, etc.

`from()` is a Creation Function that creates an Observable from arrays, Promises, iterables, and Observable-like objects.

## Overview

`from()` converts existing data structures (arrays, Promises, iterables, etc.) into Observable streams. In particular, it is frequently used to integrate asynchronous processing (Promise) into the RxJS world.

**Signature**:
```typescript
function from<T>(input: ObservableInput<T>, scheduler?: SchedulerLike): Observable<T>
```

**Official Documentation**: [📘 RxJS Official: from()](https://rxjs.dev/api/index/function/from)

## Basic Usage

`from()` accepts a variety of input types.

```typescript
import { from } from 'rxjs';

// Create from array
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Array value:', value),
  complete: () => console.log('Array complete')
});

// Create from Promise
const promise$ = from(Promise.resolve('Promise result'));
promise$.subscribe({
  next: value => console.log('Promise result:', value),
  complete: () => console.log('Promise complete')
});

// Create from iterable
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Iterable value:', value),
  complete: () => console.log('Iterable complete')
});

// Output:
// Array value: 1
// Array value: 2
// Array value: 3
// Array complete
// Iterable value: 1
// Iterable value: 2
// Iterable value: 3
// Iterable complete
// Promise result: Promise result
// Promise complete
```

## Important Characteristics

### 1. Emit Each Element of Array Individually

When `from()` receives an array, it emits each element of the array individually in sequence.

```typescript
import { from } from 'rxjs';

from([10, 20, 30]).subscribe(value => console.log(value));

// Output:
// 10
// 20
// 30
```

> [!IMPORTANT]
> **Difference from `of()`**:
> - `of([1, 2, 3])` → Emits the array itself as a single value
> - `from([1, 2, 3])` → Emits each element `1`, `2`, `3` separately

### 2. Automatically Process Promise

Passing a Promise will emit the resolved value and complete immediately.

```typescript
import { from } from 'rxjs';

const fetchData = (): Promise<string> => {
  return new Promise(resolve => {
    setTimeout(() => resolve('Data fetch complete'), 1000);
  });
};

from(fetchData()).subscribe({
  next: value => console.log(value),
  complete: () => console.log('Complete')
});

// Output after 1 second:
// Data fetch complete
// Complete
```

> [!WARNING]
> If the Promise is rejected, Observable emits an error.
> ```typescript
> import { from } from "rxjs";
> from(Promise.reject('Error')).subscribe({
>   error: err => console.error('Error occurred:', err)
> });
> ```

### 3. Support for Iterables

In addition to arrays, it supports iterable objects such as `Set`, `Map`, and `Generator`.

```typescript
import { from } from 'rxjs';

// Set
from(new Set(['A', 'B', 'C'])).subscribe(console.log);
// Output: A, B, C

// Map (key-value pairs)
from(new Map([['key1', 'value1'], ['key2', 'value2']])).subscribe(console.log);
// Output: ['key1', 'value1'], ['key2', 'value2']

// Generator
function* numberGenerator() {
  yield 1;
  yield 2;
  yield 3;
}
from(numberGenerator()).subscribe(console.log);
// Output: 1, 2, 3
```

### 4. Cold Observable

`from()` is a **Cold Observable**. Each subscription initiates an independent execution.

```typescript
import { from } from 'rxjs';

const numbers$ = from([1, 2, 3]);

numbers$.subscribe(val => console.log('Subscriber A:', val));
numbers$.subscribe(val => console.log('Subscriber B:', val));

// Each subscriber processes the array independently
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
> - Promises are also evaluated per subscription
>
> See [Cold Observable and Hot Observable](/en/guide/observables/cold-and-hot-observables) for more information.

## Difference Between from() and of()

The most important difference between the two is the way arrays are handled.

```typescript
import { from, of } from 'rxjs';

const array = [1, 2, 3];

// of() - emits the array as a single value
of(array).subscribe(value => {
  console.log('of():', value); // [1, 2, 3]
});

// from() - emits each element of the array individually
from(array).subscribe(value => {
  console.log('from():', value); // 1, 2, 3
});
```

| Creation Function | Array Handling | Purpose |
|-------------------|-----------|------|
| `of([1, 2, 3])` | Emits the array itself | Want to treat array as data |
| `from([1, 2, 3])` | Emits each element individually | Want to process array elements one by one |

## Practical Use Cases

### 1. Stream API Calls

Stream Promise-based HTTP clients such as Fetch API and axios.

```typescript
import { from, Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
}

function fetchUser(id: number): Observable<User> {
  return from(
    fetch(`https://jsonplaceholder.typicode.com/users/${id}`)
      .then(response => response.json())
  ).pipe(
    catchError(error => {
      console.error('API Error:', error);
      return of({ id: 0, name: 'Unknown', email: '' });
    })
  );
}

fetchUser(1).subscribe(user => console.log('User:', user));
```

### 2. Sequential Processing of Array Elements

Execute asynchronous processing sequentially for each element of the array.

```typescript
import { from } from 'rxjs';
import { concatMap, delay } from 'rxjs';

const urls = [
  'https://jsonplaceholder.typicode.com/posts/1',
  'https://jsonplaceholder.typicode.com/posts/2',
  'https://jsonplaceholder.typicode.com/posts/3'
];

from(urls).pipe(
  concatMap(url =>
    from(fetch(url).then(res => res.json())).pipe(
      delay(500) // Rate limiting
    )
  )
).subscribe(data => console.log('Fetched:', data));
```

### 3. Asynchronous Iterator Processing

Asynchronous iterators (async generators) are also supported.

```typescript
import { from } from 'rxjs';

async function* asyncGenerator() {
  yield await Promise.resolve(1);
  yield await Promise.resolve(2);
  yield await Promise.resolve(3);
}

from(asyncGenerator()).subscribe(value => console.log(value));
// Output: 1, 2, 3
```

### 4. Event Emitter Integration

Stream Node.js EventEmitter and custom event systems.

```typescript
import { from } from 'rxjs';

// Iterable custom object
class DataSource {
  *[Symbol.iterator]() {
    yield 'Data A';
    yield 'Data B';
    yield 'Data C';
  }
}

from(new DataSource()).subscribe(console.log);
// Output: Data A, Data B, Data C
```

## Using in Pipeline

`from()` is useful when using existing data as a starting point for pipeline processing.

```typescript
import { from } from 'rxjs';
import { map, filter, reduce } from 'rxjs';

interface Product {
  id: number;
  name: string;
  price: number;
}

const products: Product[] = [
  { id: 1, name: 'Product A', price: 1000 },
  { id: 2, name: 'Product B', price: 2000 },
  { id: 3, name: 'Product C', price: 500 }
];

from(products).pipe(
  filter(product => product.price >= 1000),
  map(product => product.price),
  reduce((sum, price) => sum + price, 0)
).subscribe(total => console.log('Total amount:', total));
// Output: Total amount: 3000
```

## Common Mistakes

### 1. Misunderstanding the Timing of Promise Execution

```typescript
// ❌ Wrong - Promise starts executing at creation time
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1'); // Already started
from(promise).subscribe(console.log); // Not at subscription time

// ✅ Correct - use defer() if you want to execute at subscription time
import { defer, from } from 'rxjs';

const deferred$ = defer(() =>
  from(fetch('https://jsonplaceholder.typicode.com/posts/1'))
);
deferred$.subscribe(console.log); // Executes at subscription time
```

> [!WARNING]
> **Promise is Not Lazily Evaluated**
>
> Promise starts executing when it is created. `from(promise)` only wraps a Promise that is already running. If you want to execute at subscription time, use `defer(() => from(promise))`.

### 2. Confusing Array with of()

```typescript
import { from, map, of } from "rxjs";

// ❌ Different from intention - entire array is emitted
of([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Output: [1, 2, 3] (the array itself)

// ✅ Correct - process each element individually
from([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Output: 2, 4, 6
```

## Performance Considerations

The performance of `from()` depends on the input type.

> [!TIP]
> **Optimization Tips**:
> - When processing large amounts of data (thousands to tens of thousands of elements), limit the number of concurrent operations when combining with `concatMap` and `mergeMap`.
> - When processing Promise arrays, consider using `forkJoin` or `combineLatest`.

```typescript
import { from } from 'rxjs';
import { mergeMap } from 'rxjs';

const urls = [...Array(100)].map((_, i) => `https://jsonplaceholder.typicode.com/posts/${i + 1}`);

from(urls).pipe(
  mergeMap(
    url => from(fetch(url).then(res => res.json())),
    5 // Limit concurrent execution to 5
  )
).subscribe(data => console.log(data));
```

## Related Creation Functions

| Function | Difference | Usage |
|----------|------|----------|
| **[of()](/en/guide/creation-functions/basic/of)** | Emit arguments in sequence | Want to emit values as-is |
| **[fromEvent()](/en/guide/creation-functions/basic/fromEvent)** | Stream events | Handle DOM events or EventEmitter |
| **[defer()](/en/guide/creation-functions/conditional/defer)** | Defer generation until subscription | Need lazy Promise execution |
| **ajax()** | Dedicated to HTTP communication | Want to complete HTTP requests within RxJS |

## Summary

- `from()` creates Observable from arrays, Promises, and iterables
- Emits each element of an array separately (different from `of()`)
- Automatically processes Promise and emits the result
- Ideal for integrating asynchronous processing into the RxJS world
- Note that Promise is executed at the point of creation (use `defer()` for lazy execution)

## Next Steps

- [fromEvent() - Convert Events to Observable](/en/guide/creation-functions/basic/fromEvent)
- [defer() - Defer Generation Until Subscription](/en/guide/creation-functions/conditional/defer)
- [Return to Basic Creation Functions](/en/guide/creation-functions/basic/)
