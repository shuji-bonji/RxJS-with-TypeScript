---
description: partition is an RxJS Creation Function that splits one Observable into two Observables based on conditions. It is ideal for bifurcation processing such as success/failure, valid/invalid, etc.
---

# partition - split into two streams on condition

`partition` is a Creation Function that **divides** an Observable into two Observables based on a condition.
You can specify the condition with a predicate function (predicate) and get the values that satisfy the condition and the values that do not satisfy the condition as separate streams.

## Basic syntax and usage

```ts
import { partition, of } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Split into even and odd numbers
const [evens$, odds$] = partition(source$, (value) => value % 2 === 0);

evens$.subscribe((value) => console.log('Even:', value));
// Output: Even: 2, Even: 4, Even: 6

odds$.subscribe((value) => console.log('Odd:', value));
// Output: Odd: 1, Odd: 3, Odd: 5
```

- `partition` returns an **array containing two Observables**.
- `[0]`: a stream of values that satisfy the condition.
- `[1]`: a stream of values that do not satisfy the condition.

[🌐 RxJS Official Documentation - `partition`](https://rxjs.dev/api/index/function/partition)

## Typical utilization patterns

- **Split processing of success/failure** (sorting by HTTP status code)
- **Event classification** (left click/right click)
- **Data classification** (valid/invalid, adult/child, etc.)
- **Stream splitting based on conditions**.

## Practical code examples (with UI)

When a button is clicked, the process is branched depending on whether the click coordinates are for the left or right half of the screen.

```ts
import { partition, fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Create output area
const leftArea = document.createElement('div');
leftArea.innerHTML = '<h3>Left Click</h3><ul id="left-list"></ul>';
leftArea.style.float = 'left';
leftArea.style.width = '45%';
leftArea.style.padding = '10px';
leftArea.style.background = '#e3f2fd';
document.body.appendChild(leftArea);

const rightArea = document.createElement('div');
rightArea.innerHTML = '<h3>Right Click</h3><ul id="right-list"></ul>';
rightArea.style.float = 'right';
rightArea.style.width = '45%';
rightArea.style.padding = '10px';
rightArea.style.background = '#fce4ec';
document.body.appendChild(rightArea);

// Click events
const clicks$ = fromEvent<MouseEvent>(document, 'click');

// Center X coordinate of screen
const centerX = window.innerWidth / 2;

// Split into left and right halves
const [leftClicks$, rightClicks$] = partition(
  clicks$,
  (event) => event.clientX < centerX
);

// Process left clicks
leftClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const leftList = document.getElementById('left-list')!;
  const li = document.createElement('li');
  li.textContent = `Position: (${pos.x}, ${pos.y})`;
  leftList.appendChild(li);
});

// Process right clicks
rightClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const rightList = document.getElementById('right-list')!;
  const li = document.createElement('li');
  li.textContent = `Position: (${pos.x}, ${pos.y})`;
  rightList.appendChild(li);
});
```

- Clicking on the screen will be recorded in the left and right lists according to the click position.
- Two independent streams can be created from a single source.

## Practical example: Branch processing of API responses

Example of splitting success and failure by HTTP status code

```ts
import { partition, from, of } from 'rxjs';
import { mergeMap, map, catchError, share } from 'rxjs';

interface ApiResponse {
  status: number;
  data?: any;
  error?: string;
}

// Dummy API calls
const apiCalls$ = from([
  fetch('/api/users/1'),
  fetch('/api/users/999'), // Non-existent user
  fetch('/api/users/2'),
]);

// Process Response and convert to ApiResponse
const responses$ = apiCalls$.pipe(
  mergeMap(fetchPromise => from(fetchPromise)),
  mergeMap(response =>
    from(response.json()).pipe(
      map(data => ({
        status: response.status,
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : (data.message || 'Error')
      } as ApiResponse)),
      catchError(err => of({
        status: response.status,
        data: undefined,
        error: err.message || 'Failed to parse response'
      } as ApiResponse))
    )
  ),
  share() // Handle 2 subscriptions from partition
);

// Split into success (200s) and failure (others)
const [success$, failure$] = partition(
  responses$,
  (response: ApiResponse) => response.status >= 200 && response.status < 300
);

// Handle success responses
success$.subscribe((response) => {
  console.log('✅ Success:', response.data);
  // Display success data in UI
});

// Handle failure responses
failure$.subscribe((response) => {
  console.error('❌ Failure:', response.error);
  // Display error message
});
```

## Comparison with filter

### Basic differences

| Method | Description | Output | Use Case |
|--------|-------------|--------|----------|
| `partition` | Split one source into two streams | 2 Observables | When you want to use both streams **simultaneously** |
| `filter` | Only passes values that meet the condition | 1 Observable | When only one stream is needed |

### Examples of usage

**Use partition to process both streams simultaneously**

```ts
import { partition, interval } from 'rxjs';
import { map, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Success</h4><ul id="success-list"></ul>';
successArea.style.float = 'left';
successArea.style.width = '45%';
output.appendChild(successArea);

const failureArea = document.createElement('div');
failureArea.innerHTML = '<h4 style="color: red;">❌ Failure</h4><ul id="failure-list"></ul>';
failureArea.style.float = 'right';
failureArea.style.width = '45%';
output.appendChild(failureArea);

// Random success/failure stream
const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Task ${i + 1}`
  }))
);

// ✅ partition - handle success and failure simultaneously
const [success$, failure$] = partition(tasks$, task => task.success);

success$.subscribe(task => {
  const successList = document.getElementById('success-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  successList.appendChild(li);
});

failure$.subscribe(task => {
  const failureList = document.getElementById('failure-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  failureList.appendChild(li);
});
```

**Use filter if only one stream is needed**

```ts
import { interval } from 'rxjs';
import { map, take, filter } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Display success only</h4><ul id="success-only"></ul>';
output.appendChild(successArea);

const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Task ${i + 1}`
  }))
);

// ✅ filter - process only success (ignore failures)
tasks$
  .pipe(filter(task => task.success))
  .subscribe(task => {
    const successList = document.getElementById('success-only')!;
    const li = document.createElement('li');
    li.textContent = task.message;
    successList.appendChild(li);
  });
```

**Use filter twice vs. partition**

```ts
import { of } from 'rxjs';
import { filter } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// ❌ Use filter twice - source may be executed twice
const evens1$ = numbers$.pipe(filter(n => n % 2 === 0));
const odds1$ = numbers$.pipe(filter(n => n % 2 !== 0));

evens1$.subscribe(n => console.log('Even:', n));
odds1$.subscribe(n => console.log('Odd:', n));
// Problem: if numbers$ is a cold observable, it will be executed twice

// ✅ Use partition - create both streams in one execution
const [evens2$, odds2$] = partition(numbers$, n => n % 2 === 0);

evens2$.subscribe(n => console.log('Even:', n));
odds2$.subscribe(n => console.log('Odd:', n));
// Advantage: efficiently create two streams from one source
```

**Use filter if you want to branch in the pipeline**

```ts
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

interface User {
  id: number;
  name: string;
  age: number;
  isActive: boolean;
}

const users$ = from([
  { id: 1, name: 'Alice', age: 25, isActive: true },
  { id: 2, name: 'Bob', age: 30, isActive: false },
  { id: 3, name: 'Carol', age: 35, isActive: true }
]);

// ❌ partition is a Creation Function, so it can't be used in a pipeline
// users$.pipe(
//   map(user => user.name),
//   partition(name => name.startsWith('A')) // Error
// );

// ✅ Use filter - available in pipeline
users$
  .pipe(
    filter(user => user.isActive),  // Only active users
    map(user => user.name)           // Extract name
  )
  .subscribe(console.log);
// Output: Alice, Carol
```

### Summary

| Situation | Recommended Method | Reason |
|-----------|-------------------|--------|
| Want to process **both** success and failure | `partition` | Can create two streams in one source execution |
| Want to process **only** success | `filter` | Simple and clear |
| Want to branch conditions in pipeline | `filter` | `partition` cannot be used as it is a Creation Function |
| Want to branch into 3 or more with complex conditions | `groupBy` | Can split into multiple groups |

## Notes.

### 1. subscribe to both streams

The two Observables created in a `partition` **share** the original source.
If you do not subscribe to both, the original stream may not be fully processed.

```ts
const [success$, failure$] = partition(source$, predicate);

// Subscribe to both
success$.subscribe(handleSuccess);
failure$.subscribe(handleFailure);
```

### 2. the source is executed twice

The `partition` internally subscribes to the original source twice.
Be aware of any side effects.

```ts
let callCount = 0;
const source$ = new Observable(observer => {
  callCount++;
  console.log(`Subscription count: ${callCount}`);
  observer.next(1);
  observer.complete();
});

const [a$, b$] = partition(source$, n => n > 0);
a$.subscribe(); // Subscription count: 1
b$.subscribe(); // Subscription count: 2
```

To avoid side effects, use `share()`.

```ts
import { share } from 'rxjs';

const shared$ = source$.pipe(share());
const [a$, b$] = partition(shared$, n => n > 0);
```

### 3. Not provided as Pipeable Operator

Since RxJS 7, `partition` is provided as **Creation Function only**.
It cannot be used within a pipeline.

```ts
// ❌ Not possible
source$.pipe(
  partition(n => n % 2 === 0) // Error
);

// ✅ Correct usage
const [evens$, odds$] = partition(source$, n => n % 2 === 0);
```

## Alternative Patterns

If you want to branch within a pipeline, use `filter`.

```ts
const source$ = of(1, 2, 3, 4, 5, 6);

const evens$ = source$.pipe(filter(n => n % 2 === 0));
const odds$ = source$.pipe(filter(n => n % 2 !== 0));

// Or share the source with share
const shared$ = source$.pipe(share());
const evens$ = shared$.pipe(filter(n => n % 2 === 0));
const odds$ = shared$.pipe(filter(n => n % 2 !== 0));
```

## Related operators

- [`filter`](../../operators/filtering/filter.md) - passes only values that satisfy a condition
- [`groupBy`](../../operators/transformation/groupBy.md) - Split into multiple groups
- [`share`](../../operators/multicasting/share.md) - Share a source

## Summary

`partition` is a powerful tool to split one Observable into two based on a condition.

- ✅ Ideal for success/failure split processing
- ✅ Creates two independent streams
- ⚠️ Sources are subscribed to twice (note side effects)
- ⚠️ Not offered as Pipeable Operator
