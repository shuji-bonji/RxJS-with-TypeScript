---
description: mergeAll is an operator that takes a Higher-order Observable (Observable of Observables) and subscribes to all internal Observables in parallel to flatten the values.
---

# mergeAll - Flatten All Internal Observables in Parallel

The `mergeAll` operator takes a **Higher-order Observable** (Observable of Observables),
**subscribes to all internal Observables in parallel**, and flattens their values.

## 🔰 Basic Syntax and Usage

```ts
import { fromEvent, interval } from 'rxjs';
import { map, mergeAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Start a new counter for each click (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Subscribe to all counters in parallel
higherOrder$
  .pipe(mergeAll())
  .subscribe(x => console.log(x));

// Output (with 3 clicks):
// 0 (1st counter)
// 1 (1st counter)
// 0 (2nd counter) ← parallel execution
// 2 (1st counter)
// 1 (2nd counter)
// 0 (3rd counter) ← parallel execution
// ...
```

- **Subscribe in parallel** to each internal Observable emitted from Higher-order Observable
- **Combine values** from all internal Observables into a **single stream**
- Can limit number of concurrent subscriptions (`mergeAll(2)` = up to 2 concurrent)

[🌐 RxJS Official Documentation - `mergeAll`](https://rxjs.dev/api/index/function/mergeAll)

## 💡 Typical Usage Patterns

- **Execute multiple API calls in parallel**
- **Start independent streams for each user action**
- **Integrate multiple real-time connections such as WebSocket and EventSource**

## 🧠 Practical Code Example

Example of executing concurrent API calls (simulated) on each input change

```ts
import { fromEvent, of } from 'rxjs';
import { map, mergeAll, delay, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Enter search keywords';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

// Debounce input events
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: Simulated API call for each input value
const results$ = search$.pipe(
  map(query =>
    // Simulated API call (500ms delay)
    of(`Result: "${query}"`).pipe(delay(500))
  ),
  mergeAll() // Execute all API calls in parallel
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- **All API calls are executed in parallel**, even if user quickly changes input
- Old search results may appear after new results (no ordering guarantee)

## 🔄 Related Operators

| Operator | Description |
|---|---|
| `mergeMap` | Shorthand for `map` + `mergeAll` (commonly used) |
| [concatAll](/en/guide/operators/combination/concatAll) | Subscribe to internal Observables in order (wait for previous completion) |
| [switchAll](/en/guide/operators/combination/switchAll) | Switch to new internal Observable (cancel old one) |
| [exhaustAll](/en/guide/operators/combination/exhaustAll) | Ignore new internal Observables while executing |

## ⚠️ Important Notes

### Limiting Concurrent Subscriptions

Failure to limit concurrent subscriptions may result in performance problems.

```ts
// Limit concurrent subscriptions to 2
higherOrder$.pipe(
  mergeAll(2) // Up to 2 concurrent executions
).subscribe();
```

### No Order Guarantee

Because `mergeAll` executes concurrently, **ordering of values is not guaranteed**.
If ordering is critical, use [concatAll](/en/guide/operators/combination/concatAll).
