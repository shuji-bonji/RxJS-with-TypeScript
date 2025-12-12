---
description: concatAll is an operator that takes a Higher-order Observable (Observable of Observables) and flattens values by subscribing to internal Observables in order. It starts the next one after the previous Observable completes.
titleTemplate: ':title | RxJS'
---

# concatAll - Flatten Internal Observables Sequentially

The `concatAll` operator takes a **Higher-order Observable** (Observable of Observables),
**subscribes to internal Observables in order**, and flattens their values. It will not start the next until the previous Observable completes.

## 🔰 Basic Syntax and Usage

```ts
import { fromEvent, interval } from 'rxjs';
import { map, concatAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Start a new counter for each click (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Subscribe to counters in order (start next after previous completes)
higherOrder$
  .pipe(concatAll())
  .subscribe(x => console.log(x));

// Output (with 3 clicks):
// 0 (1st counter)
// 1 (1st counter)
// 2 (1st counter) ← Complete
// 0 (2nd counter) ← Start after 1st completes
// 1 (2nd counter)
// 2 (2nd counter) ← Complete
// 0 (3rd counter) ← Start after 2nd completes
// ...
```

- **Subscribe in order** to each internal Observable emitted from Higher-order Observable
- **Do not start next** until previous internal Observable completes
- Order of values is guaranteed

[🌐 RxJS Official Documentation - `concatAll`](https://rxjs.dev/api/index/function/concatAll)

## 💡 Typical Usage Patterns

- **Execute API calls in sequence (execute next after previous request completes)**
- **Play animations in order**
- **Process file uploads sequentially**

## 🧠 Practical Code Example

Example of executing API calls (simulated) in order for each button click

```ts
import { fromEvent, of } from 'rxjs';
import { map, concatAll, delay } from 'rxjs';

const button = document.createElement('button');
button.textContent = 'API Call';
document.body.appendChild(button);

const output = document.createElement('div');
document.body.appendChild(output);

let callCount = 0;

// Button click event
const clicks$ = fromEvent(button, 'click');

// Higher-order Observable: Simulated API call for each click
const results$ = clicks$.pipe(
  map(() => {
    const id = ++callCount;
    const start = Date.now();

    // Simulated API call (2 second delay)
    return of(`API call #${id} completed`).pipe(
      delay(2000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed} seconds)`;
      })
    );
  }),
  concatAll() // Execute all API calls in order
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- Even with consecutive button clicks, **API calls are executed in order**
- Next API call starts after previous one completes

## 🔄 Related Operators

| Operator | Description |
|---|---|
| `concatMap` | Shorthand for `map` + `concatAll` (commonly used) |
| [mergeAll](/pt/guide/operators/combination/mergeAll) | Subscribe to all internal Observables in parallel |
| [switchAll](/pt/guide/operators/combination/switchAll) | Switch to new internal Observable (cancel old one) |
| [exhaustAll](/pt/guide/operators/combination/exhaustAll) | Ignore new internal Observables while executing |

## ⚠️ Important Notes

### Backpressure (Queue Buildup)

If internal Observable emission rate is faster than completion rate, **unprocessed Observables will accumulate in the queue**.

```ts
// Click every second → API call takes 2 seconds
// → Queue may accumulate continuously
```

In this case, consider these countermeasures:
- Use `switchAll` (process only latest)
- Use `exhaustAll` (ignore during execution)
- Add debounce or throttling

### Beware of Infinite Observables

If previous Observable **never completes, next will never start**.

#### ❌ interval never completes, so 2nd counter never starts
```ts
clicks$.pipe(
  map(() => interval(1000)), // Never completes
  concatAll()
).subscribe();
```
#### ✅ Complete with take
```ts
clicks$.pipe(
  map(() => interval(1000).pipe(take(3))), // Completes after 3
  concatAll()
).subscribe();
```
