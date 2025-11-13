---
description: switchAll is an operator that takes a Higher-order Observable (Observable of Observables), switches to a new internal Observable, and cancels the old one.
---

# switchAll - Switch to New Internal Observable

The `switchAll` operator takes a **Higher-order Observable** (Observable of Observables),
**switches each time a new internal Observable is emitted**, and cancels the old internal Observable.

## 🔰 Basic Syntax and Usage

```ts
import { fromEvent, interval } from 'rxjs';
import { map, switchAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Start a new counter for each click (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Switch to new counter (cancel old counter)
higherOrder$
  .pipe(switchAll())
  .subscribe(x => console.log(x));

// Output (with 3 clicks):
// 0 (1st counter)
// 1 (1st counter)
// ← Click here (1st cancelled)
// 0 (2nd counter) ← Switch to new counter
// ← Click here (2nd cancelled)
// 0 (3rd counter) ← Switch to new counter
// 1 (3rd counter)
// 2 (3rd counter)
```

- When a new internal Observable is emitted from Higher-order Observable, **immediately switches**
- Previous internal Observable is **automatically cancelled**
- Only the latest internal Observable is always executing

[🌐 RxJS Official Documentation - `switchAll`](https://rxjs.dev/api/index/function/switchAll)

## 💡 Typical Usage Patterns

- **Search functionality (cancel old searches on each input)**
- **Autocomplete**
- **Real-time data updates (switch to latest data source)**

## 🧠 Practical Code Example

Example of canceling old searches and executing only the latest search on each input

```ts
import { fromEvent, of } from 'rxjs';
import { map, switchAll, debounceTime, delay } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Enter search keywords';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

let searchCount = 0;

// Debounce input events
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: Simulated search API call for each input value
const results$ = search$.pipe(
  map(query => {
    const id = ++searchCount;
    const start = Date.now();

    // Simulated search API call (1 second delay)
    return of(`Search results: "${query}"`).pipe(
      delay(1000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `[Search #${id}] ${msg} (${elapsed} seconds)`;
      })
    );
  }),
  switchAll() // Cancel old search when new one starts
);

results$.subscribe(result => {
  output.innerHTML = ''; // Clear previous results
  const item = document.createElement('div');
  item.textContent = result;
  output.appendChild(item);
});
```

- **Old searches are automatically cancelled** when user changes input
- Only the latest search results are always displayed

## 🔄 Related Operators

| Operator | Description |
|---|---|
| `switchMap` | Shorthand for `map` + `switchAll` (most commonly used) |
| [mergeAll](/en/guide/operators/combination/mergeAll) | Subscribe to all internal Observables in parallel |
| [concatAll](/en/guide/operators/combination/concatAll) | Subscribe to internal Observables in order (wait for previous completion) |
| [exhaustAll](/en/guide/operators/combination/exhaustAll) | Ignore new internal Observables while executing |

## ⚠️ Important Notes

### Memory Leak Prevention

`switchAll` helps prevent memory leaks by **automatically canceling** old internal Observables.
It is ideal for frequent new requests such as searches or autocomplete.

### Non-completing Internal Observables

Even if internal Observable doesn't complete, it will automatically switch when a new internal Observable is emitted.

```ts
// interval never completes, but is automatically cancelled on next click
clicks$.pipe(
  map(() => interval(1000)), // Never completes
  switchAll()
).subscribe();
```

### Optimal When Only Last Value Matters

Use `switchAll` when you don't need results of old processing and **only the latest result is important**.
If all results are needed, use [mergeAll](/en/guide/operators/combination/mergeAll).
