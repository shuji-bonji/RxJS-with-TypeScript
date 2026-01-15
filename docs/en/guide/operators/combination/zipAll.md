---
description: zipAll is an operator that takes a Higher-order Observable (Observable of Observables) and pairs the corresponding ordered values of each internal Observable and outputs them as an array.
titleTemplate: ':title'
---

# zipAll - Pair Inner Values by Order

The `zipAll` operator takes a **Higher-order Observable** (Observable of Observables),
**pairs the corresponding ordered values of each internal Observable** and outputs them as an array.

## 🔰 Basic Syntax and Usage

```ts
import { interval, of } from 'rxjs';
import { zipAll, take } from 'rxjs';

// Higher-order Observable with three internal Observables
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Pair corresponding ordered values of each internal Observable
higherOrder$
  .pipe(zipAll())
  .subscribe(values => console.log(values));

// Output:
// [0, 0, 0] ← All 1st values
// [1, 1, 1] ← All 2nd values
// (Completes here: 3rd Observable only emits 2 values)
```

- Collects internal Observables when Higher-order Observable **completes**
- **Pairs the same index values** of each internal Observable
- **When the shortest internal Observable completes**, the whole completes

[🌐 RxJS Official Documentation - `zipAll`](https://rxjs.dev/api/index/function/zipAll)

## 💡 Typical Usage Patterns

- **Match multiple API responses in sequence**
- **Compare same timing values of multiple streams**
- **Combine parallel processing results in sequence**

## 🧠 Practical Code Example

Example of pairing the corresponding values of multiple counters

```ts
import { interval, of } from 'rxjs';
import { zipAll, take, map } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// Create three counters with different speeds
const counters$ = of(
  interval(1000).pipe(take(4), map(n => `Slow: ${n}`)),
  interval(500).pipe(take(5), map(n => `Normal: ${n}`)),
  interval(300).pipe(take(6), map(n => `Fast: ${n}`))
);

// Pair corresponding ordered values of each counter
counters$
  .pipe(zipAll())
  .subscribe(values => {
    const item = document.createElement('div');
    item.textContent = `[${values.join(', ')}]`;
    output.appendChild(item);
  });

// Output:
// [Slow: 0, Normal: 0, Fast: 0]
// [Slow: 1, Normal: 1, Fast: 1]
// [Slow: 2, Normal: 2, Fast: 2]
// [Slow: 3, Normal: 3, Fast: 3]
// (Completes here: "Slow" counter only emits 4 values)
```

## 🔄 Related Creation Function

While `zipAll` is mainly used for flattening Higher-order Observables,
use the **Creation Function** `zip` for normal pairing of multiple Observables.

```ts
import { zip, interval } from 'rxjs';
import { take } from 'rxjs';

// Creation Function version (more common usage)
const zipped$ = zip(
  interval(1000).pipe(take(3)),
  interval(500).pipe(take(4)),
  interval(2000).pipe(take(2))
);

zipped$.subscribe(console.log);
```

See [Chapter 3: Creation Functions - zip](/en/guide/creation-functions/combination/zip).

## 🔄 Related Operators

| Operator | Description |
|---|---|
| [combineLatestAll](/en/guide/operators/combination/combineLatestAll) | Combine latest values of all internal Observables |
| [mergeAll](/en/guide/operators/combination/mergeAll) | Subscribe to all internal Observables in parallel |
| [concatAll](/en/guide/operators/combination/concatAll) | Subscribe to internal Observables in order |
| [switchAll](/en/guide/operators/combination/switchAll) | Switch to new internal Observable |

## 🔄 zipAll vs combineLatestAll

| Operator | Combination Method | Completion Timing |
|---|---|---|
| `zipAll` | Pairs values at the **same index** | When the **shortest** internal Observable completes |
| `combineLatestAll` | Combines **latest values** | When **all** internal Observables complete |

```ts
// zipAll: [0th, 0th, 0th], [1st, 1st, 1st], ...
// combineLatestAll: [latest, latest, latest], [latest, latest, latest], ...
```

## ⚠️ Important Notes

### Higher-order Observable Must Complete

`zipAll` waits to collect internal Observables until the Higher-order Observable (outer Observable) **completes**.

#### ❌ Nothing output because Higher-order Observable doesn't complete
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log); // Nothing output
```

#### ✅ Complete with take
```ts
interval(1000).pipe(
  take(3), // Complete after 3
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log);
```

### Completes with Shortest Internal Observable

When the **shortest internal Observable completes**, the whole completes.

```ts
import { of, zipAll } from "rxjs";

of(
  of(1, 2, 3, 4, 5), // 5 values
  of(1, 2)           // 2 values ← Shortest
).pipe(
  zipAll()
).subscribe(console.log);

// Output: [1, 1], [2, 2]
// (Completes at 2. 3, 4, 5 are not used)
```

### Backpressure (Memory Usage)

When internal Observables emit at different speeds, **values from faster internal Observables accumulate in memory**.

```ts
import { interval, of, take, zipAll } from "rxjs";

// Fast counter (100ms) values accumulate in memory while waiting for slow counter (10000ms)
of(
  interval(10000).pipe(take(3)), // Slow
  interval(100).pipe(take(100))  // Fast
).pipe(
  zipAll()
).subscribe(console.log);
```

If the speed difference is large, pay attention to memory usage.
