---
description: "combineLatestAll is an operator that takes a Higher-order Observable (Observable of Observables) and combines the latest value of each with the output once all internal Observables have fired at least once."
---

# combineLatestAll - combine the latest value of the internal Observable

The `combineLatestAll` operator takes a **Higher-order Observable** (Observable of Observables),
**Once all internal Observables have fired at least once**, combine their **latest values** and output them as an **array**.

## 🔰 Basic Syntax and Usage

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// 3The two internalObservablewithHigher-order Observable
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// All internalsObservablehave at least1fires once, combine the latest value
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Output:
// [1, 3, 0] ← When all have fired at least1When all have fired at least once (after2seconds after)
// [2, 3, 0] ← 1The secondObservableis fired (after 1.5 seconds)2fires (after3seconds after)
// [2, 3, 1] ← 3The secondObservableis fired (after 1.5 seconds)1fires (after4seconds after)
```

- Collect internal Observables when Higher-order Observable is **complete**.
- **Start combining when all internal Observables have fired** at least once
- Whenever any internal Observable issues a value, **combine** all the latest values and **output

[🌐 RxJS Official Documentation - `combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 Typical utilization pattern

- **combine the latest results of multiple API calls**
- **Synchronize the latest values of multiple form inputs**
- **Integrate multiple real-time data sources**

## 🧠 Practical Code Examples

Example of combining the latest results of multiple API calls

```ts
import { of, timer, Observable } from 'rxjs';
import { map, combineLatestAll, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// 3Two simulatedAPICreate a call (Higher-order ObservableCreate a call ()
const apiCalls$: Observable<Observable<string>> = of(
  // API 1: User information (1Completed in seconds,3updated once)
  timer(0, 1000).pipe(
    take(3),
    map(n => `User: User${n}`)
  ),
  // API 2: Number of notifications (0.5Completed in seconds,4updated once)
  timer(0, 500).pipe(
    take(4),
    map(n => `Notifications: ${n}Notifications`)
  ),
  // API 3: Status2Completed in seconds,2updated once)
  timer(0, 2000).pipe(
    take(2),
    map(n => n === 0 ? 'Status: Offline' : 'Status: Online')
  )
);

// AllAPICombined latest value of calls
apiCalls$
  .pipe(combineLatestAll())
  .subscribe(values => {
    output.innerHTML = '<strong>Latest status:</strong><br>';
    values.forEach((value, index) => {
      const item = document.createElement('div');
      item.textContent = `${index + 1}. ${value}`;
      output.appendChild(item);
    });
  });
```

- Three API calls are executed in parallel
- After **all** have fired at least once, the combined results will be displayed
- Whenever any API is updated, the **latest combination** is displayed

## 🔄 Related Creation Functions

While `combineLatestAll` is primarily used for flattening Higher-order Observables,
For normal combinations of multiple Observables, use the **Creation Function** `combineLatest`.

```ts
import { combineLatest, interval } from 'rxjs';

// Creation FunctionEdition (more common usage)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

See [Chapter 3 Creation Functions - combineLatest](/en/guide/creation-functions/combination/combineLatest).

## 🔄 Related Operators

| Operator | Description. |
|---|---|
| [mergeAll](. /mergeAll) | Subscribe all internal Observables in parallel |
| [concatAll](. /concatAll) | Subscribe to internal Observables in order |
| [switchAll](. /switchAll) | Switch to a new internal Observable |
| [zipAll](. /zipAll) | Pair the values of each internal Observable in the corresponding order |

## ⚠️ Notes.

### Higher-order Observable must be completed

`combineLatestAll` waits to collect the internal Observable until the Higher-order Observable (outer Observable) is **completed**.

#### ❌ Higher-order Observable does not complete, so nothing is output

```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // Nothing is output
```

#### ✅ Take to complete

### All internal Observables must fire at least once

No value will be output until all internal Observables have fired **at least once**.

```ts
// 1If any of the internalObservableNothing is output if there are
of(
  of(1, 2, 3),
  NEVER // Never fires
).pipe(
  combineLatestAll()
).subscribe(console.log); // Nothing is output
```

### memory usage

Note the memory usage if there are many internal Observables, since the **latest values of all internal Observables are kept in memory**.
