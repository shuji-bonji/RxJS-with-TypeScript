---
description: "merge Creation Function subscribes to multiple Observables simultaneously and merges values in real-time: Essential for integrating parallel event streams"
---

# merge - merge multiple streams simultaneously

`merge` is a Creation Function that subscribes to multiple Observables simultaneously,
Creation Function that subscribes to multiple Observables simultaneously and outputs the values as they are issued from each Observable.

## Basic syntax and usage

```ts
import { merge, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Stream 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Stream 2: ${val}`),
  take(2)
);

merge(source1$, source2$).subscribe(console.log);
// Example output:
// Stream 1: 0
// Stream 2: 0
// Stream 1: 1
// Stream 1: 2
// Stream 2: 1
```

- Subscribe to all Observables simultaneously, and values will flow in **order of issuance**.
- There is no guarantee of order, and it **depends** on the timing of each Observable's issuance.


[🌐 RxJS Official Documentation - `merge`](https://rxjs.dev/api/index/function/merge)

## Typical utilization patterns

- **Merge** multiple asynchronous events (e.g., user input and backend notifications)
- **Aggregate multiple data streams into a single stream**.
- **Combine parallel sources of information, e.g., real-time updates and polling integration**.

## Practical code examples (with UI)

Combines click and timer events in real time.

```ts
import { merge, fromEvent, timer } from 'rxjs';
import { map } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>merge practical example:</h3>';
document.body.appendChild(output);

// Create button element
const button = document.createElement('button');
button.textContent = 'Click to fire event';
document.body.appendChild(button);

// Click stream
const click$ = fromEvent(button, 'click').pipe(
  map(() => '✅ Button click detected')
);

// Timer stream
const timer$ = timer(3000, 3000).pipe(
  map((val) => `⏰ Timer event (${val})`)
);

// merge and display
merge(click$, timer$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- **Click on a button and an event is generated** immediately,
- **Timer fires a repeating event** every 3 seconds.
- You can experience how two different types of Observables can be merged in **real time**.


## Related Operators

- **[mergeWith](/en/guide/operators/combination/mergeWith)** - Pipeable Operator version (used in pipeline)
- **[mergeMap](/en/guide/operators/transformation/mergeMap)** - map and concatenate each value in parallel
- **[concat](/en/guide/creation-functions/combination/concat)** - Sequential concatenation Creation Function
