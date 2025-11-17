---
description: windowCount is an RxJS conversion operator that divides an Observable by a specified number of items. It is ideal for count-based stream processing, aggregation per fixed count, and pagination processing. Unlike bufferCount, it can apply independent processing to each window. TypeScript type inference allows type-safe window splitting and stream operations.
titleTemplate: ':title | RxJS'
---

# windowCount - Split Observable by Specified Count

The `windowCount` operator **divides** emitted values into new Observables for each specified count.
While `bufferCount` returns an array, `windowCount` returns an **Observable&lt;T&gt;**, allowing additional operators to be applied to each window.

## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { windowCount, mergeAll } from 'rxjs';

// Emit values every 100ms
const source$ = interval(100);

source$.pipe(
  windowCount(5),
  mergeAll() // Flatten each window
).subscribe(value => {
  console.log('Value in window:', value);
});

// Output:
// Value in window: 0
// Value in window: 1
// Value in window: 2
// Value in window: 3
// Value in window: 4
// (New window starts)
// Value in window: 5
// ...
```

- A new window (Observable) is created for every 5 values.
- It is unique in that it divides on a count basis.

[🌐 RxJS Official Documentation - `windowCount`](https://rxjs.dev/api/operators/windowCount)

## 💡 Typical Usage Patterns

- Aggregate processing for each fixed count
- Batch transmission of data (different processing for each window)
- Pagination processing
- Calculate statistics per window

## 🔍 Difference from bufferCount

| Operator | Output | Use Case |
|:---|:---|:---|
| `bufferCount` | **Array (T[])** | Process grouped values together |
| `windowCount` | **Observable&lt;T&gt;** | Different stream processing for each group |

```ts
import { interval } from 'rxjs';
import { bufferCount, windowCount, mergeAll } from 'rxjs';

const source$ = interval(100);

// bufferCount - Output as array
source$.pipe(
  bufferCount(5)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Output: Buffer (array): [0, 1, 2, 3, 4]
});

// windowCount - Output as Observable
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  console.log('Window (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Value in window:', value);
  });
});
```

## 🧠 Practical Code Example 1: Sum Per Window

This is an example of calculating the sum of every 5 values.

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, reduce } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>Sum Every 5 Values</h3>';
document.body.appendChild(output);

const source$ = interval(200);

let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;
    console.log(`Window ${current} started`);

    // Calculate sum for each window
    return window$.pipe(
      reduce((sum, value) => sum + value, 0),
      map(sum => ({ windowNum: current, sum }))  // Include window number
    );
  }),
  mergeAll()
).subscribe(result => {
  const div = document.createElement('div');
  div.textContent = `Window ${result.windowNum} sum: ${result.sum}`;
  output.appendChild(div);
});

// Output:
// Window 1 sum: 10  (0+1+2+3+4)
// Window 2 sum: 35  (5+6+7+8+9)
// Window 3 sum: 60  (10+11+12+13+14)
```

## 🎯 Practical Code Example 2: Specifying Start Index

You can specify a starting index with the second argument. This creates overlap windows.

```ts
import { range } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// Emit values from 0 to 9
range(0, 10).pipe(
  windowCount(3, 2), // 3 items each, start shifted by 2
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Window:', values);
});

// Output:
// Window: [0, 1, 2]
// Window: [2, 3, 4]    ← Started shifted by 2 (from 2)
// Window: [4, 5, 6]    ← Started shifted by 2 (from 4)
// Window: [6, 7, 8]
// Window: [8, 9]       ← Last 2 items
```

### Start Index Operation Patterns

```ts
// windowCount(bufferSize, startBufferEvery)

windowCount(3, 3) // Continuous (default): [0,1,2], [3,4,5], [6,7,8]
windowCount(3, 2) // Overlap: [0,1,2], [2,3,4], [4,5,6]
windowCount(3, 4) // With gap: [0,1,2], [4,5,6], [8,9,10]
```

## 🎯 Practical Example: Different Processing for Each Window

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, take } from 'rxjs';

const source$ = interval(100);
let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Even windows: Get only first 2 items
      console.log(`Window ${current}: Get first 2 items`);
      return window$.pipe(take(2));
    } else {
      // Odd windows: Get all
      console.log(`Window ${current}: Get all`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Value: ${value} (Window ${windowNumber})`);
});
```

## 🧠 Practical Code Example 3: Pagination-like Processing

```ts
import { from } from 'rxjs';
import { windowCount, mergeMap, toArray, map } from 'rxjs';

// Data from 1-20
const data$ = from(Array.from({ length: 20 }, (_, i) => i + 1));

// Paginate by 5 items
data$.pipe(
  windowCount(5),
  mergeMap((window$, index) => {
    const pageNumber = index + 1;
    return window$.pipe(
      toArray(),
      map(items => ({ page: pageNumber, items }))
    );
  })
).subscribe(page => {
  console.log(`Page ${page.page}:`, page.items);
});

// Output:
// Page 1: [1, 2, 3, 4, 5]
// Page 2: [6, 7, 8, 9, 10]
// Page 3: [11, 12, 13, 14, 15]
// Page 4: [16, 17, 18, 19, 20]
```

## ⚠️ Notes

### 1. Window Subscription Management

Each window is an independent Observable and must be explicitly subscribed to.

```ts
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  // Values won't flow unless you subscribe to the window itself
  window$.subscribe(value => {
    console.log('Value:', value);
  });
});
```

Or use `mergeAll()`, `concatAll()`, `switchAll()`, etc. to flatten.

### 2. Last Window

On completion of the source Observable, the last window is output even if it contains fewer than the specified number of items.

```ts
import { of } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

of(1, 2, 3, 4, 5, 6, 7).pipe(
  windowCount(3),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Window:', values);
});

// Output:
// Window: [1, 2, 3]
// Window: [4, 5, 6]
// Window: [7]  ← Only 1 item
```

### 3. Memory Usage by Start Index

If `startBufferEvery` is less than `bufferSize` (overlap), multiple windows will be active at the same time, increasing memory usage.

```ts
// Overlap: Maximum 2 windows active simultaneously
windowCount(5, 3)

// Countermeasure: Limit with take() if necessary
source$.pipe(
  take(100), // Maximum 100 items
  windowCount(5, 3)
)
```

## 🆚 Comparison of window Operators

| Operator | Timing of Delimiter | Use Case |
|:---|:---|:---|
| `window` | Another Observable emits | Event-driven partitioning |
| `windowTime` | Fixed time interval | Time-based partitioning |
| `windowCount` | **Fixed count** | **Count-based partitioning** |
| `windowToggle` | Start and end Observables | Dynamic start/end control |
| `windowWhen` | Dynamic closing condition | Different end condition per window |

## 📚 Related Operators

- [`bufferCount`](/en/guide/operators/transformation/bufferCount) - Collect values as array (array version of windowCount)
- [`window`](/en/guide/operators/transformation/window) - Split window at different Observable timings
- [`windowTime`](/en/guide/operators/transformation/windowTime) - Time-based window splitting
- [`windowToggle`](/en/guide/operators/transformation/windowToggle) - Window control with start and end Observables
- [`windowWhen`](/en/guide/operators/transformation/windowWhen) - Window splitting with dynamic closing conditions

## Summary

The `windowCount` operator is a useful tool for partitioning streams on a count basis and treating each group as an independent Observable.

- ✅ Ideal for aggregation and processing by a fixed count
- ✅ Different processing can be applied to each window
- ✅ Can be overlapped by start index
- ⚠️ Requires subscription management
- ⚠️ Be aware of memory usage when overlapping
