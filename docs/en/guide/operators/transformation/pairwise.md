---
description: The pairwise operator is an RxJS operator that outputs two consecutive values as an array of pairs, and is utilized to compare the previous value with the current value or to calculate the difference.
titleTemplate: ':title | RxJS'
---

# pairwise - Process Two Consecutive Values as a Pair

The `pairwise` operator **pairs together two consecutive values issued from a stream as an array `[previous value, current value]` and outputs them together**.
This is useful for comparing the previous value with the current value or for calculating the amount of change.

## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { pairwise, take } from 'rxjs';

interval(1000).pipe(
  take(6),
  pairwise()
).subscribe(console.log);

// Output:
// [0, 1]
// [1, 2]
// [2, 3]
// [3, 4]
// [4, 5]
```

- The first value (0) is not output by itself, but is output as `[0, 1]` when the second value (1) arrives.
- Always a pair of **previous value and current value** is output.

[🌐 RxJS Official Documentation - `pairwise`](https://rxjs.dev/api/operators/pairwise)

## 💡 Typical Usage Patterns

- Calculation of the amount of mouse or touch movement
- Calculation of the amount of change (difference) in prices or values
- State change detection (comparison of previous state and current state)
- Determination of scroll direction

## 🧠 Practical Code Example (with UI)

This example displays the direction and amount of mouse movement.

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Mouse move event
fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY })),
  pairwise()
).subscribe(([prev, curr]) => {
  const deltaX = curr.x - prev.x;
  const deltaY = curr.y - prev.y;
  const direction = deltaX > 0 ? 'Right' : deltaX < 0 ? 'Left' : 'Stopped';

  output.innerHTML = `
    Previous: (${prev.x}, ${prev.y})<br>
    Current: (${curr.x}, ${curr.y})<br>
    Movement: Δx=${deltaX}, Δy=${deltaY}<br>
    Direction: ${direction}
  `;
});
```

- When the mouse is moved, the previous and current coordinates and the amount of movement are displayed.
- With `pairwise`, the previous and current coordinates can be automatically obtained in pairs.

## 🎯 Example of Calculating the Amount of Change in a Number

Here is a practical example of calculating the amount of change (difference) in a numerical value stream.

```ts
import { interval } from 'rxjs';
import { map, pairwise, take } from 'rxjs';

// 0, 1, 4, 9, 16, 25 (square numbers)
interval(500).pipe(
  take(6),
  map(n => n * n),
  pairwise(),
  map(([prev, curr]) => ({
    prev,
    curr,
    diff: curr - prev
  }))
).subscribe(result => {
  console.log(`${result.prev} → ${result.curr} (difference: +${result.diff})`);
});

// Output:
// 0 → 1 (difference: +1)
// 1 → 4 (difference: +3)
// 4 → 9 (difference: +5)
// 9 → 16 (difference: +7)
// 16 → 25 (difference: +9)
```

## 🎯 Determining Scroll Direction

The following is an example of determining the scroll direction (up/down).

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise, throttleTime } from 'rxjs';

// Create fixed display output area
const output = document.createElement('div');
output.style.position = 'fixed';
output.style.top = '10px';
output.style.right = '10px';
output.style.padding = '15px';
output.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
output.style.color = 'white';
output.style.fontFamily = 'monospace';
output.style.fontSize = '14px';
output.style.borderRadius = '5px';
output.style.zIndex = '9999';
document.body.appendChild(output);

// Dummy content for scrolling
const content = document.createElement('div');
content.style.height = '200vh'; // Double the page height
content.innerHTML = '<h1>Please scroll down</h1>';
document.body.appendChild(content);

// Get scroll position
fromEvent(window, 'scroll').pipe(
  throttleTime(100), // Throttle every 100ms
  map(() => window.scrollY),
  pairwise()
).subscribe(([prevY, currY]) => {
  const diff = currY - prevY;
  const direction = diff > 0 ? '↓ Down' : '↑ Up';
  const arrow = diff > 0 ? '⬇️' : '⬆️';

  output.innerHTML = `
    ${arrow} Scroll direction: ${direction}<br>
    Previous position: ${prevY.toFixed(0)}px<br>
    Current position: ${currY.toFixed(0)}px<br>
    Movement: ${Math.abs(diff).toFixed(0)}px
  `;
});
```

- As the page is scrolled, the direction and position information is displayed in a fixed area in the upper right corner.
- `pairwise` allows you to automatically get the previous and current scroll position in pairs.

## 🎯 Utilizing Type-Safe pairwise

This is an example of utilizing TypeScript's type inference.

```ts
import { from } from 'rxjs';
import { pairwise } from 'rxjs';

interface Stock {
  symbol: string;
  price: number;
  timestamp: number;
}

const stockPrices: Stock[] = [
  { symbol: 'AAPL', price: 150, timestamp: 1000 },
  { symbol: 'AAPL', price: 152, timestamp: 2000 },
  { symbol: 'AAPL', price: 148, timestamp: 3000 },
  { symbol: 'AAPL', price: 155, timestamp: 4000 },
];

from(stockPrices).pipe(
  pairwise()
).subscribe(([prev, curr]) => {
  const change = curr.price - prev.price;
  const changePercent = ((change / prev.price) * 100).toFixed(2);
  const trend = change > 0 ? '📈' : change < 0 ? '📉' : '➡️';

  console.log(
    `${curr.symbol}: $${prev.price} → $${curr.price} ` +
    `(${changePercent}%) ${trend}`
  );
});

// Output:
// AAPL: $150 → $152 (1.33%) 📈
// AAPL: $152 → $148 (-2.63%) 📉
// AAPL: $148 → $155 (4.73%) 📈
```

## 🔍 Comparison with bufferCount(2, 1)

`pairwise()` is equivalent to `bufferCount(2, 1)`.

```ts
import { of } from 'rxjs';
import { pairwise, bufferCount } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== pairwise ===');
source$.pipe(pairwise()).subscribe(console.log);
// Output: [1,2], [2,3], [3,4], [4,5]

console.log('=== bufferCount(2, 1) ===');
source$.pipe(bufferCount(2, 1)).subscribe(console.log);
// Output: [1,2], [2,3], [3,4], [4,5]
```

**Usage Differences**:
- `pairwise()`: Explicitly deals with pairs of two consecutive values, and the intent of the code is clear
- `bufferCount(2, 1)`: More flexible (can handle more than 3 window sizes)

## ⚠️ Notes

### The First Value is Not Output

Since `pairwise` does not output anything until the two values are aligned, the first value cannot be obtained alone.

```ts
import { of } from 'rxjs';
import { pairwise } from 'rxjs';

of(1).pipe(pairwise()).subscribe({
  next: console.log,
  complete: () => console.log('Completed')
});

// Output:
// Completed
// (No values are output)
```

**Countermeasure**: If you want to process the first value as well, add an initial value with `startWith`.

```ts
import { of } from 'rxjs';
import { startWith, pairwise } from 'rxjs';

of(10, 20, 30).pipe(
  startWith(0),
  pairwise()
).subscribe(console.log);

// Output:
// [0, 10]
// [10, 20]
// [20, 30]
```

### Memory Usage

Since `pairwise` always keeps only one previous value, it is memory efficient.

## 📚 Related Operators

- [`scan`](/en/guide/operators/transformation/scan) - More complex accumulation process
- [`bufferCount`](/en/guide/operators/transformation/bufferCount) - Summarize values for each specified number of items
- [`distinctUntilChanged`](/en/guide/operators/filtering/distinctUntilChanged) - Remove consecutive duplicate values
- [`startWith`](/en/guide/operators/utility/startWith) - Add initial value

## Summary

The `pairwise` operator outputs two consecutive values as `[previous value, current value]` pairs. This is very useful for **situations where a comparison of the previous value and the current value is needed**, such as tracking mouse movements, calculating price changes, and detecting state transitions. Note that the first value is not output until the second value arrives, but this can be handled by adding an initial value with `startWith`.
