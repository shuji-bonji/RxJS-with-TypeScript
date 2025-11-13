---
description: The scan operator is an RxJS operator that outputs intermediate results while accumulating each value sequentially, and is used for real-time aggregation and state management.
---

# scan - Generates Values Cumulatively

The `scan` operator applies a cumulative function to each value in the stream and outputs **sequential intermediate results**.
Similar to `Array.prototype.reduce` for arrays, except that the intermediate result is output sequentially before all values are reached.

## 🔰 Basic Syntax and Usage

```ts
import { of } from 'rxjs';
import { scan } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(scan((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Output: 1, 3, 6, 10, 15

```

- `acc` is the cumulative value, `curr` is the current value.
- It starts with an initial value (`0` in this case) and accumulates sequentially.

[🌐 RxJS Official Documentation - `scan`](https://rxjs.dev/api/operators/scan)

## 💡 Typical Usage Patterns

- Counting up and score aggregation
- Real-time form validation status management
- Cumulative processing of buffered events
- Data construction for real-time aggregate graphs

## 🧠 Practical Code Example (with UI)

Display the cumulative number of clicks each time a button is clicked.

```ts
import { fromEvent } from 'rxjs';
import { scan, tap } from 'rxjs';

// Create button
const button = document.createElement('button');
button.textContent = 'Click';
document.body.appendChild(button);

// Create output area
const counter = document.createElement('div');
counter.style.marginTop = '10px';
document.body.appendChild(counter);

// Accumulate click events
fromEvent(button, 'click')
  .pipe(
    tap((v) => console.log(v)),
    scan((count) => count + 1, 0)
  )
  .subscribe((count) => {
    counter.textContent = `Click count: ${count}`;
  });
```

- Each time a button is clicked, the counter is incremented by 1.
- By using `scan`, you can write **simple counting logic without state management**.
