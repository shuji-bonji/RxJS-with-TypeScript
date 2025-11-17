---
description: reduce is an RxJS conversion operator that accumulates all values in a stream and outputs only the final result upon completion. It is ideal for situations where only the final aggregation result is needed, such as calculating sums, averages, maximums, minimums, aggregating objects, and building arrays. Unlike scan, it does not output intermediate results and cannot be used with infinite streams because stream completion is required.
titleTemplate: ':title | RxJS'
---

# reduce - Outputs Only the Final Accumulated Result

The `reduce` operator applies a cumulative function to each value in the stream and outputs **only the final cumulative result** at stream completion.
It works the same as `Array.prototype.reduce` for arrays, with no intermediate results output.

## 🔰 Basic Syntax and Usage

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Output: 15 (final result only)
```

- `acc` is the cumulative value, `curr` is the current value.
- The values are accumulated sequentially, starting from the initial value (`0` in this case).
- No value is output until the stream completes, and **only the final result** is output on completion.

[🌐 RxJS Official Documentation - `reduce`](https://rxjs.dev/api/operators/reduce)

## 💡 Typical Usage Patterns

- Calculating sums, averages, maximums, and minimums of numbers
- Aggregating and transforming objects
- Building or combining arrays
- When only the final aggregate result is needed

## 🔍 Difference from scan

| Operator | Output Timing | Output Content | Usage |
|:---|:---|:---|:---|
| `reduce` | **Only once on completion** | Final cumulative result | Aggregation where only final result is needed |
| `scan` | **Every time for each value** | All including intermediate results | Real-time aggregation/state management |

```ts
import { of } from 'rxjs';
import { reduce, scan } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== reduce ===');
source$.pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 15

console.log('=== scan ===');
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 1, 3, 6, 10, 15
```

## 🧠 Practical Code Example (with UI)

This example sums the values of multiple input fields and displays the final result at the click of a button.

```ts
import { fromEvent, from } from 'rxjs';
import { map, reduce, switchMap } from 'rxjs';

// Create input fields
const inputs: HTMLInputElement[] = [];
for (let i = 1; i <= 3; i++) {
  const label = document.createElement('label');
  label.textContent = `Value ${i}: `;
  const input = document.createElement('input');
  input.type = 'number';
  input.value = '0';
  label.appendChild(input);
  document.body.appendChild(label);
  document.body.appendChild(document.createElement('br'));
  inputs.push(input);
}

// Calculate button
const button = document.createElement('button');
button.textContent = 'Calculate Sum';
document.body.appendChild(button);

// Result display area
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Calculate sum on button click
fromEvent(button, 'click').pipe(
  switchMap(() => {
    // Get all input values
    const values = inputs.map(input => Number(input.value) || 0);
    return from(values).pipe(
      reduce((acc, curr) => acc + curr, 0)
    );
  })
).subscribe(total => {
  output.textContent = `Total: ${total}`;
  console.log('Total:', total);
});
```

- Upon button click, all input values are summed and only the final total is displayed.
- Intermediate results are not output.

## 🎯 Object Aggregation Example

This is a practical example of aggregating multiple values into an object.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface Product {
  category: string;
  price: number;
}

const products: Product[] = [
  { category: 'Food', price: 500 },
  { category: 'Beverage', price: 200 },
  { category: 'Food', price: 800 },
  { category: 'Beverage', price: 150 },
  { category: 'Food', price: 300 },
];

// Aggregate total price by category
from(products).pipe(
  reduce((acc, product) => {
    acc[product.category] = (acc[product.category] || 0) + product.price;
    return acc;
  }, {} as Record<string, number>)
).subscribe(result => {
  console.log('Total by category:', result);
});

// Output:
// Total by category: { Food: 1600, Beverage: 350 }
```

## 🎯 Array Construction Example

Here is an example of combining stream values into an array.

```ts
import { interval } from 'rxjs';
import { take, reduce } from 'rxjs';

interval(100).pipe(
  take(5),
  reduce((acc, value) => {
    acc.push(value);
    return acc;
  }, [] as number[])
).subscribe(array => {
  console.log('Collected array:', array);
});

// Output:
// Collected array: [0, 1, 2, 3, 4]
```

::: tip
When building an array, consider using the more concise [`toArray`](/en/guide/operators/utility/toArray) operator.
```ts
interval(100).pipe(
  take(5),
  toArray()
).subscribe(console.log);
// Output: [0, 1, 2, 3, 4]
```
:::

## 💡 Utilizing Type-Safe reduce

Here is an example of utilizing TypeScript's type inference.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface UserAction {
  type: 'click' | 'scroll' | 'input';
  timestamp: number;
}

const actions: UserAction[] = [
  { type: 'click', timestamp: 100 },
  { type: 'scroll', timestamp: 200 },
  { type: 'click', timestamp: 300 },
  { type: 'input', timestamp: 400 },
];

const actions$ = from(actions);

// Aggregate count by action type
actions$.pipe(
  reduce((acc, action) => {
    acc[action.type] = (acc[action.type] || 0) + 1;
    return acc;
  }, {} as Record<UserAction['type'], number>)
).subscribe(result => {
  console.log('Action aggregation:', result);
});

// Output:
// Action aggregation: { click: 2, scroll: 1, input: 1 }
```

## ⚠️ Notes

### ❌ Infinite Streams Do Not Complete (Important)

> [!WARNING]
> **`reduce` will not output a single value until `complete()` is called.** Infinite streams (`interval`, `fromEvent`, etc.) cause accidents in practice, since no value is permanently available.

```ts
import { interval } from 'rxjs';
import { reduce } from 'rxjs';

// ❌ Bad example: Infinite stream so no value is output
interval(1000).pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// No output (stream does not complete)
```

**Countermeasure 1: Use `scan` when rolling aggregation is required**

```ts
import { interval, scan, take } from 'rxjs';

// ✅ Good example: Get intermediate results in real-time
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 0, 1, 3, 6, 10 (outputs cumulative value every time)
```

**Countermeasure 2: If only the final value is needed, use `scan` + `takeLast(1)`**

```ts
import { interval, scan, take, takeLast } from 'rxjs';

// ✅ Good example: Accumulate with scan, get only final value
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0),
  takeLast(1)
).subscribe(console.log);
// Output: 10 (final result only)
```

**Countermeasure 3: Use `take` to specify the end condition**

```ts
import { interval, take, reduce } from 'rxjs';

// ✅ Good example: Set end condition with take
interval(1000).pipe(
  take(5),
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Output: 10
```

> [!TIP]
> **Selection Criteria**:
> - Intermediate results are required → `scan`
> - Only the final result is needed & stream completion is guaranteed → `reduce`
> - Only final result needed & infinite stream → `scan` + `takeLast(1)` or `take` + `reduce`

### Memory Usage

When the cumulative value is a large object or array, memory usage should be taken into account.

```ts
// Example requiring memory attention
from(largeDataArray).pipe(
  reduce((acc, item) => {
    acc.push(item); // Accumulate large amounts of data
    return acc;
  }, [])
).subscribe();
```

## 📚 Related Operators

- [`scan`](/en/guide/operators/transformation/scan) - Outputs an intermediate result for each value
- [`toArray`](/en/guide/operators/utility/toArray) - Combine all values into an array
- [`count`](https://rxjs.dev/api/operators/count) - Counts the number of values
- [`min`](https://rxjs.dev/api/operators/min) / [`max`](https://rxjs.dev/api/operators/max) - Get minimum and maximum values

## Summary

The `reduce` operator accumulates all the values in a stream and outputs **only the final result on completion**. This is suitable when intermediate results are not needed and only the final aggregate result is needed. However, since no result is obtained if the stream is not completed, you must use `scan` for infinite streams, or set an exit condition with `take` or similar.
