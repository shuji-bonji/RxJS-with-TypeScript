---
description: The filter operator is a basic RxJS operator that filters values based on specified conditions and is used to control data streams. Like Array.prototype.filter(), it uses a predicate function to determine which values to pass through, allowing for conditional value selection and type-safe filtering.
---

# filter - Filter Values Based on Conditions

The `filter` operator passes only values that **satisfy a specified condition** (predicate function).
This is the same concept as JavaScript's `Array.prototype.filter()` applied to Observables.

## 🔰 Basic Syntax and Usage

```ts
import { of } from 'rxjs';
import { filter } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

source$.pipe(
  filter(x => x % 2 === 0) // Pass only even numbers
).subscribe(value => {
  console.log('Value:', value);
});

// Output:
// Value: 2
// Value: 4
// Value: 6
// Value: 8
// Value: 10
```

- The predicate function `(value) => boolean` determines which values to pass through.
- Only values that return `true` are passed to the next operator.

## 💡 Typical Usage Patterns

- **Data filtering**: Select only values that satisfy specific conditions
- **Input validation**: Allow only valid values
- **Conditional processing**: Process different streams based on specific conditions
- **Type guard**: Narrow TypeScript types

## 🧠 Practical Code Example: User Input Validation

This example filters input values to allow only numeric characters.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Create input field
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Enter numbers only...';
input.style.padding = '8px';
input.style.margin = '10px';
document.body.appendChild(input);

// Output area
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
document.body.appendChild(output);

// Input event
const input$ = fromEvent<InputEvent>(input, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  filter(value => /^\d*$/.test(value)) // Allow only numeric characters
);

input$.subscribe(value => {
  output.textContent = `Valid value: ${value}`;
  console.log('Numeric value:', value);
});

// If non-numeric characters are entered, the event is filtered out
```

## 🔍 Difference from buffer

| Operator | Behavior | Output |
|:---|:---|:---|
| `filter` | Discards values that do **not match** the condition | Individual value `T` |
| `buffer` | **Accumulates** values into an array | Array `T[]` |

```ts
import { interval } from 'rxjs';
import { filter, buffer, take } from 'rxjs';

const source$ = interval(1000).pipe(take(5)); // 0, 1, 2, 3, 4

// filter - Pass only values that match the condition
source$.pipe(
  filter(x => x % 2 === 0)
).subscribe(x => {
  console.log('filter:', x);
  // Output: filter: 0
  // Output: filter: 2
  // Output: filter: 4
});

// buffer - Accumulate values as an array
source$.pipe(
  buffer(interval(2500))
).subscribe(arr => {
  console.log('buffer:', arr);
  // Output: buffer: [0, 1]
  // Output: buffer: [2, 3, 4]
});
```

[🌐 RxJS Official Documentation - `filter`](https://rxjs.dev/api/operators/filter)

## ⚠️ Notes

### 1. Predicate Functions Should Be Pure Functions

Avoid predicate functions with side effects.

```ts
// ❌ Bad example: With side effects
let counter = 0;
source$.pipe(
  filter(x => {
    counter++; // Side effect
    return x > 10;
  })
).subscribe();

// ✅ Good example: Pure function
source$.pipe(
  filter(x => x > 10)
).subscribe();
```

### 2. Using Type Guard Functions

You can leverage TypeScript's type safety.

```ts
interface User {
  id: number;
  name: string;
  email?: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob' }
);

// Use as type guard function
users$.pipe(
  filter((user): user is User & { email: string } => user.email !== undefined)
).subscribe(user => {
  console.log(user.email.toLowerCase()); // email is inferred as string type
});
```

## 📚 Related Operators

- [take](/en/guide/operators/filtering/take) - Get only the first N values
- [first](/en/guide/operators/filtering/first) - Get only the first value (conditionally possible)
- [distinct](/en/guide/operators/filtering/distinct) - Exclude duplicate values
- [distinctUntilChanged](/en/guide/operators/filtering/distinctUntilChanged) - Exclude values that are the same as the previous one

## Summary

The `filter` operator is the most basic filtering tool in RxJS.

- ✅ Passes only values that match the condition
- ✅ Can be used in the same way as `.filter()` for arrays
- ✅ Can be used as a TypeScript type guard
- ⚠️ Predicate functions should be pure functions
