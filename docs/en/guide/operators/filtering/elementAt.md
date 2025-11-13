---
description: The elementAt operator is an RxJS filtering operator that retrieves only the value at a specified index position. It behaves similarly to array index access.
---

# elementAt - Get Value at Specified Index

The `elementAt` operator retrieves **only the value at the specified index position** from an Observable and immediately completes the stream. It behaves similarly to `array[index]`.

## 🔰 Basic Syntax and Usage

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Output: 30 (value at index 2)
```

**Flow of operation**:
1. 10 (index 0) → Skip
2. 20 (index 1) → Skip
3. 30 (index 2) → Output and complete
4. 40, 50 are not evaluated

[🌐 RxJS Official Documentation - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Typical Usage Patterns

- **Pagination**: Get first item of a specific page
- **Ordered data retrieval**: Get Nth event or message
- **Testing and debugging**: Verify value at specific position
- **Array-like access**: Treat Observable like an array

## 🧠 Practical Code Example: Event Countdown

Example of executing an action on the Nth click.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// Create UI
const output = document.createElement('div');
output.innerHTML = '<h3>Display message on 5th click</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Click';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'Please click 5 more times';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Click event
const clicks$ = fromEvent(button, 'click');

// Count display
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `${remaining} more clicks`;
  } else {
    counter.textContent = '';
  }
});

// Detect 5th click (index 4)
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Achieved!';
  result.style.color = 'green';
  button.disabled = true;
});
```

- Completes on the 5th click (index 4).
- Starts from 0, same as array index.

## 🆚 Comparison with Similar Operators

### elementAt vs take vs first

| Operator | Retrieved Value | Output Count | Use Case |
|:---|:---|:---|:---|
| `elementAt(n)` | Only value at index n | 1 | Get Nth value |
| `take(n)` | First n values | n | Get first N values |
| `first()` | First value | 1 | Get first one |
| `skip(n) + first()` | First after skipping n | 1 | Same as elementAt (not recommended) |

## ⚠️ Notes

### 1. When Index is Out of Range

If the specified index is not reached before the stream completes, an error occurs.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // Only 3 items

numbers$.pipe(
  elementAt(5) // Request index 5
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Output: Error: no elements in sequence
```

### 2. Specifying Default Value

You can specify a default value to prevent errors.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Specify default value
numbers$.pipe(
  elementAt(5, 999) // Return 999 if index 5 does not exist
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Output: 999
```

### 3. Use with Asynchronous Streams

For asynchronous streams, it waits until reaching the index position.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// Emit value every second
interval(1000).pipe(
  elementAt(3) // Index 3 (4th value)
).subscribe(console.log);
// Output after 3 seconds: 3
```

### 4. Negative Index Not Available

Negative indexes cannot be specified.

To get from the end of the array, use `takeLast` or `last`.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Get last value
numbers$.pipe(
  last()
).subscribe(console.log);
// Output: 50

// ✅ Get last N values
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Output: 40, 50
```

## 📚 Related Operators

- **[take](/en/guide/operators/filtering/take)** - Get first N values
- **[first](/en/guide/operators/filtering/first)** - Get first value
- **[last](/en/guide/operators/filtering/last)** - Get last value
- **[skip](/en/guide/operators/filtering/skip)** - Skip first N values
- **[takeLast](/en/guide/operators/filtering/takeLast)** - Get last N values

## Summary

The `elementAt` operator retrieves only the value at the specified index position.

- ✅ Same behavior as array index access
- ✅ Ideal for getting Nth value
- ✅ Can avoid errors by specifying default value
- ⚠️ Error if index is out of range (without default value)
- ⚠️ Negative index not available
- ⚠️ Waits until reaching position for asynchronous streams
