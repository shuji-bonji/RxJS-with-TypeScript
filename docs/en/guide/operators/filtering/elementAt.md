---
description: "The elementAt operator is an RxJS filtering operator that retrieves only the values at a given index position. It works similar to array index access."
---

# elementAt - get by index specification

The `elementAt` operator retrieves **only the value at the specified index position** from the Observable and completes the stream immediately. It works similar to `array[index]` of an array.

## 🔰 Basic Syntax and Usage

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Output: 30(Index2value)
```

**Flow of operation**:.
1. 10 (index 0) → skip
2. 20 (index 1) → skip
3. 30 (index 2) → output and complete
4. 40, 50 are not evaluated

[🌐 RxJS Official Documentation - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Typical utilization pattern

- **Pagination**: Get the first item on a specific page
- **Get Order Guarantee Data**: Get the Nth event or message
- **Testing and Debugging**: Validate the value of a specific position
- **Array-like access**: treat Observable like an array

## 🧠 Practical Code Example 1: Event Countdown

This is an example of executing an action on the Nth click.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UICreate
const output = document.createElement('div');
output.innerHTML = '<h3>5Click once to display message</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Click';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'more5Click once';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Click event
const clicks$ = fromEvent(button, 'click');

// For count display
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `more${remaining}Click once`;
  } else {
    counter.textContent = '';
  }
});

// 5Second (index)4Detected clicks of
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Achieved！';
  result.style.color = 'green';
  button.disabled = true;
});
```

- The fifth click (index 4) completes the action.
- It starts at 0, just like the array index.

## 🎯 Practical Code Example 2: Retrieve the Nth from the data stream

This is an example of retrieving a specific order of values from data issued at regular intervals.

```ts
import { interval } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UICreate
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'From data streamNGet the second';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Input the index (0~ (~)9)';
input.min = '0';
input.max = '9';
input.style.marginRight = '10px';
container.appendChild(input);

const getButton = document.createElement('button');
getButton.textContent = 'Get';
container.appendChild(getButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Data stream (0.5A value is issued every second,10(up to 1 piece)
const data$ = interval(500).pipe(
  map(i => ({ index: i, value: Math.floor(Math.random() * 100), timestamp: Date.now() }))
);

getButton.addEventListener('click', () => {
  const index = parseInt(input.value);

  if (isNaN(index) || index < 0 || index > 9) {
    status.textContent = '0~ (~)9Please enter a range of';
    status.style.color = 'red';
    return;
  }

  status.textContent = `Index ${index} value is being retrieved...`;
  status.style.color = 'blue';
  result.style.display = 'none';
  getButton.disabled = true;
  input.disabled = true;

  data$.pipe(
    elementAt(index)
  ).subscribe({
    next: data => {
      status.textContent = '';
      result.style.display = 'block';
      result.innerHTML = `
        <strong>✅ Success</strong><br>
        Index: ${data.index}<br>
        Value: ${data.value}<br>
        Timestamp: ${new Date(data.timestamp).toLocaleTimeString()}
      `;
      result.style.color = 'green';
      result.style.backgroundColor = '#e8f5e9';
      getButton.disabled = false;
      input.disabled = false;
    },
    error: err => {
      status.textContent = '';
      result.style.display = 'block';
      result.textContent = `❌ Error: ${err.message}`;
      result.style.color = 'red';
      result.style.backgroundColor = '#ffebee';
      getButton.disabled = false;
      input.disabled = false;
    }
  });
});
```

- Obtains values at a specified index from a stream issued every 0.5 seconds.
- If the index is out of range, an error is generated.

## 🆚 Comparison with similar operators

### elementAt vs take vs first

```ts
import { from } from 'rxjs';
import { elementAt, take, first, skip } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// elementAt: Retrieving only the value of a specific index
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Output: 30

// take: From the beginningNObtain one index from the beginning
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Output: 10, 20, 30

// skip + first: elementAt Equivalent to (redundant)
numbers$.pipe(
  skip(2),
  first()
).subscribe(console.log);
// Output: 30
```

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Output: 30(Index2value)
```

## ⚠️ Notes.

### 1. if the index is out of range

If the specified index is not reached before the stream completes, an error is generated.


```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // 3only one

numbers$.pipe(
  elementAt(5) // Index5Request
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Output: Error: no elements in sequence
```

### 2. specify default value

To prevent errors, default values can be specified.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Specify default value
numbers$.pipe(
  elementAt(5, 999) // Index5If not present, returns999Returns
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Output: 999
```

### 3. use with asynchronous streams

For asynchronous streams, wait until the index position is reached.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// 1Issue value every second
interval(1000).pipe(
  elementAt(3) // Index3(The4(the second value)
).subscribe(console.log);
// 3Output after 2 seconds: 3
```

### 4. negative indexes are not allowed

Negative indexes cannot be specified.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ❌ Negative indexes are errors
numbers$.pipe(
  elementAt(-1)
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Error: ArgumentOutOfRangeError: index out of range
```

Use `takeLast` or `last` if you want to get from the end of the array.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Get last value
numbers$.pipe(
  last()
).subscribe(console.log);
// Output: 50

// ✅ Get lastNGet the last value
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Output: 40, 50
```

## 📚 Related Operators

- **[take](. /take)** - take N from the beginning
- **[first](. /first)** - get the first value
- **[last](. /last)** - get the last value
- **[skip](. /skip)** - skip the first N values
- **[takeLast](. /takeLast)** - get the last N values

## Summary

The `elementAt` operator retrieves only the value at a given index position.

- ✅ Same behavior as array index access
- ✅ Ideal for retrieving the Nth value
- ✅ Can specify default values to avoid errors
- ⚠️ Error if index is out of range (no default value)
- ⚠️ Negative indexes are not allowed
- ⚠️ Asynchronous streams wait until reached
