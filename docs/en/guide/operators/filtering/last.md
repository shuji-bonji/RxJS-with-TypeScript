---
description: "The last operator retrieves only the last value at stream completion or the last value matching a condition: Essential for final state extraction"
titleTemplate: ':title'
---

# last - Get Last Value

The `last` operator retrieves the **last value** or **last value satisfying a condition** from the stream and completes the stream.


## 🔰 Basic Syntax and Usage

```ts
import { from } from 'rxjs';
import { last } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Get only the last value
numbers$.pipe(
  last()
).subscribe(console.log);

// Get only the last value that satisfies the condition
numbers$.pipe(
  last(n => n < 5)
).subscribe(console.log);

// Output:
// 5
// 4
```

- `last()` outputs the **last value issued** upon stream completion.
- If a condition is passed, only the **last value** that satisfies the condition will be retrieved.
- If no value matching the condition exists, an error is generated.

[🌐 RxJS Official Documentation - `last`](https://rxjs.dev/api/operators/last)


## 💡 Typical Usage Patterns

- Get the last element of filtered data
- Retrieve the latest state at stream completion
- Retrieve the last significant operation in the session or operation log


## 🧠 Practical Code Example (with UI)

Retrieve and display the last value that was less than 5 out of the multiple values entered.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, take, last } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>last Practical Example:</h3>';
document.body.appendChild(output);

// Create input field
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Enter a number and press Enter';
document.body.appendChild(input);

// Input event stream
fromEvent<KeyboardEvent>(input, 'keydown')
  .pipe(
    filter((e) => e.key === 'Enter'),
    map(() => parseInt(input.value, 10)),
    take(5), // Complete when only the first 5 values are taken
    filter((n) => !isNaN(n) && n < 5), // Only pass values less than 5
    last() // Get the last value less than 5
  )
  .subscribe({
    next: (value) => {
      const item = document.createElement('div');
      item.textContent = `Last value less than 5: ${value}`;
      output.appendChild(item);
    },
    complete: () => {
      const complete = document.createElement('div');
      complete.textContent = 'Completed';
      complete.style.fontWeight = 'bold';
      output.appendChild(complete);
    },
  });

```
1. Enter a number 5 times and press Enter
2. Pick up only "less than 5" from the numbers entered
3. Display only the last number entered that is less than 5
4. The stream naturally completes and ends
