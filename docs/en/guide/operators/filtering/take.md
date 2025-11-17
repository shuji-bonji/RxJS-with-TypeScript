---
description: The take operator retrieves only the first specified number of values from the Observable stream and automatically completes the stream, ignoring subsequent values. This is useful when you only want to retrieve the first few pieces of data.
titleTemplate: ':title | RxJS'
---

# take - Retrieve Only the First Specified Number of Values

The `take` operator retrieves only the **first specified number** of values from the stream and ignores subsequent values.
After completion, the stream automatically `completes`.

## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  take(3)
).subscribe(console.log);
// Output: 0, 1, 2
```

- Subscribes to only the first 3 values.
- After retrieving 3 values, the Observable automatically `completes`.

[🌐 RxJS Official Documentation - `take`](https://rxjs.dev/api/operators/take)

## 💡 Typical Usage Patterns

- Display or log only the first few items in UI
- Temporary subscription to retrieve only the first response
- Limited retrieval of test or demo data

## 🧠 Practical Code Example (with UI)

Retrieves and displays only the first 5 values from numbers emitted every second.

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>take Practical Example:</h3>';
document.body.appendChild(output);

// Emit values every second
const source$ = interval(1000);

// Take only the first 5 values
source$.pipe(take(5)).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `Value: ${value}`;
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

- The first 5 values (`0`, `1`, `2`, `3`, `4`) are displayed in order,
- Then the message "Completed" is displayed.
