---
description: The first operator retrieves only the first value from the stream, or the first value that satisfies the specified condition, and then completes the stream. This is useful when you want to process only the first event reached or to retrieve initial data.
---

# first - Retrieve the First Value or the First Value That Satisfies a Condition

The `first` operator retrieves only the **first value** or **first value satisfying a condition** from a stream and completes the stream.


## 🔰 Basic Syntax and Usage

```ts
import { from } from 'rxjs';
import { first } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Get only the first value
numbers$.pipe(
  first()
).subscribe(console.log);

// Get only the first value that satisfies the condition
numbers$.pipe(
  first(n => n > 3)
).subscribe(console.log);

// Output:
// 1
// 4
```

- `first()` gets the first value that flows and completes.
- If a condition is passed, the **first value that meets the condition** is retrieved.
- If no value matching the condition exists, an error is raised.

[🌐 RxJS Official Documentation - `first`](https://rxjs.dev/api/operators/first)


## 💡 Typical Usage Patterns

- Process only the first event reached
- Detect the first data that meets the criteria (e.g., a score of 5 or higher)
- Adopt only the first data that came in before a timeout or cancellation


## 🧠 Practical Code Example (with UI)

Process **only the first click** even if the button is clicked multiple times.

```ts
import { fromEvent } from 'rxjs';
import { first } from 'rxjs';

const title = document.createElement('div');
title.innerHTML = '<h3>first Practical Example:</h3>';
document.body.appendChild(title);

// Create button
const button = document.createElement('button');
button.textContent = 'Please click (respond only the first time)';
document.body.appendChild(button);

// Create output area
let count = 0;
const output = document.createElement('div');
document.body.appendChild(output);
// Button click stream
fromEvent(button, 'click')
  .pipe(first())
  .subscribe(() => {
    const message = document.createElement('div');
    count++;
    message.textContent = `First click detected! ${count}`;
    output.appendChild(message);
  });
```

- Only the first click event is received, and subsequent events are ignored.
- The stream will automatically `complete` after the first click.
