---
description: The exhaustMap operator is a conversion operator that ignores new input until the currently processing Observable completes. It is effective in situations where you want to limit concurrency, such as preventing multiple clicks on a form submit button or duplicate API request submissions.
---

# exhaustMap - Ignore New Input During Execution

The `exhaustMap` operator **ignores new input** until the currently processing Observable completes.
This is ideal for preventing duplicate clicks or multiple submissions of requests.

## 🔰 Basic Syntax and Usage

```ts
import { fromEvent, of } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$
  .pipe(exhaustMap(() => of('Request completed').pipe(delay(1000))))
  .subscribe(console.log);

// Output example:
// (Only the first click outputs "Request completed" after 1 second)

```

- Subsequent input is ignored until the running request completes.

[🌐 RxJS Official Documentation - `exhaustMap`](https://rxjs.dev/api/operators/exhaustMap)

## 💡 Typical Usage Patterns

- Prevention of multiple clicks on form submit buttons
- Prevention of double requests (especially for login and payment processes)
- Single display control of a modal or dialog

## 🧠 Practical Code Example (with UI)

Clicking the Send button starts the sending process.
**No matter how many times you click during transmission, it will be ignored** and the next transmission will not be accepted until the first process is completed.

```ts
import { fromEvent } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Create button
const submitButton = document.createElement('button');
submitButton.textContent = 'Submit';
document.body.appendChild(submitButton);

// Create output area
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Submit processing
fromEvent(submitButton, 'click')
  .pipe(
    exhaustMap(() => {
      output.textContent = 'Sending...';
      return ajax
        .post('https://jsonplaceholder.typicode.com/posts', {
          title: 'foo',
          body: 'bar',
          userId: 1,
        })
        .pipe(delay(2000)); // Simulate 2 second send delay
    })
  )
  .subscribe({
    next: (response) => {
      output.textContent = 'Submission successful!';
      console.log('Submission successful:', response);
    },
    error: (error) => {
      output.textContent = 'Submission error';
      console.error('Submission error:', error);
    },
  });

```

- Any other clicks while the button is being clicked will be ignored.
- After 2 seconds, you will see "Submission successful!" or "Submission error" will be displayed.
