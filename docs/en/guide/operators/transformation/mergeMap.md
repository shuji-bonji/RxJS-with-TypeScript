---
description: The mergeMap operator converts each value into a new Observable, executes them concurrently, and combines them flat. This is useful when multiple API requests need to be executed in parallel without waiting in sequence, or to manage nested asynchronous processing.
titleTemplate: ':title'
---

# mergeMap - Converts Each Value to an Observable and Merges Them Concurrently

The `mergeMap` (aka `flatMap`) operator converts each value into a new Observable and **merges them flat concurrently**.
It is very useful when you want to execute requests immediately without waiting in sequence, or for nested asynchronous processing.

## 🔰 Basic Syntax and Usage

```ts
import { of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  mergeMap(value =>
    of(`${value} completed`).pipe(delay(1000))
  )
).subscribe(console.log);

// Output example (in no particular order):
// A completed
// B completed
// C completed
```

- Generates a new Observable for each value.
- Those Observables are **executed in parallel** and the results are output in no particular order.

[🌐 RxJS Official Documentation - `mergeMap`](https://rxjs.dev/api/operators/mergeMap)

## 💡 Typical Usage Patterns

- Throw API request for every button click
- Initiate file upload for every file drop event
- Trigger asynchronous tasks concurrently triggered by user operations

## 🧠 Practical Code Example (with UI)

This is an example of triggering an asynchronous request (response after 2 seconds) each time a button is clicked.

```ts
import { fromEvent, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

// Create button
const button = document.createElement('button');
button.textContent = 'Send Request';
document.body.appendChild(button);

// Output area
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Click event
fromEvent(button, 'click').pipe(
  mergeMap((_, index) => {
    const requestId = index + 1;
    console.log(`Request ${requestId} started`);
    return of(`Response ${requestId}`).pipe(delay(2000));
  })
).subscribe((response) => {
  const div = document.createElement('div');
  div.textContent = `✅ ${response}`;
  output.appendChild(div);
});
```

- With each click, an asynchronous request is immediately issued.
- **Wait 2 seconds for each request individually**, so results are not ordered by order of arrival.
- This is a great sample for understanding parallel processing.
