---
description: ConcatMap is a conversion operator that processes each Observable in turn and waits for the next until the previous one completes, ideal for scenarios where execution order is important, such as serial execution of API calls or file upload order guarantees. TypeScript type inference enables type-safe asynchronous chaining, and the differences from mergeMap and switchMap are also explained.
---

# concatMap - Execute Each Observable in Order

The `concatMap` operator converts each value in the input stream into an Observable and **executes and concatenates them in turn**.
It does not start the next Observable **until the previous Observable has completed**.

## 🔰 Basic Syntax and Usage

```ts
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  concatMap(value =>
    of(`${value} completed`).pipe(delay(1000))
  )
).subscribe(console.log);

// Output (in order):
// A completed
// B completed
// C completed
```
- Converts each value to an Observable.
- The next Observable is executed after the previous Observable completes.

[🌐 RxJS Official Documentation - concatMap](https://rxjs.dev/api/index/function/concatMap)

## 💡 Typical Usage Patterns
- Execution of order-critical API requests
- Queue-based task processing
- Control of animations and step-by-step UI
- Message sending process where sending order is important


## 🧠 Practical Code Example (with UI)

This is an example where a request is generated each time a button is clicked and the requests are always processed in order.

```ts
import { fromEvent, of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

// Create button
const button = document.createElement('button');
button.textContent = 'Send Request';
document.body.appendChild(button);

// Output area
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Click event
fromEvent(button, 'click')
  .pipe(
    concatMap((_, index) => {
      const requestId = index + 1;
      console.log(`Request ${requestId} started`);
      return of(`Response ${requestId}`).pipe(delay(2000));
    })
  )
  .subscribe((response) => {
    const div = document.createElement('div');
    div.textContent = `✅ ${response}`;
    output.appendChild(div);
  });

```

- Each request is always sent and completed in order.
- The next request is issued after the previous request has completed.
