---
description: timeout is an RxJS utility operator that throws an error if no value is emitted from Observable within a specified time. Ideal for time-constrained reactive processing such as API request timeout control, waiting for user action responses, or stream delay detection. It can be combined with catchError to implement fallback behavior, and TypeScript type inference enables type-safe timeout processing.
---

# timeout - Timeout Configuration

The `timeout` operator is an operator that **throws an error if no value is emitted by Observable within a specified time**.
It is often used for reactive processing, such as waiting for a response to an API request or user operation.


## 🔰 Basic Syntax and Operation

If the timeout is not exceeded, the operation continues as usual; if it exceeds a certain period, an error occurs.

```ts
import { of } from 'rxjs';
import { delay, timeout, catchError } from 'rxjs';

of('response')
  .pipe(
    delay(500), // 👈 If set to 1500, outputs `Timeout error: fallback`
    timeout(1000),
    catchError((err) => of('Timeout error: fallback', err))
  )
  .subscribe(console.log);
// Output:
// response
```

In this example, `'response'` is normally displayed since the value is emitted after 500ms due to `delay(500)` and the condition of `timeout(1000)` is satisfied.

If `delay(1200)` is specified, a `timeout error` is output as follows:
```sh
Timeout error: fallback
TimeoutErrorImpl {stack: 'Error\n    at _super (http://localhost:5174/node_mo…s/.vite/deps/chunk-RF6VPQMH.js?v=f6400bce:583:26)', message: 'Timeout has occurred', name: 'TimeoutError', info: {…}}
```

[🌐 RxJS Official Documentation - timeout](https://rxjs.dev/api/index/function/timeout)

## 💡 Typical Usage Example

The following example shows both a **pattern that causes a timeout if the stream delays and does not emit a value** and a **pattern that emits normally**.

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

const slow$ = interval(1500).pipe(take(3));
const fast$ = interval(500).pipe(take(3));

fast$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback: timeout occurred'))
  )
  .subscribe(console.log);

slow$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback: timeout triggered'))
  )
  .subscribe(console.log);
// Output:
// 0
// 1
// fallback: timeout triggered
// 2
```


## 🧪 Practical Code Example (with UI)

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

// Output display area
const timeoutOutput = document.createElement('div');
timeoutOutput.innerHTML = '<h3>timeout Example:</h3>';
document.body.appendChild(timeoutOutput);

// Timeout success case
const normalStream$ = interval(500).pipe(take(5));

const timeoutSuccess = document.createElement('div');
timeoutSuccess.innerHTML = '<h4>Normal Stream (No Timeout):</h4>';
timeoutOutput.appendChild(timeoutSuccess);

normalStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Error: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutSuccess.appendChild(errorMsg);
      return of('Fallback value after error');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Value: ${val}`;
    timeoutSuccess.appendChild(item);
  });

// Timeout error case
const slowStream$ = interval(1500).pipe(take(5));

const timeoutError = document.createElement('div');
timeoutError.innerHTML = '<h4>Slow Stream (Timeout Occurs):</h4>';
timeoutOutput.appendChild(timeoutError);

slowStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Error: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutError.appendChild(errorMsg);
      return of('Fallback value after timeout');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Value: ${val}`;
    timeoutError.appendChild(item);
  });
```


## ✅ Summary

- `timeout` is a control operator that **throws an error if no emission occurs within a certain time**
- Effective for timeout processing while waiting for network APIs or UI operations
- Can be combined with `catchError` to specify **fallback behavior**
