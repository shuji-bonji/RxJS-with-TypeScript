---
description: The retry operator resubscribes and retries the source a specified number of times when an error occurs in Observable. This is useful for recovering from temporary communication failures, such as network failures, or for processes that may succeed if retried after a failure.
---

# retry - Retry on Error

The `retry` operator is an operator that **resubscribes the source Observable a specified number of times** when an error occurs.
It is suitable for **processes that may succeed if retried after failure**, such as temporary network failures.

## 🔰 Basic Syntax and Operation

### retry(count) - Basic Form

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError } from 'rxjs';

throwError(() => new Error('Temporary error'))
  .pipe(
    retry(2), // Retry up to 2 times
    catchError((error) => of(`Final error: ${error.message}`))
  )
  .subscribe(console.log);
// Output:
// Final error: Temporary error
```

In this example, up to two retries are made after the first failure, and a message is output on fallback if all fail.

### retry(config) - Configuration Object Format (RxJS 7.4+)

In RxJS 7.4 and later, more detailed control is possible by passing a configuration object.

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

let attemptCount = 0;

throwError(() => new Error('Temporary error'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Attempt ${attemptCount}`);
      }
    }),
    retry({
      count: 2,           // Retry up to 2 times
      delay: 1000,        // Wait 1 second before retrying (uses asyncScheduler internally)
      resetOnSuccess: true // Reset count on success
    }),
    catchError((error) => of(`Final error: ${error.message}`))
  )
  .subscribe(console.log);

// Output:
// Attempt 1
// Attempt 2
// Attempt 3
// Final error: Temporary error
```

> [!NOTE] Retry Timing Control
> When the `delay` option is specified, **asyncScheduler** is used internally. For more detailed retry timing control (exponential backoff, etc.), see [Scheduler Types and Usage - Error Retry Control](/en/guide/schedulers/types#error-retry-control).

[🌐 RxJS Official Documentation - retry](https://rxjs.dev/api/index/function/retry)

## 💡 Typical Usage Example

The following example is a configuration that retries **asynchronous processing with random success/failure** up to 3 times.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

let attempt = 0;

interval(1000)
  .pipe(
    mergeMap(() => {
      attempt++;
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        return throwError(() => new Error(`Failure #${attempt}`));
      } else {
        return of(`Success #${attempt}`);
      }
    }),
    retry(3),
    catchError((err) => of(`Final failure: ${err.message}`))
  )
  .subscribe(console.log);
// Output:
// Success #1
// Success #5
// Success #6
// Final failure: Failure #7
```

## 🧪 Practical Code Example (with UI)

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

// Output display area
const retryOutput = document.createElement('div');
retryOutput.innerHTML = '<h3>retry Example (API Request Simulation):</h3>';
document.body.appendChild(retryOutput);

// Request status display
const requestStatus = document.createElement('div');
requestStatus.style.marginTop = '10px';
requestStatus.style.padding = '10px';
requestStatus.style.border = '1px solid #ddd';
requestStatus.style.maxHeight = '200px';
requestStatus.style.overflowY = 'auto';
retryOutput.appendChild(requestStatus);

// API request that randomly succeeds or fails
let attemptCount = 0;

function simulateRequest() {
  attemptCount++;

  const logEntry = document.createElement('div');
  logEntry.textContent = `Attempt #${attemptCount} Sending request...`;
  requestStatus.appendChild(logEntry);

  return interval(1000).pipe(
    mergeMap(() => {
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        const errorMsg = document.createElement('div');
        errorMsg.textContent = `Attempt #${attemptCount} Failed: Network error`;
        errorMsg.style.color = 'red';
        requestStatus.appendChild(errorMsg);

        return throwError(() => new Error('Network error'));
      } else {
        const successMsg = document.createElement('div');
        successMsg.textContent = `Attempt #${attemptCount} Success!`;
        successMsg.style.color = 'green';
        requestStatus.appendChild(successMsg);

        return of({ id: 1, name: 'Data retrieved successfully' });
      }
    }),
    retry(3),
    catchError((err) => {
      const finalError = document.createElement('div');
      finalError.textContent = `All retries failed: ${err.message}`;
      finalError.style.color = 'red';
      finalError.style.fontWeight = 'bold';
      requestStatus.appendChild(finalError);

      return of({ error: true, message: 'Retry failed' });
    })
  );
}

// Request start button
const startButton = document.createElement('button');
startButton.textContent = 'Start Request';
startButton.style.padding = '8px 16px';
startButton.style.marginTop = '10px';
retryOutput.insertBefore(startButton, requestStatus);

startButton.addEventListener('click', () => {
  attemptCount = 0;
  requestStatus.innerHTML = '';
  startButton.disabled = true;

  simulateRequest().subscribe((result) => {
    const resultElement = document.createElement('div');
    if ('error' in result) {
      resultElement.textContent = `Final result: ${result.message}`;
      resultElement.style.backgroundColor = '#ffebee';
    } else {
      resultElement.textContent = `Final result: ${result.name}`;
      resultElement.style.backgroundColor = '#e8f5e9';
    }

    resultElement.style.padding = '10px';
    resultElement.style.marginTop = '10px';
    resultElement.style.borderRadius = '5px';
    requestStatus.appendChild(resultElement);

    startButton.disabled = false;
  });
});
```

## ✅ Summary

- `retry(n)` retries up to `n` times if Observable fails
- `retry` is **retried until it completes successfully** (continued failure results in an error)
- Useful for **asynchronous APIs and network requests** where temporary failures occur
- Commonly combined with `catchError` to specify **fallback processing**
- As of RxJS 7.4+, it is possible to specify `delay`, `resetOnSuccess`, etc. in configuration object format

## Related Pages

- [retry and catchError](/en/guide/error-handling/retry-catch) - Patterns for combining retry and catchError, practical usage examples
- [Retry Debugging](/en/guide/error-handling/retry-catch#retry-debugging) - How to track attempt count (5 implementation patterns)
- [Scheduler Types and Usage](/en/guide/schedulers/types#error-retry-control) - Detailed retry timing control, exponential backoff implementation
- [RxJS Debugging Techniques](/en/guide/debugging/#scenario-6-track-retry-attempt-count) - Retry debugging scenarios
