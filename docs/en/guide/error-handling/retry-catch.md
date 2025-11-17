---
description: Robust error handling strategies that combine retry and catchError operators will be explained. You will learn through practical API request examples such as retries for temporary failures, exponential backoff patterns, and proper fallback handling.
titleTemplate: ':title | RxJS'
---
# retry and catchError - Effective Error Handling Combination

The two operators at the heart of error handling in RxJS, `retry` and `catchError`, are described in detail. Together, they provide a robust error handling strategy.

## retry - Retry on Failure (Basic Pattern)

The `retry` operator is used to **resume execution of a stream a specified number of times** when an error occurs in the stream. This is especially useful for operations that may fail temporarily, such as network requests.

[🌐 RxJS Official Documentation - retry](https://rxjs.dev/api/index/function/retry)

### Basic Pattern

```ts
import { Observable, of } from 'rxjs';
import { retry, map } from 'rxjs';

// Function that randomly generates errors
function getDataWithRandomError(): Observable<string> {
  return of('data').pipe(
    map(() => {
      if (Math.random() < 0.7) {
        throw new Error('Random error occurred');
      }
      return 'Data retrieval successful!';
    })
  );
}

// Retry up to 3 times
getDataWithRandomError()
  .pipe(retry(3))
  .subscribe({
    next: (data) => console.log('Success:', data),
    error: (err) => console.error('Error (after 3 retries):', err.message),
  });

// Output:
// Success: Data retrieval successful!
// Error (after 3 retries): Random error occurred ⇦ Displayed when all 3 retries fail
```

### Real-time Retry Status Monitoring

```ts
import { Observable, of } from 'rxjs';
import { retry, tap, catchError, map } from 'rxjs';

let attempts = 0;

function simulateFlakyRequest(): Observable<string> {
  return of('request').pipe(
    tap(() => {
      attempts++;
      console.log(`Attempt #${attempts}`);
    }),
    map(() => {
      if (attempts < 3) {
        throw new Error(`Error #${attempts}`);
      }
      return 'Success!';
    })
  );
}

simulateFlakyRequest()
  .pipe(
    retry(3),
    catchError((error) => {
      console.log('All retries failed:', error.message);
      return of('Fallback value');
    })
  )
  .subscribe({
    next: (result) => console.log('Final result:', result),
    complete: () => console.log('Complete'),
  });

// Output:
// Attempt #1
// Attempt #2
// Attempt #3
// Final result: Success!
// Complete
```

> [!NOTE] Retry Timing and Schedulers
> When specifying a delay time in the `retry` operator (such as `retry({ delay: 1000 })`), **asyncScheduler** is used internally. By utilizing schedulers, you can control the timing of retries in detail and use virtual time during testing.
>
> For more information, see [Scheduler Types and Usage - Controlling Error Retries](/en/guide/schedulers/types#error-retry-control).

## catchError - Error Catching and Alternative Handling (Basic Pattern)

The `catchError` operator catches errors that occur in the stream and handles them by **returning an alternative Observable**. This allows processing to continue without interrupting the stream when an error occurs.

[🌐 RxJS Official Documentation - catchError](https://rxjs.dev/api/index/function/catchError)

### Basic Pattern

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('API call error')) // RxJS 7+, functional form recommended
  .pipe(
    catchError((error) => {
      console.error('Error occurred:', error.message);
      return of('Default value on error');
    })
  )
  .subscribe({
    next: (value) => console.log('Value:', value),
    complete: () => console.log('Complete'),
  });

// Output:
// Error occurred: API call error
// Value: Default value on error
// Complete
```

### Rethrowing Errors

If you want to re-throw an error after it has been logged

```ts
import { throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('Original error')) // RxJS 7+, functional form recommended
  .pipe(
    catchError((error) => {
      console.error('Logging error:', error.message);
      // Re-throw error
      return throwError(() => new Error('Converted error'));
    })
  )
  .subscribe({
    next: (value) => console.log('Value:', value),
    error: (err) => console.error('Final error:', err.message),
    complete: () => console.log('Complete'),
  });

// Output:
// Logging error: Original error
// Final error: Converted error
```

## Combination of retry and catchError

In actual applications, it is common to use a combination of `retry` and `catchError`. This combination allows temporary errors to be resolved by retrying, while providing a fallback value in the event of eventual failure.

```ts
import { of, throwError } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

function fetchData() {
  // Observable that generates an error
  return throwError(() => new Error('Network error')) // RxJS 7+, functional form recommended
    .pipe(
    // For debugging
    tap(() => console.log('Attempting data retrieval')),
    // Retry up to 3 times
    retry(3),
    // If all retries fail
    catchError((error) => {
      console.error('All retries failed:', error.message);
      // Return default value
      return of({
        error: true,
        data: null,
        message: 'Failed to retrieve data',
      });
    })
  );
}

fetchData().subscribe({
  next: (result) => console.log('Result:', result),
  complete: () => console.log('Processing complete'),
});

// Output:
// All retries failed: Network error
// Result: {error: true, data: null, message: 'Failed to retrieve data'}
// Processing complete
```

## Advanced Retry Strategy: retryWhen

If you need a more flexible retry strategy, you can use the `retryWhen` operator. This allows you to customize the retry timing and logic.


[🌐 RxJS Official Documentation - retryWhen](https://rxjs.dev/api/index/function/retryWhen)

### Retry with Exponential Backoff

The exponential backoff pattern (gradually increasing retry intervals) is common for retries of network requests. This reduces the load on the server while waiting for temporary problems to be resolved.

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, tap, concatMap, catchError } from 'rxjs';

function fetchWithRetry() {
  let retryCount = 0;

  return throwError(() => new Error('Network error')).pipe(
    retryWhen((errors) =>
      errors.pipe(
        // Count error occurrences
        tap((error) => console.log('Error occurred:', error.message)),
        // Delay with exponential backoff
        concatMap(() => {
          retryCount++;
          const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
          console.log(`Retry attempt ${retryCount} after ${delayMs}ms`);
          // timer uses asyncScheduler internally
          return timer(delayMs);
        }),
        // Retry up to 5 times
        tap(() => {
          if (retryCount >= 5) {
            throw new Error('Maximum retry attempts exceeded');
          }
        })
      )
    ),
    // Final fallback
    catchError((error) => {
      console.error('All retries failed:', error.message);
      return of({
        error: true,
        message: 'Connection failed. Please try again later.',
      });
    })
  );
}

fetchWithRetry().subscribe({
  next: (result) => console.log('Result:', result),
  error: (err) => console.error('Unhandled error:', err),
});

// Output:
// Error occurred: Network error
// Retry attempt 1 after 2000ms
// Error occurred: Network error
// Retry attempt 2 after 4000ms
// Error occurred: Network error
// Retry attempt 3 after 8000ms
```

> [!TIP] Detailed Retry Control Using Schedulers
> In the above example, `timer()` is used, but if more advanced control is needed, you can specify a scheduler explicitly to fine-tune the timing of retries or use virtual time during testing.
>
> For more information, see [Scheduler Types and Usage - Controlling Error Retries](/en/guide/schedulers/types#error-retry-control).

## Debugging Retries

When debugging the retry process, it is important to keep track of the number of attempts and the results of each attempt. Below are some practical ways to monitor retry status in real time.

### Method 1: error Callback for tap (Basic)

The `tap` operator's `error` callback can be used to count the number of attempts when an error occurs.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Temporary error'))
  .pipe(
    tap({
      error: () => {
        attemptCount++;
        console.log(`Attempt count: ${attemptCount}`);
      }
    }),
    retry(2),
    catchError((error) => {
      console.log(`Final attempt count: ${attemptCount}`);
      return of(`Final error: ${error.message}`);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Subscription error:', err)
  });

// Output:
// Attempt count: 1
// Attempt count: 2
// Attempt count: 3
// Final attempt count: 3
// Final error: Temporary error
```

> [!NOTE] Limitations with throwError
> `throwError` emits an error immediately without emitting a value, so the `tap` `next` callback is not executed. The `error` callback must be used.

### Method 2: Detailed Tracking with retryWhen (Recommended)

To track more detailed information (number of attempts, delay time, error details), use `retryWhen`.

```typescript
import { throwError, of, timer, retryWhen, mergeMap, catchError } from 'rxjs';
throwError(() => new Error('Temporary error'))
  .pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
          console.log(`🔄 Retry ${retryCount}`);
          console.log(`   Error: ${error.message}`);

          if (retryCount > 2) {
            console.log(`❌ Maximum retry count reached`);
            console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
            throw error;
          }

          const delayMs = 1000;
          console.log(`⏳ Retrying after ${delayMs}ms...`);
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

          return timer(delayMs);
        })
      )
    ),
    catchError((error) => {
      console.log(`\nFinal result: All retries failed`);
      return of(`Final error: ${error.message}`);
    })
  )
  .subscribe(result => console.log('Result:', result));

// Output:
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 1
//    Error: Temporary error
// ⏳ Retrying after 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (Wait 1 second)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 2
//    Error: Temporary error
// ⏳ Retrying after 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (Wait 1 second)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 3
//    Error: Temporary error
// ❌ Maximum retry count reached
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
//
// Final result: All retries failed
// Result: Final error: Temporary error
```

### Method 3: Track Attempts with a Custom Observable

For Observables that issue values, such as actual API requests, you can manage the number of attempts with a custom Observable.

```typescript
import { Observable, of, retry, catchError } from 'rxjs';
let attemptCount = 0;

// Observable that can count attempts
const retryableStream$ = new Observable(subscriber => {
  attemptCount++;
  console.log(`[Attempt ${attemptCount}]`);

  // First 2 attempts fail, 3rd attempt succeeds
  if (attemptCount < 3) {
    subscriber.error(new Error(`Failed (attempt ${attemptCount})`));
  } else {
    subscriber.next('Success data');
    subscriber.complete();
  }
});

retryableStream$
  .pipe(
    retry(2),
    catchError((error) => {
      console.log(`[Completed] Failed after a total of ${attemptCount} attempts`);
      return of(`Final error: ${error.message}`);
    })
  )
  .subscribe({
    next: data => console.log('[Result]', data),
    complete: () => console.log('[Completed]')
  });

// Output:
// [Attempt 1]
// [Attempt 2]
// [Attempt 3]
// [Result] Success data
// [Completed]
```

### Method 4: Exponential Backoff and Logging

This is a detailed logging pattern for practical API requests.

```typescript
import { timer, throwError, of, retryWhen, mergeMap, catchError, finalize } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchWithRetryLogging(url: string, maxRetries = 3) {
  let startTime = Date.now();

  return ajax.getJSON(url).pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          const elapsed = Date.now() - startTime;

          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
          console.log(`🔄 Retry Information`);
          console.log(`   Count: ${retryCount}/${maxRetries}`);
          console.log(`   Error: ${error.message || error.status}`);
          console.log(`   Elapsed time: ${elapsed}ms`);

          if (retryCount >= maxRetries) {
            console.log(`❌ Maximum number of retries reached`);
            console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
            throw error;
          }

          // Exponential backoff
          const delayMs = Math.min(1000 * Math.pow(2, index), 10000);
          console.log(`⏳ Retrying after ${delayMs}ms...`);
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

          return timer(delayMs);
        })
      )
    ),
    catchError((error) => {
      const totalTime = Date.now() - startTime;
      console.log(`\n❌ Final failure (total time: ${totalTime}ms)`);
      return of({ error: true, message: 'Data acquisition failed' });
    }),
    finalize(() => {
      const totalTime = Date.now() - startTime;
      console.log(`\n✅ Processing complete (total time: ${totalTime}ms)`);
    })
  );
}

// Usage example
fetchWithRetryLogging('https://jsonplaceholder.typicode.com/users/1').subscribe({
  next: data => console.log('Data:', data),
  error: err => console.error('Error:', err)
});
```

### Method 5: RxJS 7.4+ retry Configuration Object

In RxJS 7.4+ and later, you can pass a configuration object to `retry`.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Temporary error'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Attempt ${attemptCount}`);
      },
      error: (err) => console.log(`Error occurred:`, err.message)
    }),
    retry({
      count: 2,
      delay: 1000, // Wait 1 second before retry (uses asyncScheduler internally)
      resetOnSuccess: true
    }),
    catchError((error) => {
      console.log(`Final failure (total of ${attemptCount} attempts)`);
      return of(`Final error: ${error.message}`);
    })
  )
  .subscribe(result => console.log('Result:', result));

// Output:
// Attempt 1
// Error occurred: Temporary error
// Attempt 2
// Error occurred: Temporary error
// Attempt 3
// Error occurred: Temporary error
// Final failure (total of 3 attempts)
// Result: Final error: Temporary error
```

> [!TIP] Recommended Approach for Retry Debugging
> - **During development**: Method 2 (retryWhen) or Method 4 (detailed logging) is optimal
> - **Production environment**: Based on Method 4, add log sending to error monitoring service
> - **Simple cases**: Method 1 (tap error) or Method 5 (retry config) is sufficient
>
> **Related Information**:
> - For retry timing control, see [Scheduler Types and Usage - Controlling Error Retries](/en/guide/schedulers/types#error-retry-control)
> - For the overall picture of debugging techniques, see [RxJS Debugging Techniques - Tracking Retry Attempts](/en/guide/debugging/#scenario-6-tracking-retry-attempts)

## Example of Use in a Real Application: API Request

Here is an example of utilizing these operators in a real API request.

```ts
import { Observable, of } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { retry, catchError, finalize, tap } from 'rxjs';

// Loading state
let isLoading = false;

function fetchUserData(userId: string): Observable<any> {
  isLoading = true;

  return ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`).pipe(
    // Request debugging
    tap((response) => console.log('API Response:', response)),
    // Retry network errors up to 2 times
    retry(2),
    // Error handling
    catchError((error) => {
      if (error.status === 404) {
        return of({ error: true, message: 'User not found' });
      } else if (error.status >= 500) {
        return of({ error: true, message: 'A server error has occurred' });
      }
      return of({ error: true, message: 'An unknown error has occurred' });
    }),
    // Always execute regardless of success or failure
    finalize(() => {
      isLoading = false;
      console.log('Loading complete');
    })
  );
}

// Usage example
fetchUserData('123').subscribe({
  next: (data) => {
    if (data.error) {
      // Display error information
      console.error('Error:', data.message);
    } else {
      // Display data
      console.log('User data:', data);
    }
  },
});


// Output:
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// An unknown error has occurred
// Loading complete
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
```

## Best Practices

### When Should I Use retry?

- **When temporary errors** are expected (e.g., network connection problems)
- **Temporary problems** on the server side (e.g., high load, timeouts)
- For errors that **may** be resolved by retry

### When Should retry Not Be Used?

- **Authentication error** (401, 403) - retry will not resolve
- **Resource does not exist** (404) - retry does not find it
- **Validation error** (400) - there is a problem with the request itself
- **Client-side program error** - retry is pointless

### Effective Use of catchError

- Handle **differently** depending on **type** of error
- Provide **clear message** to user
- Return **fallback data** when appropriate
- Convert errors as needed

## Summary

The combination of `retry` and `catchError` provides robust error handling. Temporary errors can be recovered by retrying, and persistent errors can be appropriately fallbacked to improve the user experience. In real-world applications, it is important to select the appropriate strategy and provide a fallback mechanism depending on the nature of the error.

The following sections describe the `finalize` operator for resource release and the stream completion process.
