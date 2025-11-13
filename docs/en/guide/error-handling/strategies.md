---
description: A comprehensive error handling strategy for RxJS will be described, including how to combine catchError, retry, retryWhen, and finalize operators, retry with exponential backoff, resource release on error, fallback handling, and other practical patterns.
---
# RxJS Error Handling Strategies

Error handling in RxJS is an important aspect of reactive programming. Implementing proper error handling improves the robustness and reliability of your application. This document describes the various error handling strategies available in RxJS.

## Basic Pattern

RxJS handles errors as part of the Observable lifecycle. Basic error handling includes the following methods:

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

// Observable that generates an error
const error$ = throwError(() => new Error('An error occurred')); // RxJS 7+, functional form recommended

// Basic error handling
error$
  .pipe(
    catchError((error) => {
      console.error('Error caught:', error.message);
      return of('Fallback value after error');
    })
  )
  .subscribe({
    next: (value) => console.log('Value:', value),
    error: (err) => console.error('Unhandled error:', err),
    complete: () => console.log('Complete'),
  });

// Output:
// Error caught: An error occurred
// Value: Fallback value after error
// Complete
```

## Various Error Handling Strategies

### 1. Catch Errors and Provide Alternate Values

Use the `catchError` operator to catch errors and provide alternate values or alternate streams.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

const source$ = throwError(() => new Error('Data retrieval error'));

source$.pipe(
  catchError(error => {
    console.error('Error occurred:', error.message);
    // Return alternate data
    return of({ isError: true, data: [], message: 'Displaying default data' });
  })
).subscribe(data => console.log('Result:', data));

// Output:
// Error occurred: Data retrieval error
// Result: {isError: true, data: Array(0), message: 'Displaying default data'}
```

### 2. Retry if Error Occurs

Use the `retry` or `retryWhen` operator to retry the stream if an error occurs.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, tap } from 'rxjs';

let attemptCount = 0;

interval(1000).pipe(
  mergeMap(val => {
    if (++attemptCount <= 2) {
      return throwError(() => new Error(`Error #${attemptCount}`));
    }
    return of('Success!');
  }),
  tap(() => console.log('Execution:', attemptCount)),
  retry(2), // Retry up to 2 times
).subscribe({
  next: value => console.log('Value:', value),
  error: err => console.error('Final error:', err.message),
});

// Output:
// Execution: 3
// Value: Success!
// Execution: 4
// Value: Success!
// Execution: 5
// ...
```

### 3. Retry with Exponential Backoff

For network requests, for example, "exponential backoff," which gradually increases the retry interval, is effective.

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
// Error occurred: Network error
// Retry attempt 4 after 10000ms
// Error occurred: Network error
// Retry attempt 5 after 10000ms
// All retries failed: Maximum retry attempts exceeded
// Result: {error: true, message: 'Connection failed. Please try again later.'}
```

### 4. Resource Release When an Error Occurs

The `finalize` operator is used to release resources when a stream **completes or terminates with an error**.
`finalize` is useful when you want to ensure that cleanup processing is performed not only when an error occurs, but also when normal completion occurs.

```ts
import { throwError } from 'rxjs';
import { catchError, finalize } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Processing error'))
  .pipe(
    catchError((error) => {
      console.error('Error handling:', error.message);
      return throwError(() => error); // Re-throw error
    }),
    finalize(() => {
      isLoading = false;
      console.log('Reset loading state:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Value:', value),
    error: (err) => console.error('Final error:', err.message),
    complete: () => console.log('Complete'),
  });

// Output:
// Error handling: Processing error
// Final error: Processing error
// Reset loading state: false
```

## Error Handling Patterns

### Error Handling Including Display Control of UI Elements

```ts
import { of, throwError } from 'rxjs';
import { catchError, finalize, tap } from 'rxjs';

function fetchData(shouldFail = false) {
  // Show loading indicator
  showLoadingIndicator();

  // Data retrieval (success or error)
  return (
    shouldFail
      ? throwError(() => new Error('API error'))
      : of({ name: 'Data', value: 42 })
  ).pipe(
    tap((data) => {
      // Processing on success
      updateUI(data);
    }),
    catchError((error) => {
      // Update UI on error
      showErrorMessage(error.message);
      // Return empty data or default value
      return of({ name: 'Default', value: 0 });
    }),
    finalize(() => {
      // Hide loading indicator regardless of success or error
      hideLoadingIndicator();
    })
  );
}

// Helper functions for UI operations
function showLoadingIndicator() {
  console.log('Show loading');
}
function hideLoadingIndicator() {
  console.log('Hide loading');
}
function updateUI(data: { name: string; value: number }) {
  console.log('Update UI:', data);
}
function showErrorMessage(message: any) {
  console.log('Show error:', message);
}

// Usage example
fetchData(true).subscribe();

// Output:
// Show loading
// Show error: API error
// Hide loading
```

### Handling Multiple Error Sources

```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, map } from 'rxjs';

// Simulate multiple API requests
function getUser() {
  return of({ id: 1, name: 'Taro Yamada' });
}

function getPosts() {
  return throwError(() => new Error('Post retrieval error'));
}

function getComments() {
  return throwError(() => new Error('Comment retrieval error'));
}

// Retrieve all data and allow partial errors
forkJoin({
  user: getUser().pipe(
    catchError((error) => {
      console.error('User retrieval error:', error.message);
      return of(null); // Return null on error
    })
  ),
  posts: getPosts().pipe(
    catchError((error) => {
      console.error('Post retrieval error:', error.message);
      return of([]); // Return empty array on error
    })
  ),
  comments: getComments().pipe(
    catchError((error) => {
      console.error('Comment retrieval error:', error.message);
      return of([]); // Return empty array on error
    })
  ),
})
  .pipe(
    map((result) => ({
      ...result,
      // Add flag indicating partial errors
      hasErrors:
        !result.user ||
        result.posts.length === 0 ||
        result.comments.length === 0,
    }))
  )
  .subscribe((data) => {
    console.log('Final result:', data);

    if (data.hasErrors) {
      console.log(
        'Some data retrieval failed, but displaying available data'
      );
    }
  });

// Output:
// Post retrieval error: Post retrieval error
// Comment retrieval error: Comment retrieval error
// Final result: {user: {…}, posts: Array(0), comments: Array(0), hasErrors: true}
// Some data retrieval failed, but displaying available data
```

## Error Handling Best Practices

1. **Always catch errors**: Always add error handling in the Observable chain. This is especially important for long-running streams.

2. **Provide meaningful error messages**: Error objects should include information that helps determine the location and cause of the error.

3. **Properly release resources**: Use `finalize` to ensure that resources are released regardless of success or failure.

4. **Consider a retry strategy**: Implementing a proper retry strategy, especially for network operations, will improve reliability.

5. **User-friendly error handling**: The UI should provide information that users can understand, rather than just displaying technical error messages as is.

```ts
// Example: Converting to user-friendly error messages
function getErrorMessage(error: any): string {
  if (error.status === 401) {
    return 'Session has expired. Please log in again.';
  } else if (error.status === 404) {
    return 'The requested resource was not found.';
  } else if (error.status >= 500) {
    return 'A server error occurred. Please try again later.';
  }
  return 'An unexpected error occurred.';
}
```

## Summary

Error handling in RxJS is an important part of ensuring the robustness of the application. With the right combination of operators such as `catchError`, `retry`, and `finalize`, a variety of error scenarios can be handled. Design a comprehensive error handling strategy that goes beyond simply catching errors to improve the user experience.

## 🔗 Related Sections

- **[Common Mistakes and Solutions](/en/guide/anti-patterns/common-mistakes#9-error-suppression)** - Check anti-patterns related to error handling
- **[retry and catchError](/en/guide/error-handling/retry-catch)** - Detailed usage methods explained
