---
description: "Learn how to handle stream completion and resource cleanup in RxJS using finalize and complete: Essential techniques for preventing memory leaks"
titleTemplate: ':title'
---
# finalize - Limpeza de Recursos

In RxJS, it is important to properly manage stream completion and resource release. This page explains how the `finalize` operator and `complete` notifications work.

## finalize - Operator for Resource Release

The `finalize` operator is the operator that executes the specified cleanup code when the Observable exits **complete, error, or unsubscribe**.
finalize is always called **only once** at the end of the stream and never more than once.

[🌐 RxJS Official Documentation - finalize](https://rxjs.dev/api/index/function/finalize)

### Basic Usage of finalize

```ts
import { of } from 'rxjs';
import { finalize, tap } from 'rxjs';

// Variable to manage loading state
let isLoading = true;

// Stream that succeeds
of('data')
  .pipe(
    tap((data) => console.log('Processing data:', data)),
    // Executed in all cases: success, failure, or cancellation
    finalize(() => {
      isLoading = false;
      console.log('Loading state reset:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Value:', value),
    complete: () => console.log('Complete'),
  });

// Output:
// Processing data: data
// Value: data
// Complete
// Loading state reset: false
```

### finalize on Error

```ts
import { throwError } from 'rxjs';
import { finalize, catchError } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Data fetch error'))
  .pipe(
    catchError((err) => {
      console.error('Error handling:', err.message);
      throw err; // Re-throw error
    }),
    finalize(() => {
      isLoading = false;
      console.log('Resource release after error:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Value:', value),
    error: (err) => console.error('Error in subscriber:', err.message),
    complete: () => console.log('Complete'), // Not called on error
  });

// Output:
// Error handling: Data fetch error
// Error in subscriber: Data fetch error
// Resource release after error: false
```

### finalize on Unsubscribe

```ts
import { interval } from 'rxjs';
import { finalize } from 'rxjs';

let resource = 'Active';

// Count every second
const subscription = interval(1000)
  .pipe(
    finalize(() => {
      resource = 'Released';
      console.log('Resource state:', resource);
    })
  )
  .subscribe((count) => {
    console.log('Count:', count);

    // Manually unsubscribe after counting 3 times
    if (count >= 2) {
      subscription.unsubscribe();
    }
  });

// Output:
// Count: 0
// Count: 1
// Count: 2
// Resource state: Released
```

Finalize is useful not only when an error occurs, but also when you want to ensure cleanup processing upon successful completion or manual unsubscribe.

## complete - Notification of Successful Completion of Stream

When an Observable finishes successfully, the Observer's `complete` callback is invoked. This is the last step in the Observable's life cycle.

### Automatic complete

Some Observables complete automatically when certain conditions are met.

```ts
import { of } from 'rxjs';
import { take } from 'rxjs';

// Finite sequences complete automatically
of(1, 2, 3).subscribe({
  next: (value) => console.log('Value:', value),
  complete: () => console.log('Finite stream complete'),
});

// Stream limited with interval + take
interval(1000)
  .pipe(
    take(3) // Complete after obtaining 3 values
  )
  .subscribe({
    next: (value) => console.log('Count:', value),
    complete: () => console.log('Limited stream complete'),
  });

// Output:
// Value: 1
// Value: 2
// Value: 3
// Finite stream complete
// Count: 0
// Count: 1
// Count: 2
// Limited stream complete

```

### Manual complete

For Subject and custom ones, complete can be called manually.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.subscribe({
  next: (value) => console.log('Value:', value),
  complete: () => console.log('Subject complete'),
});

subject.next(1);
subject.next(2);
subject.complete(); // Complete manually
subject.next(3); // Ignored after completion

// Output:
// Value: 1
// Value: 2
// Subject complete
```

## Difference between finalize and complete

It is important to understand the important differences.

1. **Execution Timing**
   - `complete`: called only when an Observable completes **successfully**
   - `finalize`: called when the Observable terminates with **completion, error, or unsubscribe**

2. **Usage**
   - `complete`: Receive notification of a successful completion (processing on success)
   - `finalize`: Ensure that resources are released or cleaned up (processing that must be performed regardless of success or failure)

## Practical Use Cases

### API Calls and Loading State Management

```ts
import { ajax } from 'rxjs/ajax';
import { finalize, catchError } from 'rxjs';
import { of } from 'rxjs';

// Loading state
let isLoading = false;

function fetchData(id: string) {
  // Start loading
  isLoading = true;
  const loading = document.createElement('p');
  loading.style.display = 'block';
  document.body.appendChild(loading);
  // document.getElementById('loading')!.style.display = 'block';

  // API request
  return ajax.getJSON(`https://jsonplaceholder.typicode.com/posts/${id}`).pipe(
    catchError((error) => {
      console.error('API error:', error);
      return of({ error: true, message: 'Failed to retrieve data' });
    }),
    // End loading regardless of success or failure
    finalize(() => {
      isLoading = false;
      loading!.style.display = 'none';
      console.log('Loading state reset complete');
    })
  );
}

// Usage example
fetchData('123').subscribe({
  next: (data) => console.log('Data:', data),
  complete: () => console.log('Data fetch complete'),
});

// Output:
//  API error: AjaxErrorImpl {message: 'ajax error', name: 'AjaxError', xhr: XMLHttpRequest, request: {…}, status: 0, …}
//  Data: {error: true, message: 'Failed to retrieve data'}
//  Data fetch complete
//  Loading state reset complete
//   GET https://jsonplaceholder.typicode.com/posts/123 net::ERR_NAME_NOT_RESOLVED
```

### Resource Cleanup

```ts
import { interval } from 'rxjs';
import { finalize, takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

class ResourceManager {
  private destroy$ = new Subject<void>();
  private timerId: number | null = null;

  constructor() {
    // Initialize some resource
    this.timerId = window.setTimeout(() => console.log('Timer execution'), 10000);

    // Periodic processing
    interval(1000)
      .pipe(
        // Stop on component destruction
        takeUntil(this.destroy$),
        // Ensure resource release
        finalize(() => {
          console.log('Interval stopped');
        })
      )
      .subscribe((count) => {
        console.log('Running...', count);
      });
  }

  dispose() {
    // Dispose processing
    if (this.timerId) {
      window.clearTimeout(this.timerId);
      this.timerId = null;
    }

    // Stream stop signal
    this.destroy$.next();
    this.destroy$.complete();

    console.log('Resource manager disposal complete');
  }
}

// Usage example
const manager = new ResourceManager();

// Dispose after 5 seconds
setTimeout(() => {
  manager.dispose();
}, 5000);

// Output:
// Running... 0
// Running... 1
// Running... 2
// Running... 3
// Running... 4
// Interval stopped
// Resource manager disposal complete
```

[📘 RxJS Official: takeUntil()](https://rxjs.dev/api/index/function/takeUntil)

## Best Practices

1. **Always free resources**: use `finalize` to ensure cleanup when the stream ends
2. **Loading state management**: always reset loading state using `finalize`
3. **Component lifecycle management**: use `takeUntil` in combination with `finalize` to clean up resources when components are destroyed (this pattern is recommended, especially for Angular)
4. **Use with error handling**: Combine `catchError` and `finalize` to provide fallback handling and reliable cleanup after errors
5. **Knowing completion status**: use `complete` callback to determine if the stream has completed successfully

## Summary

`finalize` and `complete` are important tools for resource management and processing completion in RxJS. `finalize` is ideal for resource release because it ensures that the stream is executed no matter how it terminates. On the other hand, `complete` is used when you want to perform normal exit processing. By combining these tools appropriately, you can prevent memory leaks and build reliable applications.
