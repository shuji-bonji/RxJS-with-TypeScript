---
description: This section provides an overview of ajax and fromFetch, Creation Functions for HTTP communication in RxJS, the differences between them, and guidelines for their usage.
---

# HTTP Communication Creation Functions

RxJS provides Creation Functions to handle HTTP communication as Observable. This section describes two functions, `ajax()` and `fromFetch()`, in detail.

## What are HTTP Communication Creation Functions?

HTTP Communication Creation Functions are a set of functions that allow communication with external APIs and servers to be handled as an Observable stream. By using these functions, asynchronous HTTP communication can be integrated into the RxJS operator chain, and error handling and retry processing can be described declaratively.

### Main Features

- **Declarative HTTP communication**: By treating HTTP communication as Observable, declarative processing using operators is possible
- **Uniform error handling**: Uniform error handling with operators such as `catchError()` and `retry()`
- **Cancelable**: Requests can be canceled with `unsubscribe()`
- **Integration with other streams**: Combine with other Observables via `switchMap()`, etc.

## List of HTTP Communication Creation Functions

| Function | Description | Base Technology | Main Uses |
|------|------|-----------|---------|
| [ajax()](/pt/guide/creation-functions/http-communication/ajax) | XMLHttpRequest-based HTTP communication | XMLHttpRequest | Legacy browser support, progress monitoring |
| [fromFetch()](/pt/guide/creation-functions/http-communication/fromFetch) | Fetch API-based HTTP communication | Fetch API | Modern browsers, lightweight HTTP communication |

## Comparison: ajax() vs fromFetch()

### Basic Differences

```typescript
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { fromFetch } from 'rxjs/fetch';

// ajax() - Automatically parses response
const ajax$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');
ajax$.subscribe(data => console.log(data));

// fromFetch() - Manually parse response
const fetch$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1').pipe(
  switchMap(response => response.json())
);
fetch$.subscribe(data => console.log(data));

interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}
```

### Feature Comparison Chart

| Feature | ajax() | fromFetch() |
|------|--------|-------------|
| Base Technology | XMLHttpRequest | Fetch API |
| Automatic JSON Parsing | ✅ Supported with `getJSON()` | ❌ Manually call `.json()` |
| Progress Events | ✅ Supported | ❌ Not supported |
| Timeout | ✅ Built-in support | ❌ Manual implementation required |
| Automatic HTTP Error Detection | ✅ Automatically errors on 4xx/5xx | ❌ Manual status check required |
| Request Cancellation | ✅ Possible with unsubscribe() | ✅ Possible with unsubscribe() |
| IE11 Support | ✅ Supported | ❌ Polyfill required |
| Bundle Size | Somewhat larger | Smaller |

## Usage Guidelines

### When to Choose ajax()

1. **Legacy browser support is required**
   - When you need to support older browsers such as IE11

2. **Progress monitoring is required**
   - When you want to display file upload/download progress

3. **Simple JSON retrieval**
   - When you want to get JSON easily with `getJSON()`

4. **Automatic error detection is needed**
   - When you want to use automatic error detection by HTTP status code

### When to Choose fromFetch()

1. **Only modern browsers are supported**
   - When you only support environments where the Fetch API is available

2. **Want to reduce bundle size**
   - When a lightweight HTTP communication function is sufficient

3. **Want to use Fetch API features**
   - When you want to manipulate Request/Response objects directly
   - When you want to use it in a Service Worker

4. **Need fine control**
   - When you want to customize the response processing in detail

## Practical Usage Examples

### API Call Pattern

```typescript
import { of, catchError, retry, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

// Practical pattern using ajax()
const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout after 5 seconds
  retry(2), // Retry twice on failure
  catchError(error => {
    console.error('User fetch error:', error);
    return of(null); // Return null on error
  })
);

fetchUser$.subscribe({
  next: user => {
    if (user) {
      console.log('User:', user);
    } else {
      console.log('Failed to fetch user');
    }
  }
});
```

### Form Submission Pattern

```typescript
import { fromEvent, switchMap, map } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Convert form submit event to Observable
const form = document.querySelector('form') as HTMLFormElement;
const submit$ = fromEvent(form, 'submit').pipe(
  map(event => {
    event.preventDefault();
    const formData = new FormData(form);
    return Object.fromEntries(formData.entries());
  }),
  switchMap(data =>
    ajax.post('https://api.example.com/submit', data, {
      'Content-Type': 'application/json'
    })
  )
);

submit$.subscribe({
  next: response => console.log('Submission successful:', response),
  error: error => console.error('Submission error:', error)
});
```

## Frequently Asked Questions

### Q1: Should I use ajax() or fromFetch()?

**A:** We recommend `fromFetch()` if only modern browsers are supported. The reasons are as follows:
- Fetch API is the latest web standard
- Small bundle size
- High future compatibility

However, choose `ajax()` in the following cases:
- IE11 support is required
- Progress monitoring is required
- Simple JSON retrieval is sufficient

### Q2: How are HTTP errors (4xx, 5xx) handled?

**A:**
- **ajax()**: HTTP status code over 400 is automatically treated as an error and the `error` callback is called
- **fromFetch()**: HTTP errors still trigger the `next` callback. You need to check `response.ok` manually

### Q3: How do I cancel a request?

**A:** Both can be canceled with `unsubscribe()`.

```typescript
const subscription = ajax.getJSON('/api/data').subscribe(...);

// Cancel after 3 seconds
setTimeout(() => subscription.unsubscribe(), 3000);
```

## Next Steps

For detailed usage of each function, please refer to the following pages:

- [ajax() in detail](/pt/guide/creation-functions/http-communication/ajax) - XMLHttpRequest-based HTTP communication
- [fromFetch() in detail](/pt/guide/creation-functions/http-communication/fromFetch) - Fetch API-based HTTP communication

## Reference Resources

- [RxJS Official Documentation - ajax](https://rxjs.dev/api/ajax/ajax)
- [RxJS Official Documentation - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/en-US/docs/Web/API/XMLHttpRequest)
