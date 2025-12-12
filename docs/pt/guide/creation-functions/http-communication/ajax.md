---
description: ajax() is an RxJS Creation Function that handles XMLHttpRequest based HTTP communication as Observable, supporting HTTP methods such as GET, POST, PUT, and DELETE, and providing practical functions such as progress monitoring, timeout processing, automatic JSON parsing, and support for legacy browsers. getJSON() can easily call the JSON API to achieve type-safe HTTP communication.
---

# ajax()

[📘 RxJS Official Documentation - ajax](https://rxjs.dev/api/ajax/ajax)

`ajax()` is a Creation Function for handling HTTP communication based on XMLHttpRequest as Observable, supporting HTTP methods such as GET, POST, PUT, and DELETE, and providing practical functions such as progress monitoring and timeout handling.

## Basic Usage

### Simple GET Request

The simplest example of using `ajax()` is simply passing a URL as a string.

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax('https://jsonplaceholder.typicode.com/todos/1');

api$.subscribe({
  next: response => console.log('Response:', response),
  error: error => console.error('Error:', error),
  complete: () => console.log('Complete')
});

// Output:
// Response: {
//   status: 200,
//   response: { userId: 1, id: 1, title: "delectus aut autem", completed: false },
//   ...
// }
// Complete
```

### Fetching JSON using getJSON()

If you want to get data from the JSON API, you can use `ajax.getJSON()`. It automatically parses the response and returns only the `response` property.

```typescript
import { ajax } from 'rxjs/ajax';

interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todos$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

todos$.subscribe({
  next: todo => console.log('Todo:', todo),
  error: error => console.error('Error:', error),
  complete: () => console.log('Complete')
});

// Output:
// Todo: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Complete
```

> [!TIP]
> **TypeScript Type Safety**
>
> Type safety of the response can be ensured by specifying a generic type for `ajax.getJSON<T>()`.

## Usage by HTTP Method

### GET Request

```typescript
import { ajax } from 'rxjs/ajax';

// Method 1: Simple string specification
const get1$ = ajax('https://api.example.com/users');

// Method 2: Automatic parsing with getJSON()
const get2$ = ajax.getJSON('https://api.example.com/users');

// Method 3: Detailed configuration
const get3$ = ajax({
  url: 'https://api.example.com/users',
  method: 'GET',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer token123'
  }
});
```

### POST Request

```typescript
import { ajax } from 'rxjs/ajax';

interface CreateUserRequest {
  name: string;
  email: string;
}

interface CreateUserResponse {
  id: number;
  name: string;
  email: string;
  createdAt: string;
}

const newUser: CreateUserRequest = {
  name: 'Taro Yamada',
  email: 'taro@example.com'
};

// Method 1: Using ajax.post()
const post1$ = ajax.post<CreateUserResponse>(
  'https://api.example.com/users',
  newUser,
  { 'Content-Type': 'application/json' }
);

// Method 2: Detailed configuration
const post2$ = ajax<CreateUserResponse>({
  url: 'https://api.example.com/users',
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer token123'
  },
  body: newUser
});

post1$.subscribe({
  next: response => console.log('Creation succeeded:', response.response),
  error: error => console.error('Creation failed:', error)
});
```

### PUT Request

```typescript
import { ajax } from 'rxjs/ajax';

interface UpdateUserRequest {
  name: string;
  email: string;
}

const updatedUser: UpdateUserRequest = {
  name: 'Jiro Tanaka',
  email: 'jiro@example.com'
};

const put$ = ajax.put(
  'https://api.example.com/users/1',
  updatedUser,
  { 'Content-Type': 'application/json' }
);

put$.subscribe({
  next: response => console.log('Update succeeded:', response.response),
  error: error => console.error('Update failed:', error)
});
```

### PATCH Request

```typescript
import { ajax } from 'rxjs/ajax';

interface PatchUserRequest {
  email?: string;
}

const patch$ = ajax.patch(
  'https://api.example.com/users/1',
  { email: 'new-email@example.com' } as PatchUserRequest,
  { 'Content-Type': 'application/json' }
);

patch$.subscribe({
  next: response => console.log('Partial update succeeded:', response.response),
  error: error => console.error('Partial update failed:', error)
});
```

### DELETE Request

```typescript
import { ajax } from 'rxjs/ajax';

const delete$ = ajax.delete('https://api.example.com/users/1');

delete$.subscribe({
  next: response => console.log('Deletion succeeded:', response),
  error: error => console.error('Deletion failed:', error)
});
```

## Practical Patterns

### Error Handling and Retries

HTTP communication requires handling network and server errors.

```typescript
import { of, retry, catchError, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout in 5 seconds
  retry(2), // Retry twice on failure
  catchError(error => {
    console.error('User fetch error:', error);
    // Return default value
    return of({
      id: 0,
      name: 'Unknown',
      email: 'unknown@example.com'
    } as User);
  })
);

fetchUser$.subscribe({
  next: user => console.log('User:', user),
  error: error => console.error('Fatal error:', error)
});
```

### Conditional Branching by HTTP Status Code

```typescript
import { throwError, catchError } from 'rxjs';
import { ajax, AjaxError } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/data').pipe(
  catchError((error: AjaxError) => {
    if (error.status === 404) {
      console.error('Resource not found');
    } else if (error.status === 401) {
      console.error('Authentication required');
    } else if (error.status === 500) {
      console.error('Server error occurred');
    } else {
      console.error('Unexpected error:', error);
    }
    return throwError(() => error);
  })
);
```

### Execute Multiple Requests in Parallel

```typescript
import { ajax } from 'rxjs/ajax';
import { forkJoin } from 'rxjs';

interface User {
  id: number;
  name: string;
}

interface Post {
  id: number;
  title: string;
  userId: number;
}

interface Comment {
  id: number;
  body: string;
  postId: number;
}

const users$ = ajax.getJSON<User[]>('https://jsonplaceholder.typicode.com/users');
const posts$ = ajax.getJSON<Post[]>('https://jsonplaceholder.typicode.com/posts');
const comments$ = ajax.getJSON<Comment[]>('https://jsonplaceholder.typicode.com/comments');

// Wait for all requests to complete
forkJoin({
  users: users$,
  posts: posts$,
  comments: comments$
}).subscribe({
  next: ({ users, posts, comments }) => {
    console.log('Users:', users);
    console.log('Posts:', posts);
    console.log('Comments:', comments);
  },
  error: error => console.error('Any request failed:', error)
});
```

### Searching Based on User Input (switchMap)

```typescript
import { fromEvent, map, debounceTime, distinctUntilChanged, switchMap, of } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface SearchResult {
  id: number;
  title: string;
}

const searchInput = document.querySelector('#search') as HTMLInputElement;

const search$ = fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // Wait 300ms
  distinctUntilChanged(), // Ignore same value
  switchMap(query => {
    if (query.length === 0) {
      return of([]);
    }
    // Cancel previous request if new query is entered
    return ajax.getJSON<SearchResult[]>(`https://api.example.com/search?q=${query}`);
  })
);

search$.subscribe({
  next: results => console.log('Search results:', results),
  error: error => console.error('Search error:', error)
});
```

> [!IMPORTANT]
> **Importance of switchMap()**
>
> Using `switchMap()`, a previous HTTP request is automatically canceled when a new search query is entered. This prevents old search results from overwriting new results.

### Progress Monitoring (File Upload)

`ajax()` can monitor upload and download progress using the `progress` event of `XMLHttpRequest`.

```typescript
import { tap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const fileInput = document.querySelector('#file') as HTMLInputElement;
const file = fileInput.files?.[0];

if (file) {
  const formData = new FormData();
  formData.append('file', file);

  const upload$ = ajax({
    url: 'https://api.example.com/upload',
    method: 'POST',
    body: formData,
    // Enable progress events
    progressSubscriber: {
      next: (progress) => {
        const percentage = (progress.loaded / progress.total) * 100;
        console.log(`Upload progress: ${percentage.toFixed(2)}%`);
      }
    }
  });

  upload$.subscribe({
    next: response => console.log('Upload complete:', response),
    error: error => console.error('Upload failed:', error)
  });
}
```

### Custom Headers and Cross-Domain Requests

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected-resource',
  method: 'GET',
  headers: {
    'Authorization': 'Bearer your-token-here',
    'X-Custom-Header': 'CustomValue'
  },
  crossDomain: true, // CORS request
  withCredentials: true // Include cookies
});

api$.subscribe({
  next: response => console.log('Response:', response),
  error: error => console.error('Error:', error)
});
```

## Common Use Cases

### 1. API Call with Pagination

```typescript
import { expand, takeWhile, reduce } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface PaginatedResponse {
  data: any[];
  page: number;
  totalPages: number;
}

const fetchAllPages$ = ajax.getJSON<PaginatedResponse>(
  'https://api.example.com/items?page=1'
).pipe(
  expand(response =>
    response.page < response.totalPages
      ? ajax.getJSON<PaginatedResponse>(`https://api.example.com/items?page=${response.page + 1}`)
      : []
  ),
  takeWhile(response => response.page <= response.totalPages, true),
  reduce((acc, response) => [...acc, ...response.data], [] as any[])
);

fetchAllPages$.subscribe({
  next: allItems => console.log('All items:', allItems),
  error: error => console.error('Error:', error)
});
```

### 2. Polling (Periodic Data Fetching)

```typescript
import { interval, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Status {
  status: string;
  lastUpdate: string;
}

// Call API every 5 seconds
const polling$ = interval(5000).pipe(
  switchMap(() => ajax.getJSON<Status>('https://api.example.com/status'))
);

const subscription = polling$.subscribe({
  next: status => console.log('Status:', status),
  error: error => console.error('Error:', error)
});

// Stop after 30 seconds
setTimeout(() => subscription.unsubscribe(), 30000);
```

### 3. Dependent Requests

```typescript
import { switchMap, map } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
}

interface UserDetails {
  userId: number;
  address: string;
  phone: string;
}

// First fetch user info, then fetch detailed information
const userWithDetails$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  switchMap(user =>
    ajax.getJSON<UserDetails>(`https://api.example.com/users/${user.id}/details`).pipe(
      map(details => ({ ...user, ...details }))
    )
  )
);

userWithDetails$.subscribe({
  next: userWithDetails => console.log('User details:', userWithDetails),
  error: error => console.error('Error:', error)
});
```

## ajax() Options

`ajax()` provides options for advanced configuration.

```typescript
interface AjaxConfig {
  url: string;                    // Request URL
  method?: string;                // HTTP method (GET, POST, PUT, DELETE, etc.)
  headers?: object;               // Request header
  body?: any;                     // Request body
  timeout?: number;               // Timeout time (in milliseconds)
  responseType?: string;          // Response type (json, text, blob, etc.)
  crossDomain?: boolean;          // CORS request or not
  withCredentials?: boolean;      // Whether to include cookies
  progressSubscriber?: Subscriber; // Subscriber for progress monitoring
}
```

## Common Errors and Workarounds

### 1. CORS Error

**Error Example:**
```
Access to XMLHttpRequest at 'https://api.example.com' from origin 'http://localhost:3000'
has been blocked by CORS policy
```

**Solutions:**
- Set the CORS header on the server side
- Use a proxy server
- Try `crossDomain: true` and `withCredentials: false` during development

### 2. Network Timeout

**Solution:**
```typescript
import { timeout, retry } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/slow-endpoint').pipe(
  timeout(10000), // Timeout in 10 seconds
  retry(2) // Retry twice
);
```

### 3. Authentication Error (401 Unauthorized)

**Solution:**
```typescript
import { throwError, catchError, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected',
  headers: {
    'Authorization': `Bearer ${getAccessToken()}`
  }
}).pipe(
  catchError(error => {
    if (error.status === 401) {
      // Refresh the token and retry
      return refreshToken().pipe(
        switchMap(newToken =>
          ajax({
            url: 'https://api.example.com/protected',
            headers: { 'Authorization': `Bearer ${newToken}` }
          })
        )
      );
    }
    return throwError(() => error);
  })
);
```

## ajax() vs fromFetch() Comparison

| Feature | ajax() | fromFetch() |
|------|--------|-------------|
| Automatic JSON parsing | ✅ `getJSON()` | ❌ Manually `.json()` |
| Progress monitoring | ✅ | ❌ |
| Automatic HTTP error detection | ✅ | ❌ |
| Bundle size | Slightly larger | Smaller |
| IE11 support | ✅ | ❌ |

> [!TIP]
> **How to Choose**
>
> - **Need progress monitoring**: Use `ajax()`
> - **Legacy browser support**: Use `ajax()`
> - **Lightweight HTTP communication**: Consider `fromFetch()`
> - **Simple JSON fetching**: `ajax.getJSON()` is easiest

## Best Practices

### 1. Ensure Type Safety

```typescript
// ✅ Good example: Specify a generic type
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todos$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Bad example: No type specified
const todos$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
```

### 2. Always Implement Error Handling

```typescript
// ✅ Good example: Error handling with catchError
const api$ = ajax.getJSON('/api/data').pipe(
  catchError(error => {
    console.error('Error:', error);
    return of(defaultValue);
  })
);

// ❌ Bad example: No error handling
const api$ = ajax.getJSON('/api/data');
```

### 3. Remember to Unsubscribe

```typescript
// ✅ Good example: Unsubscribe when destroying a component
class MyComponent {
  private subscription: Subscription;

  ngOnInit() {
    this.subscription = ajax.getJSON('/api/data').subscribe(...);
  }

  ngOnDestroy() {
    this.subscription.unsubscribe();
  }
}

// Or use takeUntil
class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    ajax.getJSON('/api/data')
      .pipe(takeUntil(this.destroy$))
      .subscribe(...);
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Summary

`ajax()` is a powerful Creation Function for HTTP communication in RxJS.

**Key Features:**
- Based on XMLHttpRequest, supports a wide range of browsers
- Easy JSON retrieval with `getJSON()`
- Practical functions such as progress monitoring, timeout, retry, etc.
- Automatic HTTP error detection

**Usage Scenarios:**
- Legacy browsers (e.g. IE11) support required
- Need to display file upload/download progress
- Simple and straightforward JSON API calls

**Important Notes:**
- Always implement error handling
- Always unsubscribe when no longer needed
- Utilize TypeScript types to ensure type safety

## Related Pages

- [fromFetch()](/pt/guide/creation-functions/http-communication/fromFetch) - Fetch API based HTTP communication
- [HTTP Communication Creation Functions](/pt/guide/creation-functions/http-communication/) - ajax() vs. fromFetch()
- [switchMap()](/pt/guide/operators/transformation/switchMap) - Useful operator for canceling HTTP communication
- [Error Handling Strategies](/pt/guide/error-handling/strategies) - Error handling patterns for HTTP communication

## References

- [RxJS Official Documentation - ajax](https://rxjs.dev/api/ajax/ajax)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/en-US/docs/Web/API/XMLHttpRequest)
- [Learn RxJS - ajax](https://www.learnrxjs.io/learn-rxjs/operators/creation/ajax)
