---
description: fromFetch() is a RxJS Creation Function that handles Fetch API-based HTTP communication as Observable, supporting HTTP methods such as GET, POST, PUT, DELETE, etc. It is lightweight, modern, web standards compliant, and Service Worker compatible for modern HTTP communication. Manual error checking and JSON parsing are required, but the bundle size is small and type-safe HTTP communication is possible.
---

# fromFetch()

[📘 RxJS Official Documentation - fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` is a Creation Function for handling HTTP communication as Observable based on the modern Fetch API. It is lightweight compared to `ajax()` and conforms to modern web standards.

## Basic Usage

### Simple GET Request

The simplest example of using `fromFetch()` is to pass a URL and parse the response manually.

```typescript
import { of, switchMap, catchError, throwError } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const data$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1').pipe(
  switchMap(response => {
    if (response.ok) {
      // If response is successful, parse JSON
      return response.json();
    } else {
      // If HTTP error, throw error
      return throwError(() => new Error(`HTTP Error: ${response.status}`));
    }
  }),
  catchError(error => {
    console.error('Error:', error);
    return of({ error: true, message: error.message });
  })
);

data$.subscribe({
  next: data => console.log('Data:', data),
  error: error => console.error('Subscription error:', error),
  complete: () => console.log('Complete')
});

// Output:
// Data: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Complete
```

> [!IMPORTANT]
> **Important Difference from ajax()**
>
> - `fromFetch()` does not call `error` callback on HTTP errors (4xx, 5xx)
> - The `ok` property of the response must be checked manually
> - Parsing operations such as `.json()` are also done manually

## Usage by HTTP Method

### GET Request

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface User {
  id: number;
  name: string;
  email: string;
}

const users$ = fromFetch('https://jsonplaceholder.typicode.com/users').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json() as Promise<User[]>;
  })
);

users$.subscribe({
  next: users => console.log('User list:', users),
  error: error => console.error('Error:', error)
});
```

### POST Request

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

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

const createUser$ = fromFetch('https://api.example.com/users', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer token123'
  },
  body: JSON.stringify(newUser)
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json() as Promise<CreateUserResponse>;
  })
);

createUser$.subscribe({
  next: user => console.log('Creation succeeded:', user),
  error: error => console.error('Creation failed:', error)
});
```

### PUT Request

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface UpdateUserRequest {
  name: string;
  email: string;
}

const updatedUser: UpdateUserRequest = {
  name: 'Jiro Tanaka',
  email: 'jiro@example.com'
};

const updateUser$ = fromFetch('https://api.example.com/users/1', {
  method: 'PUT',
  headers: {
    'Content-Type': 'application/json'
  },
  body: JSON.stringify(updatedUser)
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
);

updateUser$.subscribe({
  next: user => console.log('Update succeeded:', user),
  error: error => console.error('Update failed:', error)
});
```

### DELETE Request

```typescript
import { switchMap, of } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const deleteUser$ = fromFetch('https://api.example.com/users/1', {
  method: 'DELETE',
  headers: {
    'Authorization': 'Bearer token123'
  }
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    // DELETE usually returns empty response or status only
    return response.status === 204 ? of(null) : response.json();
  })
);

deleteUser$.subscribe({
  next: result => console.log('Deletion succeeded:', result),
  error: error => console.error('Deletion failed:', error)
});
```

## Practical Patterns

### Generic HTTP Error Handling Function

Since `fromFetch()` requires manual error checking, it is useful to create a generic function.

```typescript
import { Observable, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

function fetchJSON<T>(url: string, options?: RequestInit): Observable<T> {
  return fromFetch(url, options).pipe(
    switchMap(response => {
      if (!response.ok) {
        throw new Error(`HTTP Error ${response.status}: ${response.statusText}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// Usage example
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

todo$.subscribe({
  next: todo => console.log('Todo:', todo),
  error: error => console.error('Error:', error)
});
```

### Detailed Processing by HTTP Status Code

```typescript
import { throwError, switchMap, of } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/data').pipe(
  switchMap(response => {
    switch (response.status) {
      case 200:
        return response.json();
      case 204:
        // No Content - empty response
        return of(null);
      case 401:
        throw new Error('Authentication required');
      case 403:
        throw new Error('Access denied');
      case 404:
        throw new Error('Resource not found');
      case 500:
        throw new Error('Server error occurred');
      default:
        throw new Error(`Unexpected HTTP status: ${response.status}`);
    }
  })
);

api$.subscribe({
  next: data => console.log('Data:', data),
  error: error => console.error('Error:', error)
});
```

### Timeout and Retry

```typescript
import { switchMap, timeout, retry } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow-endpoint').pipe(
  timeout(5000), // Timeout in 5 seconds
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  }),
  retry(2) // Retry twice on failure
);

api$.subscribe({
  next: data => console.log('Data:', data),
  error: error => console.error('Error:', error)
});
```

### Request Cancellation (AbortController)

`fromFetch()` supports canceling requests using the Fetch API's `AbortController`.

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const controller = new AbortController();
const signal = controller.signal;

const api$ = fromFetch('https://api.example.com/data', {
  signal // Pass AbortController signal
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
);

const subscription = api$.subscribe({
  next: data => console.log('Data:', data),
  error: error => console.error('Error:', error)
});

// Cancel request after 3 seconds
setTimeout(() => {
  controller.abort();
  // Or subscription.unsubscribe();
}, 3000);
```

> [!TIP]
> **Automatic Cancellation by RxJS**
>
> Just call `unsubscribe()` and RxJS will cancel the request internally using `AbortController`. There is no need to manually set up an `AbortController`.

### Searching Based on User Input (switchMap)

```typescript
import { fromEvent, map, debounceTime, distinctUntilChanged, switchMap, of } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface SearchResult {
  id: number;
  title: string;
}

const searchInput = document.querySelector('#search') as HTMLInputElement;

const search$ = fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),
  distinctUntilChanged(),
  switchMap(query => {
    if (query.length === 0) {
      return of([]);
    }
    return fromFetch(`https://api.example.com/search?q=${encodeURIComponent(query)}`).pipe(
      switchMap(response => {
        if (!response.ok) {
          throw new Error(`HTTP Error: ${response.status}`);
        }
        return response.json() as Promise<SearchResult[]>;
      })
    );
  })
);

search$.subscribe({
  next: results => console.log('Search results:', results),
  error: error => console.error('Search error:', error)
});
```

### Execute Multiple Requests in Parallel

```typescript
import { forkJoin, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface User {
  id: number;
  name: string;
}

interface Post {
  id: number;
  title: string;
}

const users$ = fromFetch('https://jsonplaceholder.typicode.com/users').pipe(
  switchMap(response => response.json() as Promise<User[]>)
);

const posts$ = fromFetch('https://jsonplaceholder.typicode.com/posts').pipe(
  switchMap(response => response.json() as Promise<Post[]>)
);

forkJoin({
  users: users$,
  posts: posts$
}).subscribe({
  next: ({ users, posts }) => {
    console.log('Users:', users);
    console.log('Posts:', posts);
  },
  error: error => console.error('Any request failed:', error)
});
```

## Common Use Cases

### 1. Request with Authentication Token

```typescript
import { switchMap, Observable } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

function getAuthToken(): string {
  return localStorage.getItem('authToken') || '';
}

function fetchWithAuth<T>(url: string, options: RequestInit = {}): Observable<T> {
  return fromFetch(url, {
    ...options,
    headers: {
      ...options.headers,
      'Authorization': `Bearer ${getAuthToken()}`,
      'Content-Type': 'application/json'
    }
  }).pipe(
    switchMap(response => {
      if (response.status === 401) {
        throw new Error('Authentication required. Please log in again.');
      }
      if (!response.ok) {
        throw new Error(`HTTP Error: ${response.status}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// Usage example
interface UserProfile {
  id: number;
  name: string;
  email: string;
}

const profile$ = fetchWithAuth<UserProfile>('https://api.example.com/profile');

profile$.subscribe({
  next: profile => console.log('Profile:', profile),
  error: error => console.error('Error:', error)
});
```

### 2. File Download (Blob)

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const downloadFile$ = fromFetch('https://api.example.com/files/report.pdf').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    // Retrieve as Blob
    return response.blob();
  })
);

downloadFile$.subscribe({
  next: blob => {
    // Generate download link from Blob
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'report.pdf';
    a.click();
    window.URL.revokeObjectURL(url);
    console.log('Download complete');
  },
  error: error => console.error('Download error:', error)
});
```

### 3. GraphQL Query

```typescript
import { switchMap, map } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface GraphQLResponse<T> {
  data?: T;
  errors?: Array<{ message: string }>;
}

interface User {
  id: string;
  name: string;
  email: string;
}

function graphqlQuery<T>(query: string, variables?: any): Observable<T> {
  return fromFetch('https://api.example.com/graphql', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ query, variables })
  }).pipe(
    switchMap(response => {
      if (!response.ok) {
        throw new Error(`HTTP Error: ${response.status}`);
      }
      return response.json() as Promise<GraphQLResponse<T>>;
    }),
    map(result => {
      if (result.errors) {
        throw new Error(result.errors.map(e => e.message).join(', '));
      }
      if (!result.data) {
        throw new Error('No data returned');
      }
      return result.data;
    })
  );
}

// Usage example
const query = `
  query GetUser($id: ID!) {
    user(id: $id) {
      id
      name
      email
    }
  }
`;

const user$ = graphqlQuery<{ user: User }>(query, { id: '1' });

user$.subscribe({
  next: ({ user }) => console.log('User:', user),
  error: error => console.error('Error:', error)
});
```

### 4. API with Pagination

```typescript
import { expand, takeWhile, reduce, switchMap, Observable } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface PaginatedResponse<T> {
  data: T[];
  page: number;
  totalPages: number;
}

function fetchAllPages<T>(baseUrl: string): Observable<T[]> {
  return fromFetch(`${baseUrl}?page=1`).pipe(
    switchMap(response => response.json() as Promise<PaginatedResponse<T>>),
    expand(response =>
      response.page < response.totalPages
        ? fromFetch(`${baseUrl}?page=${response.page + 1}`).pipe(
            switchMap(res => res.json() as Promise<PaginatedResponse<T>>)
          )
        : []
    ),
    takeWhile(response => response.page <= response.totalPages, true),
    reduce((acc, response) => [...acc, ...response.data], [] as T[])
  );
}

// Usage example
interface Item {
  id: number;
  name: string;
}

const allItems$ = fetchAllPages<Item>('https://api.example.com/items');

allItems$.subscribe({
  next: items => console.log('All items:', items),
  error: error => console.error('Error:', error)
});
```

## fromFetch() Options

`fromFetch()` can use the `RequestInit` option of the Fetch API without modification.

```typescript
interface RequestInit {
  method?: string;              // HTTP method (GET, POST, PUT, DELETE, etc.)
  headers?: HeadersInit;        // Request header
  body?: BodyInit | null;       // Request body
  mode?: RequestMode;           // cors, no-cors, same-origin
  credentials?: RequestCredentials; // omit, same-origin, include
  cache?: RequestCache;         // Cache mode
  redirect?: RequestRedirect;   // Redirect processing
  referrer?: string;            // Referrer
  integrity?: string;           // Subresource integrity
  signal?: AbortSignal;         // AbortController signal
}
```

## ajax() vs fromFetch() Comparison

| Feature | ajax() | fromFetch() |
|------|--------|-------------|
| Base Technology | XMLHttpRequest | Fetch API |
| Automatic JSON parsing | ✅ `getJSON()` | ❌ Manually `.json()` |
| Automatic HTTP error detection | ✅ Automatic error in 4xx/5xx | ❌ Manual `response.ok` check |
| Progress monitoring | ✅ | ❌ |
| Timeout | ✅ Built-in | ❌ Implemented with RxJS `timeout()` |
| Request cancellation | ✅ unsubscribe() | ✅ unsubscribe() or AbortController |
| IE11 support | ✅ | ❌ polyfill required |
| Bundle size | Slightly larger | Smaller |
| Service Worker support | ❌ | ✅ |

> [!TIP]
> **How to Choose**
>
> - **Modern browsers only**: `fromFetch()` recommended
> - **Need legacy browser support**: Use `ajax()`
> - **Progress monitoring required**: Use `ajax()`
> - **Lightweight HTTP communication**: `fromFetch()` is best
> - **Use in Service Worker**: Only `fromFetch()` supported

## Common Errors and Workarounds

### 1. HTTP Error Not Caught in `error` Callback

**Problem:**
```typescript
// ❌ next is called even on 404 error
fromFetch('https://api.example.com/not-found').subscribe({
  next: response => console.log('Success:', response), // ← Called even on 404
  error: error => console.error('Error:', error)
});
```

**Solution:**
```typescript
// ✅ Manually check response.ok
fromFetch('https://api.example.com/not-found').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
).subscribe({
  next: data => console.log('Data:', data),
  error: error => console.error('Error:', error) // ← This is called
});
```

### 2. CORS Error

**Solutions:**
- Set CORS headers on server side
- Explicitly specify `mode: 'cors'`
- Use proxy server during development

```typescript
fromFetch('https://api.example.com/data', {
  mode: 'cors',
  credentials: 'include' // If including cookies
});
```

### 3. Implementing Timeout

Fetch API does not have timeout functionality, so use RxJS `timeout()`.

```typescript
import { timeout, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow').pipe(
  timeout(5000), // Timeout in 5 seconds
  switchMap(response => response.json())
);
```

## Best Practices

### 1. Create Generic fetchJSON Function

```typescript
function fetchJSON<T>(url: string, options?: RequestInit): Observable<T> {
  return fromFetch(url, options).pipe(
    switchMap(response => {
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }
      return response.json() as Promise<T>;
    })
  );
}
```

### 2. Utilize TypeScript Types

```typescript
// ✅ Good example: Explicitly specify type
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Bad example: No type specified
const todo$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1')
  .pipe(switchMap(res => res.json()));
```

### 3. Always Implement Error Handling

```typescript
// ✅ Good example: response.ok and catchError
const api$ = fromFetch('/api/data').pipe(
  switchMap(response => {
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return response.json();
  }),
  catchError(error => {
    console.error('Error:', error);
    return of(defaultValue);
  })
);
```

### 4. Remember to Unsubscribe

```typescript
// ✅ Good example: Automatic release with takeUntil
class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    fromFetch('/api/data')
      .pipe(
        switchMap(res => res.json()),
        takeUntil(this.destroy$)
      )
      .subscribe(...);
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Summary

`fromFetch()` is a lightweight Creation Function for HTTP communication based on the modern Fetch API.

**Key Features:**
- Based on Fetch API and compliant with the latest web standards
- Lightweight and small bundle size
- Can be used within a Service Worker
- Manual error checking and response parsing required

**Usage Scenarios:**
- When supporting only modern browsers
- When bundle size needs to be reduced
- When HTTP communication is performed within a Service Worker
- When you want to use Fetch API functions (e.g., Request/Response objects) directly

**Important Notes:**
- The `error` callback is not called on HTTP errors (check `response.ok` manually)
- JSON parsing is done manually (`response.json()`)
- Progress monitoring is not supported
- polyfill is required for legacy browsers such as IE11

**Recommended Usage:**
- Create a generic `fetchJSON()` function for reuse
- Ensure type safety by utilizing TypeScript types
- Always implement error handling
- Always unsubscribe when no longer needed

## Related Pages

- [ajax()](/en/guide/creation-functions/http-communication/ajax) - XMLHttpRequest based HTTP communication
- [HTTP Communication Creation Functions](/en/guide/creation-functions/http-communication/) - ajax() vs. fromFetch()
- [switchMap()](/en/guide/operators/transformation/switchMap) - Useful operator for canceling HTTP communication
- [Error Handling Strategies](/en/guide/error-handling/strategies) - Error handling patterns for HTTP communication

## References

- [RxJS Official Documentation - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API)
- [MDN Web Docs - AbortController](https://developer.mozilla.org/en-US/docs/Web/API/AbortController)
