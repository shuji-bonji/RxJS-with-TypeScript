---
description: Robust, type-safe reactive design through custom operator type definitions, utilization of conditional types, and state management patterns in TypeScript and RxJS integration.
---

# TypeScript and RxJS Integration

This document presents various techniques and best practices for using RxJS type-safely in TypeScript.

## What you will learn in this document

- How to work with Observable types in TypeScript
- How to define custom operator types
- Using Conditional and Utility types
- Type-safe state management using RxJS
- Recommended settings for tsconfig.json and why

The combination of TypeScript and RxJS allows asynchronous programming while maintaining type safety.
This document will show you how to effectively utilize RxJS with TypeScript.

## Utilizing type definitions

### Specifying the Observable type

One of the greatest advantages of using RxJS with TypeScript is the ability to explicitly define the type of values that flow into an Observable.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

// Explicit type definition
const numbers$: Observable<number> = of(1, 2, 3);

// Transformation using generic types
interface User {
  id: number;
  name: string;
}

const users$: Observable<User> = of(
  { id: 1, name: '山田' },
  { id: 2, name: '佐藤' }
);

// Operation where types are transformed
const userNames$: Observable<string> = users$.pipe(
  map(user => user.name)
);
```

### Custom operator type definition

Types can also be handled properly when creating custom operators.

```ts
import { Observable, of, OperatorFunction } from 'rxjs';
import { map } from 'rxjs';

// Define type-safe custom operators using OperatorFunction
function doubleMap<T, R, S>(
  first: (value: T, index: number) => R,
  second: (value: R, index: number) => S
): OperatorFunction<T, S> {
  return (source: Observable<T>) => source.pipe(map(first), map(second));
}

// Usage example
of(1, 2, 3)
  .pipe(
    doubleMap(
      (x) => x * 2,
      (x) => `Result: ${x}`
    )
  )
  .subscribe(console.log);
// Result: 2
// Result: 4
// Result: 6
```

## Interfaces and type aliases

For complex event-driven designs, having the event structure type-defined makes integration with RxJS much more efficient.

When dealing with complex data structures, it is useful to define interfaces and type aliases.

```ts
// Event type definition
interface AppEvent {
  type: string;
  payload: unknown;
}

// Specific event types
interface UserLoginEvent extends AppEvent {
  type: 'USER_LOGIN';
  payload: {
    userId: string;
    timestamp: number;
  };
}

interface DataUpdateEvent extends AppEvent {
  type: 'DATA_UPDATE';
  payload: {
    items: Array<{ id: string; value: number }>;
  };
}

// Event type composition
type ApplicationEvent = UserLoginEvent | DataUpdateEvent;

// Event bus implementation
const eventBus$ = new Subject<ApplicationEvent>();

// Type-safe event publishing
eventBus$.next({
  type: 'USER_LOGIN',
  payload: {
    userId: 'user123',
    timestamp: Date.now(),
  },
});

// Type-safe event filtering
const userLoginEvents$ = eventBus$.pipe(
  filter((event): event is UserLoginEvent => event.type === 'USER_LOGIN')
);

userLoginEvents$.subscribe((event) => {
  // Here event.payload.userId can be accessed type-safely
  console.log(`User logged in: ${event.payload.userId}`);
});
```

## Utilizing advanced types

### Utility types

You can further enhance your integration with RxJS by taking advantage of TypeScript's utility types.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
  role: 'admin' | 'user';
}

// Select a subset of properties using Pick
type UserBasicInfo = Pick<User, 'id' | 'name'>;

const users$: Observable<User> = fetchUsers();
const usersBasicInfo$: Observable<UserBasicInfo> = users$.pipe(
  map(user => ({ id: user.id, name: user.name }))
);

// Use Omit to exclude specific properties
type UserPublicInfo = Omit<User, 'email'>;

// Use Partial to make all properties optional
type UserUpdate = Partial<User>;

function updateUser(id: number, update: UserUpdate): Observable<User> {
  return patchUser(id, update);
}
```

### Conditional and Mapping Types

For more complex cases, conditional and mapping types can be used.

```ts
import { filter, map, Observable} from 'rxjs';

// API response types
type ApiResponse<T> =
  | { status: 'success'; data: T; }
  | { status: 'error'; error: string; };

// Type that extracts only data from response
type ExtractData<T> = T extends ApiResponse<infer U> ? U : never;

function handleApiResponse<T>(response$: Observable<ApiResponse<T>>): Observable<T> {
  return response$.pipe(
    filter((response): response is ApiResponse<T> & { status: 'success' } =>
      response.status === 'success'
    ),
    map(response => response.data)
  );
}

// Usage example
const userResponse$: Observable<ApiResponse<User>> = fetchUserApi(1);
const user$: Observable<User> = handleApiResponse(userResponse$);
```

## Optimize tsconfig.json

Proper tsconfig.json configuration is important for effective use of RxJS and TypeScript.

```json
{
  "compilerOptions": {
    "target": "es2020",
    "module": "esnext",
    "moduleResolution": "node",
    "strict": true,
    "noImplicitAny": true,
    "strictNullChecks": true,
    "noUnusedLocals": true,
    "noUnusedParameters": true,
    "esModuleInterop": true,
    "sourceMap": true,
    "declaration": true,
    "lib": ["es2020", "dom"]
  }
}
```

The following settings are of particular importance

💡 In particular, `"strict": true` is essential to get the most out of RxJS.

- `strict`: strict type checking to take full advantage of RxJS type safety
- `noImplicitAny`: disallow implicit any types.
- `strictNullChecks`: null/undefined must be handled explicitly.

## Optimization of RxJS imports

When using RxJS in a TypeScript project, the import method is also important.

```ts
// Recommended method
import { Observable, of, from } from 'rxjs';
import { map, filter, catchError } from 'rxjs';
```

## RxJS pattern for state management (Redux-less configuration)

This section shows how to build state management using only RxJS without using Redux or NgRx.

```ts
// Application state interface
interface AppState {
  user: User | null;
  isLoading: boolean;
  data: Record<string, unknown>;
  error: Error | null;
}

// Initial state
const initialState: AppState = {
  user: null,
  isLoading: false,
  data: {},
  error: null
};

// BehaviorSubject for state management
const state$ = new BehaviorSubject<AppState>(initialState);

// Action types
type Action =
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_USER'; payload: User | null }
  | { type: 'SET_DATA'; payload: Record<string, unknown> }
  | { type: 'SET_ERROR'; payload: Error | null };

// State update function
function reducer(state: AppState, action: Action): AppState {
  switch (action.type) {
    case 'SET_LOADING':
      return { ...state, isLoading: action.payload };
    case 'SET_USER':
      return { ...state, user: action.payload };
    case 'SET_DATA':
      return { ...state, data: action.payload };
    case 'SET_ERROR':
      return { ...state, error: action.payload };
    default:
      return state;
  }
}

// Action dispatch
const actions$ = new Subject<Action>();

// State updates
actions$.pipe(
  scan(reducer, initialState)
).subscribe(state$);

// Usage example
actions$.next({ type: 'SET_LOADING', payload: true });

// Monitor part of the state
const isLoading$ = state$.pipe(
  map(state => state.isLoading),
  distinctUntilChanged()
);

isLoading$.subscribe(isLoading => {
  console.log(`Loading状態: ${isLoading}`);
});
```

## Examples of generic usage

More advanced generic types are useful in complex data flows.

```ts
// Service that wraps HTTP requests
class ApiService {
  // Generic method
  get<T>(url: string): Observable<T> {
    return fromFetch(url).pipe(
      switchMap(response => {
        if (response.ok) {
          return response.json() as Promise<T>;
        } else {
          return throwError(() => new Error(`Error ${response.status}`));
        }
      }),
      retry(3),
      catchError(err => this.handleError<T>(err))
    );
  }

  private handleError<T>(error: Error): Observable<T> {
    console.error('API error:', error);
    return EMPTY;
  }
}

// Usage example
interface Product {
  id: string;
  name: string;
  price: number;
}

const apiService = new ApiService();
const products$ = apiService.get<Product[]>('/api/products');

products$.subscribe(products => {
  // products are treated as type Product[]
  products.forEach(p => console.log(`${p.name}: ${p.price}円`));
});

// By using generic types in this way, API response types can be defined in a reusable form, making service classes more generic.
```

## Summary

The combination of TypeScript and RxJS offers the following advantages

- Type-safe asynchronous programming
- Increased development efficiency through IDE support
- Early detection of errors through compile-time type checking
- Self-documented code
- Refactoring safety

Development with RxJS can be further enhanced by utilizing proper type definitions and the advanced features of TypeScript.
