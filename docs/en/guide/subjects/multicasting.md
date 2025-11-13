---
description: Explains in detail how RxJS multicasting works, including basic patterns using Subject, using share and shareReplay operators, avoiding duplicate API request calls, caching strategies, sharing application state, and other practical design patterns with TypeScript code examples.
---

# Multicasting Mechanism

Multicasting is a method of efficiently distributing a stream of data from a single Observable to multiple subscribers (Observer).
In RxJS, this can be accomplished by Subject and operators.

## What is Multicasting?

A normal Observable (Cold Observable) creates a new data stream each time it is subscribed to. This means that if there are multiple subscribers, the same process will be executed multiple times.

With multicasting, a data source can be executed only once and the results distributed to multiple subscribers. This is especially important when:

- You do not want to invoke duplicate HTTP/API requests
- Want to perform a high-cost operation (computation or side-effect) only once
- Share application state with multiple components

## Basic Pattern of Multicasting

### Basic Multicast with Subject

```ts
import { Observable, Subject } from 'rxjs';
import { tap } from 'rxjs';

// Data source (Cold Observable)
function createDataSource(): Observable<number> {
  return new Observable<number>(observer => {
    console.log('Data source: Connected');
    // Data generation logic (assuming high-cost operation)
    const id = setInterval(() => {
      const value = Math.round(Math.random() * 100);
      console.log(`Data source: Generated value -> ${value}`);
      observer.next(value);
    }, 1000);

    // Cleanup function
    return () => {
      console.log('Data source: Disconnected');
      clearInterval(id);
    };
  });
}

// Multicast implementation
function multicast() {
  // Original data source
  const source$ = createDataSource().pipe(
    tap(value => console.log(`Source processing: ${value}`))
  );

  // Subject for multicasting
  const subject = new Subject<number>();

  // Connect source to Subject
  const subscription = source$.subscribe(subject);

  // Multiple subscribers subscribe to the Subject
  console.log('Observer 1 subscription started');
  const subscription1 = subject.subscribe(value => console.log(`Observer 1: ${value}`));

  // Add another subscriber after 3 seconds
  setTimeout(() => {
    console.log('Observer 2 subscription started');
    const subscription2 = subject.subscribe(value => console.log(`Observer 2: ${value}`));

    // Unsubscribe all after 5 seconds
    setTimeout(() => {
      console.log('All subscriptions terminated');
      subscription.unsubscribe();
      subscription1.unsubscribe();
      subscription2.unsubscribe();
    }, 5000);
  }, 3000);
}

// Execute
multicast();
```

#### Execution Result
```
Data source: Connected
Observer 1 subscription started
Data source: Generated value -> 71
Source processing: 71
Observer 1: 71
Data source: Generated value -> 79
Source processing: 79
Observer 1: 79
Data source: Generated value -> 63
Source processing: 63
Observer 1: 63
Observer 2 subscription started
Data source: Generated value -> 49
Source processing: 49
Observer 1: 49
Observer 2: 49
Data source: Generated value -> 94
Source processing: 94
Observer 1: 94
Observer 2: 94
Data source: Generated value -> 89
Source processing: 89
Observer 1: 89
Observer 2: 89
Data source: Generated value -> 10
Source processing: 10
Observer 1: 10
Observer 2: 10
Data source: Generated value -> 68
Source processing: 68
Observer 1: 68
Observer 2: 68
All subscriptions terminated
Data source: Disconnected
```

## Multicast Operators

RxJS provides dedicated operators to implement multicasting.

### `share()` Operator
[📘 RxJS Official: share()](https://rxjs.dev/api/index/function/share)

This is the easiest operator to implement multicast.
Internally, it combines `multicast()` and `refCount()`.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

// Observable counting at intervals
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source: ${value}`)),
  share() // Enable multicasting
);

// First subscriber
console.log('Observer 1 subscription started');
const subscription1 = source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Add second subscriber after 2.5 seconds
setTimeout(() => {
  console.log('Observer 2 subscription started');
  const subscription2 = source$.subscribe(value => console.log(`Observer 2: ${value}`));

  // Unsubscribe subscriber 1 after 5 seconds
  setTimeout(() => {
    console.log('Observer 1 unsubscribed');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

#### Execution Result
```
Observer 1 subscription started
Source: 0
Observer 1: 0
Observer 2 subscription started
Source: 1
Observer 1: 1
Observer 2: 1
Source: 2
Observer 1: 2
Observer 2: 2
Source: 3
Observer 1: 3
Observer 2: 3
Observer 1 unsubscribed
Source: 4
Observer 2: 4
```

### Detailed Control of `share()`

Instead of `refCount()`, you can control the behavior more explicitly by passing options to `share()` in RxJS 7 and later.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Source: ${value}`)),
  share({
    resetOnError: true,
    resetOnComplete: true,
    resetOnRefCountZero: true,
  })
);

// First subscriber
console.log('Observer 1 subscription started');
const subscription1 = source$.subscribe((value) =>
  console.log(`Observer 1: ${value}`)
);

// Add second subscriber after 2.5 seconds
setTimeout(() => {
  console.log('Observer 2 subscription started');
  const subscription2 = source$.subscribe((value) =>
    console.log(`Observer 2: ${value}`)
  );

  setTimeout(() => {
    console.log('Observer 1 unsubscribed');
    subscription1.unsubscribe();
  }, 1500);
}, 2500);
```

#### Execution Result
```
Observer 1 subscription started
Source: 0
Observer 1: 0
Source: 1
Observer 1: 1
Observer 2 subscription started
Source: 2
Observer 1: 2
Observer 2: 2
Source: 3
Observer 1: 3
Observer 2: 3
Observer 1 unsubscribed
Source: 4
Observer 2: 4
Source: 5
Observer 2: 5
```

This way, you can clearly control the behavior when the stream ends or when the number of subscribers reaches zero.

### `shareReplay()` Operator

[📘 RxJS Official: shareReplay()](https://rxjs.dev/api/index/function/shareReplay)

Similar to `share()`, but stores a specified number of historical values and makes them available to subscribers who join later.

```ts
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// Using shareReplay (buffer size 2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source: ${value}`)),
  shareReplay(2) // Buffer latest 2 values
);

// First subscriber
console.log('Observer 1 subscription started');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Add second subscriber after 3.5 seconds
setTimeout(() => {
  console.log('Observer 2 subscription started - receives latest 2 values');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

#### Execution Result
```
Observer 1 subscription started
Source: 0
Observer 1: 0
Source: 1
Observer 1: 1
Observer 2 subscription started - receives latest 2 values
Observer 2: 0
Observer 2: 1
Source: 2
Observer 1: 2
Observer 2: 2
Source: 3
Observer 1: 3
Observer 2: 3
Source: 4
Observer 1: 4
Observer 2: 4
```

## Timing and Lifecycle in Multicasting

It is important to understand the lifecycle of a multicast stream. In particular, when using the `share()` operator, the following behavior should be noted:

1. **First subscriber**: `share()` initiates a connection to the source Observable at the time the first subscription is made.
2. **All subscribers are unsubscribed**: If `share({ resetOnRefCountZero: true })` is set, the connection to the source will be disconnected when the number of subscribers reaches zero.
3. **Completion or error**: By default, `share()` resets its internal state upon `complete` or `error` (if `resetOnComplete`/`resetOnError` is `true`).
4. **Resubscription**: If the stream is reset and then re-subscribed, it will be rebuilt as a new Observable.

Thus, the `share()` options control when the stream starts, stops, and regenerates depending on the number of subscriptions and completion status.

## Practical Use Cases

### API Request Sharing

Example of avoiding duplicate requests to the same API endpoint.

```ts
import { Observable, of, throwError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, catchError, shareReplay, tap } from 'rxjs';

// API service simulation
class UserService {
  private cache = new Map<string, Observable<any>>();

  getUser(id: string): Observable<any> {
    // Return from cache if available
    if (this.cache.has(id)) {
      console.log(`Getting user ID ${id} from cache`);
      return this.cache.get(id)!;
    }

    // Create new request
    console.log(`Getting user ID ${id} from API`);
    const request$ = ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${id}`).pipe(
      tap(response => console.log('API response:', response)),
      catchError(error => {
        console.error('API error:', error);
        // Remove from cache
        this.cache.delete(id);
        return throwError(() => new Error('Failed to retrieve user'));
      }),
      // Share with shareReplay (cache value even after completion)
      shareReplay(1)
    );

    // Save to cache
    this.cache.set(id, request$);
    return request$;
  }
}

// Usage example
const userService = new UserService();

// Multiple components request the same user data
console.log('Component 1: Request user data');
userService.getUser('1').subscribe(user => {
  console.log('Component 1: Received user data', user);
});

// Another component requests the same data slightly later
setTimeout(() => {
  console.log('Component 2: Request same user data');
  userService.getUser('1').subscribe(user => {
    console.log('Component 2: Received user data', user);
  });
}, 1000);

// Request another user
setTimeout(() => {
  console.log('Component 3: Request different user data');
  userService.getUser('2').subscribe(user => {
    console.log('Component 3: Received user data', user);
  });
}, 2000);
```

#### Execution Result
```
Component 1: Request user data
Getting user ID 1 from API
API response: {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Component 1: Received user data {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Component 2: Request same user data
Getting user ID 1 from cache
Component 2: Received user data {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Component 3: Request different user data
Getting user ID 2 from API
API response: {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
Component 3: Received user data {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
```

## Design Patterns for Multicasting

### Singleton Observable

A pattern in which a single Observable is shared throughout the application.

```ts
import { Subject } from 'rxjs';

// Application-wide global state management
class AppState {
  // Singleton instance
  private static instance: AppState;

  // Global notification stream
  private notificationsSubject = new Subject<string>();

  // Public Observable (read-only)
  readonly notifications$ = this.notificationsSubject.asObservable();

  // Singleton access
  static getInstance(): AppState {
    if (!AppState.instance) {
      AppState.instance = new AppState();
    }
    return AppState.instance;
  }

  // Method to send notifications
  notify(message: string): void {
    this.notificationsSubject.next(message);
  }
}

// Usage example
const appState = AppState.getInstance();

// Monitor notifications (from multiple components)
appState.notifications$.subscribe((msg) =>
  console.log('Component A:', msg)
);
appState.notifications$.subscribe((msg) =>
  console.log('Component B:', msg)
);

// Send notification
appState.notify('System update available');
```

#### Execution Result
```ts
Component A: System update available
Component B: System update available
```

## Summary

Multicasting is an important technique to improve the efficiency and performance of RxJS applications. The main points are as follows:

- Multicasting allows a single data source to be shared by multiple subscribers
- Can be implemented using operators such as `share()`, `shareReplay()`, and `publish()`
- Can avoid duplicate API requests and optimize computationally expensive processes
- Useful for state management and communication between components

Choosing the right multicast strategy can reduce the amount of code and improve maintainability while increasing application responsiveness and efficiency.
