---
description: This section explains in detail the characteristics of each of the four types of Subject (Subject, BehaviorSubject, ReplaySubject, and AsyncSubject), the situations in which they are used, and a comparison of how to choose one.
---

# Types of Subject

In addition to the basic `Subject`, RxJS provides several derived classes specialized for specific use cases. Each of them has different behavioral characteristics and can be utilized in appropriate situations for more effective reactive programming.

Here, the four main types of Subject, their characteristics, and usage scenarios are explained in detail.

## Four Basic Types of Subjects

| Type | Features | Main Use Cases |
|------|------|----------------|
| [`Subject`](#subject) | Simplest Subject<br>Receive only values after subscription | Event notification, multicast |
| [`BehaviorSubject`](#behaviorsubject) | Keep the latest value and provide it immediately upon new subscription | Status management, current value of UI components |
| [`ReplaySubject`](#replaysubject) | Replay specified number of past values to new subscribers | Operation history, recent updates |
| [`AsyncSubject`](#asyncsubject) | Publish only the last value upon completion | Result of HTTP/API request |

## Standard `Subject` {#subject}

[📘 RxJS Official: Subject](https://rxjs.dev/api/index/class/Subject)

This is the simplest type of Subject and receives only values that occur after subscription.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

// No initial value, nothing received on subscription
subject.subscribe(value => console.log('Observer 1:', value));

subject.next(1);
subject.next(2);

// Second subscription (only receives values after subscription)
subject.subscribe(value => console.log('Observer 2:', value));

subject.next(3);
subject.complete();
```

#### Execution Result
```
Observer 1: 1
Observer 1: 2
Observer 1: 3
Observer 2: 3
```

## BehaviorSubject  {#behaviorsubject}

[📘 RxJS Official: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Requires an initial value and always keeps the latest value.
New subscribers receive the latest value immediately upon subscription.

```ts
import { BehaviorSubject } from 'rxjs';

// Create with initial value 0
const behaviorSubject = new BehaviorSubject<number>(0);

// Immediately receives initial value
behaviorSubject.subscribe(value => console.log('Observer 1:', value));

behaviorSubject.next(1);
behaviorSubject.next(2);

// Second subscription (immediately receives latest value 2)
behaviorSubject.subscribe(value => console.log('Observer 2:', value));

behaviorSubject.next(3);
behaviorSubject.complete();
```

#### Execution Result
```
Observer 1: 0
Observer 1: 1
Observer 1: 2
Observer 2: 2
Observer 1: 3
Observer 2: 3
```

### BehaviorSubject Usage Examples

#### User Authentication Status Management

```ts
import { BehaviorSubject } from 'rxjs';

interface User {
  id: string;
  name: string;
}

// Initial value is null (not logged in)
const currentUser$ = new BehaviorSubject<User | null>(null);

// Monitor login status in components, etc.
currentUser$.subscribe(user => {
  if (user) {
    console.log(`Logged in: ${user.name}`);
  } else {
    console.log('Not logged in');
  }
});

// Login process
function login(user: User) {
  currentUser$.next(user);
}

// Logout process
function logout() {
  currentUser$.next(null);
}

// Usage example
console.log('Application started');
// → Not logged in

login({ id: 'user123', name: 'Taro Yamada' });
// → Logged in: Taro Yamada

logout();
// → Not logged in
```

#### Execution Result
```sh
Not logged in
Application started
Logged in: Taro Yamada
Not logged in
```

## `ReplaySubject` {#replaysubject}
[📘 RxJS Official: ReplaySubject](https://rxjs.dev/api/index/class/ReplaySubject)

Memorize a specified number of previous values and resend to new subscribers.
Configurable buffer size and time window.

```ts
import { ReplaySubject } from 'rxjs';

// Buffer the last 3 values
const replaySubject = new ReplaySubject<number>(3);

replaySubject.next(1);
replaySubject.next(2);
replaySubject.next(3);
replaySubject.next(4);

// Start subscription (receive last 3 values: 2, 3, 4)
replaySubject.subscribe(value => console.log('Observer 1:', value));

replaySubject.next(5);

// Second subscription (receive last 3 values: 3, 4, 5)
replaySubject.subscribe(value => console.log('Observer 2:', value));

replaySubject.complete();
```

#### Execution Result
```
Observer 1: 2
Observer 1: 3
Observer 1: 4
Observer 1: 5
Observer 2: 3
Observer 2: 4
Observer 2: 5
```

### ReplaySubject with Time Window

Time-based buffering is also available.

```ts
import { ReplaySubject } from 'rxjs';

// Buffer up to 5 values within 500ms
const timeWindowSubject = new ReplaySubject<number>(5, 500);

timeWindowSubject.next(1);

setTimeout(() => {
  timeWindowSubject.next(2);

  // Subscribe after 1000ms (1 is not received because it exceeded the 500ms time window)
  setTimeout(() => {
    timeWindowSubject.subscribe(value => console.log('Received:', value));
  }, 1000);
}, 100);
```

#### Execution Result
```
Received: 2
```

### ReplaySubject Usage Examples

#### Manage Recent Search History

```ts
import { ReplaySubject } from 'rxjs';

// Keep the latest 5 search queries
const searchHistory$ = new ReplaySubject<string>(5);

// Search execution function
function search(query: string) {
  console.log(`Search executed: ${query}`);
  searchHistory$.next(query);
  // Actual search process...
}

// Component that displays search history
function showSearchHistory() {
  console.log('--- Search History ---');
  searchHistory$.subscribe(query => {
    console.log(query);
  });
}

// Usage example
search('TypeScript');
search('RxJS');
search('Angular');
search('React');

showSearchHistory();
// Display the latest 5 items (4 items in this case)
```

#### Execution Result
```sh
Search executed: TypeScript
Search executed: RxJS
Search executed: Angular
Search executed: React
--- Search History ---
TypeScript
RxJS
Angular
React
```

## `AsyncSubject` {#asyncsubject}
[📘 RxJS Official: AsyncSubject](https://rxjs.dev/api/index/class/AsyncSubject)

Only the last value is issued on completion. The value before completion will not be issued.

```ts
import { AsyncSubject } from 'rxjs';

const asyncSubject = new AsyncSubject<number>();

asyncSubject.subscribe(value => console.log('Observer 1:', value));

asyncSubject.next(1);
asyncSubject.next(2);
asyncSubject.next(3);

// Receive only the last value, regardless of subscription timing
asyncSubject.subscribe(value => console.log('Observer 2:', value));

asyncSubject.next(4);
asyncSubject.complete(); // Last value (4) is issued on completion
```

#### Execution Result
```
Observer 1: 4
Observer 2: 4
```

### AsyncSubject Usage Examples

#### API Request Result Sharing

```ts
import { AsyncSubject } from 'rxjs';

interface ApiResponse {
  data: any;
  status: number;
}

function fetchData(url: string) {
  const subject = new AsyncSubject<ApiResponse>();

  // Simulate API request
  console.log(`API request: ${url}`);
  setTimeout(() => {
    const response = {
      data: { id: 1, name: 'Sample data' },
      status: 200
    };

    subject.next(response);
    subject.complete();
  }, 1000);

  return subject;
}

// Usage example
const data$ = fetchData('/api/users/1');

// Multiple components can share the same request result
data$.subscribe(response => {
  console.log('Component 1:', response.data);
});

setTimeout(() => {
  data$.subscribe(response => {
    console.log('Component 2:', response.data);
  });
}, 1500); // Can receive value even after completion
```

#### Execution Result
```sh
API request: /api/users/1
Component 1: {id: 1, name: 'Sample data'}
Component 2: {id: 1, name: 'Sample data'}
```

## Comparison and Selection Guide for Each Subject

Here is a summary of points to help you choose each Subject type.

### How to Choose a Subject

|Type|Selection Criteria|
|---|---|
|`Subject`|Used for simple event notification and multicast distribution|
|`BehaviorSubject`|<li>Cases where initial values are always needed </li><li>Data representing current state (user state, settings, flags, etc.) </li><li>Current values of UI components</li>|
|`ReplaySubject`|<li>Cases where the most recent operation history needs to be kept </li><li>Cases where historical data needs to be provided for subscribers who joined later </li><li>Buffered data streams</li>|
|`AsyncSubject`|<li>When only the end result is important (e.g., API responses) </li><li>When no intermediate steps are needed and only the value at completion is to be shared</li>|

### Selection Decision Flow

1. Only the last value at completion is required ⇨ `AsyncSubject`
2. Last N values required ⇨ `ReplaySubject`
3. Always need current state/value ⇨ `BehaviorSubject`
4. Other (e.g. pure event notification) ⇨ `Subject`

## Utilization Patterns in Application Design

### Example of Inter-Module Communication

```ts
// Application-wide state management service
class AppStateService {
  // Current user (BehaviorSubject because initial value is required)
  private userSubject = new BehaviorSubject<User | null>(null);
  // Expose as read-only Observable
  readonly user$ = this.userSubject.asObservable();

  // Notification (Subject because it's simple event notification)
  private notificationSubject = new Subject<Notification>();
  readonly notifications$ = this.notificationSubject.asObservable();

  // Recent searches (ReplaySubject because history is needed)
  private searchHistorySubject = new ReplaySubject<string>(10);
  readonly searchHistory$ = this.searchHistorySubject.asObservable();

  // API call result cache (AsyncSubject because only final result is needed)
  private readonly apiCaches = new Map<string, AsyncSubject<any>>();

  // Method examples
  setUser(user: User | null) {
    this.userSubject.next(user);
  }

  notify(notification: Notification) {
    this.notificationSubject.next(notification);
  }

  addSearch(query: string) {
    this.searchHistorySubject.next(query);
  }

  // Cache API results
  fetchData(url: string): Observable<any> {
    if (!this.apiCaches.has(url)) {
      const subject = new AsyncSubject<any>();
      this.apiCaches.set(url, subject);

      // Actual API call
      fetch(url)
        .then(res => res.json())
        .then(data => {
          subject.next(data);
          subject.complete();
        })
        .catch(err => {
          subject.error(err);
        });
    }

    return this.apiCaches.get(url)!.asObservable();
  }
}
```

## Summary

RxJS Subject is a powerful tool that can be used for a wide variety of use cases. By understanding the characteristics of each type and utilizing them appropriately, you can build efficient and maintainable reactive applications.

- `Subject`: Provides the simplest, most basic multicast functionality
- `BehaviorSubject`: keeps the current state at all times and provides it immediately to new subscribers
- `ReplaySubject`: keeps a history of the most recent values and offers them to subscribers who join later
- `AsyncSubject`: Publish only the last value on completion

Choosing the right `Subject` for every situation, including state management, event notification, and data sharing, is the key to efficient reactive programming.
