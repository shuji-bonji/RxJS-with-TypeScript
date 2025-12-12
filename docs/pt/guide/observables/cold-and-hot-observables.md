---
description: This article details the differences between Cold Observable and Hot Observable. Practical application examples are presented, including the independence of the data stream per subscription, how to convert Cold to Hot using share and shareReplay, and caching of API requests.
---
# Cold Observable and Hot Observable

One of the key concepts in using RxJS is the distinction between "Cold Observable" and "Hot Observable". Understanding this distinction is essential to learning how to use Observable efficiently.

## Why Understanding Cold/Hot is Important

If you do not understand the Cold/Hot distinction, you will face the following problems:

- **Unintended Duplicate Execution** - API calls are executed multiple times
- **Memory leaks** - Subscriptions are not properly managed
- **Performance issues** - Unnecessary processing is repeated
- **Data inconsistencies** - Expected data not received

## Cold vs Hot Differences (Comparison Chart)

First, let's get the big picture.

| Comparison Item | Cold Observable | Hot Observable |
|----------|------------------|----------------|
| **Execution without subscription** | Not executed (executed only when subscribed) | Executed (flows values even without subscribe) |
| **Data publishing timing** | Starts when `subscribe()` is called | Starts at publisher's timing (independent of subscription) |
| **Execution reuse** | Executed anew each time | Shares existing stream with multiple subscribers |
| **Data consistency** | Each subscription receives independent values | Cannot receive past values if subscribed mid-stream |
| **Main use cases** | HTTP requests, asynchronous processing | UI events, WebSocket, real-time communication |
| **Usage scenarios** | When each process is independent | State sharing, event broadcasting |

**Criteria:** Should processing be re-run for each subscriber? Or should the stream be shared?

## Cold vs. Hot Decision Criteria

The following criteria can be used to determine whether an Observable is actually Cold or Hot:

| Decision Point | Cold | Hot |
|-------------|------|-----|
| **Is execution logic re-run for each subscription?** | ✅ Re-run every time | ❌ Share execution |
| **Does data flow before subscription?** | ❌ Waits until subscribed | ✅ Flows independently of subscription |
| **Do multiple subscriptions receive the same data?** | ❌ Independent data | ✅ Share the same data |

### Practical Ways to Distinguish

The following test can easily determine:

```typescript
const observable$ = /* Observable to examine */;

observable$.subscribe(/* subscription 1 */);
observable$.subscribe(/* subscription 2 */);

// ✅ Cold: console.log inside Observable executes twice
//         (execution logic re-runs for each subscription)
// ✅ Hot:  console.log inside Observable executes only once
//         (execution is shared)
```

**Concrete Example:**

```typescript
import { Observable, Subject } from 'rxjs';

// Cold Observable
const cold$ = new Observable(subscriber => {
  console.log('Cold: Execution started');
  subscriber.next(Math.random());
});

cold$.subscribe(v => console.log('Subscription 1:', v));
cold$.subscribe(v => console.log('Subscription 2:', v));
// Output:
// Cold: Execution started  ← 1st time
// Subscription 1: 0.123...
// Cold: Execution started  ← 2nd time (re-executed)
// Subscription 2: 0.456...

// Hot Observable
const hot$ = new Subject();

hot$.subscribe(v => console.log('Subscription 1:', v));
hot$.subscribe(v => console.log('Subscription 2:', v));
hot$.next(1); // Data published only once
// Output:
// Subscription 1: 1
// Subscription 2: 1  ← Shares the same data
```

## Cold/Hot Classification Table by Creation Function

This table classifies Cold/Hot for all major Creation Functions. This allows you to see at a glance which function produces which Observable.

| Category | Creation Function | Cold/Hot | Notes |
|---------|-------------------|----------|------|
| **Basic creation** | `of()` | ❄️ Cold | Re-publishes values for each subscription |
| | `from()` | ❄️ Cold | Re-executes array/Promise for each subscription |
| | `fromEvent()` | ❄️ Cold | Adds independent listener for each subscription [^fromEvent] |
| | `interval()` | ❄️ Cold | Independent timer for each subscription |
| | `timer()` | ❄️ Cold | Independent timer for each subscription |
| **Loop generation** | `range()` | ❄️ Cold | Regenerates range for each subscription |
| | `generate()` | ❄️ Cold | Re-executes loop for each subscription |
| **HTTP communication** | `ajax()` | ❄️ Cold | New HTTP request for each subscription |
| | `fromFetch()` | ❄️ Cold | New Fetch request for each subscription |
| **Combination** | `concat()` | ❄️ Cold | Inherits nature of source Observables [^combination] |
| | `merge()` | ❄️ Cold | Inherits nature of source Observables [^combination] |
| | `combineLatest()` | ❄️ Cold | Inherits nature of source Observables [^combination] |
| | `zip()` | ❄️ Cold | Inherits nature of source Observables [^combination] |
| | `forkJoin()` | ❄️ Cold | Inherits nature of source Observables [^combination] |
| **Selection/Partition** | `race()` | ❄️ Cold | Inherits nature of source Observables [^combination] |
| | `partition()` | ❄️ Cold | Inherits nature of source Observables [^combination] |
| **Conditional** | `iif()` | ❄️ Cold | Inherits nature of conditionally selected Observable |
| | `defer()` | ❄️ Cold | Executes factory function for each subscription |
| **Control** | `scheduled()` | ❄️ Cold | Inherits nature of source Observable |
| | `using()` | ❄️ Cold | Creates resource for each subscription |
| **Subject family** | `new Subject()` | 🔥 Hot | Always Hot |
| | `new BehaviorSubject()` | 🔥 Hot | Always Hot |
| | `new ReplaySubject()` | 🔥 Hot | Always Hot |
| | `new AsyncSubject()` | 🔥 Hot | Always Hot |
| **WebSocket** | `webSocket()` | 🔥 Hot | Shares WebSocket connection |

[^fromEvent]: `fromEvent()` is Cold because it adds an independent event listener for each subscription. However, the event itself occurs independently of subscription, so it is easily misunderstood as Hot.

[^combination]: Combination Creation Functions are Cold if the source Observable is Cold, and Hot if it is Hot. Usually, Cold Observables are combined.

> [!IMPORTANT] Key Principle
> **Almost all Creation Functions generate Cold.**
> Only the following generate Hot:
> - Subject family (Subject, BehaviorSubject, ReplaySubject, AsyncSubject)
> - webSocket()

## Cold Observable

### Characteristics

- **A new data stream is created each time a subscription is made**
- **Data publishing does not start until subscribed (lazy execution)**
- **All subscribers receive all data from the beginning of the Observable**

Cold Observable creates a new execution context each time you subscribe.
This is appropriate for HTTP requests, asynchronous processing, and other situations where new processing is required each time.

### Code Example

```typescript
import { Observable } from 'rxjs';

// Cold Observable example
const cold$ = new Observable<number>(subscriber => {
  console.log('Data source creation - new subscription');
  const randomValue = Math.random();
  subscriber.next(randomValue);
  subscriber.complete();
});

// First subscription
console.log('--- First subscription ---');
cold$.subscribe(value => console.log('Subscriber 1:', value));

// Second subscription (different data is generated)
console.log('--- Second subscription ---');
cold$.subscribe(value => console.log('Subscriber 2:', value));
```

#### Output
```sh
--- First subscription ---
Data source creation - new subscription
Subscriber 1: 0.259632...
--- Second subscription ---
Data source creation - new subscription  ← Re-executed
Subscriber 2: 0.744322...  ← Different value
```

> [!TIP] Important Point
> Each subscription executes "Data source creation" and generates different values.

### Common Cold Observables (How to Identify)

The following Observables are usually Cold:

```typescript
import { of, from, interval, timer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Creation Functions
of(1, 2, 3)                    // Cold
from([1, 2, 3])                // Cold
from(fetch('/api/data'))       // Cold

// Time operators
interval(1000)                 // Cold
timer(1000)                    // Cold

// HTTP requests
ajax('/api/users')             // Cold
```

> [!TIP] Rule
> Creation Functions, time operators, and HTTP requests are basically Cold

## Hot Observable

### Characteristics

- **Flows values even if not subscribed (runs with or without subscription)**
- **Only receives data from the point of subscription initiation onward**
- **One data source is shared by multiple subscribers**

With Hot Observable, the timing of stream publication is independent of subscription, and subscribers join in the middle of the stream.

### Code Example

```typescript
import { Subject } from 'rxjs';

// Hot Observable example (using Subject)
const hot$ = new Subject<number>();

// First subscription
console.log('--- Subscriber 1 starts ---');
hot$.subscribe(value => console.log('Subscriber 1:', value));

// Publish data
hot$.next(1);
hot$.next(2);

// Second subscription (late subscription)
console.log('--- Subscriber 2 starts ---');
hot$.subscribe(value => console.log('Subscriber 2:', value));

// Publish more data
hot$.next(3);
hot$.next(4);

hot$.complete();
```

#### Output
```sh
--- Subscriber 1 starts ---
Subscriber 1: 1
Subscriber 1: 2
--- Subscriber 2 starts ---
Subscriber 1: 3
Subscriber 2: 3  ← Subscription 2 joins from 3 (cannot receive 1, 2)
Subscriber 1: 4
Subscriber 2: 4
```

> [!TIP] Important Point
> Subscriber 2 joined mid-stream and cannot receive past values (1, 2).

### Common Hot Observables (How to Identify)

The following Observables are always Hot:

```typescript
import { Subject, BehaviorSubject, ReplaySubject } from 'rxjs';
import { webSocket } from 'rxjs/webSocket';

// Subject family (always Hot)
new Subject()                  // Hot
new BehaviorSubject(0)         // Hot
new ReplaySubject(1)           // Hot

// WebSocket (always Hot)
webSocket('ws://localhost:8080') // Hot
```

> [!TIP] Rule
> **Only Subject family and webSocket() generate Hot**

> [!WARNING] fromEvent() is Cold
> `fromEvent(button, 'click')` is often misunderstood as Hot, but it is actually **Cold**. It adds an independent event listener for each subscription. The event itself occurs independently of subscription, but each subscriber has an independent listener.

## How to Convert Cold Observable to Hot

In RxJS, the following are the primary means of converting Cold Observable to Hot:

- `share()` - Simple hot conversion (recommended)
- `shareReplay()` - Cache past values and convert to hot
- ~~`multicast()`~~ - Deprecated (deprecated in RxJS v7, removed in v8)

### share() Operator

`share()` is the most common way to convert a Cold Observable to a Hot Observable.

```typescript
import { interval } from 'rxjs';
import { share, take } from 'rxjs';

// Simulate HTTP call
const makeHttpRequest = () => {
  console.log('HTTP call executed!');
  return interval(1000).pipe(take(3));
};

// ❌ Cold Observable (no sharing)
const cold$ = makeHttpRequest();

cold$.subscribe(val => console.log('Subscriber 1:', val));
cold$.subscribe(val => console.log('Subscriber 2:', val));
// → HTTP call executed twice

// ✅ Hot Observable (using share)
const shared$ = makeHttpRequest().pipe(share());

shared$.subscribe(val => console.log('Shared subscriber 1:', val));
shared$.subscribe(val => console.log('Shared subscriber 2:', val));
// → HTTP call executed only once, result is shared
```

**Output (Cold):**
```sh
HTTP call executed!  ← 1st time
Subscriber 1: 0
HTTP call executed!  ← 2nd time (duplicate!)
Subscriber 2: 0
...
```

**Output (Hot):**
```sh
HTTP call executed!  ← Only once
Shared subscriber 1: 0
Shared subscriber 2: 0  ← Shares the same stream
...
```

> [!NOTE] Use Cases
> - Use the same API results in multiple components
> - Avoid duplicate side effects (e.g., HTTP calls)

### shareReplay() Operator

`shareReplay()` is an extension of `share()` that **caches** past values and replays them to new subscribers.

```typescript
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

const request$ = interval(1000).pipe(
  take(3),
  shareReplay(2)  // Cache the last 2 values
);

// First subscription
request$.subscribe(val => console.log('Subscriber 1:', val));

// Second subscription after 3.5 seconds (after stream completion)
setTimeout(() => {
  console.log('--- Subscriber 2 starts (after completion) ---');
  request$.subscribe(val => console.log('Subscriber 2:', val));
}, 3500);
```

#### Output
```sh
Subscriber 1: 0
Subscriber 1: 1
Subscriber 1: 2
--- Subscriber 2 starts (after completion) ---
Subscriber 2: 1  ← Cached value (last 2)
Subscriber 2: 2  ← Cached value
```

> [!NOTE] Use Cases
> - Cache API results
> - Share initial state (cache only the latest one)
> - Provide historical data to late subscribers

> [!WARNING] Notes on shareReplay
> `shareReplay()` continues to hold the cache even when subscriptions go to 0, which may cause memory leaks. See [Chapter 10: Misuse of shareReplay](/pt/guide/anti-patterns/common-mistakes#4-sharereplay-misuse) for more information.

### About multicast()

> [!NOTE]
> `multicast()` is flexible, but was deprecated in RxJS v7 and removed in v8. Use `share()` or `shareReplay()` now. See [share() operator description](/pt/guide/operators/multicasting/share) for more information.

## Practical Example: API Cache Service

Common pattern in real applications: multiple components need the same API data.

```typescript
import { Observable, of, throwError } from 'rxjs';
import { catchError, shareReplay, delay, tap } from 'rxjs';

// Simple cache service
class UserService {
  private cache$: Observable<User[]> | null = null;

  getUsers(): Observable<User[]> {
    // Return cache if it exists
    if (this.cache$) {
      console.log('Returning from cache');
      return this.cache$;
    }

    // Create new request and cache
    console.log('Executing new request');
    this.cache$ = this.fetchUsersFromAPI().pipe(
      catchError(err => {
        this.cache$ = null;  // Clear cache on error
        return throwError(() => err);
      }),
      shareReplay(1)  // Cache the last result
    );

    return this.cache$;
  }

  private fetchUsersFromAPI(): Observable<User[]> {
    // Simulate actual API request
    return of([
      { id: 1, name: 'Taro Yamada' },
      { id: 2, name: 'Hanako Sato' }
    ]).pipe(
      delay(1000),
      tap(() => console.log('Data received from API'))
    );
  }

  clearCache(): void {
    this.cache$ = null;
    console.log('Cache cleared');
  }
}

interface User {
  id: number;
  name: string;
}

// Usage example
const userService = new UserService();

// Component 1: Request data
userService.getUsers().subscribe(users =>
  console.log('Component 1:', users)
);

// Component 2: Request data after 2 seconds
setTimeout(() => {
  userService.getUsers().subscribe(users =>
    console.log('Component 2:', users)
  );
}, 2000);

// Clear cache and request again
setTimeout(() => {
  userService.clearCache();
  userService.getUsers().subscribe(users =>
    console.log('Component 3:', users)
  );
}, 4000);
```

#### Output
```sh
Executing new request
Data received from API
Component 1: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
Returning from cache  ← No API call
Component 2: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
Cache cleared
Executing new request  ← API call again
Data received from API
Component 3: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
```

**Points:**
- Cache the last response with `shareReplay(1)`
- Multiple components share data (only one API call)
- Discard cache appropriately on error or clear

## When to Use

<div class="comparison-cards">

::: tip Cold
#### When to use
- When each subscriber needs its own dataset
- When representing a newly starting process or action
- When duplicate side effects are not a problem

#### Examples
- Send a new POST request for each form submission
- Different timer needed for each user
- Execute independent calculation for each subscription
:::

::: tip Hot
#### When to use
- When sharing data among multiple components
- When you want to save resources (e.g., reduce HTTP call count)
- When representing event streams
- State management or inter-service communication

#### Examples
- Configuration information shared across the application
- User login state
- Real-time messages (WebSocket)
- DOM events (click, scroll, etc.)
:::

</div>

## Summary

Understanding and properly using Cold Observable and Hot Observable is an important skill for building efficient RxJS applications.

::: tip Key Points
- **Cold Observable**: A stream that starts running only after being subscribed (independent execution per subscription)
- **Hot Observable**: Shares a stream that is already running (same execution for multiple subscriptions)
- **share()**: Easiest way to convert Cold to Hot
- **shareReplay()**: Cache past values and convert to Hot (useful for sharing API results)
:::

::: tip Design Decision Criteria
- Do you need to share data among multiple subscribers?
- Is it necessary to cache past values and provide them to new subscribers?
- How will duplicate side effects (e.g., HTTP requests) be managed?
:::

Based on these considerations, selecting the appropriate Observable type and operator will help you build an efficient and robust reactive application.

## Related Sections

- **[share() operator](/pt/guide/operators/multicasting/share)** - Detailed explanation of share()
- **[Misuse of shareReplay](/pt/guide/anti-patterns/common-mistakes#4-sharereplay-misuse)** - Common mistakes and solutions
- **[Subject](/pt/guide/subjects/what-is-subject)** - Understanding Hot Subjects

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>
