---
description: shareReplay is an RxJS multicast operator that buffers past values in addition to multicasting and provides them to delayed subscribers. It is ideal for situations where you want to remember past values and distribute them to multiple subscribers, such as API response caching, sharing configuration information, and state management. It is possible to prevent memory leaks with refCount and windowTime options, and TypeScript type inference enables type-safe cache processing.
---

# shareReplay - Cache and Share Past Values

The `shareReplay()` operator achieves multicasting like `share()`, but also **remembers a specified number of past values** and provides them to subscribers who join later.

This enables more advanced use cases such as API response caching and state sharing.

[📘 RxJS Official Documentation - `shareReplay()`](https://rxjs.dev/api/index/function/shareReplay)

## 🔰 Basic Usage

```typescript
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

### Execution Result

```
Observer 1 subscription started
Source: 0
Observer 1: 0
Source: 1
Observer 1: 1
Source: 2
Observer 1: 2
Source: 3
Observer 1: 3
Observer 2 subscription started - receives latest 2 values
Observer 2: 2  // ← Buffered past value
Observer 2: 3  // ← Buffered past value
Source: 4
Observer 1: 4
Observer 2: 4
```

**Important Points**:
- Delayed subscribers can immediately receive buffered past values
- Values up to the buffer size are remembered (2 in this example)

## 💡 shareReplay() Syntax

```typescript
shareReplay(bufferSize?: number, windowTime?: number, scheduler?: SchedulerLike)
shareReplay(config: ShareReplayConfig)
```

### Parameters

| Parameter | Type | Description | Default |
|-----------|---|------|----------|
| `bufferSize` | `number` | Number of values to buffer | `Infinity` |
| `windowTime` | `number` | Buffer validity period (milliseconds) | `Infinity` |
| `scheduler` | `SchedulerLike` | Scheduler for timing control | - |

### Configuration Object (RxJS 7+)

```typescript
interface ShareReplayConfig {
  bufferSize?: number;
  windowTime?: number;
  refCount?: boolean;  // Unsubscribe when subscriber count reaches zero
  scheduler?: SchedulerLike;
}
```

## 📊 Difference Between share and shareReplay

### share() Behavior

```typescript
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source: ${value}`)),
  share()
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Observer 2 subscription started');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Execution Result**:
```
Source: 0
Observer 1: 0
Source: 1
Observer 1: 1
Observer 2 subscription started
Source: 2
Observer 1: 2
Observer 2: 2  // ← Cannot receive past values (0, 1)
```

### shareReplay() Behavior

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source: ${value}`)),
  shareReplay(2) // Buffer latest 2 values
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Observer 2 subscription started');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Execution Result**:
```
Source: 0
Observer 1: 0
Source: 1
Observer 1: 1
Observer 2 subscription started
Observer 2: 0  // ← Buffered past value
Observer 2: 1  // ← Buffered past value
Source: 2
Observer 1: 2
Observer 2: 2
```

## 💼 Practical Use Cases

### 1. API Response Caching

```typescript
import { Observable } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, shareReplay, tap } from 'rxjs';

interface User {
  id: number;
  name: string;
  username: string;
  email: string;
}

class UserService {
  // Cache user information
  private userCache$ = ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
    tap(() => console.log('API request executed')),
    shareReplay(1) // Permanently cache latest 1 value
  );

  getUser(): Observable<User> {
    return this.userCache$;
  }
}

const userService = new UserService();

// First component
userService.getUser().subscribe(user => {
  console.log('Component 1:', user);
});

// Another component after 2 seconds
setTimeout(() => {
  userService.getUser().subscribe(user => {
    console.log('Component 2:', user); // ← Retrieved from cache, no API request
  });
}, 2000);
```

**Execution Result**:
```
API request executed
Component 1: { id: 1, name: "John" }
Component 2: { id: 1, name: "John" }  // ← No API request
```

### 2. Sharing Configuration Information

```typescript
import { of } from 'rxjs';
import { delay, shareReplay, tap } from 'rxjs';

// Get application settings (executed only once)
const appConfig$ = of({
  apiUrl: 'https://api.example.com',
  theme: 'dark',
  language: 'en'
}).pipe(
  delay(1000), // Simulate loading
  tap(() => console.log('Settings loaded')),
  shareReplay(1)
);

// Use settings in multiple services
appConfig$.subscribe(config => console.log('Service A:', config.apiUrl));
appConfig$.subscribe(config => console.log('Service B:', config.theme));
appConfig$.subscribe(config => console.log('Service C:', config.language));
```

**Execution Result**:
```
Settings loaded
Service A: https://api.example.com
Service B: dark
Service C: en
```

### 3. Time-Limited Cache

```typescript
import { ajax } from 'rxjs/ajax';
import { shareReplay, tap } from 'rxjs';

// Cache for 5 seconds only (using TODO data as example)
const todoData$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1').pipe(
  tap(() => console.log('TODO data retrieved')),
  shareReplay({
    bufferSize: 1,
    windowTime: 5000, // Valid for 5 seconds
    refCount: true    // Unsubscribe when subscriber count reaches zero
  })
);

// First subscription
todoData$.subscribe(data => console.log('Get 1:', data));

// After 3 seconds (cache valid)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Get 2:', data)); // From cache
}, 3000);

// After 6 seconds (cache expired)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Get 3:', data)); // New request
}, 6000);
```

## ⚠️ Beware of Memory Leaks

`shareReplay()` keeps values in a buffer, which can cause memory leaks if not properly managed.

### Problematic Code

```typescript
// ❌ Risk of memory leak
const infiniteStream$ = interval(1000).pipe(
  shareReplay() // bufferSize not specified = Infinity
);

// This stream continues to accumulate values forever
```

### Recommended Countermeasures

```typescript
// ✅ Limit buffer size
const safeStream$ = interval(1000).pipe(
  shareReplay(1) // Keep only latest 1
);

// ✅ Use refCount
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    refCount: true // Clear buffer when subscriber count reaches zero
  })
);

// ✅ Set time limit
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    windowTime: 10000 // Expires in 10 seconds
  })
);
```

## 🎯 Choosing Buffer Size

| Buffer Size | Use Case | Example |
|--------------|-----------|---|
| `1` | Only latest state needed | Current user info, settings |
| `3-5` | Recent history needed | Chat history, notification history |
| `Infinity` | All history needed | Logs, audit trails (use with caution) |

## 🔄 Related Operators

- **[share()](/pt/guide/operators/multicasting/share)** - Simple multicast (no buffering)
- **[publish()](/pt/guide/subjects/multicasting)** - Low-level multicast control
- **[ReplaySubject](/pt/guide/subjects/types-of-subject)** - Subject that forms the basis of shareReplay

## Summary

The `shareReplay()` operator:
- Buffers past values and provides them to delayed subscribers
- Ideal for API response caching
- Requires attention to memory leaks
- Can be used safely with `refCount` and `windowTime`

When state sharing or caching is needed, `shareReplay()` is a very powerful tool, but it's important to set appropriate buffer size and expiration settings.

## 🔗 Related Sections

- **[Common Mistakes and Solutions](/pt/guide/anti-patterns/common-mistakes#4-misuse-of-sharereplay)** - Proper use of shareReplay and memory leak countermeasures
- **[share()](/pt/guide/operators/multicasting/share)** - Simple multicast
- **[ReplaySubject](/pt/guide/subjects/types-of-subject)** - Subject that forms the basis of shareReplay
