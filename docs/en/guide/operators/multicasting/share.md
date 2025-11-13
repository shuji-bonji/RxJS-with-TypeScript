---
description: Describes how to implement multicasting using the share() operator. Describes how to share the same Observable with multiple subscribers to reduce duplicate processing and provides detailed control options.
---

# share - Share an Observable with Multiple Subscribers

The `share()` operator is the easiest multicasting operator to implement in RxJS.
Multiple subscribers can share the same data source to reduce duplicate processing (API requests, computation processing, etc.).

[📘 RxJS Official Documentation - `share()`](https://rxjs.dev/api/index/function/share)

## 🔰 Basic Usage

```typescript
import { interval, share, take, tap } from 'rxjs';

// Observable counting at intervals
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source: ${value}`)),
  share() // Enable multicasting
);

// First subscriber
console.log('Observer 1 subscription started');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1: ${value}`)
);

// Add second subscriber after 2.5 seconds
setTimeout(() => {
  console.log('Observer 2 subscription started');
  source$.subscribe(value =>
    console.log(`Observer 2: ${value}`)
  );

  // Unsubscribe subscriber 1 after 2.5 seconds
  setTimeout(() => {
    console.log('Observer 1 unsubscribed');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

### Execution Result

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
```

**Important Points**:
- Source processing (`tap`) is executed only once
- All subscribers receive the same value
- Subscribers who join in the middle will only receive values after joining

## 💡 How share() Works

`share()` is a standard RxJS multicasting operator. Internally, it uses Subject to broadcast to multiple subscribers.

> [!NOTE]
> **Changes in RxJS v7 and later**: Previously explained as a combination of `multicast()` and `refCount()`, these operators were deprecated in v7 and removed in v8. Currently, `share()` is the standard multicasting method. For details, see [RxJS Official Documentation - Multicasting](https://rxjs.dev/deprecations/multicasting).

**Flow of Operation**:
- **On first subscription**: Initiate a connection to the source Observable and create an internal Subject
- **Add subscribers**: Share existing connection (broadcast values through Subject)
- **All subscribers unsubscribed**: Disconnect from the source (if `resetOnRefCountZero: true`)
- **Resubscribe**: Start as a new connection (depending on reset setting)

## 🎯 Advanced Control Options (RxJS 7+)

In RxJS 7 and later, you can pass options to `share()` to finely control its behavior.

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Source: ${value}`)),
  share({
    resetOnError: true,       // Reset on error
    resetOnComplete: true,     // Reset on completion
    resetOnRefCountZero: true, // Reset when subscriber count reaches zero
  })
);
```

### Options in Detail

| Option | Default | Description |
|-----------|----------|------|
| `resetOnError` | `true` | Reset internal state on error |
| `resetOnComplete` | `true` | Reset internal state on stream completion |
| `resetOnRefCountZero` | `true` | Disconnect when subscriber count reaches zero |
| `connector` | `() => new Subject()` | Specify custom Subject |

### Advanced Control Using connector Option

Using the `connector` option, you can achieve behavior equivalent to `shareReplay`.

```typescript
import { interval, ReplaySubject } from 'rxjs';
import { take, share, tap } from 'rxjs';

// Buffer latest 1 item using ReplaySubject
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source: ${value}`)),
  share({
    connector: () => new ReplaySubject(1),
    resetOnError: false,
    resetOnComplete: false,
    resetOnRefCountZero: false
  })
);

// First subscriber
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Subscribe after 2.5 seconds (receives past 1 item)
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 2500);
```

**Execution Result**:
```
Source: 0
Observer 1: 0
Source: 1
Observer 1: 1
Observer 2: 1  // ← Receives previous value even when joining midway
Source: 2
Observer 1: 2
Observer 2: 2
...
```

> [!TIP]
> This method can be used as an alternative to `shareReplay(1)`. By setting `resetOnRefCountZero: false`, you can maintain the connection even when the reference count reaches zero, avoiding the "persistent cache" problem of `shareReplay`.

## 📊 Comparison with and without share()

### ❌ Without share() (Cold Observable)

```typescript
import { interval, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source: ${value}`))
);

// Subscriber 1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Subscriber 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Execution Result**:
```
Source: 0
Observer 1: 0
Source: 1
Observer 1: 1
Source: 0    // ← New stream starts
Observer 2: 0
Source: 2
Observer 1: 2
Source: 1
Observer 2: 1
Source: 2
Observer 2: 2
```

Each subscriber has an independent stream, and source processing is executed redundantly.

### ✅ With share() (Hot Observable)

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source: ${value}`)),
  share()
);

// Subscriber 1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Subscriber 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Execution Result**:
```
Source: 0
Observer 1: 0
Source: 1
Observer 1: 1
Observer 2: 1  // ← Shares the same stream
Source: 2
Observer 1: 2
Observer 2: 2
```

## 💼 Practical Use Cases

### Preventing Duplicate API Requests

```typescript
import { ajax } from 'rxjs/ajax';
import { share, tap } from 'rxjs';

// Observable to get user information
const getUser$ = ajax.getJSON('https://jsonplaceholder.typicode.com/users/1').pipe(
  tap(() => console.log('API request executed')),
  share() // Prevent duplicate requests in multiple components
);

// Component 1
getUser$.subscribe(user => console.log('Component 1:', user));

// Component 2 (requests almost simultaneously)
getUser$.subscribe(user => console.log('Component 2:', user));

// Result: API request is executed only once
```

### Sharing Periodic Data Retrieval

```typescript
import { timer, share, switchMap, tap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Get TODO list every 5 seconds (share API request)
const sharedTodos$ = timer(0, 5000).pipe(
  tap(() => console.log('API request executed')),
  switchMap(() => ajax.getJSON('https://jsonplaceholder.typicode.com/todos?_limit=3')),
  share() // Share API request among multiple subscribers
);

// Use same data stream in multiple components
sharedTodos$.subscribe(todos => console.log('Component A:', todos));
sharedTodos$.subscribe(todos => console.log('Component B:', todos));

// Result: API request is executed only once every 5 seconds, both components receive same data
```

## ⚠️ Important Notes

1. **Be careful with timing**: Subscribers who join midway cannot receive past values
2. **Error propagation**: When an error occurs, all subscribers are affected
3. **Memory management**: Not properly unsubscribing can cause memory leaks

## 🔄 Related Operators

- **[shareReplay()](/en/guide/operators/multicasting/shareReplay)** - Buffers past values and provides them to subsequent subscribers
- **[Subject](/en/guide/subjects/what-is-subject)** - The class that forms the basis of multicasting

> [!WARNING]
> **Deprecated operators**: Old multicasting APIs such as `publish()`, `multicast()`, `refCount()` were deprecated in RxJS v7 and removed in v8. Use `share()` or `connectable()`/`connect()` instead.

## Summary

The `share()` operator:
- Shares the same Observable among multiple subscribers
- Prevents duplicate execution of API requests and heavy processing
- Easy-to-use multicasting basics
- Fine control options available in RxJS 7+

When multiple components need the same data source, using `share()` can significantly improve performance.
