---
description: Observable is a core concept of RxJS and represents a data stream that occurs over time. The difference from Promise, the subscription and unsubscription mechanism, the distinction between cold and hot, and lifecycle management are explained with practical code examples.
---

# What is Observable?

[📘 RxJS Official: Observable](https://rxjs.dev/api/index/class/Observable)

Observable in RxJS is a core component that represents "a stream of data that occurs over time" and is designed based on the Observer pattern to handle asynchronous and event-driven processing in a unified manner.

## Role of Observable

An Observable acts as a "data producer" that publishes multiple values over time. In contrast, an Observer acts as a "consumer" and subscribes to values via `subscribe()`.

In the following example, we create an **Observable (producer)** called `observable$` and an **Observer (consumer)** subscribes and receives values.

```ts
import { Observable } from 'rxjs';

// Create Observable (producer)
const observable$ = new Observable<number>(subscriber => {
  // Logic to be executed on subscription
  subscriber.next(1);
  subscriber.next(2);
  subscriber.complete();
});

// Observer (consumer) subscribes
observable$.subscribe({
  next: value => console.log('Next value:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Complete')
});

// Output:
// Next value: 1
// Next value: 2
// Complete
```

> [!NOTE]
> The function passed as the argument to `new Observable(function)` defines the **logic to be executed when the Observable is subscribed**. The function itself is not the producer; the Observable as a whole is the producer.

## Types of Notifications

Observable sends the following three types of notifications to Observer:

- `next`: notification of a value
- `error`: notification when an error occurs (no further notifications are sent)
- `complete`: notification of successful completion

For more information, see the [Observer in "Observable's Lifecycle"](./observable-lifecycle.md#_2-observer-observer) section.

## Difference between Observable and Promise

| Feature | Observable | Promise |
|---|---|---|
| Multiple values | ◯ | ×(Single only) |
| Cancelable | ◯(`unsubscribe()`) | × |
| Lazy execution | ◯ | ◯ |
| Sync/Async | Both | Async only |

The biggest difference between Observable and Promise is "whether it can handle multiple values" and "whether it can cancel in the middle."
Promise is suitable for one-time asynchronous processing, while Observable has strengths in "continuously occurring asynchronous data" such as event streams.

Observable is also important in terms of resource management, such as preventing memory leaks and stopping unnecessary communication, since subscriptions can be cancelled in the middle of a process by `unsubscribe()`.

On the other hand, Promise is widely adopted in the standard API and can be written in an intuitive way combined with `async/await`. It is desirable to use both according to the application.

## Distinction between Cold and Hot

There are two types of Observable in RxJS: "Cold" and "Hot".

- **Cold Observable**: Each subscriber has its own data stream, which starts executing when subscribed. (e.g., `of()`, `from()`, `fromEvent()`, `ajax()`)
- **Hot Observable**: Subscribers share the same data stream and data continues to flow regardless of whether they are subscribed or not. (e.g., `Subject`, Observable multicast with `share()`)

This distinction has a significant impact on data sharing and resource efficiency.
For more information, see the ["Cold Observable and Hot Observable"](./cold-and-hot-observables.md) section.

## Observable and Pipeline

The true value of an Observable is realized by combining it with operators using the `pipe()` method.

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5);
numbers$.pipe(
  filter(n => n % 2 === 0), // Pass only even numbers
  map(n => n * 10)          // Multiply by 10
).subscribe(value => console.log(value));
// Output: 20, 40
```

## Observable Lifecycle

Observable has the following lifecycle:

1. **Creation** - creation of an Observable instance
2. **Subscribe** - start receiving data by `subscribe()`
3. **Execution** - publish data (`next`), error (`error`), or completion (`complete`)
4. **Unsubscribe** - end subscription with `unsubscribe()`

It is important to unsubscribe Observable subscriptions that are no longer needed to prevent resource leaks.
For details, see the ["Observable Lifecycle"](./observable-lifecycle.md) section.

## Where to Use Observable

- UI events (clicks, scrolling, keyboard operations, etc.)
- HTTP requests
- Time-based processing (intervals and timers)
- WebSocket and real-time communication
- Application state management

## Summary

Observable is the foundation for flexible and unified handling of asynchronous data. As a central concept in ReactiveX (RxJS), it provides a concise representation of complex asynchronous processing and event streams.
