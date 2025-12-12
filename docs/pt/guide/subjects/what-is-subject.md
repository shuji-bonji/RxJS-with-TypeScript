---
description: Subject is a special class in RxJS that has both Observable and Observer properties. It can publish and subscribe to data at the same time, and can distribute the same value to multiple subscribers via multicast; it can implement practical patterns such as event bus and state management, while maintaining type safety with TypeScript type parameters.
---

# What is Subject

[📘 RxJS Official: Subject](https://rxjs.dev/api/index/class/Subject)

Subject is a special kind of Observable in RxJS. Whereas a normal Observable provides unidirectional data flow, a Subject is a hybrid entity with both "Observable" and "Observer" properties.

Subject has the following characteristics:

- Can publish data (Observable function)
- Can subscribe to data (Observer function)
- Can deliver the same value to multiple subscribers (Multicast)
- Receive only values that occur after subscription (Hot Observable feature)


## Basic Usage of Subject

```ts
import { Subject } from 'rxjs';

// Create a Subject
const subject = new Subject<number>();

// Subscribe as an Observer
subject.subscribe(value => console.log('Observer A:', value));
subject.subscribe(value => console.log('Observer B:', value));

// Publish values as an Observable
subject.next(1); // Publish values to both subscribers
subject.next(2); // Publish values to both subscribers

// Add a new subscriber (delayed subscription)
subject.subscribe(value => console.log('Observer C:', value));

subject.next(3); // Publish values to all subscribers

// Notify completion
subject.complete();
```

#### Execution Result
```
Observer A: 1
Observer B: 1
Observer A: 2
Observer B: 2
Observer A: 3
Observer B: 3
Observer C: 3
```

### Difference from Normal Observable

The Subject is a **Hot Observable** and differs from a normal Cold Observable in the following ways:

- Data is issued with or without subscriptions
- The same value can be shared by multiple subscribers (multicast)
- Values can be published externally with `.next()`
- Past values are not retained, only post-subscription values are received


## Subject and Multicasting

One of the key features of Subject is "multicasting".
This is the ability to efficiently distribute a single data source to multiple subscribers.

```ts
import { Subject, interval } from 'rxjs';
import { take } from 'rxjs';

// Data source
const source$ = interval(1000).pipe(take(3));

// Subject for multicasting
const subject = new Subject<number>();

// Connect source to Subject
source$.subscribe(subject); // Subject functions as a subscriber

// Multiple observers subscribe to the Subject
subject.subscribe(value => console.log('Observer 1:', value));
subject.subscribe(value => console.log('Observer 2:', value));
```

#### Execution Result
```
Observer 1: 0
Observer 2: 0
Observer 1: 1
Observer 2: 1
Observer 1: 2
Observer 2: 2
```

This pattern, also called single-source multicast, is used to efficiently distribute a single data source to multiple subscribers.


## Two Uses of Subject

There are two main uses for Subject. Each has a different use and behavior.

### 1. Pattern of Calling `.next()` Yourself

The Subject is used as the **Observable**.
This pattern is suitable for "sending explicit values" such as event notifications or status updates.

```ts
const subject = new Subject<string>();

subject.subscribe(val => console.log('Observer A:', val));
subject.next('Hello');
subject.next('World');

// Output:
// Observer A: Hello
// Observer A: World
```

---

### 2. Pattern of Relaying Observable (Multicast)

The Subject acts as an **Observer**, receiving values from the Observable and relaying them.
This usage is useful for **converting a Cold Observable to Hot and multicasting** it.

```ts
const source$ = interval(1000).pipe(take(3));
const subject = new Subject<number>();

// Observable → Subject (relay)
source$.subscribe(subject);

// Subject → Deliver to multiple subscribers
subject.subscribe(val => console.log('Observer 1:', val));
subject.subscribe(val => console.log('Observer 2:', val));

// Output:
// Observer 1: 0
// Observer 2: 0
// Observer 1: 1
// Observer 2: 1
// Observer 1: 2
// Observer 2: 2
```



> [!TIP]
> It is easier to understand if you imagine `.next()` as "a person who speaks himself" when you call `.next()` yourself, or as "a person who uses a microphone to amplify another person's speech" when you receive it from an Observable and relay it.


## Practical Use Cases for Subject

Subject is particularly useful in the following scenarios:

1. **State Management** - sharing and updating application state
2. **Event Bus** - communication between components
3. **HTTP response sharing** - multiple components share the results of the same API call
4. **Centralized UI Event Management** - handle various UI operations in one place

#### Example: Event Bus Implementation
```ts
import { Subject } from 'rxjs';
import { filter } from 'rxjs';

interface AppEvent {
  type: string;
  payload: any;
}

// Application-wide event bus
const eventBus = new Subject<AppEvent>();

// Subscribe to specific event types
eventBus.pipe(
  filter(event => event.type === 'USER_LOGGED_IN')
).subscribe(event => {
  console.log('User logged in:', event.payload);
});

// Subscribe to another event type
eventBus.pipe(
  filter(event => event.type === 'DATA_UPDATED')
).subscribe(event => {
  console.log('Data updated:', event.payload);
});

// Publish events
eventBus.next({ type: 'USER_LOGGED_IN', payload: { userId: '123', username: 'test_user' } });
eventBus.next({ type: 'DATA_UPDATED', payload: { items: [1, 2, 3] } });
```

#### Execution Result
```
User logged in: {userId: '123', username: 'test_user'}
Data updated: {items: Array(3)}
```

## Summary

Subject is an important component of the RxJS ecosystem that plays the following roles:

- Has the characteristics of both an Observer and an Observable
- Provides a means to convert a cold Observable into a hot one
- Efficiently delivers the same data stream to multiple subscribers
- Facilitates communication between components and services
- Provides a foundation for state management and event processing

## 🔗 Related Sections

- **[Common Mistakes and Solutions](/pt/guide/anti-patterns/common-mistakes#1-external-publication-of-subject)** - Best practices to avoid misuse of Subject
- **[Types of Subject](./types-of-subject)** - BehaviorSubject, ReplaySubject, AsyncSubject, etc.
