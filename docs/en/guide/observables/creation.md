---
description: This course provides a comprehensive explanation of how to create Observables in RxJS, from basic generating functions such as of and from, to defining custom Observables, HTTP communication, and event streaming, with practical code examples.
---
# How to Create an Observable

An Observable defines a "stream of data," and there are a wide variety of ways to create one.
RxJS provides a variety of means to create custom Observables or to easily generate Observables from events, arrays, HTTP responses, etc.

This section provides a comprehensive overview of how to create Observables in RxJS, from basic syntax to practical applications.

## Classification of Observable Creation Methods

The following is a list of the main creation methods by category.

| Category | Main Methods | Description |
|----------|----------|------|
| Custom Creation | [`new Observable()`](#new-observable) | High flexibility but requires more code. Manual cleanup required |
| Creation Functions | [`of()`](#of), [`from()`](#from), [`fromEvent()`](#fromevent), [`interval()`](#interval-timer), [`timer()`](#interval-timer), [`ajax()`](#ajax), [`fromFetch()`](#fromfetch), [`scheduled()`](#scheduled) | Commonly used data, event, and time-based generation functions |
| Special Creation Functions | [`defer()`](#defer), [`range()`](#range), [`generate()`](#generate), [`iif()`](#iif) | Control-oriented, loop-oriented generation, conditional switching, etc. |
| Special Observables | [`EMPTY`](#empty-never-throwerror), [`NEVER`](#empty-never-throwerror), [`throwError()`](#empty-never-throwerror) | For completion, no action, and error emission |
| Subject Family | [`Subject`](#subject-behaviorsubject), [`BehaviorSubject`](#subject-behaviorsubject) | Special Observable that functions as both observer and sender |
| Callback Conversion | [`bindCallback()`](#bindcallback), [`bindNodeCallback()`](#bindnodecallback) | Convert callback-based functions to Observable |
| Resource Control | [`using()`](#using) | Perform resource control at the same time as subscribing to Observable |
| WebSocket | [`webSocket()`](#websocket) | Handle WebSocket communication as bidirectional Observable |

## Custom Creation

### new Observable()
[📘 RxJS Official: Observable](https://rxjs.dev/api/index/class/Observable)

The most basic method is to use the `Observable` constructor directly. This method is most flexible when you want to define custom Observable logic. Fine-grained behavior control is possible through explicit `next`, `error`, and `complete` calls.

```ts
import { Observable } from 'rxjs';

const observable$ = new Observable<number>(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  setTimeout(() => {
    subscriber.next(4);
    subscriber.complete();
  }, 1000);
});

observable$.subscribe({
  next: value => console.log('Value:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Complete')
});
// Output:
// Value: 1
// Value: 2
// Value: 3
// Value: 4
// Complete
```

> [!CAUTION]
> If you use `new Observable()`, you must write the explicit resource release (cleanup process) yourself.
> ```ts
> const obs$ = new Observable(subscriber => {
>   const id = setInterval(() => subscriber.next(Date.now()), 1000);
>   return () => {
>     clearInterval(id); // Explicit resource release
>   };
> });
> ```
> On the other hand, RxJS built-in creation functions such as `fromEvent()` and `interval()` have appropriate cleanup processes inside.
> ```ts
> const click$ = fromEvent(document, 'click');
> const timer$ = interval(1000);
> ```
> They use `addEventListener` or `setInterval` internally and are designed so that RxJS automatically calls `removeEventListener` or `clearInterval()` when `unsubscribe()`.
>
> Note that even if the cleanup process is implemented inside RxJS, that process will not be executed unless `unsubscribe()` is called.
> ```ts
>  const subscription = observable$.subscribe({
>  // Omitted...
>  });
>
>  subscription.unsubscribe(); // 👈
> ```
> - No matter which method you use to create an Observable, be sure to get into the habit of `unsubscribe()` when you no longer need it.
> - Forgetting to unsubscribe will keep event listeners and timers running, causing memory leaks and unexpected side effects.

## Creation Functions

For more concise and application-specific Observable creation, RxJS provides "Creation Functions". These can be used to simplify code for repeated use cases.

> [!NOTE]
> In the official RxJS documentation, these are categorized as "Creation Functions".
> Previously (RxJS 5.x ~ 6) they were called "creation operators," but since RxJS 7, "Creation Functions" is the official term.

### of()
[📘 RxJS Official: of()](https://rxjs.dev/api/index/function/of)

The simplest Observable creation function that issues multiple values **one at a time in sequence**.

```ts
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Value:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Complete')
});
// Output: Value: 1, Value: 2, Value: 3, Value: 4, Value: 5, Complete
```

> [!IMPORTANT]
> Difference between `of()` and `from()`
> - `of([1, 2, 3])` → issues a single array.
> - `from([1, 2, 3])` → issues individual values `1`, `2`, `3` in sequence.
>
> Note that this is often confused.

> [!TIP]
> For detailed usage and practical examples, see [of() detail page](/en/guide/creation-functions/basic/of).

### from()
[📘 RxJS Official: from()](https://rxjs.dev/api/index/function/from)

Generates an Observable from an **existing data structure** such as an array, Promise, or iterable.

```ts
import { from } from 'rxjs';

// Create from array
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Array value:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Complete')
});

// Create from Promise
const promise$ = from(Promise.resolve('Promise result'));
promise$.subscribe({
  next: value => console.log('Promise result:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Complete')
});

// Create from iterable
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Iterable value:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Complete')
});

// Output:
// Array value: 1
// Array value: 2
// Array value: 3
// Complete
// Iterable value: 1
// Iterable value: 2
// Iterable value: 3
// Complete
// Promise result: Promise result
// Complete
```

> [!TIP]
> For detailed usage and practical examples, see [from() detail page](/en/guide/creation-functions/basic/from).

### fromEvent()
[📘 RxJS Official: fromEvent](https://rxjs.dev/api/index/function/fromEvent)

Function to **handle event sources** such as DOM events as an Observable.

```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe({
  next: event => console.log('Click event:', event),
  error: err => console.error('Error:', err),
  complete: () => console.log('Complete')
});

// Output:
// Click event: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!CAUTION]
> Note that it cannot be used outside of the DOM
> - `fromEvent()` is only available in the browser environment and not in Node.js.
> - Multiple subscriptions may add multiple event listeners.

> 👉 For more detailed examples of event stream utilization, see [Event Streaming](../observables/events).

> [!TIP]
> For detailed usage and practical examples, see [fromEvent() detail page](/en/guide/creation-functions/basic/fromEvent).

### interval(), timer()
[📘 RxJS Official: interval](https://rxjs.dev/api/index/function/interval), [📘 RxJS Official: timer](https://rxjs.dev/api/index/function/timer)

This function is used when you want to issue values continuously at regular intervals or when you need **time control**.

```ts
import { interval, timer } from 'rxjs';

// Issue values every second
const interval$ = interval(1000);
interval$.subscribe({
  next: value => console.log('Interval:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Complete')
});

// Start after 3 seconds, then issue values every second
const timer$ = timer(3000, 1000);
timer$.subscribe({
  next: value => console.log('Timer:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Complete')
});

// Output:
// Interval: 0
// Interval: 1
// Interval: 2
// Timer: 0
// Interval: 3
// Timer: 1
// Interval: 4
// Timer: 2
// .
// .
```
`interval()` and `timer()` are frequently used for time-controlled processing, especially suitable for animation, polling, and asynchronous event delays.

> [!CAUTION]
> Note that Cold Observable
> - `interval()` and `timer()` are Cold Observable and are executed independently for each subscription.
> - You can consider making them Hot with `share()` or other methods if necessary.
>
> For details, see the ["Cold Observable and Hot Observable"](./cold-and-hot-observables.md) section.

> [!TIP]
> For detailed usage and practical examples, see [interval() detail page](/en/guide/creation-functions/basic/interval) and [timer() detail page](/en/guide/creation-functions/basic/timer).

### ajax()
[📘 RxJS Official: ajax](https://rxjs.dev/api/ajax/ajax)

Function for **asynchronous handling** of HTTP communication results as **Observable**.

```ts
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
api$.subscribe({
  next: response => console.log('API response:', response),
  error: error => console.error('API error:', error),
  complete: () => console.log('API complete')
});

// Output:
// API response: {userId: 1, id: 1, title: 'delectus aut autem', completed: false}
// API complete
```

> [!NOTE]
> RxJS ajax uses XMLHttpRequest internally. On the other hand, RxJS also has an operator called fromFetch, which uses the Fetch API to make HTTP requests.

> [!TIP]
> For detailed usage and practical examples, see [ajax() detail page](/en/guide/creation-functions/http-communication/ajax). For an overview of HTTP communication functions, see [HTTP Communication Creation Functions](/en/guide/creation-functions/http-communication/).

### fromFetch()
[📘 RxJS Official: fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` wraps the Fetch API and allows you to treat HTTP requests as Observables.
It is similar to `ajax()`, but more modern and lightweight.

```ts
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs';

const api$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1');

api$.pipe(
  switchMap(response => response.json())
).subscribe({
  next: data => console.log('Data:', data),
  error: err => console.error('Error:', err),
  complete: () => console.log('Complete')
});

// Output:
// Data: {completed: false, id: 1, title: "delectus aut autem", userId: 1}
// Complete
```

> [!NOTE]
> Because `fromFetch()` uses the Fetch API, unlike `ajax()`, initialization of request settings and `.json()` conversion of responses must be done manually.
> Proper error handling and HTTP status checking are also required.

> [!TIP]
> For detailed usage and practical examples, see [fromFetch() detail page](/en/guide/creation-functions/http-communication/fromFetch). For an overview of HTTP communication functions, see [HTTP Communication Creation Functions](/en/guide/creation-functions/http-communication/).

### scheduled()
[📘 RxJS Official: scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` is a function that allows you to explicitly specify a scheduler for published functions such as `of()` and `from()`.
Use this function when you want to control the timing of synchronous or asynchronous execution in detail.

```ts
import { scheduled, asyncScheduler } from 'rxjs';

const observable$ = scheduled([1, 2, 3], asyncScheduler);
observable$.subscribe({
  next: val => console.log('Value:', val),
  complete: () => console.log('Complete')
});

// Execution is asynchronous
// Output:
// Value: 1
// Value: 2
// Value: 3
// Complete
```

> [!NOTE]
> `scheduled()` allows existing synchronous functions (e.g. `of()`, `from()`) to work asynchronously.
> This is useful for testing and UI performance optimization where asynchronous processing control is required.

> [!TIP]
> For detailed usage and practical examples, see [scheduled() detail page](/en/guide/creation-functions/control/scheduled). For an overview of control functions, see [Control Creation Functions](/en/guide/creation-functions/control/).

### defer()
[📘 RxJS Official: defer](https://rxjs.dev/api/index/function/defer)

It is used when you want to **delay the generation of an Observable until subscription time**.

```ts
import { defer, of } from 'rxjs';

const random$ = defer(() => of(Math.random()));

random$.subscribe(value => console.log('1st:', value));
random$.subscribe(value => console.log('2nd:', value));

// Output:
// 1st: 0.123456789
// 2nd: 0.987654321
```

> [!NOTE]
> `defer()` is useful when you want to create a new Observable on each subscription. You can achieve lazy evaluation.

> [!TIP]
> For detailed usage and practical examples, see [defer() detail page](/en/guide/creation-functions/conditional/defer).

### range()
[📘 RxJS Official: range](https://rxjs.dev/api/index/function/range)

Generates a continuous integer value in the specified range as an Observable.

```ts
import { range } from 'rxjs';

const numbers$ = range(1, 5);
numbers$.subscribe({
  next: value => console.log('Number:', value),
  complete: () => console.log('Complete')
});

// Output:
// Number: 1
// Number: 2
// Number: 3
// Number: 4
// Number: 5
// Complete
```

> [!TIP]
> For detailed usage and practical examples, see [range() detail page](/en/guide/creation-functions/loop/range).

### generate()
[📘 RxJS Official: generate](https://rxjs.dev/api/index/function/generate)

Generates Observable like a loop structure. Allows fine control over initial values, conditions, increases/decreases, and output of values.

```ts
import { generate } from 'rxjs';

const fibonacci$ = generate({
  initialState: [0, 1],
  condition: ([, b]) => b < 100,
  iterate: ([a, b]) => [b, a + b],
  resultSelector: ([a]) => a
});

fibonacci$.subscribe({
  next: value => console.log('Fibonacci:', value),
  complete: () => console.log('Complete')
});

// Output:
// Fibonacci: 0
// Fibonacci: 1
// Fibonacci: 1
// Fibonacci: 2
// Fibonacci: 3
// Fibonacci: 5
// Fibonacci: 8
// Fibonacci: 13
// Fibonacci: 21
// Fibonacci: 34
// Fibonacci: 55
// Fibonacci: 89
// Complete
```

> [!TIP]
> For detailed usage and practical examples, see [generate() detail page](/en/guide/creation-functions/loop/generate).

### iif()
[📘 RxJS Official: iif](https://rxjs.dev/api/index/function/iif)

Use this function when you want to **switch Observable by conditional branching**.

```ts
import { iif, of, EMPTY } from 'rxjs';

const condition = true;
const iif$ = iif(() => condition, of('Condition is true'), EMPTY);

iif$.subscribe({
  next: val => console.log('iif:', val),
  complete: () => console.log('Complete')
});

// Output:
// iif: Condition is true
// Complete
```

> [!NOTE]
> `iif()` can dynamically switch the Observable to return depending on conditions. This is useful for flow control.

## Special Observables

### EMPTY, NEVER, throwError()
[📘 RxJS Official: EMPTY](https://rxjs.dev/api/index/const/EMPTY), [📘 RxJS Official: NEVER](https://rxjs.dev/api/index/const/NEVER), [📘 RxJS Official: throwError](https://rxjs.dev/api/index/function/throwError)

RxJS also provides special Observables that are useful for execution control, exception handling, and learning.

```ts
import { EMPTY, throwError, NEVER } from 'rxjs';

// Observable that completes immediately
const empty$ = EMPTY;
empty$.subscribe({
  next: () => console.log('This is not displayed'),
  complete: () => console.log('Completes immediately')
});

// Observable that issues an error
const error$ = throwError(() => new Error('Error occurred'));
error$.subscribe({
  next: () => console.log('This is not displayed'),
  error: err => console.error('Error:', err.message),
  complete: () => console.log('Complete')
});

// Observable that does not issue anything and does not complete
const never$ = NEVER;
never$.subscribe({
  next: () => console.log('This is not displayed'),
  complete: () => console.log('This is also not displayed')
});

// Output:
// Completes immediately
// Error: Error occurred
```

> [!IMPORTANT]
> Mainly for control, verification, and learning purposes
> - `EMPTY`, `NEVER`, and `throwError()` are used for **flow control, exception handling validation**, or learning purposes, not for normal data streams.

## Subject Family

### Subject, BehaviorSubject, etc. {#subject-behaviorsubject}
[📘 RxJS Official: Subject](https://rxjs.dev/api/index/class/Subject), [📘 RxJS Official: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Observable that can issue its own value, suitable for **multicast and state sharing**.

```ts
import { Subject } from 'rxjs';

const subject$ = new Subject<number>();

// Use as Observer
subject$.subscribe(value => console.log('Observer 1:', value));
subject$.subscribe(value => console.log('Observer 2:', value));

// Use as Observable
subject$.next(1);
subject$.next(2);
subject$.next(3);

// Output:
// Observer 1: 1
// Observer 2: 1
// Observer 1: 2
// Observer 2: 2
// Observer 1: 3
// Observer 2: 3
```

> [!IMPORTANT]
> Subject has the properties of both Observable and Observer. Multiple subscribers can share the same data stream (multicast).

> [!TIP]
> For details on various Subject types (BehaviorSubject, ReplaySubject, AsyncSubject), see [Subject and Multicast](/en/guide/subjects/what-is-subject).

## Callback Conversion

### bindCallback()
[📘 RxJS Official: bindCallback](https://rxjs.dev/api/index/function/bindCallback)

A function that allows callback-based asynchronous functions to be treated as Observable.

```ts
import { bindCallback } from 'rxjs';

// Callback-based function (legacy style)
function asyncFunction(value: number, callback: (result: number) => void) {
  setTimeout(() => callback(value * 2), 1000);
}

// Convert to Observable
const asyncFunction$ = bindCallback(asyncFunction);
const observable$ = asyncFunction$(5);

observable$.subscribe({
  next: result => console.log('Result:', result),
  complete: () => console.log('Complete')
});

// Output:
// Result: 10
// Complete
```

> [!TIP]
> `bindCallback()` is useful for converting legacy callback-based APIs to Observable.

### bindNodeCallback()
[📘 RxJS Official: bindNodeCallback](https://rxjs.dev/api/index/function/bindNodeCallback)

A function specialized for converting callback-based functions in Node.js style (error-first callback) to Observable.

```ts
import { bindNodeCallback } from 'rxjs';

// Node.js style callback function (error-first callback)
function readFile(path: string, callback: (err: Error | null, data: string) => void) {
  if (path === 'valid.txt') {
    callback(null, 'file content');
  } else {
    callback(new Error('File not found'), '');
  }
}

// Convert to Observable
const readFile$ = bindNodeCallback(readFile);

readFile$('valid.txt').subscribe({
  next: data => console.log('Data:', data),
  error: err => console.error('Error:', err.message),
  complete: () => console.log('Complete')
});

// Output:
// Data: file content
// Complete
```

### Difference between bindCallback() and bindNodeCallback()

#### Example: Target of bindCallback()

```ts
// General callback (success only)
function getData(cb: (data: string) => void) {
  cb('success');
}
```
→ Use bindCallback() for simple "return only a value" callbacks.

#### Example: Target of bindNodeCallback() (Node.js style)

```ts
// Error-first callback
function readFile(path: string, cb: (err: Error | null, data: string) => void) {
  if (path === 'valid.txt') cb(null, 'file content');
  else cb(new Error('not found'), '');
}
```
→ If you use bindNodeCallback(), errors will be notified as Observable errors.

> [!NOTE]
> How to use
> - bindNodeCallback() if the first argument of the callback is "error or not"
> - bindCallback() for a simple "return only a value" callback

## Resource Control

### using()
[📘 RxJS Official: using](https://rxjs.dev/api/index/function/using)

`using()` functions to associate the creation and release of resources with the Observable's lifecycle.
It is useful in combination with **processes that require manual cleanup**, such as WebSockets, event listeners, and external resources.

```ts
import { using, interval, Subscription } from 'rxjs';

const resource$ = using(
  () => new Subscription(() => console.log('Resource released')),
  () => interval(1000)
);

const sub = resource$.subscribe(value => console.log('Value:', value));

// Unsubscribe after a few seconds
setTimeout(() => sub.unsubscribe(), 3500);

// Output:
// Value: 0
// Value: 1
// Value: 2
// Resource released
```

> [!IMPORTANT]
> `using()` is useful for matching the scope of a resource with the Observable's subscription.
> An explicit cleanup process is automatically called when `unsubscribe()` is done.

> [!TIP]
> For detailed usage and practical examples, see [using() detail page](/en/guide/creation-functions/control/using). For an overview of control functions, see [Control Creation Functions](/en/guide/creation-functions/control/).

## WebSocket()
[📘 RxJS Official: webSocket](https://rxjs.dev/api/webSocket/webSocket)

The `rxjs/webSocket` module of RxJS provides a `webSocket()` function that allows WebSocket to be treated as an Observable/Observer.

```ts
import { webSocket } from 'rxjs/webSocket';

const socket$ = webSocket('wss://echo.websocket.org');

socket$.subscribe({
  next: msg => console.log('Received:', msg),
  error: err => console.error('Error:', err),
  complete: () => console.log('Complete')
});

// Send message (as Observer)
socket$.next('Hello WebSocket!');
```

> [!IMPORTANT]
> `webSocket()` is an Observable/Observer hybrid that allows two-way communication.
> It is useful for real-time communication because WebSocket connections, sending, and receiving can be easily handled as Observable.

## Summary

There are a wide variety of ways to create Observables in RxJS, and it is important to choose the appropriate method for your application.

- If you need custom processing, use `new Observable()`
- `of()`, `from()`, `fromEvent()`, etc. for handling existing data and events
- `ajax()` or `fromFetch()` for HTTP communication
- `Subject` family for sharing data among multiple subscribers

By using them appropriately, you can take full advantage of the flexibility of RxJS.
