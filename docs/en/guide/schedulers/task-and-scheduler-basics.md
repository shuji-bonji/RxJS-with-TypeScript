---
description: This course explains the classification of tasks (synchronous, microtask, and macrotask) and their relationship with each scheduler in RxJS from the basics, including how JavaScript's event loop works, the difference in execution order, and the implementation and operation of setTimeout, Promise, queueMicrotask, etc. and acquire knowledge that can be used to select a scheduler for RxJS.
---

# Basic Knowledge of Tasks and Schedulers

## What is Synchronous Processing?
Synchronous processing is executed immediately in the order in which the code is written, and does not proceed to the next process until the previous process is completed.

#### Example
```ts
console.log('A');
console.log('B');
console.log('C');

// Output:
// A
// B
// C
```


## What is Asynchronous Processing?
Asynchronous processing is processing that is not executed immediately, but is executed after the current synchronous processing is finished.
Asynchronous processing includes "macro tasks" and "micro tasks".


## Macro Task
- A task that is executed in the next cycle of the event loop.
- Examples: `setTimeout`, `setInterval`, browser events

#### Execution Example
```ts
console.log('Start');
setTimeout(() => console.log('Macro Task'), 0);
console.log('End');

// Output:
// Start
// End
// Micro Task
```

### RxJS Support
- `asyncScheduler`
  - Uses `setTimeout` internally
  - Works as a macro task

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hello')
  .pipe(observeOn(asyncScheduler))
  .subscribe(console.log);

// Output:
// Hello
```


## Micro Task
- A task that is executed immediately after the current task finishes, but before the next task starts.
- Examples: `Promise.then`, `queueMicrotask`

#### Execution Example
```ts
console.log('Start');
Promise.resolve().then(() => console.log('Micro Task'));
console.log('End');

// Output:
// Start
// End
// Micro Task
```

### RxJS Support
- `asapScheduler`
  - Uses `Promise.resolve().then()` internally
  - Works as a microtask

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hi')
  .pipe(observeOn(asapScheduler))
  .subscribe(console.log);

// Output:
// Hi
```


## Synchronous Task
- Normal code to be executed immediately.

### RxJS Support
- `queueScheduler`
  - Appears to be synchronous, but allows fine control through task queuing.

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Now')
  .pipe(observeOn(queueScheduler))
  .subscribe(console.log);

// Output:
// Now
```


## Execution Order Summary

#### Code Example
```ts
console.log('1');

setTimeout(() => console.log('2 (setTimeout)'), 0);
Promise.resolve().then(() => console.log('3 (Promise)'));

console.log('4');

// Output:
// 1
// 4
// 3 (Promise) 👈 Microtask
// 2 (setTimeout) 👈 Macrotask
```


## Task and RxJS Scheduler Correspondence Table

| Type         | Example                       | RxJS Scheduler     |
|--------------|-------------------------------|---------------------|
| Synchronous  | Normal code                   | `queueScheduler`    |
| Microtask    | Promise.then, queueMicrotask  | `asapScheduler`     |
| Macrotask    | setTimeout, setInterval       | `asyncScheduler`    |
