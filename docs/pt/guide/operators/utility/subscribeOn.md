---
description: The subscribeOn operator controls when to start subscribing to Observable with the specified scheduler and changes the execution context of the entire stream.
---

# subscribeOn - Control When to Start Subscribing

The `subscribeOn` operator controls Observable's **subscription start timing and execution context with the specified scheduler**. It affects the execution timing of the entire stream.

## 🔰 Basic Syntax and Operation

Asynchronizes the start of a subscription by specifying a scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler)
  )
  .subscribe(v => console.log('Value:', v));

console.log('End');

// Output:
// Start
// End
// Value: 1
// Value: 2
// Value: 3
```

The subscription start itself is asynchronized, so the call to `subscribe()` returns immediately.

[🌐 RxJS Official Documentation - subscribeOn](https://rxjs.dev/api/index/function/subscribeOn)

## 💡 Typical Usage Examples

- **Asynchronize heavy initialization processes**: Delay the start of data loading, etc.
- **Prevent UI freezes**: Asynchronously start subscriptions to maintain responsiveness
- **Prioritize processing**: Control start timing of multiple streams
- **Timing control in testing**: Control using TestScheduler

## 🧪 Practical Code Example 1: Asynchronize Heavy Initialization Processing

This is an example of starting data reading and initialization asynchronously.

```ts
import { Observable, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// UI creation
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'subscribeOn - Heavy initialization processing';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const timestamp = now.toLocaleTimeString('en-US', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${timestamp}] ${message}`;
  output.appendChild(logItem);
}

// Simulate heavy initialization processing
const heavyInit$ = new Observable<string>(subscriber => {
  addLog('Data loading started...', '#fff9c4');

  // Simulate heavy processing
  let sum = 0;
  for (let i = 0; i < 10000000; i++) {
    sum += i;
  }

  addLog('Data loading completed', '#c8e6c9');
  subscriber.next(`Result: ${sum}`);
  subscriber.complete();
});

addLog('Subscription start (UI operable)', '#e3f2fd');

heavyInit$
  .pipe(
    subscribeOn(asyncScheduler)  // Asynchronize subscription start
  )
  .subscribe({
    next: result => addLog(`Received: ${result}`, '#c8e6c9'),
    complete: () => addLog('Completed', '#e3f2fd')
  });

addLog('After subscription request (execution continues immediately)', '#e3f2fd');
```

- Subscription start is asynchronous, UI responds immediately
- Heavy processing is performed asynchronously
- Main thread is not blocked

## 🧪 Practical Code Example 2: Multiple Stream Priority Control

This is an example of controlling the start timing of multiple streams.

```ts
import { interval, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn, take, tap } from 'rxjs';

// UI creation
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'subscribeOn - Priority control';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string, color: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('en-US', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.marginBottom = '2px';
  logItem.style.backgroundColor = color;
  logItem.style.fontSize = '12px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Start', '#e3f2fd');

// High priority task (asapScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asapScheduler),
    tap(v => addLog2(`High priority: ${v}`, '#c8e6c9'))
  )
  .subscribe();

// Normal priority task (asyncScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asyncScheduler),
    tap(v => addLog2(`Normal priority: ${v}`, '#fff9c4'))
  )
  .subscribe();

addLog2('Subscription request completed', '#e3f2fd');
```

- Different schedulers control priorities
- `asapScheduler` starts execution earlier than `asyncScheduler`

## 🆚 Differences from observeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

// observeOn example
console.log('=== observeOn ===');
console.log('1: Start');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('2: tap (sync)')),
    observeOn(asyncScheduler),
    tap(() => console.log('4: tap (async)'))
  )
  .subscribe(() => console.log('5: subscribe'));

console.log('3: End');

// subscribeOn example
console.log('\n=== subscribeOn ===');
console.log('1: Start');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('3: tap (async)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(() => console.log('4: subscribe'));

console.log('2: End');
```

**Main differences**:

| Item | observeOn | subscribeOn |
|:---|:---|:---|
| **Scope of Effects** | Subsequent processing only | Entire stream |
| **Control Target** | Timing of publishing value | Timing of subscription start |
| **Positioning** | Important (behavior changes depending on where you place it) | Same wherever you place it |
| **Multiple Use** | Last one applies | First one applies |

> [!NOTE]
> For more information on `observeOn`, see [observeOn](./observeOn.md).

## ⚠️ Important Notes

### 1. Placement Position Has No Effect

`subscribeOn` has the same effect no matter where you place it in the pipeline.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, map } from 'rxjs';

// Pattern 1: First
of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),
    map(x => x * 2)
  )
  .subscribe();

// Pattern 2: Last
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Both work the same
```

### 2. Multiple subscribeOn's Apply the First One

```ts
import { of, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),  // This is used
    subscribeOn(asapScheduler)    // This is ignored
  )
  .subscribe();
```

The first `subscribeOn` scheduler (`asyncScheduler`) is used.

### 3. Some Observables Have No Effect

Observables with their own scheduler, such as `interval` and `timer`, are not affected by `subscribeOn`.

```ts
import { interval, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// ❌ subscribeOn has no effect
interval(1000)
  .pipe(
    subscribeOn(asyncScheduler)  // interval uses its own scheduler
  )
  .subscribe();

// ✅ Specify scheduler in interval argument
interval(1000, asyncScheduler)
  .subscribe();
```

## Practical Combination Examples

```ts
import { of, asyncScheduler, animationFrameScheduler } from 'rxjs';
import { subscribeOn, observeOn, map, tap } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Tap 1 (async)')),
    subscribeOn(asyncScheduler),        // Asynchronize subscription start
    map(x => x * 2),
    observeOn(animationFrameScheduler), // Sync value emission to animation frame
    tap(() => console.log('Tap 2 (animation frame)'))
  )
  .subscribe(v => console.log('Value:', v));

console.log('End');

// Execution order:
// Start
// End
// Tap 1 (async)
// Tap 1 (async)
// Tap 1 (async)
// Tap 2 (animation frame)
// Value: 2
// ... (continued below)
```

## Usage Guidelines

### Case 1: You Want to Delay the Start of Subscriptions
```ts
// → use subscribeOn
of(data)
  .pipe(subscribeOn(asyncScheduler))
  .subscribe();
```

### Case 2: I Want to Make a Specific Process Asynchronous
```ts
// → use observeOn
of(data)
  .pipe(
    map(heavy processing),
    observeOn(asyncScheduler),  // Asynchronize only after heavy processing
    map(light processing)
  )
  .subscribe();
```

### Case 3: Asynchronize the Whole Process + Further Control a Part of It
```ts
// → use subscribeOn + observeOn together
of(data)
  .pipe(
    subscribeOn(asyncScheduler),           // Asynchronize entire process
    map(processing 1),
    observeOn(animationFrameScheduler),    // Change for animation
    map(processing 2)
  )
  .subscribe();
```

## 📚 Related Operators

- **[observeOn](./observeOn)** - Controls when values are issued
- **[delay](./delay)** - Fixed time delay

## 📖 Related Documents

- **[Control of Asynchronous Processing](/pt/guide/schedulers/async-control.md)** - Scheduler Basics
- **[Types and Usage of Schedulers](/pt/guide/schedulers/types.md)** - Details of each scheduler

## ✅ Summary

The `subscribeOn` operator controls the timing and execution context for subscription initiation.

- ✅ Asynchronizes subscription start for entire stream
- ✅ Useful for asynchronizing heavy initialization processes
- ✅ Useful for preventing UI freezes
- ✅ Position of placement has no effect
- ⚠️ When multiple Observables are used, the first one is applied
- ⚠️ Not effective for some Observables
- ⚠️ Different purpose from `observeOn`
