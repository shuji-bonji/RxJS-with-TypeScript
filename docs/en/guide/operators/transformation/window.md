---
description: window is an RxJS operator that splits a source Observable into nested Observables at the time another Observable emits values, ideal for advanced event-driven stream processing.
titleTemplate: ':title | RxJS'
---

# window - Split Observable at Another Observable's Timing

The `window` operator groups the values of a source Observable **until another Observable emits values** and outputs that group as a **new Observable**.
While `buffer` returns an array, `window` returns an **Observable&lt;T&gt;**, allowing further operators to be applied to each window.

## 🔰 Basic Syntax and Usage

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

// Emit values every 100ms
const source$ = interval(100);

// Use click event as trigger
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  window(clicks$),
  mergeAll() // Flatten each window
).subscribe(value => {
  console.log('Value in window:', value);
});

// A new window starts with each click
```

- Each time `clicks$` emits a value, a new window (Observable) is created.
- Each window can be treated as an independent Observable.

[🌐 RxJS Official Documentation - `window`](https://rxjs.dev/api/operators/window)

## 💡 Typical Usage Patterns

- Event-driven stream partitioning
- Apply different processing to each window
- Data grouping with dynamic delimitation
- Aggregate processing for each window

## 🔍 Difference from buffer

| Operator | Output | Use Case |
|:---|:---|:---|
| `buffer` | **Array (T[])** | Process grouped values together |
| `window` | **Observable&lt;T&gt;** | Different stream processing for each group |

```ts
import { interval, timer } from 'rxjs';
import { buffer, window, mergeAll } from 'rxjs';

const source$ = interval(100);
const trigger$ = timer(1000, 1000);

// buffer - Output as array
source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Output: Buffer (array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// window - Output as Observable
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  console.log('Window (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Value in window:', value);
  });
});
```

## 🧠 Practical Code Example 1: Count Per Window

This example triggers on button click and counts the number of events up to that point.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, map, mergeAll, scan } from 'rxjs';

// Create button
const button = document.createElement('button');
button.textContent = 'Delimit Window';
document.body.appendChild(button);

// Output area
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Emit values every 100ms
const source$ = interval(100);

// Trigger on button click
const clicks$ = fromEvent(button, 'click');

let windowCount = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const currentWindow = ++windowCount;
    console.log(`Window ${currentWindow} started`);

    // Count values in each window
    return window$.pipe(
      scan((count) => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  output.textContent = `Current window: ${windowCount}, Count: ${count}`;
});
```

- Each time a button is clicked, a new window is created.
- The number of values in each window is counted in real time.

## 🎯 Practical Code Example 2: Different Processing for Each Window

This is an advanced example that applies different processing to each window.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, take, mergeAll, map } from 'rxjs';

const source$ = interval(200);
const clicks$ = fromEvent(document, 'click');

let windowNumber = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Even windows: Get only first 3 items
      console.log(`Window ${current}: Get first 3 items`);
      return window$.pipe(take(3));
    } else {
      // Odd windows: Get all
      console.log(`Window ${current}: Get all`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Value: ${value} (Window ${windowNumber})`);
});
```

- You can conditionally apply different processing for each window.
- Each window is an independent Observable, so you can freely combine operators.

## 🎯 Practical Example: Control with Multiple Triggers

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { window, mergeAll, scan, map } from 'rxjs';

const source$ = interval(100);

// Multiple triggers: click or 3 seconds elapsed
const clicks$ = fromEvent(document, 'click');
const threeSeconds$ = timer(3000, 3000);
const trigger$ = merge(clicks$, threeSeconds$);

source$.pipe(
  window(trigger$),
  map((window$, index) => {
    console.log(`Window ${index + 1} started`);

    // Calculate sum for each window
    return window$.pipe(
      scan((sum, value) => sum + value, 0)
    );
  }),
  mergeAll()
).subscribe(sum => {
  console.log('Current sum:', sum);
});
```

## ⚠️ Notes

### 1. Window Subscription Management

Each window is an independent Observable, so it must be explicitly subscribed to.

```ts
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  // Values won't flow unless you subscribe to the window itself
  window$.subscribe(value => {
    console.log('Value:', value);
  });
});
```

Or use `mergeAll()`, `concatAll()`, `switchAll()`, etc. to flatten.

```ts
source$.pipe(
  window(trigger$),
  mergeAll() // Merge all windows
).subscribe(value => {
  console.log('Value:', value);
});
```

### 2. Beware of Memory Leaks

**Problem**: If the trigger Observable doesn't emit values, the first window remains open forever and values accumulate infinitely.

#### ❌ Bad Example: Trigger Does Not Occur

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100); // Continues to emit values every 100ms

// Button doesn't exist, or user doesn't click
const button = document.querySelector('#start-button'); // Possibly null
const clicks$ = fromEvent(button, 'click'); // Error or never fires

source$.pipe(
  window(clicks$), // First window won't close if clicks$ doesn't fire
  mergeAll()
).subscribe();

// Problems:
// - If clicks$ doesn't emit, the first window stays open
// - source$ values (0, 1, 2, 3...) continue to accumulate in memory
// - Causes memory leak
```

#### ✅ Good Example 1: Set Timeout

Set a timeout to prevent the first window from staying open too long.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = button ? fromEvent(button, 'click') : interval(0); // fallback

// Close window on click or after 5 seconds, whichever comes first
const autoClose$ = timer(5000); // Emit after 5 seconds
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // Window will always close within 5 seconds
  mergeAll()
).subscribe();
```

#### ✅ Good Example 2: Periodically Close Windows

Close windows periodically even without clicks.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = fromEvent(button, 'click');

// Close window on click or every 3 seconds
const autoClose$ = timer(3000, 3000); // After first 3 seconds, then every 3 seconds
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // Window closes every 3 seconds even without clicks
  mergeAll()
).subscribe();

// Result:
// - Windows automatically close every 3 seconds even without user clicks
// - Prevents infinite value accumulation in memory
```

### 3. Window Overlap

By default, windows do not overlap (next window starts after previous closes).
If overlap is needed, use `windowToggle` or `windowWhen`.

## 🆚 Comparison of window Operators

| Operator | Timing of Delimiter | Use Case |
|:---|:---|:---|
| `window` | Another Observable emits | Event-driven partitioning |
| `windowTime` | Fixed time interval | Time-based partitioning |
| `windowCount` | Fixed count | Count-based partitioning |
| `windowToggle` | Start and end Observables | Dynamic start/end control |
| `windowWhen` | Dynamic closing condition | Different end condition per window |

## 📚 Related Operators

- [`buffer`](/en/guide/operators/transformation/buffer) - Collect values as array (array version of window)
- [`windowTime`](/en/guide/operators/transformation/windowTime) - Time-based window partitioning
- [`windowCount`](/en/guide/operators/transformation/windowCount) - Count-based window partitioning
- [`windowToggle`](/en/guide/operators/transformation/windowToggle) - Window control with start and end Observables
- [`windowWhen`](/en/guide/operators/transformation/windowWhen) - Window partitioning with dynamic closing condition
- [`groupBy`](/en/guide/operators/transformation/groupBy) - Group Observables by key

## Summary

The `window` operator is a powerful tool that splits streams triggered by an external Observable and can process each group as an independent Observable.

- ✅ Can apply different processing to each window
- ✅ Flexible event-driven control
- ✅ Supports advanced stream operations
- ⚠️ Subscription management required
- ⚠️ Beware of memory leaks
