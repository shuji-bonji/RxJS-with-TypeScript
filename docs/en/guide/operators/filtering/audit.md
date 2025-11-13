---
description: The audit operator is an RxJS filtering operator that emits only the last value within a period controlled by a custom Observable. It is ideal for dynamic timing control.
---

# audit - Emit the Last Value During a Period Controlled by a Custom Observable

The `audit` operator waits for a custom Observable to emit a value and emits the **last value** from the source during that period.
While `auditTime` controls with a fixed time, `audit` can **control the period dynamically with an Observable**.

## 🔰 Basic Syntax and Usage

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Click event
const clicks$ = fromEvent(document, 'click');

// Separate period every second
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Click was recorded');
});
```

- When a click occurs, a 1-second period begins.
- Only the last click during that 1 second is emitted.
- The next period starts after 1 second.

[🌐 RxJS Official Documentation - `audit`](https://rxjs.dev/api/operators/audit)

## 💡 Typical Usage Patterns

- **Sampling at dynamic intervals**: Adjust period according to load
- **Custom timing control**: Period control based on other Observables
- **Adaptive event limiting**: Thinning according to circumstances

## 🔍 Difference from auditTime

| Operator | Period Control | Use Case |
|:---|:---|:---|
| `auditTime` | Fixed time (milliseconds) | Simple time-based control |
| `audit` | **Custom Observable** | **Dynamic period control** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Fixed 1 second
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Fixed 1 second'));

// audit - Dynamic period
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // Random period 0-2 seconds
    return timer(period);
  })
).subscribe(() => console.log(`Dynamic period: ${period}ms`));
```

## 🧠 Practical Code Example: Dynamic Sampling According to Load

Example of adjusting sampling interval according to system load.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// Create UI
const output = document.createElement('div');
output.innerHTML = '<h3>Dynamic Sampling</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Change Load';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Load level (0: low, 1: medium, 2: high)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Low Load', 'Medium Load', 'High Load'];
  statusDiv.textContent = `Current load: ${levels[loadLevel]}`;
});

// Mouse move event
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Adjust period according to load
    const periods = [2000, 1000, 500]; // Low load → long period, high load → short period
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Mouse position: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Display maximum 10 items
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- When load is low, thin out at 2-second intervals (power saving mode)
- When load is high, sample finely at 500ms intervals
- Period can be adjusted dynamically according to load


## ⚠️ Notes

### 1. First Value is Not Emitted Immediately

`audit` waits until the period ends after receiving the first value.

```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// Output:
// 9  (after 1 second, last value of 0-9)
// 19 (after 2 seconds, last value of 10-19)
// 29 (after 3 seconds, last value of 20-29)
```

### 2. Duration Observable Must Be Newly Generated Each Time

The function passed to `audit` must **return a new Observable each time**.

```ts
// ❌ Bad example: Reuse same Observable instance
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // Doesn't work after 2nd time
).subscribe();

// ✅ Good example: Generate new Observable each time
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

## 🆚 Comparison with Similar Operators

| Operator | Emission Timing | Emitted Value | Use Case |
|:---|:---|:---|:---|
| `audit` | At period **end** | **Last** value within period | Get latest state within period |
| `throttle` | At period **start** | **First** value within period | Get first of consecutive events |
| `debounce` | **After silence** | Value just before silence | Wait for input completion |
| `sample` | **When another Observable fires** | Latest value at that time | Periodic snapshots |


## 📚 Related Operators

- **[auditTime](/en/guide/operators/filtering/auditTime)** - Control with fixed time (simplified version of `audit`)
- **[throttle](/en/guide/operators/filtering/throttleTime)** - Emit first value at period start
- **[debounce](/en/guide/operators/filtering/debounceTime)** - Emit value after silence
- **[sample](/en/guide/operators/filtering/sampleTime)** - Sample at timing of another Observable

## Summary

The `audit` operator emits the last value within a period dynamically controlled by a custom Observable.

- ✅ Dynamic period control possible
- ✅ Adaptive sampling according to load
- ✅ Control based on other streams
- ⚠️ Must generate new Observable each time
- ⚠️ Be mindful of memory with frequent emissions
