---
description: The sampleTime operator is an RxJS filtering operator that periodically samples the latest value from a stream at specified time intervals. It is ideal for taking periodic snapshots.
titleTemplate: ':title | RxJS'
---

# sampleTime - Sample Latest Value at Specified Time Intervals

The `sampleTime` operator **periodically samples** and outputs the **latest value** from the source Observable at **specified time intervals**.
Like periodic snapshots, it gets the latest value at that point in time.

## 🔰 Basic Syntax and Usage

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Sample every 2 seconds');
});
```

**Flow of operation**:
1. Timer fires periodically every 2 seconds
2. If there is a latest click event at that time, output it
3. If there is no value during the sample period, nothing is output

[🌐 RxJS Official Documentation - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

## 💡 Typical Usage Patterns

- **Periodic sensor data acquisition**: Latest temperature or position information every second
- **Real-time dashboard**: Periodic status updates
- **Performance monitoring**: Metrics collection at regular intervals
- **Game frame processing**: Periodic sampling for FPS control

## 🧠 Practical Code Example: Periodic Mouse Position Sampling

Example of sampling and displaying mouse position every second.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// Create UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Mouse Position Sampling (every second)';
container.appendChild(title);

const area = document.createElement('div');
area.style.width = '100%';
area.style.height = '300px';
area.style.border = '2px solid #4CAF50';
area.style.backgroundColor = '#f5f5f5';
area.style.display = 'flex';
area.style.alignItems = 'center';
area.style.justifyContent = 'center';
area.style.fontSize = '18px';
area.textContent = 'Move your mouse within this area';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// Mouse move event
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // Sample every second
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Sample #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Position: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Display maximum 10 items
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- Even if you keep moving the mouse, only the latest position at that moment is sampled every second.
- If you don't move the mouse for 1 second, nothing is output during that period.

## 🆚 Comparison with Similar Operators

### sampleTime vs throttleTime vs auditTime

| Operator | Firing Timing | Emitted Value | Use Case |
|:---|:---|:---|:---|
| `sampleTime(1000)` | **Regular timing every 1 second** | Latest value at that time | Periodic snapshots |
| `throttleTime(1000)` | Ignore for 1 second after value reception | First value at period start | Event thinning |
| `auditTime(1000)` | 1 second after value reception | Last value within period | Latest state within period |

**Visual Difference**:

```
Input: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (periodically sample)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (pass first and ignore during period)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (last value at period end)
```

## ⚠️ Notes

### 1. When There is No Value During Sample Period

If there is no new value at the sample timing, nothing is output.

### 2. Wait Until First Sample Timing

`sampleTime` outputs nothing until the specified time has elapsed.

### 3. Completion Timing

Even if the source completes, completion is not propagated until the next sample timing.

### 4. Memory Usage

Memory efficiency is good because it holds only one latest value internally.

## 💡 Difference from sample

`sample` uses another Observable as a trigger, while `sampleTime` uses fixed time intervals.

```ts
import { interval, fromEvent } from 'rxjs';
import { sample, sampleTime } from 'rxjs';

const source$ = interval(100);

// sampleTime: Fixed time interval (every 1 second)
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));

// sample: Use another Observable as trigger
const clicks$ = fromEvent(document, 'click');
source$.pipe(
  sample(clicks$)
).subscribe(val => console.log('sample:', val));
// Output latest value at that time every time you click
```

| Operator | Trigger | Use Case |
|:---|:---|:---|
| `sampleTime(ms)` | Fixed time interval | Periodic sampling |
| `sample(notifier$)` | Another Observable | Sampling at dynamic timing |

## 📚 Related Operators

- **[sample](https://rxjs.dev/api/operators/sample)** - Sample using another Observable as trigger (official documentation)
- **[throttleTime](/en/guide/operators/filtering/throttleTime)** - Get first value at period start
- **[auditTime](/en/guide/operators/filtering/auditTime)** - Get last value at period end
- **[debounceTime](/en/guide/operators/filtering/debounceTime)** - Emit value after silence

## Summary

The `sampleTime` operator periodically samples the latest value at specified time intervals.

- ✅ Ideal for periodic snapshot acquisition
- ✅ Effective for thinning high frequency streams
- ✅ Good memory efficiency (holds only 1 latest value)
- ✅ Ideal for dashboards and monitoring
- ⚠️ Outputs nothing if there is no value during sample period
- ⚠️ Wait time until first sample
- ⚠️ Completion propagates at next sample timing
