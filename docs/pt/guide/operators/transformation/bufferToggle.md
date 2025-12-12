---
description: The bufferToggle operator is an advanced buffering operator that allows start and end triggers to be controlled by separate Observables and multiple buffering periods to be managed independently.
titleTemplate: ':title'
---

# bufferToggle - Buffer with Independent Control of Start and End

The `bufferToggle` operator controls the **start trigger** and **end trigger** with separate Observables and issues the values in an array. This is an advanced buffering operator that can manage multiple buffering periods simultaneously.

## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // Emit values every 0.5 seconds

// Start trigger: every 2 seconds
const opening$ = interval(2000);

// End trigger: 1 second after start
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output:
// [3, 4, 5]     (Starts at 2 sec, ends at 3 sec)
// [7, 8, 9]     (Starts at 4 sec, ends at 5 sec)
// [11, 12, 13]  (Starts at 6 sec, ends at 7 sec)
```

**Flow of operation**:
1. `opening$` emits a value → Buffering starts
2. Observable returned by `closing()` emits a value → Buffering ends, outputs array
3. Multiple buffering periods can overlap

[🌐 RxJS Official Documentation - `bufferToggle`](https://rxjs.dev/api/operators/bufferToggle)

## 🆚 Contrast with Other Buffer Operators

`bufferToggle` is unique compared to other buffer operators in that it allows **independent control** of start and end.

### Comparison of Each Operator

| Operator | Trigger | Feature | Use Case |
|---|---|---|---|
| `buffer(trigger$)` | Single Observable | Simple | Event-driven buffering |
| `bufferTime(ms)` | Time | Periodic | Data aggregation at regular intervals |
| `bufferCount(n)` | Count | Quantitative | Processing in units of N |
| `bufferToggle(open$, close)` | Separate start/end control | Flexible | Complex period management |

### Code Example Comparison

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // Emit 0-9 every 300ms

// bufferToggle: Independent control of start and end
const opening$ = interval(1000); // Start every 1 second
const closing = () => interval(500); // End 500ms after start

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output: [3, 4], [6, 7], [9]
//
// Timeline:
// 0ms  300ms 600ms 900ms 1200ms 1500ms 1800ms 2100ms 2400ms 2700ms
// 0    1     2     3     4      5      6      7      8      9
//                  [Start       End]   [Start       End]   [Start End]
//                  └→ [3,4]            └→ [6,7]            └→ [9]
```

**Usage guidelines**:
- **`buffer`** → Output buffer each time trigger Observable emits a value
- **`bufferTime`** → Automatically output buffer at regular intervals
- **`bufferCount`** → Output buffer when specified count is reached
- **`bufferToggle`** → Separate start/end control, overlapping periods possible

> [!TIP]
> For more details on each operator, see [buffer](/pt/guide/operators/transformation/buffer), [bufferTime](/pt/guide/operators/transformation/bufferTime), [bufferCount](/pt/guide/operators/transformation/bufferCount).

## 💡 Typical Usage Patterns

1. **Data Collection During Business Hours**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // Sensor data (always acquiring)
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       value: Math.random() * 100
     }))
   );

   // Business open: 9:00 (Simulation: after 2 seconds)
   const businessOpen$ = timer(2000, 10000); // After 2 sec, then every 10 sec

   // Business close: 5 seconds after start
   const businessClose = () => timer(5000);

   sensorData$.pipe(
     bufferToggle(businessOpen$, businessClose)
   ).subscribe(data => {
     console.log(`Data during business hours: ${data.length} items`);
     console.log(`Average: ${(data.reduce((sum, d) => sum + d.value, 0) / data.length).toFixed(2)}`);
   });
   ```

2. **Event Recording During Button Press**
   ```ts
   import { fromEvent, interval } from 'rxjs';
   import { bufferToggle, map, take } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Hold';
   document.body.appendChild(button);

   const display = document.createElement('div');
   display.style.marginTop = '10px';
   document.body.appendChild(display);

   // Data stream
   const data$ = interval(100).pipe(
     map(i => ({ id: i, timestamp: Date.now() }))
   );

   // Start: Mouse down
   const mouseDown$ = fromEvent(button, 'mousedown');

   // End: Mouse up (from mousedown to mouseup)
   const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

   data$.pipe(
     bufferToggle(mouseDown$, mouseUp)
   ).subscribe(events => {
     display.textContent = `Events recorded during hold: ${events.length} items`;
     console.log('Recorded events:', events);
   });
   ```

3. **Active User Action Recording**
   ```ts
   import { fromEvent, merge, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // User actions
   const clicks$ = fromEvent(document, 'click').pipe(
     map(() => ({ type: 'click' as const, timestamp: Date.now() }))
   );

   const scrolls$ = fromEvent(window, 'scroll').pipe(
     map(() => ({ type: 'scroll' as const, timestamp: Date.now() }))
   );

   const keypresses$ = fromEvent(document, 'keypress').pipe(
     map(() => ({ type: 'keypress' as const, timestamp: Date.now() }))
   );

   const actions$ = merge(clicks$, scrolls$, keypresses$);

   // Active state start: first action
   const activeStart$ = actions$;

   // Active state end: no action for 5 seconds
   const activeEnd = () => timer(5000);

   actions$.pipe(
     bufferToggle(activeStart$, activeEnd)
   ).subscribe(bufferedActions => {
     console.log(`Active session: ${bufferedActions.length} actions`);
     const summary = bufferedActions.reduce((acc, action) => {
       acc[action.type] = (acc[action.type] || 0) + 1;
       return acc;
     }, {} as Record<string, number>);
     console.log('Breakdown:', summary);
   });
   ```

## 🧠 Practical Code Example (Download Period Management)

This is an example of managing data download periods with start and stop buttons.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { bufferToggle, map, take } from 'rxjs';

// Create UI elements
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Data Download Management';
container.appendChild(title);

const startButton = document.createElement('button');
startButton.textContent = 'Start';
container.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Stop';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
container.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Waiting...';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// Data stream (generate download data every 1 second)
const downloadData$ = interval(1000).pipe(
  map(i => ({
    id: i,
    size: Math.floor(Math.random() * 1000) + 100, // 100-1100KB
    timestamp: new Date()
  }))
);

// Start and stop triggers
const start$ = fromEvent(startButton, 'click');
const stop$ = new Subject<void>();

fromEvent(stopButton, 'click').subscribe(() => {
  stop$.next();
  status.textContent = 'Stopped';
  startButton.disabled = false;
  stopButton.disabled = true;
});

start$.subscribe(() => {
  status.textContent = 'Downloading...';
  startButton.disabled = true;
  stopButton.disabled = false;
});

// Buffering
downloadData$.pipe(
  bufferToggle(start$, () => stop$)
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Download Complete</strong><br>
    Count: ${downloads.length} items<br>
    Total size: ${(totalSize / 1024).toFixed(2)} MB<br>
    Average size: ${avgSize.toFixed(0)} KB
  `;

  console.log('Download data:', downloads);
});
```

## 🎯 Overlapping Buffer Periods

One feature of `bufferToggle` is that it can manage multiple buffering periods simultaneously.

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Start: every 1 second
const opening$ = interval(1000);

// End: 1.5 seconds after start
const closing = () => interval(1500);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output:
// [4, 5, 6]        (Starts at 1 sec → Ends at 2.5 sec)
// [9, 10, 11, 12]  (Starts at 2 sec → Ends at 3.5 sec) ※Partially overlaps
// [14, 15, 16, 17] (Starts at 3 sec → Ends at 4.5 sec)
```

**Timeline**:
```
Source:    0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Start:     ----1sec----2sec----3sec----4sec
Period 1:  [------1.5sec-----]
            └→ Output: [4,5,6]
Period 2:         [------1.5sec-----]
                   └→ Output: [9,10,11,12]
Period 3:                [------1.5sec-----]
                          └→ Output: [14,15,16,17]
```

## 📋 Type-Safe Usage

Here is an example of a type-safe implementation utilizing generics in TypeScript.

```ts
import { Observable, Subject, interval } from 'rxjs';
import { bufferToggle, map } from 'rxjs';

interface MetricData {
  timestamp: Date;
  cpu: number;
  memory: number;
}

interface SessionControl {
  start$: Observable<void>;
  stop$: Observable<void>;
}

class MetricsCollector {
  private startSubject = new Subject<void>();
  private stopSubject = new Subject<void>();

  start(): void {
    this.startSubject.next();
  }

  stop(): void {
    this.stopSubject.next();
  }

  collectMetrics(source$: Observable<MetricData>): Observable<MetricData[]> {
    return source$.pipe(
      bufferToggle(
        this.startSubject,
        () => this.stopSubject
      )
    );
  }
}

// Usage example
const metricsStream$ = interval(500).pipe(
  map(() => ({
    timestamp: new Date(),
    cpu: Math.random() * 100,
    memory: Math.random() * 100
  } as MetricData))
);

const collector = new MetricsCollector();

collector.collectMetrics(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avgCpu = metrics.reduce((sum, m) => sum + m.cpu, 0) / metrics.length;
    const avgMemory = metrics.reduce((sum, m) => sum + m.memory, 0) / metrics.length;
    console.log(`Collection period: ${metrics.length} items`);
    console.log(`Average CPU: ${avgCpu.toFixed(1)}%`);
    console.log(`Average Memory: ${avgMemory.toFixed(1)}%`);
  }
});

// Start after 3 seconds
setTimeout(() => {
  console.log('Start collection');
  collector.start();
}, 3000);

// Stop after 6 seconds
setTimeout(() => {
  console.log('Stop collection');
  collector.stop();
}, 6000);
```

## 🔄 Difference from bufferWhen

`bufferToggle` and `bufferWhen` are similar, but control methods are different.

```ts
import { interval, timer } from 'rxjs';
import { bufferToggle, bufferWhen } from 'rxjs';

const source$ = interval(200);

// bufferToggle: Separate control of start and end
source$.pipe(
  bufferToggle(
    interval(1000),          // Start trigger
    () => timer(500)         // End trigger (500ms after start)
  )
).subscribe(console.log);

// bufferWhen: Control only end timing (next starts immediately after end)
source$.pipe(
  bufferWhen(() => timer(1000)) // Buffer every 1 second
).subscribe(console.log);
```

| Operator | Control | Buffer Period | Use Case |
|---|---|---|---|
| `bufferToggle(open$, close)` | Separate start/end control | Can overlap | Complex start/end conditions |
| `bufferWhen(closing)` | End only control | Continuous | Simple periodic buffer |

## ⚠️ Common Mistakes

> [!WARNING]
> `bufferToggle` can manage multiple buffer periods simultaneously, but if start triggers fire too frequently, many buffers will exist at the same time, consuming memory.

### Error: Start Triggers Too Frequent

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ❌ Bad example: Start every 100ms, end after 5 seconds
const opening$ = interval(100); // Too frequent
const closing = () => interval(5000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Up to 50 buffers can exist simultaneously → Memory risk
```

### Correct: Set Appropriate Interval

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ✅ Good example: Set appropriate interval for start
const opening$ = interval(2000); // Every 2 seconds
const closing = () => interval(1000); // Buffer for 1 second

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// At most 1-2 buffers exist simultaneously
```

## 🎓 Summary

### When to Use bufferToggle
- ✅ When you want to control start and end independently
- ✅ When you want to collect data for a limited period, such as during button press
- ✅ When you want to manage multiple buffering periods simultaneously
- ✅ Data collection under complex conditions, such as during business hours only

### When to Use buffer/bufferTime/bufferCount
- ✅ When simple periodic buffering is sufficient
- ✅ When a single trigger is sufficient for control

### When to Use bufferWhen
- ✅ When only the end condition needs to be dynamically controlled
- ✅ When continuous buffering periods are needed

### Notes
- ⚠️ Frequent start triggers cause many buffers to exist simultaneously, consuming memory
- ⚠️ Buffering periods may overlap
- ⚠️ Can be difficult to debug due to complex controls

## 🚀 Next Steps

- [buffer](/pt/guide/operators/transformation/buffer) - Learn basic buffering
- [bufferTime](/pt/guide/operators/transformation/bufferTime) - Learn time-based buffering
- [bufferCount](/pt/guide/operators/transformation/bufferCount) - Learn count-based buffering
- [bufferWhen](https://rxjs.dev/api/operators/bufferWhen) - Learn dynamic end control (official documentation)
- [Transformation Operator Practical Use Cases](/pt/guide/operators/transformation/practical-use-cases) - Learn real-world use cases
