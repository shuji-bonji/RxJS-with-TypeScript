---
description: windowToggle is an advanced RxJS conversion operator that allows multiple window periods to be managed independently, with start and end triggers controlled by separate Observables. It is ideal for situations where dynamic period management is required, such as data collection during business hours or event recording during button presses. TypeScript type inference ensures type-safe window splitting processing.
titleTemplate: ':title | RxJS'
---

# windowToggle - Window with Independent Start and End Control

The `windowToggle` operator controls **start trigger** and **end trigger** with separate Observables, issuing each period as a new Observable. This is an advanced window operator that can manage multiple window periods simultaneously.

## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500); // Emit values every 0.5 seconds

// Start trigger: every 2 seconds
const opening$ = interval(2000);

// End trigger: 1 second after start
const closing = () => interval(1000);

source$.pipe(
  windowToggle(opening$, closing),
  mergeAll()
).subscribe(value => {
  console.log('Value in window:', value);
});

// Starts at 2 sec, ends at 3 sec → Values: 4, 5
// Starts at 4 sec, ends at 5 sec → Values: 8, 9
// Starts at 6 sec, ends at 7 sec → Values: 12, 13
```

**Flow of operation**:
1. `opening$` emits a value → Window starts
2. Observable returned by `closing()` emits a value → Window ends
3. Multiple window periods can overlap

[🌐 RxJS Official Documentation - `windowToggle`](https://rxjs.dev/api/operators/windowToggle)

## 💡 Typical Usage Patterns

- Data collection during business hours
- Event recording during button presses
- Action tracking during active sessions
- Stream processing requiring dynamic period management

## 🔍 Difference from bufferToggle

| Operator | Output | Use Case |
|:---|:---|:---|
| `bufferToggle` | **Array (T[])** | Process grouped values together |
| `windowToggle` | **Observable&lt;T&gt;** | Different stream processing for each group |

```ts
import { interval } from 'rxjs';
import { bufferToggle, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500);
const opening$ = interval(2000);
const closing = () => interval(1000);

// bufferToggle - Output as array
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Output: Buffer (array): [4, 5]
});

// windowToggle - Output as Observable
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  console.log('Window (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Value in window:', value);
  });
});
```

## 🧠 Practical Code Example 1: Recording Events During Button Press

This is an example of recording data between mouse down and mouse up.

```ts
import { fromEvent, interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

// Create button
const button = document.createElement('button');
button.textContent = 'Hold';
document.body.appendChild(button);

// Output area
const display = document.createElement('div');
display.style.marginTop = '10px';
document.body.appendChild(display);

// Data stream (every 100ms)
const data$ = interval(100);

// Start: Mouse down
const mouseDown$ = fromEvent(button, 'mousedown');

// End: Mouse up
const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

data$.pipe(
  windowToggle(mouseDown$, mouseUp),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(events => {
  display.textContent = `Events recorded during hold: ${events.length} items`;
  console.log('Recorded data:', events);
});
```

## 🎯 Practical Code Example 2: Data Collection During Business Hours

This is an example of collecting sensor data from the start of business hours to the end of business hours.

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, mergeMap, scan, map } from 'rxjs';

// Sensor data (always acquiring)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10, // 20-30 degrees
    humidity: 40 + Math.random() * 20     // 40-60%
  }))
);

// Business open: after 2 seconds, then every 10 seconds
const businessOpen$ = timer(2000, 10000);

// Business close: 5 seconds after start
const businessClose = () => timer(5000);

let sessionNumber = 0;

sensorData$.pipe(
  windowToggle(businessOpen$, businessClose),
  mergeMap(window$ => {
    const current = ++sessionNumber;
    console.log(`Business session ${current} started`);

    // Calculate statistics for each window
    return window$.pipe(
      scan((stats, data) => ({
        count: stats.count + 1,
        totalTemp: stats.totalTemp + data.temperature,
        totalHumidity: stats.totalHumidity + data.humidity
      }), { count: 0, totalTemp: 0, totalHumidity: 0 }),
      map(stats => ({
        session: current,
        count: stats.count,
        avgTemp: stats.totalTemp / stats.count,
        avgHumidity: stats.totalHumidity / stats.count
      }))
    );
  })
).subscribe(stats => {
  console.log(`Session ${stats.session}: ${stats.count} samples`);
  console.log(`  Average temperature: ${stats.avgTemp.toFixed(1)}°C`);
  console.log(`  Average humidity: ${stats.avgHumidity.toFixed(1)}%`);
});
```

## 🎯 Practical Example: Download Period Management

This is an example of managing data download periods with start and stop buttons.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { windowToggle, mergeMap, toArray, map } from 'rxjs';

// Create UI elements
const startButton = document.createElement('button');
startButton.textContent = 'Start';
document.body.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Stop';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
document.body.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Waiting...';
document.body.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
document.body.appendChild(result);

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

// Window management
downloadData$.pipe(
  windowToggle(start$, () => stop$),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Download Complete</strong><br>
    Count: ${downloads.length} items<br>
    Total size: ${(totalSize / 1024).toFixed(2)} MB<br>
    Average size: ${avgSize.toFixed(0)} KB
  `;
});
```

## 🎯 Overlapping Window Periods

One feature of `windowToggle` is that it can manage multiple window periods simultaneously.

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Start: every 1 second
const opening$ = interval(1000);

// End: 1.5 seconds after start
const closing = () => interval(1500);

source$.pipe(
  windowToggle(opening$, closing),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Window:', values);
});

// Output:
// Window: [4, 5, 6, 7]       (Starts at 1 sec → Ends at 2.5 sec)
// Window: [9, 10, 11, 12]    (Starts at 2 sec → Ends at 3.5 sec)
// Window: [14, 15, 16, 17]   (Starts at 3 sec → Ends at 4.5 sec)
```

**Timeline**:
```
Source:    0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Start:     ----1sec----2sec----3sec----4sec
Period 1:  [------1.5sec-----]
            └→ Window 1: [4,5,6,7]
Period 2:         [------1.5sec-----]
                   └→ Window 2: [9,10,11,12]
Period 3:                [------1.5sec-----]
                          └→ Window 3: [14,15,16,17]
```

## ⚠️ Notes

### 1. Window Subscription Management

Each window is an independent Observable, so you must either subscribe to it explicitly or flatten it with `mergeAll()` or similar.

```ts
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  // Values won't flow unless you subscribe to the window itself
  window$.subscribe(value => {
    console.log('Value:', value);
  });
});
```

### 2. Watch Out for Memory Leaks

If start triggers are too frequent, many windows will exist at the same time, consuming memory.

```ts
// ❌ Bad example: Start every 100ms, end after 5 seconds
const opening$ = interval(100); // Too frequent
const closing = () => interval(5000);

source$.pipe(
  windowToggle(opening$, closing)
).subscribe();
// Up to 50 windows can exist simultaneously → Memory risk

// ✅ Good example: Set appropriate interval
const opening$ = interval(2000); // Every 2 seconds
const closing = () => interval(1000); // For 1 second
```

### 3. Overlapping Window Periods

Overlapping window periods result in the same value being contained in multiple windows. Check if this is the intended behavior.

```ts
// With overlap
opening$ = interval(1000);    // Start every 1 second
closing = () => interval(1500); // For 1.5 seconds

// Without overlap
opening$ = interval(2000);    // Start every 2 seconds
closing = () => interval(1000); // For 1 second
```

## 🆚 Comparison of window Operators

| Operator | Control | Window Period | Use Case |
|:---|:---|:---|:---|
| `window` | Another Observable emits | Continuous | Event-driven partitioning |
| `windowTime` | Fixed time interval | Continuous | Time-based partitioning |
| `windowCount` | Fixed count | Continuous | Count-based partitioning |
| `windowToggle` | **Separate start/end control** | **Can overlap** | **Complex start/end conditions** |
| `windowWhen` | End only control | Continuous | Simple periodic control |

## 🔄 Difference from windowWhen

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, windowWhen, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowToggle: Separate control of start and end
source$.pipe(
  windowToggle(
    interval(1000),          // Start trigger
    () => timer(500)         // End trigger (500ms after start)
  ),
  mergeAll()
).subscribe();

// windowWhen: Control only end timing (next starts immediately after end)
source$.pipe(
  windowWhen(() => timer(1000)), // Window every 1 second
  mergeAll()
).subscribe();
```

| Operator | Control | Window Period | Use Case |
|:---|:---|:---|:---|
| `windowToggle(open$, close)` | Separate start/end control | Can overlap | Complex start/end conditions |
| `windowWhen(closing)` | End only control | Continuous | Simple periodic window |

## 📚 Related Operators

- [bufferToggle](/en/guide/operators/transformation/bufferToggle) - Collect values as array (array version of windowToggle)
- [window](/en/guide/operators/transformation/window) - Split window at different Observable timings
- [windowTime](/en/guide/operators/transformation/windowTime) - Time-based window splitting
- [windowCount](/en/guide/operators/transformation/windowCount) - Count-based window splitting
- [windowWhen](/en/guide/operators/transformation/windowWhen) - Window splitting with dynamic closing conditions

## Summary

The `windowToggle` operator is an advanced tool that allows you to control the start and end independently and treat each period as an independent Observable.

- ✅ Start and end can be controlled separately
- ✅ Multiple windows can be managed simultaneously
- ✅ Different processing can be applied to each window
- ⚠️ Subscription management required
- ⚠️ Frequent start triggers consume memory
- ⚠️ Be aware of overlapping window periods
