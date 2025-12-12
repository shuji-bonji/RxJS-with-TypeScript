---
description: bufferWhen is an RxJS conversion operator that dynamically controls the end condition and publishes values in an array. It enables continuous buffering where the next buffer starts immediately after the buffer ends, and can be used for flexible data aggregation based on load, such as adaptive batch processing and log collection. TypeScript type inference enables type-safe dynamic buffering.
---

# bufferWhen - Dynamic End Control Buffer

The `bufferWhen` operator publishes an array of values with **dynamically controlled end conditions**. It provides a continuous buffering pattern where one buffer ends and the next buffer starts immediately.

## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(500); // Emit values every 0.5 seconds

// End condition: after 1 second
const closingSelector = () => interval(1000);

source$.pipe(
  bufferWhen(closingSelector),
  take(4)
).subscribe(console.log);
// Output:
// [0]           (Starts at 0 sec → Ends at 1 sec, value 0 only)
// [1, 2, 3]     (Starts at 1 sec → Ends at 2 sec, values 1,2,3)
// [4, 5]        (Starts at 2 sec → Ends at 3 sec, values 4,5)
// [6, 7]        (Starts at 3 sec → Ends at 4 sec, values 6,7)
```

**Flow of operation**:
1. First buffer starts automatically
2. Observable returned by `closingSelector()` emits a value → Buffer ends, outputs array
3. **Next buffer starts immediately** (often at the same time as source$ emission)
4. Repeat 2-3

> [!NOTE]
> The first buffer only contains `[0]` because it's the 1 second period until `interval(1000)` emits its first value. From the second buffer onwards, buffer start and `source$` emission coincide, so they contain more values.

[🌐 RxJS Official Documentation - `bufferWhen`](https://rxjs.dev/api/operators/bufferWhen)

## 🆚 Difference from bufferToggle

`bufferWhen` and `bufferToggle` are similar, but **their control methods and behavior patterns are very different**.

### bufferWhen Behavior

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Emit 0-11 every 300ms

// bufferWhen: Control only end (next starts immediately after end)
source$.pipe(
  bufferWhen(() => interval(1000))
).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8, 9], [10, 11]
//
// Timeline:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms 3600ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//  [----------1sec----------][----------1sec----------][----------1sec----------][-----1sec-----]
//   Buffer1(0-2)              Buffer2(3-5)              Buffer3(6-9)             Buffer4(10-11)
//   Continuous, no overlap, next starts immediately
```

### bufferToggle Behavior

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Emit 0-11 every 300ms

// bufferToggle: Separate control of start and end (can overlap)
const opening$ = interval(1000); // Start every 1 second
const closing = () => interval(800); // End 800ms after start

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output: [3, 4, 5], [6, 7, 8], [9, 10, 11]
//
// Timeline:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//        ----Start1(1000ms)----[---End after 800ms(1800ms)---]
//                        3      4      5
//                        └→ [3,4,5]
//                    ----Start2(2000ms)----[---End after 800ms(2800ms)---]
//                                            6      7      8
//                                            └→ [6,7,8]
//                              ----Start3(3000ms)----[---End after 800ms(3800ms)---]
//                                                      9      10     11
//                                                      └→ [9,10,11]
//  Waits for start trigger, periods are independent (0-2 before buffer start not included)
```

### Main Differences

| Operator | Start Control | End Control | Buffer Period | Feature |
|---|---|---|---|---|
| `bufferWhen(closing)` | Auto (immediately after end) | Dynamic | Continuous | No gap between buffers |
| `bufferToggle(open$, close)` | Independent Observable | Dynamic | Independent, can overlap | Gap between buffers |

**Usage guidelines**:
- **`bufferWhen`**: Buffer all data continuously without omission (logging, data aggregation, etc.)
- **`bufferToggle`**: Collect data only during specific periods (during business hours, button presses, etc.)

> [!TIP]
> - **Continuous buffering** (no data leakage) → `bufferWhen`
> - **Limited period buffering** (explicit start/end control) → `bufferToggle`

## 💡 Typical Usage Patterns

1. **Data Collection at Dynamic Time Intervals**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // Sensor data
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       temperature: 20 + Math.random() * 10
     }))
   );

   // End condition: Dynamically change based on previous temperature
   let previousAvgTemp = 25;

   sensorData$.pipe(
     bufferWhen(() => {
       // Higher temperature = shorter buffer interval
       const duration = previousAvgTemp > 27 ? 500 : 1000;
       return timer(duration);
     })
   ).subscribe(data => {
     const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
     previousAvgTemp = avgTemp;
     console.log(`Average temp: ${avgTemp.toFixed(1)}°C, Samples: ${data.length}`);
   });
   ```

2. **Adaptive Batch Processing Based on Load**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   interface Task {
     id: number;
     timestamp: number;
   }

   // Task stream
   let taskCounter = 0;
   const tasks$ = fromEvent(document, 'click').pipe(
     map(() => ({
       id: taskCounter++,
       timestamp: Date.now()
     } as Task))
   );

   // Adjust next buffer period based on buffer size
   tasks$.pipe(
     bufferWhen(() => timer(2000))
   ).subscribe(bufferedTasks => {
     if (bufferedTasks.length > 0) {
       console.log(`Batch processing: ${bufferedTasks.length} tasks`);
       console.log('Task IDs:', bufferedTasks.map(t => t.id));

       // Dynamically determine next buffer period
       // (In practice, move this logic inside bufferWhen function)
     }
   });
   ```

3. **Sampling at Random Intervals**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // Data stream
   const data$ = interval(100).pipe(
     map(i => ({
       value: Math.sin(i / 10) * 100,
       timestamp: Date.now()
     }))
   );

   // Buffer at random intervals (500ms ~ 2000ms)
   data$.pipe(
     bufferWhen(() => {
       const randomDelay = 500 + Math.random() * 1500;
       return timer(randomDelay);
     })
   ).subscribe(samples => {
     const avg = samples.reduce((sum, s) => sum + s.value, 0) / samples.length;
     console.log(`Sample count: ${samples.length}, Average: ${avg.toFixed(2)}`);
   });
   ```

## 🧠 Practical Code Example (Load-Based Log Collection)

This is an example of dynamically changing log collection frequency based on system load.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { bufferWhen, map, share } from 'rxjs';

// Create UI elements
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Adaptive Log Collection System';
container.appendChild(title);

const loadButton = document.createElement('button');
loadButton.textContent = 'Generate Load';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#f0f0f0';
status.textContent = 'Low load: Collect at 5-second intervals';
container.appendChild(status);

const logDisplay = document.createElement('pre');
logDisplay.style.marginTop = '10px';
logDisplay.style.padding = '10px';
logDisplay.style.backgroundColor = '#f9f9f9';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// Log stream (always generating)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    message: `Log message ${logCounter}`,
    timestamp: new Date()
  })),
  share()
);

// Load counter (increment on button click)
let loadLevel = 0;
fromEvent(loadButton, 'click').subscribe(() => {
  loadLevel = Math.min(loadLevel + 1, 5);
  updateStatus();
});

// Decrease load every 30 seconds
interval(30000).subscribe(() => {
  loadLevel = Math.max(loadLevel - 1, 0);
  updateStatus();
});

function updateStatus() {
  const interval = getBufferInterval(loadLevel);
  const loadText = loadLevel === 0 ? 'Low load' :
                   loadLevel <= 2 ? 'Medium load' : 'High load';
  status.textContent = `${loadText} (Level ${loadLevel}): Collect at ${interval / 1000}-second intervals`;
  status.style.backgroundColor =
    loadLevel === 0 ? '#d4edda' :
    loadLevel <= 2 ? '#fff3cd' : '#f8d7da';
}

function getBufferInterval(load: number): number {
  // Higher load = shorter buffer interval
  switch (load) {
    case 0: return 5000;  // 5 seconds
    case 1: return 3000;  // 3 seconds
    case 2: return 2000;  // 2 seconds
    case 3: return 1000;  // 1 second
    case 4: return 500;   // 0.5 seconds
    default: return 300;  // 0.3 seconds
  }
}

// Adaptive buffering
logs$.pipe(
  bufferWhen(() => timer(getBufferInterval(loadLevel)))
).subscribe(bufferedLogs => {
  if (bufferedLogs.length > 0) {
    const errors = bufferedLogs.filter(log => log.level === 'ERROR').length;
    const timestamp = new Date().toLocaleTimeString();

    const summary = `[${timestamp}] Collected: ${bufferedLogs.length} items (Errors: ${errors})\n`;
    logDisplay.textContent = summary + logDisplay.textContent;

    console.log('Collected logs:', bufferedLogs);
  }
});
```

## 📋 Type-Safe Usage

Here is an example of a type-safe implementation utilizing generics in TypeScript.

```ts
import { Observable, interval, timer } from 'rxjs';
import { bufferWhen, map } from 'rxjs';

interface MetricData {
  value: number;
  timestamp: Date;
  source: string;
}

interface BufferConfig {
  minDuration: number;
  maxDuration: number;
  adaptive: boolean;
}

class AdaptiveBuffer<T> {
  constructor(private config: BufferConfig) {}

  private getNextBufferDuration(previousCount: number): number {
    if (!this.config.adaptive) {
      return this.config.minDuration;
    }

    // Adjust next buffer period based on data volume
    const ratio = Math.min(previousCount / 10, 1);
    const duration =
      this.config.minDuration +
      (this.config.maxDuration - this.config.minDuration) * (1 - ratio);

    return Math.floor(duration);
  }

  apply(source$: Observable<T>): Observable<T[]> {
    let previousCount = 0;

    return source$.pipe(
      bufferWhen(() => {
        const duration = this.getNextBufferDuration(previousCount);
        return timer(duration);
      }),
      map(buffer => {
        previousCount = buffer.length;
        return buffer;
      })
    );
  }
}

// Usage example
const metricsStream$ = interval(300).pipe(
  map(i => ({
    value: Math.random() * 100,
    timestamp: new Date(),
    source: `sensor-${i % 3}`
  } as MetricData))
);

const buffer = new AdaptiveBuffer<MetricData>({
  minDuration: 1000,  // Minimum 1 second
  maxDuration: 5000,  // Maximum 5 seconds
  adaptive: true      // Adaptive
});

buffer.apply(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avg = metrics.reduce((sum, m) => sum + m.value, 0) / metrics.length;
    console.log(`Buffer size: ${metrics.length}, Average: ${avg.toFixed(2)}`);
  }
});
```

## 🎯 Comparison with Other Buffer Operators

```ts
import { interval, timer, Subject } from 'rxjs';
import { buffer, bufferTime, bufferCount, bufferWhen, bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0-9

// 1. buffer: Fixed trigger
const trigger$ = new Subject<void>();
source$.pipe(buffer(trigger$)).subscribe(console.log);
setInterval(() => trigger$.next(), 1000);
// Output: [0, 1, 2], [3, 4, 5], ... (at trigger timing)

// 2. bufferTime: Fixed time interval
source$.pipe(bufferTime(1000)).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 3. bufferCount: Fixed count
source$.pipe(bufferCount(3)).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 4. bufferWhen: Dynamic end control (continuous)
source$.pipe(
  bufferWhen(() => timer(1000))
).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 5. bufferToggle: Independent start/end control (can overlap)
const opening$ = interval(1000);
const closing = () => timer(800);
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output: [3, 4, 5], [6, 7, 8]
```

| Operator | Trigger | Dynamic Control | Overlap | Use Case |
|---|---|---|---|---|
| `buffer` | External Observable | ❌ | ❌ | Event-driven |
| `bufferTime` | Fixed time | ❌ | ❌ | Periodic aggregation |
| `bufferCount` | Fixed count | ❌ | ❌ | Quantitative processing |
| `bufferWhen` | Dynamic (end only) | ✅ | ❌ | Adaptive batch processing |
| `bufferToggle` | Dynamic (start and end) | ✅ | ✅ | Complex period management |

## ⚠️ Common Mistakes

> [!WARNING]
> The `bufferWhen` end condition function **must return a new Observable each time**. If it returns the same Observable instance, it will not work properly.

### Error: Returning the Same Observable Instance

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ❌ Bad example: Reusing the same Observable instance
const closingObservable = timer(1000);

source$.pipe(
  bufferWhen(() => closingObservable) // Won't work from 2nd time onwards!
).subscribe(console.log);
// Only the first buffer is output, nothing after that
```

### Correct: Return a New Observable Each Time

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ✅ Good example: Generate new Observable each time
source$.pipe(
  bufferWhen(() => timer(1000)) // Generate new timer each time
).subscribe(console.log);
// Output: [0, 1], [2, 3], [4, 5], ...
```

> [!IMPORTANT]
> The `closingSelector` function is **always called** each time the previous buffer ends, and is expected to return a new Observable.

## 🎓 Summary

### When to Use bufferWhen
- ✅ When you want to dynamically control the end condition
- ✅ When continuous buffering periods are needed
- ✅ When you want to adjust the next period based on previous buffer results
- ✅ When you want to implement adaptive batch processing

### When to Use bufferToggle
- ✅ When you want to control start and end independently
- ✅ When buffer periods may overlap
- ✅ When there are clear start/end events, such as button presses

### When to Use bufferTime
- ✅ When buffering at fixed time intervals is sufficient
- ✅ When a simple implementation is required

### Notes
- ⚠️ `closingSelector` must return a new Observable each time
- ⚠️ Overly complex end conditions make debugging difficult
- ⚠️ With adaptive controls, testing is important to avoid unexpected behavior

## 🚀 Next Steps

- [buffer](/pt/guide/operators/transformation/buffer) - Learn basic buffering
- [bufferTime](/pt/guide/operators/transformation/bufferTime) - Learn time-based buffering
- [bufferCount](/pt/guide/operators/transformation/bufferCount) - Learn count-based buffering
- [bufferToggle](/pt/guide/operators/transformation/bufferToggle) - Learn buffering with independent start and end controls
- [Transformation Operator Practical Use Cases](/pt/guide/operators/transformation/practical-use-cases) - Learn real-world use cases
