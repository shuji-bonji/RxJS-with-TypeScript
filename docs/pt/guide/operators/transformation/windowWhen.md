---
description: The windowWhen operator is an RxJS operator that dynamically controls the end condition and splits Observable. It enables continuous stream processing where the next window starts immediately after the window ends.
---

# windowWhen - Dynamic End Control Window

The `windowWhen` operator divides Observable with **dynamic control of end conditions**. It provides a continuous stream processing pattern in which the next window starts immediately after the window ends.

## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500); // Emit values every 0.5 seconds

// End condition: after 1 second
const closingSelector = () => interval(1000);

source$.pipe(
  windowWhen(closingSelector),
  take(4),
  mergeAll()
).subscribe(value => {
  console.log('Value in window:', value);
});

// Window 1: 0       (Starts at 0 sec → Ends at 1 sec)
// Window 2: 1, 2    (Starts at 1 sec → Ends at 2 sec)
// Window 3: 3, 4    (Starts at 2 sec → Ends at 3 sec)
// Window 4: 5, 6    (Starts at 3 sec → Ends at 4 sec)
```

**Flow of operation**:
1. First window automatically starts
2. Observable returned by `closingSelector()` emits a value → Window ends
3. **Next window starts immediately**
4. Repeat 2-3

[🌐 RxJS Official Documentation - `windowWhen`](https://rxjs.dev/api/operators/windowWhen)

## 💡 Typical Usage Patterns

- Data collection at dynamic time intervals
- Adaptive stream processing based on load
- Window control based on previous results
- Continuous data grouping

## 🔍 Difference from bufferWhen

| Operator | Output | Use Case |
|:---|:---|:---|
| `bufferWhen` | **Array (T[])** | Process grouped values together |
| `windowWhen` | **Observable&lt;T&gt;** | Different stream processing for each group |

```ts
import { interval } from 'rxjs';
import { bufferWhen, windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500);
const closing = () => interval(1000);

// bufferWhen - Output as array
source$.pipe(
  bufferWhen(closing),
  take(3)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Output: Buffer (array): [0]
  // Output: Buffer (array): [1, 2]
  // Output: Buffer (array): [3, 4]
});

// windowWhen - Output as Observable
source$.pipe(
  windowWhen(closing),
  take(3),
  mergeAll()
).subscribe(value => {
  console.log('Value in window:', value);
  // Output: Value in window: 0
  // Output: Value in window: 1
  // Output: Value in window: 2
  // ...
});
```

## 🧠 Practical Code Example 1: Data Collection at Dynamic Time Intervals

This is an example of adjusting the next window period based on the results of the previous window.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, scan, map } from 'rxjs';

// Sensor data (always generating)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10 // 20-30 degrees
  }))
);

let windowNumber = 0;
let previousAvgTemp = 25;

sensorData$.pipe(
  windowWhen(() => {
    const current = ++windowNumber;
    // Shorter interval when temperature is higher
    const duration = previousAvgTemp > 27 ? 500 : 1000;
    console.log(`Window ${current} started (duration: ${duration}ms)`);
    return timer(duration);
  }),
  mergeMap(window$ => {
    const currentWindow = windowNumber;  // Keep current window number
    return window$.pipe(
      toArray(),
      map(data => {
        const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
        previousAvgTemp = avgTemp;
        return {
          window: currentWindow,  // Use kept window number
          count: data.length,
          avgTemp
        };
      })
    );
  })
).subscribe(stats => {
  console.log(`Window ${stats.window}: Average temp ${stats.avgTemp.toFixed(1)}°C, ${stats.count} samples`);
});
```

## 🎯 Practical Code Example 2: Adaptive Stream Processing Based on Load

This is an example of dynamically changing window length based on system load.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { windowWhen, mergeMap, scan, map } from 'rxjs';

// Create output area
const container = document.createElement('div');
document.body.appendChild(container);

const loadButton = document.createElement('button');
loadButton.textContent = 'Generate Load';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Low load: Collect at 5-second intervals';
container.appendChild(status);

const logDisplay = document.createElement('div');
logDisplay.style.marginTop = '10px';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// Log stream (always generating)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    timestamp: new Date()
  }))
);

// Load level
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
  const interval = getWindowDuration(loadLevel);
  const loadText = loadLevel === 0 ? 'Low load' :
                   loadLevel <= 2 ? 'Medium load' : 'High load';
  status.textContent = `${loadText} (Level ${loadLevel}): Collect at ${interval / 1000}-second intervals`;
}

function getWindowDuration(load: number): number {
  // Higher load = shorter interval
  switch (load) {
    case 0: return 5000;
    case 1: return 3000;
    case 2: return 2000;
    case 3: return 1000;
    case 4: return 500;
    default: return 300;
  }
}

let windowNum = 0;

// Adaptive window processing
logs$.pipe(
  windowWhen(() => {
    windowNum++;
    return timer(getWindowDuration(loadLevel));
  }),
  mergeMap(window$ =>
    window$.pipe(
      scan((stats, log) => ({
        count: stats.count + 1,
        errors: stats.errors + (log.level === 'ERROR' ? 1 : 0),
        window: windowNum
      }), { count: 0, errors: 0, window: windowNum })
    )
  )
).subscribe(stats => {
  const timestamp = new Date().toLocaleTimeString();
  const div = document.createElement('div');
  div.textContent = `[${timestamp}] Window ${stats.window}: ${stats.count} items (Errors: ${stats.errors})`;
  logDisplay.insertBefore(div, logDisplay.firstChild);
});
```

## 🆚 Difference from windowToggle

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowWhen: Control only end (next starts immediately after end)
source$.pipe(
  windowWhen(() => timer(1000)),
  mergeAll()
).subscribe();

// windowToggle: Separate control of start and end
source$.pipe(
  windowToggle(
    interval(1000),          // Start trigger
    () => timer(500)         // End trigger (500ms after start)
  ),
  mergeAll()
).subscribe();
```

| Operator | Control | Window Period | Use Case |
|:---|:---|:---|:---|
| `windowWhen(closing)` | End only control | Continuous | Simple periodic window |
| `windowToggle(open$, close)` | Separate start/end control | Can overlap | Complex start/end conditions |

**Usage guidelines**:
- **`windowWhen`**: Process all data continuously without omission (logging, data aggregation, etc.)
- **`windowToggle`**: Process data only for a specific period (during business hours, button presses, etc.)

## 🎯 Practical Example: Adaptive Window Size Control

Here is an example of automatically adjusting the next window period based on the results of the previous window.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, map } from 'rxjs';

interface WindowStats {
  count: number;
  nextDuration: number;
}

const data$ = interval(100);

let previousCount = 0;

// Adjust next window period based on data volume
function getNextDuration(count: number): number {
  if (count > 20) {
    return 500;  // High data volume → Short interval
  } else if (count > 10) {
    return 1000; // Medium → Medium interval
  } else {
    return 2000; // Low data volume → Long interval
  }
}

data$.pipe(
  windowWhen(() => timer(getNextDuration(previousCount))),
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(data => {
        previousCount = data.length;
        return {
          count: data.length,
          nextDuration: getNextDuration(data.length)
        } as WindowStats;
      })
    )
  )
).subscribe(stats => {
  console.log(`Window size: ${stats.count} items, Next duration: ${stats.nextDuration}ms`);
});
```

## ⚠️ Notes

### 1. Window Subscription Management

Each window is an independent Observable, so you must either subscribe to it explicitly or flatten it with `mergeAll()` or similar.

```ts
source$.pipe(
  windowWhen(closing)
).subscribe(window$ => {
  // Values won't flow unless you subscribe to the window itself
  window$.subscribe(value => {
    console.log('Value:', value);
  });
});
```

### 2. Need to Return a New Observable Each Time

The `closingSelector` function **must return a new Observable each time**. If it returns the same instance, it will not work properly.

```ts
// ❌ Bad example: Reusing the same Observable instance
const closingObservable = timer(1000);

source$.pipe(
  windowWhen(() => closingObservable) // Won't work from 2nd time onwards!
).subscribe();

// ✅ Good example: Generate new Observable each time
source$.pipe(
  windowWhen(() => timer(1000)) // Generate new timer each time
).subscribe();
```

### 3. Beware of Overly Complex End Conditions

Overly complex end conditions can make debugging difficult.

```ts
// Too complex example
let counter = 0;
source$.pipe(
  windowWhen(() => {
    counter++;
    const duration = counter % 3 === 0 ? 500 :
                     counter % 2 === 0 ? 1000 : 1500;
    return timer(duration);
  })
).subscribe();
// Difficult to debug
```

## 🆚 Comparison of window Operators

| Operator | Control | Window Period | Use Case |
|:---|:---|:---|:---|
| `window` | Another Observable emits | Continuous | Event-driven partitioning |
| `windowTime` | Fixed time interval | Continuous | Time-based partitioning |
| `windowCount` | Fixed count | Continuous | Count-based partitioning |
| `windowToggle` | Separate start/end control | Can overlap | Complex start/end conditions |
| `windowWhen` | **Dynamic end control only** | **Continuous** | **Adaptive window processing** |

## 📚 Related Operators

- [bufferWhen](/pt/guide/operators/transformation/bufferWhen) - Collect values as array (array version of windowWhen)
- [window](/pt/guide/operators/transformation/window) - Split window at different Observable timings
- [windowTime](/pt/guide/operators/transformation/windowTime) - Time-based window splitting
- [windowCount](/pt/guide/operators/transformation/windowCount) - Count-based window splitting
- [windowToggle](/pt/guide/operators/transformation/windowToggle) - Window control with start and end Observables

## Summary

The `windowWhen` operator is a useful tool for dynamically controlling end conditions and continuous window processing.

- ✅ End conditions can be dynamically controlled
- ✅ Continuous window processing (no data leakage)
- ✅ Can adjust next window based on previous results
- ⚠️ Subscription management required
- ⚠️ Need to return a new Observable each time
- ⚠️ Be careful not to overcomplicate end conditions
