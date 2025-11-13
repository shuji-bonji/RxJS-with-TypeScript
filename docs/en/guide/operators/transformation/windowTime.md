---
description: windowTime is an RxJS operator that can divide an Observable at regular time intervals and process values issued in each time frame as a separate Observable.
---

# windowTime - Split Observable at Regular Time Intervals

The `windowTime` operator groups the values of the source Observable **at regular intervals** and outputs that group as a **new Observable**.
While `bufferTime` returns an array, `windowTime` returns an **Observable&lt;T&gt;**, allowing further operators to be applied to each window.

## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, take } from 'rxjs';

// Emit values every 100ms
const source$ = interval(100);

source$.pipe(
  windowTime(1000), // Create window every 1 second
  take(3),          // Only first 3 windows
  mergeAll()        // Flatten each window
).subscribe(value => {
  console.log('Value:', value);
});

// Output:
// 1st second: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// 2nd second: 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
// 3rd second: 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
```

- A new window (Observable) is created every specified time (1000ms).
- Each window can be processed as an independent Observable.

[🌐 RxJS Official Documentation - `windowTime`](https://rxjs.dev/api/operators/windowTime)

## 💡 Typical Usage Patterns

- **Time-based batch processing**: Data is processed in batches at regular intervals
- **Aggregate real-time data**: Count the number of events per second
- **Performance monitoring**: Collect metrics at regular intervals
- **Analysis of time-series data**: Statistical processing by time frame

## 🔍 Difference from bufferTime

| Operator | Output | Use Case |
|:---|:---|:---|
| `bufferTime` | **Array (T[])** | Process grouped values together |
| `windowTime` | **Observable&lt;T&gt;** | Different stream processing for each time frame |

```ts
import { interval } from 'rxjs';
import { bufferTime, windowTime, take } from 'rxjs';

const source$ = interval(100);

// bufferTime - Output as array
source$.pipe(
  bufferTime(1000),
  take(2)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Output: Buffer (array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// windowTime - Output as Observable
source$.pipe(
  windowTime(1000),
  take(2)
).subscribe(window$ => {
  console.log('Window (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Value:', value);
  });
});
```

## 🧠 Practical Code Example 1: Count Clicks Per Second

This is an example of counting the number of clicks on a button every second.

```ts
import { fromEvent } from 'rxjs';
import { windowTime, map, mergeAll, scan } from 'rxjs';

// Create button
const button = document.createElement('button');
button.textContent = 'Click';
document.body.appendChild(button);

// Output area
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Click event
const clicks$ = fromEvent(button, 'click');

let windowNumber = 0;

clicks$.pipe(
  windowTime(1000), // Create window every 1 second
  map(window$ => {
    ++windowNumber;

    // Count clicks in each window
    return window$.pipe(
      scan(count => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  const timestamp = new Date().toLocaleTimeString();
  output.textContent = `[${timestamp}] Window ${windowNumber}: ${count} clicks`;
});
```

- A new window is created every second.
- The number of clicks in each window is counted in real time.

## 🎯 Practical Code Example 2: Statistical Processing by Time Frame

This example calculates the sum and average of the values for each time frame.

```ts
import { interval } from 'rxjs';
import { windowTime, map, mergeMap, toArray, take } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>Statistical Processing by Time Frame (every 1 second)</h3>';
document.body.appendChild(output);

const table = document.createElement('table');
table.style.borderCollapse = 'collapse';
table.style.marginTop = '10px';
table.innerHTML = `
  <thead>
    <tr style="background: #f0f0f0;">
      <th style="border: 1px solid #ccc; padding: 8px;">Window</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Count</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Sum</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Average</th>
    </tr>
  </thead>
  <tbody id="stats-body"></tbody>
`;
output.appendChild(table);

const source$ = interval(100).pipe(
  map(() => Math.floor(Math.random() * 100)) // Random value
);

let windowNumber = 0;

source$.pipe(
  windowTime(1000), // Every 1 second
  take(5),          // Only 5 windows
  mergeMap(window$ => {
    const current = ++windowNumber;

    // Convert values in each window to array and process statistics
    return window$.pipe(
      toArray(),
      map(values => ({
        window: current,
        count: values.length,
        sum: values.reduce((a, b) => a + b, 0),
        avg: values.length > 0
          ? (values.reduce((a, b) => a + b, 0) / values.length).toFixed(2)
          : 0
      }))
    );
  })
).subscribe(stats => {
  const tbody = document.getElementById('stats-body')!;
  const row = document.createElement('tr');
  row.innerHTML = `
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.window}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.count}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.sum}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.avg}</td>
  `;
  tbody.appendChild(row);
});
```

- Statistics for each window can be calculated separately.
- Different processing can be applied to each window.
- Statistics are displayed visually in a table format.

## 📊 Overlapping Windows (windowCreationInterval)

You can overlap windows by specifying `windowCreationInterval` as the second argument.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray, take, map } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>Overlapping Windows</h3>';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.marginTop = '10px';
document.body.appendChild(output);

const source$ = interval(100);

source$.pipe(
  windowTime(
    2000,  // Window length: 2 seconds
    1000   // Window creation interval: 1 second
  ),
  take(3),
  mergeMap((window$, index) =>
    window$.pipe(
      toArray(),
      map(values => ({ window: index + 1, values }))
    )
  )
).subscribe(result => {
  const div = document.createElement('div');
  div.style.marginTop = '10px';
  div.style.padding = '5px';
  div.style.backgroundColor = '#f5f5f5';
  div.style.borderLeft = '3px solid #4CAF50';

  const title = document.createElement('strong');
  title.textContent = `Window ${result.window}:`;
  div.appendChild(title);

  div.appendChild(document.createElement('br'));

  const values = document.createElement('span');
  values.textContent = `Values: [${result.values.join(', ')}]`;
  div.appendChild(values);

  div.appendChild(document.createElement('br'));

  const info = document.createElement('span');
  info.style.color = '#666';
  info.textContent = `(${result.values.length} values, ${(result.window - 1)} sec ~ ${(result.window + 1)} sec)`;
  div.appendChild(info);

  output.appendChild(div);

  // Chrome workaround: Force rendering
  void output.offsetHeight;
});
```

**How it works:**
- **Window 1**: Values from 0 to 2 seconds `[0, 1, 2, ..., 19]` (20 values)
- **Window 2**: Values from 1 to 3 seconds `[10, 11, 12, ..., 29]` (20 values) ← Values 10-19 overlap with Window 1
- **Window 3**: Values from 2 to 4 seconds `[20, 21, 22, ..., 39]` (20 values) ← Values 20-29 overlap with Window 2

- Creating a new window with an interval (1 second) shorter than the window length (2 seconds) will result in overlap.
- Useful for sliding window implementations.

## 🎯 Practical Example: Real-time Event Monitoring

```ts
import { fromEvent } from 'rxjs';
import { windowTime, mergeMap, toArray, map } from 'rxjs';

// Output area
const output = document.createElement('div');
output.innerHTML = '<h3>Mouse Movement Monitoring (every 5 seconds)</h3>';
document.body.appendChild(output);

const list = document.createElement('ul');
output.appendChild(list);

// Mouse move event
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  windowTime(5000), // Every 5 seconds
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(events => ({
        count: events.length,
        timestamp: new Date().toLocaleTimeString()
      }))
    )
  )
).subscribe(result => {
  const item = document.createElement('li');
  item.textContent = `[${result.timestamp}] Mouse movements: ${result.count} times`;
  list.insertBefore(item, list.firstChild);

  // Display up to 10 items
  while (list.children.length > 10) {
    list.removeChild(list.lastChild!);
  }
});
```

## ⚠️ Notes

### 1. Window Subscription Management

Each window is an independent Observable and must be explicitly subscribed to.

```ts
source$.pipe(
  windowTime(1000)
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
  windowTime(1000),
  mergeAll() // Merge all windows
).subscribe(value => {
  console.log('Value:', value);
});
```

### 2. Memory Management

When running for long periods of time, it is important to unsubscribe properly.

```ts
import { takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

const destroy$ = new Subject<void>();

source$.pipe(
  windowTime(1000),
  mergeAll(),
  takeUntil(destroy$) // Unsubscribe on destroy
).subscribe();

// When component is destroyed, etc.
destroy$.next();
destroy$.complete();
```

### 3. Specify Maximum Value (maxWindowSize)

The third argument allows you to limit the maximum number of values in each window.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray } from 'rxjs';

interval(100).pipe(
  windowTime(
    2000,      // Window length: 2 seconds
    undefined, // Window creation interval: default (no overlap)
    5          // Max value count: up to 5
  ),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Window:', values);
  // Contains maximum of 5 values only
});
```

## 🆚 Comparison of window Operators

| Operator | Timing of Delimiter | Use Case |
|:---|:---|:---|
| `window` | Another Observable emits | Event-driven partitioning |
| `windowTime` | **Fixed time interval** | **Time-based partitioning** |
| `windowCount` | Fixed count | Count-based partitioning |
| `windowToggle` | Start and end Observables | Dynamic start/end control |
| `windowWhen` | Dynamic closing condition | Different end condition per window |

## 📚 Related Operators

- [bufferTime](/en/guide/operators/transformation/bufferTime) - Collect values as array (array version of windowTime)
- [window](/en/guide/operators/transformation/window) - Split window at different Observable timings
- [windowCount](/en/guide/operators/transformation/windowCount) - Count-based window splitting
- [windowToggle](/en/guide/operators/transformation/windowToggle) - Window control with start and end Observables
- [windowWhen](/en/guide/operators/transformation/windowWhen) - Window splitting with dynamic closing conditions

## Summary

The `windowTime` operator is a powerful tool for splitting streams on a time basis and treating each time frame as an independent Observable.

- ✅ Automatically creates windows at regular intervals
- ✅ Different processing can be applied to each window
- ✅ Supports sliding windows (overlap)
- ✅ Ideal for real-time data aggregation and analysis
- ⚠️ Subscription management required
- ⚠️ Be aware of memory management
