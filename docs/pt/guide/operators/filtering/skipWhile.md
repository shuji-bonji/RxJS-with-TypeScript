---
description: The skipWhile operator skips values while the specified condition is met and emits all subsequent values from the point the condition becomes false. It is useful when you want to control a stream with a dynamic start condition.
---

# skipWhile - Skip Values While Condition is Met

The `skipWhile` operator continues to skip values **while the specified condition is met**, and emits **all subsequent values** from the point when the condition becomes `false`.

## 🔰 Basic Syntax and Usage

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0 to 9

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9
```

**Flow of operation**:
1. 0 is emitted → `0 < 5` is `true` → Skip
2. 1 is emitted → `1 < 5` is `true` → Skip
3. 2 is emitted → `2 < 5` is `true` → Skip
4. 3 is emitted → `3 < 5` is `true` → Skip
5. 4 is emitted → `4 < 5` is `true` → Skip
6. 5 is emitted → `5 < 5` is `false` → Start output
7. 6 and after → All output (condition is not re-evaluated)

[🌐 RxJS Official Documentation - `skipWhile`](https://rxjs.dev/api/operators/skipWhile)

## 💡 Typical Usage Patterns

- **Skip initial unnecessary data**: Exclude data during warm-up period
- **Skip until threshold is reached**: Wait until specific conditions are met
- **Skip header rows**: Exclude CSV headers, etc.
- **Skip preparation period**: Wait until system is ready

## 🧠 Practical Code Example 1: Skip Sensor Warm-Up Period

Example of skipping initial data until the sensor stabilizes.

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs';

// Create UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Temperature Sensor Monitoring';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginBottom = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#fff3e0';
status.style.border = '1px solid #FF9800';
status.textContent = '🔄 Sensor preparing... (Measurement starts when temperature >= 20°C)';
container.appendChild(status);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

let isWarmedUp = false;

// Temperature sensor simulation (gradually warming up)
interval(500).pipe(
  take(20),
  map(i => {
    // Start low temperature, gradually increase
    const baseTemp = 15 + i * 0.5;
    const noise = (Math.random() - 0.5) * 2;
    return baseTemp + noise;
  }),
  skipWhile(temp => temp < 20) // Skip below 20°C
).subscribe({
  next: temp => {
    // Update status when first value arrives
    if (!isWarmedUp) {
      isWarmedUp = true;
      status.textContent = '✅ Sensor ready (Measurement started)';
      status.style.backgroundColor = '#e8f5e9';
      status.style.borderColor = '#4CAF50';
    }

    const log = document.createElement('div');
    log.style.padding = '5px';
    log.style.marginBottom = '3px';
    log.style.backgroundColor = temp > 25 ? '#ffebee' : '#f1f8e9';
    log.textContent = `[${new Date().toLocaleTimeString()}] Temperature: ${temp.toFixed(1)}°C`;
    output.insertBefore(log, output.firstChild);

    // Display max 10 items
    while (output.children.length > 10) {
      output.removeChild(output.lastChild!);
    }
  },
  complete: () => {
    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = 'Measurement complete';
    container.appendChild(summary);
  }
});
```

- Data is skipped while the sensor is below 20°C.
- All data is recorded from the point it reaches 20°C or above.

## 🎯 Practical Code Example 2: Event Processing After Ready

Example of skipping events until system initialization completes.

```ts
import { fromEvent, merge, Subject } from 'rxjs';
import { skipWhile, map, tap } from 'rxjs';

// Create UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Event Processing System';
container.appendChild(title);

const initButton = document.createElement('button');
initButton.textContent = 'Complete Initialization';
initButton.style.marginRight = '10px';
container.appendChild(initButton);

const eventButton = document.createElement('button');
eventButton.textContent = 'Fire Event';
container.appendChild(eventButton);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
statusDiv.style.padding = '10px';
statusDiv.style.backgroundColor = '#ffebee';
statusDiv.style.border = '1px solid #f44336';
statusDiv.innerHTML = '<strong>⏸️ System Not Initialized</strong><br>Events will be skipped';
container.appendChild(statusDiv);

const eventLog = document.createElement('div');
eventLog.style.marginTop = '10px';
eventLog.style.border = '1px solid #ccc';
eventLog.style.padding = '10px';
eventLog.style.minHeight = '100px';
container.appendChild(eventLog);

// Initialization state
let isInitialized = false;
const initSubject = new Subject<boolean>();

// Initialization button
fromEvent(initButton, 'click').subscribe(() => {
  if (!isInitialized) {
    isInitialized = true;
    initSubject.next(true);
    statusDiv.style.backgroundColor = '#e8f5e9';
    statusDiv.style.borderColor = '#4CAF50';
    statusDiv.innerHTML = '<strong>✅ System Initialized</strong><br>Processing events';
    initButton.disabled = true;
  }
});

// Event processing (skip until initialized)
let eventCount = 0;
fromEvent(eventButton, 'click').pipe(
  map(() => {
    eventCount++;
    return {
      id: eventCount,
      timestamp: new Date(),
      initialized: isInitialized
    };
  }),
  tap(event => {
    if (!event.initialized) {
      const skipLog = document.createElement('div');
      skipLog.style.padding = '5px';
      skipLog.style.marginBottom = '3px';
      skipLog.style.color = '#999';
      skipLog.textContent = `⏭️ Event #${event.id} skipped (not initialized)`;
      eventLog.insertBefore(skipLog, eventLog.firstChild);
    }
  }),
  skipWhile(event => !event.initialized)
).subscribe(event => {
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.marginBottom = '3px';
  log.style.backgroundColor = '#e8f5e9';
  log.style.border = '1px solid #4CAF50';
  log.innerHTML = `
    <strong>✅ Event #${event.id} Processed</strong>
    [${event.timestamp.toLocaleTimeString()}]
  `;
  eventLog.insertBefore(log, eventLog.firstChild);

  // Display max 10 items
  while (eventLog.children.length > 10) {
    eventLog.removeChild(eventLog.lastChild!);
  }
});
```

- All events are skipped until the system is initialized.
- After initialization completes, all events are processed.

## 🆚 Comparison with Similar Operators

### skipWhile vs takeWhile vs skip vs filter

```ts
import { range } from 'rxjs';
import { skipWhile, takeWhile, skip, filter } from 'rxjs';

const numbers$ = range(0, 10); // 0 to 9

// skipWhile: Skip while condition is met, then output all
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9

// takeWhile: Take only while condition is met
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// skip: Skip first N values
numbers$.pipe(
  skip(5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9

// filter: Only pass values matching condition (evaluated for all)
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9
```

| Operator | Behavior | Re-evaluate Condition | Completion Timing |
|:---|:---|:---|:---|
| `skipWhile(predicate)` | Skip while condition is met | No (ends once false) | When source stream completes |
| `takeWhile(predicate)` | Take while condition is met | Every time | When condition becomes false |
| `skip(n)` | Skip first n values | None (count-based) | When source stream completes |
| `filter(predicate)` | Only pass matching values | **Every time** | When source stream completes |

**Visual differences**:

```
Input: 0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0

skipWhile(n => n < 5):
[0,1,2,3,4 skipped] | 5, 4, 3, 2, 1, 0
                      ^All output after condition becomes false

filter(n => n >= 5):
[0,1,2,3,4 excluded] 5 [4,3,2,1,0 excluded]
                     ^Only output matching values (evaluated each time)

takeWhile(n => n < 5):
0, 1, 2, 3, 4 | [ignore everything after 5 and complete]
```

## ⚠️ Important Notes

### 1. Condition is Not Re-evaluated Once False

This is the biggest difference from `filter`.

```ts
import { from } from 'rxjs';
import { skipWhile, filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 4, 3, 2, 1]);

// skipWhile: Once condition becomes false, output all subsequent values
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(val => console.log('skipWhile:', val));
// Output: skipWhile: 5, 4, 3, 2, 1 (all after 5)

// filter: Evaluate condition every time
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(val => console.log('filter:', val));
// Output: filter: 5 (only 5)
```

### 2. If Condition is False From the Start

If the condition is `false` from the beginning, all values are output.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(5, 5).pipe( // 5 to 9
  skipWhile(n => n < 3) // Condition is false from start
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9 (all output)
```

### 3. If All Values Satisfy Condition

If all values satisfy the condition, nothing is output.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(0, 5).pipe( // 0 to 4
  skipWhile(n => n < 10) // All values satisfy condition
).subscribe({
  next: console.log,
  complete: () => console.log('Complete (nothing output)')
});
// Output: Complete (nothing output)
```

### 4. TypeScript Types

`skipWhile` does not change the type.

```ts
import { Observable, from } from 'rxjs';
import { skipWhile } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

const users$: Observable<User> = from([
  { id: 1, name: 'Alice', isActive: false },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true },
  { id: 4, name: 'Dave', isActive: true }
]);

// Type remains Observable<User>
const activeUsers$: Observable<User> = users$.pipe(
  skipWhile(user => !user.isActive)
);

activeUsers$.subscribe(user => {
  console.log(`${user.name} (ID: ${user.id})`);
});
// Output: Charlie (ID: 3), Dave (ID: 4)
```

## 💡 Practical Combination Patterns

### Pattern 1: Skip Header Row

Skip CSV header rows

```ts
import { from } from 'rxjs';
import { skipWhile, map } from 'rxjs';

const csvLines$ = from([
  'Name,Age,City',     // Header row
  'Alice,25,Tokyo',
  'Bob,30,Osaka',
  'Charlie,35,Kyoto'
]);

let isFirstLine = true;

csvLines$.pipe(
  skipWhile(() => {
    if (isFirstLine) {
      isFirstLine = false;
      return true; // Skip first row (header)
    }
    return false;
  }),
  map(line => {
    const [name, age, city] = line.split(',');
    return { name, age: Number(age), city };
  })
).subscribe(console.log);
// Output:
// { name: 'Alice', age: 25, city: 'Tokyo' }
// { name: 'Bob', age: 30, city: 'Osaka' }
// { name: 'Charlie', age: 35, city: 'Kyoto' }
```

### Pattern 2: Timestamp-Based Filtering

Process only data after specific time

```ts
import { from } from 'rxjs';
import { skipWhile } from 'rxjs';

interface LogEntry {
  timestamp: Date;
  message: string;
}

const startTime = new Date('2025-01-01T12:00:00');

const logs$ = from([
  { timestamp: new Date('2025-01-01T10:00:00'), message: 'Log 1' },
  { timestamp: new Date('2025-01-01T11:00:00'), message: 'Log 2' },
  { timestamp: new Date('2025-01-01T12:00:00'), message: 'Log 3' },
  { timestamp: new Date('2025-01-01T13:00:00'), message: 'Log 4' }
] as LogEntry[]);

logs$.pipe(
  skipWhile(log => log.timestamp < startTime)
).subscribe(log => {
  console.log(`[${log.timestamp.toISOString()}] ${log.message}`);
});
// Output:
// [2025-01-01T12:00:00.000Z] Log 3
// [2025-01-01T13:00:00.000Z] Log 4
```

### Pattern 3: State-Based Skip

Skip until system is ready

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs';

interface SystemState {
  tick: number;
  isReady: boolean;
  data: number;
}

// System state simulation
interval(500).pipe(
  take(10),
  map(i => ({
    tick: i,
    isReady: i >= 3, // Ready after 3 seconds
    data: Math.floor(Math.random() * 100)
  } as SystemState)),
  skipWhile(state => !state.isReady)
).subscribe(state => {
  console.log(`Tick ${state.tick}: data=${state.data}`);
});
// Output: Only data from Tick 3 onwards
```

## 📚 Related Operators

- **[takeWhile](/pt/guide/operators/filtering/takeWhile)** - Take values only while condition is met
- **[skip](/pt/guide/operators/filtering/skip)** - Skip first N values
- **[skipLast](/pt/guide/operators/filtering/skipLast)** - Skip last N values
- **[skipUntil](/pt/guide/operators/filtering/skipUntil)** - Skip until another Observable fires
- **[filter](/pt/guide/operators/filtering/filter)** - Only pass values matching condition

## Summary

The `skipWhile` operator skips values while a condition is met and emits all subsequent values from the point the condition becomes false.

- ✅ Ideal for skipping initial unnecessary data
- ✅ Condition is not re-evaluated once it becomes false
- ✅ Useful for skipping warm-up or preparation periods
- ✅ Can be used to skip header rows
- ⚠️ Unlike `filter`, condition is evaluated only once
- ⚠️ If all values satisfy condition, nothing is output
- ⚠️ Continues until source stream completes
