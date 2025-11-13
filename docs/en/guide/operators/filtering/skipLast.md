---
description: "skipLast operator skips the last N values in Observable streams and outputs only earlier values: Perfect for excluding unconfirmed pending data"
---

# skipLast - Skip Last N Values

The `skipLast` operator **skips the last N values** emitted from the source Observable and outputs only the values before them. It keeps the last N values in a buffer until the stream completes and outputs the rest.

## 🔰 Basic Syntax and Usage

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 to 9

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 are skipped)
```

**Flow of operation**:
1. Stream emits 0, 1, 2, ...
2. Holds last 3 values (7, 8, 9) in buffer
3. Outputs values exceeding buffer size (0~6)
4. When stream completes, buffer values (7, 8, 9) are discarded without output

[🌐 RxJS Official Documentation - `skipLast`](https://rxjs.dev/api/operators/skipLast)

## 💡 Typical Usage Patterns

- **Exclude latest data**: Exclude unconfirmed latest data
- **Batch processing**: Exclude pending data before processing completes
- **Data validation**: When validation is required on subsequent values
- **Delayed finalized data processing**: When the last N items are not finalized

## 🧠 Practical Code Example 1: Data Processing Pipeline

Example of skipping the last pending data in data processing.

```ts
import { from, interval } from 'rxjs';
import { skipLast, map, take, concatMap, delay } from 'rxjs';

// Create UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Data Processing Pipeline';
container.appendChild(title);

const description = document.createElement('div');
description.style.marginBottom = '10px';
description.style.color = '#666';
description.textContent = 'Skip last 2 items (pending data) before processing';
container.appendChild(description);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

interface DataPoint {
  id: number;
  value: number;
  status: 'processing' | 'confirmed' | 'skipped';
}

// Data stream (10 items)
const data: DataPoint[] = Array.from({ length: 10 }, (_, i) => ({
  id: i,
  value: Math.floor(Math.random() * 100),
  status: 'processing' as const
}));

// Emit data every 0.5 seconds
from(data).pipe(
  concatMap(item => interval(500).pipe(
    take(1),
    map(() => item)
  )),
  skipLast(2) // Skip last 2 items
).subscribe({
  next: item => {
    const div = document.createElement('div');
    div.style.padding = '5px';
    div.style.marginBottom = '5px';
    div.style.backgroundColor = '#e8f5e9';
    div.style.border = '1px solid #4CAF50';
    div.innerHTML = `
      <strong>✅ Confirmed</strong>
      ID: ${item.id} |
      Value: ${item.value}
    `;
    output.appendChild(div);
  },
  complete: () => {
    // Display skipped items
    const skippedItems = data.slice(-2);
    skippedItems.forEach(item => {
      const div = document.createElement('div');
      div.style.padding = '5px';
      div.style.marginBottom = '5px';
      div.style.backgroundColor = '#ffebee';
      div.style.border = '1px solid #f44336';
      div.innerHTML = `
        <strong>⏭️ Skipped</strong>
        ID: ${item.id} |
        Value: ${item.value} |
        (Pending data)
      `;
      output.appendChild(div);
    });

    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = `Processing complete: ${data.length - 2} confirmed, 2 skipped`;
    output.appendChild(summary);
  }
});
```

- Data is processed sequentially, but the last 2 items are treated as pending and skipped.
- After completion, the skipped items are also displayed.

## 🎯 Practical Code Example 2: Log Filtering

Example of skipping the latest unconfirmed logs from a log stream.

```ts
import { interval } from 'rxjs';
import { skipLast, map, take } from 'rxjs';

// Create UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Log Monitoring';
container.appendChild(title);

const info = document.createElement('div');
info.style.marginBottom = '10px';
info.textContent = 'Latest 3 logs are skipped as pending confirmation';
info.style.color = '#666';
container.appendChild(info);

const confirmedLogs = document.createElement('div');
confirmedLogs.innerHTML = '<strong>📋 Confirmed Logs:</strong>';
confirmedLogs.style.marginBottom = '10px';
container.appendChild(confirmedLogs);

const confirmedList = document.createElement('div');
confirmedList.style.border = '1px solid #4CAF50';
confirmedList.style.padding = '10px';
confirmedList.style.backgroundColor = '#f1f8e9';
confirmedList.style.minHeight = '100px';
container.appendChild(confirmedList);

const pendingLogs = document.createElement('div');
pendingLogs.innerHTML = '<strong>⏳ Pending Logs (Skipped):</strong>';
pendingLogs.style.marginTop = '10px';
pendingLogs.style.marginBottom = '10px';
container.appendChild(pendingLogs);

const pendingList = document.createElement('div');
pendingList.style.border = '1px solid #FF9800';
pendingList.style.padding = '10px';
pendingList.style.backgroundColor = '#fff3e0';
pendingList.style.minHeight = '60px';
container.appendChild(pendingList);

interface LogEntry {
  id: number;
  timestamp: Date;
  level: 'info' | 'warn' | 'error';
  message: string;
}

// Generate logs (12 total, every 1 second)
const logs$ = interval(1000).pipe(
  take(12),
  map(i => {
    const levels: ('info' | 'warn' | 'error')[] = ['info', 'warn', 'error'];
    const messages = [
      'User login',
      'Data fetch started',
      'Cache updated',
      'Connection error',
      'Retry executed',
      'Data processing complete'
    ];
    return {
      id: i,
      timestamp: new Date(),
      level: levels[Math.floor(Math.random() * levels.length)],
      message: messages[Math.floor(Math.random() * messages.length)]
    } as LogEntry;
  })
);

const allLogs: LogEntry[] = [];

// Record all logs (for verification)
logs$.subscribe(log => {
  allLogs.push(log);
});

// Display confirmed logs, skipping last 3
logs$.pipe(
  skipLast(3)
).subscribe({
  next: log => {
    const logDiv = document.createElement('div');
    logDiv.style.padding = '3px';
    logDiv.style.marginBottom = '3px';
    const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
    logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
    confirmedList.appendChild(logDiv);
  },
  complete: () => {
    // Display last 3 logs (skipped logs)
    const skippedLogs = allLogs.slice(-3);
    skippedLogs.forEach(log => {
      const logDiv = document.createElement('div');
      logDiv.style.padding = '3px';
      logDiv.style.marginBottom = '3px';
      const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
      logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
      pendingList.appendChild(logDiv);
    });
  }
});
```

- Logs are added sequentially, but the latest 3 are skipped as pending confirmation.
- After completion, the skipped logs are also displayed.

## 🆚 Comparison with Similar Operators

### skipLast vs takeLast vs skip

```ts
import { range } from 'rxjs';
import { skipLast, takeLast, skip } from 'rxjs';

const numbers$ = range(0, 10); // 0 to 9

// skipLast: Skip last N values
numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6

// takeLast: Take only last N values
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9

// skip: Skip first N values
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, 7, 8, 9
```

| Operator | Skip Position | Output Timing | Requires Completion |
|:---|:---|:---|:---|
| `skipLast(n)` | Last n values | Output when buffer is exceeded | Required |
| `takeLast(n)` | All except last n | Output all together after completion | Required |
| `skip(n)` | First n values | Output immediately | Not required |

**Visual differences**:

```
Input: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

skipLast(3): 0, 1, 2, 3, 4, 5, 6 | [7, 8, 9 skipped]
                                   ^Last 3

takeLast(3): [0~6 skipped] | 7, 8, 9
                             ^Last 3 only

skip(3): [0, 1, 2 skipped] | 3, 4, 5, 6, 7, 8, 9
          ^First 3
```

## ⚠️ Important Notes

### 1. Behavior with Infinite Streams

Because `skipLast` cannot identify the last N values until completion, it does not work as intended with infinite streams.

```ts
import { interval } from 'rxjs';
import { skipLast } from 'rxjs';

// ❌ Bad example: Use skipLast with infinite stream
interval(1000).pipe(
  skipLast(3)
).subscribe(console.log);
// Output: 0 (after 3 seconds), 1 (after 4 seconds), 2 (after 5 seconds), ...
// All values continue to output infinitely with N delay
// Last 3 remain in buffer forever and never output
```

With infinite streams, since the last N values are not determined, all values continue to be output with a delay of N. Since there is no true "last N", the original purpose of `skipLast` cannot be achieved.

**Solution**: Make it a finite stream with `take`

```ts
import { interval } from 'rxjs';
import { take, skipLast } from 'rxjs';

// ✅ Good example: Make finite stream before using skipLast
interval(1000).pipe(
  take(10),      // Complete with first 10 values
  skipLast(3)    // Skip last 3
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 are skipped)
```

### 2. Be Mindful of Buffer Size

`skipLast(n)` always keeps n values in the buffer.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

// ⚠️ Caution: Hold 1000 items in buffer
range(0, 1000000).pipe(
  skipLast(1000)
).subscribe(console.log);
```

### 3. Output Delay

`skipLast(n)` does not output anything until n buffers are filled.

```ts
import { interval } from 'rxjs';
import { take, skipLast, tap } from 'rxjs';

interval(1000).pipe(
  take(5),
  tap(val => console.log('Input:', val)),
  skipLast(2)
).subscribe(val => console.log('Output:', val));
// Input: 0
// Input: 1
// Input: 2
// Output: 0  ← Output starts after buffer fills with 2
// Input: 3
// Output: 1
// Input: 4
// Output: 2
// Complete (3, 4 are skipped)
```

### 4. skipLast(0) Behavior

`skipLast(0)` skips nothing.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

range(0, 5).pipe(
  skipLast(0)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4 (all output)
```

## 💡 Practical Combination Patterns

### Pattern 1: Get Only Middle Section

Skip both beginning and end to get only the middle section

```ts
import { range } from 'rxjs';
import { skip, skipLast } from 'rxjs';

range(0, 10).pipe(
  skip(2),      // Skip first 2
  skipLast(2)   // Skip last 2
).subscribe(console.log);
// Output: 2, 3, 4, 5, 6, 7
```

### Pattern 2: Data Validation

When validation is required on subsequent values

```ts
import { from } from 'rxjs';
import { skipLast, map } from 'rxjs';

interface Transaction {
  id: number;
  amount: number;
  pending: boolean;
}

const transactions$ = from([
  { id: 1, amount: 100, pending: false },
  { id: 2, amount: 200, pending: false },
  { id: 3, amount: 150, pending: false },
  { id: 4, amount: 300, pending: true },  // Pending
  { id: 5, amount: 250, pending: true }   // Pending
]);

// Skip pending transactions (last 2)
transactions$.pipe(
  skipLast(2)
).subscribe(tx => {
  console.log(`Confirmed: ID ${tx.id}, Amount ${tx.amount}`);
});
// Output:
// Confirmed: ID 1, Amount 100
// Confirmed: ID 2, Amount 200
// Confirmed: ID 3, Amount 150
```

### Pattern 3: Window Processing

Window processing with data excluding the latest N items

```ts
import { range } from 'rxjs';
import { skipLast, bufferCount } from 'rxjs';

range(0, 10).pipe(
  skipLast(2),      // Skip last 2
  bufferCount(3, 1) // Windows of 3
).subscribe(window => {
  console.log('Window:', window);
});
// Output:
// Window: [0, 1, 2]
// Window: [1, 2, 3]
// Window: [2, 3, 4]
// ...
```

## 📚 Related Operators

- **[skip](/en/guide/operators/filtering/skip)** - Skip first N values
- **[takeLast](/en/guide/operators/filtering/takeLast)** - Take only last N values
- **[take](/en/guide/operators/filtering/take)** - Take only first N values
- **[skipUntil](/en/guide/operators/filtering/skipUntil)** - Skip until another Observable fires
- **[skipWhile](/en/guide/operators/filtering/skipWhile)** - Skip while condition is met

## Summary

The `skipLast` operator skips the last N values in the stream.

- ✅ Ideal when the last N data are not needed
- ✅ Useful for excluding unconfirmed data
- ✅ Buffer size is only N (memory efficient)
- ✅ Requires stream completion
- ⚠️ Cannot use with infinite streams
- ⚠️ No output until buffer fills with N values
- ⚠️ Often needs to be combined with `take` to make finite stream
