---
description: "takeLast is an RxJS filtering operator that outputs only the last N values when an Observable stream is completed. It is ideal for situations where only the last value from the entire stream is needed, such as getting the latest count in a log, displaying the top N values on a leaderboard, or a final data summary upon completion. It cannot be used with infinite streams as it is held in a buffer until completion."
---

# takeLast - get the last N values

The `takeLast` operator outputs only the last N values at the time the stream is **completed**. It keeps the values in a buffer until the stream completes, and outputs them together after completion.

## 🔰 Basic Syntax and Usage

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0from9to

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9
```

**Flow of operation**: 1.
Stream issues 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 2.
Internally hold last 3 in buffer 3.
3. stream completes 4. buffer value 7, 8, 9
4. output buffer values 7, 8, 9 in sequence

[🌐 RxJS Official Documentation - `takeLast`](https://rxjs.dev/api/operators/takeLast)

## 🆚 Contrast with take

`take` and `takeLast` have contrasting behavior.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0from9to

// take: FirstNGet the first piece
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Output: 0, 1, 2(output immediately)

// takeLast: Get the lastNGet the first piece
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9(wait for completion before outputting)
```

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0from9to

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9
```

## 💡 Typical utilization pattern

1. **Get the latest N log entries**.


```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'warn' as const, message: 'Slow query detected' },
     { timestamp: 4, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 5, level: 'info' as const, message: 'Retry successful' },
   ] as LogEntry[]);

   // Get the latest3Retrieve logs of
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // Output:
   // [warn] Slow query detected
   // [error] Connection failed
   // [info] Retry successful
   ```

2. **Top of leaderboardNRetrieve the top**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface Score {
     player: string;
     score: number;
   }

   const scores$ = from([
     { player: 'Alice', score: 100 },
     { player: 'Bob', score: 150 },
     { player: 'Charlie', score: 200 },
     { player: 'Dave', score: 180 },
     { player: 'Eve', score: 220 }
   ] as Score[]).pipe(
     // Assume sorted by score
   );

   // Get the top3Retrieve the
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Output: Charlie: 200, Dave: 180, Eve: 220
   ```

3. **Final summary after data processing is completeNSummary of cases**
   ```ts
   import { interval } from 'rxjs';
   import { take, map, takeLast } from 'rxjs';

   // Simulation of sensor data
   const sensorData$ = interval(100).pipe(
     take(20),
     map(i => ({
       id: i,
       temperature: 20 + Math.random() * 10
     }))
   );

   // Get the last5Calculate average temperature of
   sensorData$.pipe(
     takeLast(5)
   ).subscribe({
     next: data => {
       console.log(`data${data.id}: ${data.temperature.toFixed(1)}°C`);
     },
     complete: () => {
       console.log('Latest5Completed data acquisition for the case');
     }
   });
   ```

## 🧠 Practical code example (input history)

This is an example of displaying the latest3The following is an example of displaying the latest values entered by the user.

```

ts
import { fromEvent, Subject } from 'rxjs';
import { takeLast } from 'rxjs';

// Create UI elements
const container = document.createElement('div');
document.body.appendChild(container);

const input = document.createElement('input');
input.placeholder = 'Enter a value and Enter';
container.appendChild(input);

const submitButton = document.createElement('button');
submitButton.textContent = 'Show history (latest 3)';
container.appendChild(submitButton);

const historyDisplay = document.createElement('div');
historyDisplay.style.marginTop = '10px';
container.appendChild(historyDisplay);

// Subject to hold input values
const inputs$ = new Subject<string>();

// **IMPORTANT**: set takeLast subscription first
inputs$.pipe(
  takeLast(3)
).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `- ${value}`;
    historyDisplay.appendChild(item);
  },
  complete: () => {
    const note = document.createElement('div');
    note.style.marginTop = '5px';
    note.style.color = 'gray';
    note.textContent = '(You can type again if you reload the page)';
    historyDisplay.appendChild(note);

    // disable input fields and buttons
    input.disabled = true;
    submitButton.disabled = true;
  }
});

// Add input with Enter key
fromEvent\<KeyboardEvent>(input, 'keydown').subscribe(event => {
  if (event.key === 'Enter' && input.value.trim()) {
    inputs$.next(input.value);
    console.log(`Add: ${input.value}`);
    input.value = '';
  }
});

// Complete with button click and display history
fromEvent(submitButton, 'click').subscribe(() => {
  historyDisplay.innerHTML = '<strong>History (latest 3):</strong><br>';
  inputs$.complete(); // stream completed → takeLast fires
});

```

> [!IMPORTANT]
> **Important point**:
> - `takeLast(3)` Subscribe to the**must be set up ahead of time**must be set up first
> - When the button is clicked `complete()` the last of the values received up to that point will be output.3is output, the last value received up to that point is output.
> - `complete()` After calling**After calling**to `subscribe` does not cause any values to flow.

## ⚠️ An important note

> [!WARNING]
> `takeLast` is to wait until the stream**waits until the stream completes**It does not work with infinite streams because it waits until the stream completes. Also, the`takeLast(n)` is large, it will consume a lot of memory.nis large, it consumes a lot of memory.

### 1. Cannot be used with infinite streams

`takeLast` does not work with infinite streams because it waits until the stream completes.

```

ts
import { interval } from 'rxjs';
import { takeLast } from 'rxjs';

// ❌ Bad example: using takeLast with infinite streams
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);
// nothing is output (because the stream never completes)

```

**Solution**: `take` Make it a finite stream by combining it with

```

ts
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs';

// ✅ Good example: finite stream, then use takeLast
interval(1000).pipe(
  take(10), // first 10 complete
  takeLast(3) // take the last 3 of them
).subscribe(console.log);
// output: 7, 8, 9

```

### 2. Pay attention to memory usage

`takeLast(n)` does not work with infinite streams because it waits until the lastnto keep the last one in the buffer,nis large, it consumes more memory.

```

ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

// ⚠️ Note: keep large amount of data in buffer
range(0, 1000000).pipe(
  takeLast(100000) // 100,000 records held in memory
).subscribe(console.log);

```

## 🎯 last Difference between

```

ts
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs';

const numbers$ = range(0, 10);

// last: take only the last one
numbers$.pipe(
  last()
).subscribe(console.log);
// output: 9

// takeLast(1): last one (output as a single value, not an array)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);
// Output: 9

// takeLast(3): last 3
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9

```

| operator | Number of acquisitions | Condition specification | Use case |
|---|---|---|---|
| `last()` | 1Number of | Possible | Get the last1Pieces or the last piece that satisfies the condition1Number of |
| `takeLast(n)` | nNumber of | Impossible | Get the lastnSimply get the last one that satisfies the condition |

## 📋 Type-Safe Usage

TypeScript The following is an example of a type-safe implementation that utilizes generics in

```

ts
import { Observable, from } from 'rxjs';
import { takeLast } from 'rxjs';

interface Transaction {
  id: string;
  amount: number;
  timestamp: Date;
  status: 'pending' | 'completed' | 'failed'; }
}

function getRecentTransactions(
  transactions$: Observable\<Transaction>, count: number
  count: number
): Observable\<Transaction> {
  return transactions$.pipe(
    takeLast(count)
  ); }
}

// Example of use
const transactions$ = from([
  { id: '1', amount: 100, timestamp: new Date('2025-01-01'), status: 'completed' as const }
  { id: '2', amount: 200, timestamp: new Date('2025-01-02'), status: 'completed' as const }
  { id: '3', amount: 150, timestamp: new Date('2025-01-03'), status: 'pending' as const }
  { id: '4', amount: 300, timestamp: new Date('2025-01-04'), status: 'completed' as const }
  { id: '5', amount: 250, timestamp: new Date('2025-01-05'), status: 'failed' as const }
] as Transaction[]);

// Get the last 3 transactions
getRecentTransactions(transactions$, 3).subscribe(tx => {
  console.log(`${tx.id}: ${tx.amount} yen (${tx.status})`);
});
// Output: ${tx.id}: ${tx.status}
// 3: 150 yen (pending)
// 4: 300 yen (completed)
// 5: 250 yen (failed)

```

## 🔄 skip and takeLast Combination of

can be used to exclude values in the middle and retrieve only the lastNOnly the last one can be retrieved, excluding values in the middle part.

```

```ts
import { range } from 'rxjs';
import { skip, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 to 9

// skip the first 5 and take the last 3
numbers$.pipe(
  skip(5), // skip 0, 1, 2, 3, 4
  takeLast(3) // take the last 3 from the remaining 5, 6, 7, 8, 9
).subscribe(console.log);
// Output: 7, 8, 9
```

## 🎓 Summary

### When should takeLast be used?
- ✅ If you need the last N data in a stream
- ✅ If you want to get the last N logs or transactions
- ✅ If the stream is guaranteed to complete
- ✅ If you want to display a summary or top N records of data

### When should you use take?
- ✅ If you need the first N data in the stream
- ✅ If you want to get results immediately
- ✅ If you want to get a portion of an infinite stream

### Notes.
- ⚠️ Cannot be used with infinite streams (because it never completes)
- ⚠️ Large n in `takeLast(n)` consumes memory
- ⚠️ Output is compiled after completion (not immediately)
- ⚠️ Often needs to be combined with `take(n)` to make a finite stream

## 🚀 Next step.

- **[take](./take)** - learn how to get the first n values.
- **[last](./last)** - learn how to get the last 1 value
- **[skip](./skip)** - learn how to skip the first N values
- **[filter](./filter)** - learn how to filter based on conditions
- **[filtering operator practical-use-cases](./practical-use-cases)** - learn how to use real use cases
