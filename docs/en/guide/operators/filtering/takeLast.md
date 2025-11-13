---
description: takeLast is an RxJS filtering operator that outputs only the last N values when the Observable stream completes. It is ideal for scenarios where only the last values from the entire stream are needed, such as getting the latest log entries, displaying top N items on a leaderboard, and final data summaries upon completion. It cannot be used with infinite streams because it holds values in a buffer until completion.
---

# takeLast - Get Last N Values

The `takeLast` operator outputs only the last N values when the stream **completes**. It holds values in a buffer until the stream completes, then outputs them all at once.


## 🔰 Basic Syntax and Usage

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 to 9

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9
```

**Flow of operation**:
1. Stream emits 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
2. Internally holds last 3 values in buffer
3. Stream completes
4. Outputs buffer values 7, 8, 9 in order

[🌐 RxJS Official Documentation - `takeLast`](https://rxjs.dev/api/operators/takeLast)


## 🆚 Contrast with take

`take` and `takeLast` have contrasting behavior.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 to 9

// take: Get first N values
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Output: 0, 1, 2 (output immediately)

// takeLast: Get last N values
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9 (output after waiting for completion)
```

| Operator | Get Position | Output Timing | Behavior Before Completion |
|---|---|---|---|
| `take(n)` | First n values | Output immediately | Auto-complete after n values |
| `takeLast(n)` | Last n values | Output all together after completion | Hold in buffer |


## 💡 Typical Usage Patterns

1. **Get Latest N Log Entries**
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

   // Get latest 3 log entries
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

2. **Get Top N on Leaderboard**
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
   ] as Score[]);

   // Get top 3
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Output: Charlie: 200, Dave: 180, Eve: 220
   ```


## ⚠️ Important Notes

> [!WARNING]
> `takeLast` **waits until the stream completes**, so it does not work with infinite streams. Also, if n in `takeLast(n)` is large, it consumes a lot of memory.

### 1. Cannot Use with Infinite Streams

`takeLast` waits for the stream to complete, so it does not work with infinite streams.

```ts
import { interval } from 'rxjs';
import { takeLast } from 'rxjs';

// ❌ Bad example: Use takeLast with infinite stream
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);
// Nothing output (because stream never completes)
```

**Solution**: Make it a finite stream by combining with `take`

```ts
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs';

// ✅ Good example: Use takeLast after making it finite stream
interval(1000).pipe(
  take(10),      // Complete with first 10 values
  takeLast(3)    // Get last 3 from them
).subscribe(console.log);
// Output: 7, 8, 9
```

### 2. Be Mindful of Memory Usage

`takeLast(n)` holds the last n values in a buffer, so large n consumes memory.

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

// ⚠️ Caution: Hold large amount of data in buffer
range(0, 1000000).pipe(
  takeLast(100000) // Hold 100,000 items in memory
).subscribe(console.log);
```


## 🎯 Difference from last

```ts
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs';

const numbers$ = range(0, 10);

// last: Only last 1 value
numbers$.pipe(
  last()
).subscribe(console.log);
// Output: 9

// takeLast(1): Last 1 value (output as single value, not array)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);
// Output: 9

// takeLast(3): Last 3 values
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9
```

| Operator | Get Count | Condition Specification | Use Case |
|---|---|---|---|
| `last()` | 1 value | Possible | Last 1 value or last 1 value meeting condition |
| `takeLast(n)` | n values | Not possible | Simply get last n values |


## 🎓 Summary

### When to Use takeLast
- ✅ When you need the last N data from the stream
- ✅ When you want to get latest N entries of logs or transactions
- ✅ When stream completion is guaranteed
- ✅ When you want to display data summary or top N items

### When to Use take
- ✅ When you need the first N data from the stream
- ✅ When you want to get results immediately
- ✅ When you want to get a portion from infinite stream

### Notes
- ⚠️ Cannot use with infinite streams (doesn't complete)
- ⚠️ Large n in `takeLast(n)` consumes memory
- ⚠️ Output is done all together after completion (not output immediately)
- ⚠️ Often need to combine with `take(n)` to make finite stream


## 🚀 Next Steps

- **[take](/en/guide/operators/filtering/take)** - Learn how to get first N values
- **[last](/en/guide/operators/filtering/last)** - Learn how to get last 1 value
- **[skip](/en/guide/operators/filtering/skip)** - Learn how to skip first N values
- **[filter](/en/guide/operators/filtering/filter)** - Learn how to filter based on conditions
- **[Filtering Operator Practical Examples](/en/guide/operators/filtering/practical-use-cases)** - Learn real use cases
