---
description: "takeWhile is an RxJS filtering operator that continues to take values while the specified condition is met and completes the stream when the condition becomes false. It is ideal for situations where you want to control a stream with dynamic conditions, such as data acquisition up to a threshold, priority-based processing, paging, etc. The inclusive option allows you to include values for which the condition becomes false."
---

# takeWhile - Take Values While Condition is Met

The `takeWhile` operator continues to take values **while the specified condition is met**, and completes the stream when the condition becomes `false`.


## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { takeWhile } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('Complete')
});
// Output: 0, 1, 2, 3, 4, Complete
```

**Flow of operation**:
1. 0 is emitted → `0 < 5` is `true` → Output
2. 1 is emitted → `1 < 5` is `true` → Output
3. 2 is emitted → `2 < 5` is `true` → Output
4. 3 is emitted → `3 < 5` is `true` → Output
5. 4 is emitted → `4 < 5` is `true` → Output
6. 5 is emitted → `5 < 5` is `false` → Complete (5 is not output)

[🌐 RxJS Official Documentation - `takeWhile`](https://rxjs.dev/api/operators/takeWhile)


## 🆚 Contrast with take

`take` and `takeWhile` have different acquisition conditions.

```ts
import { interval } from 'rxjs';
import { take, takeWhile } from 'rxjs';

const source$ = interval(1000);

// take: Control by count
source$.pipe(
  take(5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// takeWhile: Control by condition
source$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4
```

| Operator | Control Method | Completion Condition | Last Value |
|---|---|---|---|
| `take(n)` | Count | After n values | Includes nth value |
| `takeWhile(predicate)` | Condition function | When condition becomes `false` | Does not include value that became `false`* |

\* By default, the value that became `false` is not output, but can be included with the `inclusive: true` option


## 🎯 inclusive Option

If you want to include the value for which the condition became `false`, specify `inclusive: true`.

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

const numbers$ = range(0, 10);

// Default (inclusive: false)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// inclusive: true
numbers$.pipe(
  takeWhile(n => n < 5, true)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5 (includes 5 which made condition false)
```


## 💡 Typical Usage Patterns

1. **Data Acquisition Up to Threshold**
   ```ts
   import { interval } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   // Temperature sensor simulation
   const temperature$ = interval(100).pipe(
     map(() => 20 + Math.random() * 15)
   );

   // Record only while below 30 degrees
   temperature$.pipe(
     takeWhile(temp => temp < 30)
   ).subscribe({
     next: temp => console.log(`Temperature: ${temp.toFixed(1)}°C`),
     complete: () => console.log('Warning: Temperature exceeded 30 degrees!')
   });
   ```

2. **Conditional Array Processing**
   ```ts
   import { from } from 'rxjs';
   import { takeWhile } from 'rxjs';

   interface Task {
     id: number;
     priority: 'high' | 'medium' | 'low';
     completed: boolean;
   }

   const tasks$ = from([
     { id: 1, priority: 'high' as const, completed: false },
     { id: 2, priority: 'high' as const, completed: false },
     { id: 3, priority: 'medium' as const, completed: false },
     { id: 4, priority: 'low' as const, completed: false },
   ] as Task[]);

   // Process only while priority is high
   tasks$.pipe(
     takeWhile(task => task.priority === 'high')
   ).subscribe(task => {
     console.log(`Processing task ${task.id}`);
   });
   // Output: Processing task 1, Processing task 2
   ```

3. **Paging Processing**
   ```ts
   import { range } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   interface Page {
     pageNumber: number;
     hasMore: boolean;
   }

   const pages$ = range(1, 10).pipe(
     map(pageNum => ({
       pageNumber: pageNum,
       hasMore: pageNum < 5
     } as Page))
   );

   // Load pages only while hasMore is true
   pages$.pipe(
     takeWhile(page => page.hasMore, true) // inclusive: true
   ).subscribe(page => {
     console.log(`Loading page ${page.pageNumber}`);
   });
   // Output: Loading page 1~5
   ```


## 🧠 Practical Code Example (Count-Up Limit)

Example of continuing count-up until a specific condition is reached.

```ts
import { fromEvent, interval } from 'rxjs';
import { takeWhile, scan, switchMap } from 'rxjs';

// Create UI elements
const container = document.createElement('div');
document.body.appendChild(container);

const startButton = document.createElement('button');
startButton.textContent = 'Start Count';
container.appendChild(startButton);

const counter = document.createElement('div');
counter.style.fontSize = '24px';
counter.style.marginTop = '10px';
counter.textContent = 'Count: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = 'Counts while below 10';
container.appendChild(message);

// Start counting on button click
fromEvent(startButton, 'click').pipe(
  switchMap(() =>
    interval(500).pipe(
      scan(count => count + 1, 0),
      takeWhile(count => count < 10)
    )
  )
).subscribe({
  next: (count) => {
    counter.textContent = `Count: ${count}`;
    startButton.disabled = true;
  },
  complete: () => {
    message.textContent = 'Completed after reaching 10!';
    message.style.color = 'green';
    startButton.disabled = false;
  }
});
```

This code counts up from 0 to 9 and automatically completes just before reaching 10.


## 🎯 Contrast with skipWhile

`takeWhile` and `skipWhile` have contrasting behavior.

```ts
import { range } from 'rxjs';
import { takeWhile, skipWhile } from 'rxjs';

const numbers$ = range(0, 10);

// takeWhile: Take while condition is met
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// skipWhile: Skip while condition is met
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9
```

| Operator | Behavior | Completion Timing |
|---|---|---|
| `takeWhile(predicate)` | **Take** while condition is met | When condition becomes `false` |
| `skipWhile(predicate)` | **Skip** while condition is met | When source stream completes |


## 📋 Type-Safe Usage

Type-safe implementation example utilizing TypeScript generics.

```ts
import { Observable, from } from 'rxjs';
import { takeWhile } from 'rxjs';

interface SensorReading {
  timestamp: Date;
  value: number;
  unit: string;
  status: 'normal' | 'warning' | 'critical';
}

function getReadingsUntilWarning(
  readings$: Observable<SensorReading>
): Observable<SensorReading> {
  return readings$.pipe(
    takeWhile(reading => reading.status === 'normal')
  );
}

// Usage example
const readings$ = from([
  { timestamp: new Date(), value: 25, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 28, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 32, unit: '°C', status: 'warning' as const },
  { timestamp: new Date(), value: 35, unit: '°C', status: 'critical' as const },
] as SensorReading[]);

getReadingsUntilWarning(readings$).subscribe(reading => {
  console.log(`${reading.value}${reading.unit} - ${reading.status}`);
});
// Output:
// 25°C - normal
// 28°C - normal
```


## 🔄 Difference Between takeWhile and filter

`takeWhile` differs from `filter` in that it completes.

```ts
import { range } from 'rxjs';
import { takeWhile, filter } from 'rxjs';

const numbers$ = range(0, 10);

// filter: Only pass values matching condition (stream continues)
numbers$.pipe(
  filter(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter complete')
});
// Output: 0, 1, 2, 3, 4, filter complete

// takeWhile: Only while condition is met (completes when false)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('takeWhile complete')
});
// Output: 0, 1, 2, 3, 4, takeWhile complete
```

| Operator | Behavior | Stream Completion |
|---|---|---|
| `filter(predicate)` | Only pass values matching condition | When source stream completes |
| `takeWhile(predicate)` | Take while condition is met | When condition becomes `false` |


## ⚠️ Common Mistakes

> [!NOTE]
> `takeWhile` completes without outputting anything if the condition is `false` from the beginning. Make sure the condition is set appropriately.

### Wrong: Condition is False From the Start

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ❌ Bad example: Condition is false at first value
range(5, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Nothing output (condition is false at first value 5)
```

### Correct: Verify Condition

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ✅ Good example: Set condition appropriately
range(0, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4
```


## 🎓 Summary

### When to Use takeWhile
- ✅ When you want to control stream with dynamic conditions
- ✅ When you want to acquire data up to a threshold
- ✅ When you want to process only while a specific state continues
- ✅ When condition-based early completion is needed

### When to Use take
- ✅ When the number to acquire is fixed
- ✅ When simple count limit is needed

### When to Use filter
- ✅ When you want to extract only values matching a condition from the entire stream
- ✅ When you don't want to complete the stream

### Notes
- ⚠️ If condition is `false` from the beginning, completes without outputting anything
- ⚠️ By default, values for which condition becomes `false` are not output (can be included with `inclusive: true`)
- ⚠️ With infinite streams where condition is always `true`, continues forever


## 🚀 Next Steps

- **[take](/pt/guide/operators/filtering/take)** - Learn how to take first N values
- **[takeLast](/pt/guide/operators/filtering/takeLast)** - Learn how to take last N values
- **[takeUntil](../utility/takeUntil)** - Learn how to take values until another Observable fires
- **[filter](/pt/guide/operators/filtering/filter)** - Learn how to filter based on conditions
- **[Filtering Operator Practical Examples](/pt/guide/operators/filtering/practical-use-cases)** - Learn real use cases
