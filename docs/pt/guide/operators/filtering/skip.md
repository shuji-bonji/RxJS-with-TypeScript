---
description: The skip operator skips the first specified number of values from the Observable stream and outputs only subsequent values. This is useful when you want to ignore initial data or skip a warm-up period.
---

# skip - Skip the First N Values

The `skip` operator skips the **first specified number** of values from the stream and outputs only the subsequent values.


## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, 7, ...
```

- Skips the first 3 values (0, 1, 2)
- The fourth and subsequent values (3, 4, 5, ...) are all output
- The stream completes at the original completion time

[🌐 RxJS Official Documentation - `skip`](https://rxjs.dev/api/operators/skip)


## 🆚 Contrast with take

`skip` and `take` have contrasting behavior.

```ts
import { range } from 'rxjs';
import { skip, take } from 'rxjs';

const numbers$ = range(0, 10); // 0 to 9

// take: Get the first N values
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Output: 0, 1, 2

// skip: Skip the first N values
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, 7, 8, 9

// Combination: Skip the first 3 and get the next 3
numbers$.pipe(
  skip(3),
  take(3)
).subscribe(console.log);
// Output: 3, 4, 5
```

| Operator | Behavior | Completion Timing |
|---|---|---|
| `take(n)` | Get the first n values | Automatically completes after n values |
| `skip(n)` | Skip the first n values | When the original stream completes |


## 💡 Typical Usage Patterns

1. **Skipping Initial Values**
   ```ts
   import { BehaviorSubject } from 'rxjs';
   import { skip } from 'rxjs';

   const state$ = new BehaviorSubject<number>(0);

   // Skip initial value and monitor only changes
   state$.pipe(
     skip(1)
   ).subscribe(value => {
     console.log(`State changed: ${value}`);
   });

   state$.next(1); // Output: State changed: 1
   state$.next(2); // Output: State changed: 2
   ```

2. **Skip Warm-up Period**
   ```ts
   import { interval } from 'rxjs';
   import { skip, map } from 'rxjs';

   // Simulate sensor data
   const sensorData$ = interval(100).pipe(
     map(() => Math.random() * 100)
   );

   // Skip the first 10 values (1 second) as calibration period
   sensorData$.pipe(
     skip(10)
   ).subscribe(data => {
     console.log(`Sensor value: ${data.toFixed(2)}`);
   });
   ```

3. **Pagination**
   ```ts
   import { from } from 'rxjs';
   import { skip, take } from 'rxjs';

   interface Item {
     id: number;
     name: string;
   }

   const allItems$ = from([
     { id: 1, name: 'Item 1' },
     { id: 2, name: 'Item 2' },
     { id: 3, name: 'Item 3' },
     { id: 4, name: 'Item 4' },
     { id: 5, name: 'Item 5' },
     { id: 6, name: 'Item 6' },
   ] as Item[]);

   const pageSize = 2;
   const pageNumber = 2; // 0-indexed

   // Get the items on page 2 (items 5 and 6)
   allItems$.pipe(
     skip(pageNumber * pageSize),
     take(pageSize)
   ).subscribe(item => {
     console.log(item);
   });
   // Output: { id: 5, name: 'Item 5' }, { id: 6, name: 'Item 6' }
   ```


## 🧠 Practical Code Example (Counter)

This example skips the first 3 clicks and counts only the 4th and subsequent clicks.

```ts
import { fromEvent } from 'rxjs';
import { skip, scan } from 'rxjs';

// Create UI elements
const container = document.createElement('div');
document.body.appendChild(container);

const button = document.createElement('button');
button.textContent = 'Click';
container.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'Count: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = 'The first 3 clicks will be skipped';
container.appendChild(message);

// Click event
fromEvent(button, 'click').pipe(
  skip(3), // Skip first 3 clicks
  scan((count) => count + 1, 0)
).subscribe(count => {
  counter.textContent = `Count: ${count}`;
  if (count === 1) {
    message.textContent = 'Count starts after 4th click!';
    message.style.color = 'green';
  }
});
```

This code ignores the first 3 clicks and starts counting as "1" from the 4th click.


## 🎯 Difference between skip and skipWhile

```ts
import { of } from 'rxjs';
import { skip, skipWhile } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6);

// skip: Specify first N values by number
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 4, 5, 6

// skipWhile: Skip while condition is met
numbers$.pipe(
  skipWhile(n => n < 4)
).subscribe(console.log);
// Output: 4, 5, 6
```

| Operator | Skip Condition | Use Case |
|---|---|---|
| `skip(n)` | Skip the first n values by number | Fixed number skip |
| `skipWhile(predicate)` | Skip while condition is satisfied | Condition-based skip |
| `skipUntil(notifier$)` | Skip until another Observable fires | Time-based skip |


## 📋 Type-Safe Usage

Here is an example of a type-safe implementation utilizing generics in TypeScript.

```ts
import { Observable, from } from 'rxjs';
import { skip, take } from 'rxjs';

interface User {
  id: number;
  name: string;
  role: 'admin' | 'user';
}

function getPaginatedUsers(
  users$: Observable<User>,
  page: number,
  pageSize: number
): Observable<User> {
  return users$.pipe(
    skip(page * pageSize),
    take(pageSize)
  );
}

// Example usage
const users$ = from([
  { id: 1, name: 'Alice', role: 'admin' as const },
  { id: 2, name: 'Bob', role: 'user' as const },
  { id: 3, name: 'Charlie', role: 'user' as const },
  { id: 4, name: 'Dave', role: 'admin' as const },
  { id: 5, name: 'Eve', role: 'user' as const },
] as User[]);

// Get page 1 (second page, 0-indexed)
getPaginatedUsers(users$, 1, 2).subscribe(user => {
  console.log(`${user.name} (${user.role})`);
});
// Output: Charlie (user), Dave (admin)
```


## ⚠️ Common Mistakes

> [!NOTE]
> `skip` only skips the first N values and does not complete the stream. In infinite streams, combine with `take` to set the termination condition.

### Wrong: Use skip Only with Infinite Streams

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

// ❌ Bad example: Infinite stream continues as is
interval(1000).pipe(
  skip(5)
).subscribe(console.log);
// 5, 6, 7, 8, ... continues forever
```

### Correct: Combine with take to Set Termination Condition

```ts
import { interval } from 'rxjs';
import { skip, take } from 'rxjs';

// ✅ Good example: Limit the number of values after skip
interval(1000).pipe(
  skip(5),
  take(3)
).subscribe({
  next: console.log,
  complete: () => console.log('Completed')
});
// 5, 6, 7, Completed
```


## 🎓 Summary

### When to Use skip
- ✅ When you want to ignore the initial value or the first N data
- ✅ When you want to skip the initial value of BehaviorSubject
- ✅ When you want to get data for a specific page in pagination
- ✅ When you want to skip the sensor calibration period

### When to Combine with take
- ✅ When you want to get only a specific range of data
- ✅ When you want to get the middle portion of data from an infinite stream

### Notes
- ⚠️ In infinite streams, combine with `take` to set the termination condition
- ⚠️ `skip(0)` works the same as the original stream (skips nothing)
- ⚠️ If the skip count is greater than the total data count, it completes without outputting anything


## 🚀 Next Steps

- **[take](/pt/guide/operators/filtering/take)** - Learn how to get the first N values
- **[first](/pt/guide/operators/filtering/first)** - Learn how to get the first value or the first value that satisfies a condition
- **[last](/pt/guide/operators/filtering/last)** - Learn how to get the last value
- **[filter](/pt/guide/operators/filtering/filter)** - Learn how to filter based on conditions
- **[Filtering Operator Practical Examples](/pt/guide/operators/filtering/practical-use-cases)** - Learn real use cases
