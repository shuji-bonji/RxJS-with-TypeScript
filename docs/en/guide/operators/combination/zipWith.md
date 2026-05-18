---
description: "zipWith is an RxJS join operator that aligns and pairs the original Observable with the corresponding ordered values from other Observables. It is ideal for order-critical stream joins, such as combining parallel processing results with order guarantees, mapping IDs to data, synchronizing related data published at different times, etc. The pipeable operator version is convenient for use in pipelines."
---

# zipWith - pair corresponding values

The `zipWith` operator will group together **corresponding ordered values** issued by the original Observable and the other Observable(s) specified.
It waits for values to arrive from all Observables, one at a time, and creates pairs when they are ready.
This is the Pipeable Operator version of Creation Function's `zip`.

## 🔰 Basic Syntax and Usage

```ts
import { of, interval } from 'rxjs';
import { zipWith, map, take } from 'rxjs';

const letters$ = of('A', 'B', 'C', 'D');
const numbers$ = interval(1000).pipe(
  map(val => val * 10),
  take(3)
);

letters$
  .pipe(zipWith(numbers$))
  .subscribe(([letter, number]) => {
    console.log(`${letter} - ${number}`);
  });

// Output:
// A - 0
// B - 10
// C - 20
// (Dis not output because there is no corresponding value)
```

- A pair is output when **one value from each Observable** is complete.
- When one Observable completes, the remaining values are discarded.

[🌐 RxJS Official Documentation - `zipWith`](https://rxjs.dev/api/operators/zipWith)

## 💡 Typical utilization pattern

- **Combine parallel processing results in guaranteed order**: pair multiple API call results
- **Mapping IDs to data**: Combine user IDs with corresponding profile data
- **Stream synchronization**: Synchronize related data issued at different times

## 🧠 Practical code example (with UI)

This is an example of pairing a list of user IDs and corresponding user names in sequence.

```ts
import { from, of } from 'rxjs';
import { zipWith, delay, concatMap } from 'rxjs';

// Output Area Creation
const output = document.createElement('div');
output.innerHTML = '<h3>zipWith Practical examples of:</h3>';
document.body.appendChild(output);

// UserIDStream (immediately issued)
const userIds$ = from([101, 102, 103, 104]);

// User name stream (issued every1(issued every second)
const userNames$ = from(['Alice', 'Bob', 'Carol']).pipe(
  concatMap(name => of(name).pipe(delay(1000)))
);

// zipand display
userIds$
  .pipe(zipWith(userNames$))
  .subscribe(([id, name]) => {
    const item = document.createElement('div');
    item.textContent = `👤 UserID ${id}: ${name}`;
    output.appendChild(item);
  });

// Output:
// 👤 UserID 101: Alice
// 👤 UserID 102: Bob
// 👤 UserID 103: Carol
// (104is not displayed because there is no corresponding name)
```

- IDs and names are paired in **one-to-one correspondence**.
- When one is complete, the remaining values are discarded.

## 🔄 Difference from Creation Function `zip`.

### Basic differences

|  | `zip` (Creation Function) | `zipWith` (Pipeable Operator) |
|---|---|---|
| **Where used** | Used as a standalone function | Used in `.pipe()` chain |
| **How to write** | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **First stream**. | Treat all as equal | Treat as main stream |
| **Advantages**. | Simple and readable | Easy to combine with other operators |

### Specific examples of usage

**For simple pairing, Creation Function is recommended**.

```ts
import { zip, of } from 'rxjs';

const questions$ = of('The name is？', 'The age is？', 'Address is？');
const answers$ = of('Taro', '30', 'Tokyo');
const scores$ = of(10, 20, 30);

// Simple and easy to read
zip(questions$, answers$, scores$).subscribe(([q, a, s]) => {
  console.log(`Q: ${q}, A: ${a}, Score: ${s}Score`);
});
// Output:
// Q: The name is？, A: Taro, Score: 10Score
// Q: The age is？, A: 30, Score: 20Score
// Q: Address is？, A: Tokyo, Score: 30Score
```

**If you want to add conversion to the main stream, Pipeable Operator is recommended**.

```ts
import { from, interval } from 'rxjs';
import { zipWith, map, take, filter } from 'rxjs';

// Task List
const tasks$ = from([
  { id: 1, name: 'Reporting', priority: 'high' },
  { id: 2, name: 'Email Reply', priority: 'low' },
  { id: 3, name: 'Meeting preparation', priority: 'high' },
  { id: 4, name: 'Organize documents', priority: 'medium' }
]);

// List of assignees (assigned every1(assigned every second)
const assignees$ = from(['Alice', 'Bob', 'Carol', 'Dave']).pipe(
  zipWith(interval(1000).pipe(take(4))),
  map(([name]) => name)
);

// ✅ Pipeable OperatorEdition - Complete in one pipeline
tasks$
  .pipe(
    filter(task => task.priority === 'high'),  // High priority only
    map(task => task.name),                     // Extract task names
    zipWith(assignees$),                        // Assign person in charge
    map(([taskName, assignee]) => ({
      task: taskName,
      assignee,
      assignedAt: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(assignment => {
    console.log(`[${assignment.assignedAt}] ${assignment.task} → Assigned to: ${assignment.assignee}`);
  });
// Output:
// [Time] Reporting → Assigned to: Alice
// [Time] Meeting preparation → Assigned to: Bob

// ❌ Creation FunctionEdition - Redundant
import { zip } from 'rxjs';
zip(
  tasks$.pipe(
    filter(task => task.priority === 'high'),
    map(task => task.name)
  ),
  assignees$
).pipe(
  map(([taskName, assignee]) => ({
    task: taskName,
    assignee,
    assignedAt: new Date().toLocaleTimeString()
  }))
).subscribe(assignment => {
  console.log(`[${assignment.assignedAt}] ${assignment.task} → Assigned to: ${assignment.assignee}`);
});
```

**Synchronization of order-critical data**

```ts
import { from } from 'rxjs';
import { zipWith, map, concatMap, delay } from 'rxjs';
import { of } from 'rxjs';

// UICreate
const output = document.createElement('div');
output.innerHTML = '<h3>Quiz game</h3>';
document.body.appendChild(output);

const questionArea = document.createElement('div');
questionArea.style.marginTop = '10px';
output.appendChild(questionArea);

// List of questions (prepared immediately)
const questions$ = from([
  'The capital of Japan is？',
  '1+1is？',
  'What is the number of the Earth's planets?？'
]);

// Answer list (simulating user input：2(every second)
const answers$ = from(['Tokyo', '2', '3']).pipe(
  concatMap((answer, index) =>
    of(answer).pipe(delay((index + 1) * 2000))
  )
);

// Correct answer list
const correctAnswers$ = from(['Tokyo', '2', '3']);

// ✅ Pipeable OperatorEdition - Process questions as mainstream
questions$
  .pipe(
    zipWith(answers$, correctAnswers$),
    map(([question, answer, correct], index) => ({
      no: index + 1,
      question,
      answer,
      correct,
      isCorrect: answer === correct
    }))
  )
  .subscribe(result => {
    const div = document.createElement('div');
    div.style.marginTop = '10px';
    div.style.padding = '10px';
    div.style.border = '1px solid #ccc';
    div.style.backgroundColor = result.isCorrect ? '#e8f5e9' : '#ffebee';
    div.innerHTML = `
      <strong>Question${result.no}:</strong> ${result.question}<br>
      <strong>Answers:</strong> ${result.answer}<br>
      <strong>Result:</strong> ${result.isCorrect ? '✅ Correct Answer！' : '❌ Incorrect'}
    `;
    questionArea.appendChild(div);
  });
```

### Summary

- **`zip`**: best if you just want to map multiple streams in order
- **`zipWith`**: best if you want to merge the main stream with other streams in guaranteed order while transforming and processing the main stream

## ⚠️ Notes.

### For different lengths.

When the shorter Observable completes, the remaining values of the longer one are discarded.

```ts
import { of } from 'rxjs';
import { zipWith } from 'rxjs';

const short$ = of(1, 2, 3);
const long$ = of('A', 'B', 'C', 'D', 'E');

short$.pipe(zipWith(long$)).subscribe(console.log);
// Output: [1, 'A'], [2, 'B'], [3, 'C']
// 'D'and'E'are discarded
```

### Memory Accumulation

If one Observable continues to issue values, the values will accumulate in memory until the other catches up.

```ts
import { interval} from 'rxjs';
import { zipWith, take } from 'rxjs';

// Fast Stream (100msper stream)
const fast$ = interval(100).pipe(take(10));

// Low-speed stream (1(every second)
const slow$ = interval(1000).pipe(take(3));

fast$.pipe(zipWith(slow$)).subscribe(console.log);
// Output: [0, 0] (1seconds later), [1, 1] (2seconds later), [2, 2] (3seconds later)
// fast$values are stored in memory and wait
```

### Difference from combineLatestWith

`zipWith` pairs the values in the corresponding order, while `combineLatestWith` combines the latest values.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(1000).pipe(take(3));
const source2$ = interval(1500).pipe(take(2));

// zipWith: Paired in corresponding order
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// Output: [0, 0], [1, 1]

// combineLatestWith: Combine the latest values
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// Output: [0, 0], [1, 0], [2, 0], [2, 1]
```

## 📚 Related Operators

- **[zip](/en/guide/creation-functions/combination/zip)** - Creation Function version
- **[combineLatestWith](/en/guide/operators/combination/combineLatestWith)** - Combine Latest Value
- **[withLatestFrom](/en/guide/operators/combination/withLatestFrom)** - only mainstream triggers
