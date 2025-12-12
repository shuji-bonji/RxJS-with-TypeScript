---
description: The groupBy operator is an RxJS operator that groups stream values based on a specified key and creates a separate Observable for each group, which is utilized for data classification and aggregation processes.
---

# groupBy - Group Values Based on a Key

The `groupBy` operator **groups** values issued from a stream based on a specified key and outputs each group as a separate Observable.
This is useful for categorizing data or applying different processing to each group.

## 🔰 Basic Syntax and Usage

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

interface Person {
  name: string;
  age: number;
}

const people: Person[] = [
  { name: 'Taro', age: 25 },
  { name: 'Hanako', age: 30 },
  { name: 'Jiro', age: 25 },
  { name: 'Misaki', age: 30 },
  { name: 'Kenta', age: 35 },
];

from(people).pipe(
  groupBy(person => person.age), // Group by age
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(arr => ({ age: group.key, people: arr }))
    )
  )
).subscribe(result => {
  console.log(`Age ${result.age}:`, result.people);
});

// Output:
// Age 25: [{name: 'Taro', age: 25}, {name: 'Jiro', age: 25}]
// Age 30: [{name: 'Hanako', age: 30}, {name: 'Misaki', age: 30}]
// Age 35: [{name: 'Kenta', age: 35}]
```

- `groupBy(person => person.age)` groups data by age as a key
- Each group is treated as a `GroupedObservable` and the group's key is accessible via the `key` property
- `mergeMap` handles each grouped Observable

[🌐 RxJS Official Documentation - `groupBy`](https://rxjs.dev/api/operators/groupBy)

## 💡 Typical Usage Patterns

- Categorization of data by category
- Aggregate processing by group
- Processing of logs and events by type
- Data grouping and transformation

## 🧠 Practical Code Example (with UI)

This example shows how to display the number of pieces grouped by color when a button is clicked.

```ts
import { fromEvent, from } from 'rxjs';
import { groupBy, mergeMap, toArray, switchMap, map } from 'rxjs';

// Create buttons
const colors = ['Red', 'Blue', 'Green', 'Yellow'];
colors.forEach(color => {
  const button = document.createElement('button');
  button.textContent = color;
  button.style.margin = '5px';
  button.style.padding = '10px';
  button.dataset.color = color;
  document.body.appendChild(button);
});

const calculateButton = document.createElement('button');
calculateButton.textContent = 'Aggregate';
calculateButton.style.margin = '5px';
calculateButton.style.padding = '10px';
document.body.appendChild(calculateButton);

// Create output area
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Record clicked colors
const clicks: string[] = [];

// Click events for color buttons
fromEvent(document, 'click').subscribe((event: Event) => {
  const target = event.target as HTMLElement;
  const color = target.dataset.color;
  if (color) {
    clicks.push(color);
    output.innerHTML = `Selected colors: ${clicks.join(', ')}`;
  }
});

// Group and display when aggregate button is clicked
fromEvent(calculateButton, 'click').pipe(
  switchMap(() =>
    from(clicks).pipe(
      groupBy(color => color),
      mergeMap(group =>
        group.pipe(
          toArray(),
          map(items => ({ color: group.key, count: items.length }))
        )
      ),
      toArray()
    )
  )
).subscribe(results => {
  if (results.length === 0) {
    output.innerHTML = '<p>No colors selected yet</p>';
    return;
  }
  const resultText = results
    .map(r => `${r.color}: ${r.count} times`)
    .join('<br>');
  output.innerHTML = `<h3>Aggregate Results</h3>${resultText}`;
});
```

- Click on color buttons to select colors
- Click the "Aggregate" button to group by color and display the number of pieces
- Group by color with `groupBy` and count the number of elements in each group

## 🎯 Example of Aggregation by Category

Here is an example of grouping products by category and calculating the total amount for each category.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, reduce, map } from 'rxjs';

interface Product {
  name: string;
  category: string;
  price: number;
}

const products: Product[] = [
  { name: 'Apple', category: 'Fruit', price: 150 },
  { name: 'Orange', category: 'Fruit', price: 100 },
  { name: 'Carrot', category: 'Vegetable', price: 80 },
  { name: 'Tomato', category: 'Vegetable', price: 120 },
  { name: 'Milk', category: 'Dairy', price: 200 },
  { name: 'Cheese', category: 'Dairy', price: 300 },
];

from(products).pipe(
  groupBy(product => product.category),
  mergeMap(group =>
    group.pipe(
      reduce((total, product) => total + product.price, 0),
      map(total => ({ category: group.key, total }))
    )
  )
).subscribe(result => {
  console.log(`${result.category}: $${result.total}`);
});

// Output:
// Fruit: $250
// Vegetable: $200
// Dairy: $500
```

## 🎯 Example of Element Selector Usage

When grouping, values can also be converted.

```ts
import { from } from 'rxjs';
import { groupBy, map, mergeMap, toArray } from 'rxjs';

interface Student {
  name: string;
  grade: number;
  score: number;
}

const students: Student[] = [
  { name: 'Taro', grade: 1, score: 85 },
  { name: 'Hanako', grade: 2, score: 92 },
  { name: 'Jiro', grade: 1, score: 78 },
  { name: 'Misaki', grade: 2, score: 88 },
];

from(students).pipe(
  groupBy(
    student => student.grade,           // Key selector
    student => student.name             // Element selector (keep only names)
  ),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(names => ({ grade: group.key, students: names }))
    )
  )
).subscribe(result => {
  console.log(`Grade ${result.grade}:`, result.students.join(', '));
});

// Output:
// Grade 1: Taro, Jiro
// Grade 2: Hanako, Misaki
```

- 1st argument: Key selector (criteria for grouping)
- 2nd argument: Element selector (value to be stored in the group)

## 🎯 Utilizing Type-Safe groupBy

This is an example of leveraging TypeScript's type inference.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

type LogLevel = 'info' | 'warning' | 'error';

interface LogEntry {
  level: LogLevel;
  message: string;
  timestamp: number;
}

const logs: LogEntry[] = [
  { level: 'info', message: 'App started', timestamp: 1000 },
  { level: 'warning', message: 'Warning message', timestamp: 2000 },
  { level: 'error', message: 'Error occurred', timestamp: 3000 },
  { level: 'info', message: 'Process completed', timestamp: 4000 },
  { level: 'error', message: 'Connection error', timestamp: 5000 },
];

from(logs).pipe(
  groupBy(log => log.level),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(entries => ({
        level: group.key,
        count: entries.length,
        messages: entries.map(e => e.message)
      }))
    )
  )
).subscribe(result => {
  console.log(`[${result.level.toUpperCase()}] ${result.count} entries`);
  result.messages.forEach(msg => console.log(`  - ${msg}`));
});

// Output:
// [INFO] 2 entries
//   - App started
//   - Process completed
// [WARNING] 1 entries
//   - Warning message
// [ERROR] 2 entries
//   - Error occurred
//   - Connection error
```

## 🎯 Apply Different Processing to Each Group

Here is an example of applying different processing to each group.

```ts
import { from, of } from 'rxjs';
import { groupBy, mergeMap, delay, map } from 'rxjs';

interface Task {
  id: number;
  priority: 'high' | 'medium' | 'low';
  name: string;
}

const tasks: Task[] = [
  { id: 1, priority: 'high', name: 'Urgent task' },
  { id: 2, priority: 'low', name: 'Low priority task' },
  { id: 3, priority: 'high', name: 'Important task' },
  { id: 4, priority: 'medium', name: 'Normal task' },
];

from(tasks).pipe(
  groupBy(task => task.priority),
  mergeMap(group => {
    // Set delay time according to priority
    const delayTime =
      group.key === 'high' ? 0 :
      group.key === 'medium' ? 1000 :
      2000;

    return group.pipe(
      delay(delayTime),
      map(task => ({ ...task, processedAt: Date.now() }))
    );
  })
).subscribe(task => {
  console.log(`[${task.priority}] Processing ${task.name}`);
});

// Output (in priority order):
// [high] Processing Urgent task
// [high] Processing Important task
// (After 1 second)
// [medium] Processing Normal task
// (After another 1 second)
// [low] Processing Low priority task
```

## ⚠️ Notes

### Subscription Management for Group Observable

`groupBy` creates an Observable for each group. These Observables can cause memory leaks if not properly subscribed to.

```ts
// ❌ Bad example: Not subscribing to group Observables
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd')
).subscribe(group => {
  // Not subscribing to group Observable
  console.log('Group:', group.key);
});
```

**Countermeasure**: Always use `mergeMap`, `concatMap`, `switchMap`, etc. to handle each group.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray } from 'rxjs';

// ✅ Good example: Properly handle each group
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd'),
  mergeMap(group =>
    group.pipe(toArray())
  )
).subscribe(console.log);
```

### Dynamic Creation of Groups

A new group Observable is created each time a new key appears. Be careful when there are many key types.

```ts
// Example where the number of key types can increase infinitely
fromEvent(document, 'click').pipe(
  groupBy(() => Math.random()) // Different key every time
).subscribe(); // Memory leak risk
```

## 📚 Related Operators

- [`partition`](https://rxjs.dev/api/index/function/partition) - Split into two Observables by condition
- [`reduce`](/pt/guide/operators/transformation/reduce) - Get the final aggregate result
- [`scan`](/pt/guide/operators/transformation/scan) - Cumulative aggregation
- [`toArray`](/pt/guide/operators/utility/toArray) - Combine all values into an array

## Summary

The `groupBy` operator allows you to group values in a stream based on keys and **treat each group as a separate Observable**. This is very useful for complex data processing, such as classifying data, aggregating by category, and processing each group differently. However, each group Observable must be properly subscribed to and is usually used in conjunction with `mergeMap` or similar.
