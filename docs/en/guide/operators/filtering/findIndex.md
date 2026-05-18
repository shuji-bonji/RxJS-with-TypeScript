---
description: "The findIndex operator is an RxJS filtering operator that returns the index of the first value that satisfies the condition. If not found, it returns -1."
---

# findIndex - get the index matching the condition

The `findIndex` operator returns **the index of the first value that matches the condition** and completes the stream immediately. If no value is found, returns `-1`.

## 🔰 Basic Syntax and Usage

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Output: 4(first even8index)
```

**Flow of operation**:.
1. 1 (index 0) -> odd, skip
2. 3 (index 1) -> Odd, skip
3. 5 (index 2) → Odd, skip
4. 7 (index 3) → Odd, skip
5. 8 (index 4) → even number, output index 4 and complete

[🌐 RxJS Official Documentation - `findIndex`](https://rxjs.dev/api/operators/findIndex)

## 💡 Typical utilization pattern

- **Positioning in an array**: Obtain the position of an element that satisfies a specific condition
- **Checking the order**: how many times an element satisfying a certain condition appears
- **Rearranging data**: Processing using index information
- **Existence check**: Checks for the existence of an element by checking whether it is 1 or not.

## 🧠 Practical code example 1: Searching a task list

This is an example of finding the location of a task with specific conditions from a task list.

```ts
import { from, fromEvent } from 'rxjs';
import { findIndex } from 'rxjs';

interface Task {
  id: number;
  title: string;
  priority: 'high' | 'medium' | 'low';
  completed: boolean;
}

const tasks: Task[] = [
  { id: 1, title: 'Email reply', priority: 'low', completed: true },
  { id: 2, title: 'Document preparation', priority: 'medium', completed: true },
  { id: 3, title: 'Meeting Preparation', priority: 'high', completed: false },
  { id: 4, title: 'Code review', priority: 'high', completed: false },
  { id: 5, title: 'Document update', priority: 'low', completed: false }
];

// UICreate
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Search Tasks';
container.appendChild(title);

// Task List Display
const taskList = document.createElement('ul');
taskList.style.listStyle = 'none';
taskList.style.padding = '0';
tasks.forEach((task, index) => {
  const li = document.createElement('li');
  li.style.padding = '5px';
  li.style.borderBottom = '1px solid #eee';
  const status = task.completed ? '✅' : '⬜';
  const priorityBadge = task.priority === 'high' ? '🔴' : task.priority === 'medium' ? '🟡' : '🟢';
  li.textContent = `[${index}] ${status} ${priorityBadge} ${task.title}`;
  taskList.appendChild(li);
});
container.appendChild(taskList);

// Search Button
const buttonContainer = document.createElement('div');
buttonContainer.style.marginTop = '10px';
container.appendChild(buttonContainer);

const button1 = document.createElement('button');
button1.textContent = 'Search for the first uncompleted task';
button1.style.marginRight = '10px';
buttonContainer.appendChild(button1);

const button2 = document.createElement('button');
button2.textContent = 'Find the first high priority task';
buttonContainer.appendChild(button2);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Search for the first uncompleted task
// Note: Originally, the recommended pattern is to flatten with switchMap is the recommended pattern,
// for readability (in production code, it is subscribe (In production code, we recommend switchMap is recommended).
fromEvent(button1, 'click').subscribe(() => {
  // Nesting subscribe: Originally, the recommended pattern is to flatten with switchMap Flattening with
  from(tasks).pipe(
    findIndex(task => !task.completed)
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Found in</strong><br>
        position: Index ${index}<br>
        Task: ${task.title}<br>
        Priority: ${task.priority}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ No uncompleted tasks found';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});

// Find the first high priority task
// Note: Originally, the recommended pattern is to flatten with switchMap The recommended pattern (in production code) is to flatten by switchMap is recommended).
fromEvent(button2, 'click').subscribe(() => {
  // Nesting subscribe: Originally, the recommended pattern is to flatten with switchMap Flattening with
  from(tasks).pipe(
    findIndex(task => task.priority === 'high')
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Found in</strong><br>
        position: Index ${index}<br>
        Task: ${task.title}<br>
        Completed status: ${task.completed ? 'Completed' : 'Uncompleted'}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ No high priority tasks were found';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});
```

- Searches the task list for the location of the first task that satisfies the condition.
- If not found, `-1` is returned.

## 🎯 Practical Code Example 2: Real-Time Data Location

This is an example of detecting the location of the first value from a stream that satisfies the condition.

```ts
import { interval } from 'rxjs';
import { findIndex, map, take } from 'rxjs';

// UICreate
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Real-time data search';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = '50Searching for locations where values greater than or equal to...';
container.appendChild(status);

const dataDisplay = document.createElement('div');
dataDisplay.style.marginTop = '10px';
dataDisplay.style.padding = '10px';
dataDisplay.style.border = '1px solid #ccc';
dataDisplay.style.maxHeight = '150px';
dataDisplay.style.overflow = 'auto';
container.appendChild(dataDisplay);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.fontWeight = 'bold';
container.appendChild(result);

// Generating random values (0~ (~)100)
const data$ = interval(500).pipe(
  take(20),
  map(i => ({ index: i, value: Math.floor(Math.random() * 100) }))
);

// Data Display
data$.subscribe(data => {
  const div = document.createElement('div');
  const highlight = data.value >= 50 ? 'background-color: #fff9c4;' : '';
  div.style.cssText = `padding: 5px; ${highlight}`;
  div.textContent = `[${data.index}] Value: ${data.value}`;
  dataDisplay.appendChild(div);
  dataDisplay.scrollTop = dataDisplay.scrollHeight;
});

// 50Search the index for the first value greater than or equal to
data$.pipe(
  findIndex(data => data.value >= 50)
).subscribe(index => {
  status.textContent = '';
  if (index !== -1) {
    result.innerHTML = `
      ✅ 50More than or equal to value found<br>
      position: Index ${index}
    `;
    result.style.color = 'green';
  } else {
    result.textContent = '❌ 50No values greater than or equal to the first value found';
    result.style.color = 'orange';
  }
});
```

- Detects the position of the first value greater than 50 from random values generated every 0.5 seconds.
- Highlighting is used for visual clarity.

## 🆚 Comparison with similar operators

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Returns the index of the first value that satisfies the condition
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Output: 2Returns the index of the first value that satisfies the condition30index)

// find: Returns the first value that satisfies the condition
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Output: 30

// elementAt: Returns the value at the specified index
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Output: 30
```

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Output: 4(first even8index)
```

## 🔄 Comparison with JavaScript's Array.findIndex()

RxJS `findIndex` works similar to JavaScript's array method `Array.prototype.findIndex()`.


```ts
// JavaScript Array of
const numbers = [10, 20, 30, 40, 50];
const index1 = numbers.findIndex(n => n > 25);
console.log(index1); // 2

// RxJS of Observable
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log); // 2
```

**Main differences**:.
- **Array**: returns the result synchronously and immediately
- **Observable**: asynchronous, waits for values to flow from the stream

## ⚠️ Notes.

### 1. returns -1 if not found

If no value satisfies the condition, returns `-1` instead of an error.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('No value found that satisfies the condition');
  } else {
    console.log(`Index: ${index}`);
  }
});
// Output: No value found that satisfies the condition
```

### 2. complete when first found

The stream completes immediately upon the first value found that satisfies the condition.

```ts
import { interval } from 'rxjs';
import { findIndex, tap } from 'rxjs';

interval(1000).pipe(
  tap(val => console.log(`Value: ${val}`)),
  findIndex(n => n >= 3)
).subscribe(index => {
  console.log(`Index: ${index}`);
});
// Output:
// Value: 0
// Value: 1
// Value: 2
// Value: 3
// Index: 3
```

### 3. Type safety in TypeScript

`findIndex` always returns type `number`.

```ts
import { Observable, from } from 'rxjs';
import { findIndex } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

function findFirstInactiveUserIndex(
  users$: Observable<User>
): Observable<number> {
  return users$.pipe(
    findIndex(user => !user.isActive)
  );
}

const users$ = from([
  { id: 1, name: 'Alice', isActive: true },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true }
]);

findFirstInactiveUserIndex(users$).subscribe(index => {
  // index is an array of number type
  if (index !== -1) {
    console.log(`The first inactive user is the index ${index} is`);
  }
});
// Output: The first inactive user is the index 1 is
```

### 4. index starts from 0

As with arrays, indexes start at 0.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// Output: 0(first element)
```

## 📚 Related Operators

- **[find](. /find)** - get the first value that satisfies the condition
- **[elementAt](. /elementAt)** - get the value at the specified index
- **[first](. /first)** - get the first value
- **[filter](. /filter)** - get all values that satisfy the condition

## Summary

The `findIndex` operator returns the index of the first value that satisfies the condition.

- ✅ Similar behavior to JavaScript's `Array.findIndex()`.
- ✅ Ideal when index information is needed
- ✅ Returns `-1` if not found (not an error)
- ✅ Completes immediately when found
- ⚠️ Return value is always a `number` (-1 or an integer greater than or equal to 0)
- ⚠️ Use `find` if you need the value itself
