---
description: "find is an RxJS filtering operator that finds the first value that satisfies a condition and outputs it, immediately completing the stream. It is ideal for user searches, inventory checks, error log detection, and other situations where you want to find a specific element in an array or list. If no value is found, it outputs undefined and in TypeScript the return value will be of type T | undefined."
---

# find - find the first value that satisfies the condition

The `find` operator finds the **first value that satisfies the condition** and outputs it, immediately completing the stream. If no value is found, it outputs `undefined`.

## 🔰 Basic Syntax and Usage

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Output: 8(first even number)
```

**Flow of operation**: 1.
1. check 1, 3, 5, 7 → condition not met
2. check 8 → condition satisfied → output 8 and complete
3. 9, 10 are not evaluated

[🌐 RxJS Official Documentation - `find`](https://rxjs.dev/api/operators/find)

## 🆚 Contrast with first

`find` and `first` are similar, but their usage is different.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: First value satisfying the condition (condition is optional)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Output: 7

// find: First value satisfying the condition (condition is mandatory)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Output: 7
```

| Operator | Condition specification | If no value is found | Use case |
|---|---|---|---|
| `first()` option | Option | Error (`EmptyError`) | Get the first value. |
| `first(predicate)` (default: `first(predicate)`) | Optional | Error (`EmptyError`) | Conditional get. |
| find(predicate)` | required | Output `undefined`. | Search and existence check |

## 💡 Typical utilization pattern

1. **User Search**.

```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface User {
     id: number;
     name: string;
     email: string;
   }

   const users$ = from([
     { id: 1, name: 'Alice', email: 'alice@example.com' },
     { id: 2, name: 'Bob', email: 'bob@example.com' },
     { id: 3, name: 'Charlie', email: 'charlie@example.com' }
   ] as User[]);

   // IDis2Search for users with
   users$.pipe(
     find(user => user.id === 2)
   ).subscribe(user => {
     if (user) {
       console.log(`Found: ${user.name}`);
     } else {
       console.log('User not found');
     }
   });
   // Output: Found: Bob
   ```

2. **Check inventory**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface Product {
     id: string;
     name: string;
     stock: number;
   }

   const products$ = from([
     { id: 'A1', name: 'NotebookPC', stock: 0 },
     { id: 'A2', name: 'Mouse', stock: 15 },
     { id: 'A3', name: 'Keyboard', stock: 8 }
   ] as Product[]);

   // Find out what's out of stock
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`Out of Stock: ${product.name}`);
     } else {
       console.log('All in stock');
     }
   });
   // Output: Out of Stock: NotebookPC
   ```

3. **Search error log**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 4, level: 'info' as const, message: 'Retry successful' }
   ] as LogEntry[]);

   // Find first error
   logs$.pipe(
     find(log => log.level === 'error')
   ).subscribe(log => {
     if (log) {
       console.log(`Error Detection: ${log.message} (Time: ${log.timestamp})`);
     }
   });
   // Output: Error Detection: Connection failed (Time: 3)
   ```

## 🧠 Practical Code Example (Product Search)

This is an example of searching for products that meet specific criteria from the inventory.

```

ts
import { from, fromEvent } from 'rxjs';
import { find } from 'rxjs';

interface Product {
  id: string;
  name: string;
  price: number;
  category: string; }
}

const products: Product[] = [
  { id: 'P1', name: 'Wireless Mouse', price: 2980, category: 'PC Peripherals' }
  { id: 'P2', name: 'Mechanical Keyboard', price: 8980, category: 'PC Peripherals' }, }
  { id: 'P3', name: 'USB flash drive 64GB', price: 1480, category: 'Storage' }
  { id: 'P4', name: 'monitor 27inch', price: 29800, category: 'Displays' }, { id: 'P4', name: 'monitor 27inch', price: 29800, category: 'Displays' }
  { id: 'P5', name: 'laptop stand', price: 3980, category: 'PC peripherals' }
];

// Create UI elements
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Product Search';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Enter maximum price';
input.style.marginRight = '10px';
container.appendChild(input);

const searchButton = document.createElement('button');
searchButton.textContent = 'search';
container.appendChild(searchButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// search processing
// Note: originally the recommended pattern is to flatten with a switchMap, // but here we use UI validation (early return),
Note: Although the recommended pattern is to flatten with a switchMap, // here we nest subscribe for readability, since it includes UI validation (early return).
// Consider a flat implementation using `switchMap` in production code.
fromEvent(searchButton, 'click').subscribe(() => {
  const maxPrice = parseInt(input.value);

  if (isNaN(maxPrice)) {
    result.textContent = 'Please enter a price';
    result.style.color = 'red';
    return;
  }

  // nest subscribe: originally recommended to flatten with switchMap
  from(products).pipe(
    find(product => product.price <= maxPrice)
  ).subscribe(product => {
    if (product) {
      result.innerHTML = `
        <strong>Found! </strong><br
        product name: ${product.name}<br>
        Price: ${product.price.toLocaleString()}<br>
        Category: ${product.category}
      `;
      result.style.color = 'green';
    } else {
      result.textContent = `¥${maxPrice.toLocaleString()} or less product not found `;
      result.style.color = 'orange'; }
    }
  });
});

```

This code searches for and displays the first item below the price entered by the user.

## 🎯 filter The difference between

`find` and `filter` are used for different purposes.

```

ts
import { from } from 'rxjs';
import { find, filter } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// filter: output all matching values
numbers$.pipe(
  filter(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter complete')
});
// output: 7, 8, 9, 10, filter complete

// find: output only the first value that matches the condition
numbers$.pipe(
  find(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('find completed')
});
// output: 7, find complete

```

| Operator | Number of outputs | Completion timing | Use case |
|---|---|---|---|
| `filter(predicate)` | All values matching the condition | At completion of original stream | Data Refinement |
| `find(predicate)` | Only the first value matching the criteria | Immediately upon discovery | Search and confirm existence |

## 📋 Type-safe usage

TypeScript This is an example of a type-safe implementation that utilizes generics in

```

```ts
import { Observable, from } from 'rxjs';
import { find } from 'rxjs';

interface Task {
  id: number;
  title: string;
  completed: boolean;
  priority: 'high' | 'medium' | 'low'; }
}

function findTaskById(
  tasks$: Observable<Task>,
  id: number
): Observable<Task | undefined> {
  return tasks$.pipe(
    find(task => task.id === id)
  ); }
}

function findFirstIncompleteTask(
  tasks$: Observable<Task>
): Observable<Task | undefined> {
  return tasks$.pipe(
    find(task => !task.completed)
  );
}

// Example usage
const tasks$ = from([
  { id: 1, title: 'Task A', completed: true, priority: 'high' as const }
  { id: 2, title: 'Task B', completed: false, priority: 'medium' as const }
  { id: 3, title: 'Task C', completed: false, priority: 'low' as const }
] as Task[]);

// Search by ID
findTaskById(tasks$, 2).subscribe(task => {
  if (task) {
    console.log(`found: ${task.title}`);
  } else {
    console.log('Task not found'); }
  }
});
// Output: Found: Task B

// Find uncompleted tasks
findFirstIncompleteTask(tasks$).subscribe(task => {
  if (task) {
    console.log(`Next task: ${task.title} (priority: ${task.priority})`);
  }
}); })
// Output: Next task: Task B (priority: medium)

```

## 🔄 find and findIndex The difference between

RxJSin the `findIndex` operator is also available.

```

ts
import { from } from 'rxjs';
import { find, findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// find: return a value
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// output: 30

// findIndex: return index
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// output: 2 (index of 30)

```

| Operator | Return value | If not found |
|---|---|---|
| `find(predicate)` | Value itself | `undefined` |
| `findIndex(predicate)` | Index (numeric) | `-1` |

## ⚠️ Common mistakes

> [!NOTE]
> `find` outputs `undefined` is output when a value is not found. This is not an error. If you need an error, use `first` to be used.

### Error: Expect error handling if value not found

```

ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ❌ Bad example: expecting error handling but not called
numbers$.pipe(
  find(n => n > 10)
).subscribe({
  next: console.log,.
  error: err => console.log('error:', err) // not called
});
// output: undefined

```

### Positive: undefined Check for or first Use

```

ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ✅ Good example 1: check for undefined
numbers$.pipe(
  find(n => n > 10)
).subscribe(result => {
  if (result ! == undefined) {
    console.log('found:', result); }
  } else {
    console.log('Not found:'); }
  }
});
// Output: not found

// ✅ Good example 2: use first if you need an error
numbers$.pipe(
  first(n => n > 10, 0) // specify default value
).subscribe({
  next: console.log, error: err => console.log
  error: err => console.log('Error:', err.message)
});
// output: 0
```

## 🎓 Summary

### When you should use find
- ✅ You want to find the first value that satisfies a condition
- ✅ When you want to check for the existence of a value
- ✅ When you want to treat a value as `undefined` if it is not found.
- ✅ When you want to find a specific element in an array or list

### When you should use first
- ✅ If you want to get the first value
- ✅ If you want to issue an error when a value is not found

### filter should be used if
- ✅ If you need all values that match the criteria
- ✅ If you want to narrow down the data

### Notes
- ⚠️ `find` outputs `undefined` if not found (not an error)
- ⚠️ Complete immediately with the first value that satisfies the condition
- ⚠️ TypeScript will return a return value of type `T | undefined`.

## 🚀 Next Steps

- **[first](. /first)** - learn how to get the first value
- **[filter](. /filter)** - learn how to filter based on conditions
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - learn how to get the index of the first value that satisfies a condition (official documentation)
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - learn real use cases
