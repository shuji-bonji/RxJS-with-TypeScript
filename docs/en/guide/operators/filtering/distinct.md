---
description: The distinct operator removes all duplicate values and outputs only unique values that have never been output. Care should be taken with infinite streams, as it internally uses Set to store previously emitted values.
---

# distinct - Remove All Duplicate Values

The `distinct` operator monitors all values emitted by Observable and outputs **only values that have never been output before**. Internally, it uses Set to remember previously emitted values.


## 🔰 Basic Syntax and Usage

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

const numbers$ = of(1, 2, 1, 3, 2, 4, 1, 5);

numbers$.pipe(
  distinct()
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5
```

- Removes duplicates throughout the stream
- Once a value is output, it is ignored no matter how many times it appears subsequently
- `distinctUntilChanged` removes only **consecutive** duplicates, while `distinct` removes **all** duplicates

[🌐 RxJS Official Documentation - `distinct`](https://rxjs.dev/api/operators/distinct)


## 🆚 Difference from distinctUntilChanged

```ts
import { of } from 'rxjs';
import { distinct, distinctUntilChanged } from 'rxjs';

const values$ = of(1, 2, 1, 2, 3, 1, 2, 3);

// distinctUntilChanged: Remove only consecutive duplicates
values$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Output: 1, 2, 1, 2, 3, 1, 2, 3

// distinct: Remove all duplicates
values$.pipe(
  distinct()
).subscribe(console.log);
// Output: 1, 2, 3
```

| Operator | Removal Target | Use Case |
|---|---|---|
| `distinctUntilChanged` | Only consecutive duplicates | Input fields, sensor data |
| `distinct` | All duplicates | List of unique values, ID list |


## 🎯 Comparison Customization with keySelector

Use the `keySelector` function to determine duplicates for a specific property of an object.

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const users$ = of(
  { id: 1, name: 'Alice' } as User,
  { id: 2, name: 'Bob' } as User,
  { id: 1, name: 'Alice (updated)' } as User, // Same ID
  { id: 3, name: 'Charlie' } as User
);

users$.pipe(
  distinct(user => user.id) // Determine duplicates by ID
).subscribe(console.log);
// Output:
// { id: 1, name: 'Alice' }
// { id: 2, name: 'Bob' }
// { id: 3, name: 'Charlie' }
```


## 💡 Typical Usage Patterns

1. **Get List of Unique IDs**
   ```ts
   import { from } from 'rxjs';
   import { distinct, map } from 'rxjs';

   interface Order {
     orderId: string;
     userId: number;
     amount: number;
   }

   const orders$ = from([
     { orderId: 'A1', userId: 1, amount: 100 },
     { orderId: 'A2', userId: 2, amount: 200 },
     { orderId: 'A3', userId: 1, amount: 150 },
     { orderId: 'A4', userId: 3, amount: 300 }
   ] as Order[]);

   // Get only unique user IDs
   orders$.pipe(
     map(order => order.userId),
     distinct()
   ).subscribe(userId => {
     console.log(`User ID: ${userId}`);
   });
   // Output: 1, 2, 3
   ```

2. **Extract Unique Event Types from Event Log**
   ```ts
   import { fromEvent, merge } from 'rxjs';
   import { map, distinct, take } from 'rxjs';

   // Create UI elements dynamically
   const container = document.createElement('div');
   document.body.appendChild(container);

   const button1 = document.createElement('button');
   button1.textContent = 'Button 1';
   container.appendChild(button1);

   const button2 = document.createElement('button');
   button2.textContent = 'Button 2';
   container.appendChild(button2);

   const input = document.createElement('input');
   input.placeholder = 'Please input';
   container.appendChild(input);

   const log = document.createElement('div');
   log.style.marginTop = '10px';
   container.appendChild(log);

   // Merge multiple event streams to extract unique event types
   const events$ = merge(
     fromEvent(button1, 'click').pipe(map(() => 'button1-click')),
     fromEvent(button2, 'click').pipe(map(() => 'button2-click')),
     fromEvent(input, 'input').pipe(map(() => 'input-change'))
   );

   events$.pipe(
     distinct(),
     take(3) // Complete when all 3 types of events are present
   ).subscribe({
     next: (eventType) => {
       log.textContent += `Unique event: ${eventType}\n`;
       console.log(`Unique event: ${eventType}`);
     },
     complete: () => {
       log.textContent += 'All event types detected';
     }
   });
   ```


## 🧠 Practical Code Example (Tag Input)

Here is an example of a UI that automatically removes duplicates from tags entered by the user.

```ts
import { fromEvent, Subject } from 'rxjs';
import { map, distinct, scan } from 'rxjs';

// Create UI elements
const container = document.createElement('div');
document.body.appendChild(container);

const tagInput = document.createElement('input');
tagInput.type = 'text';
tagInput.placeholder = 'Enter tag and press Enter';
container.appendChild(tagInput);

const tagList = document.createElement('ul');
tagList.style.marginTop = '10px';
container.appendChild(tagList);

// Tag addition stream
const tagSubject$ = new Subject<string>();

tagSubject$.pipe(
  map(tag => tag.trim().toLowerCase()),
  distinct() // Remove duplicate tags
).subscribe(tag => {
  const li = document.createElement('li');
  li.textContent = tag;
  tagList.appendChild(li);
});

// Add a tag with the Enter key
fromEvent<KeyboardEvent>(tagInput, 'keydown').subscribe(event => {
  if (event.key === 'Enter') {
    const value = tagInput.value.trim();
    if (value) {
      tagSubject$.next(value);
      tagInput.value = '';
    }
  }
});
```

This code ensures that the same tag is added to the list only once, even if it is entered multiple times.


## ⚠️ Note on Memory Usage

> [!WARNING]
> The `distinct` operator uses **Set** internally to store all previously emitted values. Using it with an infinite stream may cause memory leaks.

### Problem: Memory Leak in Infinite Streams

```ts
import { interval } from 'rxjs';
import { distinct, map } from 'rxjs';

// ❌ Bad example: Using distinct with infinite streams
interval(100).pipe(
  map(n => n % 10), // 0-9 cycle
  distinct() // Only outputs the first 10, then keeps them in memory
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// Nothing is output after that, but the Set continues to be stored
```

### Solution: Clear Set with flushes Parameter

```ts
import { interval, timer } from 'rxjs';
import { distinct, map } from 'rxjs';

// ✅ Good example: Periodically clear Set
interval(100).pipe(
  map(n => n % 5),
  distinct(
    value => value,
    timer(1000) // Clear Set every 1 second
  )
).subscribe(console.log);
// Every 1 second, 0, 1, 2, 3, 4 are re-output
```

### Best Practices

1. **Use with finite streams**: HTTP responses, conversion from arrays, etc.
2. **Use flushes**: Clear periodically for infinite streams
3. **Consider distinctUntilChanged**: Use this to remove only consecutive duplicates


## 📋 Type-Safe Usage

Here is an example of a type-safe implementation utilizing generics in TypeScript.

```ts
import { Observable } from 'rxjs';
import { distinct, map } from 'rxjs';

interface Product {
  id: number;
  name: string;
  categoryId: number;
}

function getUniqueCategories(
  products$: Observable<Product>
): Observable<number> {
  return products$.pipe(
    distinct(product => product.categoryId)
  ).pipe(
    map(product => product.categoryId)
  );
}

// Example usage
import { of } from 'rxjs';

const products$ = of(
  { id: 1, name: 'Laptop', categoryId: 10 } as Product,
  { id: 2, name: 'Mouse', categoryId: 10 } as Product,
  { id: 3, name: 'Book', categoryId: 20 } as Product
);

getUniqueCategories(products$).subscribe(categoryId => {
  console.log(`Category ID: ${categoryId}`);
});
// Output: 10, 20
```


## 🎓 Summary

### When to Use distinct
- ✅ When you need a list of unique values
- ✅ When you want to remove duplicates in a finite stream
- ✅ Creating a list of IDs or categories

### When to Use distinctUntilChanged
- ✅ When you want to remove only consecutive duplicates
- ✅ Input field change detection
- ✅ When you want to save memory with infinite streams

### Notes
- ⚠️ Use `flushes` parameter for infinite streams to prevent memory leaks
- ⚠️ Be aware of memory usage when large numbers of unique values are streamed
- ⚠️ If performance is critical, monitor the size of the Set


## 🚀 Next Steps

- **[distinctUntilChanged](/en/guide/operators/filtering/distinctUntilChanged)** - Learn how to remove only consecutive duplicates
- **[distinctUntilKeyChanged](/en/guide/operators/filtering/distinctUntilKeyChanged)** - Learn how to compare objects by key
- **[filter](/en/guide/operators/filtering/filter)** - Learn how to filter based on conditions
- **[Filtering Operator Practical Examples](/en/guide/operators/filtering/practical-use-cases)** - Learn real use cases
