---
description: find is an RxJS filtering operator that finds and outputs the first value that satisfies a condition and immediately completes the stream. It is ideal for scenarios where you want to search for a specific element from an array or list, such as user search, inventory checking, and error log detection. If no value is found, it outputs undefined, and in TypeScript the return value is of type T | undefined.
titleTemplate: ':title | RxJS'
---

# find - Find the First Value That Satisfies a Condition

The `find` operator finds and outputs the **first value that satisfies a condition** and immediately completes the stream. If no value is found, it outputs `undefined`.


## 🔰 Basic Syntax and Usage

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Output: 8 (first even number)
```

**Flow of operation**:
1. Check 1, 3, 5, 7 → Do not satisfy condition
2. Check 8 → Satisfies condition → Output 8 and complete
3. 9, 10 are not evaluated

[🌐 RxJS Official Documentation - `find`](https://rxjs.dev/api/operators/find)


## 🆚 Contrast with first

`find` and `first` are similar but used differently.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: First value that satisfies condition (condition is optional)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Output: 7

// find: First value that satisfies condition (condition is required)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Output: 7
```

| Operator | Condition Specification | When Value Not Found | Use Case |
|---|---|---|---|
| `first()` | Optional | Error (`EmptyError`) | Get first value |
| `first(predicate)` | Optional | Error (`EmptyError`) | Conditional get |
| `find(predicate)` | Required | Output `undefined` | Search/existence check |


## 💡 Typical Usage Patterns

1. **User Search**
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

   // Search for user with ID 2
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

2. **Inventory Check**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface Product {
     id: string;
     name: string;
     stock: number;
   }

   const products$ = from([
     { id: 'A1', name: 'Laptop', stock: 0 },
     { id: 'A2', name: 'Mouse', stock: 15 },
     { id: 'A3', name: 'Keyboard', stock: 8 }
   ] as Product[]);

   // Find out of stock product
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`Out of stock: ${product.name}`);
     } else {
       console.log('All in stock');
     }
   });
   // Output: Out of stock: Laptop
   ```


## 🎯 Difference from filter

`find` and `filter` are used for different purposes.

```ts
import { from } from 'rxjs';
import { find, filter } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// filter: Output all values that match condition
numbers$.pipe(
  filter(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter complete')
});
// Output: 7, 8, 9, 10, filter complete

// find: Output only first value that matches condition
numbers$.pipe(
  find(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('find complete')
});
// Output: 7, find complete
```

| Operator | Output Count | Completion Timing | Use Case |
|---|---|---|---|
| `filter(predicate)` | All values that match condition | When original stream completes | Data filtering |
| `find(predicate)` | Only first value that matches condition | Immediately when found | Search/existence check |


## ⚠️ Common Mistakes

> [!NOTE]
> `find` outputs `undefined` when value is not found. It does not error. Use `first` if you need an error.

### Wrong: Expecting Error Handling When Value Not Found

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ❌ Bad example: Expecting error handling but not called
numbers$.pipe(
  find(n => n > 10)
).subscribe({
  next: console.log,
  error: err => console.log('Error:', err) // Not called
});
// Output: undefined
```

### Correct: Check undefined or Use first

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ✅ Good example 1: Check undefined
numbers$.pipe(
  find(n => n > 10)
).subscribe(result => {
  if (result !== undefined) {
    console.log('Found:', result);
  } else {
    console.log('Not found');
  }
});
// Output: Not found

// ✅ Good example 2: Use first if error is needed
numbers$.pipe(
  first(n => n > 10, 0) // Specify default value
).subscribe({
  next: console.log,
  error: err => console.log('Error:', err.message)
});
// Output: 0
```


## 🎓 Summary

### When to Use find
- ✅ When you want to search for the first value that satisfies a condition
- ✅ When you want to check for value existence
- ✅ When you want to handle not found case with `undefined`
- ✅ When you want to search for a specific element from an array or list

### When to Use first
- ✅ When you want to get the first value
- ✅ When you want to emit an error if value is not found

### When to Use filter
- ✅ When you need all values that match the condition
- ✅ When the purpose is data filtering

### Notes
- ⚠️ `find` outputs `undefined` when not found (not an error)
- ⚠️ Completes immediately with first value that satisfies condition
- ⚠️ In TypeScript, return value is of type `T | undefined`


## 🚀 Next Steps

- **[first](/en/guide/operators/filtering/first)** - Learn how to get the first value
- **[filter](/en/guide/operators/filtering/filter)** - Learn how to filter based on conditions
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - Learn how to get the index of the first value that satisfies condition (official documentation)
- **[Filtering Operator Practical Examples](/en/guide/operators/filtering/practical-use-cases)** - Learn real use cases
