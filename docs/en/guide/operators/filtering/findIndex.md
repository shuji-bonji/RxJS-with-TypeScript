---
description: The findIndex operator is an RxJS filtering operator that returns the index of the first value that satisfies the condition. If not found, it returns -1.
---

# findIndex - Get Index of First Value Matching Condition

The `findIndex` operator returns **the index of the first value that satisfies the condition** and immediately completes the stream. If the value is not found, it returns `-1`.

## 🔰 Basic Syntax and Usage

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Output: 4 (index of first even number 8)
```

**Flow of operation**:
1. 1 (index 0) → Odd, skip
2. 3 (index 1) → Odd, skip
3. 5 (index 2) → Odd, skip
4. 7 (index 3) → Odd, skip
5. 8 (index 4) → Even, output index 4 and complete

[🌐 RxJS Official Documentation - `findIndex`](https://rxjs.dev/api/operators/findIndex)

## 💡 Typical Usage Patterns

- **Locate position in array**: Get position of element matching specific condition
- **Check order**: Determine at what position an element matching a condition appears
- **Data sorting**: Processing using index information
- **Existence check**: Check existence by whether it's -1 or not

## 🆚 Comparison with Similar Operators

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Return index of first value matching condition
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Output: 2 (index of 30)

// find: Return first value matching condition
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Output: 30

// elementAt: Return value at specified index
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Output: 30
```

| Operator | Argument | Return Value | When Not Found |
|:---|:---|:---|:---|
| `findIndex(predicate)` | Condition function | Index (number) | `-1` |
| `find(predicate)` | Condition function | Value itself | `undefined` |
| `elementAt(index)` | Index | Value itself | Error (no default value) |

## ⚠️ Important Notes

### 1. Returns -1 if Not Found

If no value satisfies the condition, returns `-1` instead of an error.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('No value matching condition found');
  } else {
    console.log(`Index: ${index}`);
  }
});
// Output: No value matching condition found
```

### 2. Completes When First Match Found

Stream completes immediately upon finding the first value matching the condition.

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

### 3. Type Safety in TypeScript

`findIndex` always returns a `number` type.

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
  // index is number type
  if (index !== -1) {
    console.log(`First inactive user is at index ${index}`);
  }
});
// Output: First inactive user is at index 1
```

### 4. Indexes Start at 0

Like arrays, indexes start at 0.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// Output: 0 (first element)
```

## 📚 Related Operators

- **[find](/en/guide/operators/filtering/find)** - Get first value matching condition
- **[elementAt](/en/guide/operators/filtering/elementAt)** - Get value at specified index
- **[first](/en/guide/operators/filtering/first)** - Get first value
- **[filter](/en/guide/operators/filtering/filter)** - Get all values matching condition

## Summary

The `findIndex` operator returns the index of the first value matching the condition.

- ✅ Similar behavior to JavaScript's `Array.findIndex()`
- ✅ Ideal when index information is needed
- ✅ Returns `-1` if not found (not an error)
- ✅ Completes immediately when found
- ⚠️ Return value is always `number` type (-1 or integer ≥ 0)
- ⚠️ Use `find` if you need the value itself
