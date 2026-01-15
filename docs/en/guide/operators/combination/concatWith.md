---
description: concatWith is an RxJS combination operator that joins other Observables in sequence after completion of the original Observable. It is ideal for sequential processing within a pipeline, follow-up processing after completion, staged data loading, and other situations where you want to add subsequent processing as an extension of the main stream. The pipeable operator version is convenient for use within a pipeline.
titleTemplate: ':title'
---

# concatWith - Sequential Stream Concat

The `concatWith` operator **sequentially concatenates** the specified other Observables after the original Observable `completes`.
This is the Pipeable Operator version of the Creation Function `concat`.

## 🔰 Basic Syntax and Usage

```ts
import { of, delay } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));
const obs3$ = of('E', 'F').pipe(delay(100));

obs1$
  .pipe(concatWith(obs2$, obs3$))
  .subscribe(console.log);

// Output: A → B → C → D → E → F
```

- After `obs1$` completes, `obs2$` starts, and after `obs2$` completes, `obs3$` starts.
- Can be used within `.pipe()` chains, making it easy to combine with other operators.

[🌐 RxJS Official Documentation - `concatWith`](https://rxjs.dev/api/operators/concatWith)


## 💡 Typical Usage Patterns

- **Sequential processing within a pipeline**: Combine additional data in sequence to the transformed stream
- **Follow-up processing after completion**: Add cleanup and notifications after main processing completes
- **Staged data loading**: Acquire additional data sequentially after initial data acquisition


## 🧠 Practical Code Example (with UI)

Example of displaying related recommended items in order after displaying main search results.

```ts
import { of, delay } from 'rxjs';
import { concatWith, map } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>concatWith Practical Example:</h3>';
document.body.appendChild(output);

// Main search results
const searchResults$ = of('🔍 Search Result 1', '🔍 Search Result 2', '🔍 Search Result 3').pipe(
  delay(500)
);

// Recommended items 1
const recommendations1$ = of('💡 Recommended Product A', '💡 Recommended Product B').pipe(
  delay(300)
);

// Recommended items 2
const recommendations2$ = of('⭐ Popular Product X', '⭐ Popular Product Y').pipe(
  delay(300)
);

// Combine in sequence and display
searchResults$
  .pipe(
    concatWith(recommendations1$, recommendations2$),
    map((value, index) => `${index + 1}. ${value}`)
  )
  .subscribe((value) => {
    const item = document.createElement('div');
    item.textContent = value;
    output.appendChild(item);
  });
```

- Search results are displayed first,
- Then recommended products are displayed in order.
- Can be used in combination with other operators like `map` within the pipeline.


## 🔄 Difference from Creation Function `concat`

### Basic Differences

| | `concat` (Creation Function) | `concatWith` (Pipeable Operator) |
|:---|:---|:---|
| **Usage Location** | Used as independent function | Used within `.pipe()` chain |
| **Syntax** | `concat(obs1$, obs2$, obs3$)` | `obs1$.pipe(concatWith(obs2$, obs3$))` |
| **First Stream** | Treats all equally | Treats as main stream |
| **Advantage** | Simple and readable | Easy to combine with other operators |

### Specific Usage Examples

**Creation Function is Recommended for Simple Combination Only**

```ts
import { concat, of } from 'rxjs';

const part1$ = of('A', 'B');
const part2$ = of('C', 'D');
const part3$ = of('E', 'F');

// Simple and readable
concat(part1$, part2$, part3$).subscribe(console.log);
// Output: A → B → C → D → E → F
```

**Pipeable Operator is Recommended if Transformation is Needed**

```ts
import { of } from 'rxjs';
import { concatWith, map, filter } from 'rxjs';

const userData$ = of({ name: 'Alice', age: 30 }, { name: 'Bob', age: 25 });
const additionalData$ = of({ name: 'Charlie', age: 35 });

// ❌ Creation Function version - becomes verbose
import { concat } from 'rxjs';
concat(
  userData$.pipe(
    filter(user => user.age >= 30),
    map(user => user.name)
  ),
  additionalData$.pipe(map(user => user.name))
).subscribe(console.log);

// ✅ Pipeable Operator version - completed in one pipeline
userData$
  .pipe(
    filter(user => user.age >= 30),  // 30 years or older only
    map(user => user.name),          // Extract name only
    concatWith(
      additionalData$.pipe(map(user => user.name))
    )
  )
  .subscribe(console.log);
// Output: Alice → Charlie
```

**When Adding Subsequent Processing to Main Stream**

```ts
import { fromEvent, of } from 'rxjs';
import { concatWith, take, mapTo } from 'rxjs';

// Create button and output area
const button = document.createElement('button');
button.textContent = 'Click 3 times';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

const clicks$ = fromEvent(button, 'click');

// ✅ Pipeable Operator version - natural as extension of main stream
clicks$
  .pipe(
    take(3),                          // Get first 3 clicks
    mapTo('Clicked'),
    concatWith(of('Completed'))       // Add message after completion
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = message;
    output.appendChild(div);
  });

// Writing the same behavior with Creation Function version...
// ❌ Creation Function version - needs to separate main stream
import { concat } from 'rxjs';
concat(
  clicks$.pipe(
    take(3),
    mapTo('Clicked')
  ),
  of('Completed')
).subscribe(console.log);
```

### Summary

- **`concat`**: Optimal for simply combining multiple streams
- **`concatWith`**: Optimal when you want to add subsequent processing to the main stream while transforming or processing it


## ⚠️ Important Notes

### Delay Due to Waiting for Completion

The next Observable will not start until the original Observable completes.

```ts
import { interval, of } from 'rxjs';
import { concatWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),              // Complete with 3 values
  concatWith(of('Complete'))
).subscribe(console.log);
// Output: 0 → 1 → 2 → Complete
```

### Error Handling

If an error occurs in the previous Observable, subsequent Observables will not execute.

```ts
import { throwError, of } from 'rxjs';
import { concatWith, catchError } from 'rxjs';

throwError(() => new Error('Error occurred'))
  .pipe(
    catchError(err => of('Error recovered')),
    concatWith(of('Next process'))
  )
  .subscribe(console.log);
// Output: Error recovered → Next process
```


## 📚 Related Operators

- **[concat](/en/guide/creation-functions/combination/concat)** - Creation Function version
- **[mergeWith](/en/guide/operators/combination/mergeWith)** - Pipeable version for parallel combination
- **[concatMap](/en/guide/operators/transformation/concatMap)** - Map each value sequentially
