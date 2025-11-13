---
description: The isEmpty operator determines whether Observable has completed without emitting a value and is used for empty data detection and conditional branching.
---

# isEmpty - Determine if Stream is Empty

The `isEmpty` operator **emits `true` if Observable completes without emitting any value**.
If it emits even one value, it emits `false` and completes.

## 🔰 Basic Syntax and Operation

```ts
import { of, EMPTY } from 'rxjs';
import { isEmpty } from 'rxjs';

EMPTY.pipe(isEmpty()).subscribe(console.log); // Output: true
of(1).pipe(isEmpty()).subscribe(console.log); // Output: false
```

[🌐 RxJS Official Documentation - isEmpty](https://rxjs.dev/api/index/function/isEmpty)

## 💡 Typical Usage Examples

- When you want to determine whether the filtering result or search result is empty
- When you want to issue an error or switch to another process if it is empty

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

from([1, 3, 5])
  .pipe(
    filter((x) => x % 2 === 0),
    isEmpty()
  )
  .subscribe((result) => {
    console.log('Is empty:', result);
  });

// Output:
// Is empty: true
```

## 🧪 Practical Code Examples (with UI)

### ✅ 1. Determine if Result is Empty

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>isEmpty operator example:</h3>';
document.body.appendChild(container);

const checkButton = document.createElement('button');
checkButton.textContent = 'Check if contains even numbers';
container.appendChild(checkButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
container.appendChild(output);

checkButton.addEventListener('click', () => {
  from([1, 3, 5])
    .pipe(
      filter((x) => x % 2 === 0),
      isEmpty()
    )
    .subscribe((isEmptyResult) => {
      output.textContent = isEmptyResult
        ? 'No even numbers found.'
        : 'Even numbers are included.';
      output.style.color = isEmptyResult ? 'red' : 'green';
    });
});
```

### ✅ 2. Check if User Search Results are Empty

```ts
import { fromEvent, of, from } from 'rxjs';
import { debounceTime, switchMap, map, filter, isEmpty, delay } from 'rxjs';

const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Search result check with isEmpty:</h3>';
document.body.appendChild(searchContainer);

const input = document.createElement('input');
input.placeholder = 'Enter search word';
input.style.marginBottom = '10px';
searchContainer.appendChild(input);

const resultBox = document.createElement('div');
resultBox.style.padding = '10px';
resultBox.style.border = '1px solid #ccc';
searchContainer.appendChild(resultBox);

const mockData = ['apple', 'banana', 'orange', 'grape'];

fromEvent(input, 'input')
  .pipe(
    debounceTime(300),
    map((e) => (e.target as HTMLInputElement).value.trim().toLowerCase()),
    filter((text) => text.length > 0),
    switchMap((query) =>
      of(mockData).pipe(
        delay(300),
        map((list) => list.filter((item) => item.includes(query))),
        switchMap((filtered) => from(filtered).pipe(isEmpty()))
      )
    )
  )
  .subscribe((noResults) => {
    resultBox.textContent = noResults
      ? 'No matching items found'
      : 'Matching items found';
    resultBox.style.color = noResults ? 'red' : 'green';
  });
```
