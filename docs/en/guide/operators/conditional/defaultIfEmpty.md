---
description: The defaultIfEmpty operator is an operator to return a default value if Observable does not issue a value, useful for empty data handling and initial value completion.
---

# defaultIfEmpty - Default Value if Stream is Empty

The `defaultIfEmpty` operator is an **operator that issues a specified default value if Observable is completed without issuing any value**.
It is used to deal with empty arrays or empty API results.

## 🔰 Basic Syntax and Operation

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

from([]).pipe(
  defaultIfEmpty('No value')
).subscribe(console.log);

// Output:
// No value
```

In this example, `defaultIfEmpty` will output `'No value'` for an empty array made Observable with `from`.

[🌐 RxJS Official Documentation - defaultIfEmpty](https://rxjs.dev/api/index/function/defaultIfEmpty)

## 💡 Typical Usage Examples

- If the user did not enter any information
- When the API returns an empty result
- If none of the values satisfy the conditions

This is used to **complete the "nothing returned" situation** in cases such as these.

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of(['A', 'B', 'C']).pipe(delay(500))
    : EMPTY.pipe(delay(500));
}

mockApiCall(false)
  .pipe(defaultIfEmpty('No data'))
  .subscribe(console.log);

// Output:
// No data
```

## 🧪 Practical Code Examples (with UI)

### ✅ 1. Used to Determine if an Array is Empty

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

// Build UI
const container = document.createElement('div');
container.innerHTML = '<h3>defaultIfEmpty operator example:</h3>';
document.body.appendChild(container);

const emptyBtn = document.createElement('button');
emptyBtn.textContent = 'Process empty array';
container.appendChild(emptyBtn);

const nonEmptyBtn = document.createElement('button');
nonEmptyBtn.textContent = 'Process non-empty array';
container.appendChild(nonEmptyBtn);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

emptyBtn.addEventListener('click', () => {
  result.textContent = 'Processing...';
  from([]).pipe(
    defaultIfEmpty('No data')
  ).subscribe(value => {
    result.textContent = `Result: ${value}`;
  });
});

nonEmptyBtn.addEventListener('click', () => {
  result.textContent = 'Processing...';
  from([1, 2, 3]).pipe(
    defaultIfEmpty('No data')
  ).subscribe(value => {
    result.textContent = `Result: ${value}`;
  });
});
```

### ✅ 2. Complete Default for Empty Result in API

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of([
        { id: 1, name: 'Item 1' },
        { id: 2, name: 'Item 2' },
      ]).pipe(delay(1000))
    : EMPTY.pipe(delay(1000));
}

const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>API result processing with defaultIfEmpty:</h3>';
document.body.appendChild(apiContainer);

const dataBtn = document.createElement('button');
dataBtn.textContent = 'With data';
dataBtn.style.marginRight = '10px';
apiContainer.appendChild(dataBtn);

const emptyBtn2 = document.createElement('button');
emptyBtn2.textContent = 'Without data';
apiContainer.appendChild(emptyBtn2);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
apiContainer.appendChild(output);

dataBtn.addEventListener('click', () => {
  output.textContent = 'Retrieving...';
  mockApiCall(true)
    .pipe(defaultIfEmpty('No data found'))
    .subscribe({
      next: (val) => {
        if (Array.isArray(val)) {
          const ul = document.createElement('ul');
          val.forEach((item) => {
            const li = document.createElement('li');
            li.textContent = `${item.id}: ${item.name}`;
            ul.appendChild(li);
          });
          output.innerHTML = '<h4>Result:</h4>';
          output.appendChild(ul);
        } else {
          output.textContent = val;
        }
      },
    });
});

emptyBtn2.addEventListener('click', () => {
  output.textContent = 'Retrieving...';
  mockApiCall(false)
    .pipe(defaultIfEmpty('No data found'))
    .subscribe({
      next: (val) => {
        output.textContent = val.toString();
      },
    });
});
```
