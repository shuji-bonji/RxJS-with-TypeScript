---
description: The distinctUntilChanged operator enables efficient data processing by skipping consecutive values that are the same as the previous one and outputting only the values that have changed.
---

# distinctUntilChanged - Remove Consecutive Duplicate Values

The `distinctUntilChanged` operator removes duplicates when the same value is emitted consecutively, and only outputs the new value if it differs from the previous value.


## 🔰 Basic Syntax and Usage

```ts
import { from } from 'rxjs';
import { distinctUntilChanged } from 'rxjs';

const numbers$ = from([1, 1, 2, 2, 3, 1, 2, 3]);

numbers$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Output: 1, 2, 3, 1, 2, 3
```

- If the value is the same as the previous one, it is ignored.
- This is not a batch process like `Array.prototype.filter`, but rather a **sequential decision**.

[🌐 RxJS Official Documentation - `distinctUntilChanged`](https://rxjs.dev/api/operators/distinctUntilChanged)


## 💡 Typical Usage Patterns

- Form input detection to prevent wasted requests for consecutive same input values
- Detecting changes in sensors and event streams
- Prevent unnecessary UI redraws in state management


## 🧠 Practical Code Example (with UI)

Simulation of sending an API request in a search box **only if the entered string differs from the previous one**.

```ts
import { fromEvent } from 'rxjs';
import { map, distinctUntilChanged } from 'rxjs';

// Create output area
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Enter search keywords';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Input stream
fromEvent(searchInput, 'input')
  .pipe(
    distinctUntilChanged(),
    map((event) => (event.target as HTMLInputElement).value.trim())
  )
  .subscribe((keyword) => {
    resultArea.textContent = `Execute with search value: ${keyword}`;
  });

```

- If the input text does not change, it will not be requested.
- This can be used for efficient search processing and API communication optimization.
