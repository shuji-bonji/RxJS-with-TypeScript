---
description: The debounceTime operator outputs the last value when no new value has been received for a specified time after emitting consecutive events. This is ideal for optimizing frequent input such as search box typing or window resizing events.
titleTemplate: ':title'
---

# debounceTime - Emit Last Value After Wait

The `debounceTime` operator outputs the last value after a value has been emitted in the stream if no new value has been emitted for the specified time.
It is very commonly used in situations where frequent events need to be suppressed, such as user-input search boxes.

## 🔰 Basic Syntax and Usage

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const searchBox = document.createElement('input');
document.body.appendChild(searchBox);

fromEvent(searchBox, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value),
    debounceTime(300)
  )
  .subscribe(console.log);
```

- If no further input is received within 300ms after an input event occurs, the value is emitted.
- This has the effect of consolidating events that occur consecutively in a short period of time.

[🌐 RxJS Official Documentation - `debounceTime`](https://rxjs.dev/api/operators/debounceTime)

## 💡 Typical Usage Patterns

- Send request after user finishes typing in search box
- Get final size for window resize event
- Obtain final position for scroll event

## 🧠 Practical Code Example (with UI)

When a character is entered into the search box, a search start message is displayed when the input stops for 300 ms.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

// Create output area
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Enter search word';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Input stream
fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300)
).subscribe(value => {
  resultArea.textContent = `Started searching for "${value}"`;
});
```

- No immediate response while inputting
- It will stop inputting and start searching with the latest input value 300ms later
