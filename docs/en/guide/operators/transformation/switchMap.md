---
description: switchMap is a conversion operator that cancels the previous Observable and switches to the latest one. It is ideal for use cases such as live search, navigation switching, auto-save, etc. Together with TypeScript type inference, it enables safe asynchronous processing. It also provides detailed explanations on how to use it in conjunction with mergeMap and concatMap.
---

# switchMap - Cancel Previous Observable and Switch to the Latest One

The `switchMap` operator creates a new Observable for each value in the input stream, **canceling the previous Observable and switching only to the most recent one**.
This is ideal for cases where only the most recent input is to be valid, such as in a search form.

## 🔰 Basic Syntax and Usage

```ts
import { of } from 'rxjs';
import { delay, switchMap } from 'rxjs';

of('A', 'B', 'C').pipe(
  switchMap(value =>
    of(`${value} completed`).pipe(delay(1000))
  )
).subscribe(console.log);

// Output example:
// C completed
```

- Create a new Observable for each value.
- However, **the moment a new value comes in, the previous Observable is canceled**.
- Only `C` will be output in the end.

[🌐 RxJS Official Documentation - `switchMap`](https://rxjs.dev/api/operators/switchMap)

## 💡 Typical Usage Patterns

- Autocompletion of input forms
- Live search function (only the latest input is valid)
- Resource loading when switching navigation or routing
- Switching user actions to the latest one

## 🧠 Practical Code Example (with UI)

When a user enters text in the search box, an API request is immediately sent, displaying **results of only the last entry**.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

// Create input field
const searchInput = document.createElement('input');
searchInput.placeholder = 'Search by username';
document.body.appendChild(searchInput);

// Output area
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Input event processing
fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  map(event => (event.target as HTMLInputElement).value.trim()),
  switchMap(term => {
    if (term === '') {
      return of([]);
    }
    return ajax.getJSON(`https://jsonplaceholder.typicode.com/users?username_like=${term}`);
  })
).subscribe(users => {
  output.innerHTML = '';

  (users as any[]).forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.username;
    output.appendChild(div);
  });
});
```

- Each time the input changes, the previous request is canceled.
- Only users matching the most recent search term will be displayed.
