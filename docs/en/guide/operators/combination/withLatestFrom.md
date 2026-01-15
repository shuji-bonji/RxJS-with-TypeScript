---
description: "withLatestFrom operator combines the latest value from another stream each time the main Observable emits: Ideal for form validation and state synchronization"
titleTemplate: ':title'
---

# withLatestFrom - Combine with Latest

The `withLatestFrom` operator **every time a value in the main stream is emitted**,
combines the **latest value** from another stream and outputs it.


## 🔰 Basic Syntax and Usage

```ts
import { interval, fromEvent } from 'rxjs';
import { withLatestFrom, map, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
const timer$ = interval(1000);

clicks$
  .pipe(
    withLatestFrom(timer$),
    map(([click, timerValue]) => `Counter at click time: ${timerValue}`)
  )
  .subscribe(console.log);

// Output:
// Counter at click time: 1
// Counter at click time: 2
// Counter at click time: 2
// Counter at click time: 5

```

- The main Observable (in this case, clicks) acts as the trigger,
- The **latest value** of the sub Observable (in this case, counter) is combined and output each time.

[🌐 RxJS Official Documentation - `withLatestFrom`](https://rxjs.dev/api/index/function/withLatestFrom)


## 💡 Typical Usage Patterns

- **Get latest state at user action time**
- **Reference cached data at request time**
- **Event-triggered data binding**


## 🧠 Practical Code Example (with UI)

Example of retrieving and displaying the latest value of an input field every 2 seconds.

```ts
import { fromEvent, interval } from 'rxjs';
import { map, startWith, withLatestFrom } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'withLatestFrom: Get Latest Input Every 2 Seconds:';
document.body.appendChild(title);

// Create input field
const nameInput = document.createElement('input');
nameInput.placeholder = 'Enter name';
document.body.appendChild(nameInput);

// Create output area
const output = document.createElement('div');
document.body.appendChild(output);

// Input Observable
const name$ = fromEvent(nameInput, 'input').pipe(
  map((e) => (e.target as HTMLInputElement).value),
  startWith('') // Start with empty string
);

// Timer (fires every 2 seconds)
const timer$ = interval(2000);

// Get latest input value each time timer fires
timer$.pipe(withLatestFrom(name$)).subscribe(([_, name]) => {
  const item = document.createElement('div');
  item.textContent = `2-second retrieval: Name: ${name}`;
  output.prepend(item);
});

```

- While the user continues typing,
- **The latest input is retrieved and displayed** every 2 seconds.
