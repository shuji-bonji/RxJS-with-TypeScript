---
description: "exhaustAll operator ignores new internal Observables while one is running: Essential for preventing double-clicks, duplicate submissions, and button mashing"
---

# exhaustAll - Ignore New Internal Observables While Running

The `exhaustAll` operator takes a **Higher-order Observable** (Observable of Observables),
**ignores new internal Observables** if an internal Observable is running.

## 🔰 Basic Syntax and Usage

```ts
import { fromEvent, interval } from 'rxjs';
import { map, exhaustAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Start a new counter for each click (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Ignore new clicks if counter is running
higherOrder$
  .pipe(exhaustAll())
  .subscribe(x => console.log(x));

// Output (with 3 consecutive clicks):
// 0 (1st counter)
// 1 (1st counter)
// ← Click here (ignored: 1st is running)
// 2 (1st counter) ← Complete
// ← Click here (accepted: no running counter)
// 0 (2nd counter)
// 1 (2nd counter)
// 2 (2nd counter)
```

- If internal Observable is running, **new internal Observables are ignored**
- **Accepts next one after** running Observable completes
- Ideal for preventing double execution

[🌐 RxJS Official Documentation - `exhaustAll`](https://rxjs.dev/api/index/function/exhaustAll)

## 💡 Typical Usage Patterns

- **Double-click prevention (prevent button mashing)**
- **Prevent duplicate login requests**
- **Prevent duplicate save operations**

## 🧠 Practical Code Example

Example of preventing double-clicks on save button

```ts
import { fromEvent, of } from 'rxjs';
import { map, exhaustAll, delay } from 'rxjs';

const saveButton = document.createElement('button');
saveButton.textContent = 'Save';
document.body.appendChild(saveButton);

const output = document.createElement('div');
document.body.appendChild(output);

let saveCount = 0;

// Button click event
const clicks$ = fromEvent(saveButton, 'click');

// Higher-order Observable: Simulated save operation for each click
const saves$ = clicks$.pipe(
  map(() => {
    const id = ++saveCount;
    const start = Date.now();

    // Temporarily disable button (visual feedback)
    saveButton.disabled = true;

    // Simulated save operation (2 second delay)
    return of(`Save completed #${id}`).pipe(
      delay(2000),
      map(msg => {
        saveButton.disabled = false;
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed} seconds)`;
      })
    );
  }),
  exhaustAll() // Ignore new clicks while saving
);

saves$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});

// Log ignored clicks
clicks$.subscribe(() => {
  if (saveButton.disabled) {
    console.log('Click ignored during save operation');
  }
});
```

- **New clicks are ignored** during save operation
- Next click is accepted after save completes

## 🔄 Related Operators

| Operator | Description |
|---|---|
| `exhaustMap` | Shorthand for `map` + `exhaustAll` (commonly used) |
| [mergeAll](/en/guide/operators/combination/mergeAll) | Subscribe to all internal Observables in parallel |
| [concatAll](/en/guide/operators/combination/concatAll) | Subscribe to internal Observables in order (queue them) |
| [switchAll](/en/guide/operators/combination/switchAll) | Switch to new internal Observable (cancel old one) |

## 🔄 Comparison with Other Operators

| Operator | When New Internal Observable is Emitted |
|---|---|
| `mergeAll` | Execute concurrently |
| `concatAll` | Add to queue (wait for previous completion) |
| `switchAll` | Cancel old one and switch |
| `exhaustAll` | **Ignore (wait for running completion)** |

## ⚠️ Important Notes

### Event Loss

`exhaustAll` **completely ignores** running events, so it's inappropriate if you want to process all events.

```ts
// ❌ exhaustAll is inappropriate if you want to record all clicks
// ✅ Use mergeAll or concatAll
```

### UI Feedback

It's important to visually tell users that events are being "ignored".

```ts
// Disable button
saveButton.disabled = true;

// Show toast message
showToast('Processing. Please wait a moment.');
```

### Appropriate Use Cases

#### `exhaustAll` is Optimal for:
- Login operations (prevent duplicate submissions)
- Save operations (prevent duplicate execution)
- Animations (don't start new animation while running)

#### `exhaustAll` is Not Appropriate for:
- Search operations (want to execute latest search → `switchAll`)
- All events should be processed (→ `mergeAll` or `concatAll`)
