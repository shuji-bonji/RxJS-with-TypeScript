---
description: "combineLatest combines the latest values of multiple Observables: Essential for real-time form validation, state synchronization, and dependent data"
---

# combineLatest - combine the latest values

`combineLatest` is a Creation Function that **combines all the latest values from multiple Observables**.
Whenever a new value is issued from any of the source Observables, the result of all the latest values are combined.

## Basic syntax and usage

```ts
import { combineLatest, of } from 'rxjs';

const obs1$ = of('A', 'B', 'C');
const obs2$ = of(1, 2, 3);

combineLatest([obs1$, obs2$]).subscribe(([val1, val2]) => {
  console.log(val1, val2);
});

// Output:
// C 1
// C 2
// C 3
```

- After each Observable has issued **at least one value**, the combined value is output.
- Whenever a new value comes in for either one, the most recent pair is re-output.

[🌐 RxJS Official Documentation - `combineLatest`](https://rxjs.dev/api/index/function/combineLatest)


## Typical utilization patterns

- **Real-time validation of form inputs** (e.g., simultaneous monitoring of name and email address)
- **State synchronization of multiple streams** (e.g., integration of sensor values and device status)
- **Data fetching with dependencies** (e.g., combination of user ID and configuration ID)

## Practical code examples (with UI)

Always combine and display the latest status of the two input fields of a form.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>combineLatest practical example:</h3>';
document.body.appendChild(output);

// Create form fields
const nameInput = document.createElement('input');
nameInput.placeholder = 'Enter name';
document.body.appendChild(nameInput);

const emailInput = document.createElement('input');
emailInput.placeholder = 'Enter email';
document.body.appendChild(emailInput);

// Observable of each input
const name$ = fromEvent(nameInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

const email$ = fromEvent(emailInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

// Combine the latest input values
combineLatest([name$, email$]).subscribe(([name, email]) => {
  output.innerHTML = `
    <div><strong>Name:</strong> ${name}</div>
    <div><strong>Email:</strong> ${email}</div>
  `;
});
```

- When you type in either field, the **latest two input states** are immediately displayed.
- The `startWith('')` is used to get the combined result from the beginning.


## Related Operators

- **[combineLatestWith](/en/guide/operators/combination/combineLatestWith)** - Pipeable Operator version (used in pipeline)
- **[withLatestFrom](/en/guide/operators/combination/withLatestFrom)** - triggers only the main stream
- **[zip](/en/guide/creation-functions/combination/zip)** - pairs corresponding values Creation Function
