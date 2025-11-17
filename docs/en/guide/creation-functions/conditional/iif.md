---
description: The iif operator is a RxJS conditional branch operator that selects one of two Observables depending on a conditional expression, and can be used like a ternary operator.
titleTemplate: ':title | RxJS'
---

# iif - Selection of Observable based on condition

The `iif` operator selects one of two Observables based on the result of evaluating a conditional expression.
The JavaScript ternary operator (`condition ? trueValue : falseValue`).


## Basic syntax and operation

```ts
import { iif, of } from 'rxjs';

function getData(condition: boolean) {
  return iif(() => condition, of('YES'), of('NO'));
}

getData(true).subscribe(console.log);

// Output:
// YES
```

Returns `'YES'` if the condition is `true`, `'NO'` if the condition is `false`.

[🌐 RxJS Official Documentation - iif](https://rxjs.dev/api/index/function/iif)

## Typical Application Examples

`iif` is often used in combination with `EMPTY` to return a "no-issue stream" if the condition is not met.

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(
    () => value > 0,
    of(`Positive value: ${value}`),
    EMPTY
  );
}

conditionalData(0).subscribe(console.log);
conditionalData(1).subscribe(console.log);

// Output:
// Positive value: 1
```


## Practical code examples (with UI)

The following code example with UI uses `iif` to dynamically switch what to publish and whether or not to publish an Observable in response to user actions and numerical input.
The following code example with UI uses `iif` to dynamically switch what is issued or not issued by Observable according to user operation or numerical input.

Such a pattern is suitable for the following practical use cases.

- ✅ Suppress API requests based on input values (e.g., do not send if the number is less than 0)
- ✅ Switch screen display and processing mode according to configuration flags
- ✅ Acknowledgement and modal control based on conditions

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(() => value > 0, of(`Positive value: ${value}`), EMPTY);
}

// Return different Observables based on conditions
function getDataBasedOnCondition(condition: boolean) {
  return iif(() => condition, of('Condition is true'), of('Condition is false'));
}

// Create UI elements
const iifContainer = document.createElement('div');
iifContainer.innerHTML = '<h3>iif operator example:</h3>';
document.body.appendChild(iifContainer);

const trueButton = document.createElement('button');
trueButton.textContent = 'Execute with True condition';
trueButton.style.marginRight = '10px';
iifContainer.appendChild(trueButton);

const falseButton = document.createElement('button');
falseButton.textContent = 'Execute with False condition';
iifContainer.appendChild(falseButton);

const iifResult = document.createElement('div');
iifResult.style.marginTop = '10px';
iifResult.style.padding = '10px';
iifResult.style.border = '1px solid #ddd';
iifContainer.appendChild(iifResult);

trueButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(true).subscribe((result) => {
    iifResult.textContent = result;
    iifResult.style.color = 'green';
  });
});

falseButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(false).subscribe((result) => {
    iifResult.textContent = result;
    iifResult.style.color = 'red';
  });
});

// Example combining with EMPTY (conditional branching by number)
const emptyContainer = document.createElement('div');
emptyContainer.innerHTML = '<h3>Combination of iif and EMPTY:</h3>';
document.body.appendChild(emptyContainer);

const valueInput = document.createElement('input');
valueInput.type = 'number';
valueInput.placeholder = 'Enter a number';
valueInput.style.marginRight = '10px';
emptyContainer.appendChild(valueInput);

const checkButton = document.createElement('button');
checkButton.textContent = 'Execute';
emptyContainer.appendChild(checkButton);

const emptyResult = document.createElement('div');
emptyResult.style.marginTop = '10px';
emptyResult.style.padding = '10px';
emptyResult.style.border = '1px solid #ddd';
emptyContainer.appendChild(emptyResult);

checkButton.addEventListener('click', () => {
  const value = Number(valueInput.value);
  emptyResult.textContent = '';

  conditionalData(value).subscribe({
    next: (result) => {
      emptyResult.textContent = result;
      emptyResult.style.color = 'green';
    },
    complete: () => {
      if (!emptyResult.textContent) {
        emptyResult.textContent =
          'A value of 0 or less was entered, so nothing was issued';
        emptyResult.style.color = 'gray';
      }
    },
  });
});
```
