---
description: The every operator evaluates whether all values meet a specified condition and can perform short-circuit evaluation by returning false as soon as the first value that does not meet the condition is encountered.
---

# every - Check if All Values Meet Condition

The `every` operator evaluates whether all values emitted from the source Observable meet a specified condition, and **returns `false` and terminates as soon as the first value that does not meet the condition is encountered**. If all values meet the condition, `true` is returned.

## 🔰 Basic Syntax and Operation

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 6, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// Output: true
```

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 5, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// Output: false (stops at 5)
```

[🌐 RxJS Official Documentation - every](https://rxjs.dev/api/index/function/every)

## 💡 Typical Usage Examples

- **Validation check**: Confirm that all conditions are met
- **Bulk input validation**: Evaluate multiple values together
- **Different from array filter, useful for checking overall satisfaction at once**

## 🧪 Practical Code Examples (with UI)

### ✅ 1. Determine if Array is All Even Numbers

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>every operator example:</h3>';
document.body.appendChild(container);

const allEvenButton = document.createElement('button');
allEvenButton.textContent = 'Only even [2, 4, 6, 8]';
container.appendChild(allEvenButton);

const someOddButton = document.createElement('button');
someOddButton.textContent = 'Contains odd [2, 4, 5, 8]';
container.appendChild(someOddButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

allEvenButton.addEventListener('click', () => {
  result.textContent = 'Checking...';
  from([2, 4, 6, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `All even?: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});

someOddButton.addEventListener('click', () => {
  result.textContent = 'Checking...';
  from([2, 4, 5, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `All even?: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});
```

### ✅ 2. Utilizing in Form Validation

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith, every, tap } from 'rxjs';

// Create UI elements
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>Form validation with every:</h3>';
document.body.appendChild(formContainer);

// Create form
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formContainer.appendChild(form);

// Name input
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Name: ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.width = '100%';
nameInput.style.padding = '5px';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

// Age input
const ageLabel = document.createElement('label');
ageLabel.textContent = 'Age: ';
ageLabel.style.display = 'block';
ageLabel.style.marginBottom = '5px';
form.appendChild(ageLabel);

const ageInput = document.createElement('input');
ageInput.type = 'number';
ageInput.min = '0';
ageInput.style.width = '100%';
ageInput.style.padding = '5px';
ageInput.style.marginBottom = '15px';
form.appendChild(ageInput);

// Email input
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Email: ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.width = '100%';
emailInput.style.padding = '5px';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

// Submit button
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Submit';
submitButton.disabled = true;
form.appendChild(submitButton);

// Validation message
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Name validation
const nameValid$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return value.length >= 2;
  }),
  startWith(false)
);

// Age validation
const ageValid$ = fromEvent(ageInput, 'input').pipe(
  map((event) => {
    const value = Number((event.target as HTMLInputElement).value);
    return !isNaN(value) && value > 0 && value < 120;
  }),
  startWith(false)
);

// Email validation
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const emailValid$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return emailRegex.test(value);
  }),
  startWith(false)
);

// Validate all fields
combineLatest([nameValid$, ageValid$, emailValid$])
  .pipe(
    // tap((v) => console.log(v)),
    map((validList) => validList.every((v) => v === true))
  )
  .subscribe((allValid) => {
    submitButton.disabled = !allValid;
    if (allValid) {
      validationMessage.textContent = '';
    } else {
      validationMessage.textContent =
        'Please fill in all fields correctly';
    }
  });

// Form submission
form.addEventListener('submit', (event) => {
  event.preventDefault();

  const formData = {
    name: nameInput.value,
    age: ageInput.value,
    email: emailInput.value,
  };

  validationMessage.textContent = 'Form submitted!';
  validationMessage.style.color = 'green';

  console.log('Submitted data:', formData);
});
```
