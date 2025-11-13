---
description: This presentation covers practical use cases of RxJS utility operators (tap, startWith, finalize, delay, timeout, retry, etc.). Practical patterns frequently used in UI development such as loading state management, reactive form validation, API call control, error handling, debugging assistance, etc. will be introduced with TypeScript code examples. You will learn implementation techniques to control and observe stream behavior.
---

# Practical Use Cases

## Managing Loading State

This is an example of using `tap`, `finalize`, etc. to manage loading state.

```ts
import { of, throwError } from 'rxjs';
import { tap, delay, finalize, catchError } from 'rxjs';

// UI elements
const loadingExample = document.createElement('div');
loadingExample.innerHTML = '<h3>API call and loading state management:</h3>';
document.body.appendChild(loadingExample);

// Loading indicator
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Loading...';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.backgroundColor = '#e3f2fd';
loadingIndicator.style.borderRadius = '5px';
loadingIndicator.style.display = 'none';
loadingExample.appendChild(loadingIndicator);

// Data display area
const dataContainer = document.createElement('div');
dataContainer.style.marginTop = '10px';
dataContainer.style.padding = '10px';
dataContainer.style.border = '1px solid #ddd';
dataContainer.style.borderRadius = '5px';
dataContainer.style.minHeight = '100px';
loadingExample.appendChild(dataContainer);

// Success button
const successButton = document.createElement('button');
successButton.textContent = 'Successful request';
successButton.style.marginRight = '10px';
successButton.style.padding = '8px 16px';
loadingExample.insertBefore(successButton, loadingIndicator);

// Fail button
const failButton = document.createElement('button');
failButton.textContent = 'Failed request';
failButton.style.padding = '8px 16px';
loadingExample.insertBefore(failButton, loadingIndicator);

// Simulate successful API request
function simulateSuccessRequest() {
  return of({
    id: 1,
    name: 'Sample data',
    description: 'This is data retrieved from the API.'
  }).pipe(
    // Show loading on request start
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simulate API latency
    delay(1500),
    // Always hide loading on request completion
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Simulate failed API request
function simulateFailRequest() {
  return throwError(() => new Error('API request failed')).pipe(
    // Show loading on request start
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simulate API latency
    delay(1500),
    // Error handling
    catchError(error => {
      const errorElement = document.createElement('div');
      errorElement.textContent = `Error: ${error.message}`;
      errorElement.style.color = 'red';
      dataContainer.appendChild(errorElement);

      return throwError(() => error);
    }),
    // Always hide loading on request completion
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Success button click
successButton.addEventListener('click', () => {
  // Disable buttons
  successButton.disabled = true;
  failButton.disabled = true;

  simulateSuccessRequest().subscribe({
    next: data => {
      // Display data
      const dataElement = document.createElement('div');
      dataElement.innerHTML = `
        <h4>${data.name}</h4>
        <p>${data.description}</p>
        <p><em>ID: ${data.id}</em></p>
      `;
      dataContainer.appendChild(dataElement);
    },
    error: err => {
      console.error('Error:', err);
    },
    complete: () => {
      // Re-enable buttons
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});

// Fail button click
failButton.addEventListener('click', () => {
  // Disable buttons
  successButton.disabled = true;
  failButton.disabled = true;

  simulateFailRequest().subscribe({
    next: () => {
      // Won't succeed, but just in case
    },
    error: () => {
      // Error already handled by catchError
      console.log('Error handling completed');
    },
    complete: () => {
      // Re-enable buttons
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});
```

## Form Validation and Submission

The following is an example of implementing form validation and submission using `startWith`, `tap`, `finalize`, etc.

```ts
import { fromEvent, combineLatest, of } from 'rxjs';
import { map, startWith, debounceTime, tap, finalize, catchError, delay } from 'rxjs';

// Form UI
const formExample = document.createElement('div');
formExample.innerHTML = '<h3>Reactive form example:</h3>';
document.body.appendChild(formExample);

// Create form elements
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formExample.appendChild(form);

// Name input field
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Name: ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.padding = '8px';
nameInput.style.width = '100%';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

const nameError = document.createElement('div');
nameError.style.color = 'red';
nameError.style.fontSize = '12px';
nameError.style.marginTop = '-10px';
nameError.style.marginBottom = '15px';
form.appendChild(nameError);

// Email input field
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Email address: ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.padding = '8px';
emailInput.style.width = '100%';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

const emailError = document.createElement('div');
emailError.style.color = 'red';
emailError.style.fontSize = '12px';
emailError.style.marginTop = '-10px';
emailError.style.marginBottom = '15px';
form.appendChild(emailError);

// Submit button
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Submit';
submitButton.style.padding = '8px 16px';
submitButton.disabled = true; // Initially disabled
form.appendChild(submitButton);

// Result display area
const formResult = document.createElement('div');
formResult.style.marginTop = '20px';
formResult.style.padding = '10px';
formResult.style.border = '1px solid transparent';
formResult.style.borderRadius = '5px';
formResult.style.display = 'none';
formExample.appendChild(formResult);

// Name input validation
const name$ = fromEvent(nameInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'Name is required' };
    }
    if (value.length < 2) {
      return { value, valid: false, error: 'Name must be at least 2 characters' };
    }
    return { value, valid: true, error: null };
  })
);

// Email input validation
const emailRegex = /^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'Email address is required' };
    }
    if (!emailRegex.test(value)) {
      return { value, valid: false, error: 'Please enter a valid email address' };
    }
    return { value, valid: true, error: null };
  })
);

// Monitor form-wide validation state
combineLatest([name$, email$]).pipe(
  map(([nameState, emailState]) => {
    // Is entire form valid
    const isValid = nameState.valid && emailState.valid;

    // Display validation errors
    nameError.textContent = nameState.error || '';
    emailError.textContent = emailState.error || '';

    return isValid;
  })
).subscribe(isValid => {
  // Enable/disable submit button
  submitButton.disabled = !isValid;
});

// Form submission processing
fromEvent(form, 'submit').pipe(
  tap(event => {
    // Prevent default form submission
    event.preventDefault();

    // Set to submitting state
    submitButton.disabled = true;
    submitButton.textContent = 'Submitting...';

    // Reset result display area
    formResult.style.display = 'none';
  }),
  // Get form data
  map(() => ({
    name: nameInput.value.trim(),
    email: emailInput.value.trim()
  })),
  // Simulate API request
  delay(1500),
  // Always return to submission completed state
  finalize(() => {
    submitButton.textContent = 'Submit';
    submitButton.disabled = false;
  }),
  // Error handling
  catchError(error => {
    formResult.textContent = `Error: ${error.message}`;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#ffebee';
    formResult.style.borderColor = '#f44336';

    return of(null); // Continue stream
  })
).subscribe(data => {
  if (data) {
    // Submission successful
    formResult.innerHTML = `
      <div style="font-weight: bold;">Submission successful!</div>
      <div>Name: ${data.name}</div>
      <div>Email: ${data.email}</div>
    `;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#e8f5e9';
    formResult.style.borderColor = '#4caf50';

    // Reset form
    nameInput.value = '';
    emailInput.value = '';
  }
});
```

## How to Choose a Utility Operator

| Purpose | Operator | Usage Situation |
|------|--------------|---------|
| Side effect execution | `tap` | Debugging, log output, UI update, etc. |
| Output delay of values | `delay` | Animation, timing adjustment, etc. |
| Timeout settings | `timeout` | Timeout for API requests, asynchronous processing |
| Processing on completion | `finalize` | Cleanup of resources, release loading state |
| Set initial value | `startWith` | Initialize state, display placeholders |
| Convert to an array | `toArray` | Batch processing, all results are processed together |
| Retry on error | `retry` | Network requests, recovering from temporary errors |
| Repeat a stream | `repeat` | Polling, periodic processing |

## Summary

Utility operators are important tools that make programming in RxJS more efficient and robust. The proper combination of these operators provides the following benefits:

1. **Ease of Debugging**: Using `tap`, you can easily check the intermediate state of the stream.
2. **Error Tolerance**: The combination of `retry`, `timeout`, and `catchError` provides robust error handling.
3. **Resource Management**: `finalize` can be used to ensure proper resource cleanup.
4. **Improved UI responsiveness**: `startWith`, `delay`, etc. can be used to improve the user experience.
5. **Improve code readability**: Use of utility operators can clearly separate side-effects from pure data conversion.

These operators demonstrate their true value when used in combination with other operators rather than alone. In actual application development, it is common to combine multiple operators to manage complex asynchronous processing flows.
