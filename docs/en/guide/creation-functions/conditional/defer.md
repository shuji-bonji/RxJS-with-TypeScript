---
description: The defer operator causes the Observable's factory function to be delayed until the point of subscription. This is useful when you want to evaluate a different value or process each time you subscribe, such as the current time, random values, dynamic API requests, or other processes whose results change at the time of execution.
---

# defer - Observable creation with delayed evaluation

The `defer` operator executes the Observable factory function at **the point of subscription** and returns the resulting Observable. This allows you to delay the creation of an Observable until it is actually subscribed to.

## Basic syntax and operation

```ts
import { defer, of } from 'rxjs';

const random$ = defer(() => of(Math.random()));

random$.subscribe(console.log);
random$.subscribe(console.log);

// Output:
// 0.8727962287400634
// 0.8499299688934545
```

In this example, `Math.random()` is evaluated for each subscription, so a different value is issued each time.

[🌐 RxJS Official Documentation - defer](https://rxjs.dev/api/index/function/defer)

## Typical Application Examples

This is useful when you want to perform **processes** such as APIs, external resources, current time, random numbers, etc., whose results vary depending on the timing of execution.

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchUser(userId: number) {
  return defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );
}

fetchUser(1).subscribe(console.log);

// Output:
// {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
```

## Practical code examples (with UI)

`defer` is especially useful for processes that have side effects or produce different results each time.

In the code below, you can experience what it means to use `defer` to "generate a different Observable each time it is subscribed".
This is especially useful in cases** where you want to do the fetch process **every time** instead of caching it.

### ✅ 1. generate a random number each time
```ts
import { defer, of } from 'rxjs';

// Observable that generates random numbers
const randomNumber$ = defer(() => {
  const random = Math.floor(Math.random() * 100);
  return of(random);
});

// Create UI elements
const randomContainer = document.createElement('div');
randomContainer.innerHTML = '<h3>Random value generation with defer:</h3>';
document.body.appendChild(randomContainer);

// Generate button
const generateButton = document.createElement('button');
generateButton.textContent = 'Generate random value';
randomContainer.appendChild(generateButton);

// History display area
const randomHistory = document.createElement('div');
randomHistory.style.marginTop = '10px';
randomHistory.style.padding = '10px';
randomHistory.style.border = '1px solid #ddd';
randomHistory.style.maxHeight = '200px';
randomHistory.style.overflowY = 'auto';
randomContainer.appendChild(randomHistory);

// Button event
generateButton.addEventListener('click', () => {
  randomNumber$.subscribe(value => {
    const entry = document.createElement('div');
    entry.textContent = `Generated value: ${value}`;
    entry.style.padding = '5px';
    entry.style.margin = '2px 0';
    entry.style.backgroundColor = '#f5f5f5';
    entry.style.borderRadius = '3px';
    randomHistory.insertBefore(entry, randomHistory.firstChild);
  });
});

// Explanation text
const randomExplanation = document.createElement('p');
randomExplanation.textContent = 'Each time you click the "Generate random value" button, a new random value will be generated. If you use normal of, the value will be generated only once at the beginning, but by using defer, you can generate a new value each time.';
randomContainer.appendChild(randomExplanation);
```

### ✅ 2. Execute each API request

Because `defer` creates a new Observable each time it is subscribed to, it is especially useful in situations where you want to perform different API requests based on **user input, etc.**.
For example, use the following scenario.

- ✅ Fetching at different URLs depending on dynamic queries or parameters
- ✅ Fetching the latest data each time** without using the cache
- ✅ Want to lazily evaluate processing when an event occurs

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const container = document.createElement('div');
container.innerHTML = '<h3>API request with defer:</h3>';
document.body.appendChild(container);

// Input field
const input = document.createElement('input');
input.placeholder = 'Enter user ID';
container.appendChild(input);

// Execute button
const button = document.createElement('button');
button.textContent = 'Get user information';
container.appendChild(button);

// Result display
const resultBox = document.createElement('pre');
resultBox.style.border = '1px solid #ccc';
resultBox.style.padding = '10px';
resultBox.style.marginTop = '10px';
container.appendChild(resultBox);

// Button event
button.addEventListener('click', () => {
  const userId = input.value.trim();
  if (!userId) {
    resultBox.textContent = 'Please enter user ID';
    return;
  }

  const user$ = defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );

  resultBox.textContent = 'Loading...';
  user$.subscribe({
    next: (data) => (resultBox.textContent = JSON.stringify(data, null, 2)),
    error: (err) => (resultBox.textContent = `Error: ${err.message}`),
  });
});
```

In this example, the `defer` causes `ajax.getJSON()` to be called when the user presses the button,
**`of(ajax.getJSON(...)) Unlike the `defer`, which evaluates from the beginning, you have complete control** over the timing of execution.
