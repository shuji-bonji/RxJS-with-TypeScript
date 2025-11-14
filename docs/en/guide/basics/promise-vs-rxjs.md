---
description: Understand the differences between Promise and RxJS and learn how to use them appropriately. Promise specializes in single asynchronous processing and executes immediately, while RxJS is lazy-evaluated, can handle multiple values, and can be cancelled and reused. This guide explains in detail the characteristics of each and the criteria for selecting them through code comparisons and specific use cases.
---

# Differences between Promise and RxJS

## Overview

The main tools for handling asynchronous processing in JavaScript/TypeScript are **Promise** and **RxJS (Observable)**. Although both are sometimes used for similar purposes, their design philosophy and use cases are quite different.

This page provides information to help you understand the differences between Promise and RxJS and decide which one to use.

## Basic Differences

| Item | Promise | RxJS (Observable) |
|------|---------|-------------------|
| **Standardization** | JavaScript standard (ES6/ES2015) | Third-party library |
| **Values emitted** | Single value | Zero or more multiple values |
| **Evaluation** | Eager (executes immediately upon creation) | Lazy (executes upon subscription) |
| **Cancellation** | Not possible[^1] | Possible (`unsubscribe()`) |
| **Reusability** | Not possible (result is only once) | Possible (can be subscribed multiple times) |
| **Learning cost** | Low | High (requires understanding of operators) |
| **Use cases** | Single asynchronous processing | Complex stream processing |

[^1]: While Promise-based processing (such as fetch) can be cancelled using AbortController, the Promise specification itself does not have a cancellation function.

## Code Comparison: Single Asynchronous Processing

### Promise

```ts
// Promise executes immediately upon creation (Eager)
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json())
  .then(data => console.log(data))
  .catch(error => console.error(error));
```

Promise **begins execution the moment it is defined** (Eager evaluation).

### RxJS

```ts
import { from } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

// Observable does not execute until subscribed (Lazy)
const observable$ = from(fetch('https://jsonplaceholder.typicode.com/posts/1')).pipe(
  switchMap(response => response.json()), // response.json() returns a Promise, so use switchMap
  catchError(error => {
    console.error(error);
    return of(null);
  })
);

// Execution begins only when subscribed
observable$.subscribe(data => console.log(data));
```

RxJS **does not execute until `subscribe()` is called** (Lazy evaluation). Subscribing to the same Observable multiple times results in independent executions, and processing can be interrupted with `unsubscribe()`.

> [!TIP]
> **Practical usage guidelines**
> - Immediate one-time processing → Promise
> - Processing to be executed at a specific time or multiple times → RxJS

## Code Comparison: Handling Multiple Values

One of the biggest differences between Promise and RxJS is the number of values that can be emitted. Promise can only return a single value, while RxJS can emit multiple values over time.

### Impossible with Promise

Promise can only **resolve once**.

```ts
// Promise can only return a single value
const promise = new Promise(resolve => {
  resolve(1);
  resolve(2); // This value is ignored
  resolve(3); // This value is also ignored
});

promise.then(value => console.log(value));
// Output: 1 (only the first value)
```

Once the value is determined by the first `resolve()`, subsequent `resolve()` calls are ignored.

### Possible with RxJS

Observable **can emit values any number of times**.
```ts
import { Observable } from 'rxjs';

// Observable can emit multiple values
const observable$ = new Observable(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  subscriber.complete();
});

observable$.subscribe(value => console.log(value));
// Output: 1, 2, 3
```

Each time `next()` is called, the value is delivered to the subscriber. After emitting all values, completion is notified with `complete()`. This characteristic allows natural handling of time-series changing data such as real-time communication, streaming data, and continuous event processing.

> [!NOTE]
> **Practical application examples**
> - Receiving WebSocket messages
> - Sequential keyboard input processing
> - Server event streams (SSE)
> - Continuous sensor data monitoring

## Cancellation Comparison

The ability to cancel long-running or unnecessary asynchronous processing is important from the perspectives of resource management and user experience. There are significant differences in cancellation capabilities between Promise and RxJS.

### Promise (Not Cancelable)
Promise has **no standard cancellation function**.

```ts
const promise = new Promise(resolve => {
  setTimeout(() => resolve('Complete'), 3000);
});

promise.then(result => console.log(result));
// There is no standard way to cancel this processing
```

Once execution begins, it cannot be stopped until completion, which can cause memory leaks and performance degradation.

> [!WARNING]
> **About AbortController**
> Web APIs such as `fetch()` can be cancelled using `AbortController`, but this is not a feature of Promise itself, but a mechanism provided by individual APIs. It is not available for all asynchronous processing.

### RxJS (Cancelable)

RxJS **can be cancelled at any time with `unsubscribe()`**.
```ts
import { timer } from 'rxjs';

const subscription = timer(3000).subscribe(
  () => console.log('Complete')
);

// Cancel after 1 second
setTimeout(() => {
  subscription.unsubscribe(); // Cancel
  console.log('Cancelled');
}, 1000);
// Output: Cancelled ("Complete" is not output)
```

Unsubscribing immediately stops the ongoing processing and prevents memory leaks.

> [!TIP]
> **Practical cancellation use cases**
> - Cancel HTTP requests when user leaves the screen
> - Discard old search query results and process only the latest query (`switchMap`)
> - Automatically cancel all Observables when component is destroyed (`takeUntil` pattern)

## Which One to Choose

Whether to use Promise or RxJS depends on the nature of the processing and project requirements. Use the following criteria as a reference to select the appropriate tool.

### When to Choose Promise

Promise is suitable if the following conditions apply.

| Condition | Reason |
|------|------|
| Single asynchronous processing | One API request, one file read, etc. |
| Simple workflow | `Promise.all`, `Promise.race` are sufficient |
| Small-scale projects | Want to minimize dependencies |
| Use standard API only | Want to avoid external libraries |
| Beginner-friendly code | Want to reduce learning costs |

#### Single API Request:


```ts
interface User {
  id: number;
  name: string;
  email: string;
  username: string;
}

async function getUserData(userId: string): Promise<User> {
  const response = await fetch(`https://jsonplaceholder.typicode.com/users/${userId}`);
  if (!response.ok) {
    throw new Error('Failed to retrieve user data');
  }
  return response.json();
}

// Usage example
getUserData('1').then(user => {
  console.log('User name:', user.name);
  console.log('Email:', user.email);
});
```

This code is a typical pattern for retrieving single user information. Using `async/await` makes it as readable as synchronous code. Error handling can also be unified with `try/catch`, making it simple and intuitive.

#### Parallel Execution of Multiple Asynchronous Processes:

```ts
interface Post {
  id: number;
  userId: number;
  title: string;
  body: string;
}

async function loadAllData(): Promise<[User[], Post[]]> {
  const [users, posts] = await Promise.all([
    fetch('https://jsonplaceholder.typicode.com/users').then(r => r.json()),
    fetch('https://jsonplaceholder.typicode.com/posts').then(r => r.json())
  ]);
  return [users, posts];
}

// Usage example
loadAllData().then(([users, posts]) => {
  console.log('Number of users:', users.length);
  console.log('Number of posts:', posts.length);
});
```

`Promise.all()` allows you to execute multiple API requests in parallel and wait for all of them to complete. This is very convenient for initial data loading. Note that if even one fails, the entire process errors, but its simplicity makes it easy to understand and maintain.

### When to Choose RxJS

RxJS is suitable if the following conditions apply.

| Condition | Reason |
|------|------|
| Continuous event processing | Mouse movement, keyboard input, WebSocket, etc. |
| Complex stream processing | Combining and transforming multiple event sources |
| Cancellation required | Want to finely control resource management |
| Retry/Timeout | Want flexible error handling |
| Angular projects | RxJS is integrated into the framework |
| Real-time data | Data is continuously updated |

#### Concrete Example
```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'search: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);

// Real-time search (autocomplete)
if (!searchInput) throw new Error('Search input not found');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),              // Wait 300ms before processing
  distinctUntilChanged(),         // Process only when value changes
  switchMap(query =>              // Execute only the latest request
    fetch(`https://api.github.com/search/users?q=${query}`).then(r => r.json())
  )
).subscribe(results => {
  console.log('Search results:', results.items); // GitHub API stores results in items property
});
```

This example is a typical case where RxJS shows its true value. It monitors user input, provides a 300ms wait time to reduce unnecessary requests, processes only when the value changes, and by making only the latest request valid (`switchMap`), it automatically discards the results of old requests.

> [!IMPORTANT]
> **Why it's difficult with Promise alone**
> - Must manually implement debounce (continuous input control)
> - Must manage cancellation of old requests yourself
> - Forgetting to clean up event listeners causes memory leaks
> - Must track multiple states simultaneously (timers, flags, request management)
>
> With RxJS, all of these can be realized declaratively in just a few lines.

## Interoperability between Promise and RxJS

Promise and RxJS are not mutually exclusive and can be converted to each other and combined. This is useful when integrating existing Promise-based code into RxJS pipelines, or conversely when you want to use Observable in existing Promise-based code.

## Convert Promise to Observable

RxJS provides multiple ways to convert an existing Promise to Observable.

### Conversion by `from`

The most common method is to use `from`.

```ts
import { from } from 'rxjs';

// Create Promise
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json());

// Convert to Observable with from()
const observable$ = from(promise);

observable$.subscribe({
  next: data => console.log('Data:', data),
  error: error => console.error('Error:', error),
  complete: () => console.log('Complete')
});
```

The result of the Promise flows as Observable, and completion is also called automatically.

### Conversion by `defer` (lazy evaluation)

The `defer` delays the creation of a Promise until it is subscribed.

```ts
import { defer } from 'rxjs';

// Promise is not created until subscribe
const observable$ = defer(() =>
  fetch('https://jsonplaceholder.typicode.com/posts/1').then(r => r.json())
);

// Create new Promise on each subscribe
observable$.subscribe(data => console.log('1st:', data));
observable$.subscribe(data => console.log('2nd:', data));
```

This method is useful if you want to create a new Promise each time you subscribe.

## Convert Observable to Promise

It is possible to take only one value from an Observable and turn it into a Promise.

### `firstValueFrom` and `lastValueFrom`

The following two functions are recommended in RxJS 7 and later.

| Function | Behavior |
|------|------|
| `firstValueFrom` | Returns the first value as a Promise |
| `lastValueFrom` | Returns the last value on completion as a Promise |

```ts
import { of, firstValueFrom, lastValueFrom } from 'rxjs';
import { delay } from 'rxjs';

const observable$ = of(1, 2, 3).pipe(delay(1000));

// Get first value as Promise
const firstValue = await firstValueFrom(observable$);
console.log(firstValue); // 1

// Get last value as Promise
const lastValue = await lastValueFrom(observable$);
console.log(lastValue); // 3
```

If Observable completes before the value flows, the default is an error. This can be avoided by specifying a default value.

> [!WARNING]
> `toPromise()` is deprecated. Use `firstValueFrom()` or `lastValueFrom()` instead.

> [!TIP]
> **Selection guidelines**
> - **`firstValueFrom()`**: When only the first value is needed (e.g., login authentication result)
> - **`lastValueFrom()`**: When the final result after processing all data is needed (e.g., aggregation result)

## Practical Example: Combining Both

In actual application development, Promise and RxJS are frequently combined.

> [!WARNING] Practical Precautions
> Mixing Promise and Observable can easily **fall into anti-patterns if design boundaries are not clear**.
>
> **Common problems:**
> - Becomes uncancelable
> - Separation of error handling
> - `await` inside `subscribe` (especially dangerous)
> - Parallel acquisition of the same data with Promise and Observable
>
> See **[Chapter 10: Promise and Observable Mixing Anti-patterns](/en/guide/anti-patterns/promise-observable-mixing)** for details.

### Form Submission and API Calls

Example of catching a user's form submission event in RxJS and sending it to the server using Fetch API (Promise).

```ts
import { fromEvent, from } from 'rxjs';
import { exhaustMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface FormData {
  username: string;
  email: string;
}

// Promise-based form submission
async function submitForm(data: FormData): Promise<{ success: boolean }> {
  const response = await fetch('https://api.example.com/submit', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  });
  if (!response.ok) {
    throw new Error('Submission failed');
  }
  return response.json();
}

// Event stream management with RxJS
const submitButton = document.createElement('button');
submitButton.id = 'submit-button';
submitButton.innerText = 'Submit';
submitButton.style.padding = '10px 20px';
submitButton.style.margin = '10px';
document.body.appendChild(submitButton);
if (!submitButton) throw new Error('Submit button not found');

fromEvent(submitButton, 'click').pipe(
  exhaustMap(() => {
    const formData: FormData = {
      username: 'testuser',
      email: 'test@example.com'
    };
    // Convert Promise function to Observable
    return from(submitForm(formData));
  }),
  catchError(error => {
    console.error('Submission error:', error);
    return of({ success: false });
  })
).subscribe(result => {
  if (result.success) {
    console.log('Submission successful');
  } else {
    console.log('Submission failed');
  }
});
```

Each time the form submit button is clicked, a new submission process is initiated, but **ignores new submissions during submission**.

In this example, the use of `exhaustMap` prevents duplicate requests during transmission.

### Search Autocomplete

Example of monitoring input form value changes and performing API searches.

```ts
import { fromEvent, from } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface SearchResult {
  items: Array<{
    login: string;
    id: number;
    avatar_url: string;
  }>;
  total_count: number;
}

// Promise-based API function
async function searchAPI(query: string): Promise<SearchResult> {
  const response = await fetch(`https://api.github.com/search/users?q=${query}`);
  if (!response.ok) {
    throw new Error('Search failed');
  }
  return response.json();
}

// Event stream management with RxJS
const label = document.createElement('label');
label.innerText = 'search: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);
if (!searchInput) throw new Error('Search input not found');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),
  distinctUntilChanged(),
  switchMap(query => {
    // Convert Promise function to Observable
    return from(searchAPI(query));
  }),
  catchError(error => {
    console.error(error);
    return of({ items: [], total_count: 0 }); // Return empty result on error
  })
).subscribe(result => {
  console.log('Search results:', result.items);
  console.log('Total:', result.total_count);
});
```

In this example, the following controls are realized:

- Wait 300ms for input completion with `debounceTime(300)`
- `distinctUntilChanged()` to ignore if the value is the same as the previous one
- `switchMap` to retrieve only the latest search results (old requests are automatically canceled)

> [!WARNING] Beware of anti-patterns
> The pattern to subscribe Observable in Promise can cause memory leaks and unexpected behavior.
> <!-- TODO: Add link to subscribe-in-promise anti-pattern when available -->

> [!TIP]
> **Design by separation of responsibilities**
>
> - **RxJS**: In charge of event control (debounce, switchMap, etc.)
> - **Promise**: In charge of HTTP requests (async/await)
> - **`from()`**: Bridge between both
>
> Using each technology appropriately improves code readability and maintainability.

## Advantages and Disadvantages

Each technology has its suitability and disadvantages.

### Promise
<div class="comparison-cards">

::: tip Benefits
- No dependencies required as it is JavaScript standard
- Intuitive and readable code with `async/await`
- Low learning cost
- Simple processing of single tasks
:::

::: danger Disadvantages
- Cannot handle multiple values
- No cancellation function
- Not suitable for continuous stream processing
- Complex event processing is difficult
:::

</div>

### RxJS
<div class="comparison-cards">

::: tip Benefits
- Can handle multiple values over time
- Complex control possible with a wide variety of operators
- Cancellation (`unsubscribe`) is easy
- Flexible implementation of error handling and retry
- Declarative and testable
:::

::: danger Disadvantages
- High learning cost
- Requires libraries
- Over-specified for simple processes
- Debugging can be difficult
:::

</div>

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>

## Areas Where RxJS is Particularly Active

RxJS is particularly powerful in the following situations.

| Field | Reason | Concrete Examples |
|------|------|--------|
| UI Event Processing | Efficiently control continuous events | Clicks, scrolls, keyboard inputs |
| Real-Time Communication | Support for continuous data streams | WebSocket, Server-Sent Events |
| Form Processing | Combine and process multiple input values | Real-time validation, conditional input |
| Control API Calls | Easy cancellation, retry, timeout | Search APIs, data fetching |
| Complex Asynchronous Flows | Combine multiple asynchronous processes | Dependent API call chains |

> [!NOTE]
> For details on the use of RxJS, see also [What is RxJS - Use Cases](./what-is-rxjs.md#use-cases).

## Summary

| Purpose | Recommended | Reason |
|------|------|------|
| Single HTTP request | Promise (`async/await`) | Simple, readable, standard API |
| User input event processing | RxJS | Requires control such as debounce, distinct |
| Real-time data (WebSocket) | RxJS | Can naturally handle continuous messages |
| Parallel execution of multiple asynchronous processes | Promise (`Promise.all`) | Promise is sufficient for simple parallel execution |
| Continuous event stream | RxJS | Can handle multiple values over time |
| Cancelable processing | RxJS | Reliable cancellation with unsubscribe() |
| Simple applications | Promise | Low learning cost, few dependencies |
| Angular applications | RxJS | Standardly integrated into the framework |

### Basic Policy
- **Use Promise if it can be simple**
- **Use RxJS if complex stream processing is required**
- **Combining both is also effective** (bridge with `from()`)

RxJS is powerful, but you don't need to use RxJS for all asynchronous processing. It is important to use the right tool in the right situation. Promise and RxJS are both powerful tools for handling asynchronous processing, but each has different characteristics.

- **Promise** is best suited for simple one-shot asynchronous processing. Choose Promise for basic asynchronous processing because of its low learning cost and good compatibility with async/await.
- **RxJS** is powerful when handling multiple values, event processing, or complex data flow control is required. RxJS is also suitable when advanced controls such as cancel and retry are required.

In actual development, it is important to use both appropriately. If necessary, you can be flexible by converting Promise to Observable or Observable to Promise.

> [!TIP] Next Steps
> - Learn more about Observable in [What is Observable](/en/guide/observables/what-is-observable)
> - Learn how to create Observable in [Creation Functions](/en/guide/creation-functions/index)
> - Learn how to convert and control Observables with [Operators](/en/guide/operators/index)
