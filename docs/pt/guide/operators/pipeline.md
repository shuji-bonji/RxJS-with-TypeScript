---
description: "Learn RxJS pipeline construction with pipe method: Transform, filter, and combine data streams declaratively with comprehensive operator chaining examples"
---

# What is RxJS Pipeline?

Pipelining in RxJS is a mechanism to apply a series of operations (operators) to an Observable in sequence. Pipelining allows you to transform, filter, and combine data streams in multiple stages, allowing you to control the flow of data in a declarative programming style.

## Basic Structure of a Pipeline

[📘 RxJS Official: pipe()](https://rxjs.dev/api/index/function/pipe)

The RxJS `pipe()` method is used to build a pipeline. The syntax is as follows.

```ts
import { Observable } from 'rxjs';
import { map, filter, tap } from 'rxjs';

const source$: Observable<number> = // Some Observable
source$.pipe(
  // Chain multiple operators
  operator1(),
  operator2(),
  operator3(),
  // ...
).subscribe(value => {
  // Process the result
});
```

## Practical Examples

### Basic Data Conversion

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// Stream of numbers
const numbers$ = of(1, 2, 3, 4, 5);

// Build a pipeline
numbers$.pipe(
  // Pass only even numbers
  filter(n => n % 2 === 0),
  // Double the value
  map(n => n * 2)
).subscribe(
  value => console.log(`Result: ${value}`)
);

// Output:
// Result: 4
// Result: 8
```

### Complex Data Processing

```ts
import { fromEvent, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

type User = {
  id: number;
  name: string;
  username: string;
  email: string;
};
type Post = {
  userId: number;
  id: number;
  title: string;
  body: string;
};

// Create DOM elements
const searchButton = document.createElement('button');
searchButton.innerText = 'Search';
document.body.appendChild(searchButton);

const resultBox = document.createElement('div');
resultBox.id = 'results';
document.body.appendChild(resultBox);

// API request on button click
fromEvent(searchButton, 'click')
  .pipe(
    switchMap(() =>
      // First API call
      ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
        // Second API call to get user's posts
        switchMap((user) => {
          const header = document.createElement('h3');
          header.textContent = `User: ${user.name}`;
          resultBox.innerHTML = ''; // Clear previous results
          resultBox.appendChild(header);

          return ajax.getJSON<Post[]>(
            `https://jsonplaceholder.typicode.com/posts?userId=${user.id}`
          );
        }),
        // Get only the first 3 posts
        map((posts) => posts.slice(0, 3))
      )
    )
  )
  .subscribe((posts) => {
    // Display posts on the screen
    resultBox.innerHTML += '<h4>Posts:</h4>';
    posts.forEach((post) => {
      const div = document.createElement('div');
      div.innerHTML = `<strong>${post.title}</strong><p>${post.body}</p>`;
      resultBox.appendChild(div);
    });
  });

```


## Advantages of the Pipeline

First, let's look at the code written in an imperative manner. As shown next, RxJS pipelining allows you to rewrite it in a more readable and maintainable form while making the intent of the process clear.

### 1. Improved Readability and Maintainability

```ts
// Processing in imperative style
const data = [
  { id: 3, active: true },
  { id: 1, active: false },
  { id: 2, active: true }
];

const activeItems = [];
for (const item of data) {
  if (item.active) {
    activeItems.push({ ...item, label: `Item #${item.id}` });
  }
}
activeItems.sort((a, b) => a.id - b.id);

const div1 = document.createElement('div');
div1.innerHTML = '<h3>Imperative Style</h3>';
activeItems.forEach(item => {
  const p = document.createElement('p');
  p.textContent = item.label;
  div1.appendChild(p);
});
document.body.appendChild(div1);
```
⬇️⬇️⬇️
```ts
import { of } from 'rxjs';
import { filter, map, toArray } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>Improved Readability and Maintainability</h3>';
document.body.appendChild(output);

of(
  { id: 3, active: true },
  { id: 1, active: false },
  { id: 2, active: true }
).pipe(
  filter(item => item.active),
  map(item => ({ ...item, label: `Item #${item.id}` })),
  toArray(),
  map(array => array.sort((a, b) => a.id - b.id))
).subscribe(sorted => {
  sorted.forEach(item => {
    const div = document.createElement('div');
    div.textContent = item.label;
    output.appendChild(div);
  });
});
```

Pipelining makes the flow of data clear and eliminates the need to reassign variables or manage intermediate states.



Procedural code such as the above can be written concisely in a declarative style by using RxJS pipelining. An example is shown below.

### 2. Declarative Programming Style

Pipelining promotes a declarative style that explicitly states "what to do". This makes the intent of the code clearer.

```ts
// Processing in procedural style
const usersList = [
  { status: 'active', firstName: 'Taro', lastName: 'Yamada', email: 'taro@example.com' },
  { status: 'inactive', firstName: 'Hanako', lastName: 'Yamada', email: 'hanako@example.com' },
  { status: 'active', firstName: 'John', lastName: 'Doe', email: 'john@example.com' }
];

const activeUsers2 = [];
for (const user of usersList) {
  if (user.status === 'active') {
    const name = `${user.firstName} ${user.lastName}`;
    activeUsers2.push({ name, email: user.email });
  }
}

const div2 = document.createElement('div');
div2.innerHTML = '<h3>Procedural Style</h3>';
activeUsers2.forEach(user => {
  const p = document.createElement('p');
  p.textContent = `${user.name} (${user.email})`;
  div2.appendChild(p);
});
document.body.appendChild(div2);
```

⬇️⬇️⬇️

```ts
// Declarative programming style
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

const out2 = document.createElement('div');
out2.innerHTML = '<h3>Declarative Style</h3>';
document.body.appendChild(out2);

const users = [
  { status: 'active', firstName: 'Taro', lastName: 'Yamada', email: 'taro@example.com' },
  { status: 'inactive', firstName: 'Hanako', lastName: 'Yamada', email: 'hanako@example.com' },
  { status: 'active', firstName: 'John', lastName: 'Doe', email: 'john@example.com' }
];

from(users).pipe(
  filter(user => user.status === 'active'),
  map(user => ({
    name: `${user.firstName} ${user.lastName}`,
    email: user.email
  }))
).subscribe(user => {
  const div = document.createElement('div');
  div.textContent = `${user.name} (${user.email})`;
  out2.appendChild(div);
});
```


Similarly here, let's take code that describes processing in a procedural manner and reorganize it with pipelining. Complex processing can be simply constructed by composing individual operators.

### 3. Composability

Pipelining allows you to build complex processing by combining small operations.

```ts
// Procedural (imperative) style processing
const rawUsers = [
  { firstName: 'Alice', lastName: 'Smith', status: 'active' },
  { firstName: 'Bob', lastName: 'Brown', status: 'inactive' },
  { firstName: 'Carol', lastName: 'Jones', status: 'active' }
];

const activeUsers = [];
for (const user of rawUsers) {
  if (user.status === 'active') {
    const fullName = `${user.firstName} ${user.lastName}`;
    activeUsers.push({ ...user, fullName });
  }
}
activeUsers.sort((a, b) => a.fullName.localeCompare(b.fullName));

const div0 = document.createElement('div');
div0.innerHTML = '<h3>Procedural Style</h3>';
activeUsers.forEach(user => {
  const p = document.createElement('p');
  p.textContent = user.fullName;
  div0.appendChild(p);
});
document.body.appendChild(div0);
```

⬇️⬇️⬇️

```ts
// Declarative programming style
import { from } from 'rxjs';
import { filter, map, toArray } from 'rxjs';

const out3 = document.createElement('div');
out3.innerHTML = '<h3>Composability</h3>';
document.body.appendChild(out3);

const users3 = [
  { firstName: 'Alice', lastName: 'Smith', status: 'active' },
  { firstName: 'Bob', lastName: 'Brown', status: 'inactive' },
  { firstName: 'Carol', lastName: 'Jones', status: 'active' }
];

const filterActive = filter((user: any) => user.status === 'active');
const formatFullName = map((user: any) => ({ ...user, fullName: `${user.firstName} ${user.lastName}` }));
const collectAndSort = toArray();
const sortByName = map((users: any[]) => users.sort((a, b) => a.fullName.localeCompare(b.fullName)));

from(users3).pipe(
  filterActive,
  formatFullName,
  collectAndSort,
  sortByName
).subscribe(users => {
  users.forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.fullName;
    out3.appendChild(div);
  });
});
```

## Pipeline Optimization Techniques

### 1. Importance of Operator Order

Operator order has a significant impact on both performance and functionality.

```ts
// Inefficient: map is applied to all elements
observable$.pipe(
  map(x => expensiveTransformation(x)),
  filter(x => x > 10)
)

// Efficient: filter is executed first, reducing elements to transform
observable$.pipe(
  filter(x => x > 10),
  map(x => expensiveTransformation(x))
)
```

### 2. Creating Custom Pipelines

Complex processing can be extracted into reusable pipelines.

```ts
import { Observable, pipe } from 'rxjs';
import { filter, map } from 'rxjs';

// Custom pipeline function
export function filterAndTransform<T, R>(
  filterFn: (value: T) => boolean,
  transformFn: (value: T) => R
) {
  return pipe(
    filter(filterFn),
    map(transformFn)
  );
}

// Usage example
observable$.pipe(
  filterAndTransform(
    x => x > 10,
    x => x * 2
  )
).subscribe(console.log);
```

## Common Mistakes with Pipelines

### 1. Operator Misordering

```ts
// ❌ If you apply filter before debounceTime,
// filter will be executed for each input, reducing the effect of debounce
inputEvents$.pipe(
  filter(text => text.length > 2),
  debounceTime(300)
)

// ✅ Apply debounceTime first
inputEvents$.pipe(
  debounceTime(300),
  filter(text => text.length > 2)
)
```

### 2. Side Effects in the Pipeline

```ts
// ❌ Directly execute side effects in the pipeline
observable$.pipe(
  map(data => {
    // Side effects (bad example)
    console.log(data);
    localStorage.setItem('lastData', JSON.stringify(data));
    return data;
  })
)

// ✅ Use the tap operator
observable$.pipe(
  tap(data => {
    console.log(data);
    localStorage.setItem('lastData', JSON.stringify(data));
  }),
  // Perform data transformation with map
  map(data => transformData(data))
)
```

## Summary

RxJS pipelines are a powerful mechanism for managing complex asynchronous data flows in a declarative and composable manner. Properly designed pipelines can greatly improve code readability, maintainability, and reusability.

When designing pipelines, it is a good idea to keep the following points in mind:

1. Choose the most efficient sequence of operators
2. Extract and reuse common pipeline patterns
3. Isolate side effects with `tap` operators
4. Ensure that each step in the pipeline has a single responsibility

Such a pipeline-oriented approach is especially powerful in scenarios such as complex UI event processing, API requests, and state management.
