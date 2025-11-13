---
description: The forkJoin Creation Function outputs the last value of each as an array or object after all multiple Observables have completed. This is ideal when multiple API requests are executed in parallel and all results are available before processing.
---

# forkJoin - output all last values together

`forkJoin` is a Creation Function that outputs the last value of each Observable as an array or object after **all** Observables have been completed.
This is very useful when you want to use all of the Observables at once.


## Basic syntax and usage

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

const user$ = of('User A').pipe(delay(1000));
const posts$ = of('Post list').pipe(delay(1500));

forkJoin([user$, posts$]).subscribe(([user, posts]) => {
  console.log(user, posts);
});

// Output:
// User A Post list
```

- Wait until all Observables are `complete`.
- Only the **last issued value** of each Observable is compiled and output.

[🌐 RxJS Official Documentation - `forkJoin`](https://rxjs.dev/api/index/function/forkJoin)


## Typical utilization patterns

- **Execute multiple API requests in parallel and summarize all results**
- **Get multiple datasets needed for initial load at once**
- **Get all relevant data at once and draw screen rendering at once**


## Practical code examples (with UI)

Simulate multiple API requests and display them together when all results are available.

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>forkJoin practical example:</h3>';
document.body.appendChild(output);

// Dummy data streams
const user$ = of({ id: 1, name: 'Taro Yamada' }).pipe(delay(2000));
const posts$ = of([{ id: 1, title: 'Post 1' }, { id: 2, title: 'Post 2' }]).pipe(delay(1500));
const weather$ = of({ temp: 22, condition: 'Sunny' }).pipe(delay(1000));

// Loading message
const loading = document.createElement('div');
loading.textContent = 'Loading data...';
loading.style.color = 'blue';
output.appendChild(loading);

// Output all at once after all requests are completed
forkJoin({
  user: user$,
  posts: posts$,
  weather: weather$
}).subscribe(result => {
  output.removeChild(loading);

  const pre = document.createElement('pre');
  pre.textContent = JSON.stringify(result, null, 2);
  pre.style.background = '#f5f5f5';
  pre.style.padding = '10px';
  pre.style.borderRadius = '5px';
  output.appendChild(pre);

  const summary = document.createElement('div');
  summary.textContent = `User: ${result.user.name}, Weather: ${result.weather.condition}, Posts: ${result.posts.length}`;
  output.appendChild(summary);
});
```

- Loading display first,
- When all the data is available, the results will be drawn together.
