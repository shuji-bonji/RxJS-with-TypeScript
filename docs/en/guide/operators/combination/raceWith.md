---
description: raceWith is an RxJS combination operator that adopts only the first stream that emits a value among the original Observable and other Observables. It is the Pipeable Operator version of Creation Function race, and is ideal for situations where the fastest response is a priority, such as timeout implementations, parallel acquisition from multiple CDNs (fallback), and competing acquisition from multiple data sources. It is effective when you want to compete with other streams while converting and processing the main stream.
---

# raceWith - Adopt the Fastest Stream (Within Pipeline)

The `raceWith` operator **adopts only the first stream that emits a value** among the original Observable and the specified other Observables, and ignores all others.
This is the Pipeable Operator version of the Creation Function `race`.

## 🔰 Basic Syntax and Usage

```ts
import { interval, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  take(3),
  map(val => `Source 1: ${val}`)
);

const source2$ = timer(500).pipe(
  take(3),
  map(val => `Source 2: ${val}`)
);

source1$
  .pipe(raceWith(source2$))
  .subscribe(console.log);

// Output:
// Source 2: 0 (after 500ms)
// * source1$ is ignored because source2$ emitted first
```

- **The first Observable to emit a value** wins the race, and only that stream is adopted.
- Other Observables are automatically unsubscribed and ignored.

[🌐 RxJS Official Documentation - `raceWith`](https://rxjs.dev/api/operators/raceWith)


## 💡 Typical Usage Patterns

- **Timeout implementation**: Compete main processing with timeout error after a certain time
- **Parallel acquisition from multiple CDNs**: Request multiple CDNs simultaneously and adopt the fastest response (fallback strategy)
- **Competing acquisition from multiple data sources**: Execute local cache and API call concurrently, and use whichever returns first
- **User action vs timer competition**: Compete click action with auto-advance, and adopt whichever occurs first


## 🧠 Practical Code Example (with UI)

Example of fetching data from multiple CDNs in parallel and adopting the fastest response.

```ts
import { fromFetch } from 'rxjs/fetch';
import { raceWith, map, catchError, timeout } from 'rxjs';
import { of } from 'rxjs';

// Build the UI
const container = document.createElement('div');
container.innerHTML = `
  <h3>raceWith Practical Example: Parallel Fetch from Multiple CDNs</h3>
  <button id="fetch-button">Fetch Data</button>
  <div id="status" style="margin-top: 10px; padding: 10px; border: 1px solid #ccc;">
    Waiting...
  </div>
  <div id="result" style="margin-top: 10px;"></div>
`;
document.body.appendChild(container);

const fetchButton = document.getElementById('fetch-button') as HTMLButtonElement;
const statusDiv = document.getElementById('status')!;
const resultDiv = document.getElementById('result')!;

// Start fetching data on button click
fetchButton.addEventListener('click', () => {
  statusDiv.textContent = 'Fetching from multiple CDNs in parallel...';
  statusDiv.style.backgroundColor = '#fff3e0';
  resultDiv.innerHTML = '';

  // Multiple CDNs (actually dummy endpoints)
  const cdn1$ = fromFetch('https://jsonplaceholder.typicode.com/posts/1').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 1', data: 'Data fetched successfully' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 1', data: 'Error' }))
  );

  const cdn2$ = fromFetch('https://jsonplaceholder.typicode.com/posts/2').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 2', data: 'Data fetched successfully' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 2', data: 'Error' }))
  );

  const cdn3$ = fromFetch('https://jsonplaceholder.typicode.com/posts/3').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 3', data: 'Data fetched successfully' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 3', data: 'Error' }))
  );

  // ✅ Adopt fastest response with raceWith
  cdn1$
    .pipe(raceWith(cdn2$, cdn3$))
    .subscribe({
      next: (result) => {
        statusDiv.textContent = `✅ Successfully fetched from ${result.source}`;
        statusDiv.style.backgroundColor = '#e8f5e9';
        resultDiv.innerHTML = `<strong>${result.source}</strong>: ${result.data}`;
      },
      error: (err) => {
        statusDiv.textContent = '❌ Failed to fetch from all CDNs';
        statusDiv.style.backgroundColor = '#ffebee';
        resultDiv.textContent = `Error: ${err.message}`;
      }
    });
});
```

- Requests multiple CDNs simultaneously, and **adopts the first CDN** that returns a response.
- Responses from other CDNs are automatically ignored.


## 🔄 Difference from Creation Function `race`

### Basic Differences

| | `race` (Creation Function) | `raceWith` (Pipeable Operator) |
|:---|:---|:---|
| **Usage Location** | Used as independent function | Used within `.pipe()` chain |
| **Syntax** | `race(obs1$, obs2$, obs3$)` | `obs1$.pipe(raceWith(obs2$, obs3$))` |
| **First Stream** | Treats all equally | Treats as main stream |
| **Advantage** | Simple and readable | Easy to combine with other operators |

### Specific Usage Examples

**Creation Function is Recommended for Simple Competition Only**

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const fast$ = timer(100).pipe(map(() => 'Fast wins!'));
const slow$ = timer(500).pipe(map(() => 'Slow wins!'));

// Simple and readable
race(fast$, slow$).subscribe(console.log);
// Output: Fast wins!
```

**Pipeable Operator is Recommended When Adding Transformation Processing to Main Stream**

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, mapTo, take } from 'rxjs';

// User click vs auto-advance competition
const userClick$ = fromEvent(document, 'click').pipe(
  take(1),
  mapTo('User clicked')
);

const autoAdvance$ = timer(5000).pipe(
  mapTo('Auto-advanced')
);

// ✅ Pipeable Operator version - add processing to main stream
userClick$
  .pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`),
    raceWith(autoAdvance$.pipe(
      map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
    ))
  )
  .subscribe(console.log);

// ❌ Creation Function version - becomes verbose
import { race } from 'rxjs';
race(
  userClick$.pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
  ),
  autoAdvance$.pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
  )
).subscribe(console.log);
```

### Summary

- **`race`**: Optimal for simply competing multiple streams
- **`raceWith`**: Optimal when you want to compete with other streams while transforming or processing the main stream


## ⚠️ Important Notes

### First Emission Wins

The stream with the **earliest emission timing** is adopted. Not the subscription start timing.

```ts
import { timer, of } from 'rxjs';
import { raceWith, map } from 'rxjs';

const immediate$ = of('Emit immediately');
const delayed$ = timer(1000).pipe(map(() => 'Emit after 1 second'));

immediate$
  .pipe(raceWith(delayed$))
  .subscribe(console.log);
// Output: Emit immediately
```

### All Observables are Subscribed

`raceWith` **subscribes to all Observables simultaneously**, but ignores all except the first one that emits.

```ts
import { timer } from 'rxjs';
import { raceWith, tap } from 'rxjs';

const source1$ = timer(100).pipe(
  tap(() => console.log('Source 1 emits'))
);

const source2$ = timer(200).pipe(
  tap(() => console.log('Source 2 emits'))
);

source1$
  .pipe(raceWith(source2$))
  .subscribe(console.log);
// Output:
// Source 1 emits
// 0
// Source 2 emits ← Subscribed, but value is ignored
```

### Error Handling

If there is an Observable that errors first, the entire stream terminates with an error.

```ts
import { throwError, timer } from 'rxjs';
import { raceWith, catchError } from 'rxjs';
import { of } from 'rxjs';

timer(1000).pipe(
  raceWith(
    throwError(() => new Error('Error occurred')).pipe(
      catchError(err => of('Error recovered'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Output: Error recovered
```


## 📚 Related Operators

- **[race](/en/guide/creation-functions/selection/race)** - Creation Function version
- **[mergeWith](/en/guide/operators/combination/mergeWith)** - Execute all streams in parallel
- **[concatWith](/en/guide/operators/combination/concatWith)** - Execute streams sequentially
