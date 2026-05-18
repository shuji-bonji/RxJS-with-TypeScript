---
description: "raceWith is a RxJS join operator that adopts only the first stream of the original Observable and other Observables that issued values. It is ideal for timeout implementations, fallback operations, fastest adoption from multiple data sources, and other situations where selection by race condition is required."
---

# raceWith - adopt the fastest stream

The `raceWith` operator **adopts only the first Observable that issues a value** out of the original Observable and any other Observables specified,
After that, it ignores the other Observables.
This is the Pipeable Operator version of Creation Function's `race`.

## 🔰 Basic Syntax and Usage

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Slowly (5Seconds)'));
const medium$ = timer(3000).pipe(map(() => 'Normal (3Seconds)'));
const fast$ = timer(2000).pipe(map(() => 'Fast (2Seconds)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Output: Fast (2Seconds)
```

- Only the Observable that issued the value first (`fast$` in this example) is the winner and continues with subsequent streams.
- Other Observables are ignored.

[🌐 RxJS Official Documentation - `raceWith`](https://rxjs.dev/api/operators/raceWith)

## 💡 Typical utilization pattern

- **Timeout implementation**: race the timeout timer against the main process
- **Fallback processing**: adopt the fastest one from multiple data sources
- **User Interaction**: Adopt the faster of click and auto-progress

## 🧠 Practical Code Examples (with UI)

This is an example of competing manual click and auto-progress timers and adopting the faster one.

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

// Output Area Creation
const output = document.createElement('div');
output.innerHTML = '<h3>raceWith Practical examples of:</h3>';
document.body.appendChild(output);

// Button creation
const button = document.createElement('button');
button.textContent = 'Proceed manually (5Click within seconds)';
document.body.appendChild(button);

// Waiting message
const waiting = document.createElement('div');
waiting.textContent = '5Click button within seconds or wait for automatic advance...';
waiting.style.marginTop = '10px';
output.appendChild(waiting);

// Manual Click Stream
const manualClick$ = fromEvent(button, 'click').pipe(
  take(1),
  map(() => '👆 Manual click has been selected！')
);

// Auto advance timer (5(after 2 seconds)
const autoProgress$ = timer(5000).pipe(
  map(() => '⏰ Auto progress has been selected！')
);

// Race Execution
manualClick$
  .pipe(raceWith(autoProgress$))
  .subscribe((winner) => {
    waiting.remove();
    button.disabled = true;

    const result = document.createElement('div');
    result.innerHTML = `<strong>${winner}</strong>`;
    result.style.color = 'green';
    result.style.fontSize = '18px';
    result.style.marginTop = '10px';
    output.appendChild(result);
  });
```

- If the button is clicked within 5 seconds, the manual click is adopted.
- After 5 seconds, the automatic progression is adopted.
- **First one to do so is the winner**, and the later one is ignored.

## 🔄 Difference from Creation Function `race`.

### Basic differences

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Slowly (5Seconds)'));
const medium$ = timer(3000).pipe(map(() => 'Normal (3Seconds)'));
const fast$ = timer(2000).pipe(map(() => 'Fast (2Seconds)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Output: Fast (2Seconds)
```

### Specific examples of usage

**If you only need a simple race, Creation Function is the way to go**.


```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const server1$ = timer(3000).pipe(map(() => 'Response from server1Response from'));
const server2$ = timer(2000).pipe(map(() => 'Response from server2Response from'));
const server3$ = timer(4000).pipe(map(() => 'Response from server3Response from'));

// Simple and easy to read
race(server1$, server2$, server3$).subscribe(response => {
  console.log('Adopted:', response);
});
// Output: Adopted: Response from server2Response from (Fastest2Seconds)
```

**If you want to add a conversion process to the main stream, Pipeable Operator is recommended**.

```ts
import { fromEvent, timer, of } from 'rxjs';
import { raceWith, map, switchMap, catchError } from 'rxjs';

const searchButton = document.createElement('button');
searchButton.textContent = 'Search';
document.body.appendChild(searchButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Mainstream: User search requests
const userSearch$ = fromEvent(searchButton, 'click').pipe(
  switchMap(() => {
    output.textContent = 'Searching...';

    // APISimulate a call (3(takes seconds)
    return timer(3000).pipe(
      map(() => '🔍 Search results: 100Hits per page'),
      catchError((err: unknown) => of('❌ Error occurred'))
    );
  })
);

// ✅ Pipeable OperatorEdition - Complete in one pipeline
userSearch$
  .pipe(
    raceWith(
      // Timeout (2seconds)
      timer(2000).pipe(
        map(() => '⏱️ Timeout: Search is taking a long time')
      )
    )
  )
  .subscribe(result => {
    output.textContent = result;
  });

// ❌ Creation FunctionEdition - Mainstream needs to be written separately
import { race } from 'rxjs';
race(
  userSearch$,
  timer(2000).pipe(
    map(() => '⏱️ Timeout: Search is taking a long time')
  )
).subscribe(result => {
  output.textContent = result;
});
```

**Implementation of fallback processing**

```ts
import { timer, throwError } from 'rxjs';
import { raceWith, map, mergeMap, catchError, delay } from 'rxjs';
import { of } from 'rxjs';

// UICreate
const output = document.createElement('div');
output.innerHTML = '<h3>Data acquisition (with fallback)</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Start of data acquisition';
document.body.appendChild(button);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
output.appendChild(statusArea);

button.addEventListener('click', () => {
  statusArea.textContent = 'Acquisition in progress...';

  // MainAPI(priority)：Time々Failure
  const mainApi$ = timer(1500).pipe(
    mergeMap(() => {
      const success = Math.random() > 0.5;
      if (success) {
        return of('✅ MainAPISuccessful acquisition from');
      } else {
        return throwError(() => new Error('MainAPIFailure'));
      }
    }),
    catchError((err: unknown) => {
      console.log('MainAPIFailure, go to fallback...');
      // Delay on error and yield to fallback
      return of('').pipe(delay(10000));
    })
  );

  // ✅ Pipeable OperatorEdition - MainAPIAdd fallback to
  mainApi$
    .pipe(
      raceWith(
        // BackupAPI(fallback)：Slightly slower, but reliable
        timer(2000).pipe(
          map(() => '🔄 BackupAPIRetrieve from')
        )
      )
    )
    .subscribe(result => {
      if (result) {
        statusArea.textContent = result;
        statusArea.style.color = result.includes('Main') ? 'green' : 'orange';
      }
    });
});
```

**Adopt fastest from multiple data sources** ```ts
import { timer, throwError } from 'rxjs';
import { raceWith, map, mergeMap, catchError, delay } from 'rxjs';
import { of } from 'rxjs';

// UICreate
const output = document.createElement('div');
output.innerHTML = '<h3>Data acquisition (with fallback)</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Start of data acquisition';
document.body.appendChild(button);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
output.appendChild(statusArea);

button.addEventListener('click', () => {
  statusArea.textContent = 'Acquisition in progress...';

  // MainAPI(priority)：Time々Failure
  const mainApi$ = timer(1500).pipe(
    mergeMap(() => {
      const success = Math.random() > 0.5;
      if (success) {
        return of('✅ MainAPISuccessful acquisition from');
      } else {
        return throwError(() => new Error('MainAPIFailure'));
      }
    }),
    catchError((err: unknown) => {
      console.log('MainAPIFailure, go to fallback...');
      // Delay on error and yield to fallback
      return of('').pipe(delay(10000));
    })
  );

  // ✅ Pipeable OperatorEdition - MainAPIAdd fallback to
  mainApi$
    .pipe(
      raceWith(
        // BackupAPI(fallback)：Slightly slower, but reliable
        timer(2000).pipe(
          map(() => '🔄 BackupAPIRetrieve from')
        )
      )
    )
    .subscribe(result => {
      if (result) {
        statusArea.textContent = result;
        statusArea.style.color = result.includes('Main') ? 'green' : 'orange';
      }
    });
});
```

```

```ts
import { timer, fromEvent } from 'rxjs';
import { raceWith, map, mergeMap } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>MultipleCDNFastest load from</h3>';
document.body.appendChild(output);

const loadButton = document.createElement('button');
loadButton.textContent = 'Loading libraries';
document.body.appendChild(loadButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
output.appendChild(result);

fromEvent(loadButton, 'click').pipe(
  mergeMap(() => {
    result.textContent = 'Loading...';

    // CDN1Loading (simulate) from
    const cdn1$ = timer(Math.random() * 3000).pipe(
      map(() => ({ source: 'CDN1 (US)', data: 'library.js' }))
    );

    // ✅ Pipeable OperatorEdition - CDN1as main and otherCDNas competitors
    return cdn1$.pipe(
      raceWith(
        // CDN2Loading (simulate) from
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN2 (EU)', data: 'library.js' }))
        ),
        // CDN3Loading (simulate) from
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN3 (Asia)', data: 'library.js' }))
        )
      )
    );
  })
).subscribe(response => {
  result.innerHTML = `
    <strong>✅ Loading completed</strong><br>
    Acquired from: ${response.source}<br>
    file: ${response.data}
  `;
  result.style.color = 'green';
});
```

### Summary

- **`race`**: best if you simply want to adopt the fastest one from multiple streams
- **`raceWith`**: best if you want to implement timeouts and fallbacks while transforming and processing the main stream

## ⚠️ Notes.

### Example timeout implementation

Implementing timeout processing using `raceWith`.

```ts
import { of, timer, throwError } from 'rxjs';
import { raceWith, delay, mergeMap } from 'rxjs';

// Time-consuming process (3seconds)
const slowRequest$ = of('Data acquisition succeeded').pipe(delay(3000));

// Timeout (2seconds)
const timeout$ = timer(2000).pipe(
  mergeMap(() => throwError(() => new Error('Timeout')))
);

slowRequest$
  .pipe(raceWith(timeout$))
  .subscribe({
    next: console.log,
    error: err => console.error(err.message)
  });
// Output: Timeout
```

### all streams are subscribed to

`raceWith` will subscribe to all Observables until a winner is determined.
After the winner is determined, the losing Observable is automatically unsubscribed.

```ts
import { timer } from 'rxjs';
import { raceWith, tap, map } from 'rxjs';

const slow$ = timer(3000).pipe(
  tap(() => console.log('slow$ Firing')),
  map(() => 'slow')
);

const fast$ = timer(1000).pipe(
  tap(() => console.log('fast$ Firing')),
  map(() => 'fast')
);

slow$.pipe(raceWith(fast$)).subscribe(console.log);
// Output:
// fast$ Firing
// fast
// (slow$is1Unsubscribed at second,3seconds and does not fire after)
```

### For a synchronous Observable.

If everything is issued synchronously, the first one registered is the winner.

```ts
import { of } from 'rxjs';
import { raceWith } from 'rxjs';

of('A').pipe(
  raceWith(of('B'), of('C'))
).subscribe(console.log);
// Output: A (since it was first subscribed to)
```

## 📚 Related Operators

- **[race](/en/guide/creation-functions/selection/race)** - Creation Function version
- **[timeout](/en/guide/operators/utility/timeout)** - Timeout-only operator
- **[mergeWith](/en/guide/operators/combination/mergeWith)** - Merge all streams
