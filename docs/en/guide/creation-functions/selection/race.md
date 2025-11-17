---
description: The race Creation Function achieves a special concatenation process that adopts only the first stream that issues a value out of multiple Observables and ignores the others thereafter.
titleTemplate: ':title | RxJS'
---

# race - adopt the stream that issued the value first

`race` is a special concatenated Creation Function that takes advantage of **only the first Observable that issues a value** out of multiple Observables,
and ignores the other Observables.


## Basic syntax and usage

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Slow (5 seconds)'));
const fast$ = timer(2000).pipe(map(() => 'Fast (2 seconds)'));

race(slow$, fast$).subscribe(console.log);
// Output: Fast (2 seconds)
```

- Only the Observable that issued the value first is the winner and continues with subsequent streams.
- Other Observables are ignored.

[🌐 RxJS Official Documentation - `race`](https://rxjs.dev/api/index/function/race)


## Typical utilization patterns

- **Process the earlier of multiple user actions (clicks, keystrokes, scrolling)**
- **Adopt the earlier of multiple triggers, such as manual sending and automatic saving**
- **Display the first completed data first among multiple data acquisition processes**

## Practical code examples (with UI)

Simulates a race to adopt only the first one issued from three streams firing at different times.

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>race practical example:</h3>';
document.body.appendChild(output);

// Different timing Observables
const slow$ = timer(5000).pipe(map(() => 'Slow (after 5 seconds)'));
const medium$ = timer(3000).pipe(map(() => 'Medium (after 3 seconds)'));
const fast$ = timer(2000).pipe(map(() => 'Fast (after 2 seconds)'));

const startTime = Date.now();

// Race start message
const waiting = document.createElement('div');
waiting.textContent = 'Race started... Waiting for the first stream to be issued.';
output.appendChild(waiting);

// Run race
race(slow$, medium$, fast$).subscribe(winner => {
  const endTime = Date.now();
  const elapsed = ((endTime - startTime) / 1000).toFixed(2);

  const result = document.createElement('div');
  result.innerHTML = `<strong>Winner:</strong> ${winner} (Elapsed time: ${elapsed} seconds)`;
  result.style.color = 'green';
  result.style.marginTop = '10px';
  output.appendChild(result);

  const explanation = document.createElement('div');
  explanation.textContent = '※ Only the first Observable that issued a value is selected.';
  explanation.style.marginTop = '5px';
  output.appendChild(explanation);
});
```

- After 2 seconds, the first `fast$` is issued and thereafter only `fast$` is output.
- Other `medium$` and `slow$` issues will be ignored.


## Related Operators

- **[raceWith](/en/guide/operators/combination/raceWith)** - Pipeable Operator version (used in pipeline)
- **[timeout](/en/guide/operators/utility/timeout)** - Timeout-only operator
- **[merge](/en/guide/creation-functions/combination/merge)** - Merge all streams Creation Function
