---
description: This section explains how to combine multiple Observables in sequence with the concat Creation Function and how to utilize it for step execution and UI display.
---

# concat - concatenate streams in sequence

`concat` is a Creation Function that **sequentially executes** multiple Observables in the specified order.
The next Observable is issued after the previous Observable is `complete`.

## Basic syntax and usage

```ts
import { concat, of, delay } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));

concat(obs1$, obs2$).subscribe(console.log);
// Output: A → B → C → D
```

- After all `obs1$` have been issued, `obs2$` will begin to be issued.
- The key point is that the streams are not executed simultaneously, but "in order".

[🌐 RxJS Official Documentation - `concat`](https://rxjs.dev/api/index/function/concat)


## Typical utilization patterns

- **Step-by-step processing**: When you want to proceed to the next step after the previous step is completed.
- **Order-guaranteed API requests**: Asynchronous operations that need to be performed in sequence.
- **Control of UI events** where order is important, such as animations and notifications

## Practical code examples (with UI)

This is an example of **displaying loading messages and data lists in sequential order**.

```ts
import { concat, of, timer } from 'rxjs';
import { map, take } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>concat practical example:</h3>';
document.body.appendChild(output);

// Loading stream
const loading$ = timer(0, 1000).pipe(
  map((count) => `⏳ Loading... (${count + 1}s)`),
  take(3) // Issue only for 3 seconds
);

// Data list stream
const data$ = of('🍎 Apple', '🍌 Banana', '🍇 Grape');

// concat and display in order
concat(loading$, data$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- The loading message is displayed three times first,
- The loading message is displayed three times first, and then the data list is displayed in order.
- By using **concat**, a natural "step-by-step" display can be easily achieved.


## Related Operators

- **[concatWith](/en/guide/operators/combination/concatWith)** - Pipeable Operator version (used in pipeline)
- **[concatMap](/en/guide/operators/transformation/concatMap)** - map and concatenate each value sequentially
- **[merge](/en/guide/creation-functions/combination/merge)** - concatenate Creation Function
