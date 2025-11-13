---
description: The zip Creation Function aligns and pairs the values in the corresponding order from multiple Observables and outputs them at the same time all sources have issued their values one by one.
---

# zip - pair corresponding values

`zip` is a Creation Function that groups together **corresponding ordered values** issued from multiple Observables and outputs them as an array or tuple.
It waits for values to arrive from all source Observables, one at a time, and creates pairs when they are ready.


## Basic syntax and usage

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = of('A', 'B', 'C');
const source2$ = interval(1000).pipe(
  map((val) => val * 10),
  take(3)
);

zip(source1$, source2$).subscribe(([letter, number]) => {
  console.log(letter, number);
});

// Output:
// A 0
// B 10
// C 20
```

- When each Observable issues one value at a time, a pair is created and output.
- If one is delayed, it will wait until both are aligned.

[🌐 RxJS Official Documentation - `zip`](https://rxjs.dev/api/index/function/zip)


## Typical utilization patterns

- **Mapping requests to responses**
- **Synchronously pair IDs with corresponding data**
- **Combine multiple streams processed in parallel into one set**


## Practical code examples (with UI)

Example of **combining and displaying** different data sources (fruit and price).

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>zip practical example:</h3>';
document.body.appendChild(output);

// Fruit name stream
const fruits$ = of('🍎 Apple', '🍌 Banana', '🍇 Grape');

// Price stream (issued every 2 seconds)
const prices$ = interval(2000).pipe(
  map((i) => [100, 200, 300][i]),
  take(3)
);

// zip and display
zip(fruits$, prices$).subscribe(([fruit, price]) => {
  const item = document.createElement('div');
  item.textContent = `${fruit} - ¥${price}`;
  output.appendChild(item);
});
```

- Fruit and price lists are paired and displayed **when** they are aligned in a one-to-one correspondence.
- If either is missing, it will not be output at that point.


## Related Operators

- **[zipWith](/en/guide/operators/combination/zipWith)** - Pipeable Operator version (used in pipeline)
- **[combineLatest](/en/guide/creation-functions/combination/combineLatest)** - Combine the latest values Creation Function
- **[withLatestFrom](/en/guide/operators/combination/withLatestFrom)** - only mainstream triggers
