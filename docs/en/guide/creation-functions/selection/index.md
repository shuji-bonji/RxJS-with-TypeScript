---
description: "Creation Functions (race and partition) that select one from multiple Observables or split one Observable into multiple ones are explained. Practical use cases such as conflict handling, fastest response acquisition, stream partitioning by conditional branching, and type-safe implementation in TypeScript will be introduced."
---

# Selection and partitioning systems Creation Functions

Creation Functions select one Observable from multiple Observables or split one Observable into multiple Observables according to conditions.

## What is Creation Functions for selection and splitting?

Creation Functions for selection and partitioning differ from those for combining, and have the following roles.

- **Selection**: Selects an Observable that satisfies specific conditions from among multiple Observables.
- **Splitting**: Splitting an Observable into multiple Observables according to conditions.

These operate in the opposite direction or from a different perspective than the "combine several into one" join.

## Major Selection and Division Systems Creation Functions

| Function | Description | Use Case |
|---|---|---|
| **[race](/en/guide/creation-functions/selection/race)** | Adopt first published | Multiple data source competition |
| **[partition](/en/guide/creation-functions/selection/partition)** | Split in two by condition | Branching process for success/failure |

## Usage Criteria

### race - select the fastest Observable

race` subscribes to multiple Observables at the same time, and adopts the **first Observable** that issues a value. Observables that are not subscribed will be automatically unsubscribed.

**Use Case**:.
- Adopt the fastest response from multiple API endpoints
- Timeout handling (original process vs. timer)
- Competition between cache and actual API calls

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Adopt the fastest from multiple data sources
const fast$ = timer(1000).pipe(map(() => 'Fast API'));
const slow$ = timer(3000).pipe(map(() => 'Slow API'));

race(fast$, slow$).subscribe(console.log);
// Output: 'Fast API' (1The output is output after a second,slow$is canceled)
```

### partition - partition by condition

partition` splits an Observable into **two Observables** based on a conditional function. The return value is an array `[if true, if false]`.

**Use Case**:.
- Separation of success and failure
- Separation of even and odd numbers
- Separation of valid and invalid data

```typescript
import { of, partition } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Split into even and odd numbers
const [even$, odd$] = partition(source$, n => n % 2 === 0);

even$.subscribe(val => console.log('Even:', val));
// Output: Even: 2, Even: 4, Even: 6

odd$.subscribe(val => console.log('Odd:', val));
// Output: Odd: 1, Odd: 3, Odd: 5
```

## Convert Cold to Hot

As shown in the table above, **All Selection and Split System Creation Functions generate a Cold Observable**. Each subscription initiates an independent execution.

You can convert a Cold Observable to a Hot Observable by using a multicast operator (`share()`, `shareReplay()`, etc.).

### Practical example: sharing conflicting API requests

```typescript
import { race, timer } from 'rxjs';
import { map, share } from 'rxjs';

// ❄️ Cold - Rerun competition for each subscription
const coldRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
);

coldRace$.subscribe(val => console.log('Subscribers1:', val));
coldRace$.subscribe(val => console.log('Subscribers2:', val));
// → Each subscriber runs an independent competition (a2competition once)

// 🔥 Hot - Share competition results among subscribers
const hotRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
).pipe(share());

hotRace$.subscribe(val => console.log('Subscribers1:', val));
hotRace$.subscribe(val => console.log('Subscribers2:', val));
// → 1Share the results of one competition
```

> [!TIP]

> For more information, see [Basic Creation System - Cold to Hot Conversion](/en/guide/creation-functions/basic/#cold- to -hot-).

## Correspondence with Pipeable Operator

Creation Functions for selection and division also have a corresponding Pipeable Operator.

| Creation Function | Pipeable Operator |
|---|---|
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(source$, predicate)` | Cannot be used in a pipeline (Creation Function only) |

> [!NOTE]

> There is no Pipeable Operator version of `partition`. If you need a partition, use it as a Creation Function or split it manually using `filter` twice.

## Next Steps

To learn more about how each Creation Function works and practical examples, click on the links from the table above.

Also, learn [Creation Functions for Combinations](/en/guide/creation-functions/combination/) and [Creation Functions for Conditionals](/en/guide/creation-functions/conditional/) to understand the whole picture of Creation Functions.
