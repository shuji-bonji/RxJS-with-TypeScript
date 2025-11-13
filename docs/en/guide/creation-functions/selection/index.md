---
description: This section provides an overview of Selection and Partition Creation Functions that select one Observable from multiple Observables or split one Observable into multiple Observables. It explains how to use race and partition, as well as practical examples.
---

# Selection/Partition Creation Functions

These are Creation Functions for selecting one Observable from multiple Observables or splitting one Observable into multiple Observables.

## What are Selection/Partition Creation Functions?

Selection/Partition Creation Functions are a set of functions that compete among multiple Observables to select the fastest one, or split an Observable into two streams based on conditions. This is useful for competing data sources or allocating processing based on conditions.

Check the table below to see the characteristics and usage of each Creation Function.

## Major Selection/Partition Creation Functions

| Function | Description | Use Cases |
|----------|------|-------------|
| **[race](/en/guide/creation-functions/selection/race)** | Select the fastest Observable (the one that emits first) | Competition among multiple data sources, fallback processing |
| **[partition](/en/guide/creation-functions/selection/partition)** | Split into two Observables based on a condition | Success/failure handling, branching based on conditions |

## Usage Criteria

The selection of Selection/Partition Creation Functions is determined from the following perspectives.

### 1. Purpose

- **Select fastest from multiple sources**: `race` - Select the first one that emits a value among multiple data sources
- **Split by condition**: `partition` - Split one Observable into two streams based on a condition

### 2. Emission Timing

- **Only the fastest**: `race` - Once selected, other Observable values are ignored
- **Classify all values**: `partition` - All values are sorted into two streams according to conditions

### 3. Timing of Completion

- **Depends on selected Observable**: `race` - Follows completion of the Observable that emitted first
- **Depends on original Observable**: `partition` - Both streams complete when the original Observable completes

## Practical Usage Examples

### race() - Select the Fastest from Multiple Data Sources

If you have multiple data sources and want to use the fastest responding one, use `race()`.

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Simulate multiple APIs
const api1$ = timer(1000).pipe(map(() => 'API1 Response'));
const api2$ = timer(500).pipe(map(() => 'API2 Response'));
const api3$ = timer(1500).pipe(map(() => 'API3 Response'));

// Use the fastest response
race(api1$, api2$, api3$).subscribe(console.log);
// Output: API2 Response (fastest at 500ms)
```

### partition() - Split into Two Based on Condition

If you want to split one Observable into two streams based on a condition, use `partition()`.

```typescript
import { of } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// Split into even and odd numbers
const [evens$, odds$] = partition(numbers$, n => n % 2 === 0);

evens$.subscribe(n => console.log('Even:', n));
// Output: Even: 2, Even: 4, Even: 6, Even: 8, Even: 10

odds$.subscribe(n => console.log('Odd:', n));
// Output: Odd: 1, Odd: 3, Odd: 5, Odd: 7, Odd: 9
```

## Converting Cold to Hot

As shown in the table above, **all Selection/Partition Creation Functions generate Cold Observables**. Independent execution is initiated for each subscription.

However, by using multicast operators (`share()`, `shareReplay()`, etc.), you can **convert a Cold Observable to a Hot Observable**.

### Practical Example: Sharing Execution

```typescript
import { race, timer, share } from 'rxjs';
import { map } from 'rxjs';

// ❄️ Cold - Independent execution for each subscription
const coldRace$ = race(
  timer(1000).pipe(map(() => 'API1')),
  timer(500).pipe(map(() => 'API2'))
);

coldRace$.subscribe(val => console.log('Subscriber 1:', val));
coldRace$.subscribe(val => console.log('Subscriber 2:', val));
// → Each subscriber executes independent race (2x requests)

// 🔥 Hot - Share execution among subscribers
const hotRace$ = race(
  timer(1000).pipe(map(() => 'API1')),
  timer(500).pipe(map(() => 'API2'))
).pipe(share());

hotRace$.subscribe(val => console.log('Subscriber 1:', val));
hotRace$.subscribe(val => console.log('Subscriber 2:', val));
// → Share race execution (requests only once)
```

> [!TIP]
> **Cases where Hot conversion is required**:
> - Share the result of `race()` among multiple components
> - Use the result of `partition()` in multiple locations
> - Execute high-cost processing only once
>
> For more information, see [Basic Creation - Converting Cold to Hot](/en/guide/creation-functions/basic/#converting-cold-to-hot).

## Correspondence with Pipeable Operator

For Selection/Partition Creation Functions, there is a corresponding Pipeable Operator. When used in a pipeline, the `~With` type operator is used.

| Creation Function | Pipeable Operator |
|-------------------|-------------------|
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(source$, predicate)` | No direct correspondence (use as Creation Function) |

> [!NOTE]
> `partition()` is typically used as a Creation Function. To perform stream splitting within a pipeline, use operators such as `filter()` in combination.

## Next Steps

To learn the detailed behavior and practical examples of each Creation Function, click on the links from the table above.

Also, by learning [Basic Creation Functions](/en/guide/creation-functions/basic/), [Combination Creation Functions](/en/guide/creation-functions/combination/), and [Conditional Creation Functions](/en/guide/creation-functions/conditional/), you can understand the whole picture of Creation Functions.
