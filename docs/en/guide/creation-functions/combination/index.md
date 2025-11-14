---
description: This section describes the Creation Functions that combine multiple Observables into one, and teaches how to use concat, merge, combineLatest, zip, and forkJoin, as well as practical examples.
---

# Combination Creation Functions

This is the main Creation Functions for combining multiple Observables into one Observable.

## What are Combination Creation Functions?

Combination Creation Functions take multiple Observables and combine them into a single Observable stream. The timing and order in which values are issued depends on the combination method.

The table below shows the characteristics of each Creation Function and how to use them.

## Major Combination Creation Functions

| Function | Description | Use Cases |
|----------|------|-------------|
| **[concat](/en/guide/creation-functions/combination/concat)** | Sequential combination (next starts after previous completes) | Step-by-step processing |
| **[merge](/en/guide/creation-functions/combination/merge)** | Parallel combination (subscribe simultaneously, output in emission order) | Integration of multiple events |
| **[combineLatest](/en/guide/creation-functions/combination/combineLatest)** | Combine latest values | Form input synchronization |
| **[zip](/en/guide/creation-functions/combination/zip)** | Pair corresponding values | Matching requests with responses |
| **[forkJoin](/en/guide/creation-functions/combination/forkJoin)** | Wait for all to complete and combine final values | Waiting for parallel API calls to complete |

## Usage Criteria

The selection of Combination Creation Functions is determined from the following perspectives:

### 1. Execution Timing

- **Sequential execution**: `concat` - Start the next one after the previous Observable completes
- **Parallel execution**: `merge`, `combineLatest`, `zip`, `forkJoin` - Subscribe to all Observables simultaneously

### 2. How to Emit Values

- **Emit all values**: `concat`, `merge` - Output all values emitted from each Observable
- **Combine latest values**: `combineLatest` - Combine all latest values whenever one of them emits
- **Pair corresponding values**: `zip` - Pair values from corresponding positions in each Observable and emit
- **Final values only**: `forkJoin` - Emit each final value as an array when all Observables are complete

### 3. Timing of Completion

- **After all complete**: `concat`, `forkJoin` - Wait until all Observables have completed
- **Completes with shortest stream**: `zip` - Complete when any one completes, since remaining values cannot form pairs
- **Does not complete**: `merge`, `combineLatest` - If one completes while the other continues, it will not complete

## Converting Cold to Hot

As shown in the table above, **all Combination Creation Functions generate Cold Observables**. Each subscription initiates an independent execution.

However, you can **convert a Cold Observable to a Hot Observable** by using a multicast operator (`share()`, `shareReplay()`, `publish()`, etc.).

### Practical Example: Sharing HTTP Requests

```typescript
import { merge, interval } from 'rxjs';
import { map, take, share } from 'rxjs';

// ❄️ Cold - Independent HTTP requests for each subscription
const coldApi$ = merge(
  interval(1000).pipe(map(() => 'Source A'), take(3)),
  interval(1500).pipe(map(() => 'Source B'), take(2))
);

coldApi$.subscribe(val => console.log('Subscriber 1:', val));
coldApi$.subscribe(val => console.log('Subscriber 2:', val));
// → Each subscriber executes independent intervals (2x requests)

// 🔥 Hot - Share execution among subscribers
const hotApi$ = merge(
  interval(1000).pipe(map(() => 'Source A'), take(3)),
  interval(1500).pipe(map(() => 'Source B'), take(2))
).pipe(share());

hotApi$.subscribe(val => console.log('Subscriber 1:', val));
hotApi$.subscribe(val => console.log('Subscriber 2:', val));
// → Share one interval (requests only once)
```

> [!TIP]
> **Cases where Hot conversion is required**:
> - Multiple components share the same API results
> - Use `forkJoin` to use the results of parallel requests in multiple locations
> - Manage state with `combineLatest` and distribute to multiple subscribers
>
> For more information, see [Basic Creation - Converting Cold to Hot](/en/guide/creation-functions/basic/#converting-cold-to-hot).

## Correspondence with Pipeable Operator

For Combination Creation Functions, there is a corresponding Pipeable Operator. When used in a pipeline, the `~With` type operator is used.

| Creation Function | Pipeable Operator |
|-------------------|-------------------|
| `concat(a$, b$)` | `a$.pipe(concatWith(b$))` |
| `merge(a$, b$)` | `a$.pipe(mergeWith(b$))` |
| `zip(a$, b$)` | `a$.pipe(zipWith(b$))` |
| `combineLatest([a$, b$])` | `a$.pipe(combineLatestWith(b$))` |

## Next Steps

To learn the detailed behavior and practical examples of each Creation Function, click on the links from the table above.

Also, learn [Selection/Partition Creation Functions](/en/guide/creation-functions/selection/) and [Conditional Creation Functions](/en/guide/creation-functions/conditional/), you can understand the whole picture of Creation Functions.
