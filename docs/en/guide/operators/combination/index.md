---
description: Explains how to combine multiple Observables using RxJS's combination operators (Pipeable Operators), including the use and utilization of Pipeable-style operators such as withLatestFrom.
---

# Combination Operators (Pipeable Operators)

RxJS's Combination Operators are powerful tools for combining multiple Observables to create new streams.

> [!IMPORTANT]
> This page covers **Pipeable Operators (form used in pipelines)**.
>
> For **Creation Functions (form that creates a new Observable from multiple Observables)**,
> see [Chapter 3: Creation Functions](/en/guide/creation-functions/).

## Creation Functions vs Pipeable Operators

Functions related to combination are provided in two forms.

### Creation Functions (explained in Chapter 3)

Receives multiple Observables as arguments and creates a new Observable.

```typescript
import { concat, merge, combineLatest, zip, race, forkJoin } from 'rxjs';

// Use as Creation Function
const combined$ = concat(obs1$, obs2$, obs3$);
const merged$ = merge(source1$, source2$);
```

See [Creation Functions](/en/guide/creation-functions/) for details.

### Pipeable Operators (explained on this page)

Used in `.pipe()` for existing Observable.

```typescript
import { concatWith, mergeWith, combineLatestWith } from 'rxjs';

// Use as Pipeable Operator
const result$ = source$.pipe(
  map(x => x * 2),
  concatWith(other$),
  filter(x => x > 10)
);
```

## List of Pipeable Operators

### ◾ Operators Covered on This Page

|Operator|Description|
|---|---|
|[withLatestFrom](./withLatestFrom)|Combines the latest values of other streams according to the emission of the main Observable|
|[mergeAll](./mergeAll)|Flatten Higher-order Observable in parallel|
|[concatAll](./concatAll)|Flatten Higher-order Observable sequentially|
|[switchAll](./switchAll)|Switch to the latest Higher-order Observable|
|[exhaustAll](./exhaustAll)|Ignore new Higher-order Observable during execution|
|[combineLatestAll](./combineLatestAll)|Combine the latest values of all internal Observables|
|[zipAll](./zipAll)|Pair the corresponding values of each internal Observable|

### ◾ Provided as Creation Functions

The following are mainly used as Creation Functions (see [Chapter 3](/en/guide/creation-functions/)).

|Function|Description|Pipeable Version|
|---|---|---|
|[concat](/en/guide/creation-functions/combination/concat)|Combine sequentially|`concatWith` (RxJS 7+)|
|[merge](/en/guide/creation-functions/combination/merge)|Combine in parallel|`mergeWith` (RxJS 7+)|
|[combineLatest](/en/guide/creation-functions/combination/combineLatest)|Combine latest values|`combineLatestWith` (RxJS 7+)|
|[zip](/en/guide/creation-functions/combination/zip)|Pair corresponding values|`zipWith` (RxJS 7+)|
|[race](/en/guide/creation-functions/selection/race)|Adopt the fastest stream|`raceWith` (RxJS 7+)|
|[forkJoin](/en/guide/creation-functions/combination/forkJoin)|Wait for all to complete|(No Pipeable version)|

## For Those Who Want to Learn in a More Practical Way

For realistic scenario examples using combination operators,
see [Practical Use Cases](./practical-use-cases.md) for detailed information.
