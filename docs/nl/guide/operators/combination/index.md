---
description: "RxJS-combinatie-operators (Pipeable Operators) gebruiken om meerdere Observables te combineren. Uitleg over withLatestFrom, concatWith, mergeWith, zipWith, raceWith en meer, verschillen met Creation Functions, en selectiegids per use case."
---

# Combinatie-operators (Pipeable Operators)

RxJS-combinatie-operators zijn krachtige tools voor het combineren van meerdere Observables tot nieuwe streams.

> [!IMPORTANT]
> Deze pagina behandelt **Pipeable Operators (formaat voor gebruik in pipelines)**.
>
> Voor **Creation Functions (formaat dat nieuwe Observable creëert uit meerdere Observables)**,
> zie [Hoofdstuk 3 Creation Functions](/nl/guide/creation-functions/).

## Creation Functions vs Pipeable Operators

Combinatie-gerelateerde functionaliteit wordt in twee formaten aangeboden.

### Creation Functions (uitgelegd in Hoofdstuk 3)

Neemt meerdere Observables als argumenten en creëert nieuwe Observable.

```typescript
import { concat, merge, combineLatest, zip, race, forkJoin } from 'rxjs';

// Gebruikt als Creation Function
const combined$ = concat(obs1$, obs2$, obs3$);
const merged$ = merge(source1$, source2$);
```

Zie [Creation Functions](/nl/guide/creation-functions/) voor details.

### Pipeable Operators (uitgelegd op deze pagina)

Gebruikt binnen `.pipe()` op bestaande Observable.

```typescript
import { concatWith, mergeWith, combineLatestWith } from 'rxjs';

// Gebruikt als Pipeable Operator
const result$ = source$.pipe(
  map(x => x * 2),
  concatWith(other$),
  filter(x => x > 10)
);
```

## Pipeable Operators overzicht

### ◾ Operators behandeld op deze pagina

|Operator|Beschrijving|
|---|---|
|[withLatestFrom](./withLatestFrom)|Combineert nieuwste waarden van andere streams bij uitgifte van hoofd-Observable|
|[mergeAll](./mergeAll)|Vlakt Higher-order Observable parallel af|
|[concatAll](./concatAll)|Vlakt Higher-order Observable sequentieel af|
|[switchAll](./switchAll)|Schakelt naar nieuwste Higher-order Observable|
|[exhaustAll](./exhaustAll)|Negeert nieuwe Higher-order Observable tijdens uitvoering|
|[combineLatestAll](./combineLatestAll)|Combineert nieuwste waarden van alle interne Observables|
|[zipAll](./zipAll)|Koppelt corresponderende waarden van elke interne Observable|

### ◾ Aangeboden als Creation Functions

De volgende worden voornamelijk gebruikt als Creation Function (zie [Hoofdstuk 3](/nl/guide/creation-functions/)).

|Function|Beschrijving|Pipeable versie|
|---|---|---|
|[concat](/nl/guide/creation-functions/combination/concat)|Sequentiële combinatie|`concatWith` (RxJS 7+)|
|[merge](/nl/guide/creation-functions/combination/merge)|Parallelle combinatie|`mergeWith` (RxJS 7+)|
|[combineLatest](/nl/guide/creation-functions/combination/combineLatest)|Combineert nieuwste waarden|`combineLatestWith` (RxJS 7+)|
|[zip](/nl/guide/creation-functions/combination/zip)|Koppelt corresponderende waarden|`zipWith` (RxJS 7+)|
|[race](/nl/guide/creation-functions/selection/race)|Kiest snelste stream|`raceWith` (RxJS 7+)|
|[forkJoin](/nl/guide/creation-functions/combination/forkJoin)|Wacht op voltooiing van alle|（geen Pipeable versie）|

## Verder leren met praktische voorbeelden

Voor realistische scenario's met combinatie-operators,
zie [Praktische use cases](./practical-use-cases.md) voor gedetailleerde voorbeelden.
