---
description: "Explique comment combiner plusieurs Observables avec les opérateurs de combinaison RxJS (Pipeable Operators). Fournit un guide de sélection par cas d'utilisation pour withLatestFrom, concatWith, mergeWith, zipWith, raceWith, etc., et leurs différences avec les Creation Functions."
---

# Opérateurs de combinaison (Pipeable Operators)

Les opérateurs de combinaison RxJS sont des outils puissants pour combiner plusieurs Observables et créer de nouveaux flux.

> [!IMPORTANT]
> Cette page traite des **Pipeable Operators (format utilisé dans les pipelines)**.
>
> Pour les **Creation Functions (format créant de nouveaux Observables à partir de plusieurs)**, voir
> [Chapitre 3 Creation Functions](/fr/guide/creation-functions/).

## Creation Functions vs Pipeable Operators

Les fonctionnalités liées à la combinaison sont fournies sous deux formes.

### Creation Functions (expliquées au Chapitre 3)

Reçoivent plusieurs Observables en arguments et créent un nouvel Observable.

```typescript
import { concat, merge, combineLatest, zip, race, forkJoin } from 'rxjs';

// Utilisation comme Creation Function
const combined$ = concat(obs1$, obs2$, obs3$);
const merged$ = merge(source1$, source2$);
```

Voir [Creation Functions](/fr/guide/creation-functions/) pour plus de détails.

### Pipeable Operators (expliqués sur cette page)

Utilisés dans `.pipe()` sur un Observable existant.

```typescript
import { concatWith, mergeWith, combineLatestWith } from 'rxjs';

// Utilisation comme Pipeable Operator
const result$ = source$.pipe(
  map(x => x * 2),
  concatWith(other$),
  filter(x => x > 10)
);
```

## Liste des Pipeable Operators

### ◾ Opérateurs traités sur cette page

|Opérateur|Description|
|---|---|
|[withLatestFrom](./withLatestFrom)|Combine avec les dernières valeurs des autres flux lors de l'émission du flux principal|
|[mergeAll](./mergeAll)|Aplatit les Higher-order Observables en parallèle|
|[concatAll](./concatAll)|Aplatit les Higher-order Observables séquentiellement|
|[switchAll](./switchAll)|Bascule vers le dernier Higher-order Observable|
|[exhaustAll](./exhaustAll)|Ignore les nouveaux Higher-order Observables pendant l'exécution|
|[combineLatestAll](./combineLatestAll)|Combine les dernières valeurs de tous les Observables internes|
|[zipAll](./zipAll)|Apparie les valeurs correspondantes de chaque Observable interne|

### ◾ Fournis principalement comme Creation Functions

Les suivants sont principalement utilisés comme Creation Functions (voir [Chapitre 3](/fr/guide/creation-functions/)).

|Fonction|Description|Version Pipeable|
|---|---|---|
|[concat](/fr/guide/creation-functions/combination/concat)|Combinaison séquentielle|`concatWith` (RxJS 7+)|
|[merge](/fr/guide/creation-functions/combination/merge)|Combinaison parallèle|`mergeWith` (RxJS 7+)|
|[combineLatest](/fr/guide/creation-functions/combination/combineLatest)|Combine les dernières valeurs|`combineLatestWith` (RxJS 7+)|
|[zip](/fr/guide/creation-functions/combination/zip)|Apparie les valeurs correspondantes|`zipWith` (RxJS 7+)|
|[race](/fr/guide/creation-functions/selection/race)|Adopte le flux le plus rapide|`raceWith` (RxJS 7+)|
|[forkJoin](/fr/guide/creation-functions/combination/forkJoin)|Attend la complétion de tous|(Pas de version Pipeable)|

## Pour aller plus loin

Pour des exemples de scénarios réels utilisant les opérateurs de combinaison,
voir [Cas d'utilisation pratiques](./practical-use-cases.md) pour plus de détails.
