---
description: "Une explication complète des fonctions de création RxJS (fonctions de création d'Observable), incluant les différences avec les opérateurs pipables, l'utilisation de base, et sept catégories (création de base, génération de boucles, communication HTTP, combinaison, sélection/partition, branchement conditionnel et systèmes de contrôle)."
---

# Fonctions de création

Dans RxJS, il existe deux formes différentes : **Fonctions de création** pour créer des Observables et **Opérateurs pipables** pour convertir des Observables existants.

Cette page décrit les concepts de base des fonctions de création et les sept catégories principales.

## Qu'est-ce qu'une fonction de création ?

**Les fonctions de création** sont des fonctions permettant de créer de nouveaux Observables.

```typescript
import { of, from, interval } from 'rxjs';

// Utilisation comme fonctions de création
const obs1$ = of(1, 2, 3);
const obs2$ = from([4, 5, 6]);
const obs3$ = interval(1000);
```

Elles sont importées directement du package `rxjs` et appelées en tant que fonctions pour créer des Observables.

## Différence avec l'opérateur pipable

Les fonctions de création et les opérateurs pipables ont des utilisations et des applications différentes. Voir le tableau ci-dessous pour comprendre les différences.

| Caractéristique | Fonction de création | Opérateur pipable |
|------|-------------------|-------------------|
| **Objectif** | Créer un nouvel Observable | Transformer un Observable existant |
| **Importation depuis** | `rxjs` | `rxjs/operators` |
| **Utilisation** | Appeler directement comme fonction | Utiliser à l'intérieur de `.pipe()` |
| **Exemple** | `concat(obs1$, obs2$)` | `obs1$.pipe(concatWith(obs2$))` |

### Exemple de fonction de création

Les fonctions de création sont utilisées pour combiner directement plusieurs Observables.

```typescript
import { concat, of } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Utilisation comme fonction de création
concat(obs1$, obs2$).subscribe(console.log);
// Sortie: 1, 2, 3, 4, 5, 6
```

### Exemple d'opérateur pipable

L'opérateur pipable est utilisé pour ajouter un processus de conversion à un Observable existant.

```typescript
import { of } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Utilisation comme opérateur pipable
obs1$.pipe(
  concatWith(obs2$)
).subscribe(console.log);
// Sortie: 1, 2, 3, 4, 5, 6
```

## Critères d'utilisation

Le choix entre fonction de création et opérateur pipable est déterminé par les critères suivants.

### Quand utiliser une fonction de création

La fonction de création convient lorsque plusieurs Observables doivent être opérés au même niveau ou lorsqu'un Observable doit être créé à partir de zéro.

- **Lors de la combinaison de plusieurs Observables au même niveau**
  ```typescript
  concat(obs1$, obs2$, obs3$)
  merge(click$, hover$, scroll$)
  ```

- **Lors de la création d'un Observable à partir de zéro**
  ```typescript
  of(1, 2, 3)
  from([1, 2, 3])
  interval(1000)
  ```

### Quand utiliser un opérateur pipable

L'opérateur pipable convient pour ajouter un traitement à un Observable existant ou pour enchaîner plusieurs opérations.

- **Lors de l'ajout d'opérations à un Observable existant**
  ```typescript
  obs1$.pipe(
    map(x => x * 2),
    concatWith(obs2$),
    filter(x => x > 5)
  )
  ```

- **Lors de l'enchaînement de plusieurs opérations comme pipeline**

## Catégories de fonctions de création

Dans ce chapitre, les fonctions de création sont divisées en sept catégories.

### Liste de toutes les catégories

Le tableau ci-dessous présente toutes les catégories et les fonctions qu'elles contiennent. Cliquez sur le nom de chaque fonction pour accéder à la page détaillée.

| Catégorie | Description | Fonctions principales | Cas d'utilisation typiques |
|---------|------|-----------|-------------------|
| **[Création de base](/fr/guide/creation-functions/basic/)** | Fonctions les plus basiques et les plus utilisées. Création d'Observables basés sur données, tableaux, événements et temps | [of](/fr/guide/creation-functions/basic/of), [from](/fr/guide/creation-functions/basic/from), [fromEvent](/fr/guide/creation-functions/basic/fromEvent), [interval](/fr/guide/creation-functions/basic/interval), [timer](/fr/guide/creation-functions/basic/timer) | Tests avec valeurs fixes, streaming de données existantes, gestion d'événements DOM, polling, exécution différée |
| **[Génération de boucles](/fr/guide/creation-functions/loop/)** | Exprimer le traitement de boucles comme for/while dans Observable | [range](/fr/guide/creation-functions/loop/range), [generate](/fr/guide/creation-functions/loop/generate) | Génération de nombres séquentiels, traitement par lots, transitions d'état complexes, calculs mathématiques |
| **[Communication HTTP](/fr/guide/creation-functions/http-communication/)** | Gérer la communication HTTP comme Observable | [ajax](/fr/guide/creation-functions/http-communication/ajax), [fromFetch](/fr/guide/creation-functions/http-communication/fromFetch) | Communication HTTP basée sur XMLHttpRequest, communication HTTP basée sur Fetch API, appels REST API |
| **[Combinaison](/fr/guide/creation-functions/combination/)** | Combiner plusieurs Observables en un seul. Le moment et l'ordre d'émission diffèrent selon la méthode | [concat](/fr/guide/creation-functions/combination/concat), [merge](/fr/guide/creation-functions/combination/merge), [combineLatest](/fr/guide/creation-functions/combination/combineLatest), [zip](/fr/guide/creation-functions/combination/zip), [forkJoin](/fr/guide/creation-functions/combination/forkJoin) | Traitement étape par étape, intégration d'événements multiples, synchronisation d'entrées de formulaire, attente d'achèvement d'appels API parallèles |
| **[Sélection/Partition](/fr/guide/creation-functions/selection/)** | Sélectionner un parmi plusieurs Observables ou partitionner un Observable en plusieurs | [race](/fr/guide/creation-functions/selection/race), [partition](/fr/guide/creation-functions/selection/partition) | Compétition entre sources de données, branchement succès/échec |
| **[Conditionnel](/fr/guide/creation-functions/conditional/)** | Sélectionner un Observable selon des conditions ou générer dynamiquement à l'abonnement | [iif](/fr/guide/creation-functions/conditional/iif), [defer](/fr/guide/creation-functions/conditional/defer) | Branchement selon statut de connexion, création dynamique d'Observable, évaluation paresseuse |
| **[Contrôle](/fr/guide/creation-functions/control/)** | Contrôler le timing d'exécution de l'Observable et la gestion des ressources | [scheduled](/fr/guide/creation-functions/control/scheduled), [using](/fr/guide/creation-functions/control/using) | Contrôle du timing avec planificateur, gestion du cycle de vie des ressources, prévention des fuites de mémoire |

> [!TIP]
> **Ordre d'apprentissage**
>
> Nous recommandons aux débutants d'apprendre dans l'ordre suivant :
> 1. **Création de base** - Fonctions fondamentales de RxJS
> 2. **Combinaison** - Bases de la gestion de flux multiples
> 3. **Communication HTTP** - Intégration pratique de l'API
> 4. Autres catégories - Apprendre selon les besoins

## Correspondance avec les opérateurs pipables

De nombreuses fonctions de création ont un opérateur pipable correspondant. Lorsqu'elles sont utilisées dans un pipeline, utilisez un opérateur de la famille `~With`.

| Fonction de création | Opérateur pipable | Notes |
|-------------------|-------------------|------|
| `concat(a$, b$)` | `a$.pipe(`**[concatWith](/fr/guide/operators/combination/concatWith)**`(b$))` | RxJS 7+ |
| `merge(a$, b$)` | `a$.pipe(`**[mergeWith](/fr/guide/operators/combination/mergeWith)**`(b$))` | RxJS 7+ |
| `zip(a$, b$)` | `a$.pipe(`**[zipWith](/fr/guide/operators/combination/zipWith)**`(b$))` | RxJS 7+ |
| `combineLatest([a$, b$])` | `a$.pipe(`**[combineLatestWith](/fr/guide/operators/combination/combineLatestWith)**`(b$))` | RxJS 7+ |
| `race(a$, b$)` | `a$.pipe(`**[raceWith](/fr/guide/operators/combination/raceWith)**`(b$))` | RxJS 7+ |

> [!NOTE]
> Depuis RxJS 7, **[concatWith](/fr/guide/operators/combination/concatWith)**, **[mergeWith](/fr/guide/operators/combination/mergeWith)**, **[zipWith](/fr/guide/operators/combination/zipWith)**, **[combineLatestWith](/fr/guide/operators/combination/combineLatestWith)**, **[raceWith](/fr/guide/operators/combination/raceWith)** et d'autres opérateurs de type `~With` ont été ajoutés, facilitant leur utilisation comme opérateurs pipables.

## Lequel utiliser ?

Le choix entre fonction de création et opérateur pipable dépend du contexte.

### La fonction de création est recommandée

Si plusieurs Observables doivent être opérés au même niveau, la fonction de création simplifiera le code.

```typescript
// ✅ Combiner plusieurs Observables au même niveau
const combined$ = merge(
  fromEvent(button1, 'click'),
  fromEvent(button2, 'click'),
  fromEvent(button3, 'click')
);
```

### L'opérateur pipable est recommandé

Lors de l'ajout d'opérations dans un pipeline, utilisez l'opérateur pipable pour clarifier le flux de traitement.

```typescript
// ✅ Combiner dans le cadre d'un pipeline
const result$ = source$.pipe(
  map(x => x * 2),
  mergeWith(other$),
  filter(x => x > 10)
);
```

## Résumé

- **Fonctions de création** : Fonctions pour créer et combiner des Observables
- **Opérateurs pipables** : Fonctions pour convertir des Observables existants
- Les fonctions de création se répartissent en 7 catégories :
  1. **Création de base** : Créer des Observables basés sur données, tableaux, événements et temps
  2. **Génération de boucles** : Exprimer un traitement itératif dans Observable
  3. **Communication HTTP** : Gérer la communication HTTP comme Observable
  4. **Combinaison** : Combiner plusieurs en un seul
  5. **Sélection/Partition** : Sélectionner ou partitionner
  6. **Conditionnel** : Générer dynamiquement selon des conditions
  7. **Contrôle** : Contrôler le timing d'exécution et la gestion des ressources
- Utiliser les opérateurs pipables de la famille `~With` dans les pipelines
- Chaque catégorie contient plusieurs fonctions et peut être utilisée de différentes manières selon l'application

## Prochaines étapes

Pour en savoir plus sur chaque catégorie, veuillez suivre les liens ci-dessous :

1. **[Fonctions de création de base](/fr/guide/creation-functions/basic/)** - of, from, fromEvent, interval, timer
2. **[Fonctions de génération de boucles](/fr/guide/creation-functions/loop/)** - range, generate
3. **[Fonctions de communication HTTP](/fr/guide/creation-functions/http-communication/)** - ajax, fromFetch
4. **[Fonctions de combinaison](/fr/guide/creation-functions/combination/)** - concat, merge, combineLatest, zip, forkJoin
5. **[Fonctions de sélection/partition](/fr/guide/creation-functions/selection/)** - race, partition
6. **[Fonctions conditionnelles](/fr/guide/creation-functions/conditional/)** - iif, defer
7. **[Fonctions de contrôle](/fr/guide/creation-functions/control/)** - scheduled, using

Sur chaque page, vous apprendrez plus sur le fonctionnement des fonctions de création et découvrirez des exemples pratiques.

## Ressources de référence

- [Documentation officielle RxJS - Fonctions de création](https://rxjs.dev/guide/operators#creation-operators-list)
- [Learn RxJS - Opérateurs de création](https://www.learnrxjs.io/learn-rxjs/operators/creation)
