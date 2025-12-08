---
description: "Cette section présente une vue d'ensemble des fonctions de création de sélection et de partition qui permettent de sélectionner un Observable parmi plusieurs Observables ou de diviser un Observable en plusieurs Observables. Elle explique comment utiliser race et partition, ainsi que des exemples pratiques."
---

# Fonctions de création de sélection/partition

Il s'agit de fonctions de création permettant de sélectionner un Observable parmi plusieurs Observables ou de diviser un Observable en plusieurs Observables.

## Que sont les fonctions de création de sélection/partition ?

Les fonctions de création de sélection/partition sont un ensemble de fonctions qui mettent en concurrence plusieurs Observables afin de sélectionner le plus rapide ou de diviser un Observable en deux flux en fonction de conditions. Cette fonction est utile pour mettre en concurrence des sources de données ou pour allouer des traitements en fonction de conditions.

Consultez le tableau ci-dessous pour connaître les caractéristiques et l'utilisation de chaque fonction de création.

## Principales fonctions de création de sélection/partition

| Fonction | Description | Cas d'utilisation |
|----------|-------------|-------------------|
| **[race](/fr/guide/creation-functions/selection/race)** | Sélectionner l'Observable le plus rapide (celui qui émet en premier) | Concurrence entre plusieurs sources de données, traitement de repli |
| **[partition](/fr/guide/creation-functions/selection/partition)** | Diviser en deux Observables selon une condition | Gestion des succès/échecs, branchement selon les conditions |

## Critères d'utilisation

La sélection des fonctions de création de sélection/partition est déterminée selon les perspectives suivantes.

### 1. Objectif

- **Sélectionner le plus rapide parmi plusieurs sources** : `race` - Sélectionner le premier qui émet une valeur parmi plusieurs sources de données
- **Diviser selon une condition** : `partition` - Diviser un Observable en deux flux selon une condition

### 2. Moment d'émission

- **Seulement le plus rapide** : `race` - Une fois sélectionné, les autres valeurs d'Observable sont ignorées
- **Classer toutes les valeurs** : `partition` - Toutes les valeurs sont triées en deux flux selon les conditions

### 3. Moment de l'achèvement

- **Dépend de l'Observable sélectionné** : `race` - Suit l'achèvement de l'Observable qui a émis le premier
- **Dépend de l'Observable original** : `partition` - Les deux flux s'achèvent lorsque l'Observable original s'achève

## Exemples d'utilisation pratique

### race() - Sélectionner le plus rapide parmi plusieurs sources de données

Si vous avez plusieurs sources de données et que vous souhaitez utiliser celle qui répond le plus rapidement, utilisez `race()`.

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Simuler plusieurs APIs
const api1$ = timer(1000).pipe(map(() => 'Réponse API1'));
const api2$ = timer(500).pipe(map(() => 'Réponse API2'));
const api3$ = timer(1500).pipe(map(() => 'Réponse API3'));

// Utiliser la réponse la plus rapide
race(api1$, api2$, api3$).subscribe(console.log);
// Sortie: Réponse API2 (la plus rapide à 500ms)
```

### partition() - Diviser en deux selon une condition

Si vous souhaitez diviser un Observable en deux flux selon une condition, utilisez `partition()`.

```typescript
import { of } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// Diviser en nombres pairs et impairs
const [evens$, odds$] = partition(numbers$, n => n % 2 === 0);

evens$.subscribe(n => console.log('Pair:', n));
// Sortie: Pair: 2, Pair: 4, Pair: 6, Pair: 8, Pair: 10

odds$.subscribe(n => console.log('Impair:', n));
// Sortie: Impair: 1, Impair: 3, Impair: 5, Impair: 7, Impair: 9
```

## Conversion de Cold en Hot

Comme le montre le tableau ci-dessus, **toutes les fonctions de création de sélection/partition génèrent des Observables Cold**. Une exécution indépendante est lancée pour chaque abonnement.

Cependant, en utilisant les opérateurs de multidiffusion (`share()`, `shareReplay()`, etc.), vous pouvez **convertir un Observable Cold en Observable Hot**.

### Exemple pratique : Partage de l'exécution

```typescript
import { race, timer, share } from 'rxjs';
import { map } from 'rxjs';

// ❄️ Cold - Exécution indépendante pour chaque abonnement
const coldRace$ = race(
  timer(1000).pipe(map(() => 'API1')),
  timer(500).pipe(map(() => 'API2'))
);

coldRace$.subscribe(val => console.log('Abonné 1:', val));
coldRace$.subscribe(val => console.log('Abonné 2:', val));
// → Chaque abonné exécute une course indépendante (2x requêtes)

// 🔥 Hot - Partager l'exécution entre les abonnés
const hotRace$ = race(
  timer(1000).pipe(map(() => 'API1')),
  timer(500).pipe(map(() => 'API2'))
).pipe(share());

hotRace$.subscribe(val => console.log('Abonné 1:', val));
hotRace$.subscribe(val => console.log('Abonné 2:', val));
// → Partager l'exécution de la course (requêtes une seule fois)
```

> [!TIP]
> **Cas où la conversion Hot est nécessaire** :
> - Partager le résultat de `race()` entre plusieurs composants
> - Utiliser le résultat de `partition()` à plusieurs endroits
> - Exécuter une seule fois un traitement coûteux
>
> Pour plus d'informations, voir [Création de base - Conversion de Cold en Hot](/fr/guide/creation-functions/basic/#conversion-de-cold-en-hot).

## Correspondance avec l'opérateur Pipeable

Pour les fonctions de création de sélection/partition, il existe un opérateur Pipeable correspondant. Lorsqu'il est utilisé dans un pipeline, l'opérateur de type `~With` est utilisé.

| Fonction de création | Opérateur Pipeable |
|----------------------|--------------------|
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(source$, predicate)` | Pas de correspondance directe (utiliser comme fonction de création) |

> [!NOTE]
> `partition()` est typiquement utilisée comme fonction de création. Pour diviser un flux dans un pipeline, utilisez des opérateurs tels que `filter()` en combinaison.

## Prochaines étapes

Pour connaître le comportement détaillé et les exemples pratiques de chaque fonction de création, cliquez sur les liens du tableau ci-dessus.

En apprenant les [Fonctions de création de base](/fr/guide/creation-functions/basic/), les [Fonctions de création de combinaison](/fr/guide/creation-functions/combination/) et les [Fonctions de création conditionnelles](/fr/guide/creation-functions/conditional/), vous comprendrez l'ensemble des fonctions de création.
