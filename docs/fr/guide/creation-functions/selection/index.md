---
description: "Les Creation Function (race et partition) qui sélectionnent un observable parmi plusieurs ou divisent un observable en plusieurs sont expliquées. Des cas d'utilisation pratiques et des implémentations à sécurité de type dans TypeScript, tels que la gestion des conflits, l'acquisition de la réponse la plus rapide et le partitionnement de flux par branchement conditionnel sont présentés."
---

# Systèmes de sélection et de partitionnement Creation Function

Fonctions de Creation Function qui sélectionnent un Observable à partir de plusieurs Observables ou divisent un Observable en plusieurs Observables selon des conditions.

## Système de sélection et de partitionnement Que sont les Creation Function ?

Les fonctions de Creation Function pour les systèmes de sélection et de division sont différentes de celles pour les systèmes de combinaison et ont les rôles suivants.

- Sélection** : sélection d'un Observable satisfaisant à certaines conditions parmi plusieurs Observables.
- Fractionnement** : fractionnement d'un Observable en plusieurs Observables conformément à une condition.

Ces fonctions fonctionnent dans la direction opposée ou d'un point de vue différent de celui de la jointure "combiner plusieurs observables en un seul".

## Principaux systèmes de sélection et de division Creation Function

| Fonction | Description de la fonction | Cas d'utilisation. |
|---|---|---|
| **[race](/fr/guide/création-fonctions/sélection/race)** | Adopter le premier publié | Compétition de sources de données multiples |
| **[partition](/fr/guide/creation-functions/selection/partition)** | Séparer en deux avec des conditions | Processus de ramification en cas de succès ou d'échec |

## Critères d'utilisation

### race - sélectionner l'Observable le plus rapide

`race` s'abonne à plusieurs Observables simultanément et adopte le **premier Observable qui émet une valeur. Les Observables qui ne sont pas adoptés sont automatiquement unsubscribe.

**Cas d'utilisation** :.
- Adopter la réponse la plus rapide à partir de plusieurs points d'extrémité de l'API
- Gestion des délais d'attente (processus d'origine ou temporisation)
- Concurrence entre le cache et les appels réels à l'API

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Adopter les données les plus rapides à partir de sources multiples
const fast$ = timer(1000).pipe(map(() => 'Fast API'));
const slow$ = timer(3000).pipe(map(() => 'Slow API'));

race(fast$, slow$).subscribe(console.log);
// Sortie.: 'Fast API' (1La sortie est effectuée après une seconde,slow$est annulée)
```

### partition - divisée par condition

La `partition` divise une Observable en **deux Observables** en se basant sur une fonction conditionnelle. La valeur de retour est un tableau `[if true, if false]`.

**Cas d'utilisation** :.
- Séparation du succès et de l'échec.
- Séparation des nombres pairs et impairs.
- Séparation des données valides et invalides

```typescript
import { of, partition } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Diviser en nombres pairs et impairs
const [even$, odd$] = partition(source$, n => n % 2 === 0);

even$.subscribe(val => console.log('Even:', val));
// Sortie.: Even: 2, Even: 4, Even: 6

odd$.subscribe(val => console.log('Odd:', val));
// Sortie.: Odd: 1, Odd: 3, Odd: 5
```

## Conversion du froid au chaud

Comme le montre le tableau ci-dessus, **Toutes les fonctions de sélection et de Creation Function génèrent un Observable à froid**. Chaque abonnement déclenche une exécution indépendante.

Les Observable froids peuvent être convertis en Observable chauds en utilisant des opérateurs basés sur la multidiffusion (`share()`, `shareReplay()`, etc.).

### Exemple pratique : partage de demandes d'API conflictuelles

```typescript
import { race, timer } from 'rxjs';
import { map, share } from 'rxjs';

// ❄️ Cold - Relancer la compétition pour chaque abonnement
const coldRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
);

coldRace$.subscribe(val => console.log('Abonné1:', val));
coldRace$.subscribe(val => console.log('Abonné2:', val));
// → Chaque abonné organise un concours indépendant (un2concours une fois)

// 🔥 Hot - Partager les résultats des concours entre les abonnés
const hotRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
).pipe(share());

hotRace$.subscribe(val => console.log('Abonné1:', val));
hotRace$.subscribe(val => console.log('Abonné2:', val));
// → 1Partager les résultats d'un concours
```

> [!TIP]

> Pour plus d'informations, voir [Système de création de base - Conversion de froid à chaud] (/guide/creation-functions/basic/#cold- to -hot-).

## Correspondance avec Pipeable Operator

Les fonctions de création de sélection et de division ont également un Pipeable Operator correspondant.

| Creation Function | Pipeable Operator |
|---|---|
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(source$, predicate)` | Ne peut pas être utilisé dans un pipeline (Creation Function uniquement). |

> [!NOTE]

> Il n'y a pas de version Pipeable Operator de `partition`. Si une partition est nécessaire, elle peut être utilisée comme Creation Function ou découpée manuellement en utilisant deux fois `filter`.

## Prochaines étapes.

Pour en savoir plus sur le fonctionnement détaillé et les exemples pratiques de chaque Creation Function, cliquez sur les liens du tableau ci-dessus.

Vous pouvez également vous familiariser avec les [Fonctions de création combinées] (/guide/creation-functions/combination/) et les [Fonctions de création conditionnelles] (/guide/creation-functions/conditionnelles/). Ensemble, elles permettent une compréhension globale des Creation Function.
