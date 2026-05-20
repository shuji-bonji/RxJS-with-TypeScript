---
description: "Cette section décrit les fonctions de création qui combinent plusieurs Observables en un seul, et enseigne comment utiliser concat, merge, combineLatest, zip et forkJoin, ainsi que des exemples pratiques."
---

# Fonctions de création de combinaison

Il s'agit des principales fonctions de création permettant de combiner plusieurs Observables en un seul Observable.

## Que sont les fonctions de création de combinaison ?

Les fonctions de création de combinaison prennent plusieurs Observables et les combinent en un seul flux Observable. Le moment et l'ordre dans lesquels les valeurs sont émises dépendent de la méthode de combinaison.

Le tableau ci-dessous présente les caractéristiques de chaque fonction de création et explique comment les utiliser.

## Principales fonctions de création de combinaison

| Fonction | Description | Cas d'utilisation |
|----------|-------------|-------------------|
| **[concat](/fr/guide/creation-functions/combination/concat)** | Combinaison séquentielle (la suivante commence après la fin de la précédente) | Traitement étape par étape |
| **[merge](/fr/guide/creation-functions/combination/merge)** | Combinaison parallèle (souscription simultanée, sortie dans l'ordre d'émission) | Intégration d'événements multiples |
| **[combineLatest](/fr/guide/creation-functions/combination/combineLatest)** | Combine les dernières valeurs | Synchronisation des entrées de formulaires |
| **[zip](/fr/guide/creation-functions/combination/zip)** | Associe les valeurs correspondantes | Appariement des requêtes avec les réponses |
| **[forkJoin](/fr/guide/creation-functions/combination/forkJoin)** | Attend que tout soit terminé et combine les valeurs finales | Attente de la fin des appels API parallèles |

## Critères d'utilisation

La sélection des fonctions de création de combinaison est déterminée selon les perspectives suivantes :

### 1. Moment de l'exécution

- **Exécution séquentielle** : `concat` - Commence le suivant après que l'Observable précédent soit terminé
- **Exécution parallèle** : `merge`, `combineLatest`, `zip`, `forkJoin` - S'abonne à tous les Observables simultanément

### 2. Comment émettre des valeurs

- **Émettre toutes les valeurs** : `concat`, `merge` - Produire toutes les valeurs émises par chaque Observable
- **Combiner les dernières valeurs** : `combineLatest` - Combine toutes les dernières valeurs chaque fois que l'une d'entre elles émet
- **Associer les valeurs correspondantes** : `zip` - Associe les valeurs des positions correspondantes dans chaque Observable et les émet
- **Valeurs finales uniquement** : `forkJoin` - Émet chaque valeur finale sous forme de tableau lorsque tous les Observables sont complets

### 3. Moment de l'achèvement

- **Après que tout soit terminé** : `concat`, `forkJoin` - Attendre que tous les Observables soient terminés
- **Se termine avec le flux le plus court** : `zip` - Termine quand l'un d'entre eux est terminé, car les valeurs restantes ne peuvent pas former de paires
- **Ne se termine pas** : `merge`, `combineLatest` - Si l'un se termine alors que l'autre continue, il ne se terminera pas

## Conversion de Cold en Hot

Comme le montre le tableau ci-dessus, **toutes les fonctions de création de combinaison génèrent des Observables Cold**. Chaque abonnement initie une exécution indépendante.

Cependant, vous pouvez **convertir un Observable Cold en Observable Hot** en utilisant un opérateur de multidiffusion (`share()`, `shareReplay()`, `publish()`, etc.).

### Exemple pratique : Partage de requêtes HTTP

```typescript
import { merge, interval } from 'rxjs';
import { map, take, share } from 'rxjs';

// ❄️ Cold - Requêtes HTTP indépendantes pour chaque abonnement
const coldApi$ = merge(
  interval(1000).pipe(map(() => 'Source A'), take(3)),
  interval(1500).pipe(map(() => 'Source B'), take(2))
);

coldApi$.subscribe(val => console.log('Abonné 1:', val));
coldApi$.subscribe(val => console.log('Abonné 2:', val));
// → Chaque abonné exécute des intervalles indépendants (2x requêtes)

// 🔥 Hot - Partager l'exécution entre les abonnés
const hotApi$ = merge(
  interval(1000).pipe(map(() => 'Source A'), take(3)),
  interval(1500).pipe(map(() => 'Source B'), take(2))
).pipe(share());

hotApi$.subscribe(val => console.log('Abonné 1:', val));
hotApi$.subscribe(val => console.log('Abonné 2:', val));
// → Partager un seul intervalle (requêtes une seule fois)
```

> [!TIP]
> **Cas où la conversion Hot est nécessaire** :
> - Plusieurs composants partagent les mêmes résultats d'API
> - Utiliser `forkJoin` pour utiliser les résultats de requêtes parallèles à plusieurs endroits
> - Gérer l'état avec `combineLatest` et distribuer à plusieurs abonnés
>
> Pour plus d'informations, voir [Création de base - Conversion de Cold en Hot](/fr/guide/creation-functions/basic/#conversion-de-cold-en-hot).

## Correspondance avec l'opérateur Pipeable

Pour les fonctions de création de combinaison, il existe un opérateur Pipeable correspondant. Lorsqu'il est utilisé dans un pipeline, l'opérateur de type `~With` est utilisé.

| Fonction de création | Opérateur Pipeable |
|----------------------|--------------------|
| `concat(a$, b$)` | `a$.pipe(concatWith(b$))` |
| `merge(a$, b$)` | `a$.pipe(mergeWith(b$))` |
| `zip(a$, b$)` | `a$.pipe(zipWith(b$))` |
| `combineLatest([a$, b$])` | `a$.pipe(combineLatestWith(b$))` |

## Prochaines étapes

Pour connaître le comportement détaillé et les exemples pratiques de chaque fonction de création, cliquez sur les liens du tableau ci-dessus.

Apprenez également les [Fonctions de création de selection/partition](/fr/guide/creation-functions/selection/) et les [Fonctions de création conditionnelles](/fr/guide/creation-functions/conditional/), vous comprendrez ainsi l'ensemble des fonctions de création.
