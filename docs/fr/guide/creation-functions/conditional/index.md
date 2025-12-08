---
description: "Cette section décrit les fonctions de création qui sélectionnent et créent des Observables en fonction de conditions. Apprenez à utiliser iif et defer, ainsi que des exemples pratiques."
---

# Fonctions de création conditionnelles

Les fonctions de création sélectionnent un Observable en fonction d'une condition ou génèrent dynamiquement un Observable lors de la souscription.

## Qu'est-ce qu'une fonction de création conditionnelle ?

Les fonctions de création conditionnelle ont les rôles suivants :

- **Sélection conditionnelle** : Sélectionner différents Observables selon les conditions
- **Génération différée** : Créer dynamiquement un Observable lors de l'abonnement

Contrairement aux autres fonctions de création, qui créent et combinent des Observables de manière statique, celles-ci peuvent modifier leur comportement en fonction des **conditions et états d'exécution**.

> [!NOTE]
> Bien que `iif` et `defer` aient été précédemment classés comme "opérateurs conditionnels", ce sont des **fonctions de création** (fonctions de création d'Observables), et non des opérateurs Pipeable.

## Principales fonctions de création conditionnelles

| Fonction | Description | Cas d'utilisation |
|----------|-------------|-------------------|
| **[iif](/fr/guide/creation-functions/conditional/iif)** | Sélectionner un des deux Observables selon une condition | Branchement de traitement selon le statut de connexion |
| **[defer](/fr/guide/creation-functions/conditional/defer)** | Retarder la génération de l'Observable au moment de l'abonnement | Création dynamique d'Observable |

## Critères d'utilisation

### iif - Deux branches selon la condition

`iif` sélectionne l'un des deux Observables en fonction du résultat d'une fonction conditionnelle. La condition est évaluée **au moment de l'abonnement**.

**Syntaxe** :
```typescript
iif(
  () => condition,  // Fonction de condition (évaluée au moment de l'abonnement)
  trueObservable,   // Observable si vrai
  falseObservable   // Observable si faux
)
```

**Cas d'utilisation** :
- Branchement de traitement selon le statut de connexion
- Changement de traitement selon l'existence du cache
- Changement de comportement par variables d'environnement

```typescript
import { iif, of } from 'rxjs';

const isAuthenticated = () => Math.random() > 0.5;

const data$ = iif(
  isAuthenticated,
  of('Données authentifiées'),
  of('Données publiques')
);

data$.subscribe(console.log);
// Sortie: 'Données authentifiées' ou 'Données publiques' (selon la condition au moment de l'abonnement)
```

### defer - Génération différée au moment de l'abonnement

`defer` génère un Observable à chaque fois qu'un abonnement se produit. Cela permet à l'Observable de changer son comportement en fonction de son état au moment de l'abonnement.

**Syntaxe** :
```typescript
defer(() => {
  // Exécuté au moment de l'abonnement
  return someObservable;
})
```

**Cas d'utilisation** :
- Générer un Observable reflétant le dernier état au moment de l'abonnement
- Générer une valeur aléatoire différente à chaque fois
- Effectuer un traitement différent pour chaque abonnement

```typescript
import { defer, of } from 'rxjs';

// Obtenir l'heure actuelle à l'abonnement
const timestamp$ = defer(() => of(new Date().toISOString()));

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Premier:', time));
}, 1000);

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Second:', time));
}, 2000);

// Sortie:
// Premier: 2024-10-21T01:00:00.000Z
// Second: 2024-10-21T01:00:01.000Z
// ※Des heures différentes sont émises car les moments d'abonnement diffèrent
```

## Différence entre iif et defer

| Fonctionnalité | iif | defer |
|----------------|-----|-------|
| **Choix** | Sélectionner parmi deux Observables | Générer n'importe quel Observable |
| **Moment d'évaluation** | Évaluer la condition au moment de l'abonnement | Exécuter la fonction au moment de l'abonnement |
| **Objectif** | Branchement conditionnel | Génération dynamique |

## Utilisation dans le pipeline

Les fonctions de création conditionnelle peuvent être utilisées en combinaison avec d'autres opérateurs.

```typescript
import { defer, of } from 'rxjs';
import { switchMap } from 'rxjs';

// Obtenir les informations utilisateur à partir de l'ID utilisateur
const userId$ = of(123);

userId$.pipe(
  switchMap(id =>
    defer(() => {
      // Vérifier le dernier cache au moment de l'abonnement
      const cached = cache.get(id);
      return cached ? of(cached) : fetchUser(id);
    })
  )
).subscribe(console.log);
```

## Conversion de Cold en Hot

Comme le montre le tableau ci-dessus, **toutes les fonctions de création conditionnelle génèrent des Observables Cold**. Les évaluations conditionnelles et les fonctions de génération sont exécutées à chaque abonnement.

Vous pouvez convertir un Observable Cold en Observable Hot en utilisant des opérateurs de multidiffusion (`share()`, `shareReplay()`, etc.).

### Exemple pratique : Partage des résultats de branchement conditionnel

```typescript
import { iif, of, interval } from 'rxjs';
import { take, share } from 'rxjs';

const condition = () => Math.random() > 0.5;

// ❄️ Cold - Réévaluer la condition pour chaque abonnement
const coldIif$ = iif(
  condition,
  of('La condition est vraie'),
  interval(1000).pipe(take(3))
);

coldIif$.subscribe(val => console.log('Abonné 1:', val));
coldIif$.subscribe(val => console.log('Abonné 2:', val));
// → Chaque abonné évalue indépendamment la condition (possibilité de résultats différents)

// 🔥 Hot - Partager les résultats d'évaluation de condition entre les abonnés
const hotIif$ = iif(
  condition,
  of('La condition est vraie'),
  interval(1000).pipe(take(3))
).pipe(share());

hotIif$.subscribe(val => console.log('Abonné 1:', val));
hotIif$.subscribe(val => console.log('Abonné 2:', val));
// → La condition n'est évaluée qu'une seule fois, les résultats sont partagés
```

> [!TIP]
> Pour plus d'informations, voir [Création de base - Conversion de Cold en Hot](/fr/guide/creation-functions/basic/#conversion-de-cold-en-hot).

## Prochaines étapes

Pour connaître le comportement détaillé et les exemples pratiques de chaque fonction de création, cliquez sur les liens du tableau ci-dessus.

De plus, en apprenant les [Fonctions de création de combinaison](/fr/guide/creation-functions/combination/) et les [Fonctions de création de sélection/partition](/fr/guide/creation-functions/selection/), vous comprendrez l'ensemble des fonctions de création.
