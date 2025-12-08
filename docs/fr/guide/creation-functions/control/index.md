---
description: "Cette section décrit scheduled et using, qui sont des fonctions de création de contrôle RxJS. scheduled contrôle le timing d'exécution de l'Observable en spécifiant un planificateur, et using gère automatiquement les ressources telles que WebSocket et les poignées de fichiers en fonction du cycle de vie de l'Observable. Il peut également être utilisé pour les tests et l'optimisation des performances."
---

# Fonctions de création de contrôle

RxJS fournit des fonctions de création pour contrôler en détail le timing d'exécution de l'Observable et la gestion des ressources. Cette section décrit en détail deux fonctions, `scheduled()` et `using()`.

## Que sont les fonctions de création de contrôle ?

Les fonctions de création de contrôle sont un ensemble de fonctions permettant un contrôle plus fin du comportement d'Observable. Elles supportent des cas d'utilisation avancés tels que le contrôle du temps d'exécution (scheduler) et la gestion du cycle de vie des ressources.

### Caractéristiques principales

- **Contrôle du temps d'exécution** : Utiliser le scheduler pour passer d'une exécution synchrone à une exécution asynchrone
- **Gestion des ressources** : Libération automatique des ressources en fonction du cycle de vie Observable
- **Facilité de test** : Passer d'un scheduler à l'autre pour faciliter les tests
- **Optimisation des performances** : Contrôle du temps d'exécution pour éviter le blocage de l'interface utilisateur

## Liste des fonctions de création de contrôle

| Fonction | Description | Principales utilisations |
|----------|-------------|--------------------------|
| [scheduled()](/fr/guide/creation-functions/control/scheduled) | Génère un Observable avec le scheduler spécifié | Contrôle du timing d'exécution, test |
| [using()](/fr/guide/creation-functions/control/using) | Observable avec contrôle des ressources | Gestion des ressources pour WebSocket, file handles, etc. |

## scheduled() Bases

`scheduled()` est une fonction qui vous permet de spécifier explicitement un scheduler lors de la génération d'un Observable à partir d'une source de données existante (array, Promise, Iterable, etc.).

### Utilisation de base

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Émission asynchrone d'un tableau
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Début de l\'abonnement');
observable$.subscribe({
  next: val => console.log('Valeur:', val),
  complete: () => console.log('Terminé')
});
console.log('Fin de l\'abonnement');

// Sortie:
// Début de l'abonnement
// Fin de l'abonnement
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Terminé
```

> [!NOTE]
> Avec `asyncScheduler`, l'émission de valeurs devient asynchrone. Cela permet au processus d'abonnement de s'exécuter sans bloquer le thread principal.

## using() Principes de base

`using()` est une fonction qui crée et libère automatiquement des ressources selon le cycle de vie d'Observable. Elle crée une ressource au début d'un abonnement et la libère automatiquement à la fin de l'abonnement (`complete` ou `unsubscribe`).

### Utilisation de base

```typescript
import { using, interval, Subscription, take } from 'rxjs';

const resource$ = using(
  // Fabrique de ressources : exécutée au début de l'abonnement
  () => {
    console.log('Ressource créée');
    return new Subscription(() => console.log('Ressource libérée'));
  },
  // Observable factory : création d'un Observable à partir d'une ressource
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Valeur:', value),
  complete: () => console.log('Terminé')
});

// Sortie:
// Ressource créée
// Valeur: 0
// Valeur: 1
// Valeur: 2
// Terminé
// Ressource libérée
```

> [!IMPORTANT]
> `using()` libère automatiquement les ressources à la fin d'un abonnement, évitant ainsi les fuites de mémoire.

## Comparaison : scheduled() vs using()

| Fonctionnalité | scheduled() | using() |
|----------------|-------------|---------|
| Objectif principal | Contrôle du temps d'exécution | Gestion du cycle de vie des ressources |
| Scheduler | ✅ Peut spécifier explicitement | ❌ Ne peut pas spécifier |
| Gestion des ressources | ❌ Gestion manuelle requise | ✅ Gestion automatique |
| Cas d'utilisation | Tests, optimisation de l'interface utilisateur | WebSocket, gestion de fichiers |
| Complexité | Simple | Assez complexe |

## Directives d'utilisation

### Quand choisir scheduled()

1. **Vous voulez contrôler le temps d'exécution**
   - Vous voulez transformer un traitement synchrone en un traitement asynchrone
   - Vous voulez éviter le blocage de l'interface utilisateur

2. **Nécessité de contrôler le temps pour les tests**
   - Combiner avec TestScheduler pour contrôler le temps
   - Vous voulez tester un traitement asynchrone de manière synchrone

3. **Convertir les sources de données existantes en Observable**
   - Convertir un tableau, une Promise, un Iterable en Observable
   - Vous souhaitez spécifier explicitement un scheduler

### Quand choisir using()

1. **La libération automatique des ressources est nécessaire**
   - Gestion des connexions WebSocket
   - Gestion des handles de fichiers
   - Nettoyage automatique des minuteries

2. **Vouloir éviter les fuites de mémoire**
   - Éviter d'oublier de libérer des ressources
   - Nettoyage fiable à la fin de l'abonnement

3. **Gestion de ressources complexes**
   - Gérer plusieurs ressources à la fois
   - Gérer les dépendances des ressources

## Exemples d'utilisation pratique

### Exemple d'utilisation de scheduled()

```typescript
import { scheduled, asyncScheduler, queueScheduler } from 'rxjs';

// Traite de grandes quantités de données de manière asynchrone (ne bloque pas l'interface utilisateur)
const largeArray = Array.from({ length: 10000 }, (_, i) => i);
const async$ = scheduled(largeArray, asyncScheduler);

async$.subscribe(value => {
  // Exécuter un traitement lourd ici
  // L'interface utilisateur n'est pas bloquée
});

// Exécution synchrone dans les tests
const sync$ = scheduled(largeArray, queueScheduler);
```

### Exemple d'utilisation de using()

```typescript
import { using, timer } from 'rxjs';

// Gestion automatique de la connexion WebSocket
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    console.log('Connexion WebSocket démarrée');
    return {
      unsubscribe: () => {
        ws.close();
        console.log('Connexion WebSocket terminée');
      }
    };
  },
  () => timer(0, 1000) // Reçoit des messages toutes les 1 secondes
);
```

## Types de schedulers (pour scheduled())

| Scheduler | Description | Cas d'utilisation |
|-----------|-------------|-------------------|
| `queueScheduler` | Exécution synchrone (méthode de file d'attente) | Par défaut, traitement synchrone |
| `asyncScheduler` | Exécution asynchrone (setTimeout) | Optimisation de l'interface utilisateur, traitement à long terme |
| `asapScheduler` | Exécution asynchrone la plus rapide (Promise) | Traitement asynchrone de haute priorité |
| `animationFrameScheduler` | Frame d'animation | Animation, rendu de l'interface utilisateur |

> [!TIP]
> Pour plus d'informations sur les schedulers, voir [Types de schedulers](/fr/guide/schedulers/types).

## Questions fréquemment posées

### Q1 : Quelle est la différence entre scheduled() et from() ?

**R:** `from()` utilise le scheduler par défaut (synchrone) en interne. `scheduled()` permet de spécifier explicitement le scheduler, ce qui permet un contrôle précis du temps d'exécution.

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - exécution synchrone
const sync$ = from([1, 2, 3]);

// scheduled() - exécution asynchrone
const async$ = scheduled([1, 2, 3], asyncScheduler);
```

### Q2 : Quand dois-je utiliser using() ?

**R:** Utilisez-le lorsque vous voulez éviter d'oublier de libérer des ressources. C'est particulièrement utile dans les cas suivants :
- Connexions réseau telles que WebSocket, EventSource, etc.
- Les handles de fichiers, les connexions aux bases de données
- Les processus qui nécessitent un `clearInterval()` ou un `clearTimeout()` manuel

### Q3 : Pourquoi scheduled() est-il plus facile à tester ?

**R:** TestScheduler vous permet de contrôler virtuellement le passage du temps. Les processus asynchrones peuvent être testés de manière synchrone, ce qui réduit considérablement le temps d'exécution des tests.

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled, asyncScheduler } from 'rxjs';

const testScheduler = new TestScheduler((actual, expected) => {
  expect(actual).toEqual(expected);
});

testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

## Meilleures pratiques

### 1. Éviter le blocage de l'interface utilisateur avec scheduled()

```typescript
// ❌ Mauvais exemple : Traiter de grandes quantités de données de manière synchrone
from(largeArray).subscribe(processHeavyTask);

// ✅ Bon exemple : Traitement asynchrone avec asyncScheduler
scheduled(largeArray, asyncScheduler).subscribe(processHeavyTask);
```

### 2. Assurer la libération des ressources avec using()

```typescript
// ❌ Mauvais exemple : Gestion manuelle des ressources
const ws = new WebSocket('wss://example.com');
const source$ = interval(1000);
source$.subscribe(() => ws.send('ping'));
// Fuite de ressources si désabonnement oublié

// ✅ Bon exemple : Gestion automatique avec using()
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return { unsubscribe: () => ws.close() };
  },
  () => interval(1000).pipe(tap(() => ws.send('ping')))
);
```

### 3. Utiliser un scheduler approprié pour les tests

```typescript
// ✅ Bon exemple : TestScheduler pour les tests
const testScheduler = new TestScheduler(...);
const source$ = scheduled([1, 2, 3], testScheduler);

// ✅ Bon exemple : asyncScheduler pour la production
const source$ = scheduled([1, 2, 3], asyncScheduler);
```

## Résumé

Les fonctions de création de contrôle sont des fonctions avancées permettant d'affiner le comportement des Observables.

**scheduled():**
- Contrôle explicitement le timing d'exécution (synchrone/asynchrone)
- Utile pour le contrôle du temps dans les tests
- Efficace pour éviter le blocage de l'interface utilisateur

**using():**
- Gestion automatique du cycle de vie des ressources
- Prévient les fuites de mémoire
- Idéal pour gérer les connexions telles que WebSocket

Utilisé de manière appropriée, vous pouvez construire des applications RxJS plus robustes et plus performantes.

## Prochaines étapes

Pour une utilisation détaillée de chaque fonction, veuillez vous référer aux pages suivantes :

- [scheduled() en détail](/fr/guide/creation-functions/control/scheduled) - Générer un Observable avec un scheduler
- [using() en détail](/fr/guide/creation-functions/control/using) - Observable avec contrôle des ressources

## Ressources de référence

- [Documentation officielle RxJS - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [Documentation officielle RxJS - using](https://rxjs.dev/api/index/function/using)
- [Documentation officielle RxJS - Scheduler](https://rxjs.dev/guide/scheduler)
- [Types de schedulers](/fr/guide/schedulers/types)
