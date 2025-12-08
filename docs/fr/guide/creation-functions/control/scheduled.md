---
description: "Cette présentation détaille comment utiliser la fonction scheduled() de RxJS pour spécifier un scheduler, générer un Observable, et contrôler le timing de l'exécution, avec des exemples de code pratiques."
---

# scheduled()

[📘 Documentation officielle RxJS - scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` est une fonction de création qui vous permet de spécifier explicitement un scheduler lors de la génération d'Observables à partir de sources de données telles que des tableaux, des Promises et des Iterables. Cela permet un contrôle fin du timing d'exécution (synchrone ou asynchrone) et est utile pour les tests et l'optimisation des performances de l'interface utilisateur.

## Utilisation de base

### Conversion d'un simple tableau en Observable

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Émission d'un tableau de manière asynchrone
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Abonnement démarré');
observable$.subscribe({
  next: val => console.log('Valeur:', val),
  complete: () => console.log('Terminé')
});
console.log('Abonnement terminé');

// Sortie:
// Abonnement démarré
// Abonnement terminé
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Terminé
```

> [!IMPORTANT]
> **Différence entre synchrone et asynchrone**
>
> L'utilisation de `asyncScheduler` rend l'émission de valeurs asynchrone. Par conséquent, l'ordre de sortie est le suivant : "Abonnement démarré" → "Abonnement terminé" → "Valeur: 1".

### Comparaison avec from()

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - la valeur par défaut est synchrone
console.log('=== from() ===');
from([1, 2, 3]).subscribe(val => console.log('Valeur:', val));
console.log('Abonnement terminé');

// Sortie:
// === from() ===
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Abonnement terminé

// scheduled() - explicitement asynchrone
console.log('=== scheduled() ===');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Valeur:', val));
console.log('Abonnement terminé');

// Sortie:
// === scheduled() ===
// Abonnement terminé
// Valeur: 1
// Valeur: 2
// Valeur: 3
```

## Types de schedulers

RxJS fournit plusieurs schedulers qui peuvent être utilisés à des fins différentes.

| Scheduler | Timing d'exécution | Technologie de base | Utilisation principale |
|-----------|-------------------|---------------------|------------------------|
| `queueScheduler` | Synchrone (file d'attente) | Exécution immédiate | Par défaut, traitement synchrone |
| `asyncScheduler` | Asynchrone | `setTimeout` | Optimisation de l'interface utilisateur, traitement long |
| `asapScheduler` | Asynchrone le plus rapide | `Promise` (microtask) | Traitement asynchrone de haute priorité |
| `animationFrameScheduler` | Frame d'animation | `requestAnimationFrame` | Animation, rendu de l'interface utilisateur |

### queueScheduler (exécution synchrone)

```typescript
import { scheduled, queueScheduler } from 'rxjs';

console.log('Début');
scheduled([1, 2, 3], queueScheduler).subscribe(val => console.log('Valeur:', val));
console.log('Fin');

// Sortie:
// Début
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Fin
```

### asyncScheduler (exécution asynchrone)

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

console.log('Début');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Valeur:', val));
console.log('Fin');

// Sortie:
// Début
// Fin
// Valeur: 1
// Valeur: 2
// Valeur: 3
```

### asapScheduler (microtask)

```typescript
import { scheduled, asapScheduler } from 'rxjs';

console.log('Début');
scheduled([1, 2, 3], asapScheduler).subscribe(val => console.log('Valeur:', val));
console.log('Fin');

// Sortie:
// Début
// Fin
// Valeur: 1
// Valeur: 2
// Valeur: 3
```

> [!TIP]
> **asyncScheduler vs asapScheduler**
>
> - `asyncScheduler` : basé sur `setTimeout` (macrotask)
> - `asapScheduler` : basé sur `Promise` (microtask)
>
> `asapScheduler` s'exécute plus rapidement, mais les deux sont asynchrones.

### animationFrameScheduler (animation)

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';

// Mise à jour des valeurs à chaque frame d'animation
const positions = [0, 50, 100, 150, 200];
const animation$ = scheduled(positions, animationFrameScheduler).pipe(
  map(pos => `Position: ${pos}px`)
);

animation$.subscribe(position => {
  console.log(position);
  // Mise à jour du DOM ici
});

// Sortie: (à chaque frame d'animation)
// Position: 0px
// Position: 50px
// Position: 100px
// Position: 150px
// Position: 200px
```

## Modèles pratiques

### Traitement de données en masse sans bloquer l'interface utilisateur

```typescript
import { scheduled, asyncScheduler, map, bufferCount } from 'rxjs';

// Traite 1 million de données
const largeArray = Array.from({ length: 1000000 }, (_, i) => i);

// ❌ Mauvais exemple : traitement synchrone (l'interface utilisateur sera bloquée)
// from(largeArray).subscribe(processData);

// ✅ Bon exemple : traitement asynchrone (l'interface utilisateur ne sera pas bloquée)
scheduled(largeArray, asyncScheduler).pipe(
  bufferCount(1000), // Traitement par lots de 1000 à la fois
  map(batch => batch.reduce((sum, val) => sum + val, 0))
).subscribe({
  next: sum => console.log('Total du lot:', sum),
  complete: () => console.log('Traitement terminé')
});

console.log('L\'interface utilisateur reste réactive');
```

### Combinaison avec Promise

```typescript
import { scheduled, asyncScheduler, mergeMap } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const userIds = [1, 2, 3, 4, 5];

// Récupère plusieurs utilisateurs de manière asynchrone
scheduled(userIds, asyncScheduler).pipe(
  mergeMap(id =>
    fetch(`https://api.example.com/users/${id}`).then(res => res.json())
  )
).subscribe({
  next: (user: User) => console.log('Utilisateur:', user),
  error: error => console.error('Erreur:', error),
  complete: () => console.log('Tous les utilisateurs récupérés')
});
```

### Génération à partir d'un Iterable

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Conversion d'un Set avec scheduling
const uniqueNumbers = new Set([1, 2, 3, 4, 5]);
const observable$ = scheduled(uniqueNumbers, asyncScheduler);

observable$.subscribe({
  next: val => console.log('Valeur:', val),
  complete: () => console.log('Terminé')
});

// Conversion d'une Map avec scheduling
const userMap = new Map([
  [1, 'Alice'],
  [2, 'Bob'],
  [3, 'Charlie']
]);

scheduled(userMap, asyncScheduler).subscribe({
  next: ([id, name]) => console.log(`ID: ${id}, Nom: ${name}`),
  complete: () => console.log('Terminé')
});
```

## Utilisation dans les tests

`scheduled()` peut être combiné avec TestScheduler pour écrire des tests avec contrôle du temps.

### Tests de base

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled } from 'rxjs';

describe('scheduled()', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('émet les éléments du tableau dans l\'ordre', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler);
      const expected = '(abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

### Test du traitement asynchrone

```typescript
import { scheduled, asyncScheduler, delay } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Test du traitement asynchrone', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('teste virtuellement le traitement retardé', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler).pipe(
        delay(1000, testScheduler)
      );

      // Émission après 1000ms (temps virtuel)
      const expected = '1000ms (abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

> [!TIP]
> **Avantages du TestScheduler**
>
> - Tester sans attendre de temps réel
> - Tester les traitements asynchrones de manière synchrone
> - Réduire considérablement le temps d'exécution des tests

## Exemples d'utilisation courante

### 1. Récupération de données paginées

```typescript
import { scheduled, asyncScheduler, mergeMap, toArray } from 'rxjs';

interface Page {
  page: number;
  data: any[];
}

// Liste des numéros de page
const pages = [1, 2, 3, 4, 5];

// Récupération asynchrone de chaque page
const allData$ = scheduled(pages, asyncScheduler).pipe(
  mergeMap(page =>
    fetch(`https://api.example.com/items?page=${page}`)
      .then(res => res.json())
  ),
  toArray() // Combine toutes les pages de données
);

allData$.subscribe({
  next: data => console.log('Toutes les données:', data),
  complete: () => console.log('Récupération terminée')
});
```

### 2. Traitement par lots

```typescript
import { scheduled, asyncScheduler, bufferCount, mergeMap, delay } from 'rxjs';

// Traite un grand nombre de tâches 1000 à la fois
const tasks = Array.from({ length: 10000 }, (_, i) => `Task-${i}`);

scheduled(tasks, asyncScheduler).pipe(
  bufferCount(1000), // Lots de 1000 à la fois
  mergeMap(batch => {
    console.log(`Traitement du lot: ${batch.length} éléments`);
    // Exécute le traitement par lots
    return processBatch(batch);
  })
).subscribe({
  complete: () => console.log('Tout le traitement par lots est terminé')
});

function processBatch(batch: string[]): Promise<void> {
  // Logique de traitement des lots
  return Promise.resolve();
}
```

### 3. Implémentation d'animation

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';

// Génère des valeurs de 0 à 100
const frames = Array.from({ length: 100 }, (_, i) => i);

// Exécution à chaque frame d'animation
const animation$ = scheduled(frames, animationFrameScheduler).pipe(
  map(frame => ({
    progress: frame / 100,
    position: frame * 5 // Déplacement de 0px à 500px
  }))
);

animation$.subscribe({
  next: ({ progress, position }) => {
    const element = document.getElementById('animated-box');
    if (element) {
      element.style.transform = `translateX(${position}px)`;
      console.log(`Progression: ${(progress * 100).toFixed(0)}%`);
    }
  },
  complete: () => console.log('Animation terminée')
});
```

### 4. Traitement des tâches par ordre de priorité

```typescript
import { scheduled, asapScheduler, asyncScheduler } from 'rxjs';

// Tâches de haute priorité (asapScheduler = microtask)
const highPriorityTasks = ['Tâche urgente 1', 'Tâche urgente 2'];
const highPriority$ = scheduled(highPriorityTasks, asapScheduler);

// Tâches de faible priorité (asyncScheduler = macrotask)
const lowPriorityTasks = ['Tâche normale 1', 'Tâche normale 2'];
const lowPriority$ = scheduled(lowPriorityTasks, asyncScheduler);

console.log('Début des tâches');

highPriority$.subscribe(task => console.log('Haute priorité:', task));
lowPriority$.subscribe(task => console.log('Faible priorité:', task));

console.log('Enregistrement des tâches terminé');

// Sortie:
// Début des tâches
// Enregistrement des tâches terminé
// Haute priorité: Tâche urgente 1
// Haute priorité: Tâche urgente 2
// Faible priorité: Tâche normale 1
// Faible priorité: Tâche normale 2
```

## Options de scheduled()

`scheduled()` a la signature suivante.

```typescript
function scheduled<T>(
  input: ObservableInput<T>,
  scheduler: SchedulerLike
): Observable<T>
```

### Types d'entrée supportés

- **Array** : `T[]`
- **Promise** : `Promise<T>`
- **Iterable** : `Iterable<T>` (Set, Map, Generator, etc.)
- **Observable** : `Observable<T>`
- **ArrayLike** : `ArrayLike<T>`

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Array
scheduled([1, 2, 3], asyncScheduler);

// Promise
scheduled(Promise.resolve('résultat'), asyncScheduler);

// Set
scheduled(new Set([1, 2, 3]), asyncScheduler);

// Generator
function* generator() {
  yield 1;
  yield 2;
  yield 3;
}
scheduled(generator(), asyncScheduler);
```

## Erreurs courantes et leurs solutions

### 1. Oubli de spécifier le scheduler

**Exemple d'erreur:**
```typescript
// ❌ Erreur : 2ème argument requis
const observable$ = scheduled([1, 2, 3]);
```

**Solution:**
```typescript
// ✅ Correct : spécifier le scheduler
const observable$ = scheduled([1, 2, 3], asyncScheduler);
```

### 2. Utilisation de animationFrameScheduler dans un environnement Node.js

**Problème:**
Dans les environnements Node.js, `requestAnimationFrame` n'existe pas, ce qui provoque des erreurs.

**Solution:**
```typescript
import { scheduled, animationFrameScheduler, asyncScheduler } from 'rxjs';

// Vérifier l'environnement du navigateur
const scheduler = typeof window !== 'undefined'
  ? animationFrameScheduler
  : asyncScheduler;

const observable$ = scheduled([1, 2, 3], scheduler);
```

### 3. Confusion entre traitement synchrone et asynchrone

**Problème:**
```typescript
// S'attend à une exécution asynchrone, mais est en fait synchrone
scheduled([1, 2, 3], queueScheduler).subscribe(val => {
  console.log(val);
});
console.log('Terminé'); // ← 1, 2, 3 sortent avant ceci
```

**Solution:**
```typescript
// Spécifier explicitement l'asynchronisme
scheduled([1, 2, 3], asyncScheduler).subscribe(val => {
  console.log(val);
});
console.log('Terminé'); // ← 1, 2, 3 sont produits après ceci
```

## Comparaison avec from()

| Fonctionnalité | from() | scheduled() |
|----------------|--------|-------------|
| Spécification du scheduler | ❌ Pas possible (par défaut uniquement) | ✅ Spécification explicite |
| Contrôle synchrone/asynchrone | ❌ Impossible à contrôler | ✅ Contrôlable |
| Facilité de test | Normal | ✅ Temps contrôlable avec TestScheduler |
| Simplicité | ✅ Simple | Un peu complexe |
| Cas d'utilisation | Conversion de base | Lorsque le contrôle du temps d'exécution est nécessaire |

> [!TIP]
> **Points de choix**
>
> - **En principe, utilisez `from()`** : Lorsque le contrôle du scheduler n'est pas nécessaire
> - **Utilisez `scheduled()` quand** :
>   - Vous voulez éviter le blocage de l'interface utilisateur
>   - Vous avez besoin d'un contrôle du temps dans les tests
>   - Implémentation d'animation
>   - Traitement des tâches par ordre de priorité

## Meilleures pratiques

### 1. Utiliser asyncScheduler pour le traitement de données volumineuses

```typescript
// ✅ Bon exemple : ne bloque pas l'interface utilisateur
scheduled(largeArray, asyncScheduler).pipe(
  map(processHeavyTask)
).subscribe();
```

### 2. Utiliser TestScheduler pour les tests

```typescript
// ✅ Bon exemple : contrôler le temps virtuellement
testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

### 3. Utiliser animationFrameScheduler pour l'animation

```typescript
// ✅ Bon exemple : correspondre au timing de repeinture du navigateur
scheduled(frames, animationFrameScheduler).subscribe(updateUI);
```

### 4. Sélectionner le scheduler qui convient le mieux à votre environnement

```typescript
// ✅ Bon exemple : basculer en fonction de l'environnement
const scheduler = process.env.NODE_ENV === 'test'
  ? queueScheduler
  : asyncScheduler;

const source$ = scheduled(data, scheduler);
```

## Résumé

`scheduled()` est une fonction de création qui crée un Observable en spécifiant explicitement un scheduler.

**Caractéristiques principales:**
- Contrôle explicite du timing d'exécution (synchrone ou asynchrone)
- Choix de plusieurs schedulers
- Facile à tester avec TestScheduler
- Efficace pour éviter le blocage de l'interface utilisateur

**Scénarios d'utilisation:**
- Traitement asynchrone de grandes quantités de données
- Implémentation d'animations
- Contrôle du temps dans les tests
- Traitement des tâches par ordre de priorité

**Notes:**
- Toujours spécifier un scheduler
- Sélectionner le scheduler approprié pour votre environnement
- Comprendre la différence entre from() et scheduled()

**Utilisation recommandée:**
- Optimisation de l'interface utilisateur : `asyncScheduler`
- Animation : `animationFrameScheduler`
- Test : `TestScheduler`
- Haute priorité : `asapScheduler`

## Pages connexes

- [using()](/fr/guide/creation-functions/control/using) - Observable avec contrôle des ressources
- [Fonctions de création de contrôle](/fr/guide/creation-functions/control/) - Comparaison entre scheduled() et using()
- [Types de schedulers](/fr/guide/schedulers/types) - Détails des schedulers
- [from()](/fr/guide/creation-functions/basic/from) - Génération d'Observable de base

## Références

- [Documentation officielle RxJS - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [Documentation officielle RxJS - Scheduler](https://rxjs.dev/guide/scheduler)
- [Documentation officielle RxJS - TestScheduler](https://rxjs.dev/api/testing/TestScheduler)
