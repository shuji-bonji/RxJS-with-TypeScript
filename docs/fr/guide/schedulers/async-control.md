---
description: Explication de l'utilisation des schedulers dans RxJS et comment contrôler le traitement asynchrone avec observeOn et subscribeOn. Présente des techniques pratiques d'optimisation des performances et d'évitement du blocage de l'UI, avec des exemples de code TypeScript, notamment le contrôle du timing d'exécution, la gestion du contexte d'exécution et la priorisation des tâches.
---
# Contrôle du traitement asynchrone

Les schedulers dans RxJS sont un mécanisme important pour contrôler le timing et le contexte d'exécution du traitement asynchrone. Ce chapitre explique comment contrôler le traitement asynchrone à l'aide de schedulers.

## Rôle des schedulers

Les schedulers jouent trois rôles importants.

|Rôle|Description|
|---|---|
|Contrôle du timing d'exécution|Détermine quand exécuter les tâches|
|Gestion du contexte d'exécution|Détermine dans quel thread ou environnement d'exécution les tâches s'exécutent|
|Priorisation des tâches|Gère l'ordre d'exécution de plusieurs tâches|

## Comprendre le traitement synchrone et asynchrone

### Comportement par défaut (exécution synchrone)

Par défaut, les opérateurs RxJS s'exécutent de manière synchrone dans la mesure du possible.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

console.log('Début exécution');

of(1, 2, 3)
  .pipe(
    map((x) => {
      console.log(`map: ${x}`);
      return x * 2;
    })
  )
  .subscribe((x) => console.log(`subscribe: ${x}`));

console.log('Fin exécution');

// Sortie:
// Début exécution
// map: 1
// subscribe: 2
// map: 2
// subscribe: 4
// map: 3
// subscribe: 6
// Fin exécution
```

### Asynchronisation avec le scheduler

L'utilisation d'un scheduler permet de rendre le traitement asynchrone.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Début exécution');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(x => console.log(`subscribe: ${x}`));

console.log('Fin exécution');

// Sortie:
// Début exécution
// Fin exécution
// subscribe: 1
// subscribe: 2
// subscribe: 3
```

## Opérateurs utilisant des schedulers

### Opérateur observeOn

`observeOn` modifie le contexte d'exécution du flux. Il émet les valeurs avec le scheduler spécifié.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { take, observeOn } from 'rxjs';

// Exemple d'utilisation pour l'animation
interval(16)
  .pipe(
    take(10),
    observeOn(animationFrameScheduler)
  )
  .subscribe(() => {
    // Exécution synchronisée avec les frames d'animation
    updateAnimation();
  });

function updateAnimation() {
  // Traitement de mise à jour de l'animation
}
```

> [!TIP]
> Pour une explication détaillée, des exemples pratiques et des points d'attention concernant l'opérateur `observeOn`, consultez la page de l'opérateur [observeOn](../operators/utility/observeOn.md).

### Opérateur subscribeOn

`subscribeOn` contrôle le timing du démarrage de la souscription au flux.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, tap } from 'rxjs';

console.log('Avant souscription');

of('Exécution tâche')
  .pipe(
    tap(() => console.log('Début tâche')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(value => console.log(value));

console.log('Après souscription');

// Sortie:
// Avant souscription
// Après souscription
// Début tâche
// Exécution tâche
```

> [!TIP]
> Pour une explication détaillée, des exemples pratiques et les différences avec `observeOn` concernant l'opérateur `subscribeOn`, consultez la page de l'opérateur [subscribeOn](../operators/utility/subscribeOn.md).

## Exemples pratiques de traitement asynchrone

### Contrôle des requêtes API

```ts
import { from, queueScheduler } from 'rxjs';
import { mergeMap, observeOn, tap } from 'rxjs';

interface ApiRequest {
  endpoint: string;
  id: number;
}

const requests: ApiRequest[] = [
  { endpoint: '/users', id: 1 },
  { endpoint: '/posts', id: 1 },
  { endpoint: '/comments', id: 1 },
];

// Mise en file et traitement séquentiel des requêtes
from(requests)
  .pipe(
    observeOn(queueScheduler),
    tap((req) => console.log(`Ajouté à la file: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // Simulation d'une requête API réelle
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`Résultat de ${req.endpoint}/${req.id}`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`Terminé: ${result}`));

// Sortie:
// Ajouté à la file: /users
// Ajouté à la file: /posts
// Ajouté à la file: /comments
// Terminé: Résultat de /users/1
// Terminé: Résultat de /posts/1
// Terminé: Résultat de /comments/1
```

### Éviter le blocage du thread UI

Lors du traitement de grandes quantités de données, utilisez un scheduler pour éviter de bloquer le thread UI.

```ts
import { from, asapScheduler } from 'rxjs';
import { observeOn, bufferCount } from 'rxjs';

const largeDataSet = Array.from({ length: 10000 }, (_, i) => i);

// Taille du lot
const batchSize = 100;
// Calcul du nombre total de lots
const totalBatches = Math.ceil(largeDataSet.length / batchSize);
// Compteur de lots
let batchIndex = 0;

from(largeDataSet)
  .pipe(
    bufferCount(100), // Grouper par 100
    observeOn(asapScheduler) // Le plus rapide possible, mais sans bloquer l'UI
  )
  .subscribe((batch) => {
    batchIndex++;
    processBatch(batch, batchIndex, totalBatches);
  });

function processBatch(
  batch: number[],
  batchIndex: number,
  totalBatches: number
) {
  // Traitement des données du lot
  const processed = batch.map((n) => n * 2);
  console.log(
    `Lot ${batchIndex} sur ${totalBatches} terminé: ${processed.length} éléments traités.`
  );
}

// Sortie:
// Lot 1 sur 100 terminé: 100 éléments traités.
// Lot 2 sur 100 terminé: 100 éléments traités.
// ...
// ...
// Lot 100 sur 100 terminé: 100 éléments traités.
```

## Optimisation des performances et débogage

### Test utilisant un scheduler

```ts
import { TestScheduler } from 'rxjs/testing';
import { delay } from 'rxjs';
import { beforeEach, describe, expect, it } from 'vitest';

describe('Test du traitement asynchrone', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Test de l\'opérateur delay', () => {
    scheduler.run(({ cold, expectObservable }) => {
      const source = cold('a-b-c|');
      const expected =    '1000ms a-b-(c|)';

      const result = source.pipe(delay(1000, scheduler));

      expectObservable(result).toBe(expected);
    });
  });
});
```

### Sortie de logs pour le débogage

```ts
import { of, asyncScheduler } from 'rxjs';
import { tap, observeOn } from 'rxjs';

console.log('Début');

of(1, 2, 3)
  .pipe(
    tap(value => console.log(`[Avant scheduler・synchrone] Valeur: ${value}`)),
    observeOn(asyncScheduler),  // Utilise asyncScheduler
    tap(value => console.log(`[Après scheduler・asynchrone] Valeur: ${value}`))
  )
  .subscribe();

console.log('Fin');

// Sortie réelle:
// Début
// [Avant scheduler・synchrone] Valeur: 1
// [Avant scheduler・synchrone] Valeur: 2
// [Avant scheduler・synchrone] Valeur: 3
// Fin
// [Après scheduler・asynchrone] Valeur: 1
// [Après scheduler・asynchrone] Valeur: 2
// [Après scheduler・asynchrone] Valeur: 3
```

Lorsque vous utilisez `asyncScheduler`, vous pouvez confirmer le comportement asynchrone attendu. Contrairement à `queueScheduler` qui utilise la file de microtâches et est donc traité pendant l'exécution du code synchrone, `asyncScheduler` utilise setTimeout en interne et s'exécute donc complètement de manière asynchrone.

## Exemple montrant les différences de comportement des schedulers
Cet exemple montre les différences de timing d'exécution entre différents schedulers.

```ts
import { of, queueScheduler, asyncScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Début');

// Traitement synchrone
of('sync').subscribe(value => console.log(`2: ${value}`));

// queueScheduler (microtâche)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`3: ${value}`));

// asapScheduler (microtâche)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`4: ${value}`));

// asyncScheduler (macrotâche)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`5: ${value}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Fin');

// Ordre de sortie réel:
// 1: Début
// 2: sync
// 7: Fin
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```


## Bonnes pratiques

1. **N'utiliser le scheduler que lorsque nécessaire** : Si le comportement synchrone par défaut est suffisant, n'utilisez pas de scheduler inutilement

2. **Choisir le scheduler approprié** : Sélectionnez le scheduler optimal selon l'usage
   - Animation : `animationFrameScheduler`
   - Éviter le blocage de l'UI : `asapScheduler`
   - Traitement en file : `queueScheduler`
   - Traitement asynchrone : `asyncScheduler`

3. **Surveillance des performances** : Surveillez toujours l'impact sur les performances de l'utilisation du scheduler

4. **Assurer la testabilité** : Utilisez `TestScheduler` pour écrire des tests de traitement asynchrone

## Erreurs courantes et contre-mesures

### Asynchronisation excessive

```ts
// ❌ Asynchronisation inutile
of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Asynchronisation en double
    filter(x => x > 3)
  )
  .subscribe();

// ✅ Asynchroniser uniquement là où c'est nécessaire
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    filter(x => x > 3),
    observeOn(asyncScheduler)  // Asynchroniser globalement à la fin
  )
  .subscribe();
```

### Mauvaise utilisation du scheduler

```ts
// ❌ Mauvaise utilisation
interval(1000)
  .pipe(
    subscribeOn(animationFrameScheduler)  // N'affecte pas interval
  )
  .subscribe();

// ✅ Utilisation correcte
interval(1000, animationFrameScheduler)  // Spécifier le scheduler lors de la création
  .subscribe();
```

## Résumé

Les schedulers sont un outil puissant pour contrôler finement le traitement asynchrone dans RxJS. Une utilisation appropriée permet d'optimiser les performances, d'éviter le blocage du thread UI et de faciliter les tests. Cependant, une asynchronisation excessive peut dégrader les performances, il est donc important de ne les utiliser que lorsque nécessaire.

La section suivante expliquera en détail les types de schedulers spécifiques et comment les utiliser.
