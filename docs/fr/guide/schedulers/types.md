---
description: Explication détaillée des caractéristiques, de l'implémentation et de l'utilisation des principaux schedulers de RxJS tels que asyncScheduler et queueScheduler. Comprenez les différences entre macrotâches, microtâches et traitement synchrone, ainsi que le timing d'exécution et les caractéristiques de chaque scheduler. Une utilisation appropriée permet d'optimiser les performances et le comportement de votre application.
---

# Types de schedulers et comment les utiliser

RxJS fournit plusieurs schedulers adaptés à différents cas d'usage. Chaque scheduler a un timing d'exécution et des caractéristiques spécifiques, et leur utilisation appropriée permet d'optimiser les performances et le comportement de votre application.

## Classification des schedulers

Les schedulers RxJS sont classés en trois grandes catégories.

1. **Macrotâche** : Exécution dans la prochaine file de tâches de la boucle d'événements
2. **Microtâche** : Exécution immédiatement après la fin de la tâche en cours, avant le début de la tâche suivante
3. **Traitement synchrone** : Exécution immédiate

Consultez également [Notions de base sur les tâches et les schedulers](./task-and-scheduler-basics.md) pour plus de détails.

## Principaux schedulers

### asyncScheduler

#### Caractéristiques
- **Implémentation interne** : Utilise setTimeout
- **Timing d'exécution** : Macrotâche
- **Utilisation** : Traitement asynchrone général, traitement impliquant l'écoulement du temps

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Début');

of('Traitement asynchrone')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fin');

// Sortie:
// 1: Début
// 2: Fin
// 3: Traitement asynchrone
```

#### Cas d'utilisation

```ts
import { asyncScheduler, map, observeOn, of } from "rxjs";

function heavyComputation(value: number): number {
  // Simulation d'un calcul lourd
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result = Math.sin(result);
  }
  return result;
}

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(value => heavyComputation(value))
  )
  .subscribe(result => {
    console.log(`Résultat du calcul: ${result}`);
  });
```

### queueScheduler

#### Caractéristiques
- **Implémentation interne** : File de microtâches
- **Timing d'exécution** : Dans la tâche en cours (semble synchrone)
- **Utilisation** : Mise en file d'attente des tâches, optimisation du traitement récursif

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Début');

of('Traitement en file')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: Fin');

// Sortie:
// 1: Début
// 2: Traitement en file
// 3: Fin
```

#### Cas d'utilisation

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs';

// Optimisation du traitement récursif
function fibonacci(n: number): Observable<number> {
  return of([0, 1]).pipe(
    observeOn(queueScheduler),
    expand(([a, b]) => of([b, a + b])),
    map(([a]) => a),
    take(n)
  );
}

fibonacci(10).subscribe(value => console.log(value));
```

### asapScheduler

#### Caractéristiques
- **Implémentation interne** : Promise.resolve().then() ou setImmediate
- **Timing d'exécution** : Microtâche
- **Utilisation** : Lorsque vous souhaitez exécuter de manière asynchrone le plus rapidement possible

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Début');

of('Traitement ASAP')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fin');

// Sortie:
// 1: Début
// 2: Fin
// 3: Traitement ASAP
```

#### Cas d'utilisation

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Optimisation des événements de mouvement de souris
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // Traitement de mise à jour de l'UI
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Caractéristiques
- **Implémentation interne** : requestAnimationFrame
- **Timing d'exécution** : Avant le prochain rendu de l'écran
- **Utilisation** : Animations, traitement de rendu à 60fps

#### Exemple d'animation de rotation simple

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// Création d'un élément HTML
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Configuration de l'animation
let rotation = 0;

// Animation à 60fps pendant 2 secondes
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2 secondes = 120 frames
    map(() => {
      rotation += 3;  // Rotation de 3 degrés par frame
      return rotation;
    })
  )
  .subscribe(angle => {
    // Rotation réelle de l'élément DOM
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Pourquoi animationFrameScheduler est-il nécessaire

`animationFrameScheduler` offre les avantages suivants car il exécute le traitement en synchronisation avec le cycle de rendu du navigateur.

1. **Animations fluides** : En exécutant le traitement en synchronisation avec le timing de rendu du navigateur (généralement 60fps), il permet de réaliser des animations fluides sans saccades.
2. **Utilisation efficace des ressources** : Lorsque le navigateur désactive l'onglet, l'exécution de requestAnimationFrame est automatiquement suspendue, évitant ainsi une utilisation inutile du CPU.
3. **Prévention du scintillement de l'écran** : En s'assurant que les calculs sont terminés avant le rendu de l'écran, il évite le scintillement de l'écran et l'affichage de frames incomplètes.

Voici une comparaison entre `setInterval` et `animationFrameScheduler`.

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ Animation inefficace utilisant setInterval
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // Environ 60fps

// Problèmes:
// - Non synchronisé avec le timing de rendu du navigateur
// - Continue à s'exécuter même dans les onglets en arrière-plan
// - Ne peut pas garantir exactement 60fps

// ✅ Animation efficace utilisant animationFrameScheduler
interval(0, animationFrameScheduler)
  .pipe(
    map(() => {
      position += 1;
      return position;
    })
  )
  .subscribe(pos => {
    element.style.transform = `translateX(${pos}px)`;
  });

// Avantages
// - Synchronisé avec le timing de rendu du navigateur
// - Suspendu automatiquement dans les onglets en arrière-plan
// - Réalise un 60fps stable
```


#### Exemple d'animation de suivi de souris

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Création d'un cercle qui suit
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Transparence aux événements de souris
document.body.appendChild(circle);

// Position actuelle et position cible
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Surveillance des événements de mouvement de souris
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove')
  .pipe(
    map(event => ({
      x: event.clientX,
      y: event.clientY
    }))
  );

// Boucle d'animation
interval(0, animationFrameScheduler)
  .pipe(
    withLatestFrom(mouseMove$),
    map(([_, mousePos]) => mousePos)
  )
  .subscribe(({ x, y }) => {
    // Définir la position de la souris comme cible
    targetX = x;
    targetY = y;

    // Déplacement progressif de la position actuelle vers la position cible (easing)
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;

    // Mise à jour de l'élément DOM
    circle.style.left = `${currentX - 15}px`;  // Ajustement à la position centrale
    circle.style.top = `${currentY - 15}px`;
  });
```

## Guide d'utilisation des schedulers

### Comparaison par timing d'exécution

```ts
import { of, asyncScheduler, queueScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Début');

// Traitement synchrone
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler (microtâche)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler (microtâche)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler (macrotâche)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(v => console.log(`5: ${v}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Fin');

// Ordre d'exécution:
// 1: Début
// 2: sync
// 7: Fin
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```

### Critères de sélection par usage

| Scheduler | Caractéristiques | Utilisation appropriée |
|--------------|------|----------|
| asyncScheduler | Utilise setTimeout, complètement asynchrone | Traitements longs, exécution différée |
| queueScheduler | Synchrone mais optimise la récursivité | Traitement récursif, gestion de file de tâches |
| asapScheduler | Exécution asynchrone la plus rapide possible | Gestion d'événements, traitement nécessitant une réponse rapide |
| animationFrameScheduler | Synchronisé avec le rendu d'écran | Animations, mise à jour d'UI, développement de jeux |

## Exemples d'utilisation pratiques

### Traitement de grandes quantités de données

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
```

### Traitement des messages WebSocket

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// Note : Ceci est un pseudo-code montrant le concept
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // Traité comme une chaîne
});

socket$
  .pipe(
    // Traitement des messages nécessitant une réponse rapide
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('Message reçu:', msg);
}
```

### Contrôle des tentatives d'erreur

Utiliser un scheduler avec l'opérateur `retry` permet de contrôler finement le timing des tentatives.

#### Contrôle de base des tentatives

L'option `delay` de l'opérateur `retry` utilise en interne `asyncScheduler` pour contrôler l'intervalle entre les tentatives.

```ts
import { throwError, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

// Simulation d'un appel API
function fetchData(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.7) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Network error'));
    })
  );
}

fetchData(1)
  .pipe(
    retry({
      count: 3,
      delay: 1000  // Attendre 1 seconde avec asyncScheduler avant de réessayer
    })
  )
  .subscribe({
    next: result => console.log('✅ Succès:', result),
    error: error => console.log('❌ Erreur finale:', error.message)
  });
```

#### Utilisation du scheduler avec le backoff exponentiel

Pour un contrôle plus avancé, vous pouvez implémenter un backoff exponentiel en combinant `retry` et `asyncScheduler`.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

function fetchDataWithBackoff(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.9) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Temporary error'));
    })
  );
}

fetchDataWithBackoff(1)
  .pipe(
    // RxJS 7.3+ recommandé : forme retry({ count, delay })
    retry({
      count: 3, // Maximum 3 réessais
      delay: (error, retryCount) => {
        // Backoff exponentiel: 1s, 2s, 4s...
        const delayTime = Math.pow(2, retryCount - 1) * 1000;
        console.log(`🔄 Tentative ${retryCount} (dans ${delayTime}ms)`);

        // timer utilise asyncScheduler en interne
        return timer(delayTime);
      }
    })
  )
  .subscribe({
    next: result => console.log('✅ Succès:', result),
    error: error => {
      console.log('❌ Nombre maximal de tentatives atteint');
      console.log('❌ Erreur finale:', error.message);
    }
  });

// Exemple de sortie:
// 🔄 Tentative 1 (dans 1000ms)
// 🔄 Tentative 2 (dans 2000ms)
// 🔄 Tentative 3 (dans 4000ms)
// ❌ Nombre maximal de tentatives atteint
// ❌ Erreur finale: Temporary error
```

#### Spécification explicite d'asyncScheduler

En spécifiant explicitement un scheduler spécifique, vous obtenez un contrôle plus flexible, comme le remplacement par `TestScheduler` lors des tests.

```ts
import { throwError, asyncScheduler, of } from 'rxjs';
import { retry, mergeMap, delay } from 'rxjs';

function fetchDataWithScheduler(id: number, scheduler = asyncScheduler) {
  return of(id).pipe(
    mergeMap(() => throwError(() => new Error('Error'))),
    // RxJS 7.3+ recommandé : forme retry({ count, delay })
    retry({
      count: 2, // Maximum 2 réessais
      delay: (error, retryCount) => {
        // Spécifier explicitement le scheduler
        return of(null).pipe(
          delay(1000, scheduler)
        );
      }
    })
  );
}

// Environnement de production : utilise asyncScheduler
fetchDataWithScheduler(1).subscribe({
  error: err => console.log('Erreur:', err.message)
});

// Environnement de test : peut être remplacé par TestScheduler
```

> [!TIP]
> Pour les modèles d'implémentation détaillés et les méthodes de débogage des traitements de tentative, consultez la page [retry et catchError](/fr/guide/error-handling/retry-catch).
> - Utilisation détaillée de l'opérateur retry
> - Modèles de combinaison avec catchError
> - Techniques de débogage des tentatives (suivi du nombre de tentatives, journalisation, etc.)

## Impact sur les performances

### Surcharge du scheduler

```ts
import { range, asyncScheduler, pipe } from 'rxjs';
import { bufferCount, map, observeOn, tap } from 'rxjs';

// ❌ Utilisation excessive du scheduler
range(1, 1000)
  .pipe(
    observeOn(asyncScheduler),  // 1000 setTimeout
    map(x => x * 2),
    // tap(console.log)
  )
  .subscribe();

// ✅ Optimisation par traitement par lots
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeout
    map(batch => batch.map(x => x * 2)),
    // tap(console.log)
  )
  .subscribe();
```

## Résumé

Le choix du scheduler a un impact important sur les performances et la réactivité de votre application. En comprenant les caractéristiques de chaque scheduler et en les utilisant de manière appropriée, vous pouvez réaliser un fonctionnement efficace et fluide. En règle générale,

- Utilisez `asyncScheduler` pour le traitement asynchrone général
- Utilisez `queueScheduler` pour le traitement récursif ou la mise en file synchrone
- Utilisez `asapScheduler` lorsqu'une réponse rapide est nécessaire
- Utilisez `animationFrameScheduler` pour les animations

sont recommandés.
