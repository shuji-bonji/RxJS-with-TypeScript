---
description: "En savoir plus sur les caractéristiques, la mise en œuvre et l'utilisation des principaux schedulers de RxJS, tels que asyncScheduler et queueScheduler. Comprenez les différences entre les macro-tâches, les micro-tâches et le traitement synchrone, et apprenez le calendrier d'exécution et les caractéristiques de chaque planificateur. Vous pouvez optimiser les performances et le comportement de votre application en les utilisant de manière appropriée."
---

# Types d'ordonnanceurs et leur utilisation

RxJS fournit plusieurs ordonnanceurs pour différentes applications. Chaque planificateur a ses propres caractéristiques et son propre timing d'exécution et peut être utilisé de manière appropriée pour optimiser les performances et le comportement de l'application.

## Classification des ordonnanceurs

Les ordonnanceurs de RxJS se répartissent en trois catégories principales.

1) **Tâches macro** : exécutées dans la file d'attente de la tâche suivante dans la boucle d'événements.
2. **Micro-tâche** : exécutée immédiatement après la fin de la tâche en cours et avant le début de la tâche suivante
3. **Traitement synchrone** : exécution immédiate

Pour plus d'informations, voir [Notions de base sur les tâches et les planificateurs](./task-and-scheduler-basics.md).

## Ordonnanceurs principaux.

### asyncScheduler

#### Caractéristiques.
- **Mise en œuvre interne** : utilise setTimeout.
- Temps d'exécution : macro-tâches
- Utilisation : traitement asynchrone général, traitement de laps de temps.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Démarrage');

of('Traitement asynchrone')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fin');

// Sortie:
// 1: Démarrage
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
    console.log(`Résultats des calculs: ${result}`);
  });
```

### queueScheduler

#### Caractéristiques.
- **Mise en œuvre interne** : file d'attente de micro-tâches
- Délai d'exécution : au sein de la tâche en cours (semble synchrone)
- Utilisations : mise en file d'attente des tâches, optimisation de la récursivité.

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Démarrage');

of('Traitement des files d'attente')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: Fin');

// Sortie:
// 1: Démarrage
// 2: Traitement des files d'attente
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

#### Caractéristiques.
- **Mise en œuvre interne** : Promise.resolve().then() ou setImmediate
- Temps d'exécution** : microtâches
- **Utilisation** : pour une exécution asynchrone aussi rapide que possible

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Démarrage');

of('ASAPTraitement')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fin');

// Sortie:
// 1: Démarrage
// 2: Fin
// 3: ASAPTraitement
```

#### Cas d'utilisation

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Optimisation des événements de mouvement de la souris
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // UITraitement des mises à jour
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Caractéristiques.
- **Mise en œuvre interne** : requestAnimationFrame
- Moment d'exécution : avant le prochain rendu d'écran
- Utilisation : animation, processus de dessin pour 60 images par seconde.

#### Exemple d'une animation de rotation simple

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// HTMLCréation d'éléments
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Mise en place d'animations
let rotation = 0;

// 60fpsDans les2Secondes d'animation
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2Secondes = 120L'image
    map(() => {
      rotation += 3;  // 1Chaque image3Degré de rotation
      return rotation;
    })
  )
  .subscribe(angle => {
    // DOMRotation réelle de l'élément
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Pourquoi avons-nous besoin de animationFrameScheduler ?

Le `animationFrameScheduler` effectue ses opérations de manière synchrone avec le cycle de dessin du navigateur, ce qui présente les avantages suivants.

1.**Des animations fluides** : comme le traitement est synchronisé avec le temps de rendu du navigateur (typiquement 60 fps), il est possible d'obtenir des animations fluides et sans à-coups.
2. **Utilisation efficace des ressources** : lorsque le navigateur désactive l'onglet, l'exécution de requestAnimationFrame est automatiquement mise en pause, ce qui évite une utilisation inutile de l'unité centrale.
3, **Prévenir le scintillement de l'écran** : le scintillement de l'écran et les images incomplètes sont évités car le calcul est terminé de manière fiable avant que l'écran ne soit dessiné.

Voici une comparaison entre `setInterval` et `animationFrameScheduler`.

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ setIntervalAnimation inefficace utilisant
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // approximativement60fps

// Problèmes:
// - Pas de synchronisation avec les temps de rendu du navigateur
// - Continue à s'exécuter même dans les onglets en arrière-plan
// - La précision60fpsne peut être garantie

// ✅ animationFrameSchedulerUtilisation efficace de l'animation
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
// - Synchronisation avec les temps de dessin du navigateur
// - Mise en pause automatique dans les onglets en arrière-plan
// - Stabilité60fpsPermet
```

#### Exemple d'animation de suivi de souris

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Création d'un cercle à suivre
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Transparent aux événements de la souris
document.body.appendChild(circle);

// Position actuelle et position cible
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Surveillance des mouvements de la souris
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
    
    // Mouvement progressif de la position actuelle vers la position cible々Mouvement progressif (easing) de la position actuelle vers la position cible
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;
    
    // DOMMise à jour de l'élément
    circle.style.left = `${currentX - 15}px`;  // Ajustement à la position centrale
    circle.style.top = `${currentY - 15}px`;
  });
```

## Guide d'utilisation des différents programmateurs

### Comparaison selon le temps d'exécution

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Démarrage');

of('Traitement asynchrone')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fin');

// Sortie:
// 1: Démarrage
// 2: Fin
// 3: Traitement asynchrone
```

### Critères de sélection par application


## Cas d'utilisation pratique

### Traitement de grandes quantités de données


### Traitement des messages WebSocket

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// Remarque: Voici un pseudo-code pour illustrer le concept
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // Traiter comme une chaîne
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

En utilisant l'ordonnanceur avec l'opérateur `retry`, le timing des retry peut être finement contrôlé.

#### Contrôle de répétition de base

L'option `delay` de l'opérateur `retry` utilise le `asyncScheduler` en interne pour contrôler l'intervalle de retry.


```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Démarrage');

of('Traitement asynchrone')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fin');

// Sortie:
// 1: Démarrage
// 2: Fin
// 3: Traitement asynchrone
```

#### Utilisation de l'ordonnanceur avec un back-off exponentiel

En tant que contrôle plus avancé, le backoff exponentiel peut être implémenté en combinant `retryWhen` et `asyncScheduler`.

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
    // RxJS 7.3+ 推奨: retry({ count, delay }) 形式
    retry({
      count: 3, // Max.3回まで再試行
      delay: (error, retryCount) => {
        // 指数バックオフ: 1秒, 2秒, 4秒...
        const delayTime = Math.pow(2, retryCount - 1) * 1000;
        console.log(`🔄 リトライ ${retryCount}回目 (${delayTime}ms後)`);

        // timer は内部的に asyncScheduler を使用
        return timer(delayTime);
      }
    })
  )
  .subscribe({
    next: result => console.log('✅ 成功:', result),
    error: error => {
      console.log('❌ Max.リトライ数に到達');
      console.log('❌ 最終Erreur:', error.message);
    }
  });

// SortieEx.:
// 🔄 リトライ 1回目 (1000ms後)
// 🔄 リトライ 2回目 (2000ms後)
// 🔄 リトライ 3回目 (4000ms後)
// ❌ Max.リトライ数に到達
// ❌ 最終Erreur: Temporary error
```

#### Lorsque l'on spécifie explicitement un Scheduler asynchrone

Spécifier explicitement un scheduler spécifique permet un contrôle plus flexible, comme le remplacer par `TestScheduler` pendant les tests.


```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Démarrage');

of('Traitement asynchrone')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fin');

// Sortie:
// 1: Démarrage
// 2: Fin
// 3: Traitement asynchrone
```


> - Des modèles d'implémentation détaillés et des méthodes de débogage pour la gestion des retry sont décrits sur la page [retry et catchError](/fr/guide/error-handling/retry-catch).
> - Utilisation détaillée de l'opérateur retry.
> - Les modèles de combinaison avec catchError.
> - Techniques de débogage pour les tentatives (par exemple, suivi du nombre de tentatives, journalisation, etc.)

## Impact sur les performances.

### Surcharge de l'ordonnanceur


```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Démarrage');

of('Traitement asynchrone')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fin');

// Sortie:
// 1: Démarrage
// 2: Fin
// 3: Traitement asynchrone
```

## Résumé.

Le choix d'un ordonnanceur a un impact significatif sur les performances et la réactivité d'une application. Comprendre les caractéristiques de chaque ordonnanceur et les utiliser dans les bonnes situations permet d'assurer un fonctionnement efficace et fluide. En règle générale,

- `asyncScheduler` pour le traitement asynchrone général.
- `queueScheduler` pour le traitement récursif et la mise en file d'attente synchrone.
- `asapScheduler` pour les temps de réponse rapides
- `animationFrameScheduler` pour les animations

pour l'animation.