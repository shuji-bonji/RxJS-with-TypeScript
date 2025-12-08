---
description: "L'opérateur skipUntil ignore toutes les valeurs de l'Observable source jusqu'à ce qu'un autre Observable émette une valeur, puis émet des valeurs normalement. Utile pour un démarrage différé dans le temps ou un traitement après l'occurrence d'un événement spécifique."
---

# skipUntil - Ignorer jusqu'à ce qu'un autre Observable émette

L'opérateur `skipUntil` **ignore toutes les valeurs de l'Observable source jusqu'à ce que l'Observable spécifié (déclencheur de notification) émette sa première valeur**. Après que le déclencheur émet, les valeurs sont émises normalement.


## 🔰 Syntaxe de base et utilisation

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // Émet toutes les 0.5 secondes
const notifier$ = timer(2000); // Émet après 2 secondes

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Sortie: 4, 5, 6, 7, 8, ...
// (Les 2 premières secondes 0, 1, 2, 3 sont ignorées)
```

**Flux d'opération** :
1. `source$` émet 0, 1, 2, 3 → tout ignoré
2. Après 2 secondes, `notifier$` émet une valeur
3. Les valeurs suivantes de `source$` (4, 5, 6, ...) sont émises normalement

[🌐 Documentation officielle RxJS - `skipUntil`](https://rxjs.dev/api/operators/skipUntil)


## 🆚 Comparaison avec takeUntil

`skipUntil` et `takeUntil` ont des comportements opposés.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500); // Émet toutes les 0.5 secondes
const notifier$ = timer(2000); // Émet après 2 secondes

// takeUntil: récupère les valeurs jusqu'à la notification
source$.pipe(
  takeUntil(notifier$)
).subscribe(console.log);
// Sortie: 0, 1, 2, 3 (s'arrête après 2 secondes)

// skipUntil: ignore les valeurs jusqu'à la notification
source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Sortie: 4, 5, 6, 7, ... (commence après 2 secondes)
```

| Opérateur | Comportement | Moment de fin |
|---|---|---|
| `takeUntil(notifier$)` | **Récupère** les valeurs jusqu'à la notification | Fin automatique à la notification |
| `skipUntil(notifier$)` | **Ignore** les valeurs jusqu'à la notification | Fin du flux original |


## 💡 Patterns d'utilisation typiques

1. **Démarrer le traitement des données après l'authentification**
   ```ts
   import { interval, Subject } from 'rxjs';
   import { skipUntil } from 'rxjs';

   const authenticated$ = new Subject<void>();
   const dataStream$ = interval(1000);

   // Ignorer les données jusqu'à la fin de l'authentification
   dataStream$.pipe(
     skipUntil(authenticated$)
   ).subscribe(data => {
     console.log(`Traitement des données: ${data}`);
   });

   // Authentification terminée après 3 secondes
   setTimeout(() => {
     console.log('Authentification terminée!');
     authenticated$.next();
   }, 3000);
   // Après 3 secondes: "Traitement des données: 3", "Traitement des données: 4"...
   ```

2. **Démarrer le traitement des événements après le chargement initial**
   ```ts
   import { fromEvent, BehaviorSubject } from 'rxjs';
   import { filter, skipUntil } from 'rxjs';

   const appReady$ = new BehaviorSubject<boolean>(false);
   const button = document.createElement('button');
   button.textContent = 'Cliquer';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');

   // Ignorer les clics jusqu'à ce que l'application soit prête
   clicks$.pipe(
     skipUntil(appReady$.pipe(filter(ready => ready)))
   ).subscribe(() => {
     console.log('Le clic a été traité');
   });

   // L'application est prête après 2 secondes
   setTimeout(() => {
     console.log('Application prête');
     appReady$.next(true);
   }, 2000);
   ```

3. **Démarrage différé basé sur un timer**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { skipUntil, scan } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Compter';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');
   const startTime$ = timer(3000); // Après 3 secondes

   // Ne pas compter les clics avant 3 secondes
   clicks$.pipe(
     skipUntil(startTime$),
     scan(count => count + 1, 0)
   ).subscribe(count => {
     console.log(`Compteur: ${count}`);
   });

   console.log('Le comptage commence dans 3 secondes...');
   ```


## 🧠 Exemple de code pratique (compte à rebours de jeu)

Exemple où les clics sont ignorés pendant le compte à rebours avant le début du jeu, et les clics ne sont valides qu'après la fin du compte à rebours.

```ts
import { fromEvent, timer, interval } from 'rxjs';
import { skipUntil, take, scan } from 'rxjs';

// Création des éléments UI
const container = document.createElement('div');
document.body.appendChild(container);

const countdown = document.createElement('div');
countdown.style.fontSize = '24px';
countdown.style.marginBottom = '10px';
countdown.textContent = 'Compte à rebours en cours...';
container.appendChild(countdown);

const button = document.createElement('button');
button.textContent = 'Cliquer!';
button.disabled = true;
container.appendChild(button);

const scoreDisplay = document.createElement('div');
scoreDisplay.style.marginTop = '10px';
scoreDisplay.textContent = 'Score: 0';
container.appendChild(scoreDisplay);

// Compte à rebours (3 secondes)
const countdownTimer$ = interval(1000).pipe(take(3));
countdownTimer$.subscribe({
  next: (n) => {
    countdown.textContent = `Démarrage dans ${3 - n} secondes...`;
  },
  complete: () => {
    countdown.textContent = 'Jeu démarré!';
    button.disabled = false;
  }
});

// Notification de début de jeu
const gameStart$ = timer(3000);

// Événement de clic (ignoré jusqu'au début du jeu)
const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  skipUntil(gameStart$),
  scan(score => score + 10, 0)
).subscribe(score => {
  scoreDisplay.textContent = `Score: ${score}`;
});
```

Dans ce code, les clics sont ignorés pendant les 3 secondes de compte à rebours, et seuls les clics après la fin du compte à rebours sont comptés dans le score.


## 🎯 Différence entre skip et skipUntil

```ts
import { interval, timer } from 'rxjs';
import { skip, skipUntil } from 'rxjs';

const source$ = interval(500);

// skip: ignorer les N premiers par nombre
source$.pipe(
  skip(3)
).subscribe(console.log);
// Sortie: 3, 4, 5, 6, ...

// skipUntil: ignorer jusqu'à ce qu'un autre Observable émette
source$.pipe(
  skipUntil(timer(1500))
).subscribe(console.log);
// Sortie: 3, 4, 5, 6, ... (même résultat mais méthode de contrôle différente)
```

| Opérateur | Condition d'ignorance | Cas d'utilisation |
|---|---|---|
| `skip(n)` | Ignorer les n premiers par nombre | Ignorance d'un nombre fixe |
| `skipWhile(predicate)` | Ignorer tant que la condition est satisfaite | Ignorance basée sur une condition |
| `skipUntil(notifier$)` | Ignorer jusqu'à ce qu'un autre Observable émette | Ignorance basée sur événement/temps |


## 📋 Utilisation type-safe

Exemple d'implémentation type-safe avec les génériques TypeScript.

```ts
import { Observable, Subject, fromEvent } from 'rxjs';
import { skipUntil, map } from 'rxjs';

interface GameState {
  status: 'waiting' | 'ready' | 'playing' | 'finished';
}

interface ClickEvent {
  timestamp: number;
  x: number;
  y: number;
}

class Game {
  private gameReady$ = new Subject<void>();
  private state: GameState = { status: 'waiting' };

  startGame(element: HTMLElement): Observable<ClickEvent> {
    const clicks$ = fromEvent<MouseEvent>(element, 'click').pipe(
      map(event => ({
        timestamp: Date.now(),
        x: event.clientX,
        y: event.clientY
      } as ClickEvent)),
      skipUntil(this.gameReady$)
    );

    // Notifier que le jeu est prêt
    setTimeout(() => {
      this.state = { status: 'ready' };
      this.gameReady$.next();
      console.log('Jeu prêt!');
    }, 2000);

    return clicks$;
  }
}

// Exemple d'utilisation
const game = new Game();
const canvas = document.createElement('div');
canvas.style.width = '300px';
canvas.style.height = '200px';
canvas.style.border = '1px solid black';
canvas.textContent = 'Cliquez ici';
document.body.appendChild(canvas);

game.startGame(canvas).subscribe(click => {
  console.log(`Position du clic: (${click.x}, ${click.y})`);
});
```


## 🔄 Combinaison de skipUntil et takeUntil

Pour récupérer des valeurs uniquement pendant une période spécifique, combinez les deux.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500);
const start$ = timer(2000); // Démarrer après 2 secondes
const stop$ = timer(5000);  // S'arrêter après 5 secondes

source$.pipe(
  skipUntil(start$), // Ignorer jusqu'à 2 secondes
  takeUntil(stop$)   // S'arrêter après 5 secondes
).subscribe({
  next: console.log,
  complete: () => console.log('Terminé')
});
// Sortie: 4, 5, 6, 7, 8, 9, Terminé
// (Récupère uniquement les valeurs entre 2 et 5 secondes)
```

**Chronologie** :
```
0s    1s    2s    3s    4s    5s
|-----|-----|-----|-----|-----|
0  1  2  3  4  5  6  7  8  9  10
      ↑           ↑
   début skip  fin take
   (à partir de 4) (jusqu'à 9)
```


## ⚠️ Erreurs courantes

> [!IMPORTANT]
> `skipUntil` n'est valide que pour la **première émission** de l'Observable de notification. Les émissions suivantes sont ignorées.

### Incorrect : l'Observable de notification émet plusieurs fois

```ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject<void>();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ❌ Mauvais exemple : appeler next plusieurs fois n'a d'effet que la première fois
setTimeout(() => notifier$.next(), 1000);
setTimeout(() => notifier$.next(), 2000); // Ceci est inutile
```

### Correct : comprendre que seule la première émission est valide

```ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject<void>();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ✅ Bon exemple : appeler next une seule fois
setTimeout(() => {
  console.log('Fin de l\'ignorance');
  notifier$.next();
  notifier$.complete(); // Bonne pratique de terminer
}, 1000);
```


## 🎓 Résumé

### Quand utiliser skipUntil
- ✅ Lorsque vous voulez démarrer le traitement après un événement spécifique
- ✅ Lorsque vous voulez activer les opérations utilisateur après l'initialisation
- ✅ Lorsque vous avez besoin d'un démarrage différé basé sur le temps
- ✅ Lorsque vous voulez démarrer le traitement des données après l'authentification

### Combinaison avec takeUntil
- ✅ Lorsque vous voulez récupérer des valeurs uniquement pendant une période spécifique (skipUntil + takeUntil)

### Points d'attention
- ⚠️ Seule la première émission de l'Observable de notification est valide
- ⚠️ Si l'Observable de notification n'émet jamais, toutes les valeurs sont ignorées
- ⚠️ La souscription est maintenue jusqu'à la fin du flux original


## 🚀 Prochaines étapes

- **[skip](./skip)** - Apprendre à ignorer les N premières valeurs
- **[take](./take)** - Apprendre à récupérer les N premières valeurs
- **[takeUntil](../utility/takeUntil)** - Apprendre à récupérer jusqu'à ce qu'un autre Observable émette
- **[filter](./filter)** - Apprendre le filtrage basé sur les conditions
- **[Exemples pratiques d'opérateurs de filtrage](./practical-use-cases)** - Apprendre des cas d'utilisation réels
