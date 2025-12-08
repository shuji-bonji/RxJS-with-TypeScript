---
description: Explication des bases de la classification des tâches (synchrone, microtâche, macrotâche) et de leur correspondance avec les différents schedulers de RxJS. Comprenez le mécanisme de la boucle d'événements JavaScript, les différences d'ordre d'exécution, et le fonctionnement de setTimeout, Promise, queueMicrotask, afin d'acquérir les connaissances nécessaires pour choisir le bon scheduler RxJS.
---

# Notions de base sur les tâches et les schedulers

## Qu'est-ce que le traitement synchrone

Le traitement synchrone s'exécute immédiatement dans l'ordre où le code est écrit, et ne passe pas à la tâche suivante tant que la tâche précédente n'est pas terminée.

#### Exemple
```ts
console.log('A');
console.log('B');
console.log('C');

// Sortie:
// A
// B
// C
```


## Qu'est-ce que le traitement asynchrone

Le traitement asynchrone ne s'exécute pas immédiatement, mais après la fin du traitement synchrone en cours.
Le traitement asynchrone comprend les « macrotâches » et les « microtâches ».


## Macrotâche
- Tâche exécutée au prochain cycle de la boucle d'événements.
- Exemples : `setTimeout`, `setInterval`, événements du navigateur

#### Exemple d'exécution
```ts
console.log('Start');
setTimeout(() => console.log('Macro Task'), 0);
console.log('End');

// Sortie:
// Start
// End
// Micro Task
```

### Correspondance dans RxJS
- `asyncScheduler`
  - Utilise `setTimeout` en interne
  - Fonctionne comme une macrotâche

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hello')
  .pipe(observeOn(asyncScheduler))
  .subscribe(console.log);

// Sortie:
// Hello
```


## Microtâche
- Tâche exécutée immédiatement après la fin de la tâche en cours, avant le début de la tâche suivante.
- Exemples : `Promise.then`, `queueMicrotask`

#### Exemple d'exécution
```ts
console.log('Start');
Promise.resolve().then(() => console.log('Micro Task'));
console.log('End');

// Sortie:
// Start
// End
// Micro Task
```

### Correspondance dans RxJS
- `asapScheduler`
  - Utilise `Promise.resolve().then()` en interne
  - Fonctionne comme une microtâche

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hi')
  .pipe(observeOn(asapScheduler))
  .subscribe(console.log);

// Sortie:
// Hi
```


## Tâche synchrone
- Code normal exécuté immédiatement.

### Correspondance dans RxJS
- `queueScheduler`
  - Semble synchrone, mais permet un contrôle fin grâce à la mise en file d'attente des tâches.

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Now')
  .pipe(observeOn(queueScheduler))
  .subscribe(console.log);

// Sortie:
// Now
```


## Résumé de l'ordre d'exécution

#### Exemple de code
```ts
console.log('1');

setTimeout(() => console.log('2 (setTimeout)'), 0);
Promise.resolve().then(() => console.log('3 (Promise)'));

console.log('4');

// Sortie:
// 1
// 4
// 3 (Promise) 👈 Microtâche
// 2 (setTimeout) 👈 Macrotâche
```


## Tableau de correspondance entre tâches et schedulers RxJS

| Type         | Exemple                      | Scheduler RxJS      |
|--------------|------------------------------|---------------------|
| Traitement synchrone     | Code normal                  | `queueScheduler`    |
| Microtâche | Promise.then, queueMicrotask | `asapScheduler`     |
| Macrotâche | setTimeout, setInterval      | `asyncScheduler`    |
