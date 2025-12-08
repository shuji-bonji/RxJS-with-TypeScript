---
description: "concatAll est un opérateur qui reçoit un Higher-order Observable (Observable d'Observables) et souscrit aux Observables internes séquentiellement pour aplatir les valeurs. Il attend la complétion de l'Observable précédent avant de démarrer le suivant."
---

# concatAll - Aplatir les Observables internes séquentiellement

L'opérateur `concatAll` reçoit un **Higher-order Observable** (Observable d'Observables)
et **souscrit aux Observables internes séquentiellement** pour aplatir les valeurs. Il ne démarre pas le suivant tant que l'Observable précédent n'est pas terminé.

## Syntaxe de base et utilisation

```ts
import { fromEvent, interval } from 'rxjs';
import { map, concatAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Démarre un nouveau compteur à chaque clic (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Souscrit aux compteurs séquentiellement (démarre le suivant après complétion du précédent)
higherOrder$
  .pipe(concatAll())
  .subscribe(x => console.log(x));

// Sortie (avec 3 clics):
// 0 (1er compteur)
// 1 (1er compteur)
// 2 (1er compteur) ← complété
// 0 (2ème compteur) ← démarre après complétion du 1er
// 1 (2ème compteur)
// 2 (2ème compteur) ← complété
// 0 (3ème compteur) ← démarre après complétion du 2ème
// ...
```

- **Souscrit séquentiellement** à chaque Observable interne émis par le Higher-order Observable
- **Ne démarre pas le suivant tant que l'Observable interne précédent n'est pas terminé**
- L'ordre des valeurs est garanti

[Documentation officielle RxJS - `concatAll`](https://rxjs.dev/api/index/function/concatAll)

## Patterns d'utilisation typiques

- **Exécuter des appels API séquentiellement (exécuter le suivant après complétion du précédent)**
- **Jouer des animations séquentiellement**
- **Traiter des téléchargements de fichiers séquentiellement**

## Exemple de code pratique

Un exemple qui exécute des appels API (simulés) séquentiellement pour chaque clic de bouton

```ts
import { fromEvent, of } from 'rxjs';
import { map, concatAll, delay } from 'rxjs';

const button = document.createElement('button');
button.textContent = 'Appel API';
document.body.appendChild(button);

const output = document.createElement('div');
document.body.appendChild(output);

let callCount = 0;

// Événement de clic du bouton
const clicks$ = fromEvent(button, 'click');

// Higher-order Observable: appel API simulé pour chaque clic
const results$ = clicks$.pipe(
  map(() => {
    const id = ++callCount;
    const start = Date.now();

    // Appel API simulé (délai de 2 secondes)
    return of(`Appel API #${id} terminé`).pipe(
      delay(2000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed}s)`;
      })
    );
  }),
  concatAll() // Exécute tous les appels API séquentiellement
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- Même avec des clics consécutifs sur le bouton, **les appels API sont exécutés séquentiellement**
- Le suivant démarre après la complétion du précédent

## Opérateurs associés

| Opérateur | Description |
|---|---|
| `concatMap` | Raccourci pour `map` + `concatAll` (couramment utilisé) |
| [mergeAll](./mergeAll) | Souscrit à tous les Observables internes en parallèle |
| [switchAll](./switchAll) | Bascule vers le nouvel Observable interne (annule l'ancien) |
| [exhaustAll](./exhaustAll) | Ignore les nouveaux Observables internes pendant l'exécution |

## Points d'attention

### Contre-pression (accumulation)

Si les Observables internes sont émis plus vite qu'ils ne se terminent, **les Observables non traités s'accumulent dans une file d'attente**.

```ts
// Clic toutes les 1 seconde → appel API de 2 secondes
// → possibilité d'accumulation dans la file
```

Dans ce cas, considérez les mesures suivantes:
- Utiliser `switchAll` (traiter uniquement le dernier)
- Utiliser `exhaustAll` (ignorer pendant l'exécution)
- Ajouter du debounce ou du throttling

### Attention aux Observables infinis

Si l'Observable précédent **ne se termine pas, le suivant ne démarrera jamais**.

#### interval ne se termine pas, donc le 2ème compteur ne démarre jamais
```ts
clicks$.pipe(
  map(() => interval(1000)), // Ne se termine pas
  concatAll()
).subscribe();
```
#### Utiliser take pour terminer
```ts
clicks$.pipe(
  map(() => interval(1000).pipe(take(3))), // Se termine après 3
  concatAll()
).subscribe();
```

