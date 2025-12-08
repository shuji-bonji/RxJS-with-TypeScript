---
description: "combineLatestAll est un opérateur qui reçoit un Higher-order Observable (Observable d'Observables) et, une fois que tous les Observables internes ont émis au moins une valeur, combine leurs dernières valeurs en un tableau."
---

# combineLatestAll - Combiner les dernières valeurs de tous les Observables internes

L'opérateur `combineLatestAll` reçoit un **Higher-order Observable** (Observable d'Observables)
et **une fois que tous les Observables internes ont émis au moins une valeur**, combine leurs **dernières valeurs** en un tableau.

## Syntaxe de base et utilisation

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// Higher-order Observable avec 3 Observables internes
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Combine les dernières valeurs une fois que tous les Observables internes ont émis au moins une fois
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Sortie:
// [1, 3, 0] ← au moment où tous ont émis au moins une fois (2 secondes)
// [2, 3, 0] ← le 1er Observable émet 2 (3 secondes)
// [2, 3, 1] ← le 3ème Observable émet 1 (4 secondes)
```

- Collecte les Observables internes au moment où le Higher-order Observable **se termine**
- **Une fois que tous les Observables internes ont émis au moins une fois**, commence à combiner
- Chaque fois qu'un Observable interne émet une valeur, **combine toutes les dernières valeurs** pour émission

[Documentation officielle RxJS - `combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## Patterns d'utilisation typiques

- **Combiner les derniers résultats de plusieurs appels API**
- **Synchroniser les dernières valeurs de plusieurs champs de formulaire**
- **Intégrer plusieurs sources de données en temps réel**

## Exemple de code pratique

Un exemple qui combine et affiche les derniers résultats de plusieurs appels API

```ts
import { of, timer, Observable } from 'rxjs';
import { map, combineLatestAll, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// Création de 3 appels API simulés (Higher-order Observable)
const apiCalls$: Observable<Observable<string>> = of(
  // API 1: Info utilisateur (complète après 1 seconde, 3 mises à jour)
  timer(0, 1000).pipe(
    take(3),
    map(n => `Utilisateur: User${n}`)
  ),
  // API 2: Nombre de notifications (complète après 0.5 seconde, 4 mises à jour)
  timer(0, 500).pipe(
    take(4),
    map(n => `Notifications: ${n}`)
  ),
  // API 3: Statut (complète après 2 secondes, 2 mises à jour)
  timer(0, 2000).pipe(
    take(2),
    map(n => n === 0 ? 'Statut: Hors ligne' : 'Statut: En ligne')
  )
);

// Combine les dernières valeurs de tous les appels API pour affichage
apiCalls$
  .pipe(combineLatestAll())
  .subscribe(values => {
    output.innerHTML = '<strong>État actuel:</strong><br>';
    values.forEach((value, index) => {
      const item = document.createElement('div');
      item.textContent = `${index + 1}. ${value}`;
      output.appendChild(item);
    });
  });
```

- 3 appels API sont exécutés en parallèle
- **Une fois que tous ont émis au moins une fois**, les résultats combinés sont affichés
- Chaque fois qu'un API se met à jour, **la dernière combinaison** est affichée

## Creation Function associée

`combineLatestAll` est principalement utilisé pour aplatir les Higher-order Observables.
Pour combiner plusieurs Observables normaux, utilisez la **Creation Function** `combineLatest`.

```ts
import { combineLatest, interval } from 'rxjs';

// Version Creation Function (utilisation plus courante)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

Voir [Chapitre 3 Creation Functions - combineLatest](/fr/guide/creation-functions/combination/combineLatest).

## Opérateurs associés

| Opérateur | Description |
|---|---|
| [mergeAll](./mergeAll) | Souscrit à tous les Observables internes en parallèle |
| [concatAll](./concatAll) | Souscrit aux Observables internes séquentiellement |
| [switchAll](./switchAll) | Bascule vers le nouvel Observable interne |
| [zipAll](./zipAll) | Apparie les valeurs correspondantes de chaque Observable interne |

## Points d'attention

### La complétion du Higher-order Observable est requise

`combineLatestAll` attend que le Higher-order Observable (l'Observable externe) **se termine** avant de collecter les Observables internes.

#### Le Higher-order Observable ne se termine pas, donc rien n'est émis
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // Rien n'est émis
```

#### Utiliser take pour terminer
```ts
interval(1000).pipe(
  take(3), // Se termine après 3
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### Tous les Observables internes doivent émettre au moins une fois

Aucune valeur n'est émise tant que **tous les Observables internes n'ont pas émis au moins une valeur**.

```ts
// Si même un Observable interne n'émet jamais, rien n'est émis
of(
  of(1, 2, 3),
  NEVER // N'émet jamais
).pipe(
  combineLatestAll()
).subscribe(console.log); // Rien n'est émis
```

### Utilisation mémoire

Comme il **conserve les dernières valeurs de tous les Observables internes en mémoire**, faites attention à l'utilisation mémoire s'il y a de nombreux Observables internes.

