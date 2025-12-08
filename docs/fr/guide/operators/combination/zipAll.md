---
description: "zipAll est un opérateur qui reçoit un Higher-order Observable (Observable d'Observables) et apparie les valeurs correspondantes de chaque Observable interne en un tableau."
---

# zipAll - Appairer les valeurs correspondantes de chaque Observable interne

L'opérateur `zipAll` reçoit un **Higher-order Observable** (Observable d'Observables)
et **apparie les valeurs correspondantes de chaque Observable interne** en un tableau.

## Syntaxe de base et utilisation

```ts
import { interval, of } from 'rxjs';
import { zipAll, take } from 'rxjs';

// Higher-order Observable avec 3 Observables internes
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Apparie les valeurs à l'index correspondant de chaque Observable interne
higherOrder$
  .pipe(zipAll())
  .subscribe(values => console.log(values));

// Sortie:
// [0, 0, 0] ← toutes les 1ères valeurs
// [1, 1, 1] ← toutes les 2èmes valeurs
// (complété ici: le 3ème Observable n'a que 2 valeurs)
```

- Collecte les Observables internes au moment où le Higher-order Observable **se termine**
- **Apparie les valeurs au même index** de chaque Observable interne
- **Quand l'Observable interne le plus court se termine**, tout se termine

[Documentation officielle RxJS - `zipAll`](https://rxjs.dev/api/index/function/zipAll)

## Patterns d'utilisation typiques

- **Apparier les réponses de plusieurs API par ordre**
- **Comparer les valeurs au même timing de plusieurs flux**
- **Combiner les résultats de traitements parallèles par ordre**

## Exemple de code pratique

Un exemple qui apparie les valeurs correspondantes de plusieurs compteurs

```ts
import { interval, of } from 'rxjs';
import { zipAll, take, map } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// Création de 3 compteurs à des vitesses différentes
const counters$ = of(
  interval(1000).pipe(take(4), map(n => `Lent: ${n}`)),
  interval(500).pipe(take(5), map(n => `Normal: ${n}`)),
  interval(300).pipe(take(6), map(n => `Rapide: ${n}`))
);

// Apparie les valeurs à l'index correspondant de chaque compteur
counters$
  .pipe(zipAll())
  .subscribe(values => {
    const item = document.createElement('div');
    item.textContent = `[${values.join(', ')}]`;
    output.appendChild(item);
  });

// Sortie:
// [Lent: 0, Normal: 0, Rapide: 0]
// [Lent: 1, Normal: 1, Rapide: 1]
// [Lent: 2, Normal: 2, Rapide: 2]
// [Lent: 3, Normal: 3, Rapide: 3]
// (complété ici: le compteur "Lent" n'a que 4 valeurs)
```

## Creation Function associée

`zipAll` est principalement utilisé pour aplatir les Higher-order Observables.
Pour appairer plusieurs Observables normaux, utilisez la **Creation Function** `zip`.

```ts
import { zip, interval } from 'rxjs';
import { take } from 'rxjs';

// Version Creation Function (utilisation plus courante)
const zipped$ = zip(
  interval(1000).pipe(take(3)),
  interval(500).pipe(take(4)),
  interval(2000).pipe(take(2))
);

zipped$.subscribe(console.log);
```

Voir [Chapitre 3 Creation Functions - zip](/fr/guide/creation-functions/combination/zip).

## Opérateurs associés

| Opérateur | Description |
|---|---|
| [combineLatestAll](./combineLatestAll) | Combine les dernières valeurs de tous les Observables internes |
| [mergeAll](./mergeAll) | Souscrit à tous les Observables internes en parallèle |
| [concatAll](./concatAll) | Souscrit aux Observables internes séquentiellement |
| [switchAll](./switchAll) | Bascule vers le nouvel Observable interne |

## zipAll vs combineLatestAll

| Opérateur | Méthode de combinaison | Timing de complétion |
|---|---|---|
| `zipAll` | Apparie les valeurs au **même index** | Quand l'Observable interne **le plus court** se termine |
| `combineLatestAll` | Combine les **dernières valeurs** | Quand **tous** les Observables internes se terminent |

```ts
// zipAll: [0ème, 0ème, 0ème], [1er, 1er, 1er], ...
// combineLatestAll: [dernier, dernier, dernier], [dernier, dernier, dernier], ...
```

## Points d'attention

### La complétion du Higher-order Observable est requise

`zipAll` attend que le Higher-order Observable (l'Observable externe) **se termine** avant de collecter les Observables internes.

#### Le Higher-order Observable ne se termine pas, donc rien n'est émis
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log); // Rien n'est émis
```

#### Utiliser take pour terminer
```ts
interval(1000).pipe(
  take(3), // Se termine après 3
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log);
```

### Se termine avec l'Observable interne le plus court

**Quand l'Observable interne le plus court se termine**, tout se termine.

```ts
import { of, zipAll } from "rxjs";

of(
  of(1, 2, 3, 4, 5), // 5 valeurs
  of(1, 2)           // 2 valeurs ← le plus court
).pipe(
  zipAll()
).subscribe(console.log);

// Sortie: [1, 1], [2, 2]
// (Complété après 2. 3, 4, 5 ne sont pas utilisés)
```

### Contre-pression (utilisation mémoire)

Si les Observables internes émettent à des vitesses différentes, **les valeurs de l'Observable rapide s'accumulent en mémoire**.

```ts
import { interval, of, take, zipAll } from "rxjs";

// Les valeurs du compteur rapide (100ms) s'accumulent en attendant le compteur lent (10000ms)
of(
  interval(10000).pipe(take(3)), // Lent
  interval(100).pipe(take(100))  // Rapide
).pipe(
  zipAll()
).subscribe(console.log);
```

Si la différence de vitesse est grande, faites attention à l'utilisation mémoire.

