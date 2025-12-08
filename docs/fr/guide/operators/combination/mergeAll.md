---
description: "mergeAll est un opérateur qui reçoit un Higher-order Observable (Observable d'Observables) et souscrit à tous les Observables internes en parallèle pour aplatir les valeurs."
---

# mergeAll - Aplatir tous les Observables internes en parallèle

L'opérateur `mergeAll` reçoit un **Higher-order Observable** (Observable d'Observables)
et **souscrit à tous les Observables internes en parallèle** pour aplatir les valeurs.

## Syntaxe de base et utilisation

```ts
import { fromEvent, interval } from 'rxjs';
import { map, mergeAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Démarre un nouveau compteur à chaque clic (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Souscrit à tous les compteurs en parallèle
higherOrder$
  .pipe(mergeAll())
  .subscribe(x => console.log(x));

// Sortie (avec 3 clics):
// 0 (1er compteur)
// 1 (1er compteur)
// 0 (2ème compteur) ← exécution parallèle
// 2 (1er compteur)
// 1 (2ème compteur)
// 0 (3ème compteur) ← exécution parallèle
// ...
```

- **Souscrit en parallèle** à chaque Observable interne émis par le Higher-order Observable
- **Combine toutes les valeurs** des Observables internes **en un seul flux**
- Possibilité de limiter le nombre de souscriptions parallèles (`mergeAll(2)` = maximum 2 en parallèle)

[Documentation officielle RxJS - `mergeAll`](https://rxjs.dev/api/index/function/mergeAll)

## Patterns d'utilisation typiques

- **Exécuter plusieurs appels API en parallèle**
- **Démarrer des flux indépendants pour chaque action utilisateur**
- **Intégrer plusieurs connexions temps réel comme WebSocket ou EventSource**

## Exemple de code pratique

Un exemple qui exécute des appels API (simulés) en parallèle à chaque changement de saisie

```ts
import { fromEvent, of } from 'rxjs';
import { map, mergeAll, delay, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Entrez un mot-clé de recherche';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

// Debounce des événements de saisie
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: appel API simulé pour chaque valeur de saisie
const results$ = search$.pipe(
  map(query =>
    // Appel API simulé (délai de 500ms)
    of(`Résultat: "${query}"`).pipe(delay(500))
  ),
  mergeAll() // Exécute tous les appels API en parallèle
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- Même si l'utilisateur modifie rapidement la saisie, **tous les appels API sont exécutés en parallèle**
- Les anciens résultats de recherche peuvent apparaître après les nouveaux (pas de garantie d'ordre)

## Opérateurs associés

| Opérateur | Description |
|---|---|
| `mergeMap` | Raccourci pour `map` + `mergeAll` (couramment utilisé) |
| [concatAll](./concatAll) | Souscrit aux Observables internes séquentiellement (attend la complétion) |
| [switchAll](./switchAll) | Bascule vers le nouvel Observable interne (annule l'ancien) |
| [exhaustAll](./exhaustAll) | Ignore les nouveaux Observables internes pendant l'exécution |

## Points d'attention

### Limitation du nombre de souscriptions parallèles

Sans limitation du nombre de souscriptions parallèles, des problèmes de performance peuvent survenir.

```ts
// Limite les souscriptions parallèles à 2 maximum
higherOrder$.pipe(
  mergeAll(2) // Maximum 2 exécutions en parallèle
).subscribe();
```

### Pas de garantie d'ordre

`mergeAll` exécute en parallèle, donc **l'ordre des valeurs n'est pas garanti**.
Si l'ordre est important, utilisez [concatAll](./concatAll).

