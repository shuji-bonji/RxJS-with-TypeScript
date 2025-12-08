---
description: "switchAll est un opérateur qui reçoit un Higher-order Observable (Observable d'Observables) et bascule vers le nouvel Observable interne tout en annulant l'ancien."
---

# switchAll - Basculer vers le nouvel Observable interne

L'opérateur `switchAll` reçoit un **Higher-order Observable** (Observable d'Observables)
et **bascule immédiatement vers le nouvel Observable interne** à chaque émission, annulant l'ancien.

## Syntaxe de base et utilisation

```ts
import { fromEvent, interval } from 'rxjs';
import { map, switchAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Démarre un nouveau compteur à chaque clic (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Bascule vers le nouveau compteur (annule l'ancien)
higherOrder$
  .pipe(switchAll())
  .subscribe(x => console.log(x));

// Sortie (avec 3 clics):
// 0 (1er compteur)
// 1 (1er compteur)
// ← clic ici (1er annulé)
// 0 (2ème compteur) ← bascule vers le nouveau compteur
// ← clic ici (2ème annulé)
// 0 (3ème compteur) ← bascule vers le nouveau compteur
// 1 (3ème compteur)
// 2 (3ème compteur)
```

- **Bascule immédiatement** quand un nouvel Observable interne est émis par le Higher-order Observable
- L'Observable interne précédent est **automatiquement annulé**
- Seul le dernier Observable interne est toujours en cours d'exécution

[Documentation officielle RxJS - `switchAll`](https://rxjs.dev/api/index/function/switchAll)

## Patterns d'utilisation typiques

- **Fonctionnalité de recherche (annuler l'ancienne recherche à chaque nouvelle saisie)**
- **Autocomplétion**
- **Mises à jour de données en temps réel (basculer vers la dernière source de données)**

## Exemple de code pratique

Un exemple qui annule l'ancienne recherche et exécute uniquement la dernière à chaque nouvelle saisie

```ts
import { fromEvent, of } from 'rxjs';
import { map, switchAll, debounceTime, delay } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Entrez un mot-clé de recherche';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

let searchCount = 0;

// Debounce des événements de saisie
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: appel API de recherche simulé pour chaque valeur de saisie
const results$ = search$.pipe(
  map(query => {
    const id = ++searchCount;
    const start = Date.now();

    // Appel API de recherche simulé (délai de 1 seconde)
    return of(`Résultats de recherche: "${query}"`).pipe(
      delay(1000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `[Recherche#${id}] ${msg} (${elapsed}s)`;
      })
    );
  }),
  switchAll() // Annule l'ancienne recherche quand une nouvelle commence
);

results$.subscribe(result => {
  output.innerHTML = ''; // Efface le résultat précédent
  const item = document.createElement('div');
  item.textContent = result;
  output.appendChild(item);
});
```

- Quand l'utilisateur modifie la saisie, **l'ancienne recherche est automatiquement annulée**
- Seul le dernier résultat de recherche est affiché

## Opérateurs associés

| Opérateur | Description |
|---|---|
| `switchMap` | Raccourci pour `map` + `switchAll` (le plus couramment utilisé) |
| [mergeAll](./mergeAll) | Souscrit à tous les Observables internes en parallèle |
| [concatAll](./concatAll) | Souscrit aux Observables internes séquentiellement (attend la complétion) |
| [exhaustAll](./exhaustAll) | Ignore les nouveaux Observables internes pendant l'exécution |

## Points d'attention

### Prévention des fuites mémoire

`switchAll` **annule automatiquement** les anciens Observables internes, ce qui aide à prévenir les fuites mémoire.
C'est idéal pour les cas où de nouvelles requêtes sont fréquentes, comme la recherche ou l'autocomplétion.

### Observables internes qui ne se terminent pas

Même si les Observables internes ne se terminent pas, ils basculent automatiquement quand un nouveau est émis.

```ts
// interval ne se termine pas, mais est automatiquement annulé au prochain clic
clicks$.pipe(
  map(() => interval(1000)), // Ne se termine pas
  switchAll()
).subscribe();
```

### Optimal quand seule la dernière valeur compte

Utilisez `switchAll` quand le résultat de l'ancien traitement n'est pas nécessaire et que **seul le dernier résultat compte**.
Si tous les résultats sont nécessaires, utilisez [mergeAll](./mergeAll).

