---
description: "L'opérateur windowWhen divise l'Observable en contrôlant dynamiquement les conditions de fin. Comme la fenêtre suivante démarre immédiatement après la fin de la précédente, il est idéal pour le découpage de données continu. Explique les différences avec bufferWhen, l'implémentation TypeScript type-safe et des exemples pratiques."
---

# windowWhen - Fenêtre avec contrôle de fin dynamique

L'opérateur `windowWhen` **contrôle dynamiquement les conditions de fin** pour diviser l'Observable. Il réalise un pattern de traitement de flux continu où lorsqu'une fenêtre se termine, la fenêtre suivante démarre immédiatement.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500); // Émet toutes les 0.5 secondes

// Condition de fin : après 1 seconde
const closingSelector = () => interval(1000);

source$.pipe(
  windowWhen(closingSelector),
  take(4),
  mergeAll()
).subscribe(value => {
  console.log('Valeur dans la fenêtre :', value);
});

// Fenêtre 1 : 0       (démarrage 0s → fin 1s)
// Fenêtre 2 : 1, 2    (démarrage 1s → fin 2s)
// Fenêtre 3 : 3, 4    (démarrage 2s → fin 3s)
// Fenêtre 4 : 5, 6    (démarrage 3s → fin 4s)
```

**Flux d'opération** :
1. La première fenêtre démarre automatiquement
2. L'Observable retourné par `closingSelector()` émet une valeur → fin de la fenêtre
3. **Immédiatement, la fenêtre suivante démarre**
4. Répétition de 2-3

[🌐 Documentation officielle RxJS - `windowWhen`](https://rxjs.dev/api/operators/windowWhen)

## 💡 Patterns d'utilisation typiques

- Collecte de données à intervalles dynamiques
- Traitement de flux adaptatif selon la charge
- Contrôle de fenêtre basé sur les résultats précédents
- Regroupement de données continu

## 🔍 Différences avec bufferWhen

| Opérateur | Sortie | Cas d'utilisation |
|:---|:---|:---|
| `bufferWhen` | **Tableau (T[])** | Traitement groupé des valeurs |
| `windowWhen` | **Observable\<T>** | Traitement de flux différent par groupe |

## 🧠 Exemple de code pratique : collecte de données adaptative

Exemple d'ajustement de la durée de la fenêtre suivante basé sur les résultats de la fenêtre précédente.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, map } from 'rxjs';

interface WindowStats {
  count: number;
  nextDuration: number;
}

const data$ = interval(100);

let previousCount = 0;

// Ajuste la durée de la fenêtre suivante selon la quantité de données
function getNextDuration(count: number): number {
  if (count > 20) {
    return 500;  // Beaucoup de données → intervalle court
  } else if (count > 10) {
    return 1000; // Modéré → intervalle moyen
  } else {
    return 2000; // Peu de données → intervalle long
  }
}

data$.pipe(
  windowWhen(() => timer(getNextDuration(previousCount))),
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(data => {
        previousCount = data.length;
        return {
          count: data.length,
          nextDuration: getNextDuration(data.length)
        } as WindowStats;
      })
    )
  )
).subscribe(stats => {
  console.log(`Taille fenêtre : ${stats.count} éléments, prochaine durée : ${stats.nextDuration}ms`);
});
```

## 🆚 Différences avec windowToggle

| Opérateur | Contrôle | Période de fenêtre | Cas d'utilisation |
|:---|:---|:---|:---|
| `windowWhen(closing)` | Fin uniquement | Continue | Fenêtre périodique simple |
| `windowToggle(open$, close)` | Début et fin séparés | Chevauchement possible | Conditions début/fin complexes |

**Points de différenciation** :
- **`windowWhen`** : pour traiter toutes les données en continu sans omissions (collecte de logs, agrégation de données, etc.)
- **`windowToggle`** : pour traiter les données pendant une période spécifique (heures de bureau, pendant l'appui sur un bouton, etc.)

## ⚠️ Points d'attention

### 1. Doit retourner un nouvel Observable à chaque fois

La fonction `closingSelector` doit **retourner un nouvel Observable à chaque fois**. Retourner la même instance ne fonctionnera pas correctement.

```ts
// ❌ Mauvais exemple : réutilisation de la même instance Observable
const closingObservable = timer(1000);

source$.pipe(
  windowWhen(() => closingObservable) // Ne fonctionne pas après le premier !
).subscribe();

// ✅ Bon exemple : génère un nouvel Observable à chaque fois
source$.pipe(
  windowWhen(() => timer(1000)) // Génère un nouveau timer à chaque fois
).subscribe();
```

### 2. Gestion des abonnements aux fenêtres

Chaque fenêtre est un Observable indépendant et doit être explicitement souscrite ou aplatie avec `mergeAll()`.

## 📚 Opérateurs associés

- [`bufferWhen`](./bufferWhen) - Regroupe les valeurs en tableau (version tableau de windowWhen)
- [`window`](./window) - Division de fenêtre par timing d'un autre Observable
- [`windowTime`](./windowTime) - Division de fenêtre basée sur le temps
- [`windowCount`](./windowCount) - Division de fenêtre basée sur le nombre
- [`windowToggle`](./windowToggle) - Contrôle de fenêtre avec Observables début/fin

## Résumé

L'opérateur `windowWhen` est un outil pratique qui contrôle dynamiquement les conditions de fin et réalise un traitement de fenêtre continu.

- ✅ Peut contrôler dynamiquement les conditions de fin
- ✅ Traitement de fenêtre continu (ne perd pas de données)
- ✅ Peut ajuster la fenêtre suivante basé sur les résultats précédents
- ⚠️ Nécessite une gestion des abonnements
- ⚠️ Doit retourner un nouvel Observable à chaque fois
- ⚠️ Attention à ne pas rendre les conditions de fin trop complexes
