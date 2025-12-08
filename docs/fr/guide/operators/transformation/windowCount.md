---
description: "windowCount est un opérateur de transformation RxJS qui divise l'Observable par nombre spécifié. Idéal pour le traitement de flux basé sur le nombre, l'agrégation par nombre fixe d'éléments, et le traitement de pagination. Contrairement à bufferCount, il permet d'appliquer un traitement indépendant à chaque fenêtre. L'inférence de type TypeScript permet une division de fenêtre et des opérations de flux type-safe."
---

# windowCount - Diviser l'Observable par nombre spécifié

L'opérateur `windowCount` **divise** les valeurs émises en un nouvel Observable pour chaque nombre spécifié.
Alors que `bufferCount` renvoie un tableau, `windowCount` renvoie **Observable\<T>**, permettant d'appliquer d'autres opérateurs à chaque fenêtre.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { windowCount, mergeAll } from 'rxjs';

// Émet des valeurs toutes les 100ms
const source$ = interval(100);

source$.pipe(
  windowCount(5),
  mergeAll() // Aplatit chaque fenêtre
).subscribe(value => {
  console.log('Valeur dans la fenêtre :', value);
});

// Sortie :
// Valeur dans la fenêtre : 0
// Valeur dans la fenêtre : 1
// Valeur dans la fenêtre : 2
// Valeur dans la fenêtre : 3
// Valeur dans la fenêtre : 4
// (nouvelle fenêtre démarre)
// Valeur dans la fenêtre : 5
// ...
```

- Une nouvelle fenêtre (Observable) est créée toutes les 5 valeurs.
- Elle est caractéristique de diviser sur une base de nombre.

[🌐 Documentation officielle RxJS - `windowCount`](https://rxjs.dev/api/operators/windowCount)

## 💡 Patterns d'utilisation typiques

- Traitement d'agrégation pour chaque nombre fixe d'éléments
- Transmission de données par lots (traitement différent pour chaque fenêtre)
- Traitement de pagination
- Calcul de statistiques par fenêtre

## 🔍 Différences avec bufferCount

| Opérateur | Sortie | Cas d'utilisation |
|:---|:---|:---|
| `bufferCount` | **Tableau (T[])** | Traitement groupé des valeurs |
| `windowCount` | **Observable\<T>** | Traitement de flux différent par groupe |

## 🧠 Exemple de code pratique : total par fenêtre

Exemple de calcul de la somme de toutes les 5 valeurs.

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, reduce } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Total par groupe de 5</h3>';
document.body.appendChild(output);

const source$ = interval(200);

let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;
    console.log(`Fenêtre ${current} démarrée`);

    // Calcule le total de chaque fenêtre
    return window$.pipe(
      reduce((sum, value) => sum + value, 0),
      map(sum => ({ windowNum: current, sum }))
    );
  }),
  mergeAll()
).subscribe(result => {
  const div = document.createElement('div');
  div.textContent = `Fenêtre ${result.windowNum} total : ${result.sum}`;
  output.appendChild(div);
});

// Sortie :
// Fenêtre 1 total : 10  (0+1+2+3+4)
// Fenêtre 2 total : 35  (5+6+7+8+9)
// Fenêtre 3 total : 60  (10+11+12+13+14)
```

## 🎯 Spécification de l'indice de départ

Vous pouvez spécifier un indice de départ avec le 2ème argument. Des fenêtres qui se chevauchent peuvent être créées.

```ts
import { range } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// Émet les valeurs de 0 à 9
range(0, 10).pipe(
  windowCount(3, 2), // 3 par groupe, décalage de 2
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Fenêtre :', values);
});

// Sortie :
// Fenêtre : [0, 1, 2]
// Fenêtre : [2, 3, 4]    ← commence à 2 (décalage de 2)
// Fenêtre : [4, 5, 6]    ← commence à 4 (décalage de 2)
// Fenêtre : [6, 7, 8]
// Fenêtre : [8, 9]       ← seulement 2 à la fin
```

### Patterns de comportement de l'indice de départ

```ts
// windowCount(bufferSize, startBufferEvery)

windowCount(3, 3) // Continu (par défaut) : [0,1,2], [3,4,5], [6,7,8]
windowCount(3, 2) // Chevauchement : [0,1,2], [2,3,4], [4,5,6]
windowCount(3, 4) // Avec intervalle : [0,1,2], [4,5,6], [8,9,10]
```

## ⚠️ Points d'attention

### 1. Gestion des abonnements aux fenêtres

Chaque fenêtre est un Observable indépendant et doit être explicitement souscrite.

### 2. La dernière fenêtre

À la fin de l'Observable source, la dernière fenêtre est émise même si elle contient moins que le nombre spécifié.

### 3. Utilisation mémoire avec indice de départ

Si `startBufferEvery` est plus petit que `bufferSize` (chevauchement), plusieurs fenêtres seront actives simultanément, augmentant l'utilisation mémoire.

## 📚 Opérateurs associés

- [`bufferCount`](./bufferCount) - Regroupe les valeurs en tableau (version tableau de windowCount)
- [`window`](./window) - Division de fenêtre par timing d'un autre Observable
- [`windowTime`](./windowTime) - Division de fenêtre basée sur le temps
- [`windowToggle`](./windowToggle) - Contrôle de fenêtre avec Observables début/fin
- [`windowWhen`](./windowWhen) - Division de fenêtre avec condition de fermeture dynamique

## Résumé

L'opérateur `windowCount` est un outil pratique pour diviser les flux sur une base de nombre et traiter chaque groupe comme un Observable séparé.

- ✅ Idéal pour l'agrégation et le traitement par nombre fixe d'éléments
- ✅ Peut appliquer un traitement différent à chaque fenêtre
- ✅ Chevauchement possible avec l'indice de départ
- ⚠️ Nécessite une gestion des abonnements
- ⚠️ Attention à l'utilisation mémoire lors du chevauchement
