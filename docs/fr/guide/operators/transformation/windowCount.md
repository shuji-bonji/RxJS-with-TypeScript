---
description: "windowCount est un opérateur de conversion RxJS qui divise un Observable par un nombre spécifié de morceaux. Contrairement à bufferCount, il peut être appliqué indépendamment à chaque fenêtre, ce qui permet un fractionnement de fenêtre et des opérations de flux à sécurité de type grâce à l'inférence de type TypeScript."
---

# windowCount - divisé par le nombre de pièces

L'opérateur `windowCount` **divise** la valeur émise en un nouvel Observable pour chaque nombre de morceaux spécifié.
Alors que `bufferCount` renvoie un tableau, `windowCount` renvoie un **Observable\<T>**, permettant à d'autres opérateurs d'être appliqués à chaque fenêtre.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { windowCount, mergeAll } from 'rxjs';

// 100msAttribuer une valeur à chaque fenêtre
const source$ = interval(100);

source$.pipe(
  windowCount(5),
  mergeAll() // Aplatir chaque fenêtre
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// Sortie:
// Valeur dans la fenêtre: 0
// Valeur dans la fenêtre: 1
// Valeur dans la fenêtre: 2
// Valeur dans la fenêtre: 3
// Valeur dans la fenêtre: 4
// (Démarrer une nouvelle fenêtre)
// Valeur dans la fenêtre: 5
// ...
```

- Une nouvelle fenêtre (Observable) est créée toutes les 5 valeurs.
- Elle est unique en ce sens qu'elle est divisée sur une base fragmentaire.

[🌐 Official RxJS documentation - `windowCount`](https://rxjs.dev/api/operators/windowCount)

## 💡 Modèle d'utilisation typique.

- Traitement agrégé pour chaque nombre fixe de cas.
- Transmission de données par lots (traitement différent pour chaque fenêtre)
- Traitement par pagination
- Calcul des statistiques par fenêtre

## 🔍 Différences par rapport à bufferCount

```ts
import { interval } from 'rxjs';
import { windowCount, mergeAll } from 'rxjs';

// 100msAttribuer une valeur à chaque fenêtre
const source$ = interval(100);

source$.pipe(
  windowCount(5),
  mergeAll() // Aplatir chaque fenêtre
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// Sortie:
// Valeur dans la fenêtre: 0
// Valeur dans la fenêtre: 1
// Valeur dans la fenêtre: 2
// Valeur dans la fenêtre: 3
// Valeur dans la fenêtre: 4
// (Démarrer une nouvelle fenêtre)
// Valeur dans la fenêtre: 5
// ...
```


```ts
import { interval } from 'rxjs';
import { bufferCount, windowCount, mergeAll } from 'rxjs';

const source$ = interval(100);

// bufferCount - Sortie sous forme de tableau
source$.pipe(
  bufferCount(5)
).subscribe(values => {
  console.log('Tampon (tableau):', values);
  // Sortie: Tampon (tableau): [0, 1, 2, 3, 4]
});

// windowCount - Observable Sortie en tant que
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  console.log('Fenêtre (Observable):', window$);
  // Emboîté subscribe: window Modèles requis par la spécification de l'opérateur du système
  // (Voir aussiwindowCount A l'intérieur émis par Observable requis pour consommer)
  window$.subscribe(value => {
    console.log('  Valeur dans la fenêtre:', value);
  });
});
```

## 🧠 Exemple de code pratique 1 : Valeur totale par fenêtre

Voici un exemple de calcul de la somme de toutes les cinq valeurs.

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, reduce } from 'rxjs';

// Création d'une zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>5Valeur totale pour chaque individu</h3>';
document.body.appendChild(output);

const source$ = interval(200);

let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;
    console.log(`Fenêtre ${current} Démarrage`);

    // Calculer la somme pour chaque fenêtre
    return window$.pipe(
      reduce((sum, value) => sum + value, 0),
      map(sum => ({ windowNum: current, sum }))  // Inclure le numéro de la fenêtre
    );
  }),
  mergeAll()
).subscribe(result => {
  const div = document.createElement('div');
  div.textContent = `Fenêtre ${result.windowNum} Somme de: ${result.sum}`;
  output.appendChild(div);
});

// Sortie:
// Fenêtre 1 Somme de: 10  (0+1+2+3+4)
// Fenêtre 2 Somme de: 35  (5+6+7+8+9)
// Fenêtre 3 Somme de: 60  (10+11+12+13+14)
```

## 🎯 Exemple de code pratique 2 : Spécifier l'indice de départ

Vous pouvez spécifier un indice de départ avec le deuxième argument. Une fenêtre de chevauchement peut être créée.

```ts
import { range } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// 0à9Émettre les valeurs de
range(0, 10).pipe(
  windowCount(3, 2), // 3pièce par pièce,2Décalé de 1 pièce au début
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Fenêtre:', values);
});

// Sortie:
// Fenêtre: [0, 1, 2]
// Fenêtre: [2, 3, 4]    ← 2Décalé d'une pièce pour commencer (à partir de2La dernière valeur est émise à partir de)
// Fenêtre: [4, 5, 6]    ← 2Décalé d'une pièce pour commencer (à partir de4La dernière valeur est émise à partir de)
// Fenêtre: [6, 7, 8]
// Fenêtre: [8, 9]       ← La dernière valeur est2Continu (par défaut)
```

### Modèle d'opération de l'index de départ.

```ts
// windowCount(bufferSize, startBufferEvery)

windowCount(3, 3) // Continue (par défaut): [0,1,2], [3,4,5], [6,7,8]
windowCount(3, 2) // Chevauchement: [0,1,2], [2,3,4], [4,5,6]
windowCount(3, 4) // Décalé: [0,1,2], [4,5,6], [8,9,10]
```

## 🎯 Exemple pratique : différents processus pour différentes fenêtres

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, take } from 'rxjs';

const source$ = interval(100);
let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Fenêtre paire: Première2Seul le premier est récupéré
      console.log(`Fenêtre ${current}: Première2Ne récupérer que le premier`);
      return window$.pipe(take(2));
    } else {
      // Fenêtre impaire: Tous
      console.log(`Fenêtre ${current}: Tous`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Valeur: ${value} (Fenêtre ${windowNumber})`);
});
```

## 🧠 Exemple de code pratique 3 : traitement de type pagination

```ts
import { from } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// 1-20Données jusqu'à
const data$ = from(Array.from({ length: 20 }, (_, i) => i + 1));

// 5Paginé par cas
data$.pipe(
  windowCount(5),
  mergeMap((window$, index) => {
    const pageNumber = index + 1;
    return window$.pipe(
      toArray(),
      map(items => ({ page: pageNumber, items }))
    );
  })
).subscribe(page => {
  console.log(`Page ${page.page}:`, page.items);
});

// Sortie:
// Page 1: [1, 2, 3, 4, 5]
// Page 2: [6, 7, 8, 9, 10]
// Page 3: [11, 12, 13, 14, 15]
// Page 4: [16, 17, 18, 19, 20]
```

## ⚠️ Notes.

### 1. gestion des abonnements aux fenêtres

Chaque fenêtre est un Observable indépendant et doit faire l'objet d'un abonnement explicite.

```ts
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  // Emboîté subscribe: window Modèles requis par la spécification de l'opérateur du système
  // Aucun flux de valeurs à moins que la fenêtre elle-même ne soit souscrite
  window$.subscribe(value => {
    console.log('Valeur:', value);
  });
});
```

ou aplatie en utilisant `mergeAll()`, `concatAll()`, `switchAll()`, etc.

### 2. dernière fenêtre.

A la fin de l'Observable source, la dernière fenêtre est éditée même s'il y a moins que le nombre spécifié de fenêtres.

```ts
import { of } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

of(1, 2, 3, 4, 5, 6, 7).pipe(
  windowCount(3),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Fenêtre:', values);
});

// Sortie:
// Fenêtre: [1, 2, 3]
// Fenêtre: [4, 5, 6]
// Fenêtre: [7]  ← 1Une seule
```

### Utilisation de la mémoire en fonction de l'index de départ

Si `startBufferEvery` est inférieur à `bufferSize` (chevauchement), plusieurs fenêtres sont actives en même temps, ce qui augmente l'utilisation de la mémoire.

```ts
import { interval } from 'rxjs';
import { windowCount, mergeAll } from 'rxjs';

// 100msAttribuer une valeur à chaque fenêtre
const source$ = interval(100);

source$.pipe(
  windowCount(5),
  mergeAll() // Aplatir chaque fenêtre
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// Sortie:
// Valeur dans la fenêtre: 0
// Valeur dans la fenêtre: 1
// Valeur dans la fenêtre: 2
// Valeur dans la fenêtre: 3
// Valeur dans la fenêtre: 4
// (Démarrer une nouvelle fenêtre)
// Valeur dans la fenêtre: 5
// ...
```

## 🆚 Comparaison des opérateurs basés sur les fenêtres


## 📚 Opérateurs apparentés.

- [`bufferCount`](. /bufferCount) - résume les valeurs sous forme de tableau (version tableau de windowCount).
- [`window`](. /window) - fractionnement de la fenêtre à différents moments de l'Observable.
- [`windowTime`](. /windowTime) - fractionnement de la fenêtre en fonction du temps.
- [`windowToggle`](. /windowToggle) - contrôle de la fenêtre avec un Observable de début et de fin.
- [`windowWhen`](. /windowWhen) - fractionnement de fenêtre avec conditions de fermeture dynamiques.

## Résumé.

L'opérateur `windowCount` est un outil utile pour partitionner des flux sur une base de comptage de pièces et traiter chaque groupe comme un Observable indépendant.

- ✅ Idéal pour l'agrégation et le traitement par nombre fixe de morceaux.
- ✅ Différents traitements peuvent être appliqués à chaque fenêtre.
- ✅ Peut être chevauchée par l'index de départ.
- ⚠️ Nécessite une gestion des abonnements
- ⚠️ Attention à l'utilisation de la mémoire en cas de chevauchement
