---
description: "reduce est un opérateur de conversion RxJS qui accumule toutes les valeurs d'un flux et ne produit que le résultat final à la fin. Il est idéal pour les situations où seul le résultat final est requis, comme le calcul de sommes numériques, de moyennes, de maximums et de minimums, l'agrégation d'objets, la construction de tableaux, etc. Contrairement au scan, il ne produit pas de résultats intermédiaires et ne peut pas être utilisé avec des flux infinis, car l'achèvement du flux est obligatoire."
---

# reduce - ne sortir que le résultat final cumulé

L'opérateur `reduce` applique une fonction cumulative à chaque valeur du flux et ne produit **que le résultat cumulatif final** à la fin du flux.
Il a le même comportement que `Array.prototype.reduce` pour les tableaux, sans sortie de résultats intermédiaires.

## 🔰 Syntaxe de base et utilisation

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Résultats.: 15(résultats finaux uniquement)
```

- `acc` est la valeur accumulée, `curr` est la valeur courante.
- Il commence avec une valeur initiale (dans ce cas `0`) et s'accumule séquentiellement.
- La valeur n'est pas affichée tant que le flux n'est pas terminé, et seul le résultat final **à la fin** est affiché.

[🌐 Documentation officielle de RxJS - `reduce`](https://rxjs.dev/api/operators/reduce)

## 💡 Modèle d'utilisation typique.

- Calcul de sommes, de moyennes, de maximums et de minimums de nombres.
- Agrégation et transformation d'objets
- Construction ou combinaison de tableaux
- Lorsque seul le résultat final de l'agrégation est requis

## 🔍 Différences avec le scan

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Résultats.: 15(résultats finaux uniquement)
```


```ts
import { of } from 'rxjs';
import { reduce, scan } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== reduce ===');
source$.pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Résultats.: 15

console.log('=== scan ===');
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Résultats.: 1, 3, 6, 10, 15
```

## 🧠 Exemples de code pratique (avec interface utilisateur)

Cet exemple additionne les valeurs de plusieurs champs de saisie et affiche le résultat final en cliquant sur un bouton.

```ts
import { fromEvent, from } from 'rxjs';
import { map, reduce, switchMap } from 'rxjs';

// Créer des champs de saisie
const inputs: HTMLInputElement[] = [];
for (let i = 1; i <= 3; i++) {
  const label = document.createElement('label');
  label.textContent = `Valeur${i}: `;
  const input = document.createElement('input');
  input.type = 'number';
  input.value = '0';
  label.appendChild(input);
  document.body.appendChild(label);
  document.body.appendChild(document.createElement('br'));
  inputs.push(input);
}

// Bouton de calcul
const button = document.createElement('button');
button.textContent = 'Calculer le total';
document.body.appendChild(button);

// Zone d'affichage des résultats
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Calculer le total en cliquant sur le bouton
fromEvent(button, 'click').pipe(
  switchMap(() => {
    // Obtenir toutes les valeurs d'entrée
    const values = inputs.map(input => Number(input.value) || 0);
    return from(values).pipe(
      reduce((acc, curr) => acc + curr, 0)
    );
  })
).subscribe(total => {
  output.textContent = `Total: ${total}`;
  console.log('Total:', total);
});
```

- Lorsque l'on clique sur le bouton, toutes les valeurs saisies sont additionnées et seul le total final est affiché.
- Les résultats intermédiaires ne sont pas affichés.

## 🎯 Exemple d'agrégation d'objets.

Il s'agit d'un exemple pratique de combinaison de plusieurs valeurs dans un objet.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface Product {
  category: string;
  price: number;
}

const products: Product[] = [
  { category: 'Produits alimentaires', price: 500 },
  { category: 'Boissons', price: 200 },
  { category: 'Produits alimentaires', price: 800 },
  { category: 'Boissons', price: 150 },
  { category: 'Produits alimentaires', price: 300 },
];

// Agréger les montants totaux par catégorie
from(products).pipe(
  reduce((acc, product) => {
    acc[product.category] = (acc[product.category] || 0) + product.price;
    return acc;
  }, {} as Record<string, number>)
).subscribe(result => {
  console.log('Total par catégorie:', result);
});

// Résultats.:
// Total par catégorie: { Produits alimentaires: 1600, Boissons: 350 }
```

## 🎯 Exemple de construction d'un tableau

Voici un exemple de combinaison des valeurs d'un flux dans un tableau.

```ts
import { interval } from 'rxjs';
import { take, reduce } from 'rxjs';

interval(100).pipe(
  take(5),
  reduce((acc, value) => {
    acc.push(value);
    return acc;
  }, [] as number[])
).subscribe(array => {
  console.log('Tableau collecté:', array);
});

// Résultats.:
// Tableau collecté: [0, 1, 2, 3, 4]
```

:: : conseil.
Lorsque vous construisez des tableaux, pensez à utiliser l'opérateur plus concis [`toArray`](. /utility/toArray).

```ts
interval(100).pipe(
  take(5),
  toArray()
).subscribe(console.log);
// Résultats.: [0, 1, 2, 3, 4]
```

:: :

## 💡 Utilisation de reduce à sécurité de type

Ceci est un exemple d'utilisation de l'inférence de type TypeScript.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface UserAction {
  type: 'click' | 'scroll' | 'input';
  timestamp: number;
}

const actions: UserAction[] = [
  { type: 'click', timestamp: 100 },
  { type: 'scroll', timestamp: 200 },
  { type: 'click', timestamp: 300 },
  { type: 'input', timestamp: 400 },
];

const actions$ = from(actions);

// Nombre total de fois par type d'action
actions$.pipe(
  reduce((acc, action) => {
    acc[action.type] = (acc[action.type] || 0) + 1;
    return acc;
  }, {} as Record<UserAction['type'], number>)
).subscribe(result => {
  console.log('Agrégation des actions:', result);
});

// Résultats.:
// Agrégation des actions: { click: 2, scroll: 1, input: 1 }
```

## ⚠️ Notes.

### ❌ Le flux infini n'est pas complet (important)

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Résultats.: 15(résultats finaux uniquement)
```

> ** `reduce` ne sortira pas une seule valeur jusqu'à ce que `complete()` soit appelé. ** Les flux infinis (interval`, fromEvent`, etc.) causent des accidents en pratique, car aucune valeur n'est disponible en permanence.


```ts
import { interval } from 'rxjs';
import { reduce } from 'rxjs';

// ❌ Mauvais exemple: Pas de sortie de valeur en raison d'un flux infini
interval(1000).pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Pas de sortie (parce que le flux n'est pas complet)
```

**Contre-mesure 1 : Utilisez `scan` si l'agrégation par roulement est nécessaire**.

```ts
import { interval, scan, take } from 'rxjs';

// ✅ Bon exemple: Résultats intermédiaires en temps réel
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Résultats.: 0, 1, 3, 6, 10(Les valeurs accumulées sont émises à chaque fois)
```

**Mesure 2 : `scan` + `takeLast(1)`** si seule la valeur finale est requise

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Résultats.: 15(résultats finaux uniquement)
```

**Contre-mesure 3 : Utilisez `take` pour rendre la condition de fin explicite**.


```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Résultats.: 15(résultats finaux uniquement)
```

> **Critères de sélection** :.
> - résultats intermédiaires requis → `scan`.
> - seul le résultat final est nécessaire & l'achèvement du flux est garanti → `reduce`.
> - seul le résultat final est nécessaire et le flux est infini → `scan` + `takeLast(1)` ou `take` + `reduce`.

### Utilisation de la mémoire

Lorsque la valeur cumulée est un objet ou un tableau de grande taille, l'utilisation de la mémoire doit être prise en compte.


```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Résultats.: 15(résultats finaux uniquement)
```

## 📚 Opérateurs apparentés.

- [`scan`](. /scan) - produit un résultat intermédiaire pour chaque valeur.
- [`toArray`](. /utility/toArray) - regroupe toutes les valeurs dans un tableau.
- [`count`](https://rxjs.dev/api/operators/count) - compte le nombre de valeurs.
- [`min`](https://rxjs.dev/api/operators/min) / [`max`](https://rxjs.dev/api/operators/max) - obtient les valeurs minimales et maximales.

## Résumé.

L'opérateur `reduce` accumule toutes les valeurs d'un flux et ne fournit que le résultat final **à la fin**. Cela convient lorsque les résultats intermédiaires ne sont pas nécessaires et que seul le résultat final de l'agrégat est requis. Cependant, aucun résultat n'est disponible si le flux n'est pas terminé, donc vous devez utiliser `scan` pour les flux infinis ou mettre une condition de sortie, par exemple avec `take`.
