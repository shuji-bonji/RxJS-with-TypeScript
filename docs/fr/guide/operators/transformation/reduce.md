---
description: "reduce accumule toutes les valeurs d'un flux et ne produit que le résultat final à la fin. Idéal pour calculer des sommes, moyennes, max, min de nombres, l'agrégation d'objets, et la construction de tableaux lorsque seul le résultat final est nécessaire. Contrairement à scan, il ne produit pas de résultats intermédiaires et nécessite la fin du flux, il ne peut donc pas être utilisé avec des flux infinis."
---

# reduce - Ne sortir que le résultat final accumulé

L'opérateur `reduce` applique une fonction d'accumulation à chaque valeur dans un flux et **ne sort que le résultat final accumulé à la fin du flux**.
Il fonctionne de la même manière que `Array.prototype.reduce`, sans sortie de résultats intermédiaires.

## 🔰 Syntaxe de base et utilisation

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Sortie : 15 (résultat final uniquement)
```

- `acc` est la valeur accumulée, `curr` est la valeur courante.
- Commence à partir de la valeur initiale (dans ce cas `0`) et accumule séquentiellement.
- Aucune valeur n'est émise tant que le flux n'est pas terminé, **seul le résultat final est émis à la fin**.

[🌐 Documentation officielle RxJS - reduce](https://rxjs.dev/api/operators/reduce)

## 💡 Modes d'utilisation typiques

- Calcul de la somme, moyenne, maximum, minimum de nombres
- Agrégation ou transformation d'objets
- Construction ou jonction de tableaux
- Lorsque seul le résultat final de l'agrégation est nécessaire

## 🔍 Différence avec scan

| Opérateur | Moment de sortie | Contenu de sortie | Cas d'utilisation |
|:---|:---|:---|:---|
| `reduce` | **Une fois à la fin seulement** | Résultat final accumulé | Agrégation où seul le résultat final est nécessaire |
| `scan` | **À chaque valeur** | Tous y compris les résultats intermédiaires | Agrégation en temps réel/gestion d'état |

```ts
import { of } from 'rxjs';
import { reduce, scan } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== reduce ===');
source$.pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Sortie : 15

console.log('=== scan ===');
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Sortie : 1, 3, 6, 10, 15
```

## 🧠 Exemple de code pratique (avec interface utilisateur)

Exemple qui additionne les valeurs de plusieurs champs de saisie et affiche le résultat final au clic du bouton.

```ts
import { fromEvent, from } from 'rxjs';
import { map, reduce, switchMap } from 'rxjs';

// Création des champs de saisie
const inputs: HTMLInputElement[] = [];
for (let i = 1; i <= 3; i++) {
  const label = document.createElement('label');
  label.textContent = `Valeur ${i} : `;
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

// Zone d'affichage du résultat
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Calcul du total au clic du bouton
fromEvent(button, 'click').pipe(
  switchMap(() => {
    // Récupération de toutes les valeurs saisies
    const values = inputs.map(input => Number(input.value) || 0);
    return from(values).pipe(
      reduce((acc, curr) => acc + curr, 0)
    );
  })
).subscribe(total => {
  output.textContent = `Total : ${total}`;
  console.log('Total :', total);
});
```

- Au clic du bouton, toutes les valeurs saisies sont agrégées et seul le total final est affiché.
- Les résultats intermédiaires ne sont pas affichés.

## ⚠️ Notes importantes

### ❌ Les flux infinis ne se terminent jamais (Important)

> [!WARNING]
> **`reduce` n'émet aucune valeur tant que `complete()` n'est pas appelé.** Avec des flux infinis (`interval`, `fromEvent`, etc.), les valeurs ne sont jamais obtenues, causant des problèmes dans les applications réelles.

```ts
import { interval } from 'rxjs';
import { reduce } from 'rxjs';

// ❌ Mauvais exemple : flux infini donc aucune valeur n'est émise
interval(1000).pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Pas de sortie (le flux ne se termine jamais)
```

**Solution 1 : Utiliser `scan` si une agrégation continue est nécessaire**

```ts
import { interval, scan, take } from 'rxjs';

// ✅ Bon exemple : obtenir les résultats intermédiaires en temps réel
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Sortie : 0, 1, 3, 6, 10 (valeur cumulée émise à chaque fois)
```

**Solution 2 : Utiliser `scan` + `takeLast(1)` si seule la valeur finale est nécessaire**

```ts
import { interval, scan, take, takeLast } from 'rxjs';

// ✅ Bon exemple : accumuler avec scan et récupérer uniquement la valeur finale
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0),
  takeLast(1)
).subscribe(console.log);
// Sortie : 10 (résultat final uniquement)
```

**Solution 3 : Utiliser `take` pour définir explicitement une condition de fin**

```ts
import { interval, take, reduce } from 'rxjs';

// ✅ Bon exemple : définir une condition de fin avec take
interval(1000).pipe(
  take(5),
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Sortie : 10
```

> [!TIP]
> **Critères de sélection** :
> - Besoin de résultats intermédiaires → `scan`
> - Besoin uniquement du résultat final & fin du flux garantie → `reduce`
> - Besoin uniquement du résultat final & flux infini → `scan` + `takeLast(1)` ou `take` + `reduce`

## 📚 Opérateurs associés

- [`scan`](./scan) - Émet des résultats intermédiaires à chaque valeur
- [`toArray`](/fr/guide/operators/utility/toArray) - Rassemble toutes les valeurs dans un tableau
- [`count`](https://rxjs.dev/api/operators/count) - Compte le nombre de valeurs
- [`min`](https://rxjs.dev/api/operators/min) / [`max`](https://rxjs.dev/api/operators/max) - Obtenir minimum/maximum

## Résumé

L'opérateur `reduce` accumule toutes les valeurs du flux et **n'émet que le résultat final à la fin**. Approprié lorsque les résultats intermédiaires ne sont pas nécessaires et que seul le résultat final de l'agrégation est requis. Cependant, si le flux ne se termine pas, aucun résultat n'est obtenu, donc avec des flux infinis, utilisez `scan` ou définissez une condition de fin avec `take`, etc.
