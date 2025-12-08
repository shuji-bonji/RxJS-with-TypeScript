---
description: "La fonction de création zip aligne et associe les valeurs dans l'ordre correspondant à partir de plusieurs Observables et les émet au moment où toutes les sources ont émis leurs valeurs une par une."
---

# zip - appairer les valeurs correspondantes

`zip` est une fonction de création qui regroupe les **valeurs ordonnées correspondantes** provenant de plusieurs Observables et les produit sous forme de tableau ou de tuple.
Elle attend que les valeurs arrivent de tous les Observables sources, une par une, et crée les paires quand elles sont prêtes.


## Syntaxe de base et utilisation

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = of('A', 'B', 'C');
const source2$ = interval(1000).pipe(
  map((val) => val * 10),
  take(3)
);

zip(source1$, source2$).subscribe(([letter, number]) => {
  console.log(letter, number);
});

// Sortie:
// A 0
// B 10
// C 20
```

- Lorsque chaque Observable émet une valeur à la fois, une paire est créée et émise.
- Si l'une d'entre elles est retardée, elle attendra que les deux soient alignées.

[🌐 Documentation officielle RxJS - `zip`](https://rxjs.dev/api/index/function/zip)


## Modèles d'utilisation typiques

- **Mettre en correspondance les requêtes et les réponses**
- **Associer de manière synchrone des identifiants aux données correspondantes**
- **Combiner plusieurs flux traités en parallèle en un seul ensemble**


## Exemples de code pratique (avec interface utilisateur)

Exemple de **combinaison et d'affichage** de différentes sources de données (fruits et prix).

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

// Créer une zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de zip:</h3>';
document.body.appendChild(output);

// Flux de noms de fruits
const fruits$ = of('🍎 Pomme', '🍌 Banane', '🍇 Raisin');

// Flux de prix (émis toutes les 2 secondes)
const prices$ = interval(2000).pipe(
  map((i) => [100, 200, 300][i]),
  take(3)
);

// zip et afficher
zip(fruits$, prices$).subscribe(([fruit, price]) => {
  const item = document.createElement('div');
  item.textContent = `${fruit} - ${price}€`;
  output.appendChild(item);
});
```

- Les listes de fruits et de prix sont appariées et affichées **quand** elles sont alignées dans une correspondance un à un.
- Si l'une ou l'autre est manquante, elle ne sera pas émise à ce moment-là.


## Opérateurs associés

- **[zipWith](/fr/guide/operators/combination/zipWith)** - Version opérateur Pipeable (utilisée dans le pipeline)
- **[combineLatest](/fr/guide/creation-functions/combination/combineLatest)** - Fonction de création combinant les dernières valeurs
- **[withLatestFrom](/fr/guide/operators/combination/withLatestFrom)** - Déclencheurs du flux principal uniquement
