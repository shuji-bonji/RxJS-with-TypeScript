---
description: "toArray est un opérateur utilitaire RxJS qui collecte toutes les valeurs émises jusqu'à ce qu'un Observable se termine dans un seul tableau et l'émet. Idéal pour le traitement par lots, l'affichage de l'interface utilisateur après une récupération en masse, le traitement de l'agrégation, et d'autres situations où vous voulez traiter l'ensemble du flux comme un tableau. Puisqu'il accumule les valeurs jusqu'à la fin, il ne peut pas être utilisé avec des flux infinis."
---

# toArray - Convertir des valeurs en tableau

L'opérateur `toArray` **collecte toutes les valeurs émises jusqu'à ce qu'un Observable se termine en un seul tableau et l'émet**.
Il est utile pour les traitements par lots, l'affichage de l'interface utilisateur après une récupération en masse, l'agrégation, etc.


## 🔰 Syntaxe et comportement de base

```ts
import { of } from 'rxjs';
import { toArray } from 'rxjs';

of(1, 2, 3).pipe(
  toArray()
).subscribe(console.log);

// Sortie :
// [1, 2, 3]
```

Toutes les valeurs sont rassemblées dans un tableau unique et émises lorsque l'Observable se termine.

[🌐 Documentation officielle RxJS - toArray](https://rxjs.dev/api/index/function/toArray)

## 💡 Cas d'utilisation typiques

Il peut être utilisé lorsque vous souhaitez traiter plusieurs résultats asynchrones ensemble ou les afficher à l'interface utilisateur par lots.

```ts
import { interval, of } from 'rxjs';
import { take, toArray, delayWhen, delay } from 'rxjs';

interval(500)
  .pipe(
    take(5),
    delayWhen((val) => of(val).pipe(delay(val * 200))),
    toArray()
  )
  .subscribe((result) => {
    console.log('Reçu en une seule fois à l\'achèvement :', result);
  });

// Sortie :
// Reçu en une seule fois à l'achèvement : [0, 1, 2, 3, 4]
```


## 🧪 Exemple de code pratique (avec interface utilisateur)

```ts
import { interval } from 'rxjs';
import { take, toArray } from 'rxjs';

// Zone d'affichage de la sortie
const toArrayOutput = document.createElement('div');
toArrayOutput.innerHTML = '<h3>Exemple de toArray :</h3>';
document.body.appendChild(toArrayOutput);

// Zone d'affichage des valeurs individuelles
const individualValues = document.createElement('div');
individualValues.innerHTML = '<h4>Valeurs individuelles :</h4>';
toArrayOutput.appendChild(individualValues);

// Zone d'affichage du résultat du tableau
const arrayResult = document.createElement('div');
arrayResult.innerHTML = '<h4>Résultat du tableau :</h4>';
arrayResult.style.marginTop = '20px';
toArrayOutput.appendChild(arrayResult);

// S'abonner à des valeurs individuelles
// Note : les 2 subscribe ci-dessous sont des flux séparés (souscriptions parallèles)
// et non pas des subscribe imbriqués. Ils sont intentionnellement souscrits en parallèle
// pour comparer la différence avec/sans toArray.
interval(500)
  .pipe(take(5))
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Valeur : ${val}`;
    individualValues.appendChild(valueItem);
  });

// S'abonner au même flux en tant que tableau (flux indépendant du subscribe ci-dessus)
interval(500)
  .pipe(take(5), toArray())
  .subscribe((array) => {
    const resultItem = document.createElement('div');
    resultItem.textContent = `Résultat tableau : [${array.join(', ')}]`;
    resultItem.style.fontWeight = 'bold';
    resultItem.style.padding = '10px';
    resultItem.style.backgroundColor = '#f5f5f5';
    resultItem.style.borderRadius = '5px';
    arrayResult.appendChild(resultItem);

    // Affichage individuel des éléments du tableau
    const arrayItems = document.createElement('div');
    arrayItems.style.marginTop = '10px';

    array.forEach((item, index) => {
      const arrayItem = document.createElement('div');
      arrayItem.textContent = `array[${index}] = ${item}`;
      arrayItems.appendChild(arrayItem);
    });

    arrayResult.appendChild(arrayItems);
  });
```


## ✅ Résumé

- `toArray` **émet toutes les valeurs sous forme de tableau à la fin**
- Idéal pour les situations où vous voulez agréger et gérer le flux entier
- Combiné avec `concatMap`, `delay`, etc., il peut aussi gérer **le traitement par lots de séquences asynchrones**
