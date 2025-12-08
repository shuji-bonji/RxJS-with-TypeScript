---
description: "L'opérateur startWith insère une valeur initiale spécifiée avant qu'un Observable n'émette des valeurs, ce qui convient à l'initialisation de l'état et à l'affichage initial de l'interface utilisateur."
---

# startWith - Fournir des valeurs initiales

L'opérateur `startWith` **émet une valeur initiale spécifiée avant que l'Observable source n'émette des valeurs**.
Il est utilisé pour la gestion de l'état, l'affichage initial, les valeurs de remplacement, etc.


## 🔰 Syntaxe et comportement de base

```ts
import { of } from 'rxjs';
import { startWith } from 'rxjs';

of('B', 'C').pipe(
  startWith('A')
).subscribe(console.log);
// Sortie :
// A
// B
// C
```

De cette façon, `startWith` ajoute `'A'` en premier, suivi par les valeurs de l'Observable source.

[🌐 Documentation officielle RxJS - startWith](https://rxjs.dev/api/index/function/startWith)

## 💡 Cas d'utilisation typiques

Il est utile lorsque vous souhaitez définir des valeurs initiales pour l'état ou les compteurs. Voici un exemple de compteur démarrant à partir de la valeur initiale `100`.

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

interval(1000)
  .pipe(
    startWith(-1), // Insère d'abord -1
    scan((acc, curr) => acc + 1, 100), // Incrémente à partir de la valeur initiale 100
    take(10) // Produit 10 fois au total
  )
  .subscribe(console.log);
// Sortie :
// 101
// 102
// 103
// 104
// 105
// 106
// 107
// 108
// 109
// 110
```


## 🧪 Exemple de code pratique (avec interface utilisateur)

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

// Zone d'affichage de la sortie
const startWithOutput = document.createElement('div');
startWithOutput.innerHTML = '<h3>Exemple de startWith :</h3>';
document.body.appendChild(startWithOutput);

// Zone d'affichage du compteur
const counterDisplay = document.createElement('div');
counterDisplay.style.fontSize = '24px';
counterDisplay.style.fontWeight = 'bold';
counterDisplay.style.textAlign = 'center';
counterDisplay.style.padding = '20px';
counterDisplay.style.border = '1px solid #ddd';
counterDisplay.style.borderRadius = '5px';
counterDisplay.style.margin = '10px 0';
startWithOutput.appendChild(counterDisplay);

// Zone d'affichage de la liste de valeurs
const valuesList = document.createElement('div');
valuesList.style.marginTop = '10px';
startWithOutput.appendChild(valuesList);

// Flux de comptage (toutes les 1 seconde)
interval(1000)
  .pipe(
    // Démarrage à partir de 100
    startWith(-1),
    // Ajoute 1 à la valeur précédente
    scan((acc, curr) => acc + 1, 100),
    // Fin après 10 fois
    take(10)
  )
  .subscribe((count) => {
    // Mise à jour de l'affichage du compteur
    counterDisplay.textContent = count.toString();

    // Ajout d'une valeur à la liste
    const valueItem = document.createElement('div');

    if (count === 100) {
      valueItem.textContent = `Valeur initiale : ${count} (ajoutée avec startWith)`;
      valueItem.style.color = 'blue';
    } else {
      valueItem.textContent = `Valeur suivante : ${count}`;
    }

    valuesList.appendChild(valueItem);
  });
```


## ✅ Résumé

- `startWith` est utile lorsque vous voulez **insérer une valeur fixe au début**
- Souvent utilisé pour l'initialisation de l'état, les placeholders de l'interface utilisateur, l'affichage initial du formulaire, etc.
- Combiné avec `scan` ou `combineLatest`, il est aussi utilisé pour **construire des fondations de gestion d'état**
