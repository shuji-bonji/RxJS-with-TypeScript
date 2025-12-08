---
description: "bufferCount est un opérateur de transformation RxJS qui regroupe les valeurs par nombre spécifié et les sort sous forme de tableau. Idéal pour le contrôle de flux basé sur le nombre comme le traitement par lots, l'agrégation de données par nombre fixe d'éléments, et la transmission fractionnée de paquets. L'inférence de type TypeScript permet des opérations de tableau type-safe."
---

# bufferCount - Regrouper les valeurs par nombre spécifié

L'opérateur `bufferCount` **regroupe** chaque nombre spécifié de valeurs émises et les **sort sous forme de tableau**.
Ceci est utile lorsque vous souhaitez effectuer un traitement par lots délimité par le nombre de valeurs.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { bufferCount } from 'rxjs';

// Émet des valeurs toutes les 100ms
const source$ = interval(100);

source$.pipe(
  bufferCount(5)
).subscribe(buffer => {
  console.log('Valeurs par groupe de 5 :', buffer);
});

// Sortie :
// Valeurs par groupe de 5 : [0, 1, 2, 3, 4]
// Valeurs par groupe de 5 : [5, 6, 7, 8, 9]
// ...
```

- Produit un tableau regroupant 5 valeurs ensemble.
- Il est caractéristique de regrouper sur une **base de nombre** plutôt que sur le temps.

[🌐 Documentation officielle RxJS - `bufferCount`](https://rxjs.dev/api/operators/bufferCount)

## 💡 Patterns d'utilisation typiques

- Transmission fractionnée de paquets de données
- Stockage et traitement par lots pour chaque nombre fixe d'éléments
- Agrégation des événements d'entrée par un nombre fixe de fois

## 🧠 Exemple de code pratique (avec interface utilisateur)

Cet exemple montre un résumé des frappes de clavier toutes les 5 frappes.

```ts
import { fromEvent } from 'rxjs';
import { map, bufferCount } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Flux d'événements de frappe clavier
fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  bufferCount(5)
).subscribe(keys => {
  const message = `5 entrées : ${keys.join(', ')}`;
  console.log(message);
  output.textContent = message;
});
```

- Chaque fois qu'une touche est pressée 5 fois, ces 5 frappes sont affichées ensemble.
- Vous pouvez expérimenter le processus d'agrégation en fonction du nombre d'éléments.
