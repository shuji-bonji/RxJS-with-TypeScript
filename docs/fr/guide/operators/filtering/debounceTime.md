---
description: "L'opérateur debounceTime émet la dernière valeur après qu'aucune nouvelle valeur n'a été émise pendant une durée spécifiée. Idéal pour optimiser les entrées fréquentes comme les boîtes de recherche ou les événements de redimensionnement de fenêtre."
---

# debounceTime - Émettre la dernière valeur après une période de silence

L'opérateur `debounceTime` émet la dernière valeur du flux si aucune nouvelle valeur n'a été émise pendant la durée spécifiée.
Il est très couramment utilisé pour supprimer les événements fréquents comme les boîtes de recherche.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const searchBox = document.createElement('input');
document.body.appendChild(searchBox);

fromEvent(searchBox, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value),
    debounceTime(300)
  )
  .subscribe(console.log);
```

- Après qu'un événement d'entrée se produit, si aucune autre entrée ne survient dans les 300ms, la valeur est émise.
- Effet de regroupement des événements survenant en succession rapide.

[🌐 Documentation officielle RxJS - `debounceTime`](https://rxjs.dev/api/operators/debounceTime)

## 💡 Patterns d'utilisation typiques

- Envoyer une requête après que l'utilisateur a fini de taper dans une boîte de recherche
- Obtenir la taille finale lors d'un événement de redimensionnement de fenêtre
- Obtenir la position finale lors d'un événement de défilement

## 🧠 Exemple de code pratique (avec UI)

Quand du texte est saisi dans la boîte de recherche, un message indiquant le début de la recherche s'affiche après 300ms sans saisie.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

// Création de la zone de sortie
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Entrer un terme de recherche';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Flux d'entrée
fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300)
).subscribe(value => {
  resultArea.textContent = `Recherche de "${value}" démarrée`;
});
```

- Ne réagit pas immédiatement pendant la saisie,
- Après 300ms sans saisie, démarre la recherche avec la dernière valeur saisie.
