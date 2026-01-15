---
description: "L'opérateur first récupère uniquement la première valeur d'un flux, ou la première valeur qui satisfait une condition spécifiée, puis termine le flux. Utile lorsque vous souhaitez traiter uniquement le premier événement ou récupérer des données initiales."
---

# first - Obtenir Première Valeur

L'opérateur `first` récupère uniquement **la première valeur**, ou **la première valeur satisfaisant une condition** d'un flux, puis termine le flux.


## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { first } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Récupérer uniquement la première valeur
numbers$.pipe(
  first()
).subscribe(console.log);

// Récupérer uniquement la première valeur satisfaisant la condition
numbers$.pipe(
  first(n => n > 3)
).subscribe(console.log);

// Sortie:
// 1
// 4
```

- `first()` récupère la première valeur émise et termine.
- Avec une condition, **la première valeur satisfaisant la condition** est récupérée.
- Si aucune valeur ne satisfait la condition, une erreur est émise.

[🌐 Documentation officielle RxJS - `first`](https://rxjs.dev/api/operators/first)


## 💡 Patterns d'utilisation typiques

- Traiter uniquement le premier événement reçu
- Détecter la première donnée satisfaisant une condition (ex: score supérieur à 5)
- Adopter uniquement la première donnée reçue avant un timeout ou une annulation


## 🧠 Exemple de code pratique (avec UI)

**Traiter uniquement le premier clic** même si le bouton est cliqué plusieurs fois.

```ts
import { fromEvent } from 'rxjs';
import { first } from 'rxjs';

const title = document.createElement('div');
title.innerHTML = '<h3>Exemple pratique de first :</h3>';
document.body.appendChild(title);

// Création du bouton
const button = document.createElement('button');
button.textContent = 'Cliquez (réagit uniquement au premier)';
document.body.appendChild(button);

// Création de la zone de sortie
let count = 0;
const output = document.createElement('div');
document.body.appendChild(output);
// Flux de clics du bouton
fromEvent(button, 'click')
  .pipe(first())
  .subscribe(() => {
    const message = document.createElement('div');
    count++;
    message.textContent = `Premier clic détecté! ${count}`;
    output.appendChild(message);
  });
```

- Seul le premier événement de clic est reçu, les suivants sont ignorés.
- Le flux se termine automatiquement avec `complete` après le premier clic.
