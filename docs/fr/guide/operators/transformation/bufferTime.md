---
description: "L'opérateur bufferTime regroupe les valeurs émises à intervalles de temps réguliers et les sort sous forme de tableau. Idéal pour le traitement par lots basé sur le temps comme l'envoi par lots de logs en temps réel, l'agrégation d'événements UI, et l'optimisation réseau. Explique les différences avec buffer et l'implémentation TypeScript type-safe."
---

# bufferTime - Regrouper les valeurs à intervalles de temps réguliers

L'opérateur `bufferTime` produit des **valeurs regroupées sous forme de tableau** à des intervalles de temps spécifiés.
Ceci est utile si vous voulez séparer les flux à intervalles de temps réguliers et les traiter comme un processus par lots.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { bufferTime } from 'rxjs';

// Émet des valeurs toutes les 100ms
const source$ = interval(100);

source$.pipe(
  bufferTime(1000)
).subscribe(buffer => {
  console.log('Valeurs collectées en 1 seconde :', buffer);
});

// Exemple de sortie :
// Valeurs collectées en 1 seconde : [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Valeurs collectées en 1 seconde : [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
// ...
```

- Les valeurs émises pendant 1 seconde sont regroupées dans un tableau et sorties dans l'ordre.

[🌐 Documentation officielle RxJS - `bufferTime`](https://rxjs.dev/api/operators/bufferTime)

## 💡 Patterns d'utilisation typiques

- Envoi par lots à intervalles réguliers
- Traitement groupé des opérations utilisateur (ex: opérations de glissement)
- Collecte de données depuis des capteurs et appareils IoT
- Réduction et compression des informations de log et de trace

## 🧠 Exemple de code pratique (avec interface utilisateur)

Mise en buffer des événements de clic pendant 1 seconde et sortie groupée toutes les secondes.

```ts
import { fromEvent } from 'rxjs';
import { bufferTime } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Flux d'événements de clic
const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  bufferTime(1000)
).subscribe(clickArray => {
  const message = `Nombre de clics en 1 seconde : ${clickArray.length}`;
  console.log(message);
  output.textContent = message;
});
```

- Le nombre de clics par seconde est affiché de manière résumée.
- Le processus de mise en buffer permet de gérer ensemble les occurrences successives d'événements.
