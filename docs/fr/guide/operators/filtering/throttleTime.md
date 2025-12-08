---
description: "L'opérateur throttleTime laisse passer uniquement la première valeur dans un intervalle de temps spécifié et ignore les valeurs suivantes, réduisant efficacement les événements à haute fréquence. Idéal pour optimiser les événements en temps réel comme le défilement ou le mouvement de la souris."
---

# throttleTime - Laisser passer la première valeur et ignorer les suivantes pendant une durée spécifiée

L'opérateur `throttleTime` laisse passer la première valeur émise et ignore les valeurs suivantes émises dans l'intervalle de temps spécifié.
Contrairement à l'émission de la dernière valeur à intervalles réguliers, il **laisse passer uniquement la première valeur reçue et ignore les valeurs suivantes**.

C'est efficace pour réduire les flux à haute fréquence comme les événements de défilement ou de mouvement de souris.


## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

fromEvent(document, 'click')
  .pipe(throttleTime(2000))
  .subscribe(() => console.log('Cliqué !'));

```

- Reçoit uniquement le premier événement de clic dans un intervalle de 2 secondes, ignore les clics suivants.

[🌐 Documentation officielle RxJS - `throttleTime`](https://rxjs.dev/api/operators/throttleTime)


## 💡 Patterns d'utilisation typiques

- Optimisation de la gestion des événements de défilement ou de mouvement de souris
- Prévention de soumissions multiples par clics répétés sur un bouton
- Réduction de flux de données en temps réel


## 🧠 Exemple de code pratique (avec UI)

Quand la souris est déplacée, les informations de position sont affichées toutes les 100 millisecondes.

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

// Création de la zone de sortie
const container = document.createElement('div');
container.style.height = '200px';
container.style.border = '1px solid #ccc';
container.style.padding = '10px';
container.textContent = 'Déplacez la souris dans cette zone';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
document.body.appendChild(positionDisplay);

// Événement de mouvement de souris
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => ({
    x: event.clientX,
    y: event.clientY
  })),
  throttleTime(100)
).subscribe(position => {
  positionDisplay.textContent = `Position de la souris: X=${position.x}, Y=${position.y}`;
});
```

- Limite les événements de mouvement de souris fréquents à 100ms et affiche uniquement la dernière position.
