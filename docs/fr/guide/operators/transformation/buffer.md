---
description: "L'opérateur buffer accumule les valeurs jusqu'à ce qu'un autre Observable émette, puis les sort sous forme de tableau. Idéal pour le traitement par lots piloté par événements comme l'envoi en masse au clic de bouton ou la sauvegarde de données à la fermeture de fenêtre. Explique l'implémentation TypeScript type-safe."
---

# buffer - Regrouper les valeurs au timing d'un autre Observable

L'opérateur `buffer` accumule les valeurs d'un Observable source **jusqu'à ce qu'un autre Observable émette des valeurs** et sort les valeurs accumulées à ce moment-là **sous forme de tableau**.
Ceci est utile lorsque vous souhaitez contrôler la mise en mémoire tampon en fonction d'événements ou de signaux externes, plutôt qu'en fonction du temps ou du nombre.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval, fromEvent } from 'rxjs';
import { buffer } from 'rxjs';

// Émet des valeurs toutes les 100ms
const source$ = interval(100);

// Utilise l'événement de clic comme déclencheur
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  buffer(clicks$)
).subscribe(bufferedValues => {
  console.log('Valeurs accumulées jusqu\'au clic :', bufferedValues);
});

// Exemple de sortie (à chaque clic) :
// Valeurs accumulées jusqu'au clic : [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
// Valeurs accumulées jusqu'au clic : [11, 12, 13, 14, 15, 16, 17]
// ...
```

- Chaque fois que `clicks$` émet une valeur, les valeurs accumulées jusqu'à ce point sont sorties sous forme de tableau.
- La caractéristique est que la délimitation du buffer peut être contrôlée par un Observable externe.

[🌐 Documentation officielle RxJS - `buffer`](https://rxjs.dev/api/operators/buffer)

## 💡 Patterns d'utilisation typiques

- Traitement par lots déclenché par des actions utilisateur
- Collecte et envoi de données basés sur des signaux externes
- Regroupement d'événements avec délimitation dynamique
- Envoi groupé lors de l'établissement de connexion WebSocket ou API

## 🔍 Différences avec bufferTime / bufferCount

| Opérateur | Timing de délimitation | Utilisation |
|:---|:---|:---|
| `buffer` | **Émission d'un autre Observable** | Contrôle piloté par événements |
| `bufferTime` | **Temps fixe** | Traitement par lots basé sur le temps |
| `bufferCount` | **Nombre fixe** | Traitement par lots basé sur le nombre |

```ts
import { interval, timer } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);
// Déclenchement toutes les secondes
const trigger$ = timer(1000, 1000);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Valeurs par seconde :', values);
});

// Sortie :
// Valeurs par seconde : [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Valeurs par seconde : [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
```

## 🧠 Exemple de code pratique (avec interface utilisateur)

Voici un exemple d'enregistrement de tous les événements de mouvement de la souris déclenché par un clic de bouton.

```ts
import { fromEvent } from 'rxjs';
import { map, buffer } from 'rxjs';

// Création du bouton et de la zone de sortie
const button = document.createElement('button');
button.textContent = 'Enregistrer les mouvements de souris';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Événements de mouvement de souris
const mouseMoves$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
);

// Utilise le clic de bouton comme déclencheur
const clicks$ = fromEvent(button, 'click');

mouseMoves$.pipe(
  buffer(clicks$)
).subscribe(positions => {
  const message = `Nombre d'événements détectés : ${positions.length}`;
  console.log(message);
  console.log('Données de coordonnées :', positions.slice(0, 5)); // Affiche seulement les 5 premiers
  output.textContent = message;
});
```

- Tous les mouvements de la souris jusqu'au clic sur le bouton sont stockés dans le buffer.
- Comme ils sont tous traités ensemble au moment du clic, le traitement par lots à un moment arbitraire est possible.

## 🎯 Exemple avancé avec plusieurs déclencheurs

Un contrôle plus flexible est possible en combinant plusieurs Observables de déclenchement.

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { buffer, mapTo } from 'rxjs';

const source$ = interval(100);

// Déclencheurs multiples : clic ou 5 secondes écoulées
const clicks$ = fromEvent(document, 'click').pipe(mapTo('click'));
const fiveSeconds$ = timer(5000, 5000).pipe(mapTo('timer'));
const trigger$ = merge(clicks$, fiveSeconds$);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log(`Sortie du buffer (${values.length} éléments) :`, values);
});
```

## ⚠️ Points d'attention

### Attention aux fuites de mémoire

`buffer` continue d'accumuler des valeurs jusqu'au prochain déclenchement, ce qui peut mettre la mémoire sous pression si un déclenchement ne se produit pas pendant longtemps.

```ts
// Mauvais exemple : possibilité que le déclencheur ne se produise pas
const neverTrigger$ = fromEvent(document.querySelector('.non-existent'), 'click');

source$.pipe(
  buffer(neverTrigger$) // Le déclencheur ne se produit pas, le buffer s'accumule indéfiniment
).subscribe();
```

**Mesures** :
- Limiter la taille maximale du buffer en combinaison avec `bufferTime` et `bufferCount`
- Ajouter un traitement de timeout

```ts
import { interval, fromEvent, timer, race } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);

// Déclencheurs multiples : clic ou 5 secondes écoulées
const clicks$ = fromEvent(document, 'click');
const timeout$ = timer(10000); // Timeout après 10 secondes max

source$.pipe(
  buffer(race(clicks$, timeout$)) // Émet au premier des deux
).subscribe(values => {
  console.log('Buffer :', values);
});
```

## 📚 Opérateurs associés

- [`bufferTime`](./bufferTime) - Mise en buffer basée sur le temps
- [`bufferCount`](./bufferCount) - Mise en buffer basée sur le nombre
- [`bufferToggle`](https://rxjs.dev/api/operators/bufferToggle) - Contrôle de mise en buffer avec Observables de début/fin
- [`bufferWhen`](https://rxjs.dev/api/operators/bufferWhen) - Mise en buffer avec condition de fermeture dynamique
- [`window`](./windowTime) - Retourne un Observable au lieu d'un buffer

## Résumé

L'opérateur `buffer` est un outil puissant pour traiter un lot de valeurs déclenché par un Observable externe. Il permet un traitement par lots **piloté par événements** plutôt que par le temps ou le nombre. Cependant, il faut faire attention aux fuites de mémoire lorsque les déclenchements ne se produisent pas.
