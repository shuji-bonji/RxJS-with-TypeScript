---
description: "Une fenêtre est un opérateur RxJS qui divise un Observable source en Observables imbriqués au moment où un autre Observable émet une valeur, ce qui est idéal pour le traitement avancé de flux piloté par les événements."
---

# window - Observable pour spécifier la délimitation

L'opérateur `window` regroupe les valeurs de l'Observable source jusqu'à ce qu'un **autre Observable émette une valeur** et restitue ce groupe sous la forme d'un **nouvel Observable**.
Alors que `buffer` renvoie un tableau, `window` renvoie **Observable\<T>**, permettant à d'autres opérateurs d'être appliqués à chaque fenêtre.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

// 100msAttribuer une valeur à chacun d'eux
const source$ = interval(100);

// Utiliser les événements de clic comme déclencheurs
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  window(clicks$),
  mergeAll() // Aplatir chaque fenêtre
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// Chaque clic ouvre une nouvelle fenêtre
```

- Chaque fois que `clicks$` émet une valeur, une nouvelle fenêtre (Observable) est créée.
- Chaque fenêtre peut être traitée comme un Observable indépendant.

[🌐 Official RxJS documentation - `window`](https://rxjs.dev/api/operators/window)

## 💡 Modèle d'utilisation typique.

- Partitionnement du flux en fonction des événements
- Appliquer un traitement différent à chaque fenêtre
- Regroupement des données avec délimitation dynamique
- Traitement agrégé pour chaque fenêtre

## 🔍 Différences avec le buffer

TABLEAU_11___.

```ts
import { interval, timer } from 'rxjs';
import { buffer, window, mergeAll } from 'rxjs';

const source$ = interval(100);
const trigger$ = timer(1000, 1000);

// buffer - Sortie sous forme de tableau
source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Tampon (tableau):', values);
  // Sortie en tant que: Tampon (tableau): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// window - Observable Sortie en tant que
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  console.log('Fenêtre (Observable):', window$);
  // Nid subscribe: window Modèles requis par la spécification de l'opérateur du système
  // (Voir aussiwindow A l'intérieur émis par Observable requis pour la consommation)
  window$.subscribe(value => {
    console.log('  Valeur dans la fenêtre:', value);
  });
});
```

## 🧠 Exemple de code pratique 1 : Comptage par fenêtre

Voici un exemple de déclenchement d'un clic de bouton et de comptage du nombre d'événements jusqu'à ce point.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, map, mergeAll, scan } from 'rxjs';

// Création de boutons
const button = document.createElement('button');
button.textContent = 'Fenêtre séparée';
document.body.appendChild(button);

// Zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// 100msAttribuer une valeur à chacun d'eux
const source$ = interval(100);

// Déclencher un clic sur un bouton
const clicks$ = fromEvent(button, 'click');

let windowCount = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const currentWindow = ++windowCount;
    console.log(`Fenêtre ${currentWindow} Démarrage`);

    // Compte la valeur de chaque fenêtre
    return window$.pipe(
      scan((count) => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  output.textContent = `Fenêtre actuelle: ${windowCount}, Comptage: ${count}`;
});
```

- Une nouvelle fenêtre est créée chaque fois qu'un bouton est cliqué.
- Le nombre de valeurs dans chaque fenêtre est compté en temps réel.

## 🎯 Exemple de code pratique 2 : traitement différent pour chaque fenêtre

Cet exemple avancé permet d'appliquer un traitement différent à chaque fenêtre.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, take, mergeAll, map } from 'rxjs';

const source$ = interval(200);
const clicks$ = fromEvent(document, 'click');

let windowNumber = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Fenêtre paire: Première3Récupère uniquement la première
      console.log(`Fenêtre ${current}: Première3Obtenir le premier`);
      return window$.pipe(take(3));
    } else {
      // Fenêtre impaire: Obtenir tout
      console.log(`Fenêtre ${current}: Obtenir tout`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Valeur: ${value} (Fenêtre ${windowNumber})`);
});
```

- Vous pouvez effectuer des branchements conditionnels et appliquer un traitement différent à chaque fenêtre.
- Chaque fenêtre est un Observable indépendant, vous pouvez donc combiner librement les opérateurs.

## 🎯 Exemple pratique : contrôle avec plusieurs déclencheurs

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { window, mergeAll, scan, map } from 'rxjs';

const source$ = interval(100);

// Déclencheurs multiples: Clic ou3secondes écoulées
const clicks$ = fromEvent(document, 'click');
const threeSeconds$ = timer(3000, 3000);
const trigger$ = merge(clicks$, threeSeconds$);

source$.pipe(
  window(trigger$),
  map((window$, index) => {
    console.log(`Fenêtre ${index + 1} Démarrage`);

    // Calculer le total pour chaque fenêtre
    return window$.pipe(
      scan((sum, value) => sum + value, 0)
    );
  }),
  mergeAll()
).subscribe(sum => {
  console.log('Total actuel:', sum);
});
```

## ⚠️ Notes.

### 1. gestion des abonnements aux fenêtres

Chaque fenêtre est un Observable indépendant et doit faire l'objet d'un abonnement explicite.

```ts
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  // Nid subscribe: window Modèles requis par la spécification de l'opérateur du système
  // Les valeurs ne circulent pas à moins que la fenêtre elle-même ne soit souscrite.
  window$.subscribe(value => {
    console.log('Valeur:', value);
  });
});
```

ou aplatie en utilisant `mergeAll()`, `concatAll()`, `switchAll()`, etc.

```ts
source$.pipe(
  window(trigger$),
  mergeAll() // Fusionner toutes les fenêtres
).subscribe(value => {
  console.log('Valeur:', value);
});
```

### 2. attention aux fuites de mémoire.

**Problème** : si l'Observable déclencheur n'émet pas de valeur, la première fenêtre restera ouverte pour toujours et les valeurs s'accumuleront indéfiniment.

#### ❌ Mauvais exemple : aucun trigger n'est déclenché.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100); // 100msConserver les valeurs de publication pour chaque fenêtre

// Bouton non présent ou non cliqué par l'utilisateur
const button = document.querySelector('#start-button'); // null Possibilité d'une
const clicks$ = fromEvent(button, 'click'); // erreur ou ne se déclenche pas pour toujours

source$.pipe(
  window(clicks$), // clicks$La première fenêtre ne se ferme pas tant que la première fenêtre n'est pas déclenchée.
  mergeAll()
).subscribe();

// Problème:
// - clicks$La première fenêtre reste ouverte pour toujours si la valeur de
// - source$La valeur de0, 1, 2, 3...) continue de s'accumuler dans la mémoire
// - Provoque des fuites de mémoire
```

#### ✅ Bon exemple 1 : Définir le délai d'attente

Définissez un délai d'attente pour éviter que la première fenêtre ne s'ouvre trop tôt.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = button ? fromEvent(button, 'click') : interval(0); // fallback to a dummy observable if button is null

// Clic ou5Ferme la fenêtre au bout d'une seconde, selon ce qui se produit en premier.
const autoClose$ = timer(5000); // 5Quelques secondes plus tard, la valeur est automatiquement émise
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // Toujours5La fenêtre se ferme en quelques secondes
  mergeAll()
).subscribe();
```

#### ✅ Bon exemple 2 : Fermez les fenêtres périodiquement.

Ferme périodiquement une fenêtre et en démarre une nouvelle, même sans clic.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

// 100msAttribuer une valeur à chacun d'eux
const source$ = interval(100);

// Utiliser les événements de clic comme déclencheurs
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  window(clicks$),
  mergeAll() // Aplatir chaque fenêtre
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// Chaque clic ouvre une nouvelle fenêtre
```

### Fenêtres dupliquées

Par défaut, les fenêtres ne se chevauchent pas (la suivante démarre après la fermeture de la précédente).
Si la duplication est nécessaire, utilisez `windowToggle` ou `windowWhen`.

## 🆚 Comparaison des opérateurs basés sur les fenêtres


```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

// 100msAttribuer une valeur à chacun d'eux
const source$ = interval(100);

// Utiliser les événements de clic comme déclencheurs
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  window(clicks$),
  mergeAll() // Aplatir chaque fenêtre
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// Chaque clic ouvre une nouvelle fenêtre
```

## 📚 Opérateurs apparentés.

- [`buffer`](. /buffer) - regroupe les valeurs sous forme de tableau (version tableau de window).
- [`windowTime`](. /windowTime) - division de la fenêtre en fonction du temps.
- [`windowCount`](. /windowCount) - division de la fenêtre basée sur la quantité.
- [`windowToggle`](. /windowToggle) - contrôle de la fenêtre avec un Observable de début et de fin.
- [`windowWhen`](. /windowWhen) - fractionnement de la fenêtre avec des conditions de fermeture dynamiques.
- [`groupBy`](. /groupBy) - regroupe les Observables par clé.

## Résumé.

L'opérateur `window` est un outil puissant qui peut déclencher un Observable externe pour diviser le flux et traiter chaque groupe comme un Observable indépendant.

- ✅ Différents traitements peuvent être appliqués à chaque fenêtre
- ✅ Contrôle flexible basé sur les événements
- ✅ Prise en charge de la manipulation avancée des flux
- ⚠️ Gestion des abonnements nécessaire
- ⚠️ Attention aux fuites de mémoire
