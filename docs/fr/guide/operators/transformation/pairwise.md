---
description: "L'opérateur pairwise produit deux valeurs consécutives sous la forme d'un tableau de paires [valeur précédente, valeur actuelle]. Il peut être utilisé pour des processus qui comparent des valeurs avant et après, tels que la détection de changement de valeur, le calcul de différence, l'analyse de tendance, l'interpolation animée, etc. Une implémentation à sécurité de type dans TypeScript et des exemples pratiques seront décrits."
---

# pairwise - deux valeurs consécutives sont traitées par paires

L'opérateur `pairwise` **combine deux valeurs consécutives issues d'un flux sous la forme d'un tableau `[valeur précédente, valeur actuelle]` et les restitue ensemble**.
Ceci est utile pour comparer la valeur précédente avec la valeur actuelle ou pour calculer la quantité de changement.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { pairwise, take } from 'rxjs';

interval(1000).pipe(
  take(6),
  pairwise()
).subscribe(console.log);

// Sortie:
// [0, 1]
// [1, 2]
// [2, 3]
// [3, 4]
// [4, 5]
```

- La première valeur (0) n'est jamais affichée seule, mais est affichée sous la forme `[0, 1]` lorsque la deuxième valeur (1) arrive.
- C'est toujours la paire **valeur précédente et valeur actuelle** qui est éditée.

[🌐 Official RxJS documentation - `pairwise`](https://rxjs.dev/api/operators/pairwise)

## 💡 Modèle d'utilisation typique.

- Calcul de la quantité de mouvement de la souris ou du toucher.
- Calcul de la quantité de changement (différence) dans les prix ou les valeurs.
- Détection des changements d'état (comparaison de l'état précédent et de l'état actuel)
- Détermination de la direction de défilement

## 🧠 Exemples de code pratique (avec interface utilisateur)

Exemple d'affichage de la direction et de l'importance du mouvement de la souris.

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise } from 'rxjs';

// Création d'une zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Événement de mouvement de la souris
fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY })),
  pairwise()
).subscribe(([prev, curr]) => {
  const deltaX = curr.x - prev.x;
  const deltaY = curr.y - prev.y;
  const direction = deltaX > 0 ? 'Droite' : deltaX < 0 ? 'Gauche' : 'Arrêt';

  output.innerHTML = `
    Précédent: (${prev.x}, ${prev.y})<br>
    Actuel: (${curr.x}, ${curr.y})<br>
    Quantité de mouvement: Δx=${deltaX}, Δy=${deltaY}<br>
    Direction: ${direction}
  `;
});
```

- Lorsque la souris est déplacée, les coordonnées précédentes et actuelles ainsi que l'amplitude du mouvement sont affichées.
- Avec `pairwise`, les coordonnées précédentes et actuelles peuvent être obtenues automatiquement par paires.

## 🎯 Exemple de calcul de la variation d'un nombre.

Exemple pratique de calcul de la quantité de changement (différence) dans un flux de valeurs numériques.

```ts
import { interval } from 'rxjs';
import { map, pairwise, take } from 'rxjs';

// 0, 1, 4, 9, 16, 25 (Nombre de carrés)
interval(500).pipe(
  take(6),
  map(n => n * n),
  pairwise(),
  map(([prev, curr]) => ({
    prev,
    curr,
    diff: curr - prev
  }))
).subscribe(result => {
  console.log(`${result.prev} → ${result.curr} (Différence: +${result.diff})`);
});

// Sortie:
// 0 → 1 (Différence: +1)
// 1 → 4 (Différence: +3)
// 4 → 9 (Différence: +5)
// 9 → 16 (Différence: +7)
// 16 → 25 (Différence: +9)
```

## 🎯 Détermination de la direction de défilement

Voici un exemple de détermination du sens de défilement (haut/bas).

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise, throttleTime } from 'rxjs';

// Création d'une zone de sortie à afficher de manière fixe
const output = document.createElement('div');
output.style.position = 'fixed';
output.style.top = '10px';
output.style.right = '10px';
output.style.padding = '15px';
output.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
output.style.color = 'white';
output.style.fontFamily = 'monospace';
output.style.fontSize = '14px';
output.style.borderRadius = '5px';
output.style.zIndex = '9999';
document.body.appendChild(output);

// Contenu fictif à faire défiler
const content = document.createElement('div');
content.style.height = '200vh'; // Hauteur de la page2Double
content.innerHTML = '<h1>Défilement vers le bas</h1>';
document.body.appendChild(content);

// Obtenir la position de défilement
fromEvent(window, 'scroll').pipe(
  throttleTime(100), // 100msRéduire l'épaisseur de chaque
  map(() => window.scrollY),
  pairwise()
).subscribe(([prevY, currY]) => {
  const diff = currY - prevY;
  const direction = diff > 0 ? '↓ vers le bas' : '↑ Vers le haut';
  const arrow = diff > 0 ? '⬇️' : '⬆️';

  output.innerHTML = `
    ${arrow} Direction du défilement: ${direction}<br>
    Position précédente: ${prevY.toFixed(0)}px<br>
    Position actuelle: ${currY.toFixed(0)}px<br>
    Quantité de mouvement: ${Math.abs(diff).toFixed(0)}px
  `;
});
```

- Lorsque la page défile, les informations relatives à la direction et à la position sont affichées dans une zone fixe située dans le coin supérieur droit.
- `pairwise` vous permet d'obtenir automatiquement la position de défilement précédente et actuelle par paires.

## 🎯 Utilisation sûre de pairwise

Ceci est un exemple d'utilisation de l'inférence de type TypeScript.

```ts
import { from } from 'rxjs';
import { pairwise } from 'rxjs';

interface Stock {
  symbol: string;
  price: number;
  timestamp: number;
}

const stockPrices: Stock[] = [
  { symbol: 'AAPL', price: 150, timestamp: 1000 },
  { symbol: 'AAPL', price: 152, timestamp: 2000 },
  { symbol: 'AAPL', price: 148, timestamp: 3000 },
  { symbol: 'AAPL', price: 155, timestamp: 4000 },
];

from(stockPrices).pipe(
  pairwise()
).subscribe(([prev, curr]) => {
  const change = curr.price - prev.price;
  const changePercent = ((change / prev.price) * 100).toFixed(2);
  const trend = change > 0 ? '📈' : change < 0 ? '📉' : '➡️';

  console.log(
    `${curr.symbol}: $${prev.price} → $${curr.price} ` +
    `(${changePercent}%) ${trend}`
  );
});

// Sortie:
// AAPL: $150 → $152 (1.33%) 📈
// AAPL: $152 → $148 (-2.63%) 📉
// AAPL: $148 → $155 (4.73%) 📈
```

## 🔍 bufferCount(2, 1) vs.

`pairwise()` est équivalent à `bufferCount(2, 1)`.

```ts
import { of } from 'rxjs';
import { pairwise, bufferCount } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== pairwise ===');
source$.pipe(pairwise()).subscribe(console.log);
// Sortie: [1,2], [2,3], [3,4], [4,5]

console.log('=== bufferCount(2, 1) ===');
source$.pipe(bufferCount(2, 1)).subscribe(console.log);
// Sortie: [1,2], [2,3], [3,4], [4,5]
```

**Distinction d'utilisation** :.
- `pairwise()` : traite explicitement des paires de deux valeurs consécutives, l'intention du code est claire
- `bufferCount(2, 1)` : plus flexible (peut gérer plus de trois tailles de fenêtres)

## ⚠️ Notes.

### La première valeur n'est pas affichée.

La première valeur ne peut pas être prise seule, car `pairwise` ne sort rien tant que les deux valeurs ne sont pas alignées.

```ts
import { of } from 'rxjs';
import { pairwise } from 'rxjs';

of(1).pipe(pairwise()).subscribe({
  next: console.log,
  complete: () => console.log('Terminé')
});

// Sortie:
// Terminé
// (Aucune valeur n'est éditée)1(Aucune valeur n'est émise)
```

**Mesures** : si vous voulez traiter la première valeur également, ajoutez une valeur initiale avec `startWith`.

```ts
import { of } from 'rxjs';
import { startWith, pairwise } from 'rxjs';

of(10, 20, 30).pipe(
  startWith(0),
  pairwise()
).subscribe(console.log);

// Sortie:
// [0, 10]
// [10, 20]
// [20, 30]
```

### Utilisation de la mémoire.

L'efficacité de la mémoire est bonne car `pairwise` ne conserve toujours qu'une seule valeur précédente.

## 📚 Opérateurs apparentés.

- [`scan`](. /scan) - processus d'accumulation plus complexe.
- [`bufferCount`](. /bufferCount) - résume les valeurs pour chaque nombre spécifié.
- [`distinctUntilChanged`](. /filtering/distinctUntilChanged) - supprime les valeurs dupliquées consécutives.
- [`startWith`](. /utility/startWith) - ajoute une valeur initiale.

## Résumé.

L'opérateur `pairwise` produit deux valeurs consécutives sous forme de paires `[valeur précédente, valeur courante]`. Ceci est très utile dans les situations** où une comparaison des valeurs précédentes et actuelles est nécessaire, comme le suivi des mouvements de la souris, le calcul des changements de prix et la détection des transitions d'état. Notez que la première valeur n'est pas affichée tant que la seconde n'est pas arrivée, mais cela peut être géré en ajoutant une valeur initiale avec `startWith`.
