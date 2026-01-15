---
description: "auditTime est un opérateur de filtrage RxJS qui attend une durée spécifiée après la réception d'une valeur et émet la dernière valeur de cette période. Idéal pour échantillonner périodiquement le dernier état lors d'événements à haute fréquence comme le suivi de la position de défilement, le redimensionnement de fenêtre ou le mouvement de souris. Il est important de comprendre les différences avec throttleTime et debounceTime pour un usage approprié."
---

# auditTime - Dernière valeur par période

L'opérateur `auditTime` **attend la durée spécifiée** après la réception d'une valeur et émet la **dernière valeur** de cette période. Ensuite, il attend la prochaine valeur.


## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Clic !'));
```

**Flux d'opération** :
1. Premier clic
2. Attend 1 seconde (les clics pendant cette période sont enregistrés mais non émis)
3. Émet le dernier clic après 1 seconde
4. Attend le clic suivant

[🌐 Documentation officielle RxJS - `auditTime`](https://rxjs.dev/api/operators/auditTime)


## 🆚 Comparaison avec throttleTime

`throttleTime` et `auditTime` sont similaires mais émettent des valeurs différentes.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: émet la première valeur
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Sortie: 0, 4, 8 (première valeur de chaque période)

// auditTime: émet la dernière valeur
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Sortie: 3, 6, 9 (dernière valeur de chaque période)
```

**Comparaison chronologique** :
```
Source:     0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (première) (première) (première)

audit:      -------3--------6--------9----|
                  (dernière) (dernière) (dernière)
```

| Opérateur | Valeur émise | Moment d'émission | Cas d'utilisation |
|---|---|---|---|
| `throttleTime(ms)` | **Première** valeur de la période | À la réception | Réponse immédiate nécessaire |
| `auditTime(ms)` | **Dernière** valeur de la période | Fin de période | Dernier état nécessaire |
| `debounceTime(ms)` | **Dernière** valeur après silence | Après arrêt de saisie | Attendre la fin de saisie |


## 💡 Patterns d'utilisation typiques

1. **Optimisation du redimensionnement de fenêtre**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // Obtenir la dernière taille toutes les 200ms
   ).subscribe(() => {
     console.log(`Taille de fenêtre: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Suivi de la position de défilement**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map } from 'rxjs';

   fromEvent(window, 'scroll').pipe(
     auditTime(100),
     map(() => ({
       scrollY: window.scrollY,
       scrollX: window.scrollX
     }))
   ).subscribe(position => {
     console.log(`Position de défilement: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```

3. **Mouvement de glisser-déposer fluide**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map, takeUntil, switchMap } from 'rxjs';

   // Création d'un élément déplaçable
   const box = document.createElement('div');
   box.style.width = '100px';
   box.style.height = '100px';
   box.style.backgroundColor = '#3498db';
   box.style.position = 'absolute';
   box.style.cursor = 'move';
   box.style.left = '100px';
   box.style.top = '100px';
   box.textContent = 'Glisser';
   box.style.display = 'flex';
   box.style.alignItems = 'center';
   box.style.justifyContent = 'center';
   box.style.color = 'white';
   document.body.appendChild(box);

   const mouseDown$ = fromEvent<MouseEvent>(box, 'mousedown');
   const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
   const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

   // Implémentation du glisser-déposer
   mouseDown$.pipe(
     switchMap(startEvent => {
       const startX = startEvent.clientX - box.offsetLeft;
       const startY = startEvent.clientY - box.offsetTop;

       return mouseMove$.pipe(
         auditTime(16), // Mise à jour de position à ~60FPS (16ms)
         map(moveEvent => ({
           x: moveEvent.clientX - startX,
           y: moveEvent.clientY - startY
         })),
         takeUntil(mouseUp$)
       );
     })
   ).subscribe(position => {
     box.style.left = `${position.x}px`;
     box.style.top = `${position.y}px`;
   });
   ```


## 🧠 Exemple de code pratique (Suivi de souris)

Un exemple de suivi du mouvement de la souris et d'affichage de la dernière position à intervalles réguliers.

```ts
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// Création des éléments UI
const container = document.createElement('div');
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relative';
container.textContent = 'Déplacez la souris dans cette zone';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
positionDisplay.style.fontFamily = 'monospace';
document.body.appendChild(positionDisplay);

const dot = document.createElement('div');
dot.style.width = '10px';
dot.style.height = '10px';
dot.style.borderRadius = '50%';
dot.style.backgroundColor = '#e74c3c';
dot.style.position = 'absolute';
dot.style.display = 'none';
container.appendChild(dot);

// Événement de mouvement de souris
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // Obtenir la dernière position toutes les 100ms
).subscribe(position => {
  positionDisplay.textContent = `Dernière position (intervalle 100ms): X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // Déplacer le point à la dernière position
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = 'block';
});
```

Ce code récupère et affiche uniquement la dernière position toutes les 100ms même si la souris se déplace fréquemment.


## 🎯 Différence avec debounceTime

`auditTime` et `debounceTime` **émettent tous deux la dernière valeur** mais à des **moments complètement différents**.

### Différence fondamentale

| Opérateur | Comportement | Cas d'utilisation |
|---|---|---|
| `auditTime(ms)` | **Émet après ms** après réception (même si l'entrée continue) | Échantillonnage périodique |
| `debounceTime(ms)` | Émet **après que l'entrée s'arrête** pendant ms | Attendre la fin de saisie |

### Exemple concret : Différence avec l'entrée de recherche

```ts
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Entrer terme de recherche';
document.body.appendChild(input);

// auditTime: exécute la recherche toutes les 300ms même pendant la saisie
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Recherche:', input.value);
});

// debounceTime: exécute la recherche 300ms après l'arrêt de saisie
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Recherche:', input.value);
});
```

### Visualisation chronologique

Quand l'utilisateur tape rapidement "ab" → "abc" → "abcd":

```
Événements de saisie:   a--b--c--d------------|
                        ↓
auditTime:              ------c-----d----------|
                      (après 300ms) (après 300ms)
                      → Recherche "abc", recherche "abcd" (2 fois)

debounceTime:           --------------------d-|
                                        (300ms après arrêt)
                      → Recherche "abcd" (1 fois seulement)
```

**Moyen mnémotechnique** :
- **`auditTime`**: "Audit périodique" → Vérification régulière obligatoire
- **`debounceTime`**: "Attendre le calme" → Attendre que ça se calme

### Utilisation pratique

```ts
// ✅ auditTime approprié
// - Suivi de position de défilement (obtenir périodiquement même pendant un défilement continu)
fromEvent(window, 'scroll').pipe(
  auditTime(100)  // Obtenir la dernière position toutes les 100ms
).subscribe(/* ... */);

// ✅ debounceTime approprié
// - Boîte de recherche (rechercher après la fin de saisie)
fromEvent(searchInput, 'input').pipe(
  debounceTime(300)  // Attendre 300ms après l'arrêt de saisie
).subscribe(/* ... */);
```


## 📋 Utilisation type-safe

Un exemple d'implémentation type-safe utilisant les génériques TypeScript.

```ts
import { Observable, fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

interface MousePosition {
  x: number;
  y: number;
  timestamp: number;
}

function trackMousePosition(
  element: HTMLElement,
  intervalMs: number
): Observable<MousePosition> {
  return fromEvent<MouseEvent>(element, 'mousemove').pipe(
    auditTime(intervalMs),
    map(event => ({
      x: event.clientX,
      y: event.clientY,
      timestamp: Date.now()
    } as MousePosition))
  );
}

// Exemple d'utilisation
const canvas = document.createElement('div');
canvas.style.width = '400px';
canvas.style.height = '300px';
canvas.style.border = '1px solid black';
document.body.appendChild(canvas);

trackMousePosition(canvas, 200).subscribe(position => {
  console.log(`Position: (${position.x}, ${position.y}) à ${position.timestamp}`);
});
```


## 🔄 Combinaison auditTime et throttleTime

Dans certains scénarios, les deux peuvent être combinés.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(100).pipe(take(50));

// Ordre throttleTime → auditTime
source$.pipe(
  throttleTime(1000),  // Laisser passer la première valeur chaque seconde
  auditTime(500)       // Puis attendre 500ms et émettre la dernière valeur
).subscribe(console.log);
```


## ⚠️ Erreurs courantes

> [!WARNING]
> `auditTime` et `debounceTime` ont des comportements différents. Pour les entrées de recherche où vous voulez **attendre que l'utilisateur arrête de taper**, utilisez `debounceTime`. `auditTime` émet des valeurs à intervalles réguliers même pendant la saisie.

### Incorrect : Confondre auditTime et debounceTime

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

// Création du champ de recherche
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Recherche...';
document.body.appendChild(input);

// ❌ Mauvais exemple: utiliser auditTime pour l'entrée de recherche
fromEvent(input, 'input').pipe(
  auditTime(300) // La recherche est exécutée toutes les 300ms même pendant la saisie
).subscribe(() => {
  console.log('Recherche exécutée');
});
```

### Correct : Utiliser debounceTime

```ts
import { fromEvent } from 'rxjs';
import { debounceTime } from 'rxjs';

// Création du champ de recherche
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Recherche...';
document.body.appendChild(input);

// ✅ Bon exemple: utiliser debounceTime pour l'entrée de recherche
fromEvent(input, 'input').pipe(
  debounceTime(300) // Attend 300ms après l'arrêt de saisie avant de rechercher
).subscribe(() => {
  console.log('Recherche exécutée', input.value);
});
```


## 🎓 Résumé

### Quand utiliser auditTime
- ✅ Quand vous avez besoin de la dernière valeur à intervalles réguliers
- ✅ Événements à haute fréquence comme le défilement, le redimensionnement, le mouvement de souris
- ✅ Quand un échantillonnage périodique est nécessaire
- ✅ Quand vous voulez refléter le dernier état

### Quand utiliser throttleTime
- ✅ Quand une réponse immédiate est nécessaire
- ✅ Quand vous voulez démarrer le traitement avec la première valeur
- ✅ Prévention de clics répétés

### Quand utiliser debounceTime
- ✅ Quand vous voulez attendre la fin de saisie
- ✅ Recherche, auto-complétion
- ✅ Attendre que l'utilisateur arrête de taper

### Points d'attention
- ⚠️ `auditTime` émet uniquement la dernière valeur de la période (les valeurs intermédiaires sont ignorées)
- ⚠️ Pas très efficace si défini sur un intervalle court
- ⚠️ `throttleTime` ou `debounceTime` peut être plus approprié selon le cas


## 🚀 Prochaines étapes

- **[throttleTime](./throttleTime)** - Apprendre à laisser passer la première valeur
- **[debounceTime](./debounceTime)** - Apprendre à émettre des valeurs après l'arrêt de saisie
- **[filter](./filter)** - Apprendre le filtrage basé sur les conditions
- **[Exemples pratiques d'opérateurs de filtrage](./practical-use-cases)** - Apprendre des cas d'utilisation réels
