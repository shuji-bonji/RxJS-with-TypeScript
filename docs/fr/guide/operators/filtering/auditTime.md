---
description: "auditTime est un opérateur de filtrage de RxJS qui attend un temps spécifié lorsqu'une valeur est émise et produit la dernière valeur au cours de cette période. Il est préférable de l'utiliser lorsque vous souhaitez échantillonner périodiquement le dernier état sur des événements à haute fréquence tels que le suivi de la position du défilement, le redimensionnement de la fenêtre, le mouvement de la souris, etc. Il est important de comprendre la différence entre cet opérateur et throttleTime et debounceTime et de les utiliser de manière appropriée."
---

# auditTime - dernière valeur émise après le temps spécifié

L'opérateur `auditTime` attend un **temps spécifié** après l'émission d'une valeur et produit la **dernière valeur** pendant cette période. Il attend ensuite l'arrivée de la valeur suivante.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Cliquez.！'));
```

**Flux des opérations** :.
1. le premier clic se produit
2. attendre 1 seconde (les clics pendant ce temps sont enregistrés mais ne sont pas émis)
3. sortie du dernier clic après 1 seconde
Attendre le clic suivant

[🌐 RxJS official documentation - `auditTime`](https://rxjs.dev/api/operators/auditTime)

## 🆚 Contraste avec throttleTime

`throttleTime` et `auditTime` sont similaires, mais diffèrent dans les valeurs qu'ils produisent.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Sortie de la première valeur
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Sortie.: 0, 4, 8(première valeur de chaque période)

// auditTime: Sortie de la dernière valeur
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Sortie.: 3, 6, 9(dernière valeur de chaque période)
```

**Comparaison de lignes de temps** :.

```
Source:     0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (Première)   (Première)   (Première)

audit:      -------3--------6--------9----|
                  (Dernière)   (Dernière)   (Dernière)
```

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Cliquez.！'));
```

## 💡 Modèle d'utilisation typique

1. **Optimiser le redimensionnement des fenêtres**.

```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // 200msObtenir la dernière taille dans l'intervalle
   ).subscribe(() => {
     console.log(`Taille de la fenêtre: ${window.innerWidth}x${window.innerHeight}`);
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
     console.log(`Position du défilement: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```

3. **Mouvement de glissement en douceur**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map, takeUntil, switchMap } from 'rxjs';

   // Créer des éléments pouvant être glissés
   const box = document.createElement('div');
   box.style.width = '100px';
   box.style.height = '100px';
   box.style.backgroundColor = '#3498db';
   box.style.position = 'absolute';
   box.style.cursor = 'move';
   box.style.left = '100px';
   box.style.top = '100px';
   box.textContent = 'Glissement';
   box.style.display = 'flex';
   box.style.alignItems = 'center';
   box.style.justifyContent = 'center';
   box.style.color = 'white';
   document.body.appendChild(box);

   const mouseDown$ = fromEvent<MouseEvent>(box, 'mousedown');
   const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
   const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

   // Mettre en œuvre des opérations de glissement
   mouseDown$.pipe(
     switchMap(startEvent => {
       const startX = startEvent.clientX - box.offsetLeft;
       const startY = startEvent.clientY - box.offsetTop;

       return mouseMove$.pipe(
         auditTime(16), // Approximativement.60FPS(voir aussi16ms) pour mettre à jour la position
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

## 🧠 Exemple de code pratique (suivi de la souris)

Cet exemple suit les mouvements de la souris et affiche la dernière position à intervalles réguliers.

```

ts.
import { fromEvent } from 'rxjs' ;
import { auditTime, map } from 'rxjs' ;

// Création d'éléments d'interface utilisateur
const container = document.createElement('div') ;.
container.style.height = '300px' ;
container.style.border = '2px solid #3498db' ;
container.style.padding = '20px' ;
container.style.position = 'relative' ;
container.textContent = "Veuillez déplacer la souris dans cette zone" ;
document.body.appendChild(container) ;

const positionDisplay = document.createElement('div') ;
positionDisplay.style.marginTop = '10px' ;
positionDisplay.style.fontFamily = 'monospace' ;
document.body.appendChild(positionDisplay) ;

const dot = document.createElement('div') ;
dot.style.width = '10px' ;
dot.style.height = '10px' ;
dot.style.borderRadius = '50%' ;
dot.style.backgroundColor = '#e74c3c' ;
dot.style.position = 'absolute' ;
dot.style.display = 'none' ;
container.appendChild(dot) ;

// Événement de déplacement de la souris
fromEvent\<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect() ;
    return {
      x : event.clientX - rect.left,.
      y : event.clientY - rect.top
    } ;
  }),
  auditTime(100) // Récupère la dernière position toutes les 100ms
).subscribe(position => {
  positionDisplay.textContent = `Dernière position (toutes les 100ms) : X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}` ;

  // Déplacer le point vers la dernière position
  dot.style.left = `${position.x - 5}px` ;
  dot.style.top = `${position.y - 5}px` ;
  dot.style.display = 'block' ;
}) ;

```

Ce code ne récupère et n'affiche la dernière position qu'à chaque fois que la souris est déplacée, même si la souris est déplacée fréquemment,100msLe code ne récupère et n'affiche la dernière position que pour chaque mouvement de la souris.

## 🎯 debounceTime Différences entre

`auditTime` et `debounceTime` est que**affichent tous deux la dernière valeur**mais le code**Le timing est complètement différent**la dernière valeur est émise.

### La différence décisive

| L'opérateur | l'opération | utilisation différente du système |
|---|---|---|
| `auditTime(ms)` | À l'arrivée d'une valeur**msToujours éditer après**(même si l'entrée se poursuit) | Échantillonnage à intervalles réguliers |
| `debounceTime(ms)` | **Après l'arrêt de l'entrée**msSortie après | Attendre la fin de l'entrée |

### Exemples spécifiques：Différences dans l'entrée de la recherche

```

ts.
import { fromEvent } from 'rxjs' ;
import { auditTime, debounceTime } from 'rxjs' ;

const input = document.createElement('input') ;
input.placeholder = 'Search word input' ;
document.body.appendChild(input) ;

// auditTime : Exécution de la recherche toutes les 300 ms même pendant la saisie
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Search:', input.value) ;
}) ;

// debounceTime : attendre 300ms après l'arrêt de l'entrée, puis exécuter la recherche
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Search:', input.value) ;
}) ;

```

### Différences observées dans la ligne de temps

Différence observée lorsqu'un utilisateur clique sur "ab'→'abc'→'abcd' lors d'une saisie rapide:

```

Événement d'entrée : a--b--c--d------------|
              ↓
auditTime : ------c-----d----------|
            (après 300 ms) (après 300 ms)
            → Recherche de 'abc', recherche de 'abcd' (2 fois au total)

debounceTime : --------------------d-|
                              (300 ms après l'arrêt)
            → Recherche de "abcd" (une seule fois au total)

```

**Facile à retenir**:
- **`auditTime`**: Régulièrement contrôlé (audit)"→ "Toujours vérifier à intervalles réguliers
- **`debounceTime`**: Attendez qu'il se calme (...)".debounceAttendez que ce soit calme.→ Attendre qu'il y ait du calme

### Utilisation pratique

```

ts.
// ✅ auditTime si nécessaire
// - Suivi de la position de défilement (nous voulons l'obtenir périodiquement, même si nous défilons tout le temps)
fromEvent(window, 'scroll').pipe(
  auditTime(100) // récupère la dernière position toutes les 100ms
).subscribe(/* ... */) ;

// ✅ si debounceTime est approprié.
// - boîte de recherche (nous voulons effectuer une recherche une fois la saisie terminée)
fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // attend 300ms après l'arrêt de la saisie
).subscribe(/* ... */) ;

```

## 📋 Utilisation sûre du point de vue du type

TypeScript Il s'agit d'un exemple d'implémentation sûre du point de vue du type, qui utilise les éléments génériques dans le cadre d'un projet de développement.

```

ts.
import { Observable, fromEvent } from 'rxjs' ;
import { auditTime, map } from 'rxjs' ;

interface MousePosition {
  x : nombre ;
  y : nombre ;
  timestamp : nombre ; }
}

function trackMousePosition(
  element : HTMLElement,.
  interval : nombre
) : Observable {
  return fromEvent\<MouseEvent>(element, 'mousemove').pipe(
    auditTime(intervalMs),.
    map(event => ({
      x : event.clientX, event.
      y : event.clientY,.
      timestamp : Date.now())
    } as MousePosition))
  ) ;
}

// Exemple d'utilisation
const canvas = document.createElement('div') ;
canvas.style.width = '400px' ;
canvas.style.height = '300px' ;
canvas.style.border = '1px solid black' ;
document.body.appendChild(canvas) ;

trackMousePosition(canvas, 200).subscribe(position => {
  console.log(`Position : (${position.x}, ${position.y}) at ${position.timestamp}`) ;
}) ;

```

## 🔄 auditTime et throttleTime Combinaison de

Dans certains cas, les deux peuvent être combinés.

```

ts.
import { interval } from 'rxjs' ;
import { throttleTime, auditTime, take } from 'rxjs' ;

const source$ = interval(100).pipe(take(50)) ;.

// ordre de throttleTime → auditTime
source$.pipe(
  throttleTime(1000), // passer la première valeur toutes les secondes
  auditTime(500) // puis attendre 500ms et sortir la dernière valeur
).subscribe(console.log) ;.

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Cliquez.！'));

ts.
import { fromEvent } from 'rxjs' ;
import { auditTime } from 'rxjs' ;

// Création d'un champ de saisie de recherche
const input = document.createElement('input') ;.
input.type = 'text' ;
input.placeholder = 'Rechercher...' ;
document.body.appendChild(input) ;

// ❌ Mauvais exemple : utilisation de l'auditTime pour l'entrée de recherche
fromEvent(input, 'input').pipe(
  auditTime(300) // la recherche est effectuée toutes les 300ms pendant la saisie
).subscribe(() => {
  console.log('Search executed') ;
}) ;

```

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Cliquez.！'));
```

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Cliquez !'));
```

ts.
import { fromEvent } from 'rxjs' ;
import { debounceTime } from 'rxjs' ;

// Création d'un champ de saisie de recherche
const input = document.createElement('input') ;.
input.type = 'text' ;
input.placeholder = 'Rechercher...' ;
document.body.appendChild(input) ;

// ✅ Bon exemple : utiliser debounceTime pour une entrée de recherche
fromEvent(input, 'input').pipe(
  debounceTime(300) // Attendre 300ms après l'arrêt de l'entrée avant d'effectuer une recherche
).subscribe(() => {
  console.log('Search executed', input.value) ;
}) ;
```

## 🎓 Résumé

### Quand auditTime doit être utilisé.
- ✅ Lorsque des valeurs actualisées sont requises à intervalles réguliers.
- ✅ Événements à haute fréquence tels que le défilement, le redimensionnement, le mouvement de la souris.
- ✅ Lorsqu'un échantillonnage périodique est nécessaire
- lorsque vous souhaitez refléter l'état le plus récent.

### Quand throttleTime doit être utilisé .
- ✅ Lorsqu'une réponse immédiate est nécessaire
- ✅ Si vous voulez commencer le traitement avec la première valeur
- ✅ Prévention de l'enfoncement des boutons

### Quand utiliser debounceTime.
- ✅ Si vous voulez attendre que l'entrée soit terminée
- ✅ Recherche, autocomplétion
- ✅ Attendre que l'utilisateur arrête de taper.

### Notes.
- ⚠️ `auditTime` ne produit que la dernière valeur de la période (les valeurs intermédiaires sont rejetées).
- ⚠️ N'est pas très efficace si elle est définie pour des intervalles courts.
- ⚠️ `throttleTime` ou `debounceTime` peuvent être plus appropriés en fonction de l'application.

## 🚀 Prochaines étapes.

- **[throttleTime](. /throttleTime)** - apprendre à passer la première valeur.
- **[debounceTime](. /debounceTime)** - apprenez à émettre des valeurs après l'arrêt de la saisie.
- **[filter](. /filter)** - apprendre à filtrer en fonction de conditions.
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - Apprenez à utiliser des cas d'utilisation réels.
