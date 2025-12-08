---
description: "Ce cours explique comment utiliser fromEvent pour gérer les événements DOM en tant qu'Observable. Des exemples pratiques seront présentés, allant de la gestion des clics, des mouvements de souris et des événements clavier à l'implémentation de traitements complexes de l'interface utilisateur, tels que le glisser-déposer et la validation de formulaires."
---

# Les événements en flux continu

Cette section fournit une introduction complète à la création d'Observable dans RxJS, de la syntaxe de base aux applications pratiques.

## Gestion traditionnelle des événements vs. RxJS

### Les événements de clic
#### ◇ Gestion traditionnelle des événements du DOM

```ts
document.addEventListener('click', (event) => {
  console.log('Cliqué:', event);
});

// Résultat:
// Cliqué: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

#### ◆ Gestion d'événements avec RxJS

```ts
import { fromEvent } from 'rxjs';

// Flux d'événements de clic
const clicks$ = fromEvent(document, 'click');
clicks$.subscribe(event => console.log('Clic RxJS:', event));

// Résultat:
// Clic RxJS: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Événements liés au mouvement de la souris
#### ◇ Gestion traditionnelle des événements du DOM
```ts
document.addEventListener('mousemove', (event) => {
  console.log('Position de la souris:', event.clientX, event.clientY);
});

// Résultat:
// Position de la souris: 4 357
// Position de la souris: 879 148
// Position de la souris: 879 148
```

#### ◆ Gestion d'événements avec RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, throttleTime } from 'rxjs';

// Flux d'événements de mouvement de la souris (avec limitation)
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  throttleTime(100), // Limiter à toutes les 100 millisecondes
  map(event => ({ x: event.clientX, y: event.clientY }))
);
mouseMove$.subscribe(position => console.log('Position de la souris:', position));

// Résultat:
// Position de la souris: {x: 177, y: 453}
// Position de la souris: {x: 1239, y: 297}
```

### Événements clavier
#### ◇ Gestion traditionnelle des événements du DOM
```ts
document.addEventListener('keydown', (event) => {
  console.log('Touche appuyée:', event.key);
});

// Résultat:
// Touche appuyée: h
// Touche appuyée: o
// Touche appuyée: g
// Touche appuyée: e
```

#### ◆ Gestion d'événements avec RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Flux d'événements clavier
const keyDown$ = fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  filter(key => key.length === 1) // Un seul caractère uniquement
);
keyDown$.subscribe(key => console.log('Touche appuyée:', key));

// Résultat:
// Touche appuyée: h
// Touche appuyée: o
// Touche appuyée: g
// Touche appuyée: e
```

## Comment utiliser fromEvent et applications

`fromEvent` est la manière la plus commune de convertir les événements DOM en Observables. `fromEvent` est la fonction de conversion Event → Observable la plus basique et est le point de départ du traitement des événements avec RxJS.

### Utilisation de base
```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe((event) => console.log('Clic RxJS:', event));

// Résultat:
// Clic RxJS: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Spécification de la cible et du type d'événement
```ts
import { fromEvent } from 'rxjs';

const myButton = document.querySelector('#myButton')!;
const buttonClicks$ = fromEvent<MouseEvent>(myButton, 'click');
buttonClicks$.subscribe((event) => console.log('Clic myButton:', event));

// Résultat:
// Clic myButton: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Spécification des options (écoute en phase de capture)
```ts
import { fromEvent } from 'rxjs';

const capturedClicks$ = fromEvent(document, 'click', { capture: true });
capturedClicks$.subscribe((event) => console.log('Clic sur la page:', event));

// Résultat:
// Clic sur la page: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!NOTE]
> Il y a deux phases de propagation des événements DOM : "capture" et "bouillonnement".
> Normalement, c'est le "bouillonnement" (les événements se propagent de l'élément enfant à l'élément parent), mais si `capture: true` est spécifié, l'écoute se fait en "phase de capture" (propagation de l'élément parent à l'élément enfant).
> Cela permet aux événements d'être détectés par l'élément parent avant d'être traités par l'élément enfant.

## Gestion de plusieurs sources d'événements

RxJS permet de fusionner plusieurs sources d'événements dans une logique commune via `merge` ou `combineLatest`.

```ts
import { fromEvent, merge } from 'rxjs';
import { map } from 'rxjs';

// Intégrer les clics de plusieurs boutons
const button1Clicks$ = fromEvent(document.querySelector('#button1')!, 'click')
  .pipe(map(() => 'Le bouton 1 a été cliqué'));

const button2Clicks$ = fromEvent(document.querySelector('#button2')!, 'click')
  .pipe(map(() => 'Le bouton 2 a été cliqué'));

// Fusionner les deux flux d'événements
const allButtonClicks$ = merge(button1Clicks$, button2Clicks$);
allButtonClicks$.subscribe(message => console.log(message));
```

#### Résultat de l'exécution
```
Le bouton 1 a été cliqué
```
```
Le bouton 2 a été cliqué
```

## Convertir et manipuler les flux d'événements

L'avantage des flux d'événements est qu'ils peuvent être facilement convertis et manipulés à l'aide d'opérateurs RxJS.

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  debounceTime,
  distinctUntilChanged,
} from 'rxjs';

// Surveiller les changements de valeur du champ de saisie
const input$ = fromEvent<InputEvent>(
  document.querySelector('#searchInput')!,
  'input'
).pipe(
  map((event) => (event.target as HTMLInputElement).value),
  filter((text) => text.length > 2), // Traiter uniquement si 3 caractères ou plus
  debounceTime(300), // Intervalle de 300ms (ne se déclenche pas pendant la saisie)
  distinctUntilChanged() // Ne se déclenche pas si la valeur est identique à la précédente
);

input$.subscribe((searchText) => {
  console.log('Texte de recherche:', searchText);
  // Appeler l'API de recherche ici, etc.
});
```

#### Résultat de l'exécution
```sh
Texte de recherche: abc
Texte de recherche: abcd
```
Ainsi, en traitant les événements d'entrée et les autres événements comme des flux, la réactivité et la maintenabilité de l'interface utilisateur peuvent être grandement améliorées.

## Exemple de mise en œuvre du glisser-déposer

Pour illustrer l'utilisation d'une combinaison d'événements multiples, essayons de gérer les opérations de glissement de la souris avec Observable.

```ts
import { fromEvent } from 'rxjs';
import { map, switchMap, takeUntil, tap } from 'rxjs';

function implementDragAndDrop(element: HTMLElement) {
  // Flux d'événements mousedown
  const mouseDown$ = fromEvent<MouseEvent>(element, 'mousedown');

  // Flux d'événements mousemove sur le document
  const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');

  // Flux d'événements mouseup sur le document
  const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

  // Traitement du glissement
  const drag$ = mouseDown$.pipe(
    tap(event => {
      // Empêcher le traitement de glissement par défaut du navigateur
      event.preventDefault();
    }),
    switchMap(startEvent => {
      // Enregistrer la position initiale
      const initialX = startEvent.clientX;
      const initialY = startEvent.clientY;
      const elementX = parseInt(element.style.left || '0', 10);
      const elementY = parseInt(element.style.top || '0', 10);

      // Retourner le flux de mouvement de la souris (jusqu'au mouseUp)
      return mouseMove$.pipe(
        map(moveEvent => ({
          x: elementX + (moveEvent.clientX - initialX),
          y: elementY + (moveEvent.clientY - initialY)
        })),
        takeUntil(mouseUp$) // Terminer au relâchement de la souris
      );
    })
  );

  // S'abonner au flux de glissement
  drag$.subscribe(position => {
    element.style.left = `${position.x}px`;
    element.style.top = `${position.y}px`;
    console.log(`${position.x}px, ${position.y}px`);
  });
}

// Exemple d'utilisation
const draggableElement = document.querySelector('#draggable') as HTMLElement;
implementDragAndDrop(draggableElement);
```

#### Résultat de l'exécution
```
1px, 0px
1px, -1px
0px, -2px
0px, -3px
0px, -4px
```

## Observer et valider les entrées d'un formulaire

Les processus typiques de l'interface utilisateur tels que la validation des formulaires peuvent être écrits de manière plus déclarative et plus sûre avec Observable.

```ts
import { fromEvent, combineLatest } from 'rxjs';
import { map, startWith, debounceTime } from 'rxjs';

function validateForm() {
  // Références des champs de saisie
  const usernameInput = document.querySelector('#username') as HTMLInputElement;
  const passwordInput = document.querySelector('#password') as HTMLInputElement;
  const submitButton = document.querySelector('#submit') as HTMLButtonElement;

  // Flux de changement des champs de saisie
  const username$ = fromEvent<InputEvent>(usernameInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Valeur initiale
  );

  const password$ = fromEvent<InputEvent>(passwordInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Valeur initiale
  );

  // Valider en combinant les deux entrées
  const formValid$ = combineLatest([username$, password$]).pipe(
    debounceTime(300),
    map(([username, password]) => {
      return username.length >= 3 && password.length >= 6;
    })
  );

  // Basculer l'état activé/désactivé du bouton en fonction de l'état de validation du formulaire
  formValid$.subscribe(isValid => {
    submitButton.disabled = !isValid;
  });

  // Traitement de la soumission du formulaire
  const submit$ = fromEvent(submitButton, 'click');
  submit$.subscribe(() => {
    console.log('Soumission du formulaire:', {
      username: usernameInput.value,
      password: passwordInput.value
    });
    // Effectuer le traitement de soumission réel ici
  });
}

// Exemple d'utilisation
validateForm();
```
#### Résultat de l'exécution
```
Soumission du formulaire: {username: 'testuser', password: '123456'}
```

## Lien vers la liste des événements

Une liste de tous les événements et de leur disponibilité pour `fromEvent` est disponible sur le lien suivant :

➡️ **[Tableau de la liste des événements](./events-list.md)**

Cette liste est utile pour la programmation réactive avec RxJS, car elle indique clairement si les événements JavaScript standards sont supportés par `fromEvent`.

## Événements qui ne peuvent pas être utilisés avec fromEvent {#cannot-used-fromEvent}

`fromEvent` repose sur l'interface `EventTarget` du DOM. Par conséquent, les événements suivants ne peuvent pas être gérés directement par `fromEvent`. Ceux-ci sont liés à des objets spécifiques ou ont leurs propres écouteurs d'événements.

| Nom de l'événement | Type | Raison |
| ------------------ | --------------------- | ------------------------------------------------ |
| `beforeunload` | `BeforeUnloadEvent` | Événement exécuté avant la fermeture de la fenêtre, dépend du comportement du navigateur plutôt que des écouteurs d'événements du DOM |
| `unload` | `Event` | Lorsque la page est complètement fermée, les écouteurs sont également supprimés, donc invalide dans l'Observable RxJS |
| `message` | `MessageEvent` | Les messages provenant de ServiceWorker ou WebWorker ne peuvent pas être capturés directement avec `fromEvent` |
| `popstate` | `PopStateEvent` | Les modifications de `history.pushState` ou de `replaceState` nécessitent une manipulation manuelle |
| `storage` | `StorageEvent` | Les modifications de `localStorage` ne peuvent pas être surveillées avec `fromEvent` (il faut passer par `window.addEventListener`) |
| `languagechange` | `Event` | Les changements de paramètres du navigateur dépendent du comportement de l'objet `window` |
| `fetch` | `Event` | La progression de `fetch` (comme `onprogress`) n'est pas un événement DOM normal |
| `WebSocket` | `Event` | `onmessage`, `onopen`, `onclose` ont leurs propres écouteurs d'événements |
| `ServiceWorker` | `Event` | `message`, `install`, `activate`, etc. ne peuvent pas être gérés avec `fromEvent` |

### Méthodes alternatives
Si vous souhaitez surveiller ces événements, utilisez les méthodes suivantes :

- `window.addEventListener('message', callback)`
- `window.addEventListener('popstate', callback)`
- `window.addEventListener('storage', callback)`
- Pour `WebSocket`, `ws.addEventListener('message', callback)`
- Pour `ServiceWorker`, `navigator.serviceWorker.addEventListener('message', callback)`

Lorsque RxJS est utilisé, au lieu de `fromEvent`, vous pouvez générer manuellement un Observable comme suit :

```typescript
import { Observable } from 'rxjs';

const message$ = new Observable<MessageEvent>(observer => {
  const handler = (event: MessageEvent) => observer.next(event);
  window.addEventListener('message', handler);

  // Traitement de nettoyage
  return () => window.removeEventListener('message', handler);
});

message$.subscribe(event => {
  console.log('Message reçu:', event.data);
});
```

## Résumé et bonnes pratiques

Dans cet article, nous avons vu les avantages et les applications spécifiques permettant de rendre les événements Observable.

Le traitement des événements avec RxJS offre les avantages suivants :

- Une gestion déclarative et structurée des événements est possible
- Filtrage, transformation et report faciles des événements grâce à `pipe()` et aux opérateurs
- Intégration clairement exprimée de sources d'événements multiples et contrôle d'états complexes
- Gestion centralisée des effets secondaires via `subscribe`

### Meilleures pratiques

- `fromEvent` pour chaque composant de l'interface utilisateur devrait être correctement désabonné avec `unsubscribe` (utiliser `takeUntil`, etc.)
- Stabiliser les références DOM avec des contrôles de nullité et des `!` explicites
- Les flux doivent être divisés en petites parties, et il faut être conscient de l'utilisation de `switchMap` et `mergeMap`
- La combinaison avec la communication backend peut être contrôlée par `exhaustMap`, `concatMap`, etc.

Le streaming d'événements avec RxJS est plus qu'un simple traitement des clics et des touches, c'est **le concept de base de la construction d'une interface utilisateur réactive**.
