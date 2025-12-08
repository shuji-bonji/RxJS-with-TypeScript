---
description: "fromEvent() - Fonction de création qui convertit les événements DOM et EventEmitter en flux Observable avec gestion automatique des auditeurs et prévention des fuites de mémoire"
---

# fromEvent() - Convertir les événements en Observable

`fromEvent()` est une fonction de création qui convertit les sources d'événements tels que les événements DOM et les EventEmitter de Node.js en flux Observable.

## Vue d'ensemble

`fromEvent()` permet aux pipelines RxJS de gérer des traitements asynchrones basés sur des événements. Elle enregistre automatiquement les auditeurs d'événements lors de l'abonnement et supprime automatiquement les auditeurs lors du désabonnement, ce qui réduit considérablement le risque de fuites de mémoire.

**Signature** :
```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

**Documentation officielle** : [📘 RxJS Official : fromEvent()](https://rxjs.dev/api/index/function/fromEvent)

## Utilisation de base

C'est l'exemple le plus simple pour traiter les événements DOM comme des Observable.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Bouton cliqué:', event);
});

// L'événement est émis à chaque clic
```

## Caractéristiques importantes

### 1. Enregistrement et suppression automatiques des auditeurs

`fromEvent()` enregistre un auditeur d'événement lors de l'abonnement et le supprime automatiquement lors du désabonnement.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent<MouseEvent>(document, 'click');

const subscription = clicks$.subscribe(event => {
  console.log('Position du clic:', event.clientX, event.clientY);
});

// Désabonnement après 5 secondes (l'auditeur d'événement est automatiquement supprimé)
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Désabonné');
}, 5000);
```

> [!IMPORTANT]
> **Prévention des fuites de mémoire**
>
> Lorsque `unsubscribe()` est appelé, `removeEventListener()` est automatiquement exécuté en interne. Cela élimine le besoin de supprimer manuellement les auditeurs et réduit considérablement le risque de fuites de mémoire.

### 2. Cold Observable (chaque abonnement enregistre un auditeur indépendant)

L'Observable créé par `fromEvent()` est un **Cold Observable**. Chaque abonnement enregistre un auditeur d'événement indépendant.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Abonnement 1 - Enregistre l'auditeur A
clicks$.subscribe(() => console.log('Observateur 1: Clic'));

// Ajout de l'abonnement 2 après 1 seconde - Enregistre l'auditeur B indépendamment
setTimeout(() => {
  clicks$.subscribe(() => console.log('Observateur 2: Clic'));
}, 1000);

// Les deux auditeurs se déclenchent sur un seul clic
// Cela prouve que chaque abonnement a un auditeur indépendant
```

> [!NOTE]
> **Preuve de Cold Observable**
>
> Un nouvel auditeur d'événement est enregistré à chaque abonnement et supprimé lors du désabonnement. C'est une caractéristique du Cold Observable. Cependant, puisque la source de l'événement (par exemple, un élément DOM) est externe et partagée, elle a également la propriété Hot de "ne pas recevoir d'événements avant l'abonnement".

### 3. Prise en charge des types TypeScript

Les types d'événements peuvent être explicitement spécifiés.

```typescript
import { fromEvent } from 'rxjs';

const input = document.createElement('input');
input.type = 'text';
document.body.appendChild(input);
const input$ = fromEvent<InputEvent>(input, 'input');

input$.subscribe(event => {
  // Le type de event est InputEvent
  const target = event.target as HTMLInputElement;
  console.log('Valeur saisie:', target.value);
});
```

### 4. Cold Observable

`fromEvent()` est un **Cold Observable**. Chaque abonnement initie une exécution indépendante.

```typescript
import { fromEvent } from 'rxjs';

const button = document.createElement('button');
button.innerText = "S'abonner";
document.body.appendChild(button);

const clicks$ = fromEvent(document, 'click');

// Premier abonnement - un auditeur d'événement est ajouté
clicks$.subscribe(() => console.log('Abonné A'));

// Deuxième abonnement - un autre auditeur d'événement est ajouté
clicks$.subscribe(() => console.log('Abonné B'));

// Les deux auditeurs se déclenchent lors d'un clic
// Sortie:
// Abonné A
// Abonné B
```

> [!NOTE]
> **Caractéristiques du Cold Observable** :
> - Une exécution indépendante est lancée pour chaque abonnement
> - Chaque abonné reçoit son propre flux de données
> - Un auditeur d'événement indépendant est enregistré pour chaque abonnement ; le désabonnement supprime automatiquement l'auditeur
>
> Voir [Cold Observable et Hot Observable](/fr/guide/observables/cold-and-hot-observables) pour plus d'informations.

## Cas d'utilisation pratiques

### 1. Traitement des événements de clic

Contrôler les clics sur les boutons et empêcher les clics consécutifs.

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const button = document.createElement('button');
button.innerText = "soumettre";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  debounceTime(300), // Ignorer les clics consécutifs dans les 300ms
  map(() => 'Envoi en cours...')
).subscribe(message => {
  console.log(message);
  // Traitement d'appel API, etc.
});
```

### 2. Validation des entrées de formulaire en temps réel

Diffuser les événements de saisie et effectuer la validation en temps réel.

```typescript
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'email: ';
const emailInput = document.createElement('input');
label.appendChild(emailInput);
document.body.appendChild(label);
const email$ = fromEvent<InputEvent>(emailInput, 'input');

email$.pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(500), // Traiter 500ms après l'arrêt de la saisie
  distinctUntilChanged() // Uniquement lorsque la valeur change
).subscribe(email => {
  console.log('Cible de validation:', email);
  // Traitement de validation d'email
  validateEmail(email);
});

function validateEmail(email: string): void {
  const isValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  console.log(isValid ? 'Adresse email valide' : 'Adresse email invalide');
}
```

### 3. Implémentation du glisser-déposer

Combiner les événements de souris pour implémenter le glisser-déposer.

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, takeUntil, map } from 'rxjs';

// Créer un élément déplaçable
const element = document.createElement('div');
element.style.width = '100px';
element.style.height = '100px';
element.style.backgroundColor = '#333';
element.style.position = 'absolute'; // Définir en positionnement absolu
element.style.left = '50px'; // Position initiale
element.style.top = '50px';
element.style.cursor = 'move'; // Curseur déplaçable
document.body.appendChild(element);

const mousedown$ = fromEvent<MouseEvent>(element, 'mousedown');
const mousemove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseup$ = fromEvent<MouseEvent>(document, 'mouseup');

mousedown$.pipe(
  switchMap(startEvent => {
    // Enregistrer la position du clic dans l'élément
    const startX = startEvent.clientX - element.offsetLeft;
    const startY = startEvent.clientY - element.offsetTop;

    return mousemove$.pipe(
      map(moveEvent => ({
        left: moveEvent.clientX - startX,
        top: moveEvent.clientY - startY
      })),
      takeUntil(mouseup$) // Terminer au relâchement de la souris
    );
  })
).subscribe(({ left, top }) => {
  // Mettre à jour la position de l'élément
  element.style.left = `${left}px`;
  element.style.top = `${top}px`;
});
```

### 4. Surveillance des événements de défilement

Utilisé pour suivre le défilement infini et la position de défilement.

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

const scroll$ = fromEvent(window, 'scroll');

scroll$.pipe(
  throttleTime(200), // Traiter seulement une fois toutes les 200ms
  map(() => window.scrollY)
).subscribe(scrollPosition => {
  console.log('Position de défilement:', scrollPosition);

  // Charger du contenu supplémentaire en atteignant le bas de la page
  if (scrollPosition + window.innerHeight >= document.body.scrollHeight - 100) {
    console.log('Charger du contenu supplémentaire');
    // loadMoreContent();
  }
});
```

## Utilisation dans un pipeline

`fromEvent()` est idéal pour le traitement en pipeline à partir de flux d'événements.

```typescript
import { fromEvent } from 'rxjs';
import { map, filter, scan } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Compteur";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  filter((event: Event) => {
    // Compter uniquement les clics en maintenant la touche Shift
    return (event as MouseEvent).shiftKey;
  }),
  scan((count, _) => count + 1, 0),
  map(count => `Nombre de clics: ${count}`)
).subscribe(message => console.log(message));
```

## Erreurs courantes

### 1. Oublier de se désabonner

#### ❌ Incorrect - Oublier de se désabonner provoque des fuites de mémoire

```typescript
import { fromEvent } from 'rxjs';

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  clicks$.subscribe(console.log); // Non désabonné !
}

setupEventListener();
```

#### ✅ Correct - Toujours se désabonner

```typescript
import { fromEvent } from 'rxjs';
import { Subscription } from 'rxjs';

let subscription: Subscription;

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  subscription = clicks$.subscribe(console.log);
}

function cleanup() {
  if (subscription) {
    subscription.unsubscribe();
  }
}

setupEventListener();
// Appeler cleanup() lors de la destruction du composant, etc.
```

> [!WARNING]
> **Attention aux fuites de mémoire**
>
> Dans les SPA et les frameworks basés sur des composants, assurez-vous de vous désabonner lorsque vous détruisez un composant. Si vous oubliez de vous désabonner, les auditeurs d'événements resteront en place et provoqueront des fuites de mémoire.

### 2. Enregistrement en double de plusieurs auditeurs d'événements

#### ❌ Incorrect - S'abonner plusieurs fois au même événement enregistre plusieurs auditeurs

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Observateur 1'));
clicks$.subscribe(() => console.log('Observateur 2'));
// Les deux logs s'affichent au clic (deux auditeurs sont enregistrés)
```

#### ✅ Correct - Multicast avec share() si nécessaire

```typescript
import { fromEvent } from 'rxjs';
import { share } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(share());

clicks$.subscribe(() => console.log('Observateur 1'));
clicks$.subscribe(() => console.log('Observateur 2'));
// Un seul auditeur est partagé
```

## Considérations sur les performances

Les performances doivent être prises en compte lors de la gestion d'événements qui se déclenchent à haute fréquence (scroll, mousemove, resize, etc.).

> [!TIP]
> **Optimisation des événements à haute fréquence** :
> - `throttleTime()` - Traiter seulement une fois par période de temps
> - `debounceTime()` - Traiter après l'arrêt de la saisie
> - `distinctUntilChanged()` - Traiter uniquement lorsque la valeur change

#### ❌ Problème de performance - Traitement à chaque redimensionnement

```typescript
import { fromEvent } from 'rxjs';

const resize$ = fromEvent(window, 'resize');

resize$.subscribe(() => {
  console.log('Traitement de redimensionnement'); // Traitement lourd
});
```

#### ✅ Optimisation - Traiter seulement une fois toutes les 200ms

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

const resize$ = fromEvent(window, 'resize');
resize$.pipe(
  throttleTime(200)
).subscribe(() => {
  console.log('Traitement de redimensionnement'); // Réduction de la charge
});
```

## Fonctions de création associées

| Fonction | Différence | Utilisation |
|----------|------|----------|
| **[from()](/fr/guide/creation-functions/basic/from)** | Convertir depuis un tableau/Promise | Streamer des données autres que des événements |
| **[interval()](/fr/guide/creation-functions/basic/interval)** | Émettre à intervalles réguliers | Traitement périodique nécessaire |
| **fromEventPattern()** | Enregistrement d'événements personnalisés | Systèmes d'événements personnalisés autres que EventEmitter |

## Résumé

- `fromEvent()` convertit les événements DOM et EventEmitter en Observable
- Enregistre les auditeurs lors de l'abonnement, les supprime automatiquement lors du désabonnement (évite les fuites de mémoire)
- Fonctionne comme un Cold Observable
- Toujours effectuer le désabonnement pour éviter les fuites de mémoire
- Optimiser les événements à haute fréquence avec `throttleTime()` et `debounceTime()`

## Prochaines étapes

- [interval() - Émission continue à intervalles spécifiés](/fr/guide/creation-functions/basic/interval)
- [timer() - Commencer à émettre après un délai](/fr/guide/creation-functions/basic/timer)
- [Retour aux fonctions de création de base](/fr/guide/creation-functions/basic/)
