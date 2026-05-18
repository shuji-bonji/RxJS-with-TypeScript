---
description: "fromEvent() - Creation Function pour convertir les événements DOM et les EventEmitters en Observable, la base de la programmation événementielle, qui peuvent être utilisés pour traiter divers événements tels que les clics, les frappes, les mouvements de souris et le défilement. Explique les définitions des types TypeScript et comment spécifier les cibles des événements."
---

# fromEvent() - convertit un événement en Observable

fromEvent()` est une Creation Function qui convertit les sources d'événements tels que les événements DOM et les EventEmitter de Node.js en flux Observable.

## Vue d'ensemble.

FromEvent()` permet au traitement asynchrone basé sur les événements d'être géré dans le pipeline RxJS. Il enregistre automatiquement les auditeurs d'événements lorsqu'ils sont abonnés et supprime automatiquement les auditeurs lorsqu'ils sont désabonnés, réduisant ainsi de manière significative le risque de fuites de mémoire.

**Signature** :.

```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

**Documentation officielle** : [📘 RxJS formula : fromEvent()](https://rxjs.dev/api/index/function/fromEvent)

## Utilisation de base.

C'est l'exemple le plus simple pour traiter les événements DOM comme Observable.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Bouton cliqué.:', event);
});

// Un événement est émis à chaque fois qu'il est cliqué.
```

## Caractéristiques importantes.

### 1. enregistrement et désenregistrement automatique des auditeurs

La fonction `fromEvent()` enregistre les auditeurs d'événements lorsqu'ils sont abonnés et supprime automatiquement les auditeurs lorsqu'ils sont désabonnés.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent<MouseEvent>(document, 'click');

const subscription = clicks$.subscribe(event => {
  console.log('Position du clic:', event.clientX, event.clientY);
});

// 5Désabonné après quelques secondes (l'auditeur de l'événement est également supprimé automatiquement)
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Désabonné');
}, 5000);
```

```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

> **Prévention des fuites de mémoire**
>
> Lorsque vous appelez `unsubscribe()`, un `removeEventListener()` interne est automatiquement exécuté. Cela élimine le besoin de supprimer manuellement les écouteurs et réduit considérablement le risque de fuites de mémoire.

### 2. Observable à froid (chaque abonnement enregistre un écouteur indépendant).

L'Observable créé par `fromEvent()` est un **Cold Observable. Chaque abonnement enregistre un écouteur d'événement indépendant.


```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Abonné à1 - AuditeurAS'abonner à
clicks$.subscribe(() => console.log('Observer 1: Cliquer'));

// 1S'abonner en quelques secondes2Ajouter - AuditeurBS'enregistrer indépendamment de
setTimeout(() => {
  clicks$.subscribe(() => console.log('Observer 2: Cliquer'));
}, 1000);

// 1Les deux récepteurs se déclenchent en un seul clic
// Cela prouve que chaque abonnement a des récepteurs indépendants.
```

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Bouton cliqué.:', event);
});

// Un événement est émis à chaque fois qu'il est cliqué.
```

> **Preuve de froid Observable
>
> Un nouvel écouteur d'événements est enregistré chaque fois qu'un abonnement est effectué et supprimé lorsque l'abonnement est désabonné. Il s'agit d'une caractéristique de Cold Observable. Cependant, il possède également la propriété Hot selon laquelle "les événements ne peuvent pas être reçus avant l'abonnement" parce que la source de l'événement (par exemple, les éléments DOM) est externe et partagée.

### Prise en charge des types TypeScript

Les types d'événements peuvent être explicitement spécifiés.


```typescript
import { fromEvent } from 'rxjs';

const input = document.createElement('input');
input.type = 'text';
document.body.appendChild(input);
const input$ = fromEvent<InputEvent>(input, 'input');

input$.subscribe(event => {
  // eventLe type de laInputEvent
  const target = event.target as HTMLInputElement;
  console.log('Valeur d'entrée:', target.value);
});
```

### 4. Observable à froid.

Le `fromEvent()` est un **Cold Observable. Chaque abonnement initie une exécution indépendante.

```typescript
import { fromEvent } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Abonné à";
document.body.appendChild(button);

const clicks$ = fromEvent(document, 'click');

// 1Le deuxième abonnement - Des auditeurs d'événements sont ajoutés
clicks$.subscribe(() => console.log('AbonnementA'));

// 2Le deuxième abonnement - Un autre récepteur d'événements est ajouté
clicks$.subscribe(() => console.log('AbonnementB'));

// 1Un clic déclenche les deux récepteurs
// Sortie:
// AbonnementA
// AbonnementB
```

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Bouton cliqué.:', event);
});

// Un événement est émis à chaque fois qu'il est cliqué.
```

> **Caractéristiques Observables à froid**
> Chaque abonnement démarre une exécution indépendante
> Chaque abonné reçoit son propre flux de données

>
> > Pour plus d'informations, voir [Cold Observable et Hot Observable](/fr/guide/observables/cold-and-hot-observables).

## Cas d'utilisation pratiques.

### 1. gérer les événements de clic

Contrôler les clics sur les boutons et empêcher les clics consécutifs.


```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const button = document.createElement('button');
button.innerText = "submit";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  debounceTime(300), // 300msIgnorer les clics consécutifs à l'intérieur de
  map(() => 'pendant la transmission...')
).subscribe(message => {
  console.log(message);
  // APIAppels et autres processus
});
```

### 2. validation en temps réel des entrées de formulaire

Flux d'événements de saisie et validation en temps réel.

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
  debounceTime(500), // après l'arrêt de l'entrée.500msTraitement ultérieur
  distinctUntilChanged() // Uniquement lorsque la valeur change
).subscribe(email => {
  console.log('Soumis à validation:', email);
  // Processus de validation des adresses électroniques
  validateEmail(email);
});

function validateEmail(email: string): void {
  const isValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  console.log(isValid ? 'Adresse électronique valide' : 'Adresse électronique non valide');
}
```

### Mise en œuvre du glisser-déposer

Combinez les événements de la souris pour mettre en œuvre le glisser-déposer.

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, takeUntil, map } from 'rxjs';

// Créer des éléments déplaçables
const element = document.createElement('div');
element.style.width = '100px';
element.style.height = '100px';
element.style.backgroundColor = '#333';
element.style.position = 'absolute'; // Positionnement absolu
element.style.left = '50px'; // Position initiale
element.style.top = '50px';
element.style.cursor = 'move'; // Curseur glissant
document.body.appendChild(element);

const mousedown$ = fromEvent<MouseEvent>(element, 'mousedown');
const mousemove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseup$ = fromEvent<MouseEvent>(document, 'mouseup');

mousedown$.pipe(
  switchMap(startEvent => {
    // Enregistrement de la position du clic dans l'élément
    const startX = startEvent.clientX - element.offsetLeft;
    const startY = startEvent.clientY - element.offsetTop;

    return mousemove$.pipe(
      map(moveEvent => ({
        left: moveEvent.clientX - startX,
        top: moveEvent.clientY - startY
      })),
      takeUntil(mouseup$) // Se termine lorsque la souris est relevée
    );
  })
).subscribe(({ left, top }) => {
  // Mise à jour de la position de l'élément
  element.style.left = `${left}px`;
  element.style.top = `${top}px`;
});
```

### 4. surveiller les événements de défilement

Utilisé pour suivre le défilement infini et la position du défilement.

```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Utilisation dans le pipeline.

fromEvent()` est idéal pour le traitement en pipeline à partir d'un flux d'événements.


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Erreurs courantes.

### 1. oublier de se désinscrire

#### ❌ Erreur - l'oubli de se désinscrire provoque des fuites de mémoire


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

#### ✅ Correct - toujours se désabonner


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

> [!WARNING]

> **Attention fuites de mémoire**
>
> Dans les SPA et les frameworks basés sur des composants, il faut toujours se désinscrire lorsque l'on détruit un composant. Si vous oubliez de vous désinscrire, les auditeurs d'événements resteront en place et provoqueront des fuites de mémoire.

### 2. enregistrement en double de plusieurs auditeurs d'événements

#### ❌ Erreur - le fait de s'abonner plusieurs fois au même événement entraîne l'enregistrement de plusieurs auditeurs.


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

#### ✅ Correct - multicast avec share() si nécessaire.


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Considérations sur les performances.

Les performances doivent être prises en compte lorsqu'il s'agit d'événements de déclenchement à haute fréquence (défilement, déplacement de la souris, redimensionnement, etc.)

> [!TIP]

**Optimisation des événements à haute fréquence** :.
> - `throttleTime()` - ne traiter qu'une fois tous les un certain laps de temps.
> - `debounceTime()` - Traiter après l'arrêt de l'entrée.
> - `distinctUntilChanged()` - Ne traiter que lorsque la valeur change.

#### ❌ Problèmes de performance - traitement à chaque redimensionnement


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

#### ✅ Optimisation - traitement une fois toutes les 200 ms


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Fonctions de Creation Function pertinentes.


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Résumé

- fromEvent()` convertit les événements DOM et EventEmitter en Observable
- Listener enregistré lors de l'abonnement, automatiquement supprimé lors du désabonnement (évite les fuites de mémoire)
- Fonctionne comme un Observable chaud
- Toujours effectuer le désabonnement pour éviter les fuites de mémoire
- Optimise les événements à haute fréquence avec `throttleTime()` et `debounceTime()`.

## Prochaines étapes.

- [interval() - publier des valeurs à intervalles réguliers](/fr/guide/creation-functions/basic/interval)
- [timer() - commence à publier après un délai](/fr/guide/creation-functions/basic/timer)
- [Retour à l'aperçu des fonctions de création de base](/fr/guide/creation-functions/basic/)
