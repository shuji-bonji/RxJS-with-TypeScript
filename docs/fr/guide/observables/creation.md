---
description: "Ce cours fournit une explication complète de la façon de créer des Observables dans RxJS, depuis les fonctions de génération de base telles que of et from, jusqu'à la définition d'Observables personnalisés, la communication HTTP et le streaming d'événements, avec des exemples de code pratiques."
---

# Comment créer un Observable

Un Observable définit un "flux de données", et il existe une grande variété de façons d'en créer un.
RxJS fournit une variété de moyens pour créer des Observables personnalisés ou pour générer facilement des Observables à partir d'événements, de tableaux, de réponses HTTP, etc.

Cette section fournit un aperçu complet de la façon de créer des Observables dans RxJS, de la syntaxe de base aux applications pratiques.

## Classification des méthodes de création d'Observable

Voici une liste des principales méthodes de création par catégorie.

| Catégorie | Méthodes principales | Description |
|----------|----------|------|
| Création personnalisée | [`new Observable()`](#new-observable) | Grande flexibilité mais nécessite plus de code. Nettoyage manuel requis |
| Fonctions de création | [`of()`](#of), [`from()`](#from), [`fromEvent()`](#fromevent), [`interval()`](#interval-timer), [`timer()`](#interval-timer), [`ajax()`](#ajax), [`fromFetch()`](#fromfetch), [`scheduled()`](#scheduled) | Fonctions de génération de données, d'événements et basées sur le temps couramment utilisées |
| Fonctions de création spéciales | [`defer()`](#defer), [`range()`](#range), [`generate()`](#generate), [`iif()`](#iif) | Génération orientée contrôle, orientée boucle, commutation conditionnelle, etc. |
| Observables spéciaux | [`EMPTY`](#empty-never-throwerror), [`NEVER`](#empty-never-throwerror), [`throwError()`](#empty-never-throwerror) | Pour l'achèvement, l'absence d'action et l'émission d'erreurs |
| Famille Subject | [`Subject`](#subject-behaviorsubject), [`BehaviorSubject`](#subject-behaviorsubject) | Observable spécial qui fonctionne à la fois comme observateur et émetteur |
| Conversion de callback | [`bindCallback()`](#bindcallback), [`bindNodeCallback()`](#bindnodecallback) | Convertir les fonctions basées sur les callbacks en Observable |
| Contrôle des ressources | [`using()`](#using) | Effectuer le contrôle des ressources en même temps que l'abonnement à l'Observable |
| WebSocket | [`webSocket()`](#websocket) | Gérer la communication WebSocket en tant qu'Observable bidirectionnel |

## Création personnalisée

### new Observable()
[📘 Documentation officielle RxJS : Observable](https://rxjs.dev/api/index/class/Observable)

La méthode la plus basique est d'utiliser directement le constructeur `Observable`. Cette méthode est la plus flexible lorsque vous souhaitez définir une logique Observable personnalisée. Un contrôle fin du comportement est possible grâce aux appels explicites `next`, `error` et `complete`.

```ts
import { Observable } from 'rxjs';

const observable$ = new Observable<number>(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  setTimeout(() => {
    subscriber.next(4);
    subscriber.complete();
  }, 1000);
});

observable$.subscribe({
  next: value => console.log('Valeur:', value),
  error: err => console.error('Erreur:', err),
  complete: () => console.log('Terminé')
});
// Sortie:
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Valeur: 4
// Terminé
```

> [!CAUTION]
> Si vous utilisez `new Observable()`, vous devez écrire vous-même la libération explicite des ressources (processus de nettoyage).
> ```ts
> const obs$ = new Observable(subscriber => {
>   const id = setInterval(() => subscriber.next(Date.now()), 1000);
>   return () => {
>     clearInterval(id); // Libération explicite des ressources
>   };
> });
> ```
> D'un autre côté, les fonctions de création intégrées à RxJS telles que `fromEvent()` et `interval()` ont des processus de nettoyage appropriés à l'intérieur.
> ```ts
> const click$ = fromEvent(document, 'click');
> const timer$ = interval(1000);
> ```
> Elles utilisent `addEventListener` ou `setInterval` en interne et sont conçues pour que RxJS appelle automatiquement `removeEventListener` ou `clearInterval()` lors de `unsubscribe()`.
>
> Notez que même si le processus de nettoyage est implémenté à l'intérieur de RxJS, ce processus ne sera pas exécuté à moins que `unsubscribe()` ne soit appelé.
> ```ts
>  const subscription = observable$.subscribe({
>  // Omis...
>  });
>
>  subscription.unsubscribe(); // 👈
> ```
> - Quelle que soit la méthode que vous utilisez pour créer un Observable, assurez-vous de prendre l'habitude d'appeler `unsubscribe()` lorsque vous n'en avez plus besoin.
> - Oublier de se désabonner maintiendra les écouteurs d'événements et les minuteries en cours d'exécution, causant des fuites de mémoire et des effets secondaires inattendus.

## Fonctions de création

Pour une création d'Observable plus concise et spécifique à l'application, RxJS fournit des "Fonctions de création". Celles-ci peuvent être utilisées pour simplifier le code dans des cas d'utilisation répétés.

> [!NOTE]
> Dans la documentation officielle de RxJS, ces fonctions sont classées comme "Creation Functions".
> Auparavant (RxJS 5.x ~ 6), elles étaient appelées "opérateurs de création", mais depuis RxJS 7, "Fonctions de création" est le terme officiel.

### of()
[📘 Documentation officielle RxJS : of()](https://rxjs.dev/api/index/function/of)

La fonction de création d'Observable la plus simple qui émet plusieurs valeurs **une à la fois dans l'ordre**.

```ts
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Valeur:', value),
  error: err => console.error('Erreur:', err),
  complete: () => console.log('Terminé')
});
// Sortie: Valeur: 1, Valeur: 2, Valeur: 3, Valeur: 4, Valeur: 5, Terminé
```

> [!IMPORTANT]
> Différence entre `of()` et `from()`
> - `of([1, 2, 3])` → émet un seul tableau.
> - `from([1, 2, 3])` → émet les valeurs individuelles `1`, `2`, `3` dans l'ordre.
>
> Notez que ceci est souvent confondu.

> [!TIP]
> Pour une utilisation détaillée et des exemples pratiques, voir la [page détaillée de of()](/fr/guide/creation-functions/basic/of).

### from()
[📘 Documentation officielle RxJS : from()](https://rxjs.dev/api/index/function/from)

Génère un Observable à partir d'une **structure de données existante** telle qu'un tableau, une Promise ou un itérable.

```ts
import { from } from 'rxjs';

// Créer à partir d'un tableau
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Valeur du tableau:', value),
  error: err => console.error('Erreur:', err),
  complete: () => console.log('Terminé')
});

// Créer à partir d'une Promise
const promise$ = from(Promise.resolve('Résultat de la Promise'));
promise$.subscribe({
  next: value => console.log('Résultat de la Promise:', value),
  error: err => console.error('Erreur:', err),
  complete: () => console.log('Terminé')
});

// Créer à partir d'un itérable
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Valeur de l\'itérable:', value),
  error: err => console.error('Erreur:', err),
  complete: () => console.log('Terminé')
});

// Sortie:
// Valeur du tableau: 1
// Valeur du tableau: 2
// Valeur du tableau: 3
// Terminé
// Valeur de l'itérable: 1
// Valeur de l'itérable: 2
// Valeur de l'itérable: 3
// Terminé
// Résultat de la Promise: Résultat de la Promise
// Terminé
```

> [!TIP]
> Pour une utilisation détaillée et des exemples pratiques, voir la [page détaillée de from()](/fr/guide/creation-functions/basic/from).

### fromEvent()
[📘 Documentation officielle RxJS : fromEvent](https://rxjs.dev/api/index/function/fromEvent)

Fonction permettant de **gérer les sources d'événements** tels que les événements DOM en tant qu'Observable.

```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe({
  next: event => console.log('Événement de clic:', event),
  error: err => console.error('Erreur:', err),
  complete: () => console.log('Terminé')
});

// Sortie:
// Événement de clic: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!CAUTION]
> Notez les cibles d'événements prises en charge
> - `fromEvent()` prend en charge les éléments DOM du navigateur (implémentation EventTarget), Node.js EventEmitter et les cibles d'événements de type jQuery.
> - Plusieurs abonnements peuvent ajouter plusieurs écouteurs d'événements.

> 👉 Pour des exemples plus détaillés d'utilisation du flux d'événements, voir [Les événements en flux continu](../observables/events).

> [!TIP]
> Pour une utilisation détaillée et des exemples pratiques, voir la [page détaillée de fromEvent()](/fr/guide/creation-functions/basic/fromEvent).

### interval(), timer()
[📘 Documentation officielle RxJS : interval](https://rxjs.dev/api/index/function/interval), [📘 Documentation officielle RxJS : timer](https://rxjs.dev/api/index/function/timer)

Cette fonction est utilisée lorsque vous souhaitez émettre des valeurs en continu à intervalles réguliers ou lorsque vous avez besoin d'un **contrôle du temps**.

```ts
import { interval, timer } from 'rxjs';

// Émettre des valeurs toutes les secondes
const interval$ = interval(1000);
interval$.subscribe({
  next: value => console.log('Interval:', value),
  error: err => console.error('Erreur:', err),
  complete: () => console.log('Terminé')
});

// Démarrer après 3 secondes, puis émettre des valeurs toutes les secondes
const timer$ = timer(3000, 1000);
timer$.subscribe({
  next: value => console.log('Timer:', value),
  error: err => console.error('Erreur:', err),
  complete: () => console.log('Terminé')
});

// Sortie:
// Interval: 0
// Interval: 1
// Interval: 2
// Timer: 0
// Interval: 3
// Timer: 1
// Interval: 4
// Timer: 2
// .
// .
```
`interval()` et `timer()` sont fréquemment utilisées pour des traitements contrôlés dans le temps, particulièrement adaptées à l'animation, au polling et aux délais d'événements asynchrones.

> [!CAUTION]
> Notez qu'il s'agit d'un Cold Observable
> - `interval()` et `timer()` sont des Cold Observable et sont exécutés indépendamment pour chaque abonnement.
> - Vous pouvez envisager de les rendre Hot avec `share()` ou d'autres méthodes si nécessaire.
>
> Pour plus de détails, voir la section ["Cold Observable et Hot Observable"](./cold-and-hot-observables.md).

> [!TIP]
> Pour une utilisation détaillée et des exemples pratiques, voir la [page détaillée de interval()](/fr/guide/creation-functions/basic/interval) et la [page détaillée de timer()](/fr/guide/creation-functions/basic/timer).

### ajax()
[📘 Documentation officielle RxJS : ajax](https://rxjs.dev/api/ajax/ajax)

Fonction de **manipulation asynchrone** des résultats de la communication HTTP en tant qu'**Observable**.

```ts
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
api$.subscribe({
  next: response => console.log('Réponse API:', response),
  error: error => console.error('Erreur API:', error),
  complete: () => console.log('API terminé')
});

// Sortie:
// Réponse API: {userId: 1, id: 1, title: 'delectus aut autem', completed: false}
// API terminé
```

> [!NOTE]
> RxJS ajax utilise XMLHttpRequest en interne. D'autre part, RxJS a également un opérateur appelé fromFetch, qui utilise l'API Fetch pour effectuer des requêtes HTTP.

> [!TIP]
> Pour une utilisation détaillée et des exemples pratiques, voir la [page détaillée de ajax()](/fr/guide/creation-functions/http-communication/ajax). Pour une vue d'ensemble des fonctions de communication HTTP, voir [Fonctions de création de communication HTTP](/fr/guide/creation-functions/http-communication/).

### fromFetch()
[📘 Documentation officielle RxJS : fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` encapsule l'API Fetch et vous permet de traiter les requêtes HTTP comme des Observables.
Elle est similaire à `ajax()`, mais plus moderne et plus légère.

```ts
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs';

const api$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1');

api$.pipe(
  switchMap(response => response.json())
).subscribe({
  next: data => console.log('Données:', data),
  error: err => console.error('Erreur:', err),
  complete: () => console.log('Terminé')
});

// Sortie:
// Données: {completed: false, id: 1, title: "delectus aut autem", userId: 1}
// Terminé
```

> [!NOTE]
> Parce que `fromFetch()` utilise l'API Fetch, contrairement à `ajax()`, l'initialisation des paramètres de la requête et la conversion `.json()` des réponses doivent être faites manuellement.
> Une bonne gestion des erreurs et une vérification du statut HTTP sont également nécessaires.

> [!TIP]
> Pour une utilisation détaillée et des exemples pratiques, voir la [page détaillée de fromFetch()](/fr/guide/creation-functions/http-communication/fromFetch). Pour une vue d'ensemble des fonctions de communication HTTP, voir [Fonctions de création de communication HTTP](/fr/guide/creation-functions/http-communication/).

### scheduled()
[📘 Documentation officielle RxJS : scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` est une fonction qui vous permet de spécifier explicitement un planificateur pour les fonctions publiées telles que `of()` et `from()`.
Utilisez cette fonction lorsque vous souhaitez contrôler en détail le moment de l'exécution synchrone ou asynchrone.

```ts
import { scheduled, asyncScheduler } from 'rxjs';

const observable$ = scheduled([1, 2, 3], asyncScheduler);
observable$.subscribe({
  next: val => console.log('Valeur:', val),
  complete: () => console.log('Terminé')
});

// L'exécution est asynchrone
// Sortie:
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Terminé
```

> [!NOTE]
> `scheduled()` permet aux fonctions synchrones existantes (par exemple `of()`, `from()`) de fonctionner de manière asynchrone.
> C'est utile pour les tests et l'optimisation des performances de l'interface utilisateur lorsque le contrôle du traitement asynchrone est nécessaire.

> [!TIP]
> Pour une utilisation détaillée et des exemples pratiques, voir la [page détaillée de scheduled()](/fr/guide/creation-functions/control/scheduled). Pour une vue d'ensemble des fonctions de contrôle, voir [Fonctions de création de contrôle](/fr/guide/creation-functions/control/).

### defer()
[📘 Documentation officielle RxJS : defer](https://rxjs.dev/api/index/function/defer)

Elle est utilisée lorsque vous souhaitez **reporter la génération d'un Observable jusqu'au moment de l'abonnement**.

```ts
import { defer, of } from 'rxjs';

const random$ = defer(() => of(Math.random()));

random$.subscribe(value => console.log('1er:', value));
random$.subscribe(value => console.log('2ème:', value));

// Sortie:
// 1er: 0.123456789
// 2ème: 0.987654321
```

> [!NOTE]
> `defer()` est utile lorsque vous voulez créer un nouvel Observable à chaque abonnement. Vous pouvez réaliser une évaluation paresseuse.

> [!TIP]
> Pour une utilisation détaillée et des exemples pratiques, voir la [page détaillée de defer()](/fr/guide/creation-functions/conditional/defer).

### range()
[📘 Documentation officielle RxJS : range](https://rxjs.dev/api/index/function/range)

Génère une valeur entière continue dans l'intervalle spécifié en tant qu'Observable.

```ts
import { range } from 'rxjs';

const numbers$ = range(1, 5);
numbers$.subscribe({
  next: value => console.log('Nombre:', value),
  complete: () => console.log('Terminé')
});

// Sortie:
// Nombre: 1
// Nombre: 2
// Nombre: 3
// Nombre: 4
// Nombre: 5
// Terminé
```

> [!TIP]
> Pour une utilisation détaillée et des exemples pratiques, voir la [page détaillée de range()](/fr/guide/creation-functions/loop/range).

### generate()
[📘 Documentation officielle RxJS : generate](https://rxjs.dev/api/index/function/generate)

Génère un Observable comme une structure en boucle. Permet un contrôle précis des valeurs initiales, des conditions, des augmentations/diminutions et de la sortie des valeurs.

```ts
import { generate } from 'rxjs';

const fibonacci$ = generate({
  initialState: [0, 1],
  condition: ([, b]) => b < 100,
  iterate: ([a, b]) => [b, a + b],
  resultSelector: ([a]) => a
});

fibonacci$.subscribe({
  next: value => console.log('Fibonacci:', value),
  complete: () => console.log('Terminé')
});

// Sortie:
// Fibonacci: 0
// Fibonacci: 1
// Fibonacci: 1
// Fibonacci: 2
// Fibonacci: 3
// Fibonacci: 5
// Fibonacci: 8
// Fibonacci: 13
// Fibonacci: 21
// Fibonacci: 34
// Fibonacci: 55
// Fibonacci: 89
// Terminé
```

> [!TIP]
> Pour une utilisation détaillée et des exemples pratiques, voir la [page détaillée de generate()](/fr/guide/creation-functions/loop/generate).

### iif()
[📘 Documentation officielle RxJS : iif](https://rxjs.dev/api/index/function/iif)

Utilisez cette fonction lorsque vous voulez **changer d'Observable par branchement conditionnel**.

```ts
import { iif, of, EMPTY } from 'rxjs';

const condition = true;
const iif$ = iif(() => condition, of('La condition est vraie'), EMPTY);

iif$.subscribe({
  next: val => console.log('iif:', val),
  complete: () => console.log('Terminé')
});

// Sortie:
// iif: La condition est vraie
// Terminé
```

> [!NOTE]
> `iif()` peut changer dynamiquement l'Observable à retourner en fonction des conditions. C'est utile pour le contrôle de flux.

## Observables spéciaux

### EMPTY, NEVER, throwError()
[📘 Documentation officielle RxJS : EMPTY](https://rxjs.dev/api/index/const/EMPTY), [📘 Documentation officielle RxJS : NEVER](https://rxjs.dev/api/index/const/NEVER), [📘 Documentation officielle RxJS : throwError](https://rxjs.dev/api/index/function/throwError)

RxJS fournit également des Observables spéciaux qui sont utiles pour le contrôle de l'exécution, la gestion des exceptions et l'apprentissage.

```ts
import { EMPTY, throwError, NEVER } from 'rxjs';

// Observable qui se termine immédiatement
const empty$ = EMPTY;
empty$.subscribe({
  next: () => console.log('Ceci n\'est pas affiché'),
  complete: () => console.log('Se termine immédiatement')
});

// Observable qui émet une erreur
const error$ = throwError(() => new Error('Une erreur s\'est produite'));
error$.subscribe({
  next: () => console.log('Ceci n\'est pas affiché'),
  error: err => console.error('Erreur:', err.message),
  complete: () => console.log('Terminé')
});

// Observable qui n'émet rien et ne se termine pas
const never$ = NEVER;
never$.subscribe({
  next: () => console.log('Ceci n\'est pas affiché'),
  complete: () => console.log('Ceci non plus n\'est pas affiché')
});

// Sortie:
// Se termine immédiatement
// Erreur: Une erreur s'est produite
```

> [!IMPORTANT]
> Principalement à des fins de contrôle, de vérification et d'apprentissage
> - `EMPTY`, `NEVER` et `throwError()` sont utilisés pour **le contrôle de flux, la validation de la gestion des exceptions**, ou à des fins d'apprentissage, et non pour les flux de données normaux.

## Famille Subject

### Subject, BehaviorSubject, etc. {#subject-behaviorsubject}
[📘 Documentation officielle RxJS : Subject](https://rxjs.dev/api/index/class/Subject), [📘 Documentation officielle RxJS : BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Observable pouvant émettre sa propre valeur, adapté au **multicast et au partage d'état**.

```ts
import { Subject } from 'rxjs';

const subject$ = new Subject<number>();

// Utiliser comme Observer
subject$.subscribe(value => console.log('Observer 1:', value));
subject$.subscribe(value => console.log('Observer 2:', value));

// Utiliser comme Observable
subject$.next(1);
subject$.next(2);
subject$.next(3);

// Sortie:
// Observer 1: 1
// Observer 2: 1
// Observer 1: 2
// Observer 2: 2
// Observer 1: 3
// Observer 2: 3
```

> [!IMPORTANT]
> Subject possède les propriétés d'Observable et d'Observer. Plusieurs abonnés peuvent partager le même flux de données (multicast).

> [!TIP]
> Pour plus de détails sur les différents types de Subject (BehaviorSubject, ReplaySubject, AsyncSubject), voir [Subject et Multicast](/fr/guide/subjects/what-is-subject).

## Conversion de callback

### bindCallback()
[📘 Documentation officielle RxJS : bindCallback](https://rxjs.dev/api/index/function/bindCallback)

Une fonction qui permet aux fonctions asynchrones basées sur les callbacks d'être traitées comme des Observables.

```ts
import { bindCallback } from 'rxjs';

// Fonction basée sur callback (style legacy)
function asyncFunction(value: number, callback: (result: number) => void) {
  setTimeout(() => callback(value * 2), 1000);
}

// Convertir en Observable
const asyncFunction$ = bindCallback(asyncFunction);
const observable$ = asyncFunction$(5);

observable$.subscribe({
  next: result => console.log('Résultat:', result),
  complete: () => console.log('Terminé')
});

// Sortie:
// Résultat: 10
// Terminé
```

> [!TIP]
> `bindCallback()` est utile pour convertir les anciennes API basées sur les callbacks en Observable.

### bindNodeCallback()
[📘 Documentation officielle RxJS : bindNodeCallback](https://rxjs.dev/api/index/function/bindNodeCallback)

Une fonction spécialisée pour convertir les fonctions basées sur les callbacks dans le style Node.js (callback error-first) en Observable.

```ts
import { bindNodeCallback } from 'rxjs';

// Fonction callback style Node.js (callback error-first)
function readFile(path: string, callback: (err: Error | null, data: string) => void) {
  if (path === 'valid.txt') {
    callback(null, 'contenu du fichier');
  } else {
    callback(new Error('Fichier non trouvé'), '');
  }
}

// Convertir en Observable
const readFile$ = bindNodeCallback(readFile);

readFile$('valid.txt').subscribe({
  next: data => console.log('Données:', data),
  error: err => console.error('Erreur:', err.message),
  complete: () => console.log('Terminé')
});

// Sortie:
// Données: contenu du fichier
// Terminé
```

### Différence entre bindCallback() et bindNodeCallback()

#### Exemple : Cible de bindCallback()

```ts
// Callback général (succès uniquement)
function getData(cb: (data: string) => void) {
  cb('succès');
}
```
→ Utilisez bindCallback() pour les callbacks simples "ne renvoyant qu'une valeur".

#### Exemple : Cible de bindNodeCallback() (style Node.js)

```ts
// Callback error-first
function readFile(path: string, cb: (err: Error | null, data: string) => void) {
  if (path === 'valid.txt') cb(null, 'contenu du fichier');
  else cb(new Error('non trouvé'), '');
}
```
→ Si vous utilisez bindNodeCallback(), les erreurs seront notifiées comme des erreurs Observable.

> [!NOTE]
> Comment utiliser
> - bindNodeCallback() si le premier argument du callback est "erreur ou non"
> - bindCallback() pour un callback simple "renvoyant uniquement une valeur"

## Contrôle des ressources

### using()
[📘 Documentation officielle RxJS : using](https://rxjs.dev/api/index/function/using)

`using()` permet d'associer la création et la libération des ressources au cycle de vie de l'Observable.
Elle est utile en combinaison avec les **processus qui nécessitent un nettoyage manuel**, tels que les WebSockets, les écouteurs d'événements et les ressources externes.

```ts
import { using, interval, Subscription } from 'rxjs';

const resource$ = using(
  () => new Subscription(() => console.log('Ressource libérée')),
  () => interval(1000)
);

const sub = resource$.subscribe(value => console.log('Valeur:', value));

// Se désabonner après quelques secondes
setTimeout(() => sub.unsubscribe(), 3500);

// Sortie:
// Valeur: 0
// Valeur: 1
// Valeur: 2
// Ressource libérée
```

> [!IMPORTANT]
> `using()` est utile pour faire correspondre la portée d'une ressource avec l'abonnement de l'Observable.
> Un processus de nettoyage explicite est automatiquement appelé lorsque `unsubscribe()` est exécuté.

> [!TIP]
> Pour une utilisation détaillée et des exemples pratiques, voir la [page détaillée de using()](/fr/guide/creation-functions/control/using). Pour une vue d'ensemble des fonctions de contrôle, voir [Fonctions de création de contrôle](/fr/guide/creation-functions/control/).

## WebSocket()
[📘 Documentation officielle RxJS : webSocket](https://rxjs.dev/api/webSocket/webSocket)

Le module `rxjs/webSocket` de RxJS fournit une fonction `webSocket()` qui permet de traiter WebSocket comme un Observable/Observer.

```ts
import { webSocket } from 'rxjs/webSocket';

const socket$ = webSocket('wss://echo.websocket.org');

socket$.subscribe({
  next: msg => console.log('Reçu:', msg),
  error: err => console.error('Erreur:', err),
  complete: () => console.log('Terminé')
});

// Envoyer un message (en tant qu'Observer)
socket$.next('Hello WebSocket!');
```

> [!IMPORTANT]
> `webSocket()` est un hybride Observable/Observer qui permet une communication bidirectionnelle.
> Il est utile pour la communication en temps réel car les connexions WebSocket, l'envoi et la réception peuvent être facilement gérés en tant qu'Observable.

## Résumé

Il existe une grande variété de façons de créer des Observables dans RxJS, et il est important de choisir la méthode appropriée pour votre application.

- Si vous avez besoin d'un traitement personnalisé, utilisez `new Observable()`
- `of()`, `from()`, `fromEvent()`, etc. pour traiter les données et les événements existants
- `ajax()` ou `fromFetch()` pour la communication HTTP
- La famille `Subject` pour le partage des données entre plusieurs abonnés

En les utilisant de manière appropriée, vous pouvez tirer pleinement parti de la flexibilité de RxJS.
