---
description: "Comprenez les différences entre Promise et RxJS et apprenez à les utiliser de manière appropriée. Promise est spécialisé dans le traitement asynchrone unique et s'exécute immédiatement, tandis que RxJS est évalué paresseusement, peut gérer plusieurs valeurs et peut être annulé et réutilisé. Ce guide explique en détail les caractéristiques de chacun et les critères de sélection à l'aide de comparaisons de code et de cas d'utilisation spécifiques."
---

# Différences entre Promise et RxJS

## Vue d'ensemble

Les principaux outils de traitement asynchrone en JavaScript/TypeScript sont **Promise** et **RxJS (Observable)**. Bien que ces deux outils soient parfois utilisés à des fins similaires, leur philosophie de conception et leurs cas d'utilisation sont très différents.

Cette page fournit des informations qui vous aideront à comprendre les différences entre Promise et RxJS et à décider lequel utiliser.

## Différences fondamentales

| Élément | Promise | RxJS (Observable) |
|------|---------|-------------------|
| **Standardisation** | Standard JavaScript (ES6/ES2015) | Bibliothèque tierce |
| **Valeurs émises** | Une seule valeur | Zéro ou plusieurs valeurs |
| **Évaluation** | Avide (exécute immédiatement à la création) | Paresseux (exécute à l'abonnement) |
| **Annulation** | Pas possible[^1] | Possible (`unsubscribe()`) |
| **Réutilisabilité** | Pas possible (le résultat n'est obtenu qu'une seule fois) | Possible (on peut s'abonner plusieurs fois) |
| **Coût d'apprentissage** | Faible | Élevé (nécessite la compréhension des opérateurs) |
| **Cas d'utilisation** | Traitement asynchrone simple | Traitement de flux complexe |

[^1]: Bien que les traitements basés sur Promise (comme fetch) peuvent être annulés à l'aide d'AbortController, la spécification Promise elle-même ne dispose pas d'une fonction d'annulation.

## Comparaison de code : Traitement asynchrone simple

### Promise

```ts
// Promise s'exécute immédiatement après sa création (Eager)
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json())
  .then(data => console.log(data))
  .catch(error => console.error(error));
```

Promise **commence à s'exécuter dès qu'elle est définie** (évaluation Eager).

### RxJS

```ts
import { from } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

// Observable ne s'exécute pas tant qu'il n'est pas souscrit (Lazy)
const observable$ = from(fetch('https://jsonplaceholder.typicode.com/posts/1')).pipe(
  switchMap(response => response.json()), // response.json() renvoie une Promise, donc utiliser switchMap
  catchError(error => {
    console.error(error);
    return of(null);
  })
);

// L'exécution ne commence que lorsque l'on est abonné
observable$.subscribe(data => console.log(data));
```

RxJS **ne s'exécute pas tant que `subscribe()` n'est pas appelé** (Lazy evaluation). S'abonner au même Observable plusieurs fois résulte en des exécutions indépendantes, et le traitement peut être interrompu avec `unsubscribe()`.

> [!TIP]
> **Conseils pratiques d'utilisation**
> - Traitement immédiat et ponctuel → Promise
> - Traitement à exécuter à un moment précis ou plusieurs fois → RxJS

## Comparaison de code : Traitement des valeurs multiples

L'une des plus grandes différences entre Promise et RxJS est le nombre de valeurs qui peuvent être émises. Promise ne peut renvoyer qu'une seule valeur, alors que RxJS peut émettre plusieurs valeurs au fil du temps.

### Impossible avec Promise

Promise ne peut **résoudre qu'une seule fois**.

```ts
// Promise ne peut renvoyer qu'une seule valeur
const promise = new Promise(resolve => {
  resolve(1);
  resolve(2); // Cette valeur est ignorée
  resolve(3); // Cette valeur est également ignorée
});

promise.then(value => console.log(value));
// Sortie : 1 (seulement la première valeur)
```

Une fois que la valeur est déterminée par le premier `resolve()`, les appels `resolve()` suivants sont ignorés.

### Possible avec RxJS

Observable **peut émettre des valeurs un nombre illimité de fois**.
```ts
import { Observable } from 'rxjs';

// Observable peut émettre plusieurs valeurs
const observable$ = new Observable(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  subscriber.complete();
});

observable$.subscribe(value => console.log(value));
// Sortie : 1, 2, 3
```

Chaque fois que `next()` est appelé, la valeur est délivrée à l'abonné. Après l'émission de toutes les valeurs, l'achèvement est notifié par `complete()`. Cette caractéristique permet de traiter naturellement des séries temporelles de données changeantes telles que la communication en temps réel, les données en continu et le traitement d'événements continus.

> [!NOTE]
> **Exemples d'applications pratiques**
> - Réception de messages WebSocket
> - Traitement séquentiel des entrées clavier
> - Flux d'événements du serveur (SSE)
> - Surveillance continue des données des capteurs

## Comparaison des annulations

La possibilité d'annuler un traitement asynchrone long ou inutile est importante du point de vue de la gestion des ressources et de l'expérience utilisateur. Il existe des différences significatives dans les capacités d'annulation entre Promise et RxJS.

### Promise (non annulable)
Promise n'a **aucune fonction d'annulation standard**.

```ts
const promise = new Promise(resolve => {
  setTimeout(() => resolve('Terminé'), 3000);
});

promise.then(result => console.log(result));
// Il n'y a pas de moyen standard d'annuler ce traitement
```

Une fois que l'exécution commence, elle ne peut pas être arrêtée avant la fin, ce qui peut entraîner des fuites de mémoire et une dégradation des performances.

> [!WARNING]
> **À propos d'AbortController**
> Les API Web telles que `fetch()` peuvent être annulées en utilisant `AbortController`, mais il ne s'agit pas d'une fonctionnalité de Promise elle-même, mais d'un mécanisme fourni par les API individuelles. Il n'est pas disponible pour tous les traitements asynchrones.

### RxJS (Annulable)

RxJS **peut être annulé à tout moment avec `unsubscribe()`**.
```ts
import { timer } from 'rxjs';

const subscription = timer(3000).subscribe(
  () => console.log('Terminé')
);

// Annuler après 1 seconde
setTimeout(() => {
  subscription.unsubscribe(); // Annuler
  console.log('Annulé');
}, 1000);
// Sortie : Annulé ("Terminé" n'est pas affiché)
```

Le désabonnement interrompt immédiatement le traitement en cours et évite les fuites de mémoire.

> [!TIP]
> **Cas pratiques d'annulation**
> - Annuler les requêtes HTTP lorsque l'utilisateur quitte l'écran
> - Rejeter les anciens résultats de la recherche et ne traiter que la dernière requête (`switchMap`)
> - Annuler automatiquement tous les Observables lorsque le composant est détruit (modèle `takeUntil`)

## Lequel choisir ?

Le choix entre Promise et RxJS dépend de la nature du traitement et des exigences du projet. Utilisez les critères suivants comme référence pour sélectionner l'outil approprié.

### Quand choisir Promise

Promise convient si les conditions suivantes sont réunies.

| Condition | Raison |
|------|------|
| Traitement asynchrone simple | Une requête API, une lecture de fichier, etc. |
| Flux de travail simple | `Promise.all`, `Promise.race` sont suffisants |
| Projets à petite échelle | Vouloir minimiser les dépendances |
| Utiliser uniquement l'API standard | Vouloir éviter les bibliothèques externes |
| Code convivial pour les débutants | Vouloir réduire les coûts d'apprentissage |

#### Requête API unique :


```ts
interface User {
  id: number;
  name: string;
  email: string;
  username: string;
}

async function getUserData(userId: string): Promise<User> {
  const response = await fetch(`https://jsonplaceholder.typicode.com/users/${userId}`);
  if (!response.ok) {
    throw new Error('Échec de la récupération des données utilisateur');
  }
  return response.json();
}

// Exemple d'utilisation
getUserData('1').then(user => {
  console.log('Nom d\'utilisateur:', user.name);
  console.log('Email:', user.email);
});
```

Ce code est un modèle typique pour récupérer les informations d'un seul utilisateur. L'utilisation de `async/await` le rend aussi lisible qu'un code synchrone. La gestion des erreurs peut également être unifiée avec `try/catch`, ce qui la rend simple et intuitive.

#### Exécution parallèle de plusieurs processus asynchrones :

```ts
interface Post {
  id: number;
  userId: number;
  title: string;
  body: string;
}

async function loadAllData(): Promise<[User[], Post[]]> {
  const [users, posts] = await Promise.all([
    fetch('https://jsonplaceholder.typicode.com/users').then(r => r.json()),
    fetch('https://jsonplaceholder.typicode.com/posts').then(r => r.json())
  ]);
  return [users, posts];
}

// Exemple d'utilisation
loadAllData().then(([users, posts]) => {
  console.log('Nombre d\'utilisateurs:', users.length);
  console.log('Nombre de posts:', posts.length);
});
```

`Promise.all()` vous permet d'exécuter plusieurs requêtes API en parallèle et d'attendre qu'elles soient toutes terminées. C'est très pratique pour le chargement initial des données. Notez que si l'une d'entre elles échoue, c'est tout le processus qui s'interrompt, mais sa simplicité le rend facile à comprendre et à maintenir.

### Quand choisir RxJS

RxJS convient si les conditions suivantes sont réunies.

| Condition | Raison |
|------|------|
| Traitement d'événements en continu | Mouvement de la souris, saisie au clavier, WebSocket, etc. |
| Traitement de flux complexes | Combinaison et transformation de sources d'événements multiples |
| Annulation requise | Vouloir contrôler finement la gestion des ressources |
| Retry/Timeout | Souhait d'une gestion flexible des erreurs |
| Projets Angular | RxJS est intégré dans le framework |
| Données en temps réel | Les données sont mises à jour en continu |

#### Exemple concret
```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'recherche: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);

// Recherche en temps réel (autocomplétion)
if (!searchInput) throw new Error('Entrée de recherche introuvable');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),              // Attendre 300ms avant de traiter
  distinctUntilChanged(),         // Traitement uniquement en cas de changement de valeur
  switchMap(query =>              // Exécuter uniquement la dernière requête
    fetch(`https://api.github.com/search/users?q=${query}`).then(r => r.json())
  )
).subscribe(results => {
  console.log('Résultats de la recherche:', results.items); // L'API GitHub stocke les résultats dans la propriété items
});
```

Cet exemple est un cas typique où RxJS montre sa vraie valeur. Il surveille les entrées de l'utilisateur, fournit un temps d'attente de 300ms pour réduire les requêtes inutiles, ne traite que lorsque la valeur change, et en ne rendant valide que la dernière requête (`switchMap`), il rejette automatiquement les résultats des anciennes requêtes.

> [!IMPORTANT]
> **Pourquoi c'est difficile avec Promise seul**
> - Doit implémenter manuellement le debounce (contrôle continu des entrées)
> - Il faut gérer soi-même l'annulation des anciennes requêtes
> - Oublier de nettoyer les écouteurs d'événements provoque des fuites de mémoire
> - Doit suivre plusieurs états simultanément (timers, drapeaux, gestion des requêtes)
>
> Avec RxJS, tous ces éléments peuvent être réalisés de manière déclarative en quelques lignes seulement.

## Interopérabilité entre Promise et RxJS

Promise et RxJS ne s'excluent pas mutuellement et peuvent être convertis l'un à l'autre et combinés. Ceci est utile pour intégrer du code existant basé sur Promise dans des pipelines RxJS, ou inversement lorsque vous voulez utiliser Observable dans du code existant basé sur Promise.

## Convertir une Promise en Observable

RxJS fournit plusieurs façons de convertir une Promise existante en Observable.

### Conversion par `from`

La méthode la plus courante est d'utiliser `from`.

```ts
import { from } from 'rxjs';

// Créer une Promise
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json());

// Convertir en Observable avec from()
const observable$ = from(promise);

observable$.subscribe({
  next: data => console.log('Données:', data),
  error: error => console.error('Erreur:', error),
  complete: () => console.log('Terminé')
});
```

Le résultat de la Promise s'écoule en tant qu'Observable, et l'achèvement est également appelé automatiquement.

### Conversion par `defer` (évaluation paresseuse)

Le `defer` retarde la création d'une Promise jusqu'à ce qu'elle soit souscrite.

```ts
import { defer } from 'rxjs';

// La Promise n'est pas créée avant subscribe
const observable$ = defer(() =>
  fetch('https://jsonplaceholder.typicode.com/posts/1').then(r => r.json())
);

// Création d'une nouvelle Promise à chaque subscribe
observable$.subscribe(data => console.log('1er:', data));
observable$.subscribe(data => console.log('2e:', data));
```

Cette méthode est utile si vous souhaitez créer une nouvelle Promise à chaque fois que vous vous abonnez.

## Convertir un Observable en Promise

Il est possible de prendre une seule valeur d'un Observable et de la transformer en Promise.

### `firstValueFrom` et `lastValueFrom`

Les deux fonctions suivantes sont recommandées dans RxJS 7 et plus.

| Fonction | Comportement |
|------|------|
| `firstValueFrom` | Retourne la première valeur sous forme de Promise |
| `lastValueFrom` | Retourne la dernière valeur à l'achèvement sous forme de Promise |

```ts
import { of, firstValueFrom, lastValueFrom } from 'rxjs';
import { delay } from 'rxjs';

const observable$ = of(1, 2, 3).pipe(delay(1000));

// Obtenir la première valeur sous forme de Promise
const firstValue = await firstValueFrom(observable$);
console.log(firstValue); // 1

// Obtenir la dernière valeur sous forme de Promise
const lastValue = await lastValueFrom(observable$);
console.log(lastValue); // 3
```

Si l'Observable se termine avant que la valeur ne circule, la valeur par défaut est une erreur. Cela peut être évité en spécifiant une valeur par défaut.

> [!WARNING]
> `toPromise()` est obsolète. Utilisez `firstValueFrom()` ou `lastValueFrom()` à la place.

> [!TIP]
> **Conseils de sélection**
> - **`firstValueFrom()`** : Lorsque seule la première valeur est nécessaire (par exemple, le résultat de l'authentification)
> - **`lastValueFrom()`** : Lorsque le résultat final après traitement de toutes les données est nécessaire (par exemple, le résultat de l'agrégation)

## Exemple pratique : Combiner les deux

Dans le développement d'applications réelles, Promise et RxJS sont souvent combinés.

> [!WARNING] Précautions pratiques
> Mélanger Promise et Observable peut facilement **tomber dans des anti-modèles si les limites de la conception ne sont pas claires**.
>
> **Problèmes courants :**
> - Devient impossible à annuler
> - Séparation de la gestion des erreurs
> - `await` dans `subscribe` (particulièrement dangereux)
> - Acquisition parallèle des mêmes données avec Promise et Observable
>
> Voir **[Chapitre 10 : Anti-modèles de mélange Promise et Observable](/fr/guide/anti-patterns/promise-observable-mixing)** pour plus de détails.

### Soumission de formulaire et appels à l'API

Exemple de capture d'un événement de soumission de formulaire d'un utilisateur dans RxJS et d'envoi au serveur en utilisant l'API Fetch (Promise).

```ts
import { fromEvent, from } from 'rxjs';
import { exhaustMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface FormData {
  username: string;
  email: string;
}

// Soumission de formulaire basée sur Promise
async function submitForm(data: FormData): Promise<{ success: boolean }> {
  const response = await fetch('https://api.example.com/submit', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  });
  if (!response.ok) {
    throw new Error('Échec de la soumission');
  }
  return response.json();
}

// Gestion du flux d'événements avec RxJS
const submitButton = document.createElement('button');
submitButton.id = 'submit-button';
submitButton.innerText = 'Soumettre';
submitButton.style.padding = '10px 20px';
submitButton.style.margin = '10px';
document.body.appendChild(submitButton);
if (!submitButton) throw new Error('Bouton de soumission introuvable');

fromEvent(submitButton, 'click').pipe(
  exhaustMap(() => {
    const formData: FormData = {
      username: 'testuser',
      email: 'test@example.com'
    };
    // Convertir la fonction Promise en Observable
    return from(submitForm(formData));
  }),
  catchError(error => {
    console.error('Erreur de soumission:', error);
    return of({ success: false });
  })
).subscribe(result => {
  if (result.success) {
    console.log('Soumission réussie');
  } else {
    console.log('Échec de la soumission');
  }
});
```

Chaque fois que l'on clique sur le bouton de soumission du formulaire, un nouveau processus de soumission est lancé, mais **ignore les nouvelles soumissions pendant la soumission**.

Dans cet exemple, l'utilisation de `exhaustMap` permet d'éviter les requêtes en double lors de la transmission.

### Autocomplétion de recherche

Exemple de surveillance des changements de valeur d'un formulaire de saisie et d'exécution de recherches API.

```ts
import { fromEvent, from } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface SearchResult {
  items: Array<{
    login: string;
    id: number;
    avatar_url: string;
  }>;
  total_count: number;
}

// Fonction API basée sur Promise
async function searchAPI(query: string): Promise<SearchResult> {
  const response = await fetch(`https://api.github.com/search/users?q=${query}`);
  if (!response.ok) {
    throw new Error('Échec de la recherche');
  }
  return response.json();
}

// Gestion des flux d'événements avec RxJS
const label = document.createElement('label');
label.innerText = 'recherche: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);
if (!searchInput) throw new Error('Entrée de recherche introuvable');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),
  distinctUntilChanged(),
  switchMap(query => {
    // Convertir la fonction Promise en Observable
    return from(searchAPI(query));
  }),
  catchError(error => {
    console.error(error);
    return of({ items: [], total_count: 0 }); // Retourner un résultat vide en cas d'erreur
  })
).subscribe(result => {
  console.log('Résultats de la recherche:', result.items);
  console.log('Total:', result.total_count);
});
```

Dans cet exemple, les contrôles suivants sont réalisés :

- Attendre 300ms pour la fin de la saisie avec `debounceTime(300)`
- `distinctUntilChanged()` pour ignorer si la valeur est la même que la précédente
- `switchMap` pour récupérer uniquement les derniers résultats de recherche (les anciennes requêtes sont automatiquement annulées)

> [!WARNING] Attention aux anti-modèles
> Le schéma consistant à souscrire un Observable dans une Promise peut provoquer des fuites de mémoire et des comportements inattendus.
> <!-- TODO: Add link to subscribe-in-promise anti-pattern when available -->

> [!TIP]
> **Conception par séparation des responsabilités**
>
> - **RxJS** : En charge du contrôle des événements (debounce, switchMap, etc.)
> - **Promise** : En charge des requêtes HTTP (async/await)
> - **`from()`** : Pont entre les deux
>
> L'utilisation appropriée de chaque technologie améliore la lisibilité et la maintenabilité du code.

## Avantages et inconvénients

Chaque technologie a ses avantages et ses inconvénients.

### Promise
<div class="comparison-cards">

::: tip Avantages
- Aucune dépendance nécessaire car il s'agit d'un standard JavaScript
- Code intuitif et lisible avec `async/await`
- Faible coût d'apprentissage
- Traitement simple de tâches uniques
:::

::: danger Inconvénients
- Ne peut pas gérer des valeurs multiples
- Pas de fonction d'annulation
- Ne convient pas au traitement de flux continus
- Le traitement d'événements complexes est difficile
:::

</div>

### RxJS
<div class="comparison-cards">

::: tip Avantages
- Possibilité de gérer plusieurs valeurs dans le temps
- Contrôle complexe possible avec une grande variété d'opérateurs
- L'annulation (`unsubscribe`) est facile
- Mise en œuvre souple de la gestion des erreurs et des tentatives
- Déclaratif et testable
:::

::: danger Inconvénients
- Coût d'apprentissage élevé
- Nécessite des bibliothèques
- Sur-spécification pour les processus simples
- Le débogage peut être difficile
:::

</div>

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>

## Domaines où RxJS est particulièrement efficace

RxJS est particulièrement puissant dans les domaines suivants. Il peut répondre de manière élégante à des besoins complexes qui sont difficiles à satisfaire avec Promise seul.

| Domaine | Exemples | Comparaison avec Promise |
|------|----------|-------------------------|
| **Communication en temps réel** | WebSocket, SSE, chat, mise à jour des cours boursiers | Promise est uniquement destiné à la communication ponctuelle. Non adapté au traitement continu des messages |
| **Contrôle des entrées utilisateur** | Autocomplétion de recherche, validation de formulaires | debounce, distinctUntilChanged, etc. sont standard |
| **Combinaison de sources multiples** | Combinaison de conditions de recherche × ordre de tri × filtres | Peut être décrit de manière concise avec combineLatest, withLatestFrom |
| **Support hors ligne** | PWA, surveillance de l'état du réseau, resynchronisation automatique | Contrôle flexible des tentatives avec retry, retryWhen |
| **API de streaming** | OpenAI, sortie séquentielle des tokens de réponse IA | Peut traiter des données continues en temps réel |
| **Contrôle de l'annulation** | Interruption des processus longs, rejet des anciennes requêtes | Peut annuler immédiatement avec unsubscribe() |

> [!NOTE]
> Pour plus de détails sur l'utilisation de RxJS, voir également [Qu'est-ce que RxJS - Cas d'utilisation](./what-is-rxjs.md#cas-dutilisation).

## Résumé

| Objectif | Recommandé | Raison |
|------|------|------|
| Requête HTTP unique | Promise (`async/await`) | Simple, lisible, API standard |
| Traitement des événements d'entrée utilisateur | RxJS | Nécessite un contrôle tel que debounce, distinct |
| Données en temps réel (WebSocket) | RxJS | Peut naturellement gérer des messages continus |
| Exécution parallèle de processus asynchrones multiples | Promise (`Promise.all`) | Promise est suffisant pour une simple exécution parallèle |
| Flux d'événements continus | RxJS | Peut gérer des valeurs multiples dans le temps |
| Traitement annulable | RxJS | Annulation fiable avec unsubscribe() |
| Applications simples | Promise | Faible coût d'apprentissage, peu de dépendances |
| Applications Angular | RxJS | Intégré de manière standard dans le framework |

### Politique de base
- **Utilisez Promise si cela peut être simple**
- **Utilisez RxJS si un traitement de flux complexe est nécessaire**
- **La combinaison des deux est également efficace** (pont avec `from()`)

RxJS est puissant, mais vous n'avez pas besoin d'utiliser RxJS pour tous les traitements asynchrones. Il est important d'utiliser le bon outil dans la bonne situation. Promise et RxJS sont tous deux des outils puissants pour gérer les traitements asynchrones, mais chacun a des caractéristiques différentes.

- **Promise** est le mieux adapté aux traitements asynchrones simples et ponctuels. Choisissez Promise pour un traitement asynchrone de base en raison de son faible coût d'apprentissage et de sa bonne compatibilité avec async/await.
- **RxJS** est puissant lorsqu'il s'agit de gérer des valeurs multiples, de traiter des événements ou de contrôler des flux de données complexes. RxJS convient également lorsque des contrôles avancés tels que l'annulation et la réessai sont nécessaires.

Dans le cadre d'un développement réel, il est important d'utiliser les deux de manière appropriée. Si nécessaire, vous pouvez être flexible en convertissant Promise en Observable ou Observable en Promise.

> [!TIP] Prochaines étapes
> - En savoir plus sur les Observables dans [Qu'est-ce qu'un Observable](/fr/guide/observables/what-is-observable)
> - Apprendre à créer des Observables dans [Creation Functions](/fr/guide/creation-functions/index)
> - Apprendre à convertir et à contrôler les Observables avec les [Opérateurs](/fr/guide/operators/index)
