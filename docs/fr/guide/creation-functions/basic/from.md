---
description: "from() - Fonction de création qui convertit les tableaux, Promises, itérables et objets de type Observable en flux Observable avec une intégration asynchrone transparente"
---

# from() - Conversion depuis un tableau, une Promise, etc.

`from()` est une fonction de création qui crée un Observable à partir de tableaux, Promises, itérables et objets de type Observable.

## Vue d'ensemble

`from()` convertit des structures de données existantes (tableaux, Promises, itérables, etc.) en flux Observable. En particulier, elle est fréquemment utilisée pour intégrer le traitement asynchrone (Promise) dans le monde RxJS.

**Signature** :
```typescript
function from<T>(input: ObservableInput<T>, scheduler?: SchedulerLike): Observable<T>
```

**Documentation officielle** : [📘 RxJS Official : from()](https://rxjs.dev/api/index/function/from)

## Utilisation de base

`from()` accepte une variété de types d'entrée.

```typescript
import { from } from 'rxjs';

// Créer à partir d'un tableau
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Valeur du tableau:', value),
  complete: () => console.log('Tableau terminé')
});

// Créer à partir d'une Promise
const promise$ = from(Promise.resolve('Résultat de la Promise'));
promise$.subscribe({
  next: value => console.log('Résultat de la Promise:', value),
  complete: () => console.log('Promise terminée')
});

// Créer à partir d'un itérable
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Valeur de l\'itérable:', value),
  complete: () => console.log('Itérable terminé')
});

// Sortie:
// Valeur du tableau: 1
// Valeur du tableau: 2
// Valeur du tableau: 3
// Tableau terminé
// Valeur de l'itérable: 1
// Valeur de l'itérable: 2
// Valeur de l'itérable: 3
// Itérable terminé
// Résultat de la Promise: Résultat de la Promise
// Promise terminée
```

## Caractéristiques importantes

### 1. Émettre chaque élément du tableau individuellement

Lorsque `from()` reçoit un tableau, elle émet chaque élément du tableau individuellement dans l'ordre.

```typescript
import { from } from 'rxjs';

from([10, 20, 30]).subscribe(value => console.log(value));

// Sortie:
// 10
// 20
// 30
```

> [!IMPORTANT]
> **Différence avec `of()`** :
> - `of([1, 2, 3])` → Émet le tableau lui-même comme une seule valeur
> - `from([1, 2, 3])` → Émet chaque élément `1`, `2`, `3` séparément

### 2. Traitement automatique des Promises

Passer une Promise émettra la valeur résolue et se terminera immédiatement.

```typescript
import { from } from 'rxjs';

const fetchData = (): Promise<string> => {
  return new Promise(resolve => {
    setTimeout(() => resolve('Récupération des données terminée'), 1000);
  });
};

from(fetchData()).subscribe({
  next: value => console.log(value),
  complete: () => console.log('Terminé')
});

// Sortie après 1 seconde:
// Récupération des données terminée
// Terminé
```

> [!WARNING]
> Si la Promise est rejetée, l'Observable émet une erreur.
> ```typescript
> import { from } from "rxjs";
> from(Promise.reject('Erreur')).subscribe({
>   error: err => console.error('Erreur survenue:', err)
> });
> ```

### 3. Prise en charge des itérables

En plus des tableaux, elle prend en charge les objets itérables tels que `Set`, `Map` et `Generator`.

```typescript
import { from } from 'rxjs';

// Set
from(new Set(['A', 'B', 'C'])).subscribe(console.log);
// Sortie: A, B, C

// Map (paires clé-valeur)
from(new Map([['clé1', 'valeur1'], ['clé2', 'valeur2']])).subscribe(console.log);
// Sortie: ['clé1', 'valeur1'], ['clé2', 'valeur2']

// Generator
function* numberGenerator() {
  yield 1;
  yield 2;
  yield 3;
}
from(numberGenerator()).subscribe(console.log);
// Sortie: 1, 2, 3
```

### 4. Cold Observable

`from()` est un **Cold Observable**. Chaque abonnement initie une exécution indépendante.

```typescript
import { from } from 'rxjs';

const numbers$ = from([1, 2, 3]);

numbers$.subscribe(val => console.log('Abonné A:', val));
numbers$.subscribe(val => console.log('Abonné B:', val));

// Chaque abonné traite le tableau indépendamment
// Sortie:
// Abonné A: 1
// Abonné A: 2
// Abonné A: 3
// Abonné B: 1
// Abonné B: 2
// Abonné B: 3
```

> [!NOTE]
> **Caractéristiques du Cold Observable** :
> - Une exécution indépendante est initiée pour chaque abonnement
> - Chaque abonné reçoit son propre flux de données
> - Les Promises sont également évaluées par abonnement
>
> Voir [Cold Observable et Hot Observable](/fr/guide/observables/cold-and-hot-observables) pour plus d'informations.

## Différence entre from() et of()

La différence la plus importante entre les deux est la façon dont les tableaux sont traités.

```typescript
import { from, of } from 'rxjs';

const array = [1, 2, 3];

// of() - émet le tableau comme une seule valeur
of(array).subscribe(value => {
  console.log('of():', value); // [1, 2, 3]
});

// from() - émet chaque élément du tableau individuellement
from(array).subscribe(value => {
  console.log('from():', value); // 1, 2, 3
});
```

| Fonction de création | Traitement du tableau | Objectif |
|-------------------|-----------|------|
| `of([1, 2, 3])` | Émet le tableau lui-même | Veut traiter le tableau comme donnée |
| `from([1, 2, 3])` | Émet chaque élément individuellement | Veut traiter les éléments un par un |

## Cas d'utilisation pratiques

### 1. Streamer les appels API

Streamer les clients HTTP basés sur Promise comme Fetch API et axios.

```typescript
import { from, Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
}

function fetchUser(id: number): Observable<User> {
  return from(
    fetch(`https://jsonplaceholder.typicode.com/users/${id}`)
      .then(response => response.json())
  ).pipe(
    catchError(error => {
      console.error('Erreur API:', error);
      return of({ id: 0, name: 'Inconnu', email: '' });
    })
  );
}

fetchUser(1).subscribe(user => console.log('Utilisateur:', user));
```

### 2. Traitement séquentiel des éléments d'un tableau

Exécuter un traitement asynchrone de manière séquentielle pour chaque élément du tableau.

```typescript
import { from } from 'rxjs';
import { concatMap, delay } from 'rxjs';

const urls = [
  'https://jsonplaceholder.typicode.com/posts/1',
  'https://jsonplaceholder.typicode.com/posts/2',
  'https://jsonplaceholder.typicode.com/posts/3'
];

from(urls).pipe(
  concatMap(url =>
    from(fetch(url).then(res => res.json())).pipe(
      delay(500) // Limitation du débit
    )
  )
).subscribe(data => console.log('Récupéré:', data));
```

### 3. Traitement d'itérateurs asynchrones

Les itérateurs asynchrones (générateurs async) sont également pris en charge.

```typescript
import { from } from 'rxjs';

async function* asyncGenerator() {
  yield await Promise.resolve(1);
  yield await Promise.resolve(2);
  yield await Promise.resolve(3);
}

from(asyncGenerator()).subscribe(value => console.log(value));
// Sortie: 1, 2, 3
```

### 4. Intégration d'émetteurs d'événements

Streamer Node.js EventEmitter et les systèmes d'événements personnalisés.

```typescript
import { from } from 'rxjs';

// Objet personnalisé itérable
class DataSource {
  *[Symbol.iterator]() {
    yield 'Donnée A';
    yield 'Donnée B';
    yield 'Donnée C';
  }
}

from(new DataSource()).subscribe(console.log);
// Sortie: Donnée A, Donnée B, Donnée C
```

## Utilisation dans un pipeline

`from()` est utile pour utiliser des données existantes comme point de départ d'un traitement en pipeline.

```typescript
import { from } from 'rxjs';
import { map, filter, reduce } from 'rxjs';

interface Product {
  id: number;
  name: string;
  price: number;
}

const products: Product[] = [
  { id: 1, name: 'Produit A', price: 1000 },
  { id: 2, name: 'Produit B', price: 2000 },
  { id: 3, name: 'Produit C', price: 500 }
];

from(products).pipe(
  filter(product => product.price >= 1000),
  map(product => product.price),
  reduce((sum, price) => sum + price, 0)
).subscribe(total => console.log('Montant total:', total));
// Sortie: Montant total: 3000
```

## Erreurs courantes

### 1. Mauvaise compréhension du timing d'exécution de la Promise

```typescript
// ❌ Incorrect - La Promise commence à s'exécuter à la création
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1'); // Déjà démarrée
from(promise).subscribe(console.log); // Pas au moment de l'abonnement

// ✅ Correct - utiliser defer() si vous voulez exécuter à l'abonnement
import { defer, from } from 'rxjs';

const deferred$ = defer(() =>
  from(fetch('https://jsonplaceholder.typicode.com/posts/1'))
);
deferred$.subscribe(console.log); // S'exécute au moment de l'abonnement
```

> [!WARNING]
> **La Promise n'est pas évaluée paresseusement**
>
> La Promise commence à s'exécuter lorsqu'elle est créée. `from(promise)` ne fait qu'envelopper une Promise déjà en cours d'exécution. Si vous voulez l'exécuter au moment de l'abonnement, utilisez `defer(() => from(promise))`.

### 2. Confusion entre tableau et of()

```typescript
import { from, map, of } from "rxjs";

// ❌ Différent de l'intention - le tableau entier est émis
of([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Sortie: [1, 2, 3] (le tableau lui-même)

// ✅ Correct - traiter chaque élément individuellement
from([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Sortie: 2, 4, 6
```

## Considérations sur les performances

Les performances de `from()` dépendent du type d'entrée.

> [!TIP]
> **Conseils d'optimisation** :
> - Lors du traitement de grandes quantités de données (des milliers à des dizaines de milliers d'éléments), limitez le nombre d'opérations simultanées lors de la combinaison avec `concatMap` et `mergeMap`.
> - Lors du traitement de tableaux de Promises, envisagez d'utiliser `forkJoin` ou `combineLatest`.

```typescript
import { from } from 'rxjs';
import { mergeMap } from 'rxjs';

const urls = [...Array(100)].map((_, i) => `https://jsonplaceholder.typicode.com/posts/${i + 1}`);

from(urls).pipe(
  mergeMap(
    url => from(fetch(url).then(res => res.json())),
    5 // Limiter l'exécution simultanée à 5
  )
).subscribe(data => console.log(data));
```

## Fonctions de création associées

| Fonction | Différence | Utilisation |
|----------|------|----------|
| **[of()](/fr/guide/creation-functions/basic/of)** | Émet les arguments en séquence | Veut émettre les valeurs telles quelles |
| **[fromEvent()](/fr/guide/creation-functions/basic/fromEvent)** | Streamer les événements | Gérer les événements DOM ou EventEmitter |
| **[defer()](/fr/guide/creation-functions/conditional/defer)** | Différer la génération jusqu'à l'abonnement | Besoin d'exécution paresseuse de Promise |
| **ajax()** | Dédié à la communication HTTP | Veut compléter les requêtes HTTP dans RxJS |

## Résumé

- `from()` crée un Observable à partir de tableaux, Promises et itérables
- Émet chaque élément d'un tableau séparément (différent de `of()`)
- Traite automatiquement la Promise et émet le résultat
- Idéal pour intégrer le traitement asynchrone dans le monde RxJS
- Notez que la Promise est exécutée au moment de la création (utilisez `defer()` pour une exécution paresseuse)

## Prochaines étapes

- [fromEvent() - Convertir les événements en Observable](/fr/guide/creation-functions/basic/fromEvent)
- [defer() - Différer la génération jusqu'à l'abonnement](/fr/guide/creation-functions/conditional/defer)
- [Retour aux fonctions de création de base](/fr/guide/creation-functions/basic/)
