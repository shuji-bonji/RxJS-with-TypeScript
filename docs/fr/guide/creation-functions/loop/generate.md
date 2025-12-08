---
description: "generate() - Génération de boucles génériques avec un contrôle flexible des conditions : Boucles déclaratives de type while pour Fibonacci, pagination et gestion d'état personnalisée"
---

# generate() - Génération générique de boucles

`generate()` est une fonction de création qui fournit un traitement de boucle flexible en tant qu'Observable en spécifiant l'état initial, la condition de continuation, la mise à jour de l'état et la sélection du résultat.

## Vue d'ensemble

`generate()` peut décrire de manière déclarative un traitement de boucle flexible comme les instructions while et for. Elle est utilisée lorsque des conditions ou une gestion d'état plus complexes que celles de `range()` sont requises.

**Signature** :
```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

**Paramètres** :
- `initialState` : L'état initial de la boucle
- `condition` : Fonction pour déterminer la condition de continuation (`false` termine la boucle)
- `iterate` : Fonction permettant de passer à l'état suivant (mise à jour de l'état)
- `resultSelector` : Fonction permettant de sélectionner une valeur à émettre à partir de l'état (si omise, l'état lui-même est émis)
- `scheduler` : Planificateur qui émet les valeurs (omis : émet les valeurs de manière synchrone)

**Documentation officielle** : [📘 RxJS Official : generate()](https://rxjs.dev/api/index/function/generate)

## Utilisation de base

### Pattern 1 : Compteur simple

C'est l'utilisation la plus basique.

```typescript
import { generate } from 'rxjs';

// Compter de 1 à 5
generate(
  1,              // État initial
  x => x <= 5,    // Condition de continuation
  x => x + 1      // Mise à jour de l'état
).subscribe({
  next: value => console.log('Valeur:', value),
  complete: () => console.log('Terminé')
});

// Sortie:
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Valeur: 4
// Valeur: 5
// Terminé
```

Ce code est équivalent à l'instruction while suivante :

```typescript
let x = 1;
while (x <= 5) {
  console.log('Valeur:', x);
  x = x + 1;
}
console.log('Terminé');
```

### Pattern 2 : Convertir les valeurs avec resultSelector

Vous pouvez séparer l'état de la valeur à émettre.

```typescript
import { generate } from 'rxjs';

// L'état interne est un compteur, mais la valeur émise est une valeur au carré
generate(
  1,              // État initial: 1
  x => x <= 5,    // Condition de continuation: x <= 5
  x => x + 1,     // Mise à jour de l'état: x + 1
  x => x * x      // Sélection du résultat: émettre x^2
).subscribe(console.log);

// Sortie: 1, 4, 9, 16, 25
```

### Pattern 3 : Objet d'état complexe

Les objets complexes peuvent être utilisés comme états.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

// Calculer la somme cumulative
generate<number, State>(
  { count: 1, sum: 0 },           // État initial
  state => state.count <= 5,      // Condition de continuation
  state => ({                     // Mise à jour de l'état
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => state.sum              // Sélection du résultat
).subscribe(console.log);

// Sortie: 0, 1, 3, 6, 10
// (0, 0+1, 0+1+2, 0+1+2+3, 0+1+2+3+4)
```

## Caractéristiques importantes

### 1. Comportement de type instruction while

`generate()` fournit un contrôle flexible comme une instruction while.

```typescript
import { generate } from "rxjs";

// Instruction while
let i = 1;
while (i <= 10) {
  console.log(i);
  i = i * 2;
}

// La même chose avec generate()
generate(
  1,              // let i = 1;
  i => i <= 10,   // while (i <= 10)
  i => i * 2      // i = i * 2;
).subscribe(console.log);

// Sortie: 1, 2, 4, 8
```

### 2. Émission synchrone

Par défaut, toutes les valeurs sont émises **de manière synchrone** lors de l'abonnement.

```typescript
import { generate } from 'rxjs';

console.log('Avant abonnement');

generate(1, x => x <= 3, x => x + 1).subscribe(val => console.log('Valeur:', val));

console.log('Après abonnement');

// Sortie:
// Avant abonnement
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Après abonnement
```

### 3. Attention aux boucles infinies

Si la condition est toujours `true`, vous obtiendrez une boucle infinie.

```typescript
import { generate, take } from 'rxjs';
// ❌ Danger: boucle infinie (le navigateur se fige)
// generate(0, x => true, x => x + 1).subscribe(console.log);

// ✅ Sûr: utilisez take() pour limiter le nombre
generate(
  0,
  x => true,  // Toujours vrai
  x => x + 1
).pipe(
  take(10)    // Obtenir seulement les 10 premiers
).subscribe(console.log);

// Sortie: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

> [!WARNING]
> **Attention aux boucles infinies** :
> - Si la condition est toujours `true`, une boucle infinie se produit
> - Utilisez `take()`, `takeWhile()`, ou `takeUntil()` pour limiter le nombre d'émissions
> - Ou définissez des conditions de sortie appropriées avec des fonctions conditionnelles

## Cas d'utilisation pratiques

### 1. Suite de Fibonacci

Exemple de transitions d'état complexes.

```typescript
import { generate, take } from 'rxjs';
interface FibState {
  current: number;
  next: number;
}

// 10 premiers termes de la suite de Fibonacci
generate<number, FibState>(
  { current: 0, next: 1 },           // État initial: F(0)=0, F(1)=1
  state => true,                     // Génération infinie
  state => ({                        // Mise à jour de l'état
    current: state.next,
    next: state.current + state.next
  }),
  state => state.current             // Émettre la valeur actuelle
).pipe(
  take(10)                           // 10 premiers termes
).subscribe(console.log);

// Sortie: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### 2. Backoff exponentiel

Génération de temps d'attente exponentiel utilisée dans le processus de réessai.

```typescript
import { generate } from 'rxjs';

interface RetryState {
  attempt: number;
  delay: number;
}

// Générer le délai pour le backoff exponentiel (1, 2, 4, 8, 16 secondes)
generate<number, RetryState>(
  { attempt: 0, delay: 1000 },       // État initial: 1 seconde
  state => state.attempt < 5,        // Maximum 5 tentatives
  state => ({                        // Mise à jour de l'état
    attempt: state.attempt + 1,
    delay: state.delay * 2           // Doubler le temps de délai
  }),
  state => state.delay               // Émettre le temps de délai
).subscribe(delay => {
  console.log(`Réessai dans ${delay / 1000} secondes`);
});

// Sortie:
// Réessai dans 1 seconde
// Réessai dans 2 secondes
// Réessai dans 4 secondes
// Réessai dans 8 secondes
// Réessai dans 16 secondes
```

### 3. Contrôle de pagination

Continuer à récupérer tant que la page suivante existe.

```typescript
import { generate, of, Observable, concatMap, delay } from 'rxjs';
interface PageState {
  page: number;
  hasNext: boolean;
}

interface PageData {
  page: number;
  items: string[];
  hasNext: boolean;
}

// Fonction pour simuler la récupération de données de page
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Element${page}-1`, `Element${page}-2`, `Element${page}-3`],
    hasNext: page < 10 // Jusqu'à la page 10
  }).pipe(
    delay(500) // Simuler un appel API
  );
}

// Récupérer la page tant qu'elle existe (en pratique, obtenir hasNext de la réponse API)
generate<number, PageState>(
  { page: 1, hasNext: true },        // État initial
  state => state.hasNext,            // Continuer tant qu'il y a une page suivante
  state => ({                        // Mise à jour de l'état
    page: state.page + 1,
    hasNext: state.page < 10         // Supposons qu'il y ait jusqu'à 10 pages
  }),
  state => state.page                // Émettre le numéro de page
).pipe(
  concatMap(page => fetchPage(page)) // Récupérer chaque page à tour de rôle
).subscribe(
  data => console.log(`Récupération page ${data.page}:`, data.items),
  err => console.error('Erreur:', err),
  () => console.log('Toutes les pages récupérées')
);

// Sortie:
// Récupération page 1: ['Element1-1', 'Element1-2', 'Element1-3']
// Récupération page 2: ['Element2-1', 'Element2-2', 'Element2-3']
// ...
// Récupération page 10: ['Element10-1', 'Element10-2', 'Element10-3']
// Toutes les pages récupérées
```

### 4. Timer personnalisé

Émet des événements à intervalles irréguliers.

```typescript
import { generate, of, concatMap, delay } from 'rxjs';
interface TimerState {
  count: number;
  delay: number;
}

// Timer avec délai augmentant progressivement
generate<string, TimerState>(
  { count: 0, delay: 1000 },         // État initial: 1 seconde
  state => state.count < 5,          // Jusqu'à 5 fois
  state => ({                        // Mise à jour de l'état
    count: state.count + 1,
    delay: state.delay + 500         // Augmenter le délai de 500 ms
  }),
  state => `Événement${state.count + 1}`
).pipe(
  concatMap((message, index) => {
    const delayTime = 1000 + index * 500;
    console.log(`Attente de ${delayTime}ms avant émission`);
    return of(message).pipe(delay(delayTime));
  })
).subscribe(console.log);

// Sortie:
// Attente de 1000ms avant émission
// Événement1 (après 1 seconde)
// Attente de 1500ms avant émission
// Événement2 (après 2.5 secondes)
// Attente de 2000ms avant émission
// Événement3 (après 4.5 secondes)
// ...
```

### 5. Calcul de factorielles

Représenter les calculs mathématiques sous forme de flux.

```typescript
import { generate } from 'rxjs';

interface FactorialState {
  n: number;
  result: number;
}

// Calculer la factorielle de 5 (5! = 5 × 4 × 3 × 2 × 1 = 120)
generate<number, FactorialState>(
  { n: 5, result: 1 },               // État initial
  state => state.n > 0,              // Continue pour n > 0
  state => ({                        // Mise à jour de l'état
    n: state.n - 1,
    result: state.result * state.n
  }),
  state => state.result              // Émettre le résultat intermédiaire
).subscribe(console.log);

// Sortie: 5, 20, 60, 120, 120
// (1*5, 5*4, 20*3, 60*2, 120*1)
```

## Comparaison avec d'autres fonctions de création

### generate() vs range()

```typescript
import { generate, range } from 'rxjs';

// range() - numérotation séquentielle simple
range(1, 5).subscribe(console.log);
// Sortie: 1, 2, 3, 4, 5

// generate() - la même chose, mais plus explicite
generate(
  1,
  x => x <= 5,
  x => x + 1
).subscribe(console.log);
// Sortie: 1, 2, 3, 4, 5

// Vraie valeur de generate(): étapes complexes
generate(
  1,
  x => x <= 100,
  x => x * 2  // Augmente d'un facteur de 2
).subscribe(console.log);
// Sortie: 1, 2, 4, 8, 16, 32, 64
// (pas possible avec range())
```

### generate() vs defer()

```typescript
import { generate, defer, of } from 'rxjs';

// generate() - traitement de boucle
generate(1, x => x <= 3, x => x + 1).subscribe(console.log);
// Sortie: 1, 2, 3

// defer() - génération à l'abonnement (pas une boucle)
defer(() => of(1, 2, 3)).subscribe(console.log);
// Sortie: 1, 2, 3

// Différence: generate() a un état, defer seulement une évaluation paresseuse
```

> [!TIP]
> **Critères de sélection** :
> - **Nombres séquentiels simples** → `range()`
> - **Conditions ou étapes complexes** → `generate()`
> - **Déterminé dynamiquement à l'abonnement** → `defer()`
> - **Fibonacci, factorielle, etc.** → `generate()`

## Asynchronisation avec le planificateur

Lors du traitement de grandes quantités de données, une exécution asynchrone est possible en spécifiant un planificateur.

```typescript
import { generate, asyncScheduler, observeOn } from 'rxjs';
console.log('Démarrage');

// Exécuter un million de boucles de manière asynchrone
generate(
  1,
  x => x <= 1000000,
  x => x + 1
).pipe(
  observeOn(asyncScheduler)
).subscribe({
  next: val => {
    if (val % 100000 === 0) {
      console.log(`Progression: ${val}`);
    }
  },
  complete: () => console.log('Terminé')
});

console.log('Après abonnement (asynchrone, donc exécuté immédiatement)');

// Sortie:
// Démarrage
// Après abonnement (asynchrone, donc exécuté immédiatement)
// Progression: 100000
// Progression: 200000
// ...
// Terminé
```

## Considérations sur les performances

Comme `generate()` émet des valeurs de manière synchrone, les performances doivent être prises en compte lors de la génération d'un grand nombre de valeurs ou de l'exécution de calculs complexes.

> [!WARNING]
> **Optimisation des performances** :
> ```typescript
> // ❌ Mauvais exemple: calcul complexe exécuté de manière synchrone (interface utilisateur bloquée)
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).subscribe(console.log);
>
> // ✅ Bon exemple 1: asynchrone avec planificateur
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Bon exemple 2: Limiter le nombre avec take()
> generate(
>   1,
>   x => true,  // Boucle infinie
>   x => x + 1
> ).pipe(
>   take(100)   // Seulement les 100 premiers
> ).subscribe(console.log);
> ```

## Gestion des erreurs

Bien que `generate()` lui-même n'émette pas d'erreurs, des erreurs peuvent survenir dans les pipelines et les fonctions de mise à jour d'état.

```typescript
import { generate, of, map, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,
  x => x + 1
).pipe(
  map(n => {
    if (n === 5) {
      throw new Error('Erreur à 5');
    }
    return n * 2;
  }),
  catchError(error => {
    console.error('Erreur survenue:', error.message);
    return of(-1); // Retourner la valeur par défaut
  })
).subscribe(console.log);

// Sortie: 2, 4, 6, 8, -1
```

### Erreur dans la fonction de mise à jour d'état

Une erreur dans une fonction de mise à jour d'état fera entrer l'Observable dans un état d'erreur.

```typescript
import { generate, EMPTY, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,
  x => {
    if (x === 5) {
      throw new Error('Erreur lors de la mise à jour d\'état');
    }
    return x + 1;
  }
).pipe(
  catchError(error => {
    console.error('Erreur:', error.message);
    return EMPTY; // Retourner un Observable vide
  })
).subscribe({
  next: console.log,
  complete: () => console.log('Terminé')
});

// Sortie: 1, 2, 3, 4, Erreur: Erreur lors de la mise à jour d'état, Terminé
```

## Sécurité de type en TypeScript

`generate()` peut séparer le type de l'état du type de la valeur émise.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

interface Result {
  index: number;
  average: number;
}

// État: State, valeur émise: Result
const stats$ = generate<Result, State>(
  { count: 1, sum: 0 },
  state => state.count <= 5,
  state => ({
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => ({
    index: state.count,
    average: state.sum / state.count
  })
);

stats$.subscribe(result => {
  console.log(`[${result.index}] Moyenne: ${result.average}`);
});

// Sortie:
// [1] Moyenne: 0
// [2] Moyenne: 0.5
// [3] Moyenne: 1
// [4] Moyenne: 1.5
// [5] Moyenne: 2
```

## Résumé

`generate()` est une puissante fonction de création qui permet de décrire de manière déclarative un traitement de boucle complexe.

> [!IMPORTANT]
> **Caractéristiques de generate()** :
> - ✅ Contrôle de boucle flexible comme les instructions while/for
> - ✅ Gestion d'état complexe possible
> - ✅ Idéal pour les calculs mathématiques tels que Fibonacci, factorielle, etc.
> - ✅ Les valeurs d'état et d'émission peuvent être séparées
> - ⚠️ Attention aux boucles infinies (limitées par `take()`)
> - ⚠️ Envisager l'asynchrone pour les grandes quantités de données
> - ⚠️ Utiliser `range()` pour les nombres séquentiels simples

## Sujets associés

- [range()](/fr/guide/creation-functions/loop/range) - Génération de nombres séquentiels simples
- [defer()](/fr/guide/creation-functions/conditional/defer) - Génération dynamique à l'abonnement
- [expand()](/fr/guide/operators/transformation/expand) - Expansion récursive (opérateur d'ordre supérieur)
- [scan()](/fr/guide/operators/transformation/scan) - Calcul cumulatif

## Références

- [RxJS Official : generate()](https://rxjs.dev/api/index/function/generate)
- [Learn RxJS : generate](https://www.learnrxjs.io/learn-rxjs/operators/creation/generate)
