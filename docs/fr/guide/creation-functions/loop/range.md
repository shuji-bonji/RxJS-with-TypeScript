---
description: "range() - Fonction de création qui génère des entiers consécutifs de manière déclarative : Alternative efficace en termes de mémoire aux boucles for pour le traitement par lots et la pagination"
---

# range() - Génère une plage de nombres

`range()` est une fonction de création de type instruction for qui émet un nombre spécifié d'entiers consécutifs à partir d'une valeur de départ spécifiée.

## Vue d'ensemble

`range()` émet une séquence d'entiers consécutifs en tant qu'Observable en spécifiant une valeur de départ et le nombre d'entiers. Elle est utilisée pour la génération de nombres séquentiels et le traitement par lots comme moyen déclaratif pour remplacer l'instruction traditionnelle `for`.

**Signature** :
```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

**Paramètres** :
- `start` : La valeur de départ (à partir de laquelle l'émission doit commencer)
- `count` : le nombre de valeurs à publier (omis, de 0 à moins que `start`)
- `scheduler` : le planificateur pour émettre les valeurs (omis : émission synchrone)

**Documentation officielle** : [📘 RxJS Official : range()](https://rxjs.dev/api/index/function/range)

## Utilisation de base

### Pattern 1 : Spécifier la valeur de départ et le nombre

C'est l'utilisation la plus courante.

```typescript
import { range } from 'rxjs';

// Générer 5 nombres séquentiels à partir de 1 (1, 2, 3, 4, 5)
range(1, 5).subscribe({
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

### Pattern 2 : Nombres séquentiels commençant à 0

En définissant la valeur de départ à 0, un nombre séquentiel comme un index de tableau peut être généré.

```typescript
import { range } from 'rxjs';

// Nombres séquentiels de 0 à 9 (0, 1, 2, ..., 9)
range(0, 10).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

### Pattern 3 : Commencer avec un nombre négatif

Les nombres négatifs peuvent également être générés.

```typescript
import { range } from 'rxjs';

// 5 nombres séquentiels à partir de -3 (-3, -2, -1, 0, 1)
range(-3, 5).subscribe(console.log);
// Sortie: -3, -2, -1, 0, 1
```

## Caractéristiques importantes

### 1. Émission synchrone

Par défaut, `range()` émet toutes les valeurs **de manière synchrone** lors de l'abonnement.

```typescript
import { range } from 'rxjs';

console.log('Avant abonnement');

range(1, 3).subscribe(value => console.log('Valeur:', value));

console.log('Après abonnement');

// Sortie:
// Avant abonnement
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Après abonnement
```

### 2. Se termine immédiatement

Notifie `complete` immédiatement après avoir émis toutes les valeurs.

```typescript
import { range } from 'rxjs';

range(1, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Terminé!')
});

// Sortie: 1, 2, 3, Terminé!
```

### 3. Équivalence avec l'instruction for

`range(start, count)` est équivalent à l'instruction for suivante.

```typescript
// Instruction for impérative
for (let i = start; i < start + count; i++) {
  console.log(i);
}

// range() déclaratif
range(start, count).subscribe(console.log);
```

## Cas d'utilisation pratiques

### 1. Traitement par lots

Utilisé pour exécuter plusieurs tâches de manière séquentielle.

```typescript
import { range, of, Observable, concatMap, delay, map } from 'rxjs';
// Fonction pour simuler le traitement de données
function processItem(index: number): Observable<string> {
  return of(index).pipe(
    delay(100), // Simuler un temps de traitement de 100ms
    map(i => `Résultat du traitement de l'élément ${i}`)
  );
}

// Traiter séquentiellement 10 éléments de données (1 seconde de délai entre chaque traitement)
range(1, 10).pipe(
  concatMap(index =>
    processItem(index).pipe(delay(1000))
  )
).subscribe({
  next: result => console.log(`Traitement terminé: ${result}`),
  complete: () => console.log('Tous les traitements terminés')
});

// Sortie:
// Traitement terminé: Résultat du traitement de l'élément 1 (après environ 1.1 secondes)
// Traitement terminé: Résultat du traitement de l'élément 2 (après environ 2.1 secondes)
// ...
// Traitement terminé: Résultat du traitement de l'élément 10 (après environ 10.1 sec.)
// Tous les traitements terminés
```

### 2. Pagination

Récupérer plusieurs pages de données séquentiellement.

```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Fonction pour simuler la récupération de données de page
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Element${page}-1`, `Element${page}-2`, `Element${page}-3`]
  }).pipe(
    delay(500) // Simuler un appel API
  );
}

function fetchAllPages(totalPages: number) {
  return range(1, totalPages).pipe(
    concatMap(page => fetchPage(page))
  );
}

fetchAllPages(5).subscribe({
  next: (data: PageData) => console.log(`Page ${data.page}:`, data.items),
  complete: () => console.log('Toutes les pages récupérées')
});

// Sortie:
// Page 1: ['Element1-1', 'Element1-2', 'Element1-3']
// Page 2: ['Element2-1', 'Element2-2', 'Element2-3']
// Page 3: ['Element3-1', 'Element3-2', 'Element3-3']
// Page 4: ['Element4-1', 'Element4-2', 'Element4-3']
// Page 5: ['Element5-1', 'Element5-2', 'Element5-3']
// Toutes les pages récupérées
```

### 3. Traitement des index de tableau

Utiliser comme boucle basée sur l'index lors du traitement de chaque élément d'un tableau.

```typescript
import { range, map } from 'rxjs';
const items = ['Pomme', 'Banane', 'Cerise', 'Datte', 'Sureau'];

range(0, items.length).pipe(
  map(index => ({ index, item: items[index] }))
).subscribe(({ index, item }) => {
  console.log(`[${index}] ${item}`);
});

// Sortie:
// [0] Pomme
// [1] Banane
// [2] Cerise
// [3] Datte
// [4] Sureau
```

### 4. Génération de données de test

Utile pour générer des données fictives pour les tests unitaires.

```typescript
import { range, map, toArray } from 'rxjs';
// Générer des données utilisateur fictives
range(1, 100).pipe(
  map(id => ({
    id,
    name: `Utilisateur${id}`,
    email: `utilisateur${id}@exemple.com`
  })),
  toArray()
).subscribe(users => {
  console.log(`${users.length} utilisateurs générés`);
  // Utiliser dans les tests
});
```

### 5. Compteur pour le traitement de réessai

Contrôle le nombre de réessais en cas d'erreur.

```typescript
import { range, throwError, concat, of, Observable, mergeMap, delay, catchError, map, toArray } from 'rxjs';
// Fonction pour simuler la récupération de données (échoue aléatoirement)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.7; // 30% de chances de succès

  return of(shouldFail).pipe(
    delay(300),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Échec de récupération des données'))
        : of('Récupération des données réussie')
    )
  );
}

function fetchWithRetry(maxRetries: number = 3) {
  return concat(
    range(0, maxRetries).pipe(
      map(attempt => {
        console.log(`Tentative ${attempt + 1}/${maxRetries}`);
        return fetchData().pipe(
          catchError((error: unknown) => {
            if (attempt === maxRetries - 1) {
              return throwError(() => new Error('Nombre maximum de réessais atteint'));
            }
            return throwError(() => error);
          }),
          delay(Math.pow(2, attempt) * 1000) // Backoff exponentiel
        );
      }),
      toArray()
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Résultat:', result),
  error: err => console.error('Erreur:', err.message)
});

// Exemple de sortie:
// Tentative 1/3
// Tentative 2/3
// Résultat: Récupération des données réussie
```

## Asynchronisation avec le planificateur

Lors du traitement de grandes quantités de données, une exécution asynchrone est possible en spécifiant un planificateur.

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
console.log('Démarrage');

// Émettre 1 000 000 de nombres de manière asynchrone
range(1, 1000000).pipe(
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

> [!TIP]
> **Utilisation du planificateur** :
> - Ne pas bloquer l'interface utilisateur lors du traitement de grandes quantités de données
> - Contrôle du temps dans les tests (TestScheduler)
> - Contrôle de la boucle d'événements dans l'environnement Node.js

Pour plus d'informations, veuillez consulter [Types de planificateurs et comment les utiliser](/fr/guide/schedulers/types).

## Comparaison avec d'autres fonctions de création

### range() vs of()

```typescript
import { range, of } from 'rxjs';

// range() - entiers consécutifs
range(1, 3).subscribe(console.log);
// Sortie: 1, 2, 3

// of() - énumérer des valeurs arbitraires
of(1, 2, 3).subscribe(console.log);
// Sortie: 1, 2, 3

// Différence: range() n'accepte que des nombres séquentiels, of() accepte des valeurs arbitraires
of(1, 10, 100).subscribe(console.log);
// Sortie: 1, 10, 100 (pas possible avec range())
```

### range() vs from()

```typescript
import { range, from } from 'rxjs';

// range() - générer des nombres séquentiels
range(1, 5).subscribe(console.log);
// Sortie: 1, 2, 3, 4, 5

// from() - générer à partir d'un tableau (doit créer le tableau à l'avance)
from([1, 2, 3, 4, 5]).subscribe(console.log);
// Sortie: 1, 2, 3, 4, 5

// Avantage de range(): pas de pré-allocation de tableaux en mémoire
range(1, 1000000); // Efficace en mémoire
from(Array.from({ length: 1000000 }, (_, i) => i + 1)); // Le tableau va en mémoire
```

### range() vs generate()

```typescript
import { range, generate } from 'rxjs';

// range() - numérotation séquentielle simple
range(1, 5).subscribe(console.log);
// Sortie: 1, 2, 3, 4, 5

// generate() - exemple complexe de la même chose
generate(
  1,                    // Valeur initiale
  x => x <= 5,          // Condition de continuation
  x => x + 1            // Itération
).subscribe(console.log);
// Sortie: 1, 2, 3, 4, 5

// Avantages de generate(): condition et gestion d'état complexes
generate(
  1,
  x => x <= 100,
  x => x * 2  // Incrémente par un facteur de 2
).subscribe(console.log);
// Sortie: 1, 2, 4, 8, 16, 32, 64
// (pas possible avec range())
```

> [!TIP]
> **Critères de sélection** :
> - **Nécessite des nombres séquentiels** → `range()`
> - **Énumérer n'importe quelle valeur** → `of()`
> - **Tableau/Promise existant** → `from()`
> - **Condition/étape complexe** → `generate()`

## Considérations sur les performances

Parce que `range()` émet des valeurs de manière synchrone, les performances doivent être prises en compte lors de la génération d'un grand nombre de valeurs.

> [!WARNING]
> **Gestion de grandes quantités de données** :
> ```typescript
> // ❌ Mauvais exemple: émettre 1 million de valeurs de manière synchrone (l'interface utilisateur se bloque)
> range(1, 1000000).subscribe(console.log);
>
> // ✅ Bon exemple 1: asynchrone avec planificateur
> range(1, 1000000).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Bon exemple 2: fractionnement par mise en mémoire tampon
> range(1, 1000000).pipe(
>   bufferCount(1000)
> ).subscribe(batch => console.log(`${batch.length} éléments traités`));
> ```

## Choisir entre from() et tableau

```typescript
import { range, from } from 'rxjs';

// Si vous avez besoin de nombres séquentiels → range() est plus concis
range(0, 10).subscribe(console.log);

// Pas besoin de créer un tableau puis de le convertir (inefficace)
from(Array.from({ length: 10 }, (_, i) => i)).subscribe(console.log);

// S'il existe un tableau existant → utilisez from()
const existingArray = [5, 10, 15, 20];
from(existingArray).subscribe(console.log);
```

## Gestion des erreurs

Bien que `range()` lui-même n'émette pas d'erreurs, des erreurs peuvent survenir dans le pipeline.

```typescript
import { range, of, map, catchError } from 'rxjs';
range(1, 10).pipe(
  map(n => {
    if (n === 5) {
      throw new Error('Erreur à 5');
    }
    return n * 2;
  }),
  catchError((error: unknown) => {
    console.error('Erreur survenue:', (error instanceof Error ? error.message : String(error)));
    return of(-1); // Retourner la valeur par défaut
  })
).subscribe(console.log);

// Sortie: 2, 4, 6, 8, -1
```

## Résumé

`range()` est une fonction de création simple mais puissante qui produit une séquence d'entiers consécutifs.

> [!IMPORTANT]
> **Caractéristiques de range()** :
> - ✅ Idéale pour générer des nombres consécutifs (alternative à l'instruction for)
> - ✅ Utile pour le traitement par lots, la pagination, la génération de données de test
> - ✅ Efficace en mémoire (pas de pré-création de tableaux)
> - ⚠️ Envisager l'asynchrone pour les grandes quantités de données
> - ⚠️ Utiliser `generate()` pour les conditions complexes

## Sujets associés

- [generate()](/fr/guide/creation-functions/loop/generate) - Génération de boucle générique
- [of()](/fr/guide/creation-functions/basic/of) - Énumère des valeurs arbitraires
- [from()](/fr/guide/creation-functions/basic/from) - Convertit depuis un tableau ou une Promise
- [interval()](/fr/guide/creation-functions/basic/interval) - Émet des valeurs périodiquement

## Références

- [RxJS Official : range()](https://rxjs.dev/api/index/function/range)
- [Learn RxJS : range](https://www.learnrxjs.io/learn-rxjs/operators/creation/range)
