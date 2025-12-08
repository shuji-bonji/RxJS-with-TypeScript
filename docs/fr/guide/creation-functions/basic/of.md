---
description: "of() - Fonction de création la plus simple qui émet des valeurs spécifiées en séquence : Parfait pour les données de test, les mocks, les valeurs par défaut et les branchements conditionnels"
---

# of() - Émission séquentielle de valeurs

`of()` est la fonction de création la plus simple qui émet les valeurs spécifiées une par une en séquence.

## Aperçu

`of()` émet les valeurs passées en argument dans l'ordre au fur et à mesure de l'abonnement, et se termine immédiatement après que toutes les valeurs ont été émises. Elle est fréquemment utilisée pour créer du code de test ou des données fictives.

**Signature** :
```typescript
function of<T>(...args: T[]): Observable<T>
```

**Documentation officielle** : [📘 RxJS Official : of()](https://rxjs.dev/api/index/function/of)

## Utilisation de base

`of()` permet de passer plusieurs valeurs séparées par des virgules.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Valeur:', value),
  error: err => console.error('Erreur:', err),
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

## Caractéristiques importantes

### 1. Émission synchrone

`of()` émet toutes les valeurs **de manière synchrone** lors de l'abonnement.

```typescript
import { of } from 'rxjs';

console.log('Avant abonnement');

of('A', 'B', 'C').subscribe(value => console.log('Valeur:', value));

console.log('Après abonnement');

// Sortie:
// Avant abonnement
// Valeur: A
// Valeur: B
// Valeur: C
// Après abonnement
```

### 2. Achèvement immédiat

Notifie `complete` immédiatement après avoir émis toutes les valeurs.

```typescript
import { of } from 'rxjs';

of(1, 2, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Terminé!')
});

// Sortie: 1, 2, 3, Terminé!
```

### 3. Peut émettre n'importe quel type de valeur

Des valeurs de n'importe quel type peuvent être émises, des types primitifs aux objets et tableaux.

```typescript
import { of } from 'rxjs';

// Types primitifs
of(42, 'hello', true).subscribe(console.log);

// Objets
of(
  { id: 1, name: 'Alice' },
  { id: 2, name: 'Bob' }
).subscribe(console.log);

// Tableaux (émet le tableau lui-même comme une seule valeur)
of([1, 2, 3], [4, 5, 6]).subscribe(console.log);
// Sortie: [1, 2, 3], [4, 5, 6]
```

### 4. Cold Observable

`of()` est un **Cold Observable**. Chaque abonnement initie une exécution indépendante.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3);

// Premier abonnement
values$.subscribe(val => console.log('Abonné A:', val));

// Deuxième abonnement (exécuté indépendamment)
values$.subscribe(val => console.log('Abonné B:', val));

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
> - Une exécution indépendante est lancée pour chaque abonnement
> - Chaque abonné reçoit son propre flux de données
> - Si vous avez besoin de partager des données, vous devez le rendre Hot avec `share()`, etc.
>
> Voir [Cold Observable et Hot Observable](/fr/guide/observables/cold-and-hot-observables) pour plus d'informations.

## Différence entre of() et from()

`of()` et `from()` ont un comportement différent lorsqu'il s'agit de tableaux. C'est un point de confusion courant.

```typescript
import { of, from } from 'rxjs';

// of() - émet le tableau comme une seule valeur
of([1, 2, 3]).subscribe(console.log);
// Sortie: [1, 2, 3]

// from() - émet chaque élément du tableau individuellement
from([1, 2, 3]).subscribe(console.log);
// Sortie: 1, 2, 3
```

> [!IMPORTANT]
> **Critères d'utilisation** :
> - Pour émettre le tableau lui-même → `of([1, 2, 3])`
> - Pour émettre chaque élément d'un tableau séparément → `from([1, 2, 3])`

## Cas d'utilisation pratiques

### 1. Données de test et création de mocks

`of()` est le plus souvent utilisé pour créer des données fictives dans le code de test.

```typescript
import { of } from 'rxjs';

// Données utilisateur fictives
function getMockUser$() {
  return of({
    id: 1,
    name: 'Utilisateur Test',
    email: 'test@example.com'
  });
}

// Utilisation dans les tests
getMockUser$().subscribe(user => {
  console.log('Utilisateur:', user.name); // Utilisateur: Utilisateur Test
});
```

### 2. Fournir des valeurs par défaut

Utilisé pour fournir des valeurs de repli en cas d'erreurs ou des valeurs par défaut.

```typescript
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

function fetchData(id: number) {
  if (id < 0) {
    return throwError(() => new Error('ID invalide'));
  }
  return of({ id, data: 'quelques données' });
}

fetchData(-1).pipe(
  catchError(err => {
    console.error('Erreur:', err.message);
    return of({ id: 0, data: 'données par défaut' }); // Valeur par défaut
  })
).subscribe(result => console.log(result));
// Sortie: Erreur: ID invalide
//         { id: 0, data: 'données par défaut' }
```

### 3. Émettre progressivement des valeurs multiples

Utilisé pour exécuter plusieurs étapes en séquence.

```typescript
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('Chargement...', 'Traitement...', 'Terminé!').pipe(
  concatMap(message => of(message).pipe(delay(1000)))
).subscribe(console.log);

// Sortie (toutes les 1 seconde):
// Chargement...
// Traitement...
// Terminé!
```

### 4. Valeurs de retour dans les branchements conditionnels

Utilisé en combinaison avec `iif()` et `switchMap()` pour retourner des valeurs selon des conditions.

```typescript
import { of, iif } from 'rxjs';

const isAuthenticated = true;

iif(
  () => isAuthenticated,
  of('Bon retour!'),
  of('Veuillez vous connecter')
).subscribe(console.log);
// Sortie: Bon retour!
```

## Utilisation dans un pipeline

`of()` est utilisé comme point de départ d'un pipeline ou pour injecter des données en cours de route.

```typescript
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

of(1, 2, 3, 4, 5).pipe(
  filter(n => n % 2 === 0),  // Nombres pairs uniquement
  map(n => n * 10)           // Multiplier par 10
).subscribe(console.log);
// Sortie: 20, 40
```

## Erreurs courantes

### 1. Passer un tableau directement

```typescript
// ❌ Incorrect - le tableau entier est émis comme une seule valeur
of([1, 2, 3]).subscribe(console.log);
// Sortie: [1, 2, 3]

// ✅ Correct - utiliser from() pour émettre chaque élément séparément
from([1, 2, 3]).subscribe(console.log);
// Sortie: 1, 2, 3

// ✅ Ou utiliser la syntaxe spread
of(...[1, 2, 3]).subscribe(console.log);
// Sortie: 1, 2, 3
```

### 2. Confusion avec le traitement asynchrone

Notez que `of()` émet de manière synchrone. Ce n'est pas un traitement asynchrone.

```typescript
// ❌ Ceci ne devient pas asynchrone
of(fetchDataFromAPI()).subscribe(console.log);
// fetchDataFromAPI() s'exécute immédiatement et l'objet Promise est émis

// ✅ Utiliser from() pour streamer une Promise
from(fetchDataFromAPI()).subscribe(console.log);
```

## Considérations sur les performances

`of()` est très léger et a peu d'impact sur les performances. Cependant, lors de l'émission d'un grand nombre de valeurs, gardez ce qui suit à l'esprit.

> [!TIP]
> Lors de l'émission d'un grand nombre de valeurs (des milliers ou plus) de manière séquentielle, envisagez d'utiliser `from()` ou `range()`.

## Fonctions de création associées

| Fonction | Différence | Utilisation |
|----------|------|----------|
| **[from()](/fr/guide/creation-functions/basic/from)** | Convertir depuis un tableau ou une Promise | Streamer des itérables ou des Promises |
| **range()** | Générer une plage de nombres | Émettre des nombres consécutifs |
| **EMPTY** | Terminer immédiatement sans rien émettre | Quand un flux vide est nécessaire |

## Résumé

- `of()` est la fonction de création la plus simple qui émet les valeurs spécifiées en séquence
- Émise de manière synchrone lors de l'abonnement et se termine instantanément
- Idéale pour les données de test et la création de mocks
- Si un tableau est passé, le tableau lui-même est émis (différent de `from()`)
- Utiliser `from()` pour le traitement asynchrone

## Prochaines étapes

- [from() - Convertir depuis un tableau, une Promise, etc.](/fr/guide/creation-functions/basic/from)
- [Fonctions de création de combinaison](/fr/guide/creation-functions/combination/)
- [Retour aux fonctions de création de base](/fr/guide/creation-functions/basic/)
