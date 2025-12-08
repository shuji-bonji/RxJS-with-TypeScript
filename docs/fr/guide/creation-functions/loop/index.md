---
description: "Cette section décrit les fonctions de création qui génèrent des valeurs à la manière d'une boucle, en utilisant range et generate pour apprendre à implémenter des traitements itératifs tels que les instructions for et while en tant que flux Observable. De la génération de nombres séquentiels aux transitions d'état complexes basées sur des conditions personnalisées, vous pouvez réaliser un traitement en boucle déclaratif en utilisant l'inférence de type de TypeScript."
---

# Fonctions de création de génération de boucles

Fonctions de création pour exprimer le traitement des boucles telles que les instructions for et while en tant qu'Observable.

## Que sont les fonctions de création de génération de boucle ?

Les fonctions de création de génération de boucle réalisent de manière réactive des traitements répétitifs. En remplaçant les boucles impératives conventionnelles (instructions `for` et `while`) par des flux Observable déclaratifs, un traitement flexible est possible en combinaison avec la chaîne d'opérateurs RxJS.

Consultez le tableau ci-dessous pour connaître les caractéristiques et l'utilisation de chaque fonction de création.

## Principales fonctions de création de génération de boucles

| Fonction | Description | Cas d'utilisation |
|----------|------|-------------|
| **[range](/fr/guide/creation-functions/loop/range)** | Génère une plage de nombres (comme l'instruction for) | Génération de nombres séquentiels, traitement par lots |
| **[generate](/fr/guide/creation-functions/loop/generate)** | Génération de boucle générique (comme l'instruction while) | Répétition conditionnelle, transitions d'état complexes |

## Critères d'utilisation

La sélection des fonctions de création de génération de boucle est déterminée par les perspectives suivantes.

### 1. Modèle de génération

- **Séquence numérique** : `range()` - Génération de nombres séquentiels simples avec des valeurs de début et de fin
- **Conditions complexes** : `generate()` - Contrôle libre des valeurs initiales, des conditions, de l'itération et de la sélection des résultats

### 2. Types de boucles

- **Boucle de type instruction for** : `range()` - `for (let i = start; i <= end; i++)`
- **Boucle de type instruction while** : `generate()` - `while (condition) { ... }`

### 3. Flexibilité

- **La simplicité suffit** : `range()` - Lorsque vous avez besoin d'une séquence de nombres
- **Besoin d'un contrôle avancé** : `generate()` - Gestion d'état personnalisée, branchement conditionnel, contrôle d'étape

## Exemples d'utilisation pratique

### range() - Génération de nombres séquentiels

Pour la génération de nombres séquentiels simples, `range()` est le meilleur choix.

```typescript
import { range, map } from 'rxjs';
// Générer des nombres séquentiels de 1 à 5
range(1, 5).subscribe(console.log);
// Sortie: 1, 2, 3, 4, 5

// Utilisation dans le traitement par lots
range(0, 10).pipe(
  map(i => `Traitement ${i + 1}`)
).subscribe(console.log);
// Sortie: Traitement 1, Traitement 2, ..., Traitement 10
```

### generate() - Boucle conditionnelle

Utilisez `generate()` pour des conditions complexes ou une gestion d'état personnalisée.

```typescript
import { generate } from 'rxjs';

// Générer la suite de Fibonacci (10 premiers termes)
generate(
  { current: 0, next: 1, count: 0 },  // État initial
  state => state.count < 10,           // Condition de continuation
  state => ({                          // Mise à jour de l'état
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // Sélecteur de résultat
).subscribe(console.log);
// Sortie: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## Comparaison avec la boucle impérative

Voici une comparaison entre la boucle impérative conventionnelle et les fonctions de création de génération de boucle de RxJS.

### Instruction for impérative

```typescript
// Instruction for conventionnelle
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### range() déclaratif

```typescript
import { range, map, toArray } from 'rxjs';
// RxJS range()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

> [!TIP]
> **Avantages de l'approche déclarative** :
> - Amélioration de la lisibilité grâce au traitement en pipeline
> - Gestion uniforme des erreurs
> - Facile à combiner avec le traitement asynchrone
> - Facile à annuler et interrompre (par ex., `takeUntil()`)

## Conversion de Cold vers Hot

Comme le montre le tableau ci-dessus, **toutes les fonctions de création de génération de boucles génèrent des Cold Observables**. Chaque abonnement initie une exécution indépendante.

Cependant, en utilisant les opérateurs de multicast (`share()`, `shareReplay()`, etc.), vous pouvez **convertir un Cold Observable en Hot Observable**.

### Exemple pratique : Partage des résultats de calcul

```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - Calcul indépendant pour chaque abonnement
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calcul:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('Abonné 1:', val));
cold$.subscribe(val => console.log('Abonné 2:', val));
// → Calcul exécuté deux fois (2000 calculs)

// 🔥 Hot - Partager les résultats de calcul entre les abonnés
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calcul:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('Abonné 1:', val));
hot$.subscribe(val => console.log('Abonné 2:', val));
// → Calcul exécuté une seule fois (1000 calculs)
```

> [!TIP]
> **Cas où la conversion en Hot est nécessaire** :
> - Utiliser des calculs coûteux à plusieurs endroits
> - Partager les résultats du traitement par lots avec plusieurs composants
> - Afficher les résultats de pagination dans plusieurs composants UI
>
> Pour plus d'informations, voir [Création de base - Conversion de Cold vers Hot](/fr/guide/creation-functions/basic/#conversion-de-cold-vers-hot).

## Combiné avec le traitement asynchrone

Les fonctions de création de génération de boucles présentent des fonctionnalités puissantes lorsqu'elles sont combinées au traitement asynchrone.

### Exécution séquentielle des appels API

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
    items: [`Données${page}-1`, `Données${page}-2`, `Données${page}-3`]
  }).pipe(
    delay(300) // Simuler un appel API
  );
}

// Récupérer séquentiellement les pages 1 à 10 (avec 1 seconde de délai entre chaque requête)
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe(
  data => console.log(`Page ${data.page} récupérée:`, data.items),
  err => console.error('Erreur:', err)
);
```

### Utilisation dans le traitement de réessai

```typescript
import { range, throwError, of, Observable, mergeMap, retryWhen, delay } from 'rxjs';
// Fonction pour simuler la récupération de données (échoue aléatoirement)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // Taux de succès de 40%

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Échec de récupération des données'))
        : of('Récupération des données réussie')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          // Réessayer jusqu'à 3 fois
          if (index >= 3) {
            return throwError(() => error);
          }
          console.log(`Réessai ${index + 1}/3`);
          // Backoff exponentiel: 1s, 2s, 4s
          return range(0, 1).pipe(delay(Math.pow(2, index) * 1000));
        })
      )
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Résultat:', result),
  error: err => console.error('Erreur:', err.message)
});

// Exemple de sortie:
// Réessai 1/3
// Réessai 2/3
// Résultat: Récupération des données réussie
```

## Relation avec les opérateurs pipables

Les fonctions de création de génération de boucles n'ont pas d'équivalent direct en opérateur pipable. Elles sont toujours utilisées en tant que fonctions de création.

Cependant, un traitement plus avancé est possible en les combinant avec les opérateurs suivants :

| Opérateurs à combiner | Objectif |
|-------------------|------|
| `map()` | Transformer chaque valeur |
| `filter()` | Ne laisser passer que les valeurs qui correspondent à la condition |
| `take()`, `skip()` | Contrôler le nombre de valeurs |
| `concatMap()`, `mergeMap()` | Exécuter un traitement asynchrone pour chaque valeur |
| `toArray()` | Rassembler toutes les valeurs dans un tableau |

## Notes sur les performances

Les fonctions de création de génération de boucles émettent des valeurs de manière synchrone, il faut donc faire attention aux performances lors de la génération d'un grand nombre de valeurs.

> [!WARNING]
> **Gestion de grandes quantités de données** :
> - Les grandes quantités de données, telles que `range(1, 1000000)`, sont toutes émises de manière synchrone et consomment de la mémoire
> - Mettez en mémoire tampon avec `bufferCount()` ou `windowCount()` selon les besoins
> - Ou passez à une exécution asynchrone en spécifiant un planificateur avec `scheduled()`

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// Exécuter avec un planificateur asynchrone
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## Prochaines étapes

Pour en savoir plus sur le comportement détaillé et les exemples pratiques de chaque fonction de création, cliquez sur les liens du tableau ci-dessus.

Vous pouvez également comprendre l'ensemble des fonctions de création en apprenant les [Fonctions de création de base](/fr/guide/creation-functions/basic/), les [Fonctions de création de combinaison](/fr/guide/creation-functions/combination/), les [Fonctions de création de sélection/partition](/fr/guide/creation-functions/selection/) et les [Fonctions de création conditionnelles](/fr/guide/creation-functions/conditional/).
