---
description: "Explique les stratégies complètes de gestion des erreurs RxJS. Présente comment combiner les opérateurs catchError, retry, retryWhen, finalize, les retries avec backoff exponentiel, la classification des erreurs et leur traitement approprié, les gestionnaires d'erreurs globaux, et comment implémenter une gestion robuste des erreurs avec TypeScript."
---
# Stratégies de gestion des erreurs RxJS

La gestion des erreurs dans RxJS est un aspect important de la programmation réactive. En implémentant une gestion appropriée des erreurs, vous améliorez la robustesse et la fiabilité de votre application. Ce document explique les différentes stratégies de gestion des erreurs disponibles dans RxJS.

## Patterns de base

Dans RxJS, les erreurs sont gérées dans le cadre du cycle de vie de l'Observable. Il existe plusieurs méthodes de base pour la gestion des erreurs.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

// Observable qui génère une erreur
const error$ = throwError(() => new Error('Une erreur s\'est produite')); // RxJS 7+, forme de fonction recommandée

// Gestion basique des erreurs
error$
  .pipe(
    catchError((error) => {
      console.error('Erreur capturée:', error.message);
      return of('Valeur de repli après erreur');
    })
  )
  .subscribe({
    next: (value) => console.log('Valeur:', value),
    error: (err) => console.error('Erreur non gérée:', err),
    complete: () => console.log('Terminé'),
  });

// Sortie:
// Erreur capturée: Une erreur s'est produite
// Valeur: Valeur de repli après erreur
// Terminé
```

## Diverses stratégies de gestion des erreurs

### 1. Capturer l'erreur et fournir une valeur alternative

Utilisez l'opérateur `catchError` pour capturer l'erreur et fournir une valeur ou un flux alternatif.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

const source$ = throwError(() => new Error('Erreur de récupération des données'));

source$.pipe(
  catchError(error => {
    console.error('Erreur survenue:', error.message);
    // Retourne des données alternatives
    return of({ isError: true, data: [], message: 'Affichage des données par défaut' });
  })
).subscribe(data => console.log('Résultat:', data));

// Sortie:
// Erreur survenue: Erreur de récupération des données
// Résultat: {isError: true, data: Array(0), message: 'Affichage des données par défaut'}
```

### 2. Réessayer en cas d'erreur

Utilisez les opérateurs `retry` ou `retryWhen` pour réessayer le flux en cas d'erreur.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, tap } from 'rxjs';

let attemptCount = 0;

interval(1000).pipe(
  mergeMap(val => {
    if (++attemptCount <= 2) {
      return throwError(() => new Error(`Erreur #${attemptCount}`));
    }
    return of('Succès!');
  }),
  tap(() => console.log('Exécution:', attemptCount)),
  retry(2), // Maximum 2 retries
).subscribe({
  next: value => console.log('Valeur:', value),
  error: err => console.error('Erreur finale:', err.message),
});

// Sortie:
// Exécution: 3
// Valeur: Succès!
// Exécution: 4
// Valeur: Succès!
// Exécution: 5
// ...
```

### 3. Retry avec backoff exponentiel

Pour les requêtes réseau, le « backoff exponentiel » qui augmente progressivement l'intervalle entre les retries est efficace.

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, tap, concatMap, catchError } from 'rxjs';

function fetchWithRetry() {
  let retryCount = 0;

  return throwError(() => new Error('Erreur réseau')).pipe(
    retryWhen((errors) =>
      errors.pipe(
        // Compter les erreurs
        tap((error) => console.log('Erreur survenue:', error.message)),
        // Délai avec backoff exponentiel
        concatMap(() => {
          retryCount++;
          const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
          console.log(`Retry ${retryCount} dans ${delayMs}ms`);
          return timer(delayMs);
        }),
        // Maximum 5 retries
        tap(() => {
          if (retryCount >= 5) {
            throw new Error('Nombre maximum de retries atteint');
          }
        })
      )
    ),
    // Repli final
    catchError((error) => {
      console.error('Tous les retries ont échoué:', error.message);
      return of({
        error: true,
        message: 'La connexion a échoué. Veuillez réessayer plus tard.',
      });
    })
  );
}

fetchWithRetry().subscribe({
  next: (result) => console.log('Résultat:', result),
  error: (err) => console.error('Erreur non gérée:', err),
});

// Sortie:
// Erreur survenue: Erreur réseau
// Retry 1 dans 2000ms
// Erreur survenue: Erreur réseau
// Retry 2 dans 4000ms
// ...
```

### 4. Libération des ressources en cas d'erreur

Utilisez l'opérateur `finalize` pour libérer les ressources lorsque le flux se **termine ou génère une erreur**.
finalize est efficace lorsque vous voulez effectuer un nettoyage de manière fiable non seulement en cas d'erreur mais aussi lors de la complétion normale.

```ts
import { throwError } from 'rxjs';
import { catchError, finalize } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Erreur de traitement'))
  .pipe(
    catchError((error) => {
      console.error('Gestion de l\'erreur:', error.message);
      return throwError(() => error); // Re-throw l'erreur
    }),
    finalize(() => {
      isLoading = false;
      console.log('État de chargement réinitialisé:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Valeur:', value),
    error: (err) => console.error('Erreur finale:', err.message),
    complete: () => console.log('Terminé'),
  });

// Sortie:
// Gestion de l'erreur: Erreur de traitement
// Erreur finale: Erreur de traitement
// État de chargement réinitialisé: false
```

## Patterns de gestion des erreurs

### Gestion des erreurs avec contrôle d'affichage UI

```ts
import { of, throwError } from 'rxjs';
import { catchError, finalize, tap } from 'rxjs';

function fetchData(shouldFail = false) {
  // Afficher l'indicateur de chargement
  showLoadingIndicator();

  // Récupération des données (succès ou erreur)
  return (
    shouldFail
      ? throwError(() => new Error('Erreur API'))
      : of({ name: 'Données', value: 42 })
  ).pipe(
    tap((data) => {
      // Traitement en cas de succès
      updateUI(data);
    }),
    catchError((error) => {
      // Mise à jour de l'UI en cas d'erreur
      showErrorMessage(error.message);
      // Retourne des données vides ou par défaut
      return of({ name: 'Par défaut', value: 0 });
    }),
    finalize(() => {
      // Masquer l'indicateur de chargement quel que soit le résultat
      hideLoadingIndicator();
    })
  );
}

// Fonctions utilitaires pour l'UI
function showLoadingIndicator() {
  console.log('Affichage du chargement');
}
function hideLoadingIndicator() {
  console.log('Masquage du chargement');
}
function updateUI(data: { name: string; value: number }) {
  console.log('Mise à jour de l\'UI:', data);
}
function showErrorMessage(message: any) {
  console.log('Affichage de l\'erreur:', message);
}

// Utilisation
fetchData(true).subscribe();

// Sortie:
// Affichage du chargement
// Affichage de l'erreur: Erreur API
// Masquage du chargement
```

### Gestion de plusieurs sources d'erreur

```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, map } from 'rxjs';

// Simulation de plusieurs requêtes API
function getUser() {
  return of({ id: 1, name: 'Jean Dupont' });
}

function getPosts() {
  return throwError(() => new Error('Erreur de récupération des posts'));
}

function getComments() {
  return throwError(() => new Error('Erreur de récupération des commentaires'));
}

// Récupérer toutes les données en tolérant les erreurs partielles
forkJoin({
  user: getUser().pipe(
    catchError((error) => {
      console.error('Erreur utilisateur:', error.message);
      return of(null); // Retourne null en cas d'erreur
    })
  ),
  posts: getPosts().pipe(
    catchError((error) => {
      console.error('Erreur posts:', error.message);
      return of([]); // Retourne un tableau vide en cas d'erreur
    })
  ),
  comments: getComments().pipe(
    catchError((error) => {
      console.error('Erreur commentaires:', error.message);
      return of([]); // Retourne un tableau vide en cas d'erreur
    })
  ),
})
  .pipe(
    map((result) => ({
      ...result,
      // Ajouter un flag indiquant s'il y a eu des erreurs partielles
      hasErrors:
        !result.user ||
        result.posts.length === 0 ||
        result.comments.length === 0,
    }))
  )
  .subscribe((data) => {
    console.log('Résultat final:', data);

    if (data.hasErrors) {
      console.log(
        'Certaines données n\'ont pas pu être récupérées, mais les données disponibles sont affichées'
      );
    }
  });

// Sortie:
// Erreur posts: Erreur de récupération des posts
// Erreur commentaires: Erreur de récupération des commentaires
// Résultat final: {user: {…}, posts: Array(0), comments: Array(0), hasErrors: true}
// Certaines données n'ont pas pu être récupérées, mais les données disponibles sont affichées
```

## Bonnes pratiques de gestion des erreurs

1. **Toujours capturer les erreurs**: Dans une chaîne Observable, ajoutez toujours une gestion des erreurs. C'est particulièrement important pour les flux de longue durée.

2. **Fournir des messages d'erreur significatifs**: Les objets d'erreur doivent contenir des informations utiles pour identifier l'emplacement et la cause.

3. **Libérer correctement les ressources**: Utilisez `finalize` pour vous assurer que les ressources sont libérées quel que soit le résultat.

4. **Considérer les stratégies de retry**: Pour les opérations réseau en particulier, implémenter des stratégies de retry appropriées améliore la fiabilité.

5. **Gestion conviviale des erreurs**: Dans l'UI, ne pas afficher les messages d'erreur techniques tels quels, mais fournir des informations compréhensibles par l'utilisateur.

```ts
// Exemple: Conversion en messages d'erreur conviviaux
function getErrorMessage(error: any): string {
  if (error.status === 401) {
    return 'La session a expiré. Veuillez vous reconnecter.';
  } else if (error.status === 404) {
    return 'La ressource demandée n\'a pas été trouvée.';
  } else if (error.status >= 500) {
    return 'Une erreur serveur s\'est produite. Veuillez réessayer plus tard.';
  }
  return 'Une erreur inattendue s\'est produite.';
}
```

## Résumé

La gestion des erreurs dans RxJS est une partie importante pour assurer la robustesse de l'application. En combinant correctement des opérateurs comme `catchError`, `retry`, `finalize`, vous pouvez gérer divers scénarios d'erreur. Ne vous contentez pas de capturer les erreurs, concevez une stratégie de gestion complète pour améliorer l'expérience utilisateur.

## Sections connexes

- **[Erreurs courantes et solutions](/fr/guide/anti-patterns/common-mistakes#9-étouffer-les-erreurs)** - Vérifier les anti-patterns de gestion des erreurs
- **[retry et catchError](/fr/guide/error-handling/retry-catch)** - Utilisation détaillée expliquée

