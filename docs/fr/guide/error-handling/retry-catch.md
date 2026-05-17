---
description: "Explique les stratégies robustes de gestion des erreurs combinant les opérateurs retry et catchError. Apprenez à implémenter de manière type-safe avec TypeScript les retries pour défaillances temporaires, les patterns de backoff exponentiel, les retries conditionnels et le traitement approprié des fallback à travers des exemples de code pratiques."
---
# retry + catchError

Nous allons expliquer en détail les deux opérateurs centraux de la gestion des erreurs dans RxJS : `retry` et `catchError`. En les combinant, vous pouvez réaliser une stratégie robuste de gestion des erreurs.

## retry - Retry en cas d'échec (Pattern de base)

L'opérateur `retry` est un opérateur qui **relance l'exécution du stream un nombre spécifié de fois** lorsqu'une erreur se produit dans le stream. Il est particulièrement efficace pour les opérations susceptibles d'échouer temporairement, comme les requêtes réseau.

[🌐 Documentation officielle RxJS - retry](https://rxjs.dev/api/index/function/retry)

### Pattern de base

```ts
import { Observable, of } from 'rxjs';
import { retry, map } from 'rxjs';

// Fonction qui génère des erreurs aléatoires
function getDataWithRandomError(): Observable<string> {
  return of('données').pipe(
    map(() => {
      if (Math.random() < 0.7) {
        throw new Error('Erreur aléatoire survenue');
      }
      return 'Données récupérées avec succès!';
    })
  );
}

// Maximum 3 retries
getDataWithRandomError()
  .pipe(retry(3))
  .subscribe({
    next: (data) => console.log('Succès:', data),
    error: (err) => console.error('Erreur (après 3 retries):', err.message),
  });

// Sortie:
// Succès: Données récupérées avec succès!
// Erreur (après 3 retries): Erreur aléatoire survenue ⇦ Affiché quand échoue 3 fois
```

### Surveillance des retries en temps réel

```ts
import { Observable, of } from 'rxjs';
import { retry, tap, catchError, map } from 'rxjs';

let attempts = 0;

function simulateFlakyRequest(): Observable<string> {
  return of('requête').pipe(
    tap(() => {
      attempts++;
      console.log(`Tentative #${attempts}`);
    }),
    map(() => {
      if (attempts < 3) {
        throw new Error(`Erreur #${attempts}`);
      }
      return 'Succès!';
    })
  );
}

simulateFlakyRequest()
  .pipe(
    retry(3),
    catchError((error: unknown) => {
      console.log('Tous les retries ont échoué:', (error instanceof Error ? error.message : String(error)));
      return of('Valeur de fallback');
    })
  )
  .subscribe({
    next: (result) => console.log('Résultat final:', result),
    complete: () => console.log('Terminé'),
  });

// Sortie:
// Tentative #1
// Tentative #2
// Tentative #3
// Résultat final: Succès!
// Terminé
```

> [!NOTE] Timing des retries et Scheduler
> Lorsque vous spécifiez un délai avec l'opérateur `retry` (comme `retry({ delay: 1000 })`), **asyncScheduler** est utilisé en interne. En utilisant les schedulers, vous pouvez contrôler finement le timing des retries ou utiliser le temps virtuel lors des tests.
>
> Pour plus de détails, consultez [Types de Schedulers et choix d'utilisation - Contrôle des retries d'erreur](/fr/guide/schedulers/types#contrôle-des-retries-derreur).

## catchError - Capture d'erreur et traitement alternatif (Pattern de base)

L'opérateur `catchError` capture les erreurs survenues dans le stream et les traite en **retournant un Observable alternatif**. Cela permet au stream de continuer le traitement sans être interrompu même si une erreur se produit.

[🌐 Documentation officielle RxJS - catchError](https://rxjs.dev/api/index/function/catchError)

### Pattern de base

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('Erreur d\'appel API')) // RxJS 7+, forme de fonction recommandée
  .pipe(
    catchError((error: unknown) => {
      console.error('Erreur survenue:', (error instanceof Error ? error.message : String(error)));
      return of('Valeur par défaut en cas d\'erreur');
    })
  )
  .subscribe({
    next: (value) => console.log('Valeur:', value),
    complete: () => console.log('Terminé'),
  });

// Sortie:
// Erreur survenue: Erreur d'appel API
// Valeur: Valeur par défaut en cas d'erreur
// Terminé
```

### Re-throw de l'erreur

Quand vous voulez enregistrer l'erreur puis la re-throw

```ts
import { throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('Erreur originale')) // RxJS 7+, forme de fonction recommandée
  .pipe(
    catchError((error: unknown) => {
      console.error('Erreur enregistrée:', (error instanceof Error ? error.message : String(error)));
      // Re-throw de l'erreur
      return throwError(() => new Error('Erreur transformée'));
    })
  )
  .subscribe({
    next: (value) => console.log('Valeur:', value),
    error: (err) => console.error('Erreur finale:', err.message),
    complete: () => console.log('Terminé'),
  });

// Sortie:
// Erreur enregistrée: Erreur originale
// Erreur finale: Erreur transformée
```

## Combinaison de retry et catchError

Dans les applications réelles, il est courant d'utiliser `retry` et `catchError` ensemble. Cette combinaison permet de résoudre les erreurs temporaires par les retries tout en fournissant une valeur de fallback en cas d'échec final.

```ts
import { of, throwError } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

function fetchData() {
  // Observable qui génère une erreur
  return throwError(() => new Error('Erreur réseau')) // RxJS 7+, forme de fonction recommandée
    .pipe(
    // Pour le débogage
    tap(() => console.log('Tentative de récupération des données')),
    // Maximum 3 retries
    retry(3),
    // Quand tous les retries échouent
    catchError((error: unknown) => {
      console.error('Tous les retries ont échoué:', (error instanceof Error ? error.message : String(error)));
      // Retourne une valeur par défaut
      return of({
        error: true,
        data: null,
        message: 'Échec de la récupération des données',
      });
    })
  );
}

fetchData().subscribe({
  next: (result) => console.log('Résultat:', result),
  complete: () => console.log('Traitement terminé'),
});

// Sortie:
// Tous les retries ont échoué: Erreur réseau
// Résultat: {error: true, data: null, message: 'Échec de la récupération des données'}
// Traitement terminé
```

## Stratégie avancée de retry : retry({ count, delay })

Si vous avez besoin de stratégies de retry plus flexibles, utilisez la **forme objet de configuration** de l'opérateur `retry` (depuis RxJS 7.3). Cela permet de spécifier de manière déclarative le nombre de retries et la logique de délai.

> [!IMPORTANT] retryWhen est déprécié
> L'ancien opérateur `retryWhen` (déprécié en v7.3, prévu pour suppression en v8) doit être remplacé par la forme `retry({ count, delay })` présentée dans cette section. Le callback `delay` reçoit `(error, retryCount)` en arguments et retourne un `Observable` (par exemple `timer(ms)`) pour contrôler le temps d'attente avant la nouvelle tentative.

[🌐 Documentation officielle RxJS - retry](https://rxjs.dev/api/index/function/retry)

### Retry avec backoff exponentiel

Pour les retries de requêtes réseau, le pattern de backoff exponentiel (augmentation progressive de l'intervalle entre les retries) est courant. Cela permet de réduire la charge sur le serveur tout en attendant que les problèmes temporaires se résolvent.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, catchError } from 'rxjs';

function fetchWithRetry() {
  return throwError(() => new Error('Erreur réseau')).pipe(
    // Recommandé en RxJS 7.3+ : forme retry({ count, delay })
    retry({
      count: 5, // Maximum 5 retries
      delay: (error, retryCount) => {
        console.log('Erreur survenue:', error.message);
        // Backoff exponentiel (plafonné à 10 secondes)
        const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
        console.log(`Retry ${retryCount} dans ${delayMs}ms`);
        // timer utilise asyncScheduler en interne
        return timer(delayMs);
      }
    }),
    // Fallback final
    catchError((error: unknown) => {
      console.error('Tous les retries ont échoué:', (error instanceof Error ? error.message : String(error)));
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

> [!TIP] Contrôle avancé des retries avec les Schedulers
> L'exemple ci-dessus utilise `timer()`, mais pour un contrôle plus avancé, vous pouvez spécifier explicitement un scheduler pour ajuster finement le timing des retries ou utiliser le temps virtuel lors des tests.
>
> Pour plus de détails, consultez [Types de Schedulers et choix d'utilisation - Contrôle des retries d'erreur](/fr/guide/schedulers/types#contrôle-des-retries-derreur).

## Débogage des retries

Lors du débogage des retries, il est important de suivre le nombre de tentatives et le résultat de chaque tentative. Voici des méthodes pratiques pour surveiller l'état des retries en temps réel.

### Méthode 1: Callback error de tap (Basique)

En utilisant le callback `error` de l'opérateur `tap`, vous pouvez compter les tentatives lors des erreurs.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Erreur temporaire'))
  .pipe(
    tap({
      error: () => {
        attemptCount++;
        console.log(`Nombre de tentatives: ${attemptCount}`);
      }
    }),
    retry(2),
    catchError((error: unknown) => {
      console.log(`Tentatives finales: ${attemptCount}`);
      return of(`Erreur finale: ${(error instanceof Error ? error.message : String(error))}`);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Erreur de souscription:', err)
  });

// Sortie:
// Nombre de tentatives: 1
// Nombre de tentatives: 2
// Nombre de tentatives: 3
// Tentatives finales: 3
// Erreur finale: Erreur temporaire
```

> [!NOTE] Limitation avec throwError
> `throwError` émet immédiatement une erreur sans émettre de valeur, donc le callback `next` de `tap` n'est pas exécuté. Vous devez utiliser le callback `error`.

### Méthode 2 : Suivi détaillé avec retry({ count, delay }) (Recommandé)

Pour suivre des informations plus détaillées (nombre de tentatives, temps de délai, contenu de l'erreur), utilisez la forme objet de configuration de `retry` (depuis v7.3, remplace `retryWhen`).

```typescript
import { throwError, of, timer, retry, catchError } from 'rxjs';

throwError(() => new Error('Erreur temporaire'))
  .pipe(
    retry({
      count: 2, // Maximum 2 retries
      delay: (error, retryCount) => {
        console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
        console.log(`🔄 Retry ${retryCount}`);
        console.log(`   Erreur: ${error.message}`);

        const delayMs = 1000;
        console.log(`⏳ Nouvelle tentative dans ${delayMs}ms...`);
        console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

        return timer(delayMs);
      }
    }),
    catchError((error: unknown) => {
      console.log(`\nRésultat final: Tous les retries ont échoué`);
      return of(`Erreur finale: ${(error instanceof Error ? error.message : String(error))}`);
    })
  )
  .subscribe(result => console.log('Résultat:', result));

// Sortie:
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 1
//    Erreur: Erreur temporaire
// ⏳ Nouvelle tentative dans 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (attente 1 seconde)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 2
//    Erreur: Erreur temporaire
// ⏳ Nouvelle tentative dans 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (attente 1 seconde)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 3
//    Erreur: Erreur temporaire
// ❌ Nombre maximum de retries atteint
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
//
// Résultat final: Tous les retries ont échoué
// Résultat: Erreur finale: Erreur temporaire
```

### Méthode 3: Suivi des tentatives avec un Observable personnalisé

Pour les requêtes API réelles et autres Observables qui émettent des valeurs, vous pouvez gérer le nombre de tentatives avec un Observable personnalisé.

```typescript
import { Observable, of, retry, catchError } from 'rxjs';
let attemptCount = 0;

// Observable qui peut compter les tentatives
const retryableStream$ = new Observable(subscriber => {
  attemptCount++;
  console.log(`[Tentative ${attemptCount}]`);

  // Échoue les 2 premières fois, réussit la 3ème
  if (attemptCount < 3) {
    subscriber.error(new Error(`Échec (tentative ${attemptCount})`));
  } else {
    subscriber.next('Données de succès');
    subscriber.complete();
  }
});

retryableStream$
  .pipe(
    retry(2),
    catchError((error: unknown) => {
      console.log(`[Terminé] Échec après ${attemptCount} tentatives`);
      return of(`Erreur finale: ${(error instanceof Error ? error.message : String(error))}`);
    })
  )
  .subscribe({
    next: data => console.log('[Résultat]', data),
    complete: () => console.log('[Terminé]')
  });

// Sortie:
// [Tentative 1]
// [Tentative 2]
// [Tentative 3]
// [Résultat] Données de succès
// [Terminé]
```

### Méthode 4: Backoff exponentiel et journalisation

Pattern de journalisation détaillée pour les requêtes API pratiques.

```typescript
import { timer, of, retry, catchError, finalize } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchWithRetryLogging(url: string, maxRetries = 3) {
  let startTime = Date.now();

  return ajax.getJSON(url).pipe(
    retry({
      count: maxRetries,
      delay: (error, retryCount) => {
        const elapsed = Date.now() - startTime;

        console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
        console.log(`🔄 Informations de retry`);
        console.log(`   Nombre: ${retryCount}/${maxRetries}`);
        console.log(`   Erreur: ${error.message || error.status}`);
        console.log(`   Temps écoulé: ${elapsed}ms`);

        // Backoff exponentiel
        const delayMs = Math.min(1000 * Math.pow(2, retryCount - 1), 10000);
        console.log(`⏳ Nouvelle tentative dans ${delayMs}ms...`);
        console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

        return timer(delayMs);
      }
    }),
    catchError((error: unknown) => {
      const totalTime = Date.now() - startTime;
      console.log(`\n❌ Échec final (temps total: ${totalTime}ms)`);
      return of({ error: true, message: 'Échec de récupération des données' });
    }),
    finalize(() => {
      const totalTime = Date.now() - startTime;
      console.log(`\n✅ Traitement terminé (temps total: ${totalTime}ms)`);
    })
  );
}

// Exemple d'utilisation
fetchWithRetryLogging('https://jsonplaceholder.typicode.com/users/1').subscribe({
  next: data => console.log('Données:', data),
  error: err => console.error('Erreur:', err)
});
```

### Méthode 5: Objet de configuration retry de RxJS 7.4+

À partir de RxJS 7.4, vous pouvez passer un objet de configuration à `retry`.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Erreur temporaire'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Tentative ${attemptCount}`);
      },
      error: (err) => console.log(`Erreur survenue:`, err.message)
    }),
    retry({
      count: 2,
      delay: 1000, // Attendre 1 seconde avant retry (utilise asyncScheduler en interne)
      resetOnSuccess: true
    }),
    catchError((error: unknown) => {
      console.log(`Échec final (${attemptCount} tentatives au total)`);
      return of(`Erreur finale: ${(error instanceof Error ? error.message : String(error))}`);
    })
  )
  .subscribe(result => console.log('Résultat:', result));

// Sortie:
// Tentative 1
// Erreur survenue: Erreur temporaire
// Tentative 2
// Erreur survenue: Erreur temporaire
// Tentative 3
// Erreur survenue: Erreur temporaire
// Échec final (3 tentatives au total)
// Résultat: Erreur finale: Erreur temporaire
```

> [!TIP] Approches recommandées pour le débogage des retries
> - **En développement**: Méthode 2 (retryWhen) ou Méthode 4 (logs détaillés) sont optimales
> - **En production**: Basé sur la Méthode 4, ajouter l'envoi de logs à un service de surveillance des erreurs
> - **Cas simples**: Méthode 1 (error de tap) ou Méthode 5 (configuration retry) suffisent
>
> **Informations connexes**:
> - Pour le contrôle du timing des retries, consultez [Types de Schedulers et choix d'utilisation - Contrôle des retries d'erreur](/fr/guide/schedulers/types#contrôle-des-retries-derreur)
> - Pour une vue d'ensemble des techniques de débogage, consultez [Techniques de débogage RxJS - Suivi du nombre de tentatives de retry](/fr/guide/debugging/#scénario-6-suivre-le-nombre-de-tentatives-de-retry)

## Exemple d'utilisation en application réelle: Requête API

Exemple d'utilisation de ces opérateurs dans une requête API réelle.

```ts
import { Observable, of } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { retry, catchError, finalize, tap } from 'rxjs';

// État de chargement
let isLoading = false;

function fetchUserData(userId: string): Observable<any> {
  isLoading = true;

  return ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`).pipe(
    // Débogage de la requête
    tap((response) => console.log('Réponse API:', response)),
    // Maximum 2 retries pour les erreurs réseau
    retry(2),
    // Gestion des erreurs
    catchError((error: unknown) => {
      if (error.status === 404) {
        return of({ error: true, message: 'Utilisateur non trouvé' });
      } else if (error.status >= 500) {
        return of({ error: true, message: 'Une erreur serveur s\'est produite' });
      }
      return of({ error: true, message: 'Une erreur inconnue s\'est produite' });
    }),
    // Exécuté dans tous les cas (succès ou échec)
    finalize(() => {
      isLoading = false;
      console.log('Chargement terminé');
    })
  );
}

// Exemple d'utilisation
fetchUserData('123').subscribe({
  next: (data) => {
    if (data.error) {
      // Affichage des informations d'erreur
      console.error('Erreur:', data.message);
    } else {
      // Affichage des données
      console.log('Données utilisateur:', data);
    }
  },
});


// Sortie:
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// Une erreur inconnue s'est produite
// Chargement terminé
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
```

## Bonnes pratiques

### Quand utiliser retry

- Quand des **erreurs temporaires** sont attendues (problèmes de connexion réseau, etc.)
- **Problèmes temporaires côté serveur** (forte charge ou timeout, etc.)
- Erreurs qui peuvent être **résolues par retry**

### Quand ne pas utiliser retry

- **Erreurs d'authentification** (401, 403) - ne seront pas résolues par retry
- **Ressource inexistante** (404) - ne sera pas trouvée par retry
- **Erreurs de validation** (400) - le problème est dans la requête elle-même
- **Erreurs de programme côté client** - les retries sont inutiles

### Utilisation efficace de catchError

- Effectuer **différents traitements selon le type** d'erreur
- Fournir des **messages compréhensibles** à l'utilisateur
- Retourner des **données de fallback** quand approprié
- **Transformer les erreurs** si nécessaire

## Résumé

En combinant `retry` et `catchError`, une gestion robuste des erreurs devient possible. Les erreurs temporaires tentent une récupération par retry, et les erreurs permanentes reçoivent un traitement de fallback approprié, améliorant ainsi l'expérience utilisateur. Dans les applications réelles, il est important de choisir la stratégie appropriée selon la nature de l'erreur et de fournir des mécanismes de fallback.

La section suivante expliquera l'opérateur `finalize` pour la libération des ressources et le traitement de complétion des streams.
