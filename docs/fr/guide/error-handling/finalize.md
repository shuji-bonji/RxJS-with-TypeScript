---
description: "Explique comment effectuer efficacement le traitement de complétion des streams et la libération des ressources dans RxJS en utilisant finalize et complete. Présente des patterns pratiques incluant la prévention des fuites mémoire, la libération des handles de fichiers, le nettoyage des connexions WebSocket, la réinitialisation de l'état UI. Explique également la différence avec la clause finally."
---
# finalize et complete - Libération des ressources et traitement de complétion des streams

Dans RxJS, il est important de gérer correctement la terminaison des streams et la libération des ressources. Cette page explique l'opérateur `finalize` et le mécanisme de notification `complete`.

## finalize - Opérateur pour la libération des ressources

L'opérateur `finalize` est un opérateur qui exécute le code de nettoyage spécifié lorsque l'Observable **se termine par complétion, erreur ou désabonnement**.
finalize est **appelé exactement une fois à la terminaison du stream** et n'est pas appelé plusieurs fois.

[🌐 Documentation officielle RxJS - finalize](https://rxjs.dev/api/index/function/finalize)

### Utilisation de base de finalize

```ts
import { of } from 'rxjs';
import { finalize, tap } from 'rxjs';

// Variable pour gérer l'état de chargement
let isLoading = true;

// Stream qui réussit
of('données')
  .pipe(
    tap((data) => console.log('Traitement des données:', data)),
    // Exécuté dans tous les cas (succès, échec, annulation)
    finalize(() => {
      isLoading = false;
      console.log('État de chargement réinitialisé:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Valeur:', value),
    complete: () => console.log('Terminé'),
  });

// Sortie:
// Traitement des données: données
// Valeur: données
// Terminé
// État de chargement réinitialisé: false
```

### finalize en cas d'erreur

```ts
import { throwError } from 'rxjs';
import { finalize, catchError } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Erreur de récupération des données'))
  .pipe(
    catchError((err) => {
      console.error('Traitement de l\'erreur:', err.message);
      throw err; // Re-throw de l'erreur
    }),
    finalize(() => {
      isLoading = false;
      console.log('Libération des ressources après erreur:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Valeur:', value),
    error: (err) => console.error('Erreur dans le subscriber:', err.message),
    complete: () => console.log('Terminé'), // Non appelé en cas d'erreur
  });

// Sortie:
// Traitement de l'erreur: Erreur de récupération des données
// Erreur dans le subscriber: Erreur de récupération des données
// Libération des ressources après erreur: false
```

### finalize lors du désabonnement

```ts
import { interval } from 'rxjs';
import { finalize } from 'rxjs';

let resource = 'Actif';

// Compte toutes les secondes
const subscription = interval(1000)
  .pipe(
    finalize(() => {
      resource = 'Libéré';
      console.log('État de la ressource:', resource);
    })
  )
  .subscribe((count) => {
    console.log('Compteur:', count);

    // Désabonnement manuel après 3 comptages
    if (count >= 2) {
      subscription.unsubscribe();
    }
  });

// Sortie:
// Compteur: 0
// Compteur: 1
// Compteur: 2
// État de la ressource: Libéré
```

finalize est efficace lorsque vous voulez effectuer un traitement de nettoyage de manière fiable non seulement en cas d'erreur mais aussi lors de la complétion normale ou du désabonnement manuel.

## complete - Notification de terminaison normale du stream

Lorsqu'un Observable se termine normalement, le callback `complete` de l'Observer est appelé. C'est la dernière étape du cycle de vie de l'Observable.

### complete automatique

Certains Observables se terminent automatiquement lorsque certaines conditions sont remplies.

```ts
import { of } from 'rxjs';
import { take } from 'rxjs';

// Les séquences finies se terminent automatiquement
of(1, 2, 3).subscribe({
  next: (value) => console.log('Valeur:', value),
  complete: () => console.log('Stream fini terminé'),
});

// Stream limité avec interval + take
interval(1000)
  .pipe(
    take(3) // Terminé après 3 valeurs
  )
  .subscribe({
    next: (value) => console.log('Compteur:', value),
    complete: () => console.log('Stream limité terminé'),
  });

// Sortie:
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Stream fini terminé
// Compteur: 0
// Compteur: 1
// Compteur: 2
// Stream limité terminé

```

### complete manuel

Avec les Subject ou les Observable personnalisés, vous pouvez appeler complete manuellement.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.subscribe({
  next: (value) => console.log('Valeur:', value),
  complete: () => console.log('Subject terminé'),
});

subject.next(1);
subject.next(2);
subject.complete(); // Complétion manuelle
subject.next(3); // Ignoré après complétion

// Sortie:
// Valeur: 1
// Valeur: 2
// Subject terminé
```

## Différence entre finalize et complete

Comprenons les différences importantes.

1. **Timing d'exécution**
   - `complete`: Appelé uniquement lorsque l'Observable **se termine normalement**
   - `finalize`: Appelé lorsque l'Observable **se termine par complétion, erreur ou désabonnement**

2. **Usage**
   - `complete`: Recevoir une notification de terminaison normale (traitement en cas de succès)
   - `finalize`: Effectuer la libération des ressources et le nettoyage de manière fiable (traitement à exécuter dans tous les cas, succès ou échec)

## Cas d'utilisation pratiques

### Appel API et gestion de l'état de chargement

```ts
import { ajax } from 'rxjs/ajax';
import { finalize, catchError } from 'rxjs';
import { of } from 'rxjs';

// État de chargement
let isLoading = false;

function fetchData(id: string) {
  // Début du chargement
  isLoading = true;
  const loading = document.createElement('p');
  loading.style.display = 'block';
  document.body.appendChild(loading);
  // document.getElementById('loading')!.style.display = 'block';

  // Requête API
  return ajax.getJSON(`https://jsonplaceholder.typicode.com/posts/${id}`).pipe(
    catchError((error) => {
      console.error('Erreur API:', error);
      return of({ error: true, message: 'Échec de la récupération des données' });
    }),
    // Fin du chargement dans tous les cas (succès ou échec)
    finalize(() => {
      isLoading = false;
      loading!.style.display = 'none';
      console.log('Réinitialisation de l\'état de chargement terminée');
    })
  );
}

// Exemple d'utilisation
fetchData('123').subscribe({
  next: (data) => console.log('Données:', data),
  complete: () => console.log('Récupération des données terminée'),
});

// Sortie:
//  Erreur API: AjaxErrorImpl {message: 'ajax error', name: 'AjaxError', xhr: XMLHttpRequest, request: {…}, status: 0, …}
//  Données: {error: true, message: 'Échec de la récupération des données'}
//  Récupération des données terminée
//  Réinitialisation de l'état de chargement terminée
//   GET https://jsonplaceholder.typicode.com/posts/123 net::ERR_NAME_NOT_RESOLVED
```

### Nettoyage des ressources

```ts
import { interval } from 'rxjs';
import { finalize, takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

class ResourceManager {
  private destroy$ = new Subject<void>();
  private timerId: number | null = null;

  constructor() {
    // Initialisation d'une ressource
    this.timerId = window.setTimeout(() => console.log('Timer exécuté'), 10000);

    // Traitement périodique
    interval(1000)
      .pipe(
        // Arrêt lors de la destruction du composant
        takeUntil(this.destroy$),
        // Libération des ressources garantie
        finalize(() => {
          console.log('Interval arrêté');
        })
      )
      .subscribe((count) => {
        console.log('En cours d\'exécution...', count);
      });
  }

  dispose() {
    // Traitement de destruction
    if (this.timerId) {
      window.clearTimeout(this.timerId);
      this.timerId = null;
    }

    // Signal d'arrêt du stream
    this.destroy$.next();
    this.destroy$.complete();

    console.log('ResourceManager détruit');
  }
}

// Exemple d'utilisation
const manager = new ResourceManager();

// Destruction après 5 secondes
setTimeout(() => {
  manager.dispose();
}, 5000);

// Sortie:
// En cours d'exécution... 0
// En cours d'exécution... 1
// En cours d'exécution... 2
// En cours d'exécution... 3
// En cours d'exécution... 4
// Interval arrêté
// ResourceManager détruit
```

[📘 RxJS officiel: takeUntil()](https://rxjs.dev/api/index/function/takeUntil)

## Bonnes pratiques

1. **Toujours libérer les ressources**: Utilisez `finalize` pour garantir le nettoyage lors de la terminaison du stream
2. **Gestion de l'état de chargement**: Utilisez `finalize` pour toujours réinitialiser l'état de chargement
3. **Gestion du cycle de vie des composants**: Combinez `takeUntil` et `finalize` pour nettoyer les ressources lors de la destruction du composant (ce pattern est particulièrement recommandé avec Angular)
4. **Combinaison avec la gestion des erreurs**: Combinez `catchError` et `finalize` pour réaliser le traitement de fallback après erreur et un nettoyage garanti
5. **Connaissance de l'état de complétion**: Utilisez le callback `complete` pour déterminer si le stream s'est terminé normalement

## Résumé

`finalize` et `complete` sont des outils importants pour la gestion des ressources et le traitement de complétion dans RxJS. `finalize` est optimal pour la libération des ressources car il s'exécute de manière fiable quelle que soit la façon dont le stream se termine. D'autre part, `complete` est utilisé quand vous voulez effectuer un traitement lors de la terminaison normale. En les combinant correctement, vous pouvez prévenir les fuites mémoire et construire des applications fiables.
