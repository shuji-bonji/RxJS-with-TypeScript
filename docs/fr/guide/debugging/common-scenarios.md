---
description: "Présentation de 6 scénarios de débogage RxJS courants : absence de flux de valeurs, sortie de valeurs différentes de celles attendues, abonnement ne se terminant pas, fuites mémoire, erreurs manquées, suivi des retries. Explication des causes et solutions avec des exemples de code pratiques pour les problèmes fréquents sur le terrain."
---

# Scénarios de débogage courants

Explication des problèmes typiques rencontrés lors du développement RxJS et de leurs solutions avec des exemples de code concrets.

## Scénario 1 : Absence de flux de valeurs

- **Symptôme** : Aucune valeur n'est émise malgré l'abonnement avec `subscribe`


### Cause 1 : Oubli d'abonnement à un Cold Observable

```ts
import { interval } from 'rxjs';
import { map } from 'rxjs';

// ❌ Rien n'est exécuté car il n'y a pas d'abonnement
const numbers$ = interval(1000).pipe(
  map(x => {
    console.log('Cette ligne ne sera pas exécutée');
    return x * 2;
  })
);

// ✅ S'exécute avec un abonnement
numbers$.subscribe(value => console.log('Valeur:', value));
```

### Cause 2 : Subject déjà complété

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.complete(); // Complétion

// ❌ Un abonnement après la complétion ne recevra pas de valeurs
subject.subscribe(value => console.log('Cette ligne ne sera pas exécutée'));

// ✅ S'abonner avant la complétion
const subject2 = new Subject<number>();
subject2.subscribe(value => console.log('Valeur:', value));
subject2.next(1); // Valeur: 1
subject2.complete();
```

### Cause 3 : Filtrage avec une condition incorrecte

```ts
import { of } from 'rxjs';
import { filter, tap } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('Avant filter:', value)),
    filter(x => x > 10), // Tout est exclu
    tap(value => console.log('Après filter:', value)) // Cette ligne ne sera pas exécutée
  )
  .subscribe({
    next: value => console.log('Valeur finale:', value),
    complete: () => console.log('Complété (aucune valeur)')
  });

// Sortie:
// Avant filter: 1
// Avant filter: 2
// Avant filter: 3
// Avant filter: 4
// Avant filter: 5
// Complété (aucune valeur)
```

### Technique de débogage
```ts
import { of, EMPTY } from 'rxjs';
import { filter, tap, defaultIfEmpty } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('🔵 Entrée:', value)),
    filter(x => x > 10),
    tap(value => console.log('🟢 Passé filter:', value)),
    defaultIfEmpty('Aucune valeur') // Valeur par défaut si aucune valeur
  )
  .subscribe(value => console.log('✅ Sortie:', value));

// Sortie:
// 🔵 Entrée: 1
// 🔵 Entrée: 2
// 🔵 Entrée: 3
// 🔵 Entrée: 4
// 🔵 Entrée: 5
// ✅ Sortie: Aucune valeur
```

## Scénario 2 : Sortie de valeurs différentes de celles attendues

- **Symptôme** : Les valeurs émises sont différentes de celles attendues

### Cause 1 : Ordre incorrect des opérateurs

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// ❌ Résultat différent de celui attendu
of(1, 2, 3, 4, 5)
  .pipe(
    map(x => x * 2),     // 2, 4, 6, 8, 10
    filter(x => x < 5)   // Seuls 2, 4 passent
  )
  .subscribe(value => console.log('Résultat:', value));
// Sortie: 2, 4

// ✅ Ordre correct
of(1, 2, 3, 4, 5)
  .pipe(
    filter(x => x < 5),  // Seuls 1, 2, 3, 4 passent
    map(x => x * 2)      // 2, 4, 6, 8
  )
  .subscribe(value => console.log('Résultat:', value));
// Sortie: 2, 4, 6, 8
```

### Cause 2 : Modification involontaire due au partage de référence

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const user: User = { id: 1, name: 'Alice' };

of(user)
  .pipe(
    // ❌ Modification directe de l'objet original
    map(u => {
      u.name = 'Bob'; // L'objet original est modifié
      return u;
    })
  )
  .subscribe(value => console.log('Après modification:', value));

console.log('Objet original:', user); // { id: 1, name: 'Bob' }

// ✅ Créer un nouvel objet
of(user)
  .pipe(
    map(u => ({ ...u, name: 'Charlie' })) // Nouvel objet avec la syntaxe spread
  )
  .subscribe(value => console.log('Après modification:', value));

console.log('Objet original:', user); // { id: 1, name: 'Alice' } (non modifié)
```

### Cause 3 : Timing du traitement asynchrone

```ts
import { of, delay } from 'rxjs';
import { mergeMap, tap } from 'rxjs';

// ❌ N'attend pas la fin du traitement asynchrone
of(1, 2, 3)
  .pipe(
    tap(value => console.log('Début:', value)),
    mergeMap(value =>
      of(value * 2).pipe(
        delay(100 - value * 10) // Plus la valeur est grande, plus c'est rapide
      )
    )
  )
  .subscribe(value => console.log('Terminé:', value));

// Sortie:
// Début: 1
// Début: 2
// Début: 3
// Terminé: 3  ← Délai le plus court
// Terminé: 2
// Terminé: 1  ← Délai le plus long

// ✅ Garantir l'ordre
import { concatMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(value => console.log('Début:', value)),
    concatMap(value =>  // mergeMap → concatMap
      of(value * 2).pipe(delay(100 - value * 10))
    )
  )
  .subscribe(value => console.log('Terminé:', value));

// Sortie:
// Début: 1
// Terminé: 1
// Début: 2
// Terminé: 2
// Début: 3
// Terminé: 3
```

## Scénario 3 : L'abonnement ne se termine pas (stream infini)

- **Symptôme** : `complete` n'est pas appelé et le stream ne se termine pas

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

// ❌ interval émet des valeurs indéfiniment
interval(1000)
  .pipe(
    tap(value => console.log('Valeur:', value))
  )
  .subscribe({
    complete: () => console.log('Cette ligne ne sera pas exécutée')
  });

// ✅ Terminer explicitement avec take
import { take } from 'rxjs';

interval(1000)
  .pipe(
    take(5), // Se termine après 5 valeurs
    tap(value => console.log('Valeur:', value))
  )
  .subscribe({
    complete: () => console.log('Complété')
  });
```

### Technique de débogage
```ts
import { interval, timer } from 'rxjs';
import { tap, takeUntil } from 'rxjs';

// Définir un timeout pour le débogage
const stop$ = timer(5000); // Se termine après 5 secondes

interval(1000)
  .pipe(
    tap({
      next: value => console.log('Valeur:', value),
      complete: () => console.log('Arrêt sur timeout')
    }),
    takeUntil(stop$)
  )
  .subscribe();
```

## Scénario 4 : Fuite mémoire (oubli de désabonnement)

- **Symptôme** : Les performances de l'application se dégradent progressivement

### Cause : Abonnements non désabonnés lorsqu'ils ne sont plus nécessaires

```ts
import { interval } from 'rxjs';

class UserComponent {
  private subscription: any;

  ngOnInit() {
    // ❌ Oubli de désabonnement
    interval(1000).subscribe(value => {
      console.log('Valeur:', value); // Continue de s'exécuter après la destruction du composant
    });
  }

  ngOnDestroy() {
    // Pas de désabonnement
  }
}

// ✅ Gérer correctement l'abonnement
class UserComponentFixed {
  private subscription: any;

  ngOnInit() {
    this.subscription = interval(1000).subscribe(value => {
      console.log('Valeur:', value);
    });
  }

  ngOnDestroy() {
    // Désabonnement lors de la destruction du composant
    if (this.subscription) {
      this.subscription.unsubscribe();
    }
  }
}
```

**Pattern recommandé : Utiliser `takeUntil`**

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class UserComponentBest {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    // ✅ Désabonnement automatique avec takeUntil
    interval(1000)
      .pipe(
        takeUntil(this.destroy$)
      )
      .subscribe(value => console.log('Valeur:', value));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### Détection des fuites mémoire

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

let subscriptionCount = 0;

const trackSubscriptions = <T>() =>
  tap<T>({
    subscribe: () => {
      subscriptionCount++;
      console.log('📈 Nombre d\'abonnements:', subscriptionCount);
    },
    unsubscribe: () => {
      subscriptionCount--;
      console.log('📉 Nombre d\'abonnements:', subscriptionCount);
    }
  });

// Exemple d'utilisation
const stream$ = interval(1000).pipe(
  trackSubscriptions()
);

const sub1 = stream$.subscribe();
// Sortie: 📈 Nombre d'abonnements: 1

const sub2 = stream$.subscribe();
// Sortie: 📈 Nombre d'abonnements: 2

setTimeout(() => {
  sub1.unsubscribe();
  // Sortie: 📉 Nombre d'abonnements: 1
}, 3000);
```

## Scénario 5 : Erreurs non détectées

- **Symptôme** : Des erreurs se produisent mais ne sont pas affichées et sont ignorées

```ts
import { of, throwError } from 'rxjs';
import { mergeMap, catchError } from 'rxjs';

// ❌ Pas de gestion d'erreur, donc les erreurs sont étouffées
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Erreur'));
      }
      return of(value);
    })
  )
  .subscribe(); // Pas de gestionnaire d'erreur

// ✅ Gestion appropriée des erreurs
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Erreur'));
      }
      return of(value);
    }),
    catchError((error: unknown) => {
      console.error('🔴 Erreur capturée:', (error instanceof Error ? error.message : String(error)));
      return of(-1); // Valeur de repli
    })
  )
  .subscribe({
    next: value => console.log('Valeur:', value),
    error: error => console.error('🔴 Erreur lors de l\'abonnement:', error)
  });

// Sortie:
// Valeur: 1
// 🔴 Erreur capturée: Erreur
// Valeur: -1
```

### Configuration d'un gestionnaire d'erreur global

```ts
import { Observable } from 'rxjs';

// Capturer toutes les erreurs non gérées
const originalCreate = Observable.create;

Observable.create = function(subscribe: any) {
  return originalCreate.call(this, (observer: any) => {
    try {
      return subscribe(observer);
    } catch (error) {
      console.error('🔴 Erreur non gérée:', error);
      observer.error(error);
    }
  });
};
```

## Scénario 6 : Suivi du nombre de tentatives de retry

- **Symptôme** : Utilisation de l'opérateur `retry` sans savoir combien de fois il réessaie

Lors du retry automatique en cas d'erreur, le suivi du nombre réel de tentatives facilite le débogage et l'enregistrement des logs.

### Débogage de retry de base

```ts
import { throwError, of, timer } from 'rxjs';
import { retry } from 'rxjs';

throwError(() => new Error('Erreur temporaire'))
  .pipe(
    // Recommandé en RxJS 7.3+ : forme retry({ count, delay })
    retry({
      count: 2, // Réessayer au maximum 2 fois
      delay: (error, retryCount) => {
        console.log(`🔄 Tentative ${retryCount}`);
        return timer(1000);
      }
    })
  )
  .subscribe({
    next: value => console.log('✅ Succès:', value),
    error: error => {
      console.log('❌ Nombre maximum de tentatives atteint');
      console.log('🔴 Erreur finale:', error.message);
    }
  });

// Sortie:
// 🔄 Tentative 1
// 🔄 Tentative 2
// ❌ Nombre maximum de tentatives atteint
// 🔴 Erreur finale: Erreur temporaire
```

> [!TIP]
> Pour des patterns d'implémentation plus détaillés sur le débogage des retries, consultez la section "Débogage des retries" dans [retry et catchError](/fr/guide/error-handling/retry-catch).
> - Suivi de base avec le callback error de tap
> - Enregistrement détaillé des logs avec retryWhen
> - Backoff exponentiel et enregistrement des logs
> - Objet de configuration retry de RxJS 7.4+

## Résumé

Solutions aux scénarios de débogage courants

- ✅ **Absence de flux de valeurs** → Vérifier l'oubli d'abonnement, les conditions de filtrage
- ✅ **Valeurs différentes de celles attendues** → Attention à l'ordre des opérateurs, au partage de références
- ✅ **L'abonnement ne se termine pas** → Utiliser `take` ou `takeUntil` pour les streams infinis
- ✅ **Fuite mémoire** → Désabonnement automatique avec le pattern `takeUntil`
- ✅ **Erreurs manquées** → Implémenter une gestion appropriée des erreurs
- ✅ **Suivi des retries** → Enregistrement des logs avec `retryWhen` ou objet de configuration

## Pages connexes

- [Stratégies de débogage de base](/fr/guide/debugging/) - Utilisation de l'opérateur tap et des outils de développement
- [Outils de débogage personnalisés](/fr/guide/debugging/custom-tools) - Streams nommés, opérateurs de débogage
- [Débogage des performances](/fr/guide/debugging/performance) - Surveillance des abonnements, vérification de l'utilisation mémoire
- [Gestion des erreurs](/fr/guide/error-handling/strategies) - Stratégies de traitement des erreurs
