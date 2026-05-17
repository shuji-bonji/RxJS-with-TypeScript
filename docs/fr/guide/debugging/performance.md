---
description: "Techniques de débogage des performances pour les applications RxJS : suivi du nombre d'abonnements, détection des réévaluations inutiles, surveillance de l'utilisation mémoire, configuration de débogage en environnement de développement, création d'opérateurs de mesure des performances type-safe. Techniques pratiques pour éviter les problèmes en production."
---

# Débogage des performances et meilleures pratiques

Explication des techniques pour optimiser les performances des applications RxJS et construire un environnement de débogage efficace.

## Vérification du nombre d'abonnements

Vérifiez que plusieurs abonnements ne sont pas créés involontairement.

```ts
import { Observable, defer } from 'rxjs';
import { finalize } from 'rxjs';

let globalSubscriptionId = 0;
let activeSubscriptions = 0;

/**
 * Opérateur personnalisé pour suivre le nombre d'abonnements
 */
function tracked<T>(label: string) {
  return (source: Observable<T>) =>
    defer(() => {
      const id = ++globalSubscriptionId;
      activeSubscriptions++;
      console.log(`➕ Abonnement démarré [${label}] #${id} (actifs: ${activeSubscriptions})`);

      return source.pipe(
        finalize(() => {
          activeSubscriptions--;
          console.log(`➖ Abonnement terminé [${label}] #${id} (actifs: ${activeSubscriptions})`);
        })
      );
    });
}

// Exemple d'utilisation
import { interval } from 'rxjs';
import { take } from 'rxjs';

const stream$ = interval(1000).pipe(
  take(3),
  tracked('Test Stream')
);

const sub1 = stream$.subscribe();
const sub2 = stream$.subscribe();

setTimeout(() => {
  sub1.unsubscribe();
  sub2.unsubscribe();
}, 5000);

// Sortie:
// ➕ Abonnement démarré [Test Stream] #1 (actifs: 1)
// ➕ Abonnement démarré [Test Stream] #2 (actifs: 2)
// ➖ Abonnement terminé [Test Stream] #1 (actifs: 1)
// ➖ Abonnement terminé [Test Stream] #2 (actifs: 0)
```

Cette implémentation :
- ✅ Génère un nouvel ID à chaque abonnement avec `defer`
- ✅ Exécute systématiquement le traitement lors du désabonnement avec `finalize`
- ✅ Suit en temps réel le nombre d'abonnements actifs
- ✅ Type-safe et fonctionne avec RxJS v8

## Détection des réévaluations inutiles

Vérifiez que les mêmes valeurs ne sont pas calculées plusieurs fois.

```ts
import { of } from 'rxjs';
import { map, tap, shareReplay } from 'rxjs';

let computeCount = 0;

function expensiveComputation(value: number): number {
  computeCount++;
  console.log(`💰 Calcul exécuté (${computeCount}e fois):`, value);
  // Simulation de calcul lourd
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result += Math.sin(i);
  }
  return result;
}

// ❌ Sans shareReplay → Calcul à chaque abonnement
console.log('=== Sans shareReplay ===');
computeCount = 0;
const withoutShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x))
);

withoutShare$.subscribe(v => console.log('Abonnement 1:', v));
withoutShare$.subscribe(v => console.log('Abonnement 2:', v));
// Sortie: Le calcul est exécuté 6 fois (3 valeurs × 2 abonnements)

// ✅ Avec shareReplay → Les résultats de calcul sont partagés
console.log('\n=== Avec shareReplay ===');
computeCount = 0;
const withShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x)),
  shareReplay({ bufferSize: 3, refCount: true })
);

withShare$.subscribe(v => console.log('Abonnement 1:', v));
withShare$.subscribe(v => console.log('Abonnement 2:', v));
// Sortie: Le calcul est exécuté seulement 3 fois
```

## Surveillance de l'utilisation mémoire

Méthode de surveillance pour détecter les fuites mémoire.

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MemoryMonitor {
  private intervals: ReturnType<typeof setInterval>[] = [];

  start(intervalMs: number = 5000) {
    const id = setInterval(() => {
      if (typeof performance !== 'undefined' && (performance as any).memory) {
        const memory = (performance as any).memory;
        console.log('📊 Utilisation mémoire:', {
          utilisée: `${(memory.usedJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          totale: `${(memory.totalJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          limite: `${(memory.jsHeapSizeLimit / 1024 / 1024).toFixed(2)} MB`
        });
      }
    }, intervalMs);

    this.intervals.push(id);
  }

  stop() {
    this.intervals.forEach(id => clearInterval(id));
    this.intervals = [];
  }
}

// Exemple d'utilisation
const monitor = new MemoryMonitor();
monitor.start(5000); // Afficher l'utilisation mémoire toutes les 5 secondes

// Test de fuite mémoire
const leakyStreams: any[] = [];

for (let i = 0; i < 100; i++) {
  // ❌ Stream non désabonné
  const sub = interval(100).subscribe();
  leakyStreams.push(sub);
}

// Désabonnement après 10 secondes
setTimeout(() => {
  console.log('Début du désabonnement');
  leakyStreams.forEach(sub => sub.unsubscribe());
  console.log('Désabonnement terminé');

  // Arrêter la surveillance après 10 secondes supplémentaires
  setTimeout(() => {
    monitor.stop();
  }, 10000);
}, 10000);
```

## Meilleures pratiques

### Construction d'un environnement de débogage

Méthode pour activer les logs de débogage uniquement en environnement de développement.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Détection du mode débogage (ajuster selon l'outil de build)
const IS_DEVELOPMENT =
  // Avec Vite: import.meta.env.DEV
  // Avec webpack: process.env.NODE_ENV === 'development'
  // Configuration manuelle: définir une variable globale
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

function devLog<T>(label: string) {
  if (!IS_DEVELOPMENT) {
    return (source: Observable<T>) => source;
  }

  return tap<T>({
    next: value => console.log(`[${label}]`, value),
    error: error => console.error(`[${label}] Error:`, error),
    complete: () => console.log(`[${label}] Complete`)
  });
}

// Exemple d'utilisation
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    devLog('Input'),
    map(x => x * 2),
    devLog('Output')
  )
  .subscribe();
// Aucun log en environnement de production
```

### Débogage type-safe

Méthode de débogage utilisant le système de types TypeScript.

```ts
import { tap } from 'rxjs';

type LogLevel = 'debug' | 'info' | 'warn' | 'error';

interface TypedDebugOptions<T> {
  label: string;
  level?: LogLevel;
  transform?: (value: T) => any;
  filter?: (value: T) => boolean;
}

function typedDebug<T>(options: TypedDebugOptions<T>) {
  const { label, level = 'debug', transform, filter } = options;

  const logFn = console[level] || console.log;

  return tap<T>({
    next: value => {
      if (filter && !filter(value)) return;

      const displayValue = transform ? transform(value) : value;
      logFn(`[${label}]`, displayValue);
    }
  });
}

// Exemple d'utilisation
interface User {
  id: number;
  name: string;
  email: string;
}

import { of } from 'rxjs';

of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob', email: 'bob@example.com' },
  { id: 3, name: 'Charlie', email: 'charlie@example.com' }
)
  .pipe(
    typedDebug<User>({
      label: 'User Stream',
      level: 'info',
      transform: user => `${user.name} (${user.email})`,
      filter: user => user.id > 1
    })
  )
  .subscribe();

// Sortie:
// [User Stream] Bob (bob@example.com)
// [User Stream] Charlie (charlie@example.com)
```

### Configuration des limites d'erreur

Isolez correctement les erreurs pour faciliter le débogage.

```ts
import { Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

function errorBoundary<T>(label: string) {
  return (source: Observable<T>) =>
    source.pipe(
      catchError((error: unknown) => {
        console.error(`🔴 [${label}] Erreur capturée:`, {
          message: (error instanceof Error ? error.message : String(error)),
          stack: error.stack,
          timestamp: new Date().toISOString()
        });

        // Re-lancer l'erreur ou retourner une valeur de repli
        throw error;
      })
    );
}

// Exemple d'utilisation
import { throwError } from 'rxjs';
import { mergeMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    errorBoundary('Traitement principal'),
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Erreur avec valeur 2'));
      }
      return of(value);
    }),
    errorBoundary('Traitement asynchrone')
  )
  .subscribe({
    next: value => console.log('Succès:', value),
    error: error => console.log('Erreur finale:', error.message)
  });
```

## Résumé

Débogage des performances et meilleures pratiques

### Surveillance des performances
- ✅ **Suivi des abonnements** - Gestion des abonnements avec defer et finalize
- ✅ **Détection des réévaluations** - Éviter les calculs inutiles avec shareReplay
- ✅ **Surveillance mémoire** - Suivre l'utilisation mémoire avec l'API performance

### Optimisation de l'environnement de développement
- ✅ **Configuration par environnement** - Activer les logs de débogage uniquement en développement
- ✅ **Débogage type-safe** - Utiliser le système de types TypeScript
- ✅ **Limites d'erreur** - Isoler correctement les erreurs pour le débogage

En combinant ces techniques, vous pouvez optimiser les performances des applications RxJS et construire un environnement de débogage efficace.

## Pages connexes

- [Stratégies de débogage de base](/fr/guide/debugging/) - Utilisation de l'opérateur tap et des outils de développement
- [Scénarios de débogage courants](/fr/guide/debugging/common-scenarios) - Dépannage par problème
- [Outils de débogage personnalisés](/fr/guide/debugging/custom-tools) - Streams nommés, opérateurs de débogage
- [Opérateur - shareReplay](/fr/guide/operators/multicasting/shareReplay) - Éviter les réévaluations inutiles
