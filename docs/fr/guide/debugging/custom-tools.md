---
description: "Méthodes de création d'outils personnalisés pour le débogage RxJS : opérateur de suivi de streams nommés, opérateur de débogage configurable, opérateur de mesure des performances, opérateur de limite d'erreur. Introduction à l'implémentation d'outils de débogage pratiques de manière type-safe en TypeScript."
---

# Outils de débogage personnalisés

La création d'outils de débogage personnalisés permet un débogage flexible adapté aux besoins de votre projet.

## Débogage de streams nommés

Créez un opérateur personnalisé permettant de nommer et suivre les Observables.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Map pour gérer les noms de streams
const namedStreams = new Map<string, any[]>();

/**
 * Nommer et suivre un Observable
 */
function tagStream<T>(name: string) {
  if (!namedStreams.has(name)) {
    namedStreams.set(name, []);
  }

  return tap<T>({
    next: value => {
      const log = {
        name,
        type: 'next',
        value,
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.log(`[${name}] next:`, value);
    },
    error: error => {
      const log = {
        name,
        type: 'error',
        error,
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.error(`[${name}] error:`, error);
    },
    complete: () => {
      const log = {
        name,
        type: 'complete',
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.log(`[${name}] complete`);
    }
  });
}

/**
 * Obtenir les logs d'un stream nommé spécifique
 */
function getStreamLogs(name: string) {
  return namedStreams.get(name) || [];
}

/**
 * Obtenir la liste de tous les streams nommés
 */
function getAllStreamNames() {
  return Array.from(namedStreams.keys());
}

/**
 * Effacer les logs
 */
function clearStreamLogs(name?: string) {
  if (name) {
    namedStreams.set(name, []);
  } else {
    namedStreams.clear();
  }
}
```

### Exemple d'utilisation

```ts
import { interval } from 'rxjs';
import { map, take } from 'rxjs';

// Nommer un Observable
interval(1000)
  .pipe(
    tagStream('interval-stream'),
    map(x => x * 2),
    take(5)
  )
  .subscribe();

// Vérifier les logs après 3 secondes
setTimeout(() => {
  console.log('Tous les streams:', getAllStreamNames());
  console.log('Logs de interval-stream:', getStreamLogs('interval-stream'));
}, 3000);

// Sortie:
// [interval-stream] next: 0
// [interval-stream] next: 1
// [interval-stream] next: 2
// Tous les streams: ['interval-stream']
// Logs de interval-stream: [
//   { name: 'interval-stream', type: 'next', value: 0, timestamp: 1697280000000 },
//   { name: 'interval-stream', type: 'next', value: 1, timestamp: 1697280001000 },
//   { name: 'interval-stream', type: 'next', value: 2, timestamp: 1697280002000 }
// ]
```

### Suivi de plusieurs streams

```ts
import { interval, fromEvent } from 'rxjs';
import { map, take } from 'rxjs';

// Nommer plusieurs streams
interval(1000)
  .pipe(
    tagStream('timer-stream'),
    map(x => x * 2),
    take(3)
  )
  .subscribe();

const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tagStream('click-stream'),
      take(5)
    )
    .subscribe();
}

// Vérifier tous les streams
console.log('Streams suivis:', getAllStreamNames());
// Sortie: ['timer-stream', 'click-stream']
```

> [!NOTE]
> **À propos de rxjs-spy**
>
> `rxjs-spy` était une bibliothèque utile pour le débogage d'Observables, mais elle n'est plus maintenue et présente des problèmes de compatibilité avec les dernières versions de RxJS.
>
> Nous recommandons plutôt d'utiliser des opérateurs de débogage personnalisés comme ci-dessus. Ils sont plus flexibles et peuvent être personnalisés selon les besoins de votre projet.

## Création d'opérateurs de débogage personnalisés

La création de vos propres opérateurs de débogage permet un débogage plus flexible.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

interface DebugOptions {
  enabled?: boolean;
  label?: string;
  logValues?: boolean;
  logErrors?: boolean;
  logComplete?: boolean;
  logTimestamp?: boolean;
}

/**
 * Opérateur de débogage personnalisé
 */
function debug<T>(options: DebugOptions = {}) {
  const {
    enabled = true,
    label = 'Debug',
    logValues = true,
    logErrors = true,
    logComplete = true,
    logTimestamp = false
  } = options;

  if (!enabled) {
    return (source: Observable<T>) => source;
  }

  return tap<T>({
    next: value => {
      if (logValues) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.log(`${timestamp} [${label}] next:`, value);
      }
    },
    error: error => {
      if (logErrors) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.error(`${timestamp} [${label}] error:`, error);
      }
    },
    complete: () => {
      if (logComplete) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.log(`${timestamp} [${label}] complete`);
      }
    }
  });
}

// Exemple d'utilisation
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    debug({ label: 'Input', logTimestamp: true }),
    map(x => x * 2),
    debug({ label: 'After Map', logTimestamp: true })
  )
  .subscribe();

// Sortie:
// [2025-10-14T12:00:00.000Z] [Input] next: 1
// [2025-10-14T12:00:00.001Z] [After Map] next: 2
// [2025-10-14T12:00:00.001Z] [Input] next: 2
// [2025-10-14T12:00:00.002Z] [After Map] next: 4
// [2025-10-14T12:00:00.002Z] [Input] next: 3
// [2025-10-14T12:00:00.003Z] [After Map] next: 6
// [2025-10-14T12:00:00.003Z] [Input] complete
// [2025-10-14T12:00:00.004Z] [After Map] complete
```

## Opérateur de débogage pour la mesure des performances

```ts
import { tap } from 'rxjs';

function measure<T>(label: string) {
  let startTime: number;
  let count = 0;

  return tap<T>({
    subscribe: () => {
      startTime = performance.now();
      console.log(`⏱️ [${label}] Début`);
    },
    next: value => {
      count++;
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Valeur #${count} (${elapsed.toFixed(2)}ms):`, value);
    },
    complete: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Complété (total: ${elapsed.toFixed(2)}ms, ${count} valeurs)`);
    },
    error: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Erreur (${elapsed.toFixed(2)}ms)`);
    }
  });
}

// Exemple d'utilisation
import { interval } from 'rxjs';
import { take, delay } from 'rxjs';

interval(100)
  .pipe(
    take(5),
    measure('Interval Stream'),
    delay(50)
  )
  .subscribe();

// Sortie:
// ⏱️ [Interval Stream] Début
// ⏱️ [Interval Stream] Valeur #1 (150.23ms): 0
// ⏱️ [Interval Stream] Valeur #2 (250.45ms): 1
// ⏱️ [Interval Stream] Valeur #3 (350.67ms): 2
// ⏱️ [Interval Stream] Valeur #4 (450.89ms): 3
// ⏱️ [Interval Stream] Valeur #5 (551.12ms): 4
// ⏱️ [Interval Stream] Complété (total: 551.12ms, 5 valeurs)
```

## Résumé

Grâce à la création d'outils de débogage personnalisés

- ✅ **Streams nommés** - Identifier et suivre plusieurs streams par nom
- ✅ **Configuration flexible** - Opérateurs de débogage adaptés aux besoins du projet
- ✅ **Mesure des performances** - Enregistrement automatique du temps d'exécution et du nombre de valeurs
- ✅ **Gestion des logs** - Enregistrement et récupération de logs avec timestamps

Il est recommandé d'activer ces outils uniquement en environnement de développement et de les désactiver en production.

## Pages connexes

- [Stratégies de débogage de base](/fr/guide/debugging/) - Utilisation de l'opérateur tap et des outils de développement
- [Scénarios de débogage courants](/fr/guide/debugging/common-scenarios) - Dépannage par problème
- [Débogage des performances](/fr/guide/debugging/performance) - Surveillance des abonnements, vérification de l'utilisation mémoire
