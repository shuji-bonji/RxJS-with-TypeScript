---
description: "using() est une fonction de création RxJS qui crée et libère automatiquement les ressources selon le cycle de vie Observable. Elle gère en toute sécurité les ressources qui nécessitent un nettoyage manuel, telles que les WebSockets, les handles de fichiers et les timers, et empêche les fuites de mémoire."
---

# using()

[📘 Documentation officielle RxJS - using](https://rxjs.dev/api/index/function/using)

`using()` est une fonction de création qui crée et libère automatiquement les ressources selon le cycle de vie de l'Observable, en gérant en toute sécurité les ressources qui doivent être nettoyées manuellement, comme les WebSockets, les handles de fichiers et les timers, et en évitant les fuites de mémoire.

## Utilisation de base

### Gestion simple des ressources

```typescript
import { using, interval, Subscription, take } from 'rxjs';

const resource$ = using(
  // Fabrique de ressources : exécutée au début de l'abonnement
  () => {
    console.log('Ressource créée');
    return new Subscription(() => console.log('Ressource libérée'));
  },
  // Observable factory : création d'un Observable à partir d'une ressource
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Valeur:', value),
  complete: () => console.log('Terminé')
});

// Sortie:
// Ressource créée
// Valeur: 0
// Valeur: 1
// Valeur: 2
// Terminé
// Ressource libérée
```

> [!IMPORTANT]
> **Libération automatique des ressources**
>
> `using()` libère automatiquement les ressources lorsque l'Observable se termine (`complete`) ou est désabonné (`unsubscribe`).

## Comment fonctionne using()

`using()` prend les deux fonctions suivantes.

```typescript
function using<T>(
  resourceFactory: () => Unsubscribable | void,
  observableFactory: (resource: Unsubscribable | void) => ObservableInput<T>
): Observable<T>
```

### 1. resourceFactory

S'exécute au début d'un abonnement pour créer une ressource. Doit retourner un objet avec une méthode `unsubscribe()`.

```typescript
// Retourne un Subscription
() => new Subscription(() => {
  console.log('Traitement de nettoyage');
});

// Ou retourner un objet avec une méthode unsubscribe
() => ({
  unsubscribe: () => {
    console.log('Traitement de nettoyage');
  }
});
```

### 2. observableFactory

Crée un Observable avec une ressource.

```typescript
(resource) => interval(1000);
```

## Modèles pratiques

### Gestion des connexions WebSocket

```typescript
import { using, interval, Subject, map, takeUntil } from 'rxjs';

function createWebSocketStream(url: string) {
  return using(
    // Création d'une connexion WebSocket
    () => {
      const ws = new WebSocket(url);
      console.log('Connexion WebSocket démarrée:', url);

      ws.onopen = () => console.log('Connexion terminée');
      ws.onerror = (error) => console.error('Erreur de connexion:', error);

      return {
        unsubscribe: () => {
          console.log('Connexion WebSocket fermée');
          ws.close();
        }
      };
    },
    // Création d'un flux de messages
    () => {
      const messages$ = new Subject<MessageEvent>();
      const ws = new WebSocket(url);

      ws.onmessage = (event) => messages$.next(event);
      ws.onerror = (error) => messages$.error(error);
      ws.onclose = () => messages$.complete();

      return messages$;
    }
  );
}

// Exemple d'utilisation
const websocket$ = createWebSocketStream('wss://echo.websocket.org');

const subscription = websocket$.subscribe({
  next: message => console.log('Reçu:', message.data),
  error: error => console.error('Erreur:', error),
  complete: () => console.log('Terminé')
});

// Fermeture automatique de la WebSocket après 10 secondes
setTimeout(() => subscription.unsubscribe(), 10000);
```

### Nettoyage automatique des timers

```typescript
import { using, Observable, Subscription } from 'rxjs';

function createTimerStream(intervalMs: number) {
  return using(
    // Création d'une ressource timer
    () => {
      let timerId: number | null = null;
      console.log('Timer démarré');

      return new Subscription(() => {
        if (timerId !== null) {
          clearInterval(timerId);
          console.log('Timer arrêté');
        }
      });
    },
    // Création d'un flux de timer
    () => new Observable(subscriber => {
      const timerId = setInterval(() => {
        subscriber.next(Date.now());
      }, intervalMs);

      return () => clearInterval(timerId);
    })
  );
}

// Exemple d'utilisation
const timer$ = createTimerStream(1000);

const subscription = timer$.subscribe({
  next: time => console.log('Heure actuelle:', new Date(time).toLocaleTimeString())
});

// Arrêt après 5 secondes
setTimeout(() => subscription.unsubscribe(), 5000);
```

### Manipulation de fichiers (Node.js)

```typescript
import { using, Observable } from 'rxjs';
import * as fs from 'fs';

function readFileStream(filePath: string) {
  return using(
    // Ouvrir le handle de fichier
    () => {
      const fd = fs.openSync(filePath, 'r');
      console.log('Fichier ouvert:', filePath);

      return {
        unsubscribe: () => {
          fs.closeSync(fd);
          console.log('Fichier fermé');
        }
      };
    },
    // Création d'un flux de lecture de fichier
    () => new Observable<string>(subscriber => {
      const stream = fs.createReadStream(filePath, { encoding: 'utf8' });

      stream.on('data', (chunk) => subscriber.next(chunk));
      stream.on('error', (error) => subscriber.error(error));
      stream.on('end', () => subscriber.complete());

      return () => stream.destroy();
    })
  );
}

// Exemple d'utilisation
const file$ = readFileStream('./data.txt');

file$.subscribe({
  next: chunk => console.log('Lecture:', chunk),
  error: error => console.error('Erreur:', error),
  complete: () => console.log('Lecture terminée')
});
```

### Gestion des écouteurs d'événements

```typescript
import { using, Observable } from 'rxjs';

function createClickStream(element: HTMLElement) {
  return using(
    // Enregistrement de l'écouteur d'événement
    () => {
      console.log('Écouteur d\'événement enregistré');

      return {
        unsubscribe: () => {
          console.log('Écouteur d\'événement supprimé');
          // La suppression effective est effectuée dans la fabrique d'Observable
        }
      };
    },
    // Création d'un flux d'événements de clic
    () => new Observable<MouseEvent>(subscriber => {
      const handler = (event: MouseEvent) => subscriber.next(event);

      element.addEventListener('click', handler);

      return () => {
        element.removeEventListener('click', handler);
      };
    })
  );
}

// Exemple d'utilisation
const button = document.querySelector('#myButton') as HTMLElement;
const clicks$ = createClickStream(button);

const subscription = clicks$.subscribe({
  next: event => console.log('Position du clic:', event.clientX, event.clientY)
});

// Suppression automatique après 30 secondes
setTimeout(() => subscription.unsubscribe(), 30000);
```

## Cas d'utilisation courants

### 1. Gestion des connexions aux bases de données

```typescript
import { using, from, mergeMap } from 'rxjs';

interface DbConnection {
  query: (sql: string) => Promise<any[]>;
  close: () => Promise<void>;
}

function queryWithConnection(sql: string) {
  return using(
    // Établir la connexion à la base de données
    () => {
      const connection = createDbConnection();
      console.log('Connexion DB établie');

      return {
        unsubscribe: async () => {
          await connection.close();
          console.log('Connexion DB fermée');
        }
      };
    },
    // Exécution de la requête
    () => {
      const connection = createDbConnection();
      return from(connection.query(sql));
    }
  );
}

// Exemple d'utilisation
const users$ = queryWithConnection('SELECT * FROM users');

users$.subscribe({
  next: rows => console.log('Récupéré:', rows),
  error: error => console.error('Erreur:', error),
  complete: () => console.log('Requête terminée')
});

function createDbConnection(): DbConnection {
  // Traitement de la connexion proprement dit
  return {
    query: async (sql) => [],
    close: async () => {}
  };
}
```

### 2. Gestion du pool de ressources

```typescript
import { using, Observable, defer } from 'rxjs';

class ResourcePool<T> {
  private available: T[] = [];
  private inUse = new Set<T>();

  constructor(private factory: () => T, size: number) {
    for (let i = 0; i < size; i++) {
      this.available.push(factory());
    }
  }

  acquire(): T | null {
    const resource = this.available.pop();
    if (resource) {
      this.inUse.add(resource);
      return resource;
    }
    return null;
  }

  release(resource: T): void {
    if (this.inUse.has(resource)) {
      this.inUse.delete(resource);
      this.available.push(resource);
    }
  }
}

// Exemple d'utilisation
const pool = new ResourcePool(() => ({ id: Math.random() }), 5);

function usePooledResource<T>(
  pool: ResourcePool<T>,
  work: (resource: T) => Observable<any>
) {
  return using(
    () => {
      const resource = pool.acquire();
      if (!resource) {
        throw new Error('Pool de ressources épuisé');
      }
      console.log('Ressource acquise:', resource);

      return {
        unsubscribe: () => {
          pool.release(resource);
          console.log('Ressource retournée:', resource);
        }
      };
    },
    (subscription) => {
      const resource = pool.acquire();
      return resource ? work(resource) : defer(() => {
        throw new Error('L\'acquisition de la ressource a échoué');
      });
    }
  );
}

// Traitement utilisant une ressource
const work$ = usePooledResource(pool, (resource) =>
  new Observable(subscriber => {
    subscriber.next(`Traitement: ${resource.id}`);
    setTimeout(() => subscriber.complete(), 1000);
  })
);

work$.subscribe({
  next: result => console.log(result),
  complete: () => console.log('Traitement terminé')
});
```

### 3. Gestion de la coordination de ressources multiples

```typescript
import { using, merge, Subject } from 'rxjs';

interface MultiResource {
  ws: WebSocket;
  timer: number;
}

function createMultiResourceStream() {
  return using(
    // Créer des ressources multiples
    () => {
      const ws = new WebSocket('wss://echo.websocket.org');
      const timer = setInterval(() => {
        console.log('Exécution périodique');
      }, 1000);

      console.log('Ressources multiples créées');

      return {
        unsubscribe: () => {
          ws.close();
          clearInterval(timer);
          console.log('Ressources multiples libérées');
        }
      };
    },
    // Combiner plusieurs flux
    () => {
      const messages$ = new Subject<string>();
      const ticks$ = new Subject<number>();

      return merge(messages$, ticks$);
    }
  );
}

// Exemple d'utilisation
const multiStream$ = createMultiResourceStream();

const subscription = multiStream$.subscribe({
  next: value => console.log('Reçu:', value)
});

// Libère toutes les ressources après 10 secondes
setTimeout(() => subscription.unsubscribe(), 10000);
```

### 4. Gestion conditionnelle des ressources

```typescript
import { using, interval, EMPTY, take } from 'rxjs';

function conditionalResource(shouldCreate: boolean) {
  return using(
    () => {
      if (shouldCreate) {
        console.log('Ressource créée');
        return {
          unsubscribe: () => console.log('Ressource libérée')
        };
      } else {
        console.log('Création de la ressource ignorée');
        return { unsubscribe: () => {} };
      }
    },
    () => {
      if (shouldCreate) {
        return interval(1000).pipe(take(3));
      } else {
        return EMPTY;
      }
    }
  );
}

// Lors de la création de ressources
conditionalResource(true).subscribe({
  next: val => console.log('Valeur:', val),
  complete: () => console.log('Terminé')
});

// Sans création de ressources
conditionalResource(false).subscribe({
  next: val => console.log('Valeur:', val),
  complete: () => console.log('Terminé')
});
```

## Traitement des erreurs

### Libération des ressources en cas d'erreur

```typescript
import { using, throwError, of, catchError } from 'rxjs';

const errorHandling$ = using(
  () => {
    console.log('Ressource créée');
    return {
      unsubscribe: () => console.log('Ressource libérée (exécutée même en cas d\'erreur)')
    };
  },
  () => throwError(() => new Error('Erreur intentionnelle'))
);

errorHandling$.pipe(
  catchError(error => {
    console.error('Erreur capturée:', error.message);
    return of('Valeur par défaut');
  })
).subscribe({
  next: val => console.log('Valeur:', val),
  complete: () => console.log('Terminé')
});

// Sortie:
// Ressource créée
// Ressource libérée (exécutée même en cas d'erreur)
// Erreur capturée: Erreur intentionnelle
// Valeur: Valeur par défaut
// Terminé
```

> [!IMPORTANT]
> **Libération fiable des ressources, même en cas d'erreur**
>
> `using()` libère toujours la ressource créée dans `resourceFactory`, même en cas d'erreur.

## Erreurs courantes et comment les traiter

### 1. Oubli d'implémenter la méthode unsubscribe

**Exemple d'erreur:**
```typescript
// ❌ Erreur : pas de méthode unsubscribe
using(
  () => {
    console.log('Ressource créée');
    return {}; // pas de unsubscribe
  },
  () => interval(1000)
);
```

**Solution:**
```typescript
// ✅ Correct : implémenter la méthode unsubscribe
using(
  () => {
    console.log('Ressource créée');
    return {
      unsubscribe: () => console.log('Ressource libérée')
    };
  },
  () => interval(1000)
);
```

### 2. Création de ressources asynchrones

**Problème:**
```typescript
// ❌ Problème : resourceFactory ne peut pas être asynchrone
using(
  async () => { // async ne peut pas être utilisé
    const resource = await createResourceAsync();
    return resource;
  },
  () => interval(1000)
);
```

**Solution:**
```typescript
import { defer, from, mergeMap } from 'rxjs';

// ✅ Correct : gérer le traitement asynchrone avec defer et mergeMap
defer(() =>
  from(createResourceAsync()).pipe(
    mergeMap(resource =>
      using(
        () => resource,
        () => interval(1000)
      )
    )
  )
);
```

### 3. Création de ressources en double

**Problème:**
```typescript
// ❌ Problème : créer des ressources séparément dans resourceFactory et observableFactory
let sharedResource: any;

using(
  () => {
    sharedResource = createResource(); // Créer ici
    return { unsubscribe: () => sharedResource.close() };
  },
  () => {
    const resource = createResource(); // Créer à nouveau
    return from(resource.getData());
  }
);
```

**Solution:**
```typescript
// ✅ Correct : partage des ressources
using(
  () => {
    const resource = createResource();
    return {
      resource, // Maintien de la ressource
      unsubscribe: () => resource.close()
    };
  },
  (subscription: any) => {
    return from(subscription.resource.getData());
  }
);
```

## Meilleures pratiques pour using()

### 1. Assurer la libération des ressources

```typescript
// ✅ Bon exemple : le pattern try-finally
using(
  () => {
    const resource = createResource();
    return {
      unsubscribe: () => {
        try {
          resource.close();
        } catch (error) {
          console.error('Erreur de libération de ressource:', error);
        }
      }
    };
  },
  () => interval(1000)
);
```

### 2. Journalisation de la création de ressources

```typescript
// ✅ Bon exemple : enregistrer le cycle de vie d'une ressource
using(
  () => {
    const resourceId = Math.random();
    console.log(`[${resourceId}] Ressource créée`);

    return {
      unsubscribe: () => {
        console.log(`[${resourceId}] Ressource libérée`);
      }
    };
  },
  () => interval(1000)
);
```

### 3. Gestion des ressources type-safe

```typescript
// ✅ Bon exemple : utiliser les types TypeScript
interface ManagedResource {
  id: string;
  close: () => void;
}

function createManagedStream(resource: ManagedResource) {
  return using(
    () => {
      console.log('Ressource démarrée:', resource.id);
      return {
        unsubscribe: () => {
          resource.close();
          console.log('Ressource terminée:', resource.id);
        }
      };
    },
    () => interval(1000)
  );
}
```

## Comparaison avec la gestion manuelle

### Gestion manuelle des ressources (❌ non recommandé)

```typescript
// ❌ Mauvais exemple : gestion manuelle (risque d'oublier de libérer)
const ws = new WebSocket('wss://example.com');
const subscription = interval(1000).subscribe(() => {
  ws.send('ping');
});

// Peut oublier de libérer
// subscription.unsubscribe();
// ws.close();
```

### Gestion des ressources par using() (✅ recommandé)

```typescript
// ✅ Bon exemple : gestion automatique avec using()
const stream$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return {
      unsubscribe: () => ws.close()
    };
  },
  () => interval(1000)
);

const subscription = stream$.subscribe(() => {
  // Traitement utilisant WebSocket
});

// Le WebSocket est également fermé automatiquement avec unsubscribe()
subscription.unsubscribe();
```

## Résumé

`using()` est une fonction de création qui gère automatiquement les ressources en fonction du cycle de vie de l'Observable.

**Caractéristiques principales:**
- Création d'une ressource au début d'un abonnement
- Libération automatique à la fin de l'abonnement (complete ou unsubscribe)
- Prévient les fuites de mémoire
- Libération fiable de la ressource, même en cas d'erreur

**Cas d'utilisation:**
- Connexions réseau telles que WebSocket, EventSource
- Handles de fichiers, connexions aux bases de données
- Nettoyage automatique des timers et des intervalles
- Désactivation automatique des écouteurs d'événements

**Notes:**
- `resourceFactory` doit être une fonction synchrone
- Toujours implémenter la méthode `unsubscribe`
- Assurer une bonne gestion des erreurs

**Utilisation recommandée:**
- Éviter d'oublier de libérer des ressources
- Journaliser le cycle de vie
- Utiliser les types TypeScript pour une gestion type-safe

## Pages connexes

- [scheduled()](/fr/guide/creation-functions/control/scheduled) - Génère un Observable avec le scheduler spécifié
- [Fonctions de création de contrôle](/fr/guide/creation-functions/control/) - Comparaison entre scheduled() et using()
- [finalize()](/fr/guide/error-handling/finalize) - Opérateur pour ajouter un traitement à la fin de l'abonnement

## Références

- [Documentation officielle RxJS - using](https://rxjs.dev/api/index/function/using)
- [Documentation officielle RxJS - Subscription](https://rxjs.dev/guide/subscription)
