---
description: using() is een RxJS Creation Function die automatisch resources aanmaakt en vrijgeeft volgens de Observable-lifecycle. Het beheert veilig resources die handmatige opruiming vereisen zoals WebSockets, bestandshandles en timers, en voorkomt geheugenlekken. Zorgt voor betrouwbaar resourcebeheer door resources aan te maken bij het begin van een subscription en automatisch vrij te geven aan het einde van een subscription.
---

# using()

[📘 RxJS Officiële Documentatie - using](https://rxjs.dev/api/index/function/using)

`using()` is een Creation Function die automatisch resources aanmaakt en dealloceert volgens de Observable-lifecycle, waarbij resources die handmatig moeten worden opgeruimd zoals WebSockets, bestandshandles en timers veilig worden beheerd en geheugenlekken worden voorkomen.

## Basisgebruik

### Eenvoudig resourcebeheer

```typescript
import { using, interval, Subscription, take } from 'rxjs';
const resource$ = using(
  // Resource factory: uitgevoerd bij het begin van de subscription
  () => {
    console.log('Resource aangemaakt');
    return new Subscription(() => console.log('Resource vrijgegeven'));
  },
  // Observable factory: maak Observable aan met resource
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Waarde:', value),
  complete: () => console.log('Voltooid')
});

// Output:
// Resource aangemaakt
// Waarde: 0
// Waarde: 1
// Waarde: 2
// Voltooid
// Resource vrijgegeven
```

> [!IMPORTANT]
> **Automatische resource-vrijgave**
>
> `using()` geeft automatisch resources vrij wanneer de Observable voltooid is (`complete`) of uitgeschreven (`unsubscribe`).

## Hoe using() werkt

`using()` neemt de volgende twee functies.

```typescript
function using<T>(
  resourceFactory: () => Unsubscribable | void,
  observableFactory: (resource: Unsubscribable | void) => ObservableInput<T>
): Observable<T>
```

### 1. resourceFactory

Wordt uitgevoerd bij het begin van een subscription om een resource aan te maken. Moet een object met een `unsubscribe()` methode retourneren.

```typescript
// Retourneer een Subscription
() => new Subscription(() => {
  console.log('Opruimverwerking');
});

// Of retourneer een object met een unsubscribe methode
() => ({
  unsubscribe: () => {
    console.log('Opruimverwerking');
  }
});
```

### 2. observableFactory

Maakt een Observable aan met een resource.

```typescript
(resource) => interval(1000);
```

## Praktische Patronen

### Beheer van WebSocket-verbindingen

```typescript
import { using, interval, Subject, map, takeUntil } from 'rxjs';
function createWebSocketStream(url: string) {
  return using(
    // Maak WebSocket-verbinding aan
    () => {
      const ws = new WebSocket(url);
      console.log('WebSocket-verbinding gestart:', url);

      ws.onopen = () => console.log('Verbinding voltooid');
      ws.onerror = (error) => console.error('Verbindingsfout:', error);

      return {
        unsubscribe: () => {
          console.log('WebSocket-verbinding gesloten');
          ws.close();
        }
      };
    },
    // Maak berichtenstroom aan
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

// Gebruiksvoorbeeld
const websocket$ = createWebSocketStream('wss://echo.websocket.org');

const subscription = websocket$.subscribe({
  next: message => console.log('Ontvangen:', message.data),
  error: error => console.error('Fout:', error),
  complete: () => console.log('Voltooid')
});

// WebSocket automatisch sluiten na 10 seconden
setTimeout(() => subscription.unsubscribe(), 10000);
```

### Automatische timer-opruiming

```typescript
import { using, Observable, Subscription } from 'rxjs';

function createTimerStream(intervalMs: number) {
  return using(
    // Maak timer-resource aan
    () => {
      let timerId: number | null = null;
      console.log('Timer gestart');

      return new Subscription(() => {
        if (timerId !== null) {
          clearInterval(timerId);
          console.log('Timer gestopt');
        }
      });
    },
    // Maak timerstroom aan
    () => new Observable(subscriber => {
      const timerId = setInterval(() => {
        subscriber.next(Date.now());
      }, intervalMs);

      return () => clearInterval(timerId);
    })
  );
}

// Gebruiksvoorbeeld
const timer$ = createTimerStream(1000);

const subscription = timer$.subscribe({
  next: time => console.log('Huidige tijd:', new Date(time).toLocaleTimeString())
});

// Stop na 5 seconden
setTimeout(() => subscription.unsubscribe(), 5000);
```

### Bestandsmanipulatie (Node.js)

```typescript
import { using, Observable } from 'rxjs';
import * as fs from 'fs';

function readFileStream(filePath: string) {
  return using(
    // Open bestandshandle
    () => {
      const fd = fs.openSync(filePath, 'r');
      console.log('Bestand geopend:', filePath);

      return {
        unsubscribe: () => {
          fs.closeSync(fd);
          console.log('Bestand gesloten');
        }
      };
    },
    // Maak bestandsleesstroom aan
    () => new Observable<string>(subscriber => {
      const stream = fs.createReadStream(filePath, { encoding: 'utf8' });

      stream.on('data', (chunk) => subscriber.next(chunk));
      stream.on('error', (error) => subscriber.error(error));
      stream.on('end', () => subscriber.complete());

      return () => stream.destroy();
    })
  );
}

// Gebruiksvoorbeeld
const file$ = readFileStream('./data.txt');

file$.subscribe({
  next: chunk => console.log('Lezen:', chunk),
  error: error => console.error('Fout:', error),
  complete: () => console.log('Lezen voltooid')
});
```

### Beheer van event listeners

```typescript
import { using, Observable } from 'rxjs';

function createClickStream(element: HTMLElement) {
  return using(
    // Registreer event listener
    () => {
      console.log('Event listener geregistreerd');

      return {
        unsubscribe: () => {
          console.log('Event listener verwijderd');
          // Daadwerkelijke verwijdering wordt gedaan in de Observable factory
        }
      };
    },
    // Maak click-eventstroom aan
    () => new Observable<MouseEvent>(subscriber => {
      const handler = (event: MouseEvent) => subscriber.next(event);

      element.addEventListener('click', handler);

      return () => {
        element.removeEventListener('click', handler);
      };
    })
  );
}

// Gebruiksvoorbeeld
const button = document.querySelector('#myButton') as HTMLElement;
const clicks$ = createClickStream(button);

const subscription = clicks$.subscribe({
  next: event => console.log('Klikpositie:', event.clientX, event.clientY)
});

// Automatisch verwijderen na 30 seconden
setTimeout(() => subscription.unsubscribe(), 30000);
```

## Veelvoorkomende use cases

### 1. Databaseverbindingsbeheer

```typescript
import { using, from, mergeMap } from 'rxjs';
interface DbConnection {
  query: (sql: string) => Promise<any[]>;
  close: () => Promise<void>;
}

function queryWithConnection(sql: string) {
  return using(
    // Stel databaseverbinding in
    () => {
      const connection = createDbConnection();
      console.log('DB-verbinding tot stand gebracht');

      return {
        unsubscribe: async () => {
          await connection.close();
          console.log('DB-verbinding gesloten');
        }
      };
    },
    // Voer query uit
    () => {
      const connection = createDbConnection();
      return from(connection.query(sql));
    }
  );
}

// Gebruiksvoorbeeld
const users$ = queryWithConnection('SELECT * FROM users');

users$.subscribe({
  next: rows => console.log('Opgehaald:', rows),
  error: error => console.error('Fout:', error),
  complete: () => console.log('Query voltooid')
});

function createDbConnection(): DbConnection {
  // Daadwerkelijke verbindingsverwerking
  return {
    query: async (sql) => [],
    close: async () => {}
  };
}
```

### 2. Resourcepoolbeheer

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

// Gebruiksvoorbeeld
const pool = new ResourcePool(() => ({ id: Math.random() }), 5);

function usePooledResource<T>(
  pool: ResourcePool<T>,
  work: (resource: T) => Observable<any>
) {
  return using(
    () => {
      const resource = pool.acquire();
      if (!resource) {
        throw new Error('Resourcepool uitgeput');
      }
      console.log('Resource verkregen:', resource);

      return {
        unsubscribe: () => {
          pool.release(resource);
          console.log('Resource geretourneerd:', resource);
        }
      };
    },
    (subscription) => {
      const resource = pool.acquire();
      return resource ? work(resource) : defer(() => {
        throw new Error('Resource-acquisitie mislukt');
      });
    }
  );
}

// Verwerk met behulp van resource
const work$ = usePooledResource(pool, (resource) =>
  new Observable(subscriber => {
    subscriber.next(`Verwerken: ${resource.id}`);
    setTimeout(() => subscriber.complete(), 1000);
  })
);

work$.subscribe({
  next: result => console.log(result),
  complete: () => console.log('Verwerking voltooid')
});
```

### 3. Gecoördineerd beheer van meerdere resources

```typescript
import { using, merge, Subject } from 'rxjs';

interface MultiResource {
  ws: WebSocket;
  timer: number;
}

function createMultiResourceStream() {
  return using(
    // Maak meerdere resources aan
    () => {
      const ws = new WebSocket('wss://echo.websocket.org');
      const timer = setInterval(() => {
        console.log('Periodieke uitvoering');
      }, 1000);

      console.log('Meerdere resources aangemaakt');

      return {
        unsubscribe: () => {
          ws.close();
          clearInterval(timer);
          console.log('Meerdere resources vrijgegeven');
        }
      };
    },
    // Combineer meerdere stromen
    () => {
      const messages$ = new Subject<string>();
      const ticks$ = new Subject<number>();

      return merge(messages$, ticks$);
    }
  );
}

// Gebruiksvoorbeeld
const multiStream$ = createMultiResourceStream();

const subscription = multiStream$.subscribe({
  next: value => console.log('Ontvangen:', value)
});

// Geef alle resources vrij na 10 seconden
setTimeout(() => subscription.unsubscribe(), 10000);
```

### 4. Voorwaardelijk resourcebeheer

```typescript
import { using, interval, EMPTY, take } from 'rxjs';
function conditionalResource(shouldCreate: boolean) {
  return using(
    () => {
      if (shouldCreate) {
        console.log('Resource aangemaakt');
        return {
          unsubscribe: () => console.log('Resource vrijgegeven')
        };
      } else {
        console.log('Resource-aanmaak overgeslagen');
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

// Bij het aanmaken van resources
conditionalResource(true).subscribe({
  next: val => console.log('Waarde:', val),
  complete: () => console.log('Voltooid')
});

// Bij het niet aanmaken van resources
conditionalResource(false).subscribe({
  next: val => console.log('Waarde:', val),
  complete: () => console.log('Voltooid')
});
```

## Foutafhandeling

### Resource-vrijgave bij fout

```typescript
import { using, throwError, of, catchError } from 'rxjs';
const errorHandling$ = using(
  () => {
    console.log('Resource aangemaakt');
    return {
      unsubscribe: () => console.log('Resource vrijgegeven (zelfs bij fout uitgevoerd)')
    };
  },
  () => throwError(() => new Error('Opzettelijke fout'))
);

errorHandling$.pipe(
  catchError(error => {
    console.error('Fout opgevangen:', error.message);
    return of('Standaardwaarde');
  })
).subscribe({
  next: val => console.log('Waarde:', val),
  complete: () => console.log('Voltooid')
});

// Output:
// Resource aangemaakt
// Resource vrijgegeven (zelfs bij fout uitgevoerd)
// Fout opgevangen: Opzettelijke fout
// Waarde: Standaardwaarde
// Voltooid
```

> [!IMPORTANT]
> **Betrouwbare resource-vrijgave zelfs bij fout**
>
> `using()` geeft altijd de resource vrij die is aangemaakt in `resourceFactory`, zelfs als er een fout optreedt.

## Veelvoorkomende fouten en hoe deze te behandelen

### 1. Vergeten unsubscribe-methode te implementeren

**Foutvoorbeeld:**
```typescript
// ❌ Fout: geen unsubscribe-methode
using(
  () => {
    console.log('Resource aangemaakt');
    return {}; // geen unsubscribe
  },
  () => interval(1000)
);
```

**Oplossing:**
```typescript
// ✅ Correct: implementeer unsubscribe-methode
using(
  () => {
    console.log('Resource aangemaakt');
    return {
      unsubscribe: () => console.log('Resource vrijgegeven')
    };
  },
  () => interval(1000)
);
```

### 2. Asynchroon resources aanmaken

**Probleem:**
```typescript
// ❌ Probleem: resourceFactory kan niet asynchroon zijn
using(
  async () => { // async kan niet worden gebruikt
    const resource = await createResourceAsync();
    return resource;
  },
  () => interval(1000)
);
```

**Oplossing:**
```typescript
import { defer, from, mergeMap } from 'rxjs';
// ✅ Correct: behandel asynchrone verwerking met defer en mergeMap

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

### 3. Dubbele resource-aanmaak

**Probleem:**
```typescript
// ❌ Probleem: maak resources afzonderlijk aan in resourceFactory en observableFactory
let sharedResource: any;

using(
  () => {
    sharedResource = createResource(); // Maak hier aan
    return { unsubscribe: () => sharedResource.close() };
  },
  () => {
    const resource = createResource(); // Maak opnieuw aan
    return from(resource.getData());
  }
);
```

**Oplossing:**
```typescript
// ✅ Correct: deel resources
using(
  () => {
    const resource = createResource();
    return {
      resource, // Houd resource vast
      unsubscribe: () => resource.close()
    };
  },
  (subscription: any) => {
    return from(subscription.resource.getData());
  }
);
```

## Best practices voor using()

### 1. Zorg voor resource-vrijgave

```typescript
// ✅ Goed voorbeeld: try-finally patroon
using(
  () => {
    const resource = createResource();
    return {
      unsubscribe: () => {
        try {
          resource.close();
        } catch (error) {
          console.error('Fout bij resource-vrijgave:', error);
        }
      }
    };
  },
  () => interval(1000)
);
```

### 2. Resource-aanmaak logging

```typescript
// ✅ Goed voorbeeld: log resource-levenscyclus
using(
  () => {
    const resourceId = Math.random();
    console.log(`[${resourceId}] Resource aangemaakt`);

    return {
      unsubscribe: () => {
        console.log(`[${resourceId}] Resource vrijgegeven`);
      }
    };
  },
  () => interval(1000)
);
```

### 3. Type-veilig resourcebeheer

```typescript
// ✅ Goed voorbeeld: benut TypeScript-types
interface ManagedResource {
  id: string;
  close: () => void;
}

function createManagedStream(resource: ManagedResource) {
  return using(
    () => {
      console.log('Resource gestart:', resource.id);
      return {
        unsubscribe: () => {
          resource.close();
          console.log('Resource beëindigd:', resource.id);
        }
      };
    },
    () => interval(1000)
  );
}
```

## Vergelijking met handmatig beheer

### Handmatig resourcebeheer (❌ niet aanbevolen)

```typescript
// ❌ Slecht voorbeeld: handmatig beheer (risico op vergeten vrij te geven)
const ws = new WebSocket('wss://example.com');
const subscription = interval(1000).subscribe(() => {
  ws.send('ping');
});

// Kan vergeten vrij te geven
// subscription.unsubscribe();
// ws.close();
```

### Resourcebeheer door using() (✅ aanbevolen)

```typescript
// ✅ Goed voorbeeld: automatisch beheer met using()
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
  // Verwerking met behulp van WebSocket
});

// WebSocket wordt ook automatisch gesloten met unsubscribe()
subscription.unsubscribe();
```

## Samenvatting

`using()` is een Creation Function die automatisch resources beheert volgens de levenscyclus van de Observable.

**Belangrijkste kenmerken:**
- Maakt een resource aan bij het begin van een subscription
- Automatische vrijgave aan het einde van subscription (complete of unsubscribe)
- Voorkomt geheugenlekken
- Betrouwbare resource-vrijgave zelfs bij fout

**Use cases:**
- Netwerkverbindingen zoals WebSocket, EventSource
- Bestandshandles, databaseverbindingen
- Automatische opruiming van timers en intervals
- Automatische deactivering van event listeners

**Opmerking:**
- `resourceFactory` moet een synchrone functie zijn
- Implementeer altijd de `unsubscribe` methode
- Zorg voor goede foutafhandeling

**Aanbevolen gebruik:**
- Voorkom vergeten resources vrij te geven
- Log de levenscyclus
- Benut TypeScript-types voor type-veilig beheer

## Gerelateerde Pagina's

- [scheduled()](/nl/guide/creation-functions/control/scheduled) - Genereer Observable met opgegeven scheduler
- [Control Creation Functions](/nl/guide/creation-functions/control/) - Vergelijking van scheduled() en using()
- [finalize()](/nl/guide/error-handling/finalize) - Operator om verwerking toe te voegen aan het einde van subscription

## Referenties

- [RxJS Officiële Documentatie - using](https://rxjs.dev/api/index/function/using)
- [RxJS Officiële Documentatie - Subscription](https://rxjs.dev/guide/subscription)
