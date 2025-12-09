---
description: using() ist eine RxJS Creation Function, die Ressourcen automatisch im Einklang mit dem Observable-Lebenszyklus erstellt und freigibt. Sie verwaltet sicher Ressourcen wie WebSocket, Dateihandles und Timer, die manuelles Cleanup erfordern, und verhindert Memory Leaks. Zuverlässiges Ressourcenmanagement mit automatischer Freigabe beim Abonnement-Ende.
---

# using()

[📘 RxJS Offizielle Dokumentation - using](https://rxjs.dev/api/index/function/using)

`using()` ist eine Creation Function, die Ressourcen automatisch im Einklang mit dem Lebenszyklus eines Observables erstellt und freigibt. Sie verwaltet sicher Ressourcen wie WebSocket, Dateihandles und Timer, die manuelles Cleanup erfordern, und verhindert Memory Leaks.

## Grundlegende Verwendung

### Einfaches Ressourcenmanagement

```typescript
import { using, interval, Subscription, take } from 'rxjs';
const resource$ = using(
  // Ressourcen-Factory: wird beim Start des Abonnements ausgeführt
  () => {
    console.log('Ressource erstellen');
    return new Subscription(() => console.log('Ressource freigeben'));
  },
  // Observable-Factory: Observable mit der Ressource erstellen
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Wert:', value),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Ressource erstellen
// Wert: 0
// Wert: 1
// Wert: 2
// Abgeschlossen
// Ressource freigeben
```

> [!IMPORTANT]
> **Automatische Ressourcenfreigabe**
>
> `using()` gibt Ressourcen automatisch frei, wenn das Observable abgeschlossen (`complete`) oder das Abonnement beendet (`unsubscribe`) wird.

## Funktionsweise von using()

`using()` nimmt folgende zwei Funktionen entgegen:

```typescript
function using<T>(
  resourceFactory: () => Unsubscribable | void,
  observableFactory: (resource: Unsubscribable | void) => ObservableInput<T>
): Observable<T>
```

### 1. resourceFactory (Ressourcen-Factory)

Wird beim Start des Abonnements ausgeführt und erstellt die Ressource. Es muss ein Objekt mit einer `unsubscribe()`-Methode zurückgegeben werden.

```typescript
// Subscription zurückgeben
() => new Subscription(() => {
  console.log('Cleanup-Verarbeitung');
});

// Oder ein Objekt mit unsubscribe-Methode zurückgeben
() => ({
  unsubscribe: () => {
    console.log('Cleanup-Verarbeitung');
  }
});
```

### 2. observableFactory (Observable-Factory)

Erstellt ein Observable unter Verwendung der Ressource.

```typescript
(resource) => interval(1000);
```

## Praktische Muster

### Verwaltung von WebSocket-Verbindungen

```typescript
import { using, interval, Subject, map, takeUntil } from 'rxjs';
function createWebSocketStream(url: string) {
  return using(
    // WebSocket-Verbindung erstellen
    () => {
      const ws = new WebSocket(url);
      console.log('WebSocket-Verbindung starten:', url);

      ws.onopen = () => console.log('Verbindung hergestellt');
      ws.onerror = (error) => console.error('Verbindungsfehler:', error);

      return {
        unsubscribe: () => {
          console.log('WebSocket-Verbindung beenden');
          ws.close();
        }
      };
    },
    // Nachrichten-Stream erstellen
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

// Verwendungsbeispiel
const websocket$ = createWebSocketStream('wss://echo.websocket.org');

const subscription = websocket$.subscribe({
  next: message => console.log('Empfangen:', message.data),
  error: error => console.error('Fehler:', error),
  complete: () => console.log('Abgeschlossen')
});

// WebSocket nach 10 Sekunden automatisch schließen
setTimeout(() => subscription.unsubscribe(), 10000);
```

### Automatisches Cleanup von Timern

```typescript
import { using, Observable, Subscription } from 'rxjs';

function createTimerStream(intervalMs: number) {
  return using(
    // Timer-Ressource erstellen
    () => {
      let timerId: number | null = null;
      console.log('Timer starten');

      return new Subscription(() => {
        if (timerId !== null) {
          clearInterval(timerId);
          console.log('Timer stoppen');
        }
      });
    },
    // Timer-Stream erstellen
    () => new Observable(subscriber => {
      const timerId = setInterval(() => {
        subscriber.next(Date.now());
      }, intervalMs);

      return () => clearInterval(timerId);
    })
  );
}

// Verwendungsbeispiel
const timer$ = createTimerStream(1000);

const subscription = timer$.subscribe({
  next: time => console.log('Aktuelle Zeit:', new Date(time).toLocaleTimeString())
});

// Nach 5 Sekunden stoppen
setTimeout(() => subscription.unsubscribe(), 5000);
```

### Dateioperationen (Node.js)

```typescript
import { using, Observable } from 'rxjs';
import * as fs from 'fs';

function readFileStream(filePath: string) {
  return using(
    // Datei-Handle öffnen
    () => {
      const fd = fs.openSync(filePath, 'r');
      console.log('Datei öffnen:', filePath);

      return {
        unsubscribe: () => {
          fs.closeSync(fd);
          console.log('Datei schließen');
        }
      };
    },
    // Datei-Lese-Stream erstellen
    () => new Observable<string>(subscriber => {
      const stream = fs.createReadStream(filePath, { encoding: 'utf8' });

      stream.on('data', (chunk) => subscriber.next(chunk));
      stream.on('error', (error) => subscriber.error(error));
      stream.on('end', () => subscriber.complete());

      return () => stream.destroy();
    })
  );
}

// Verwendungsbeispiel
const file$ = readFileStream('./data.txt');

file$.subscribe({
  next: chunk => console.log('Lesen:', chunk),
  error: error => console.error('Fehler:', error),
  complete: () => console.log('Lesen abgeschlossen')
});
```

### Verwaltung von Event-Listenern

```typescript
import { using, Observable } from 'rxjs';

function createClickStream(element: HTMLElement) {
  return using(
    // Event-Listener registrieren
    () => {
      console.log('Event-Listener registrieren');

      return {
        unsubscribe: () => {
          console.log('Event-Listener entfernen');
          // Tatsächliches Entfernen erfolgt in der Observable-Factory
        }
      };
    },
    // Click-Event-Stream erstellen
    () => new Observable<MouseEvent>(subscriber => {
      const handler = (event: MouseEvent) => subscriber.next(event);

      element.addEventListener('click', handler);

      return () => {
        element.removeEventListener('click', handler);
      };
    })
  );
}

// Verwendungsbeispiel
const button = document.querySelector('#myButton') as HTMLElement;
const clicks$ = createClickStream(button);

const subscription = clicks$.subscribe({
  next: event => console.log('Click-Position:', event.clientX, event.clientY)
});

// Nach 30 Sekunden automatisch entfernen
setTimeout(() => subscription.unsubscribe(), 30000);
```

## Häufige Verwendungsbeispiele

### 1. Verwaltung von Datenbankverbindungen

```typescript
import { using, from, mergeMap } from 'rxjs';
interface DbConnection {
  query: (sql: string) => Promise<any[]>;
  close: () => Promise<void>;
}

function queryWithConnection(sql: string) {
  return using(
    // Datenbankverbindung herstellen
    () => {
      const connection = createDbConnection();
      console.log('DB-Verbindung hergestellt');

      return {
        unsubscribe: async () => {
          await connection.close();
          console.log('DB-Verbindung geschlossen');
        }
      };
    },
    // Query ausführen
    () => {
      const connection = createDbConnection();
      return from(connection.query(sql));
    }
  );
}

// Verwendungsbeispiel
const users$ = queryWithConnection('SELECT * FROM users');

users$.subscribe({
  next: rows => console.log('Abgerufen:', rows),
  error: error => console.error('Fehler:', error),
  complete: () => console.log('Query abgeschlossen')
});

function createDbConnection(): DbConnection {
  // Tatsächliche Verbindungsverarbeitung
  return {
    query: async (sql) => [],
    close: async () => {}
  };
}
```

### 2. Verwaltung von Ressourcenpools

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

// Verwendungsbeispiel
const pool = new ResourcePool(() => ({ id: Math.random() }), 5);

function usePooledResource<T>(
  pool: ResourcePool<T>,
  work: (resource: T) => Observable<any>
) {
  return using(
    () => {
      const resource = pool.acquire();
      if (!resource) {
        throw new Error('Ressourcenpool erschöpft');
      }
      console.log('Ressource erhalten:', resource);

      return {
        unsubscribe: () => {
          pool.release(resource);
          console.log('Ressource zurückgegeben:', resource);
        }
      };
    },
    (subscription) => {
      const resource = pool.acquire();
      return resource ? work(resource) : defer(() => {
        throw new Error('Ressourcenabruf fehlgeschlagen');
      });
    }
  );
}

// Ressource für Verarbeitung verwenden
const work$ = usePooledResource(pool, (resource) =>
  new Observable(subscriber => {
    subscriber.next(`Verarbeitung: ${resource.id}`);
    setTimeout(() => subscriber.complete(), 1000);
  })
);

work$.subscribe({
  next: result => console.log(result),
  complete: () => console.log('Verarbeitung abgeschlossen')
});
```

### 3. Koordinierte Verwaltung mehrerer Ressourcen

```typescript
import { using, merge, Subject } from 'rxjs';

interface MultiResource {
  ws: WebSocket;
  timer: number;
}

function createMultiResourceStream() {
  return using(
    // Mehrere Ressourcen erstellen
    () => {
      const ws = new WebSocket('wss://echo.websocket.org');
      const timer = setInterval(() => {
        console.log('Periodische Ausführung');
      }, 1000);

      console.log('Mehrere Ressourcen erstellen');

      return {
        unsubscribe: () => {
          ws.close();
          clearInterval(timer);
          console.log('Mehrere Ressourcen freigeben');
        }
      };
    },
    // Mehrere Streams kombinieren
    () => {
      const messages$ = new Subject<string>();
      const ticks$ = new Subject<number>();

      return merge(messages$, ticks$);
    }
  );
}

// Verwendungsbeispiel
const multiStream$ = createMultiResourceStream();

const subscription = multiStream$.subscribe({
  next: value => console.log('Empfangen:', value)
});

// Alle Ressourcen nach 10 Sekunden freigeben
setTimeout(() => subscription.unsubscribe(), 10000);
```

### 4. Bedingte Ressourcenverwaltung

```typescript
import { using, interval, EMPTY, take } from 'rxjs';
function conditionalResource(shouldCreate: boolean) {
  return using(
    () => {
      if (shouldCreate) {
        console.log('Ressource erstellen');
        return {
          unsubscribe: () => console.log('Ressource freigeben')
        };
      } else {
        console.log('Ressourcenerstellung überspringen');
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

// Wenn Ressource erstellt werden soll
conditionalResource(true).subscribe({
  next: val => console.log('Wert:', val),
  complete: () => console.log('Abgeschlossen')
});

// Wenn Ressource nicht erstellt werden soll
conditionalResource(false).subscribe({
  next: val => console.log('Wert:', val),
  complete: () => console.log('Abgeschlossen')
});
```

## Fehlerbehandlung

### Ressourcenfreigabe bei Fehlerauftreten

```typescript
import { using, throwError, of, catchError } from 'rxjs';
const errorHandling$ = using(
  () => {
    console.log('Ressource erstellen');
    return {
      unsubscribe: () => console.log('Ressource freigeben (auch bei Fehler ausgeführt)')
    };
  },
  () => throwError(() => new Error('Absichtlicher Fehler'))
);

errorHandling$.pipe(
  catchError(error => {
    console.error('Fehler abgefangen:', error.message);
    return of('Standardwert');
  })
).subscribe({
  next: val => console.log('Wert:', val),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Ressource erstellen
// Ressource freigeben (auch bei Fehler ausgeführt)
// Fehler abgefangen: Absichtlicher Fehler
// Wert: Standardwert
// Abgeschlossen
```

> [!IMPORTANT]
> **Zuverlässige Ressourcenfreigabe auch bei Fehlern**
>
> `using()` gibt die in `resourceFactory` erstellte Ressource auch bei Fehlern zuverlässig frei.

## Häufige Fehler und Lösungen

### 1. Vergessene Implementierung der unsubscribe-Methode

**Fehlerbeispiel:**
```typescript
// ❌ Fehler: unsubscribe-Methode fehlt
using(
  () => {
    console.log('Ressource erstellen');
    return {}; // unsubscribe fehlt
  },
  () => interval(1000)
);
```

**Lösung:**
```typescript
// ✅ Korrekt: unsubscribe-Methode implementieren
using(
  () => {
    console.log('Ressource erstellen');
    return {
      unsubscribe: () => console.log('Ressource freigeben')
    };
  },
  () => interval(1000)
);
```

### 2. Asynchrone Ressourcenerstellung

**Problem:**
```typescript
// ❌ Problem: resourceFactory kann nicht asynchron sein
using(
  async () => { // async nicht verwendbar
    const resource = await createResourceAsync();
    return resource;
  },
  () => interval(1000)
);
```

**Lösung:**
```typescript
import { defer, from, mergeMap } from 'rxjs';
// ✅ Korrekt: Asynchrone Verarbeitung mit defer und mergeMap

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

### 3. Doppelte Ressourcenerstellung

**Problem:**
```typescript
// ❌ Problem: Separate Ressourcenerstellung in resourceFactory und observableFactory
let sharedResource: any;

using(
  () => {
    sharedResource = createResource(); // Hier erstellen
    return { unsubscribe: () => sharedResource.close() };
  },
  () => {
    const resource = createResource(); // Erneut erstellen
    return from(resource.getData());
  }
);
```

**Lösung:**
```typescript
// ✅ Korrekt: Ressource teilen
using(
  () => {
    const resource = createResource();
    return {
      resource, // Ressource speichern
      unsubscribe: () => resource.close()
    };
  },
  (subscription: any) => {
    return from(subscription.resource.getData());
  }
);
```

## Best Practices für using()

### 1. Zuverlässige Ressourcenfreigabe

```typescript
// ✅ Gutes Beispiel: Try-finally-Muster
using(
  () => {
    const resource = createResource();
    return {
      unsubscribe: () => {
        try {
          resource.close();
        } catch (error) {
          console.error('Ressourcenfreigabe-Fehler:', error);
        }
      }
    };
  },
  () => interval(1000)
);
```

### 2. Protokollierung des Ressourcen-Lebenszyklus

```typescript
// ✅ Gutes Beispiel: Ressourcen-Lebenszyklus protokollieren
using(
  () => {
    const resourceId = Math.random();
    console.log(`[${resourceId}] Ressource erstellen`);

    return {
      unsubscribe: () => {
        console.log(`[${resourceId}] Ressource freigeben`);
      }
    };
  },
  () => interval(1000)
);
```

### 3. Typsicheres Ressourcenmanagement

```typescript
// ✅ Gutes Beispiel: TypeScript-Typen nutzen
interface ManagedResource {
  id: string;
  close: () => void;
}

function createManagedStream(resource: ManagedResource) {
  return using(
    () => {
      console.log('Ressource starten:', resource.id);
      return {
        unsubscribe: () => {
          resource.close();
          console.log('Ressource beenden:', resource.id);
        }
      };
    },
    () => interval(1000)
  );
}
```

## Vergleich mit manueller Verwaltung

### Manuelle Ressourcenverwaltung (❌ nicht empfohlen)

```typescript
// ❌ Schlechtes Beispiel: Manuelle Verwaltung (Risiko des Vergessens)
const ws = new WebSocket('wss://example.com');
const subscription = interval(1000).subscribe(() => {
  ws.send('ping');
});

// Vergessen der Freigabe möglich
// subscription.unsubscribe();
// ws.close();
```

### Ressourcenverwaltung mit using() (✅ empfohlen)

```typescript
// ✅ Gutes Beispiel: Automatische Verwaltung mit using()
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
  // Verarbeitung mit WebSocket
});

// WebSocket wird automatisch geschlossen mit nur unsubscribe()
subscription.unsubscribe();
```

## Zusammenfassung

`using()` ist eine Creation Function zur automatischen Verwaltung von Ressourcen im Einklang mit dem Observable-Lebenszyklus.

**Hauptmerkmale:**
- Erstellt Ressourcen beim Start des Abonnements
- Automatische Freigabe beim Ende des Abonnements (complete oder unsubscribe)
- Verhindert Memory Leaks
- Zuverlässige Ressourcenfreigabe auch bei Fehlern

**Verwendungsszenarien:**
- Netzwerkverbindungen wie WebSocket, EventSource
- Dateihandles, Datenbankverbindungen
- Automatisches Cleanup von Timern und Intervallen
- Automatisches Entfernen von Event-Listenern

**Wichtige Punkte:**
- `resourceFactory` muss eine synchrone Funktion sein
- Immer `unsubscribe`-Methode implementieren
- Angemessene Fehlerbehandlung durchführen

**Empfohlene Verwendung:**
- Vergessen der Ressourcenfreigabe vermeiden
- Lebenszyklus protokollieren
- Typsichere Verwaltung mit TypeScript-Typen

## Verwandte Seiten

- [scheduled()](/de/guide/creation-functions/control/scheduled) - Observable mit angegebenem Scheduler erzeugen
- [Steuerungs-Creation-Functions](/de/guide/creation-functions/control/) - Vergleich zwischen scheduled() und using()
- [finalize()](/de/guide/error-handling/finalize) - Operator zum Hinzufügen von Verarbeitung beim Abonnement-Ende

## Referenzressourcen

- [RxJS Offizielle Dokumentation - using](https://rxjs.dev/api/index/function/using)
- [RxJS Offizielle Dokumentation - Subscription](https://rxjs.dev/guide/subscription)
