---
description: using() è una Funzione di Creazione RxJS che crea e rilascia automaticamente le risorse secondo il ciclo di vita dell'Observable. Gestisce in modo sicuro le risorse che richiedono pulizia manuale come WebSocket, file handle e timer, e previene le perdite di memoria. Assicura una gestione affidabile delle risorse creando risorse all'inizio di una subscription e rilasciandole automaticamente alla fine della subscription.
---

# using()

[📘 Documentazione Ufficiale RxJS - using](https://rxjs.dev/api/index/function/using)

`using()` è una Funzione di Creazione che crea e dealloca automaticamente le risorse secondo il ciclo di vita dell'Observable, gestendo in modo sicuro le risorse che devono essere pulite manualmente, come WebSocket, file handle e timer, e previene le perdite di memoria.

## Uso Base

### Gestione semplice delle risorse

```typescript
import { using, interval, Subscription, take } from 'rxjs';
const resource$ = using(
  // Factory delle risorse: eseguita all'inizio della subscription
  () => {
    console.log('Risorsa creata');
    return new Subscription(() => console.log('Risorsa rilasciata'));
  },
  // Factory dell'Observable: crea Observable usando la risorsa
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Valore:', value),
  complete: () => console.log('Completo')
});

// Output:
// Risorsa creata
// Valore: 0
// Valore: 1
// Valore: 2
// Completo
// Risorsa rilasciata
```

> [!IMPORTANT]
> **Rilascio automatico delle risorse**
>
> `using()` rilascia automaticamente le risorse quando l'Observable completa (`complete`) o viene annullata la subscription (`unsubscribe`).

## Come funziona using()

`using()` prende le seguenti due funzioni.

```typescript
function using<T>(
  resourceFactory: () => Unsubscribable | void,
  observableFactory: (resource: Unsubscribable | void) => ObservableInput<T>
): Observable<T>
```

### 1. resourceFactory

Viene eseguita all'inizio di una subscription per creare una risorsa. Deve restituire un oggetto con un metodo `unsubscribe()`.

```typescript
// Restituisci una Subscription
() => new Subscription(() => {
  console.log('Elaborazione di pulizia');
});

// Oppure restituisci un oggetto con metodo unsubscribe
() => ({
  unsubscribe: () => {
    console.log('Elaborazione di pulizia');
  }
});
```

### 2. observableFactory

Crea un Observable con una risorsa.

```typescript
(resource) => interval(1000);
```

## Pattern Pratici

### Gestione connessioni WebSocket

```typescript
import { using, interval, Subject, map, takeUntil } from 'rxjs';
function createWebSocketStream(url: string) {
  return using(
    // Crea connessione WebSocket
    () => {
      const ws = new WebSocket(url);
      console.log('Connessione WebSocket iniziata:', url);

      ws.onopen = () => console.log('Connessione completata');
      ws.onerror = (error) => console.error('Errore di connessione:', error);

      return {
        unsubscribe: () => {
          console.log('Connessione WebSocket chiusa');
          ws.close();
        }
      };
    },
    // Crea stream messaggi
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

// Esempio di utilizzo
const websocket$ = createWebSocketStream('wss://echo.websocket.org');

const subscription = websocket$.subscribe({
  next: message => console.log('Ricevuto:', message.data),
  error: error => console.error('Errore:', error),
  complete: () => console.log('Completo')
});

// Chiudi automaticamente WebSocket dopo 10 secondi
setTimeout(() => subscription.unsubscribe(), 10000);
```

### Pulizia automatica timer

```typescript
import { using, Observable, Subscription } from 'rxjs';

function createTimerStream(intervalMs: number) {
  return using(
    // Crea risorsa timer
    () => {
      let timerId: number | null = null;
      console.log('Timer avviato');

      return new Subscription(() => {
        if (timerId !== null) {
          clearInterval(timerId);
          console.log('Timer fermato');
        }
      });
    },
    // Crea stream timer
    () => new Observable(subscriber => {
      const timerId = setInterval(() => {
        subscriber.next(Date.now());
      }, intervalMs);

      return () => clearInterval(timerId);
    })
  );
}

// Esempio di utilizzo
const timer$ = createTimerStream(1000);

const subscription = timer$.subscribe({
  next: time => console.log('Ora corrente:', new Date(time).toLocaleTimeString())
});

// Ferma dopo 5 secondi
setTimeout(() => subscription.unsubscribe(), 5000);
```

### Manipolazione file (Node.js)

```typescript
import { using, Observable } from 'rxjs';
import * as fs from 'fs';

function readFileStream(filePath: string) {
  return using(
    // Apri file handle
    () => {
      const fd = fs.openSync(filePath, 'r');
      console.log('File aperto:', filePath);

      return {
        unsubscribe: () => {
          fs.closeSync(fd);
          console.log('File chiuso');
        }
      };
    },
    // Crea stream di lettura file
    () => new Observable<string>(subscriber => {
      const stream = fs.createReadStream(filePath, { encoding: 'utf8' });

      stream.on('data', (chunk) => subscriber.next(chunk));
      stream.on('error', (error) => subscriber.error(error));
      stream.on('end', () => subscriber.complete());

      return () => stream.destroy();
    })
  );
}

// Esempio di utilizzo
const file$ = readFileStream('./data.txt');

file$.subscribe({
  next: chunk => console.log('Lettura:', chunk),
  error: error => console.error('Errore:', error),
  complete: () => console.log('Lettura completata')
});
```

### Gestione event listener

```typescript
import { using, Observable } from 'rxjs';

function createClickStream(element: HTMLElement) {
  return using(
    // Registra event listener
    () => {
      console.log('Event listener registrato');

      return {
        unsubscribe: () => {
          console.log('Event listener rimosso');
          // La rimozione effettiva viene fatta nella factory dell'Observable
        }
      };
    },
    // Crea stream eventi click
    () => new Observable<MouseEvent>(subscriber => {
      const handler = (event: MouseEvent) => subscriber.next(event);

      element.addEventListener('click', handler);

      return () => {
        element.removeEventListener('click', handler);
      };
    })
  );
}

// Esempio di utilizzo
const button = document.querySelector('#myButton') as HTMLElement;
const clicks$ = createClickStream(button);

const subscription = clicks$.subscribe({
  next: event => console.log('Posizione click:', event.clientX, event.clientY)
});

// Rimozione automatica dopo 30 secondi
setTimeout(() => subscription.unsubscribe(), 30000);
```

## Casi d'Uso Comuni

### 1. Gestione connessioni database

```typescript
import { using, from, mergeMap } from 'rxjs';
interface DbConnection {
  query: (sql: string) => Promise<any[]>;
  close: () => Promise<void>;
}

function queryWithConnection(sql: string) {
  return using(
    // Stabilisci connessione database
    () => {
      const connection = createDbConnection();
      console.log('Connessione DB stabilita');

      return {
        unsubscribe: async () => {
          await connection.close();
          console.log('Connessione DB chiusa');
        }
      };
    },
    // Esegui query
    () => {
      const connection = createDbConnection();
      return from(connection.query(sql));
    }
  );
}

// Esempio di utilizzo
const users$ = queryWithConnection('SELECT * FROM users');

users$.subscribe({
  next: rows => console.log('Recuperati:', rows),
  error: error => console.error('Errore:', error),
  complete: () => console.log('Query completata')
});

function createDbConnection(): DbConnection {
  // Elaborazione connessione effettiva
  return {
    query: async (sql) => [],
    close: async () => {}
  };
}
```

### 2. Gestione pool di risorse

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

// Esempio di utilizzo
const pool = new ResourcePool(() => ({ id: Math.random() }), 5);

function usePooledResource<T>(
  pool: ResourcePool<T>,
  work: (resource: T) => Observable<any>
) {
  return using(
    () => {
      const resource = pool.acquire();
      if (!resource) {
        throw new Error('Pool di risorse esaurito');
      }
      console.log('Risorsa acquisita:', resource);

      return {
        unsubscribe: () => {
          pool.release(resource);
          console.log('Risorsa restituita:', resource);
        }
      };
    },
    (subscription) => {
      const resource = pool.acquire();
      return resource ? work(resource) : defer(() => {
        throw new Error('Acquisizione risorsa fallita');
      });
    }
  );
}

// Elabora usando risorsa
const work$ = usePooledResource(pool, (resource) =>
  new Observable(subscriber => {
    subscriber.next(`Elaborazione: ${resource.id}`);
    setTimeout(() => subscriber.complete(), 1000);
  })
);

work$.subscribe({
  next: result => console.log(result),
  complete: () => console.log('Elaborazione completata')
});
```

### 3. Gestione coordinata di risorse multiple

```typescript
import { using, merge, Subject } from 'rxjs';

interface MultiResource {
  ws: WebSocket;
  timer: number;
}

function createMultiResourceStream() {
  return using(
    // Crea risorse multiple
    () => {
      const ws = new WebSocket('wss://echo.websocket.org');
      const timer = setInterval(() => {
        console.log('Esecuzione periodica');
      }, 1000);

      console.log('Risorse multiple create');

      return {
        unsubscribe: () => {
          ws.close();
          clearInterval(timer);
          console.log('Risorse multiple rilasciate');
        }
      };
    },
    // Combina stream multipli
    () => {
      const messages$ = new Subject<string>();
      const ticks$ = new Subject<number>();

      return merge(messages$, ticks$);
    }
  );
}

// Esempio di utilizzo
const multiStream$ = createMultiResourceStream();

const subscription = multiStream$.subscribe({
  next: value => console.log('Ricevuto:', value)
});

// Rilascia tutte le risorse dopo 10 secondi
setTimeout(() => subscription.unsubscribe(), 10000);
```

### 4. Gestione risorse condizionale

```typescript
import { using, interval, EMPTY, take } from 'rxjs';
function conditionalResource(shouldCreate: boolean) {
  return using(
    () => {
      if (shouldCreate) {
        console.log('Risorsa creata');
        return {
          unsubscribe: () => console.log('Risorsa rilasciata')
        };
      } else {
        console.log('Creazione risorsa saltata');
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

// Quando si creano risorse
conditionalResource(true).subscribe({
  next: val => console.log('Valore:', val),
  complete: () => console.log('Completo')
});

// Quando non si creano risorse
conditionalResource(false).subscribe({
  next: val => console.log('Valore:', val),
  complete: () => console.log('Completo')
});
```

## Gestione Errori

### Rilascio risorse su errore

```typescript
import { using, throwError, of, catchError } from 'rxjs';
const errorHandling$ = using(
  () => {
    console.log('Risorsa creata');
    return {
      unsubscribe: () => console.log('Risorsa rilasciata (eseguito anche su errore)')
    };
  },
  () => throwError(() => new Error('Errore intenzionale'))
);

errorHandling$.pipe(
  catchError(error => {
    console.error('Errore catturato:', error.message);
    return of('Valore default');
  })
).subscribe({
  next: val => console.log('Valore:', val),
  complete: () => console.log('Completo')
});

// Output:
// Risorsa creata
// Risorsa rilasciata (eseguito anche su errore)
// Errore catturato: Errore intenzionale
// Valore: Valore default
// Completo
```

> [!IMPORTANT]
> **Rilascio affidabile risorse anche su errore**
>
> `using()` rilascia sempre la risorsa creata in `resourceFactory`, anche se si verifica un errore.

## Errori Comuni e Come Gestirli

### 1. Dimenticato di implementare il metodo unsubscribe

**Esempio di errore:**
```typescript
// ❌ Errore: nessun metodo unsubscribe
using(
  () => {
    console.log('Risorsa creata');
    return {}; // nessun unsubscribe
  },
  () => interval(1000)
);
```

**Soluzione:**
```typescript
// ✅ Corretto: implementa metodo unsubscribe
using(
  () => {
    console.log('Risorsa creata');
    return {
      unsubscribe: () => console.log('Risorsa rilasciata')
    };
  },
  () => interval(1000)
);
```

### 2. Creazione risorse asincrone

**Problema:**
```typescript
// ❌ Problema: resourceFactory non può essere asincrona
using(
  async () => { // async non può essere usato
    const resource = await createResourceAsync();
    return resource;
  },
  () => interval(1000)
);
```

**Soluzione:**
```typescript
import { defer, from, mergeMap } from 'rxjs';
// ✅ Corretto: gestisci elaborazione asincrona con defer e mergeMap

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

### 3. Creazione risorse duplicate

**Problema:**
```typescript
// ❌ Problema: crea risorse separatamente in resourceFactory e observableFactory
let sharedResource: any;

using(
  () => {
    sharedResource = createResource(); // Crea qui
    return { unsubscribe: () => sharedResource.close() };
  },
  () => {
    const resource = createResource(); // Crea di nuovo
    return from(resource.getData());
  }
);
```

**Soluzione:**
```typescript
// ✅ Corretto: condividi le risorse
using(
  () => {
    const resource = createResource();
    return {
      resource, // Mantieni risorsa
      unsubscribe: () => resource.close()
    };
  },
  (subscription: any) => {
    return from(subscription.resource.getData());
  }
);
```

## Best Practice per using()

### 1. Assicura il rilascio delle risorse

```typescript
// ✅ Buon esempio: pattern try-finally
using(
  () => {
    const resource = createResource();
    return {
      unsubscribe: () => {
        try {
          resource.close();
        } catch (error) {
          console.error('Errore rilascio risorsa:', error);
        }
      }
    };
  },
  () => interval(1000)
);
```

### 2. Logging creazione risorse

```typescript
// ✅ Buon esempio: log ciclo di vita risorse
using(
  () => {
    const resourceId = Math.random();
    console.log(`[${resourceId}] Risorsa creata`);

    return {
      unsubscribe: () => {
        console.log(`[${resourceId}] Risorsa rilasciata`);
      }
    };
  },
  () => interval(1000)
);
```

### 3. Gestione risorse type-safe

```typescript
// ✅ Buon esempio: utilizza tipi TypeScript
interface ManagedResource {
  id: string;
  close: () => void;
}

function createManagedStream(resource: ManagedResource) {
  return using(
    () => {
      console.log('Risorsa avviata:', resource.id);
      return {
        unsubscribe: () => {
          resource.close();
          console.log('Risorsa terminata:', resource.id);
        }
      };
    },
    () => interval(1000)
  );
}
```

## Confronto con Gestione Manuale

### Gestione manuale delle risorse (❌ non raccomandato)

```typescript
// ❌ Cattivo esempio: gestione manuale (rischio di dimenticare il rilascio)
const ws = new WebSocket('wss://example.com');
const subscription = interval(1000).subscribe(() => {
  ws.send('ping');
});

// Si può dimenticare di rilasciare
// subscription.unsubscribe();
// ws.close();
```

### Gestione risorse con using() (✅ raccomandato)

```typescript
// ✅ Buon esempio: gestione automatica con using()
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
  // Elaborazione usando WebSocket
});

// WebSocket viene chiuso automaticamente con unsubscribe()
subscription.unsubscribe();
```

## Riepilogo

`using()` è una Funzione di Creazione che gestisce automaticamente le risorse secondo il ciclo di vita dell'Observable.

**Caratteristiche Principali:**
- Crea una risorsa all'inizio della subscription
- Rilascio automatico alla fine della subscription (complete o unsubscribe)
- Previene perdite di memoria
- Rilascio affidabile delle risorse anche su errore

**Casi d'uso:**
- Connessioni di rete come WebSocket, EventSource
- File handle, connessioni database
- Pulizia automatica di timer e interval
- Disattivazione automatica degli event listener

**Note:**
- `resourceFactory` deve essere una funzione sincrona
- Implementa sempre il metodo `unsubscribe`
- Assicura una corretta gestione degli errori

**Uso raccomandato:**
- Evita di dimenticare il rilascio delle risorse
- Logga il ciclo di vita
- Utilizza i tipi TypeScript per gestione type-safe

## Pagine Correlate

- [scheduled()](/it/guide/creation-functions/control/scheduled) - Genera Observable con scheduler specificato
- [Funzioni di Creazione di Controllo](/it/guide/creation-functions/control/) - Confronto tra scheduled() e using()
- [finalize()](/it/guide/error-handling/finalize) - Operatore per aggiungere elaborazione alla fine della subscription

## Riferimenti

- [Documentazione Ufficiale RxJS - using](https://rxjs.dev/api/index/function/using)
- [Documentazione Ufficiale RxJS - Subscription](https://rxjs.dev/guide/subscription)
