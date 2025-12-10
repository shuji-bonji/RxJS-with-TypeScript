---
description: "Strumenti debug RxJS personalizzati: stream nominati, operatori configurabili, misurazione prestazioni. Implementazione type-safe TypeScript."
---

# Strumenti di Debug Personalizzati

Creando strumenti di debug personalizzati, è possibile effettuare un debug flessibile adattato ai requisiti del progetto.

## Debug di Stream Nominati

Creare un operatore personalizzato che permette di tracciare gli Observable assegnando loro un nome.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Map per gestire i nomi degli stream
const namedStreams = new Map<string, any[]>();

/**
 * Assegna un nome all'Observable per tracciarlo
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
 * Ottiene i log di uno stream nominato specifico
 */
function getStreamLogs(name: string) {
  return namedStreams.get(name) || [];
}

/**
 * Ottiene l'elenco di tutti gli stream nominati
 */
function getAllStreamNames() {
  return Array.from(namedStreams.keys());
}

/**
 * Cancella i log
 */
function clearStreamLogs(name?: string) {
  if (name) {
    namedStreams.set(name, []);
  } else {
    namedStreams.clear();
  }
}
```

### Esempio d'Uso

```ts
import { interval } from 'rxjs';
import { map, take } from 'rxjs';

// Assegnare un nome all'Observable
interval(1000)
  .pipe(
    tagStream('interval-stream'),
    map(x => x * 2),
    take(5)
  )
  .subscribe();

// Verificare i log dopo 3 secondi
setTimeout(() => {
  console.log('Tutti gli stream:', getAllStreamNames());
  console.log('Log di interval-stream:', getStreamLogs('interval-stream'));
}, 3000);

// Output:
// [interval-stream] next: 0
// [interval-stream] next: 1
// [interval-stream] next: 2
// Tutti gli stream: ['interval-stream']
// Log di interval-stream: [
//   { name: 'interval-stream', type: 'next', value: 0, timestamp: 1697280000000 },
//   { name: 'interval-stream', type: 'next', value: 1, timestamp: 1697280001000 },
//   { name: 'interval-stream', type: 'next', value: 2, timestamp: 1697280002000 }
// ]
```

### Tracciamento di Multipli Stream

```ts
import { interval, fromEvent } from 'rxjs';
import { map, take } from 'rxjs';

// Assegnare nomi a multipli stream
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

// Verificare tutti gli stream
console.log('Stream in tracciamento:', getAllStreamNames());
// Output: ['timer-stream', 'click-stream']
```

> [!NOTE]
> **Riguardo a rxjs-spy**
>
> `rxjs-spy` era una libreria utile per il debug degli Observable, ma attualmente non è più mantenuta e presenta problemi di compatibilità con le ultime versioni di RxJS.
>
> Si consiglia di utilizzare operatori di debug personalizzati come quelli sopra descritti. Sono più flessibili e possono essere personalizzati secondo i requisiti del progetto.

## Creazione di Operatori di Debug Personalizzati

Creando operatori personalizzati per il debug, è possibile effettuare un debug più flessibile.

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
 * Operatore di debug personalizzato
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

// Esempio d'uso
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    debug({ label: 'Input', logTimestamp: true }),
    map(x => x * 2),
    debug({ label: 'After Map', logTimestamp: true })
  )
  .subscribe();

// Output:
// [2025-10-14T12:00:00.000Z] [Input] next: 1
// [2025-10-14T12:00:00.001Z] [After Map] next: 2
// [2025-10-14T12:00:00.001Z] [Input] next: 2
// [2025-10-14T12:00:00.002Z] [After Map] next: 4
// [2025-10-14T12:00:00.002Z] [Input] next: 3
// [2025-10-14T12:00:00.003Z] [After Map] next: 6
// [2025-10-14T12:00:00.003Z] [Input] complete
// [2025-10-14T12:00:00.004Z] [After Map] complete
```

## Operatore di Debug per Misurazione delle Prestazioni

```ts
import { tap } from 'rxjs';

function measure<T>(label: string) {
  let startTime: number;
  let count = 0;

  return tap<T>({
    subscribe: () => {
      startTime = performance.now();
      console.log(`⏱️ [${label}] Inizio`);
    },
    next: value => {
      count++;
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Valore #${count} (${elapsed.toFixed(2)}ms):`, value);
    },
    complete: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Completato (totale: ${elapsed.toFixed(2)}ms, ${count} valori)`);
    },
    error: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Errore (${elapsed.toFixed(2)}ms)`);
    }
  });
}

// Esempio d'uso
import { interval } from 'rxjs';
import { take, delay } from 'rxjs';

interval(100)
  .pipe(
    take(5),
    measure('Interval Stream'),
    delay(50)
  )
  .subscribe();

// Output:
// ⏱️ [Interval Stream] Inizio
// ⏱️ [Interval Stream] Valore #1 (150.23ms): 0
// ⏱️ [Interval Stream] Valore #2 (250.45ms): 1
// ⏱️ [Interval Stream] Valore #3 (350.67ms): 2
// ⏱️ [Interval Stream] Valore #4 (450.89ms): 3
// ⏱️ [Interval Stream] Valore #5 (551.12ms): 4
// ⏱️ [Interval Stream] Completato (totale: 551.12ms, 5 valori)
```

## Riepilogo

Con la creazione di strumenti di debug personalizzati

- ✅ **Stream nominati** - Identificare e tracciare multipli stream tramite nome
- ✅ **Configurazione flessibile** - Operatori di debug adattati ai requisiti del progetto
- ✅ **Misurazione prestazioni** - Registrazione automatica di tempo di esecuzione e numero di valori
- ✅ **Gestione log** - Registrazione e recupero di log con timestamp

Si consiglia di abilitare questi strumenti solo nell'ambiente di sviluppo e disabilitarli in produzione.

## Pagine Correlate

- [Strategie di Base per il Debug](/it/guide/debugging/) - Come usare operatore tap e strumenti per sviluppatori
- [Scenari di Debug Comuni](/it/guide/debugging/common-scenarios) - Troubleshooting per problema
- [Debug delle Prestazioni](/it/guide/debugging/performance) - Monitoraggio subscription, verifica uso memoria
