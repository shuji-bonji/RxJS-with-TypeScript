---
description: "Debug prestazioni RxJS: tracciamento subscription, rilevamento ricalcoli, monitoraggio memoria. Best practice per ambiente sviluppo ottimizzato."
---

# Debug delle Prestazioni e Best Practice

Spiegazione delle tecniche per ottimizzare le prestazioni delle applicazioni RxJS e costruire un ambiente di debug efficiente.

## Verifica del Numero di Subscription

Verificare che non vengano create involontariamente subscription multiple.

```ts
import { Observable, defer } from 'rxjs';
import { finalize } from 'rxjs';

let globalSubscriptionId = 0;
let activeSubscriptions = 0;

/**
 * Operatore personalizzato per tracciare il numero di subscription
 */
function tracked<T>(label: string) {
  return (source: Observable<T>) =>
    defer(() => {
      const id = ++globalSubscriptionId;
      activeSubscriptions++;
      console.log(`➕ Subscription iniziata [${label}] #${id} (attive: ${activeSubscriptions})`);

      return source.pipe(
        finalize(() => {
          activeSubscriptions--;
          console.log(`➖ Subscription terminata [${label}] #${id} (attive: ${activeSubscriptions})`);
        })
      );
    });
}

// Esempio d'uso
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

// Output:
// ➕ Subscription iniziata [Test Stream] #1 (attive: 1)
// ➕ Subscription iniziata [Test Stream] #2 (attive: 2)
// ➖ Subscription terminata [Test Stream] #1 (attive: 1)
// ➖ Subscription terminata [Test Stream] #2 (attive: 0)
```

In questa implementazione:
- ✅ `defer` genera un nuovo ID ad ogni subscription
- ✅ `finalize` esegue sicuramente l'elaborazione alla cancellazione della subscription
- ✅ Traccia in tempo reale il numero di subscription attive
- ✅ Type-safe e funziona anche con RxJS v8

## Rilevamento di Ricalcoli Non Necessari

Verificare che lo stesso valore non venga calcolato più volte.

```ts
import { of } from 'rxjs';
import { map, tap, shareReplay } from 'rxjs';

let computeCount = 0;

function expensiveComputation(value: number): number {
  computeCount++;
  console.log(`💰 Esecuzione calcolo (${computeCount}° volta):`, value);
  // Simulare un calcolo pesante
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result += Math.sin(i);
  }
  return result;
}

// ❌ Senza shareReplay → calcolato per ogni subscription
console.log('=== Senza shareReplay ===');
computeCount = 0;
const withoutShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x))
);

withoutShare$.subscribe(v => console.log('Subscription 1:', v));
withoutShare$.subscribe(v => console.log('Subscription 2:', v));
// Output: il calcolo viene eseguito 6 volte (3 valori × 2 subscription)

// ✅ Con shareReplay → il risultato del calcolo viene condiviso
console.log('\n=== Con shareReplay ===');
computeCount = 0;
const withShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x)),
  shareReplay(3)
);

withShare$.subscribe(v => console.log('Subscription 1:', v));
withShare$.subscribe(v => console.log('Subscription 2:', v));
// Output: il calcolo viene eseguito solo 3 volte
```

## Monitoraggio dell'Uso della Memoria

Metodo di monitoraggio per rilevare memory leak.

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MemoryMonitor {
  private intervals: ReturnType<typeof setInterval>[] = [];

  start(intervalMs: number = 5000) {
    const id = setInterval(() => {
      if (typeof performance !== 'undefined' && (performance as any).memory) {
        const memory = (performance as any).memory;
        console.log('📊 Uso memoria:', {
          inUso: `${(memory.usedJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
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

// Esempio d'uso
const monitor = new MemoryMonitor();
monitor.start(5000); // Visualizza uso memoria ogni 5 secondi

// Test memory leak
const leakyStreams: any[] = [];

for (let i = 0; i < 100; i++) {
  // ❌ Stream non cancellati
  const sub = interval(100).subscribe();
  leakyStreams.push(sub);
}

// Cancellazione subscription dopo 10 secondi
setTimeout(() => {
  console.log('Inizio cancellazione subscription');
  leakyStreams.forEach(sub => sub.unsubscribe());
  console.log('Cancellazione subscription completata');

  // Arresto monitoraggio dopo altri 10 secondi
  setTimeout(() => {
    monitor.stop();
  }, 10000);
}, 10000);
```

## Best Practice

### Configurazione Ambiente di Debug

Metodo per abilitare i log di debug solo nell'ambiente di sviluppo.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Determinazione modalità debug (regolare in base al build tool)
const IS_DEVELOPMENT =
  // Con Vite: import.meta.env.DEV
  // Con webpack: process.env.NODE_ENV === 'development'
  // Configurazione manuale: definire variabile globale
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

// Esempio d'uso
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    devLog('Input'),
    map(x => x * 2),
    devLog('Output')
  )
  .subscribe();
// I log non vengono emessi in ambiente di produzione
```

### Debug Type-Safe

Metodo di debug che sfrutta il sistema dei tipi di TypeScript.

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

// Esempio d'uso
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

// Output:
// [User Stream] Bob (bob@example.com)
// [User Stream] Charlie (charlie@example.com)
```

### Impostazione Error Boundary

Isolare appropriatamente gli errori per facilitare il debug.

```ts
import { Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

function errorBoundary<T>(label: string) {
  return (source: Observable<T>) =>
    source.pipe(
      catchError(error => {
        console.error(`🔴 [${label}] Errore catturato:`, {
          message: error.message,
          stack: error.stack,
          timestamp: new Date().toISOString()
        });

        // Rilanciare l'errore o restituire un valore di fallback
        throw error;
      })
    );
}

// Esempio d'uso
import { throwError } from 'rxjs';
import { mergeMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    errorBoundary('Elaborazione principale'),
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Errore al valore 2'));
      }
      return of(value);
    }),
    errorBoundary('Elaborazione asincrona')
  )
  .subscribe({
    next: value => console.log('Successo:', value),
    error: error => console.log('Errore finale:', error.message)
  });
```

## Riepilogo

Debug delle prestazioni e best practice

### Monitoraggio Prestazioni
- ✅ **Tracciamento subscription** - Gestione subscription con defer e finalize
- ✅ **Rilevamento ricalcoli** - Evitare calcoli non necessari con shareReplay
- ✅ **Monitoraggio memoria** - Tracciare uso memoria con performance API

### Ottimizzazione Ambiente di Sviluppo
- ✅ **Configurazione per ambiente** - Abilitare log di debug solo in ambiente di sviluppo
- ✅ **Debug type-safe** - Sfruttare il sistema dei tipi di TypeScript
- ✅ **Error boundary** - Isolare appropriatamente gli errori per il debug

Combinando queste tecniche, è possibile ottimizzare le prestazioni delle applicazioni RxJS e costruire un ambiente di debug efficiente.

## Pagine Correlate

- [Strategie di Base per il Debug](/it/guide/debugging/) - Come usare operatore tap e strumenti per sviluppatori
- [Scenari di Debug Comuni](/it/guide/debugging/common-scenarios) - Troubleshooting per problema
- [Strumenti di Debug Personalizzati](/it/guide/debugging/custom-tools) - Stream nominati, operatori di debug
- [Operatori - shareReplay](/it/guide/operators/multicasting/shareReplay) - Evitare ricalcoli non necessari
