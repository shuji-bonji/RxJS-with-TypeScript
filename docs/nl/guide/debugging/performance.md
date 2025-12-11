---
description: "Uitleg over prestatiede bugtechnieken voor RxJS-applicaties. Praktische technieken om geen problemen te veroorzaken in de productieomgeving, zoals het volgen van subscriptions, detecteren van onnodige herberekeningen, monitoren van geheugengebruik, debug-instellingen voor ontwikkelomgeving en type-veilige prestatiemeting-operators maken."
---

# Prestatiedebuggen en best practices

Uitleg over technieken voor het optimaliseren van prestaties van RxJS-applicaties en het opzetten van een efficiënte debugomgeving.

## Controleren van aantal subscriptions

Controleer of er niet onbedoeld meerdere subscriptions worden gemaakt.

```ts
import { Observable, defer } from 'rxjs';
import { finalize } from 'rxjs';

let globalSubscriptionId = 0;
let activeSubscriptions = 0;

/**
 * Aangepaste operator om subscriptions te volgen
 */
function tracked<T>(label: string) {
  return (source: Observable<T>) =>
    defer(() => {
      const id = ++globalSubscriptionId;
      activeSubscriptions++;
      console.log(`➕ Subscription gestart [${label}] #${id} (Actief: ${activeSubscriptions})`);

      return source.pipe(
        finalize(() => {
          activeSubscriptions--;
          console.log(`➖ Subscription beëindigd [${label}] #${id} (Actief: ${activeSubscriptions})`);
        })
      );
    });
}

// Gebruiksvoorbeeld
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
// ➕ Subscription gestart [Test Stream] #1 (Actief: 1)
// ➕ Subscription gestart [Test Stream] #2 (Actief: 2)
// ➖ Subscription beëindigd [Test Stream] #1 (Actief: 1)
// ➖ Subscription beëindigd [Test Stream] #2 (Actief: 0)
```

Deze implementatie:
- ✅ Genereert telkens een nieuw ID bij subscription met `defer`
- ✅ Voert verwerking bij unsubscribe betrouwbaar uit met `finalize`
- ✅ Volgt aantal actieve subscriptions in realtime
- ✅ Type-veilig en werkt ook met RxJS v8

## Detecteren van onnodige herberekeningen

Controleer of dezelfde waarden niet meerdere keren worden berekend.

```ts
import { of } from 'rxjs';
import { map, tap, shareReplay } from 'rxjs';

let computeCount = 0;

function expensiveComputation(value: number): number {
  computeCount++;
  console.log(`💰 Berekening uitgevoerd (${computeCount}e keer):`, value);
  // Simuleer zware berekening
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result += Math.sin(i);
  }
  return result;
}

// ❌ Zonder shareReplay → berekend per subscription
console.log('=== Zonder shareReplay ===');
computeCount = 0;
const withoutShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x))
);

withoutShare$.subscribe(v => console.log('Subscription 1:', v));
withoutShare$.subscribe(v => console.log('Subscription 2:', v));
// Output: Berekening 6 keer uitgevoerd (3 waarden × 2 subscriptions)

// ✅ Met shareReplay → berekeningsresultaat wordt gedeeld
console.log('\n=== Met shareReplay ===');
computeCount = 0;
const withShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x)),
  shareReplay(3)
);

withShare$.subscribe(v => console.log('Subscription 1:', v));
withShare$.subscribe(v => console.log('Subscription 2:', v));
// Output: Berekening slechts 3 keer uitgevoerd
```

## Monitoren van geheugengebruik

Methode voor het monitoren om geheugenlekken te detecteren.

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MemoryMonitor {
  private intervals: ReturnType<typeof setInterval>[] = [];

  start(intervalMs: number = 5000) {
    const id = setInterval(() => {
      if (typeof performance !== 'undefined' && (performance as any).memory) {
        const memory = (performance as any).memory;
        console.log('📊 Geheugengebruik:', {
          'In gebruik': `${(memory.usedJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          Totaal: `${(memory.totalJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          Limiet: `${(memory.jsHeapSizeLimit / 1024 / 1024).toFixed(2)} MB`
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

// Gebruiksvoorbeeld
const monitor = new MemoryMonitor();
monitor.start(5000); // Toon geheugengebruik elke 5 seconden

// Test voor geheugenlek
const leakyStreams: any[] = [];

for (let i = 0; i < 100; i++) {
  // ❌ Stream zonder unsubscribe
  const sub = interval(100).subscribe();
  leakyStreams.push(sub);
}

// Unsubscribe na 10 seconden
setTimeout(() => {
  console.log('Unsubscribe gestart');
  leakyStreams.forEach(sub => sub.unsubscribe());
  console.log('Unsubscribe voltooid');

  // Stop monitoring na nog eens 10 seconden
  setTimeout(() => {
    monitor.stop();
  }, 10000);
}, 10000);
```

## Best practices

### Opzetten van debugomgeving

Methode om debuglogs alleen in ontwikkelomgeving in te schakelen.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Bepaal debugmodus (pas aan afhankelijk van buildtool)
const IS_DEVELOPMENT =
  // Bij gebruik van Vite: import.meta.env.DEV
  // Bij gebruik van webpack: process.env.NODE_ENV === 'development'
  // Handmatige configuratie: definieer globale variabele
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

// Gebruiksvoorbeeld
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    devLog('Input'),
    map(x => x * 2),
    devLog('Output')
  )
  .subscribe();
// Geen logs in productieomgeving
```

### Type-veilig debuggen

Debugmethode die gebruik maakt van het TypeScript-typesysteem.

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

// Gebruiksvoorbeeld
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

### Foutgrenzen instellen

Fouten goed isoleren om debuggen gemakkelijker te maken.

```ts
import { Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

function errorBoundary<T>(label: string) {
  return (source: Observable<T>) =>
    source.pipe(
      catchError(error => {
        console.error(`🔴 [${label}] Fout gevangen:`, {
          message: error.message,
          stack: error.stack,
          timestamp: new Date().toISOString()
        });

        // Gooi fout opnieuw of retourneer fallback-waarde
        throw error;
      })
    );
}

// Gebruiksvoorbeeld
import { throwError } from 'rxjs';
import { mergeMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    errorBoundary('Hoofdverwerking'),
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Fout bij waarde 2'));
      }
      return of(value);
    }),
    errorBoundary('Asynchrone verwerking')
  )
  .subscribe({
    next: value => console.log('Geslaagd:', value),
    error: error => console.log('Definitieve fout:', error.message)
  });
```

## Samenvatting

Prestatiedebuggen en best practices

### Prestatiemonitoring
- ✅ **Subscription-tracking** - Subscription-beheer met defer en finalize
- ✅ **Detectie van herberekeningen** - Vermijd onnodige berekeningen met shareReplay
- ✅ **Geheugenmonitoring** - Volg geheugengebruik met performance API

### Optimalisatie van ontwikkelomgeving
- ✅ **Omgevingsspecifieke configuratie** - Schakel debuglogs alleen in ontwikkelomgeving in
- ✅ **Type-veilig debuggen** - Gebruik TypeScript-typesysteem
- ✅ **Foutgrenzen** - Isoleer fouten goed voor debugging

Door deze technieken te combineren, kun je de prestaties van RxJS-applicaties optimaliseren en een efficiënte debugomgeving opzetten.

## Gerelateerde pagina's

- [Basis debugstrategieën](/nl/guide/debugging/) - Gebruik van tap-operator en ontwikkelaarstools
- [Veelvoorkomende debugscenario's](/nl/guide/debugging/common-scenarios) - Probleemspecifieke troubleshooting
- [Aangepaste debugtools](/nl/guide/debugging/custom-tools) - Named streams, debug-operators
- [Operators - shareReplay](/nl/guide/operators/multicasting/shareReplay) - Vermijd onnodige herberekeningen
