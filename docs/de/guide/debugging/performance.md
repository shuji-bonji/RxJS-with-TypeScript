---
description: "Performance-Debugging-Techniken für RxJS-Anwendungen: Verfolgung der Subscription-Anzahl, Erkennung unnötiger Neuberechnungen, Überwachung der Speichernutzung, Debug-Einstellungen in der Entwicklungsumgebung, Erstellung typsicherer Performance-Mess-Operatoren. Praktische Techniken zur Vermeidung von Produktionsproblemen."
---

# Performance-Debugging und Best Practices

Methoden zur Optimierung der Performance von RxJS-Anwendungen und zum Aufbau einer effizienten Debug-Umgebung.

## Überprüfung der Subscription-Anzahl

Überprüfen Sie, ob unbeabsichtigt mehrere Subscriptions erstellt werden.

```ts
import { Observable, defer } from 'rxjs';
import { finalize } from 'rxjs';

let globalSubscriptionId = 0;
let activeSubscriptions = 0;

/**
 * Benutzerdefinierter Operator zur Verfolgung der Subscription-Anzahl
 */
function tracked<T>(label: string) {
  return (source: Observable<T>) =>
    defer(() => {
      const id = ++globalSubscriptionId;
      activeSubscriptions++;
      console.log(`➕ Subscription gestartet [${label}] #${id} (Aktiv: ${activeSubscriptions})`);

      return source.pipe(
        finalize(() => {
          activeSubscriptions--;
          console.log(`➖ Subscription beendet [${label}] #${id} (Aktiv: ${activeSubscriptions})`);
        })
      );
    });
}

// Verwendungsbeispiel
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

// Ausgabe:
// ➕ Subscription gestartet [Test Stream] #1 (Aktiv: 1)
// ➕ Subscription gestartet [Test Stream] #2 (Aktiv: 2)
// ➖ Subscription beendet [Test Stream] #1 (Aktiv: 1)
// ➖ Subscription beendet [Test Stream] #2 (Aktiv: 0)
```

Diese Implementierung bietet:
- ✅ Generierung einer neuen ID bei jeder Subscription mit `defer`
- ✅ Zuverlässige Ausführung der Verarbeitung beim Unsubscribe mit `finalize`
- ✅ Echtzeit-Verfolgung der aktiven Subscription-Anzahl
- ✅ Typsicher und funktioniert auch mit RxJS v8

## Erkennung unnötiger Neuberechnungen

Überprüfen Sie, ob derselbe Wert mehrfach berechnet wird.

```ts
import { of } from 'rxjs';
import { map, tap, shareReplay } from 'rxjs';

let computeCount = 0;

function expensiveComputation(value: number): number {
  computeCount++;
  console.log(`💰 Berechnung ausgeführt (${computeCount}. Mal):`, value);
  // Simulation einer schweren Berechnung
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result += Math.sin(i);
  }
  return result;
}

// ❌ Ohne shareReplay → Berechnung für jede Subscription
console.log('=== Ohne shareReplay ===');
computeCount = 0;
const withoutShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x))
);

withoutShare$.subscribe(v => console.log('Subscription 1:', v));
withoutShare$.subscribe(v => console.log('Subscription 2:', v));
// Ausgabe: Berechnung wird 6-mal ausgeführt (3 Werte × 2 Subscriptions)

// ✅ Mit shareReplay → Berechnungsergebnisse werden geteilt
console.log('\n=== Mit shareReplay ===');
computeCount = 0;
const withShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x)),
  shareReplay(3)
);

withShare$.subscribe(v => console.log('Subscription 1:', v));
withShare$.subscribe(v => console.log('Subscription 2:', v));
// Ausgabe: Berechnung wird nur 3-mal ausgeführt
```

## Überwachung der Speichernutzung

Methode zur Erkennung von Memory Leaks.

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MemoryMonitor {
  private intervals: ReturnType<typeof setInterval>[] = [];

  start(intervalMs: number = 5000) {
    const id = setInterval(() => {
      if (typeof performance !== 'undefined' && (performance as any).memory) {
        const memory = (performance as any).memory;
        console.log('📊 Speichernutzung:', {
          Verwendet: `${(memory.usedJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          Gesamt: `${(memory.totalJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          Limit: `${(memory.jsHeapSizeLimit / 1024 / 1024).toFixed(2)} MB`
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

// Verwendungsbeispiel
const monitor = new MemoryMonitor();
monitor.start(5000); // Speichernutzung alle 5 Sekunden anzeigen

// Test für Memory Leak
const leakyStreams: any[] = [];

for (let i = 0; i < 100; i++) {
  // ❌ Streams, die nicht unsubscribed werden
  const sub = interval(100).subscribe();
  leakyStreams.push(sub);
}

// Unsubscribe nach 10 Sekunden
setTimeout(() => {
  console.log('Unsubscribe gestartet');
  leakyStreams.forEach(sub => sub.unsubscribe());
  console.log('Unsubscribe abgeschlossen');

  // Überwachung nach weiteren 10 Sekunden stoppen
  setTimeout(() => {
    monitor.stop();
  }, 10000);
}, 10000);
```

## Best Practices

### Aufbau einer Debug-Umgebung

Methode zur Aktivierung von Debug-Logs nur in der Entwicklungsumgebung.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Debug-Modus-Erkennung (anpassen je nach Build-Tool)
const IS_DEVELOPMENT =
  // Bei Vite: import.meta.env.DEV
  // Bei webpack: process.env.NODE_ENV === 'development'
  // Manuelle Einstellung: Globale Variable definieren
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

// Verwendungsbeispiel
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    devLog('Input'),
    map(x => x * 2),
    devLog('Output')
  )
  .subscribe();
// In der Produktionsumgebung werden keine Logs ausgegeben
```

### Typsicheres Debugging

Debugging-Methode unter Nutzung des TypeScript-Typsystems.

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

// Verwendungsbeispiel
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

// Ausgabe:
// [User Stream] Bob (bob@example.com)
// [User Stream] Charlie (charlie@example.com)
```

### Einrichtung von Error Boundaries

Fehler angemessen isolieren, um das Debugging zu erleichtern.

```ts
import { Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

function errorBoundary<T>(label: string) {
  return (source: Observable<T>) =>
    source.pipe(
      catchError(error => {
        console.error(`🔴 [${label}] Fehler abgefangen:`, {
          message: error.message,
          stack: error.stack,
          timestamp: new Date().toISOString()
        });

        // Fehler erneut werfen oder Fallback-Wert zurückgeben
        throw error;
      })
    );
}

// Verwendungsbeispiel
import { throwError } from 'rxjs';
import { mergeMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    errorBoundary('Hauptverarbeitung'),
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Fehler bei Wert 2'));
      }
      return of(value);
    }),
    errorBoundary('Asynchrone Verarbeitung')
  )
  .subscribe({
    next: value => console.log('Erfolg:', value),
    error: error => console.log('Endgültiger Fehler:', error.message)
  });
```

## Zusammenfassung

Performance-Debugging und Best Practices

### Performance-Überwachung
- ✅ **Verfolgung der Subscription-Anzahl** - Subscription-Verwaltung mit defer und finalize
- ✅ **Erkennung von Neuberechnungen** - Vermeidung unnötiger Berechnungen mit shareReplay
- ✅ **Speicherüberwachung** - Verfolgung der Speichernutzung mit Performance API

### Optimierung der Entwicklungsumgebung
- ✅ **Umgebungsspezifische Einstellungen** - Debug-Logs nur in der Entwicklungsumgebung aktivieren
- ✅ **Typsicheres Debugging** - Nutzung des TypeScript-Typsystems
- ✅ **Error Boundaries** - Angemessene Isolierung von Fehlern zum Debuggen

Durch die Kombination dieser Techniken können Sie die Performance von RxJS-Anwendungen optimieren und eine effiziente Debug-Umgebung aufbauen.

## Verwandte Seiten

- [Grundlegende Debug-Strategien](/de/guide/debugging/) - Verwendung von tap-Operator und Entwicklerwerkzeugen
- [Häufige Debugging-Szenarien](/de/guide/debugging/common-scenarios) - Problemspezifisches Troubleshooting
- [Benutzerdefinierte Debug-Tools](/de/guide/debugging/custom-tools) - Benannte Streams, Debug-Operatoren
- [Operatoren - shareReplay](/de/guide/operators/multicasting/shareReplay) - Vermeidung unnötiger Neuberechnungen
