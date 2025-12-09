---
description: "Erstellung benutzerdefinierter RxJS-Debug-Tools: Tracking-Operatoren für benannte Streams, konfigurierbare Debug-Operatoren, Performance-Mess-Operatoren, Error-Boundary-Operatoren. Implementierung praktischer Debug-Tools mit TypeScript für typsicheres Debugging."
---

# Benutzerdefinierte Debug-Tools

Die Erstellung eigener Debug-Tools ermöglicht flexibles Debugging entsprechend den Projektanforderungen.

## Debugging benannter Streams

Erstellen Sie einen benutzerdefinierten Operator, der Observables benennt und verfolgt.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Map zur Verwaltung von Stream-Namen
const namedStreams = new Map<string, any[]>();

/**
 * Observable benennen und verfolgen
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
 * Logs eines bestimmten benannten Streams abrufen
 */
function getStreamLogs(name: string) {
  return namedStreams.get(name) || [];
}

/**
 * Liste aller benannten Streams abrufen
 */
function getAllStreamNames() {
  return Array.from(namedStreams.keys());
}

/**
 * Logs löschen
 */
function clearStreamLogs(name?: string) {
  if (name) {
    namedStreams.set(name, []);
  } else {
    namedStreams.clear();
  }
}
```

### Verwendungsbeispiel

```ts
import { interval } from 'rxjs';
import { map, take } from 'rxjs';

// Observable benennen
interval(1000)
  .pipe(
    tagStream('interval-stream'),
    map(x => x * 2),
    take(5)
  )
  .subscribe();

// Logs nach 3 Sekunden überprüfen
setTimeout(() => {
  console.log('Alle Streams:', getAllStreamNames());
  console.log('Logs von interval-stream:', getStreamLogs('interval-stream'));
}, 3000);

// Ausgabe:
// [interval-stream] next: 0
// [interval-stream] next: 1
// [interval-stream] next: 2
// Alle Streams: ['interval-stream']
// Logs von interval-stream: [
//   { name: 'interval-stream', type: 'next', value: 0, timestamp: 1697280000000 },
//   { name: 'interval-stream', type: 'next', value: 1, timestamp: 1697280001000 },
//   { name: 'interval-stream', type: 'next', value: 2, timestamp: 1697280002000 }
// ]
```

### Mehrere Streams verfolgen

```ts
import { interval, fromEvent } from 'rxjs';
import { map, take } from 'rxjs';

// Mehrere Streams benennen
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

// Alle Streams überprüfen
console.log('Verfolgte Streams:', getAllStreamNames());
// Ausgabe: ['timer-stream', 'click-stream']
```

> [!NOTE]
> **Über rxjs-spy**
>
> `rxjs-spy` war eine nützliche Bibliothek für das Debugging von Observables, wird aber derzeit nicht mehr gewartet und hat Kompatibilitätsprobleme mit aktuellen RxJS-Versionen.
>
> Stattdessen wird empfohlen, benutzerdefinierte Debug-Operatoren wie oben beschrieben zu verwenden. Diese sind flexibler und können an Projektanforderungen angepasst werden.

## Erstellung benutzerdefinierter Debug-Operatoren

Die Erstellung eigener Debug-Operatoren ermöglicht flexibleres Debugging.

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
 * Benutzerdefinierter Debug-Operator
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

// Verwendungsbeispiel
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    debug({ label: 'Input', logTimestamp: true }),
    map(x => x * 2),
    debug({ label: 'After Map', logTimestamp: true })
  )
  .subscribe();

// Ausgabe:
// [2025-10-14T12:00:00.000Z] [Input] next: 1
// [2025-10-14T12:00:00.001Z] [After Map] next: 2
// [2025-10-14T12:00:00.001Z] [Input] next: 2
// [2025-10-14T12:00:00.002Z] [After Map] next: 4
// [2025-10-14T12:00:00.002Z] [Input] next: 3
// [2025-10-14T12:00:00.003Z] [After Map] next: 6
// [2025-10-14T12:00:00.003Z] [Input] complete
// [2025-10-14T12:00:00.004Z] [After Map] complete
```

## Debug-Operator für Leistungsmessung

```ts
import { tap } from 'rxjs';

function measure<T>(label: string) {
  let startTime: number;
  let count = 0;

  return tap<T>({
    subscribe: () => {
      startTime = performance.now();
      console.log(`⏱️ [${label}] Start`);
    },
    next: value => {
      count++;
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Wert #${count} (${elapsed.toFixed(2)}ms):`, value);
    },
    complete: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Abschluss (Gesamt: ${elapsed.toFixed(2)}ms, ${count} Werte)`);
    },
    error: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Fehler (${elapsed.toFixed(2)}ms)`);
    }
  });
}

// Verwendungsbeispiel
import { interval } from 'rxjs';
import { take, delay } from 'rxjs';

interval(100)
  .pipe(
    take(5),
    measure('Interval Stream'),
    delay(50)
  )
  .subscribe();

// Ausgabe:
// ⏱️ [Interval Stream] Start
// ⏱️ [Interval Stream] Wert #1 (150.23ms): 0
// ⏱️ [Interval Stream] Wert #2 (250.45ms): 1
// ⏱️ [Interval Stream] Wert #3 (350.67ms): 2
// ⏱️ [Interval Stream] Wert #4 (450.89ms): 3
// ⏱️ [Interval Stream] Wert #5 (551.12ms): 4
// ⏱️ [Interval Stream] Abschluss (Gesamt: 551.12ms, 5 Werte)
```

## Zusammenfassung

Durch die Erstellung benutzerdefinierter Debug-Tools

- ✅ **Benannte Streams** - Identifikation und Verfolgung mehrerer Streams anhand ihrer Namen
- ✅ **Flexible Konfiguration** - Debug-Operatoren entsprechend den Projektanforderungen
- ✅ **Leistungsmessung** - Automatische Aufzeichnung von Ausführungszeit und Anzahl der Werte
- ✅ **Log-Verwaltung** - Aufzeichnung und Abruf von Logs mit Zeitstempel

Es wird empfohlen, diese Tools nur in der Entwicklungsumgebung zu aktivieren und in der Produktionsumgebung zu deaktivieren.

## Verwandte Seiten

- [Grundlegende Debug-Strategien](/de/guide/debugging/) - Verwendung von tap-Operator und Entwicklerwerkzeugen
- [Häufige Debugging-Szenarien](/de/guide/debugging/common-scenarios) - Problemspezifisches Troubleshooting
- [Performance-Debugging](/de/guide/debugging/performance) - Überwachung der Subscription-Anzahl, Speichernutzung
