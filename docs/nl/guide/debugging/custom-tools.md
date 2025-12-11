---
description: "Uitleg over het maken van aangepaste tools voor RxJS-debugging. Introductie van praktische debugtools die type-veilig geïmplementeerd zijn in TypeScript, zoals named stream tracking-operators, configureerbare debug-operators, prestatiemeting-operators en foutgrens-operators."
---

# Aangepaste debugtools

Door eigen debugtools te maken, wordt flexibel debuggen mogelijk dat past bij de projectvereisten.

## Named stream debugging

Maak een aangepaste operator waarmee je Observable's een naam kunt geven en kunt volgen.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Map voor het beheren van streamnamen
const namedStreams = new Map<string, any[]>();

/**
 * Geef een Observable een naam en volg deze
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
 * Haal logs op voor een specifieke named stream
 */
function getStreamLogs(name: string) {
  return namedStreams.get(name) || [];
}

/**
 * Haal lijst van alle named streams op
 */
function getAllStreamNames() {
  return Array.from(namedStreams.keys());
}

/**
 * Wis logs
 */
function clearStreamLogs(name?: string) {
  if (name) {
    namedStreams.set(name, []);
  } else {
    namedStreams.clear();
  }
}
```

### Gebruiksvoorbeeld

```ts
import { interval } from 'rxjs';
import { map, take } from 'rxjs';

// Geef Observable een naam
interval(1000)
  .pipe(
    tagStream('interval-stream'),
    map(x => x * 2),
    take(5)
  )
  .subscribe();

// Controleer logs na 3 seconden
setTimeout(() => {
  console.log('Alle streams:', getAllStreamNames());
  console.log('interval-stream logs:', getStreamLogs('interval-stream'));
}, 3000);

// Output:
// [interval-stream] next: 0
// [interval-stream] next: 1
// [interval-stream] next: 2
// Alle streams: ['interval-stream']
// interval-stream logs: [
//   { name: 'interval-stream', type: 'next', value: 0, timestamp: 1697280000000 },
//   { name: 'interval-stream', type: 'next', value: 1, timestamp: 1697280001000 },
//   { name: 'interval-stream', type: 'next', value: 2, timestamp: 1697280002000 }
// ]
```

### Meerdere streams volgen

```ts
import { interval, fromEvent } from 'rxjs';
import { map, take } from 'rxjs';

// Geef meerdere streams namen
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

// Controleer alle streams
console.log('Gevolgde streams:', getAllStreamNames());
// Output: ['timer-stream', 'click-stream']
```

> [!NOTE]
> **Over rxjs-spy**
>
> `rxjs-spy` was een handige bibliotheek voor het debuggen van Observable's, maar wordt momenteel niet meer onderhouden en heeft compatibiliteitsproblemen met de nieuwste RxJS.
>
> In plaats daarvan wordt het gebruik van aangepaste debug-operators zoals hierboven aanbevolen. Deze zijn flexibeler en kunnen worden aangepast aan de projectvereisten.

## Aangepaste debug-operators maken

Door eigen debug-operators te maken, wordt flexibeler debuggen mogelijk.

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
 * Aangepaste debug-operator
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

// Gebruiksvoorbeeld
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

## Debug-operator voor prestatiemetingen

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
      console.log(`⏱️ [${label}] Waarde #${count} (${elapsed.toFixed(2)}ms):`, value);
    },
    complete: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Voltooid (totaal: ${elapsed.toFixed(2)}ms, ${count} waarden)`);
    },
    error: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Fout (${elapsed.toFixed(2)}ms)`);
    }
  });
}

// Gebruiksvoorbeeld
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
// ⏱️ [Interval Stream] Start
// ⏱️ [Interval Stream] Waarde #1 (150.23ms): 0
// ⏱️ [Interval Stream] Waarde #2 (250.45ms): 1
// ⏱️ [Interval Stream] Waarde #3 (350.67ms): 2
// ⏱️ [Interval Stream] Waarde #4 (450.89ms): 3
// ⏱️ [Interval Stream] Waarde #5 (551.12ms): 4
// ⏱️ [Interval Stream] Voltooid (totaal: 551.12ms, 5 waarden)
```

## Samenvatting

Door aangepaste debugtools te maken

- ✅ **Named streams** - Identificeer en volg meerdere streams op naam
- ✅ **Flexibele configuratie** - Debug-operators aangepast aan projectvereisten
- ✅ **Prestatiemetingen** - Automatische registratie van uitvoeringstijd en aantal waarden
- ✅ **Logbeheer** - Registratie en ophalen van logs met timestamp

Het wordt aanbevolen om deze tools alleen in de ontwikkelomgeving in te schakelen en uit te schakelen in de productieomgeving.

## Gerelateerde pagina's

- [Basis debugstrategieën](/nl/guide/debugging/) - Gebruik van tap-operator en ontwikkelaarstools
- [Veelvoorkomende debugscenario's](/nl/guide/debugging/common-scenarios) - Probleemspecifieke troubleshooting
- [Prestatiedebuggen](/nl/guide/debugging/performance) - Subscription-monitoring, geheugengebruik controleren
