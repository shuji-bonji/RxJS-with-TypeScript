---
description: Asynchrone Verarbeitung in RxJS mit observeOn und subscribeOn steuern. Praktische TypeScript-Beispiele für Timing-Kontrolle und UI-Blockierung vermeiden.
---
# Steuerung der asynchronen Verarbeitung

Scheduler in RxJS sind ein wichtiger Mechanismus zur Steuerung des Timings und Ausführungskontexts asynchroner Verarbeitung. Dieses Kapitel erklärt, wie man Scheduler zur Steuerung asynchroner Verarbeitung verwendet.

## Die Rolle von Schedulern

Scheduler erfüllen drei wichtige Rollen:

|Rolle|Beschreibung|
|---|---|
|Steuerung des Ausführungs-Timings|Bestimmt, wann Tasks ausgeführt werden|
|Verwaltung des Ausführungskontexts|Bestimmt, in welchem Thread oder welcher Ausführungsumgebung Tasks ausgeführt werden|
|Task-Priorisierung|Verwaltet die Ausführungsreihenfolge mehrerer Tasks|

## Verständnis synchroner und asynchroner Verarbeitung

### Standardverhalten (synchrone Ausführung)

RxJS-Operatoren werden standardmäßig so synchron wie möglich ausgeführt.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

console.log('Ausführung Start');

of(1, 2, 3)
  .pipe(
    map((x) => {
      console.log(`map: ${x}`);
      return x * 2;
    })
  )
  .subscribe((x) => console.log(`subscribe: ${x}`));

console.log('Ausführung Ende');

// Ausgabe:
// Ausführung Start
// map: 1
// subscribe: 2
// map: 2
// subscribe: 4
// map: 3
// subscribe: 6
// Ausführung Ende
```

### Asynchronisierung durch Scheduler

Durch Verwendung von Schedulern kann die Verarbeitung asynchronisiert werden.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Ausführung Start');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(x => console.log(`subscribe: ${x}`));

console.log('Ausführung Ende');

// Ausgabe:
// Ausführung Start
// Ausführung Ende
// subscribe: 1
// subscribe: 2
// subscribe: 3
```

## Operatoren, die Scheduler verwenden

### observeOn Operator

`observeOn` ändert den Ausführungskontext eines Streams. Es gibt Werte mit dem angegebenen Scheduler aus.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { take, observeOn } from 'rxjs';

// Beispiel für Animations-Anwendung
interval(16)
  .pipe(
    take(10),
    observeOn(animationFrameScheduler)
  )
  .subscribe(() => {
    // Wird synchron mit Animation Frames ausgeführt
    updateAnimation();
  });

function updateAnimation() {
  // Animations-Update-Verarbeitung
}
```

> [!TIP]
> Für detaillierte Erklärungen, praktische Beispiele und Hinweise zum `observeOn`-Operator siehe die Seite [observeOn](../operators/utility/observeOn.md).

### subscribeOn Operator

`subscribeOn` steuert den Startzeitpunkt einer Stream-Subscription.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, tap } from 'rxjs';

console.log('Vor Subscription-Start');

of('Task-Ausführung')
  .pipe(
    tap(() => console.log('Task-Start')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(value => console.log(value));

console.log('Nach Subscription-Start');

// Ausgabe:
// Vor Subscription-Start
// Nach Subscription-Start
// Task-Start
// Task-Ausführung
```

> [!TIP]
> Für detaillierte Erklärungen zum `subscribeOn`-Operator, praktische Beispiele und Unterschiede zu `observeOn` siehe die Seite [subscribeOn](../operators/utility/subscribeOn.md).

## Praktische Beispiele für asynchrone Verarbeitung

### API-Request-Steuerung

```ts
import { from, queueScheduler } from 'rxjs';
import { mergeMap, observeOn, tap } from 'rxjs';

interface ApiRequest {
  endpoint: string;
  id: number;
}

const requests: ApiRequest[] = [
  { endpoint: '/users', id: 1 },
  { endpoint: '/posts', id: 1 },
  { endpoint: '/comments', id: 1 },
];

// Requests in Queue einreihen und nacheinander verarbeiten
from(requests)
  .pipe(
    observeOn(queueScheduler),
    tap((req) => console.log(`Zur Queue hinzugefügt: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // Simulation eines tatsächlichen API-Requests
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`${req.endpoint}/${req.id} Ergebnis`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`Abgeschlossen: ${result}`));

// Ausgabe:
// Zur Queue hinzugefügt: /users
// Zur Queue hinzugefügt: /posts
// Zur Queue hinzugefügt: /comments
// Abgeschlossen: /users/1 Ergebnis
// Abgeschlossen: /posts/1 Ergebnis
// Abgeschlossen: /comments/1 Ergebnis
```

### Vermeidung von UI-Thread-Blockierung

Bei der Verarbeitung großer Datenmengen nutzen wir Scheduler, um zu vermeiden, dass der UI-Thread blockiert wird.

```ts
import { from, asapScheduler } from 'rxjs';
import { observeOn, bufferCount } from 'rxjs';

const largeDataSet = Array.from({ length: 10000 }, (_, i) => i);

// Batch-Größe
const batchSize = 100;
// Berechnung der Gesamtanzahl von Batches
const totalBatches = Math.ceil(largeDataSet.length / batchSize);
// Batch-Zähler
let batchIndex = 0;

from(largeDataSet)
  .pipe(
    bufferCount(100), // 100 Stück zusammenfassen
    observeOn(asapScheduler) // So schnell wie möglich, aber ohne UI zu blockieren
  )
  .subscribe((batch) => {
    batchIndex++;
    processBatch(batch, batchIndex, totalBatches);
  });

function processBatch(
  batch: number[],
  batchIndex: number,
  totalBatches: number
) {
  // Batch-Datenverarbeitung
  const processed = batch.map((n) => n * 2);
  console.log(
    `Batch ${batchIndex} von insgesamt ${totalBatches} abgeschlossen: ${processed.length} Elemente verarbeitet.`
  );
}

// Ausgabe:
// Batch 1 von insgesamt 100 abgeschlossen: 100 Elemente verarbeitet.
// Batch 2 von insgesamt 100 abgeschlossen: 100 Elemente verarbeitet.
// ...
// ...
// Batch 100 von insgesamt 100 abgeschlossen: 100 Elemente verarbeitet.
```

## Performance-Optimierung und Debugging

### Tests mit Schedulern

```ts
import { TestScheduler } from 'rxjs/testing';
import { delay } from 'rxjs';
import { beforeEach, describe, expect, it } from 'vitest';

describe('Asynchrone Verarbeitungs-Tests', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Test des delay-Operators', () => {
    scheduler.run(({ cold, expectObservable }) => {
      const source = cold('a-b-c|');
      const expected =    '1000ms a-b-(c|)';

      const result = source.pipe(delay(1000, scheduler));

      expectObservable(result).toBe(expected);
    });
  });
});
```

### Debug-Logging

```ts
import { of, asyncScheduler } from 'rxjs';
import { tap, observeOn } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    tap(value => console.log(`[Vor Scheduler・synchron] Wert: ${value}`)),
    observeOn(asyncScheduler),  // asyncScheduler verwenden
    tap(value => console.log(`[Nach Scheduler・asynchron] Wert: ${value}`))
  )
  .subscribe();

console.log('Ende');

// Tatsächliche Ausgabe:
// Start
// [Vor Scheduler・synchron] Wert: 1
// [Vor Scheduler・synchron] Wert: 2
// [Vor Scheduler・synchron] Wert: 3
// Ende
// [Nach Scheduler・asynchron] Wert: 1
// [Nach Scheduler・asynchron] Wert: 2
// [Nach Scheduler・asynchron] Wert: 3
```

Bei Verwendung von `asyncScheduler` kann das erwartete asynchrone Verhalten bestätigt werden. `queueScheduler` verwendet die Microtask-Queue, sodass es während der Ausführung synchronen Codes verarbeitet wird, während `asyncScheduler` intern setTimeout verwendet und daher vollständig asynchron ausgeführt wird.

## Beispiel zur Verdeutlichung der Scheduler-Unterschiede
Dieses Beispiel zeigt die Unterschiede im Ausführungs-Timing verschiedener Scheduler.

```ts
import { of, queueScheduler, asyncScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

// Synchrone Verarbeitung
of('sync').subscribe(value => console.log(`2: ${value}`));

// queueScheduler (Microtask)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`3: ${value}`));

// asapScheduler (Microtask)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`4: ${value}`));

// asyncScheduler (Macrotask)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`5: ${value}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Ende');

// Tatsächliche Ausgabereihenfolge:
// 1: Start
// 2: sync
// 7: Ende
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```


## Best Practices

1. **Scheduler nur bei Bedarf verwenden**: Wenn das standardmäßige synchrone Verhalten ausreicht, verwenden Sie keine Scheduler unnötig

2. **Auswahl des geeigneten Schedulers**: Wählen Sie den optimalen Scheduler je nach Verwendungszweck
   - Animation: `animationFrameScheduler`
   - UI-Blockierungs-Vermeidung: `asapScheduler`
   - Queue-Verarbeitung: `queueScheduler`
   - Asynchrone Verarbeitung: `asyncScheduler`

3. **Performance-Monitoring**: Überwachen Sie stets die Auswirkungen der Scheduler-Verwendung auf die Performance

4. **Sicherstellen der Testbarkeit**: Verwenden Sie `TestScheduler` zum Schreiben von Tests für asynchrone Verarbeitung

## Häufige Fehler und Gegenmaßnahmen

### Übermäßige Asynchronisierung

```ts
// ❌ Unnötige Asynchronisierung
of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Doppelte Asynchronisierung
    filter(x => x > 3)
  )
  .subscribe();

// ✅ Nur an notwendigen Stellen asynchronisieren
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    filter(x => x > 3),
    observeOn(asyncScheduler)  // Am Ende zusammen asynchronisieren
  )
  .subscribe();
```

### Fehlerhafte Scheduler-Verwendung

```ts
// ❌ Falsche Verwendung
interval(1000)
  .pipe(
    subscribeOn(animationFrameScheduler)  // Hat keine Auswirkung auf interval
  )
  .subscribe();

// ✅ Richtige Verwendung
interval(1000, animationFrameScheduler)  // Scheduler bei Erstellung angeben
  .subscribe();
```

## Zusammenfassung

Scheduler sind ein leistungsstarkes Tool zur detaillierten Steuerung asynchroner Verarbeitung in RxJS. Bei ordnungsgemäßer Verwendung können Performance-Optimierung, Vermeidung von UI-Thread-Blockierung und erleichterte Tests erreicht werden. Übermäßige Asynchronisierung kann jedoch die Performance verschlechtern, daher ist es wichtig, sie nur bei Bedarf zu verwenden.

Im nächsten Abschnitt werden wir die spezifischen Scheduler-Typen und deren Verwendung im Detail erläutern.
