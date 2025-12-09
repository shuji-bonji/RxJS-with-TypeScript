---
description: RxJS Scheduler-Typen (asyncScheduler, queueScheduler) erklärt. Unterschiede zwischen Macrotasks, Microtasks und synchroner Verarbeitung verstehen.
---

# Scheduler-Typen und ihre Verwendung

RxJS bietet mehrere Scheduler für unterschiedliche Zwecke. Jeder Scheduler hat ein spezifisches Ausführungs-Timing und spezifische Eigenschaften, und durch ihre korrekte Verwendung können Sie die Performance und das Verhalten Ihrer Anwendung optimieren.

## Scheduler-Klassifizierung

RxJS-Scheduler lassen sich grob in drei Kategorien einteilen:

1. **Macrotasks**: Werden in der nächsten Task-Queue des Event Loops ausgeführt
2. **Microtasks**: Werden unmittelbar nach Abschluss des aktuellen Tasks ausgeführt, bevor der nächste Task beginnt
3. **Synchrone Verarbeitung**: Sofortige Ausführung

Siehe auch [Grundlagen zu Tasks und Schedulern](./task-and-scheduler-basics.md) für Details.

## Wichtige Scheduler

### asyncScheduler

#### Eigenschaften
- **Interne Implementierung**: Verwendet setTimeout
- **Ausführungs-Timing**: Macrotask
- **Verwendungszweck**: Allgemeine asynchrone Verarbeitung, zeitbasierte Verarbeitung

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Asynchrone Verarbeitung')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Ende');

// Ausgabe:
// 1: Start
// 2: Ende
// 3: Asynchrone Verarbeitung
```

#### Anwendungsfall

```ts
import { asyncScheduler, map, observeOn, of } from "rxjs";

function heavyComputation(value: number): number {
  // Simulation schwerer Berechnungen
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result = Math.sin(result);
  }
  return result;
}

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(value => heavyComputation(value))
  )
  .subscribe(result => {
    console.log(`Berechnungsergebnis: ${result}`);
  });
```

### queueScheduler

#### Eigenschaften
- **Interne Implementierung**: Microtask-Queue
- **Ausführungs-Timing**: Innerhalb des aktuellen Tasks (erscheint synchron)
- **Verwendungszweck**: Task-Queuing, Optimierung rekursiver Verarbeitung

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Queue-Verarbeitung')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: Ende');

// Ausgabe:
// 1: Start
// 2: Queue-Verarbeitung
// 3: Ende
```

#### Anwendungsfall

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs';

// Optimierung rekursiver Verarbeitung
function fibonacci(n: number): Observable<number> {
  return of([0, 1]).pipe(
    observeOn(queueScheduler),
    expand(([a, b]) => of([b, a + b])),
    map(([a]) => a),
    take(n)
  );
}

fibonacci(10).subscribe(value => console.log(value));
```

### asapScheduler

#### Eigenschaften
- **Interne Implementierung**: Promise.resolve().then() oder setImmediate
- **Ausführungs-Timing**: Microtask
- **Verwendungszweck**: Wenn asynchrone Ausführung so schnell wie möglich gewünscht ist

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('ASAP-Verarbeitung')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Ende');

// Ausgabe:
// 1: Start
// 2: Ende
// 3: ASAP-Verarbeitung
```

#### Anwendungsfall

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Optimierung von Mausbewegungsereignissen
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // UI-Update-Verarbeitung
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Eigenschaften
- **Interne Implementierung**: requestAnimationFrame
- **Ausführungs-Timing**: Vor dem nächsten Bildschirm-Rendering
- **Verwendungszweck**: Animationen, 60fps-Rendering-Verarbeitung

#### Beispiel für einfache Rotationsanimation

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// HTML-Element erstellen
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Animations-Einstellungen
let rotation = 0;

// Animation bei 60fps für 2 Sekunden
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2 Sekunden = 120 Frames
    map(() => {
      rotation += 3;  // 3 Grad pro Frame drehen
      return rotation;
    })
  )
  .subscribe(angle => {
    // DOM-Element tatsächlich drehen
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Warum animationFrameScheduler notwendig ist

`animationFrameScheduler` führt Verarbeitung synchron mit dem Browser-Rendering-Zyklus aus und bietet folgende Vorteile:

1. **Flüssige Animationen**: Durch Ausführung im Browser-Rendering-Timing (normalerweise 60fps) werden flüssige Animationen ohne Ruckeln erreicht.
2. **Effiziente Ressourcennutzung**: Wenn der Browser einen Tab inaktiv macht, wird die Ausführung von requestAnimationFrame automatisch pausiert, was unnötige CPU-Nutzung verhindert.
3. **Vermeidung von Bildschirm-Flackern**: Durch sichere Fertigstellung von Berechnungen vor dem Bildschirm-Rendering wird Flackern und die Anzeige unvollständiger Frames verhindert.

Hier ist ein Vergleich zwischen `setInterval` und `animationFrameScheduler`:

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ Ineffiziente Animation mit setInterval
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // ca. 60fps

// Probleme:
// - Nicht mit Browser-Rendering-Timing synchronisiert
// - Wird auch in Hintergrund-Tabs weiter ausgeführt
// - Kann keine genauen 60fps garantieren

// ✅ Effiziente Animation mit animationFrameScheduler
interval(0, animationFrameScheduler)
  .pipe(
    map(() => {
      position += 1;
      return position;
    })
  )
  .subscribe(pos => {
    element.style.transform = `translateX(${pos}px)`;
  });

// Vorteile
// - Synchron mit Browser-Rendering-Timing
// - Automatische Pause in Hintergrund-Tabs
// - Stabile 60fps erreicht
```


#### Beispiel für Maus-Folge-Animation

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Folge-Kreis erstellen
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Mausereignisse durchlassen
document.body.appendChild(circle);

// Aktuelle Position und Zielposition
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Mausbewegungsereignisse überwachen
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove')
  .pipe(
    map(event => ({
      x: event.clientX,
      y: event.clientY
    }))
  );

// Animationsschleife
interval(0, animationFrameScheduler)
  .pipe(
    withLatestFrom(mouseMove$),
    map(([_, mousePos]) => mousePos)
  )
  .subscribe(({ x, y }) => {
    // Mausposition als Ziel setzen
    targetX = x;
    targetY = y;

    // Schrittweise von aktueller Position zu Zielposition bewegen (Easing)
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;

    // DOM-Element aktualisieren
    circle.style.left = `${currentX - 15}px`;  // Zentrierung anpassen
    circle.style.top = `${currentY - 15}px`;
  });
```

## Scheduler-Verwendungsleitfaden

### Vergleich nach Ausführungs-Timing

```ts
import { of, asyncScheduler, queueScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

// Synchrone Verarbeitung
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler (Microtask)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler (Microtask)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler (Macrotask)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(v => console.log(`5: ${v}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Ende');

// Ausführungsreihenfolge:
// 1: Start
// 2: sync
// 7: Ende
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```

### Auswahlkriterien nach Verwendungszweck

| Scheduler | Eigenschaften | Geeignete Verwendung |
|--------------|------|----------|
| asyncScheduler | Verwendet setTimeout, vollständig asynchron | Zeitintensive Verarbeitung, verzögerte Ausführung |
| queueScheduler | Synchron, aber optimiert Rekursion | Rekursive Verarbeitung, Task-Queue-Verwaltung |
| asapScheduler | Schnellstmögliche asynchrone Ausführung | Event-Handling, Verarbeitung mit schneller Reaktionszeit |
| animationFrameScheduler | Synchron mit Bildschirm-Rendering | Animationen, UI-Updates, Spieleentwicklung |

## Praktische Verwendungsbeispiele

### Verarbeitung großer Datenmengen

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
```

### WebSocket-Nachrichtenverarbeitung

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// Hinweis: Dies ist Pseudocode zur Veranschaulichung des Konzepts
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // Als String behandeln
});

socket$
  .pipe(
    // Nachrichtenverarbeitung mit schneller Reaktionszeit erforderlich
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('Nachricht empfangen:', msg);
}
```

### Fehler-Retry-Steuerung

Der `retry`-Operator kann Scheduler nutzen, um das Retry-Timing präzise zu steuern.

#### Grundlegende Retry-Steuerung

Die `delay`-Option des `retry`-Operators verwendet intern `asyncScheduler`, um Retry-Intervalle zu steuern.

```ts
import { throwError, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

// API-Aufruf-Simulation
function fetchData(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.7) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Network error'));
    })
  );
}

fetchData(1)
  .pipe(
    retry({
      count: 3,
      delay: 1000  // Mit asyncScheduler 1 Sekunde warten, dann Retry
    })
  )
  .subscribe({
    next: result => console.log('✅ Erfolg:', result),
    error: error => console.log('❌ Finaler Fehler:', error.message)
  });
```

#### Scheduler-Nutzung mit Exponential Backoff

Für erweiterte Steuerung kann `retryWhen` mit `asyncScheduler` kombiniert werden, um Exponential Backoff zu implementieren.

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, mergeMap, tap } from 'rxjs';

function fetchDataWithBackoff(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.9) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Temporary error'));
    })
  );
}

fetchDataWithBackoff(1)
  .pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;

          // Maximale Retry-Anzahl prüfen
          if (retryCount > 3) {
            console.log('❌ Maximale Retry-Anzahl erreicht');
            throw error;
          }

          // Exponential Backoff: 1s, 2s, 4s...
          const delayTime = Math.pow(2, index) * 1000;
          console.log(`🔄 Retry ${retryCount} (nach ${delayTime}ms)`);

          // timer verwendet intern asyncScheduler
          return timer(delayTime);
        })
      )
    )
  )
  .subscribe({
    next: result => console.log('✅ Erfolg:', result),
    error: error => console.log('❌ Finaler Fehler:', error.message)
  });

// Beispielausgabe:
// 🔄 Retry 1 (nach 1000ms)
// 🔄 Retry 2 (nach 2000ms)
// 🔄 Retry 3 (nach 4000ms)
// ❌ Maximale Retry-Anzahl erreicht
// ❌ Finaler Fehler: Temporary error
```

#### Explizite Angabe von asyncScheduler

Durch explizite Angabe eines bestimmten Schedulers wird flexiblere Steuerung möglich, wie z.B. der Austausch durch `TestScheduler` in Tests.

```ts
import { throwError, asyncScheduler, of } from 'rxjs';
import { retryWhen, mergeMap, delay } from 'rxjs';

function fetchDataWithScheduler(id: number, scheduler = asyncScheduler) {
  return of(id).pipe(
    mergeMap(() => throwError(() => new Error('Error'))),
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          if (index >= 2) throw error;

          // Scheduler explizit angeben
          return of(null).pipe(
            delay(1000, scheduler)
          );
        })
      )
    )
  );
}

// Produktionsumgebung: asyncScheduler verwenden
fetchDataWithScheduler(1).subscribe({
  error: err => console.log('Fehler:', err.message)
});

// Testumgebung: Austausch durch TestScheduler möglich
```

> [!TIP]
> Für detaillierte Implementierungsmuster und Debugging-Methoden für Retry-Verarbeitung siehe die Seite [retry und catchError](/de/guide/error-handling/retry-catch).
> - Detaillierte Verwendung des retry-Operators
> - Kombinationsmuster mit catchError
> - Retry-Debugging-Techniken (Verfolgung von Versuchen, Logging etc.)

## Auswirkungen auf die Performance

### Scheduler-Overhead

```ts
import { range, asyncScheduler, pipe } from 'rxjs';
import { bufferCount, map, observeOn, tap } from 'rxjs';

// ❌ Übermäßige Scheduler-Verwendung
range(1, 1000)
  .pipe(
    observeOn(asyncScheduler),  // 1000 setTimeout-Aufrufe
    map(x => x * 2),
    // tap(console.log)
  )
  .subscribe();

// ✅ Mit Batch-Verarbeitung optimiert
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeout-Aufrufe
    map(batch => batch.map(x => x * 2)),
    // tap(console.log)
  )
  .subscribe();
```

## Zusammenfassung

Die Wahl des Schedulers hat einen großen Einfluss auf Performance und Reaktionsfähigkeit der Anwendung. Durch Verständnis der Eigenschaften jedes Schedulers und deren korrekte Verwendung in geeigneten Situationen kann eine effiziente und flüssige Ausführung erreicht werden. Als allgemeine Richtlinie wird empfohlen:

- Für allgemeine asynchrone Verarbeitung: `asyncScheduler`
- Für rekursive Verarbeitung oder synchrones Queuing: `queueScheduler`
- Wenn schnelle Reaktionszeit erforderlich ist: `asapScheduler`
- Für Animationen: `animationFrameScheduler`
