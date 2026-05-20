---
description: "Erfahren Sie mehr über die Eigenschaften, die Implementierung und die Verwendung der wichtigsten Scheduler in RxJS, wie asyncScheduler und queueScheduler. Verstehen Sie die Unterschiede zwischen Makro-Tasks, Mikro-Tasks und synchroner Verarbeitung und lernen Sie das Ausführungs-Timing und die Eigenschaften der einzelnen Scheduler kennen. Sie können die Leistung und das Verhalten Ihrer Anwendung optimieren, indem Sie sie richtig einsetzen."
---

# Arten von Schedulern und wie man sie benutzt

RxJS bietet mehrere Scheduler für verschiedene Anwendungen. Jeder Scheduler hat sein eigenes spezifisches Ausführungs-Timing und seine eigenen Eigenschaften und kann entsprechend verwendet werden, um die Leistung und das Verhalten der Anwendung zu optimieren.

## Klassifizierung von Schedulern

RxJS-Scheduler lassen sich in drei Hauptkategorien einteilen.

1. **Makro-Tasks**: werden in der nächsten Task-Warteschlange in der Ereignisschleife ausgeführt
2. **Micro-Task**: wird unmittelbar nach Abschluss der aktuellen Aufgabe und vor Beginn der nächsten Aufgabe ausgeführt
3. **Synchrone Verarbeitung**: sofortige Ausführung

Weitere Informationen finden Sie unter [Task- und Scheduler-Grundlagen](./task-and-scheduler-basics.md).

## Haupt-Scheduler.

### asyncScheduler

#### Merkmale.
- **Interne Implementierung**: verwendet setTimeout.
- **Ausführungszeit**: Makroaufgaben
- **Verwendung**: Allgemeine asynchrone Verarbeitung, Zeitraffer-Verarbeitung

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
  // Schwere Berechnungen simulieren
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
    console.log(`Berechnungsergebnisse: ${result}`);
  });
```

### queueScheduler

#### Merkmale.
- **Interne Implementierung**: Mikro-Task-Warteschlange
- **Ausführungszeitpunkt**: innerhalb der aktuellen Aufgabe (scheint synchron zu sein)
- **Verwendung**: Task-Warteschlange, Optimierung von Rekursionen

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Verarbeitung von Warteschlangen')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: Ende');

// Ausgabe:
// 1: Start
// 2: Verarbeitung von Warteschlangen
// 3: Ende
```

#### Anwendungsfall

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs';

// Optimierung der rekursiven Verarbeitung
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

#### Merkmale.
- **Interne Implementierung**: Promise.resolve().then() oder setImmediate
- **Ausführungszeitpunkt**: Microtasks
- **Verwendung**: Für asynchrone Ausführung so schnell wie möglich

~```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('ASAPVerarbeitung von')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Ende von');

// Ausgabe:
// 1: Start
// 2: Ende von
// 3: ASAPVerarbeitung von
```

#### Anwendungsfall

```

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
    // UIUpdate-Verarbeitung von
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Merkmale.
- **Interne Implementierung**: requestAnimationFrame
- **Ausführungszeitpunkt**: vor dem nächsten Bildschirm-Rendering
- **Verwendung**: Animation, Zeichenprozess für 60fps.

#### Beispiel für eine einfache Rotationsanimation

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// HTMLErstellen von Elementen
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Einrichten von Animationen
let rotation = 0;

// 60fpsIn den2Sekunden der Animation
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2Sekunden = 120Rahmen
    map(() => {
      rotation += 3;  // 1Jeder Rahmen3Drehung in Grad
      return rotation;
    })
  )
  .subscribe(angle => {
    // DOMTatsächliche Drehung des Elements
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Warum brauchen wir animationFrameScheduler?

Der `animationFrameScheduler` führt seine Operationen synchron mit dem Zeichenzyklus des Browsers aus, was die folgenden Vorteile hat.

1. flüssige Animationen**: da die Verarbeitung mit dem Rendering-Timing des Browsers (typischerweise 60 fps) synchronisiert ist, können flüssige Animationen ohne jegliche Abbrüche erreicht werden.
2.**Effiziente Ressourcennutzung**: Wenn der Browser die Registerkarte deaktiviert, wird die Ausführung des requestAnimationFrame automatisch angehalten, um unnötige CPU-Nutzung zu vermeiden.
3, **Vermeidung von Bildschirmflimmern**: Bildschirmflimmern und unvollständige Frames werden verhindert, da die Berechnung zuverlässig abgeschlossen wird, bevor der Bildschirm gezeichnet wird.

Hier ist ein Vergleich zwischen `setInterval` und `animationFrameScheduler`.

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ setIntervalIneffiziente Animation mit
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // ungefähr60fps

// Probleme:
// - Nicht mit den Rendering-Zeiten des Browsers synchronisiert
// - Läuft auch in Hintergrund-Tabs weiter
// - Genaue60fpskann nicht garantiert werden

// ✅ animationFrameSchedulerEffiziente Animation mit
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
// - Synchronisiert mit dem Browser-Zeichentiming
// - Automatisch pausiert in Hintergrund-Tabs
// - Stabile60fpsErmöglicht
```

#### Beispiel einer mausverfolgenden Animation

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Erstellung eines zu verfolgenden Kreises
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Transparent für Mausereignisse
document.body.appendChild(circle);

// Aktuelle und Zielposition
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Überwachung von Mausbewegungsereignissen
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
    // Festlegen der Mausposition als Ziel
    targetX = x;
    targetY = y;
    
    // Allmähliche Bewegung von der aktuellen Position zur Zielposition々Allmähliche Bewegung (easing) von der aktuellen Position zur Zielposition
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;
    
    // DOMElement aktualisieren
    circle.style.left = `${currentX - 15}px`;  // Anpassung an Mittelposition
    circle.style.top = `${currentY - 15}px`;
  });
```

## Anleitung zur Verwendung verschiedener Scheduler

### Vergleich nach Ausführungszeitpunkt

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

### Auswahlkriterien nach Anwendung


## Praktische Anwendungsfälle

## Verarbeitung großer Datenmengen


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

### WebSocket Nachrichtenverarbeitung

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// 注: これは概念を示す疑似コードです
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // 文字列として扱う
});

socket$
  .pipe(
    // 高速な応答が必要なメッセージ処理
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('メッセージ受信:', msg);
}
```

### Fehler-Wiederholungskontrolle

Durch die Verwendung des Schedulers mit dem Operator `retry` kann das Timing der Retrys genau gesteuert werden.

#### Grundlegende Wiederholungskontrolle

Die Option `delay` des Operators `retry` verwendet intern den `asyncScheduler` zur Steuerung des Retry-Intervalls.


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

#### Scheduler-Auslastung mit exponentiellem Backoff

Als fortgeschrittene Kontrolle kann ein exponentielles Backoff durch die Kombination von `retryWhen` und `asyncScheduler` implementiert werden.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

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
    // RxJS 7.3+ Empfehlung: retry({ count, delay }) Format
    retry({
      count: 3, // Max.3Bis zu dreimaliger Wiederholungsversuch
      delay: (error, retryCount) => {
        // Exponentielle Rückstellung: 1Sekunden, 2Sekunden, 4Sekunden...
        const delayTime = Math.pow(2, retryCount - 1) * 1000;
        console.log(`🔄 Wiederholungen ${retryCount}mal (${delayTime}msNach)`);

        // timer ist intern asyncScheduler Verwendung von
        return timer(delayTime);
      }
    })
  )
  .subscribe({
    next: result => console.log('✅ Erfolg:', result),
    error: error => {
      console.log('❌ Max.Anzahl der Wiederholungen erreicht');
      console.log('❌ FinaleFehler:', error.message);
    }
  });

// AusgabeBsp.:
// 🔄 Wiederholungen 1mal (1000msNach)
// 🔄 Wiederholungen 2mal (2000msNach)
// 🔄 Wiederholungen 3mal (4000msNach)
// ❌ Max.Anzahl der Wiederholungen erreicht
// ❌ FinaleFehler: Temporary error
```

#### Bei expliziter Angabe eines asyncSchedulers

Die explizite Angabe eines bestimmten Schedulers ermöglicht eine flexiblere Steuerung, z. B. das Ersetzen durch `TestScheduler` während des Testens.


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


> - Detaillierte Implementierungsmuster und Debugging-Methoden für die Behandlung von retry sind auf der Seite [retry and catchError](/de/guide/error-handling/retry-catch) beschrieben.
> - Detaillierte Verwendung des retry-Operators.
> - Kombinationsmuster mit catchError.
> - Debugging-Techniken für Wiederholungen (z. B. Verfolgung der Anzahl der Versuche, Protokollierung usw.)

## Auswirkungen auf die Leistung.

### Scheduler-Overhead


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

## Zusammenfassung.

Die Wahl des Schedulers hat einen erheblichen Einfluss auf die Leistung und Reaktionsfähigkeit einer Anwendung. Wenn Sie die Eigenschaften der einzelnen Scheduler kennen und sie in den richtigen Situationen einsetzen, ist ein effizienter und reibungsloser Betrieb gewährleistet. Als allgemeine Richtlinie,

- `asyncScheduler` für allgemeine asynchrone Verarbeitung.
- `queueScheduler` für rekursive Verarbeitung und synchrone Warteschlangen
- `asapScheduler` für schnelle Reaktionszeiten
- `animationFrameScheduler` für Animationen

für Animationen.