---
description: Der subscribeOn-Operator steuert das Subscription-Start-Timing eines Observables mit einem angegebenen Scheduler und ändert den Ausführungskontext des gesamten Streams.
---

# subscribeOn - Steuerung des Subscription-Start-Timings

Der `subscribeOn`-Operator steuert **das Subscription-Start-Timing und den Ausführungskontext eines Observables mit einem angegebenen Scheduler**. Er beeinflusst das Ausführungstiming des gesamten Streams.

## 🔰 Grundlegende Syntax und Funktionsweise

Durch Angabe eines Schedulers wird der Subscription-Start asynchron gemacht.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler)
  )
  .subscribe(v => console.log('Wert:', v));

console.log('Ende');

// Ausgabe:
// Start
// Ende
// Wert: 1
// Wert: 2
// Wert: 3
```

Da der Subscription-Start selbst asynchron gemacht wird, kehrt der `subscribe()`-Aufruf sofort zurück.

[🌐 RxJS Offizielle Dokumentation - subscribeOn](https://rxjs.dev/api/index/function/subscribeOn)

## 💡 Typische Anwendungsfälle

- **Asynchronisierung schwerer Initialisierungsverarbeitung**: Verzögerung von Datenladestart usw.
- **Verhinderung von UI-Einfrieren**: Aufrechterhaltung der Reaktionsfähigkeit durch asynchronen Subscription-Start
- **Priorisierung von Verarbeitungen**: Steuerung des Starttimings mehrerer Streams
- **Timing-Steuerung in Tests**: Steuerung mit TestScheduler

## 🧪 Praktisches Codebeispiel 1: Asynchronisierung schwerer Initialisierungsverarbeitung

Beispiel für asynchronen Start von Datenladen oder Initialisierung.

```ts
import { Observable, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// UI erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'subscribeOn - Schwere Initialisierungsverarbeitung';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const timestamp = now.toLocaleTimeString('de-DE', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${timestamp}] ${message}`;
  output.appendChild(logItem);
}

// Schwere Initialisierungsverarbeitung simulieren
const heavyInit$ = new Observable<string>(subscriber => {
  addLog('Datenladestart...', '#fff9c4');

  // Schwere Verarbeitung simulieren
  let sum = 0;
  for (let i = 0; i < 10000000; i++) {
    sum += i;
  }

  addLog('Datenladen abgeschlossen', '#c8e6c9');
  subscriber.next(`Ergebnis: ${sum}`);
  subscriber.complete();
});

addLog('Subscription-Start (UI bedienbar)', '#e3f2fd');

heavyInit$
  .pipe(
    subscribeOn(asyncScheduler)  // Subscription-Start asynchron machen
  )
  .subscribe({
    next: result => addLog(`Empfangen: ${result}`, '#c8e6c9'),
    complete: () => addLog('Abgeschlossen', '#e3f2fd')
  });

addLog('Nach Subscription-Request (Ausführung läuft sofort weiter)', '#e3f2fd');
```

- Subscription-Start wird asynchron gemacht, UI reagiert sofort
- Schwere Verarbeitung wird asynchron ausgeführt
- Main-Thread wird nicht blockiert

## 🧪 Praktisches Codebeispiel 2: Prioritätssteuerung mehrerer Streams

Beispiel für Steuerung des Starttimings mehrerer Streams.

```ts
import { interval, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn, take, tap } from 'rxjs';

// UI erstellen
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'subscribeOn - Prioritätssteuerung';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string, color: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('de-DE', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.marginBottom = '2px';
  logItem.style.backgroundColor = color;
  logItem.style.fontSize = '12px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Start', '#e3f2fd');

// Hochprioritäts-Task (asapScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asapScheduler),
    tap(v => addLog2(`Hohe Priorität: ${v}`, '#c8e6c9'))
  )
  .subscribe();

// Normale Prioritäts-Task (asyncScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asyncScheduler),
    tap(v => addLog2(`Normale Priorität: ${v}`, '#fff9c4'))
  )
  .subscribe();

addLog2('Subscription-Requests abgeschlossen', '#e3f2fd');
```

- Prioritätssteuerung mit verschiedenen Schedulern
- `asapScheduler` startet früher als `asyncScheduler`

## 🆚 Unterschied zu observeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

// observeOn Beispiel
console.log('=== observeOn ===');
console.log('1: Start');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('2: tap (synchron)')),
    observeOn(asyncScheduler),
    tap(() => console.log('4: tap (asynchron)'))
  )
  .subscribe(() => console.log('5: subscribe'));

console.log('3: Ende');

// subscribeOn Beispiel
console.log('\n=== subscribeOn ===');
console.log('1: Start');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('3: tap (asynchron)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(() => console.log('4: subscribe'));

console.log('2: Ende');
```

**Hauptunterschiede**:

| Element | observeOn | subscribeOn |
|:---|:---|:---|
| **Wirkungsbereich** | Nur nachfolgende Verarbeitung | Gesamter Stream |
| **Steuerungsziel** | Emissionstiming von Werten | Subscription-Start-Timing |
| **Platzierung** | Wichtig (Verhalten ändert sich je nach Position) | Gleiche Wirkung überall |
| **Mehrfachverwendung** | Letzte wird angewendet | Erste wird angewendet |

> [!NOTE]
> Details zu `observeOn` finden Sie unter [observeOn](./observeOn.md).

## ⚠️ Wichtige Hinweise

### 1. Platzierung hat keine Auswirkung

`subscribeOn` hat die gleiche Wirkung, egal wo in der Pipeline platziert.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, map } from 'rxjs';

// Muster 1: Am Anfang
of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),
    map(x => x * 2)
  )
  .subscribe();

// Muster 2: Am Ende
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Beide haben das gleiche Verhalten
```

### 2. Bei mehrfachem subscribeOn wird das erste angewendet

```ts
import { of, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),  // Dies wird verwendet
    subscribeOn(asapScheduler)    // Dies wird ignoriert
  )
  .subscribe();
```

Der Scheduler des ersten `subscribeOn` (`asyncScheduler`) wird verwendet.

### 3. Keine Wirkung auf einige Observables

`subscribeOn` hat keine Auswirkung auf Observables wie `interval` oder `timer`, die eigene Scheduler haben.

```ts
import { interval, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// ❌ subscribeOn hat keine Wirkung
interval(1000)
  .pipe(
    subscribeOn(asyncScheduler)  // interval verwendet eigenen Scheduler
  )
  .subscribe();

// ✅ Scheduler in interval-Argument angeben
interval(1000, asyncScheduler)
  .subscribe();
```

## Praktisches Kombinationsbeispiel

```ts
import { of, asyncScheduler, animationFrameScheduler } from 'rxjs';
import { subscribeOn, observeOn, map, tap } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Tap1 (asynchron)')),
    subscribeOn(asyncScheduler),        // Subscription-Start asynchron machen
    map(x => x * 2),
    observeOn(animationFrameScheduler), // Werteemission mit Animationsframe synchronisieren
    tap(() => console.log('Tap2 (Animationsframe)'))
  )
  .subscribe(v => console.log('Wert:', v));

console.log('Ende');

// Ausführungsreihenfolge:
// Start
// Ende
// Tap1 (asynchron)
// Tap1 (asynchron)
// Tap1 (asynchron)
// Tap2 (Animationsframe)
// Wert: 2
// ... (wird fortgesetzt)
```

## Verwendungsrichtlinien

### Fall 1: Subscription-Start verzögern
```ts

// → subscribeOn verwenden
of(Daten)
  .pipe(subscribeOn(asyncScheduler))
  .subscribe();
```

### Fall 2: Nur bestimmte Verarbeitung asynchron machen
```ts
// → observeOn verwenden
of(Daten)
  .pipe(
    map(schwereVerarbeitung),
    observeOn(asyncScheduler),  // Nur nach schwerer Verarbeitung asynchron machen
    map(leichteVerarbeitung)
  )
  .subscribe();
```

### Fall 3: Gesamtes asynchron machen + Teil zusätzlich steuern
```ts
// → subscribeOn + observeOn zusammen verwenden
of(Daten)
  .pipe(
    subscribeOn(asyncScheduler),           // Gesamtes asynchron machen
    map(Verarbeitung1),
    observeOn(animationFrameScheduler),    // Für Animation ändern
    map(Verarbeitung2)
  )
  .subscribe();
```

## 📚 Verwandte Operatoren

- **[observeOn](./observeOn)** - Steuerung des Emissionstimings von Werten
- **[delay](./delay)** - Feste Zeitverzögerung

## 📖 Verwandte Dokumentation

- **[Steuerung asynchroner Verarbeitung](../../schedulers/async-control.md)** - Scheduler-Grundlagen
- **[Scheduler-Typen und Verwendung](../../schedulers/types.md)** - Details zu jedem Scheduler

## ✅ Zusammenfassung

Der `subscribeOn`-Operator steuert das Subscription-Start-Timing und den Ausführungskontext.

- ✅ Subscription-Start des gesamten Streams asynchron machen
- ✅ Effektiv zur Asynchronisierung schwerer Initialisierungsverarbeitung
- ✅ Verwendung zur Verhinderung von UI-Einfrieren
- ✅ Platzierung hat keine Auswirkung
- ⚠️ Bei mehrfacher Verwendung wird die erste angewendet
- ⚠️ Keine Wirkung auf einige Observables
- ⚠️ Unterschiedlicher Zweck als `observeOn`
