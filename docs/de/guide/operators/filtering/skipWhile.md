---
description: "Der skipWhile-Operator überspringt Werte, während die angegebene Bedingung erfüllt ist, und gibt alle nachfolgenden Werte ab dem Punkt aus, an dem die Bedingung falsch wird. Dies ist nützlich, wenn Sie einen Stream mit einer dynamischen Startbedingung steuern möchten."
---

# skipWhile - überspringt Werte, während Bedingungen erfüllt sind

Der Operator `skipWhile` überspringt weiterhin Werte **während die angegebene Bedingung erfüllt ist** und gibt **alle nachfolgenden Werte** ab dem Punkt aus, an dem die Bedingung `false` wird.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9
```

**Ablauf der Operation**:.
1. 0 ausgegeben → `0 < 5` ist `true` → überspringen
2. 1 ausgegeben → `1 < 5` ist `wahr` → überspringen
3. 2 Ausgaben → `2 < 5` ist `wahr` → überspringen
4. 3 Ausgaben → `3 < 5` ist `wahr` → überspringen
5. 4 Ausgaben → `4 < 5` ist `wahr` → überspringen
6. 5 Ausgaben → `5 < 5` ist `falsch` → Ausgabe beginnt
7. 6 und mehr → alle Ausgaben (Bedingung wird nicht neu ausgewertet)

[🌐 Offizielle RxJS Dokumentation - `skipWhile`](https://rxjs.dev/api/operators/skipWhile)

## 💡 Typisches Nutzungsmuster.

- **Anfängliche unerwünschte Daten überspringen**: Ausschluss von Daten aus der Aufwärmphase.
- **Skipping bis zum Erreichen eines Schwellenwerts**: Warten, bis bestimmte Bedingungen erfüllt sind
- **Kopfzeilen überspringen**: CSV und andere Kopfzeilen ausschließen
- **Vorbereitungszeit überspringen**: warten, bis das System bereit ist

## 🧠 Praktisches Codebeispiel 1: Überspringen der Aufwärmphase des Sensors

In diesem Beispiel werden die ersten Daten übersprungen, bis sich der Sensor stabilisiert hat.

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs';

// UIErstellt von
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Temperatursensor-Überwachung';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginBottom = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#fff3e0';
status.style.border = '1px solid #FF9800';
status.textContent = '🔄 Sensor in Vorbereitung...(Temperatur ist20°Coben, Messung beginnt).';
container.appendChild(status);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

let isWarmedUp = false;

// Simulation des Temperatursensors (langsame Erwärmung)々(allmähliche Erwärmung)
interval(500).pipe(
  take(20),
  map(i => {
    // Anfänglich niedrige Temperatur, allmählich ansteigend々Langsam ansteigend
    const baseTemp = 15 + i * 0.5;
    const noise = (Math.random() - 0.5) * 2;
    return baseTemp + noise;
  }),
  skipWhile(temp => temp < 20) // 20°CÜberspringen, wenn weniger als
).subscribe({
  next: temp => {
    // Statusaktualisierung beim Eintreffen des ersten Wertes
    if (!isWarmedUp) {
      isWarmedUp = true;
      status.textContent = '✅ Sensor bereit (Messung gestartet)';
      status.style.backgroundColor = '#e8f5e9';
      status.style.borderColor = '#4CAF50';
    }

    const log = document.createElement('div');
    log.style.padding = '5px';
    log.style.marginBottom = '3px';
    log.style.backgroundColor = temp > 25 ? '#ffebee' : '#f1f8e9';
    log.textContent = `[${new Date().toLocaleTimeString()}] Temperatur: ${temp.toFixed(1)}°C`;
    output.insertBefore(log, output.firstChild);

    // Max.10Anzeige bis zu
    while (output.children.length > 10) {
      output.removeChild(output.lastChild!);
    }
  },
  complete: () => {
    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = 'Messung abgeschlossen';
    container.appendChild(summary);
  }
});
```

- Die Daten werden übersprungen, solange der Sensor unter 20°C liegt.
- Ab dem Moment, in dem die Temperatur über 20°C liegt, werden alle Daten aufgezeichnet.

## 🎯 Praktisches Codebeispiel 2: Ereignisverarbeitung nach Bereitschaft

Dies ist ein Beispiel für das Überspringen von Ereignissen, bis die Systeminitialisierung abgeschlossen ist.

```ts
import { fromEvent, merge, Subject } from 'rxjs';
import { skipWhile, map, tap } from 'rxjs';

// UIErstellt von
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Ereignisverarbeitungssystem';
container.appendChild(title);

const initButton = document.createElement('button');
initButton.textContent = 'Initialisierung abgeschlossen';
initButton.style.marginRight = '10px';
container.appendChild(initButton);

const eventButton = document.createElement('button');
eventButton.textContent = 'Ereignis Zündung';
container.appendChild(eventButton);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
statusDiv.style.padding = '10px';
statusDiv.style.backgroundColor = '#ffebee';
statusDiv.style.border = '1px solid #f44336';
statusDiv.innerHTML = '<strong>⏸️ System nicht initialisiert</strong><br>Ereignis wird übersprungen';
container.appendChild(statusDiv);

const eventLog = document.createElement('div');
eventLog.style.marginTop = '10px';
eventLog.style.border = '1px solid #ccc';
eventLog.style.padding = '10px';
eventLog.style.minHeight = '100px';
container.appendChild(eventLog);

// Zustand der Initialisierung
let isInitialized = false;
const initSubject = new Subject<boolean>();

// Schaltfläche Initialisierung
fromEvent(initButton, 'click').subscribe(() => {
  if (!isInitialized) {
    isInitialized = true;
    initSubject.next(true);
    statusDiv.style.backgroundColor = '#e8f5e9';
    statusDiv.style.borderColor = '#4CAF50';
    statusDiv.innerHTML = '<strong>✅ Systeminitialisierung abgeschlossen</strong><br>Ereignis wird verarbeitet';
    initButton.disabled = true;
  }
});

// Ereignisverarbeitung (übersprungen bis Initialisierung abgeschlossen)
let eventCount = 0;
fromEvent(eventButton, 'click').pipe(
  map(() => {
    eventCount++;
    return {
      id: eventCount,
      timestamp: new Date(),
      initialized: isInitialized
    };
  }),
  tap(event => {
    if (!event.initialized) {
      const skipLog = document.createElement('div');
      skipLog.style.padding = '5px';
      skipLog.style.marginBottom = '3px';
      skipLog.style.color = '#999';
      skipLog.textContent = `⏭️ Ereignis #${event.id} Überspringen (uninitialisiert)`;
      eventLog.insertBefore(skipLog, eventLog.firstChild);
    }
  }),
  skipWhile(event => !event.initialized)
).subscribe(event => {
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.marginBottom = '3px';
  log.style.backgroundColor = '#e8f5e9';
  log.style.border = '1px solid #4CAF50';
  log.innerHTML = `
    <strong>✅ Ereignis #${event.id} Verarbeitung</strong>
    [${event.timestamp.toLocaleTimeString()}]
  `;
  eventLog.insertBefore(log, eventLog.firstChild);

  // Max.10Anzeige bis zu
  while (eventLog.children.length > 10) {
    eventLog.removeChild(eventLog.lastChild!);
  }
});
```

- Alle Ereignisse werden übersprungen, bis das System initialisiert ist.
- Nachdem die Initialisierung abgeschlossen ist, werden alle Ereignisse verarbeitet.

## 🆚 Vergleich mit ähnlichen Operatoren

### skipWhile vs takeWhile vs skip vs filter

```ts
import { range } from 'rxjs';
import { skipWhile, takeWhile, skip, filter } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

// skipWhile: Überspringen, solange die Bedingungen erfüllt sind, danach alle Ausgaben
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9

// takeWhile: Erfassen nur, wenn Bedingungen erfüllt sind
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4

// skip: Überspringt die ersteNÜberspringe die erste
numbers$.pipe(
  skip(5)
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9

// filter: Nur Werte weitergeben, die die Bedingung erfüllen (das Ganze auswerten)
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9
```

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9
```

**visuelle Unterschiede**:.

Eingabe: 0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0

skipWhile(n => n < 5):
[0,1,2,3,4 Überspringen] | 5, 4, 3, 2, 1, 0
                      ^Alle Ausgaben nach der BedingungfalseAlle Ausgänge nach dem

filter(n => n >= 5):
[0,1,2,3,4 Ausschluss] 5 [4,3,2,1,0 Ausschluss]
                 ^Nur Werte ausgeben, die die Bedingung erfüllen (jedes Mal ausgewertet)

takeWhile(n => n < 5):
0, 1, 2, 3, 4 | [5Alles danach ignorieren und abschließen]
```

## ⚠️ Anmerkungen.

### 1. Sobald eine Bedingung falsch ist, wird sie nicht erneut ausgewertet.

Dies ist der größte Unterschied zu `filter`.

```

```ts
import { from } from 'rxjs';
import { skipWhile, filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 4, 3, 2, 1]);

// skipWhile: Sobald die Bedingungfalse"1" wird, wird alles danach ausgegeben.
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(val => console.log('skipWhile:', val));
// Ausgabe: skipWhile: 5, 4, 3, 2, 1(Gibt alle Werte danach aus5Alle Werte danach ausgeben)

// filter: Die Bedingung jedes Mal auswerten
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(val => console.log('filter:', val));
// Ausgabe: filter: 5(Gibt alle Werte danach aus5(nur Ausgabe)
```

### 2. wenn die Bedingung von Anfang an falsch ist

Wenn die Bedingung von Anfang an `false` ist, werden alle Werte ausgegeben.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(5, 5).pipe( // 5von (bis)9bis
  skipWhile(n => n < 3) // Wenn die Bedingung von Anfang an ausgewertet wirdfalse
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9(Alle Ausgaben)
```

### 3. wenn alle Werte die Bedingung erfüllen

Wenn alle Werte die Bedingung erfüllen, wird nichts ausgegeben.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(0, 5).pipe( // 0von (bis)4bis
  skipWhile(n => n < 10) // Alle Werte erfüllen die Bedingung
).subscribe({
  next: console.log,
  complete: () => console.log('Abgeschlossen (nichts ausgeben)')
});
// Ausgabe: Abgeschlossen (nichts ausgeben)
```

### 4. Typen in TypeScript

`skipWhile` ändert den Typ nicht.

```ts
import { Observable, from } from 'rxjs';
import { skipWhile } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

const users$: Observable<User> = from([
  { id: 1, name: 'Alice', isActive: false },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true },
  { id: 4, name: 'Dave', isActive: true }
]);

// Der Typ bleibt Observable<User> immer noch derselbe wie vorher
const activeUsers$: Observable<User> = users$.pipe(
  skipWhile(user => !user.isActive)
);

activeUsers$.subscribe(user => {
  console.log(`${user.name} (ID: ${user.id})`);
});
// Ausgabe: Charlie (ID: 3), Dave (ID: 4)
```

## 💡 Praktische Kombinationsmuster

### Muster 1: Kopfzeilen überspringen

Kopfzeilen überspringen, z.B. CSV

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9
```

### Muster 2: Zeitstempel-basierte Filterung

Nur Daten nach einer bestimmten Zeit verarbeiten


### Muster 3: Zustandsbasiertes Überspringen

Überspringen, bis das System bereit ist


## 📚 Verwandte Operatoren.

- **[takeWhile](./takeWhile)** - nimmt den Wert nur, wenn die Bedingung erfüllt ist.
- **[skip](./skip)** - überspringt die ersten N Werte.
- **[skipLast](./skipLast)** - überspringt die letzten N Werte
- **[skipUntil](./skipUntil)** - überspringt, bis ein anderes Observable auslöst
- **[filter](./filter)** - nur Werte durchlassen, die die Bedingung erfüllen

## Zusammenfassung.

Der Operator "skipWhile" überspringt Werte, die eine Bedingung erfüllen, und gibt alle nachfolgenden Werte ab dem Punkt aus, an dem die Bedingung falsch wird.

- ✅ Ideal zum Überspringen von unerwünschten Anfangsdaten.
- ✅ Bedingungen werden nicht erneut ausgewertet, wenn sie falsch sind.
- ✅ Nützlich für das Überspringen von Aufwärm- und Vorbereitungsphasen
- ✅ Kann zum Überspringen von Kopfzeilen verwendet werden
- ⚠️ Im Gegensatz zu `filter` wird die Bedingung nur einmal ausgewertet
- ⚠️ Wenn alle Werte die Bedingung erfüllen, wird nichts ausgegeben
- ⚠️ Hält an, bis der ursprüngliche Datenstrom abgeschlossen ist
