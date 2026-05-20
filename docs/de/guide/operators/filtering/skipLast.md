---
description: "Der skipLast-Operator ist ein RxJS-Filteroperator, der die letzten N Werte des Observable-Streams überspringt und nur die Werte davor ausgibt."
---

# skipLast - die letzten N Werte überspringen

Der Operator "skipLast" **überspringt die letzten N Werte, die von der Quelle Observable** ausgegeben werden, und gibt nur die vorherigen Werte aus. Er behält die letzten N Werte im Puffer, bis der Stream beendet ist und gibt den Rest aus.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 wird übersprungen)
```

**Ablauf der Operation**:.
1. Der Stream gibt 0, 1, 2, ... ausgibt.
2. hält die letzten 3 Werte (7, 8, 9) in einem Puffer
3. gibt Werte aus, die die Puffergröße überschreiten (0-6)
4. bei Beendigung des Streams werden die Pufferwerte (7, 8, 9) nicht ausgegeben, sondern verworfen

[🌐 Offizielle RxJS-Dokumentation - `skipLast`](https://rxjs.dev/api/operators/skipLast)

## 💡 Typisches Nutzungsmuster.

- **letzte Daten ausschließen**: letzte Daten ausschließen, die noch nicht abgeschlossen sind
- **Stapelverarbeitung**: Ausschluss nicht abgeschlossener Daten vor Abschluss der Verarbeitung
- **Datenvalidierung**: wenn eine Validierung der nachfolgenden Werte erforderlich ist.
- **Verzögerte Verarbeitung der abgeschlossenen Daten**: wenn die letzten N Daten noch nicht abgeschlossen sind

## 🧠 Praktisches Codebeispiel 1: Datenverarbeitungspipeline

Dies ist ein Beispiel für das Überspringen der letzten nicht abgeschlossenen Daten in der Datenverarbeitung.

```ts
import { from, interval } from 'rxjs';
import { skipLast, map, take, concatMap, delay } from 'rxjs';

// UIerstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Datenverarbeitungspipeline';
container.appendChild(title);

const description = document.createElement('div');
description.style.marginBottom = '10px';
description.style.color = '#666';
description.textContent = 'Die letzten2Fälle (nicht abgeschlossene Daten) werden übersprungen und verarbeitet';
container.appendChild(description);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

interface DataPoint {
  id: number;
  value: number;
  status: 'processing' | 'confirmed' | 'skipped';
}

// Der Datenstrom (10Fall)
const data: DataPoint[] = Array.from({ length: 10 }, (_, i) => ({
  id: i,
  value: Math.floor(Math.random() * 100),
  status: 'processing' as const
}));

// 0.5Daten jede Sekunde veröffentlichen
from(data).pipe(
  concatMap(item => interval(500).pipe(
    take(1),
    map(() => item)
  )),
  skipLast(2) // Die letzten2Überspringen des letzten Falles
).subscribe({
  next: item => {
    const div = document.createElement('div');
    div.style.padding = '5px';
    div.style.marginBottom = '5px';
    div.style.backgroundColor = '#e8f5e9';
    div.style.border = '1px solid #4CAF50';
    div.innerHTML = `
      <strong>✅ Fixierung des</strong>
      ID: ${item.id} |
      Wert: ${item.value}
    `;
    output.appendChild(div);
  },
  complete: () => {
    // Übersprungene Elemente anzeigen
    const skippedItems = data.slice(-2);
    skippedItems.forEach(item => {
      const div = document.createElement('div');
      div.style.padding = '5px';
      div.style.marginBottom = '5px';
      div.style.backgroundColor = '#ffebee';
      div.style.border = '1px solid #f44336';
      div.innerHTML = `
        <strong>⏭️ Überspringen</strong>
        ID: ${item.id} |
        Wert: ${item.value} |
        (Unbestätigte Daten)
      `;
      output.appendChild(div);
    });

    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = `Verarbeitung abgeschlossen: ${data.length - 2}Position bestätigt,2Übersprungene Positionen`;
    output.appendChild(summary);
  }
});
```

- Die Daten werden sequentiell verarbeitet, aber die letzten beiden Positionen werden als nicht abgeschlossen behandelt und übersprungen.
- Nach Abschluss der Verarbeitung werden auch die übersprungenen Positionen angezeigt.

## 🎯 Praktisches Code-Beispiel 2: Log-Filterung

Dies ist ein Beispiel für das Überspringen der letzten nicht abgeschlossenen Protokolle aus einem Protokollstrom.

```ts
import { interval } from 'rxjs';
import { skipLast, map, take } from 'rxjs';

// UIerstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Log-Überwachung';
container.appendChild(title);

const info = document.createElement('div');
info.style.marginBottom = '10px';
info.textContent = 'Letzte3Fallprotokolle werden übersprungen, da sie noch nicht fertiggestellt sind';
info.style.color = '#666';
container.appendChild(info);

const confirmedLogs = document.createElement('div');
confirmedLogs.innerHTML = '<strong>📋 Logs bestätigt:</strong>';
confirmedLogs.style.marginBottom = '10px';
container.appendChild(confirmedLogs);

const confirmedList = document.createElement('div');
confirmedList.style.border = '1px solid #4CAF50';
confirmedList.style.padding = '10px';
confirmedList.style.backgroundColor = '#f1f8e9';
confirmedList.style.minHeight = '100px';
container.appendChild(confirmedList);

const pendingLogs = document.createElement('div');
pendingLogs.innerHTML = '<strong>⏳ Auf Bestätigung wartende Protokolle (übersprungen):</strong>';
pendingLogs.style.marginTop = '10px';
pendingLogs.style.marginBottom = '10px';
container.appendChild(pendingLogs);

const pendingList = document.createElement('div');
pendingList.style.border = '1px solid #FF9800';
pendingList.style.padding = '10px';
pendingList.style.backgroundColor = '#fff3e0';
pendingList.style.minHeight = '60px';
container.appendChild(pendingList);

interface LogEntry {
  id: number;
  timestamp: Date;
  level: 'info' | 'warn' | 'error';
  message: string;
}

// Erstellte Protokolle (gesamt)12Erstellte Protokolle (gesamt,1jede Sekunde)
const logs$ = interval(1000).pipe(
  take(12),
  map(i => {
    const levels: ('info' | 'warn' | 'error')[] = ['info', 'warn', 'error'];
    const messages = [
      'Benutzeranmeldung',
      'Beginn der Datenerfassung',
      'Cache-Aktualisierung',
      'Verbindungsfehler',
      'Wiederholung der Ausführung',
      'Datenverarbeitung abgeschlossen'
    ];
    return {
      id: i,
      timestamp: new Date(),
      level: levels[Math.floor(Math.random() * levels.length)],
      message: messages[Math.floor(Math.random() * messages.length)]
    } as LogEntry;
  })
);

const allLogs: LogEntry[] = [];

// Alle protokollieren (zur Bestätigung)
logs$.subscribe(log => {
  allLogs.push(log);
});

// Die letzten3Bestätigte Protokolle anzeigen, übersprungene Fälle
logs$.pipe(
  skipLast(3)
).subscribe({
  next: log => {
    const logDiv = document.createElement('div');
    logDiv.style.padding = '3px';
    logDiv.style.marginBottom = '3px';
    const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
    logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
    confirmedList.appendChild(logDiv);
  },
  complete: () => {
    // Die letzten3Den Fall anzeigen (übersprungene Protokolle)
    const skippedLogs = allLogs.slice(-3);
    skippedLogs.forEach(log => {
      const logDiv = document.createElement('div');
      logDiv.style.padding = '3px';
      logDiv.style.marginBottom = '3px';
      const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
      logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
      pendingList.appendChild(logDiv);
    });
  }
});
```

- Die Protokolle werden der Reihe nach hinzugefügt, aber die drei letzten Protokolle werden übersprungen, da sie noch nicht fertiggestellt sind.
- Nach der Fertigstellung werden auch die übersprungenen Protokolle angezeigt.

## 🆚 Vergleich mit ähnlichen Operatoren

### skipLast vs takeLast vs skip

```ts
import { range } from 'rxjs';
import { skipLast, takeLast, skip } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

// skipLast: Die letztenNEin Element überspringen
numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5, 6

// takeLast: Die letztenNNur ein Stück abrufen
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Ausgabe: 7, 8, 9

// skip: ErsteNEin Element überspringen
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Ausgabe: 3, 4, 5, 6, 7, 8, 9
```

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 wird übersprungen)
```

**visuelle Unterschiede**:.

Eingabe: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

skipLast(3): 0, 1, 2, 3, 4, 5, 6 | [7, 8, 9 Überspringen]
                                   ^Die letzten3Stücke

takeLast(3): [0~6 Überspringen] | 7, 8, 9
                             ^Die letzten3Nur 1 Stück

skip(3): [0, 1, 2 Überspringen] | 3, 4, 5, 6, 7, 8, 9
          ^Erste3Stücke
```

## ⚠️ Anmerkungen.

### 1. funktioniert mit unendlichen Strömen

`skipLast` funktioniert bei unendlichen Streams nicht wie beabsichtigt, da es das letzte N bis zur Fertigstellung nicht identifizieren kann.

```

```ts
import { interval } from 'rxjs';
import { skipLast } from 'rxjs';

// ❌ Schlechtes Beispiel: Mit unendlichen Strömen skipLast mit einem unendlichen Strom
interval(1000).pipe(
  skipLast(3)
).subscribe(console.log);
// Ausgabe: 0(3(nach einer Sekunde), 1(4(nach einer Sekunde), 2(5(nach einer Sekunde), ...
// NDie Ausgabe wird mit einer Verzögerung von 1 unendlich fortgesetzt
// Die letzten3(nach 1,5 Sekunden), mit einer Verzögerung von 1,5 Sekunden.
```

Bei unendlichen Strömen werden alle Werte mit einer Verzögerung von N weiter ausgegeben, da die letzten N nicht ermittelt werden. Der ursprüngliche Zweck von `skipLast` wird nicht erreicht, da es kein echtes "letztes N" gibt.

**Lösung**: `take` zu einem endlichen Stream

```ts
import { interval } from 'rxjs';
import { take, skipLast } from 'rxjs';

// ✅ Gutes Beispiel: Nach einem endlichen Stream skipLast mit einem unendlichen Strom
interval(1000).pipe(
  take(10),      // Erste10Beendet in 1 Stück
  skipLast(3)    // Die letzten3Ein Element überspringen
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 wird übersprungen)
```

### 2. Achten Sie auf die Puffergröße

`skipLast(n)` behält immer n Werte im Puffer.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

// ⚠️ 10001 Stück wird in einem Puffer aufbewahrt
range(0, 1000000).pipe(
  skipLast(1000)
).subscribe(console.log);
```

### 3. output delay.

`skipLast(n)` gibt nichts aus, bis n Puffer gefüllt sind.

```ts
import { interval } from 'rxjs';
import { take, skipLast, tap } from 'rxjs';

interval(1000).pipe(
  take(5),
  tap(val => console.log('Eingabe:', val)),
  skipLast(2)
).subscribe(val => console.log('Ausgabe:', val));
// Eingabe: 0
// Eingabe: 1
// Eingabe: 2
// Ausgabe: 0  ← Die Ausgabe beginnt, wenn der Puffer2Die Ausgabe beginnt, wenn der Puffer voll ist
// Eingabe: 3
// Ausgabe: 1
// Eingabe: 4
// Ausgabe: 2
// Fertigstellung (Überspringen3, 4 (Überspringen)
```

### 4. skipLast(0) Verhalten

`skipLast(0)` überspringt nichts.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 wird übersprungen)
```

## 💡 Praktische Kombinationsmuster

### Muster 1: Nur den Zwischenteil erhalten

Überspringen Sie den Anfang und das Ende und erhalten Sie nur den mittleren Teil


### Muster 2: Datenüberprüfung

Wenn eine Überprüfung der nachfolgenden Werte erforderlich ist


```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0von (bis)9bis

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 wird übersprungen)
```

### Muster 3: Fensterverarbeitung

Fensterverarbeitung mit Daten unter Ausschluss der letzten N Fälle


## 📚 Verwandte Operatoren

- **[skip](./skip)** - überspringt die ersten N Werte.
- **[takeLast](./takeLast)** - nimmt nur die letzten N Werte.
- **[take](./take)** - holt nur die ersten N Werte.
- **[skipUntil](./skipUntil)** - überspringt, bis ein anderes Observable auslöst
- **[skipWhile](./skipWhile)** - Überspringen, solange die Bedingung erfüllt ist

## Zusammenfassung.

Der `skipLast`-Operator überspringt die letzten N Werte des Streams.

- ✅ Ideal, wenn die letzten N Daten nicht benötigt werden.
- ✅ Nützlich, um unbestimmte Daten auszuschließen.
- ✅ Die Puffergröße beträgt nur N (speichereffizient)
- ✅ Abschluss des Datenstroms erforderlich
- ⚠️ Nicht verfügbar für unendliche Datenströme
- ⚠️ Keine Ausgabe, bis N Puffer akkumuliert worden sind
- ⚠️ Muss bei endlichen Datenströmen oft mit `take` kombiniert werden
