---
description: Der timestamp-Operator fügt jedem Wert einen Zeitstempel hinzu und zeichnet die Emissionszeit auf, wodurch er für Leistungsmessungen und Debugging genutzt werden kann.
---

# timestamp - Hinzufügen von Zeitstempeln

Der `timestamp`-Operator fügt jedem Wert im Stream **einen Zeitstempel hinzu**. Durch Aufzeichnung der genauen Emissionszeit kann er für Leistungsmessungen, Debugging und Zeitreihenanalysen von Ereignissen genutzt werden.

## 🔰 Grundlegende Syntax und Funktionsweise

Wandelt jeden Wert in ein Objekt mit Zeitstempel um.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

interval(1000)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(console.log);

// Ausgabe:
// { value: 0, timestamp: 1640000000000 }
// { value: 1, timestamp: 1640000001000 }
// { value: 2, timestamp: 1640000002000 }
```

Das zurückgegebene Objekt hat folgende Struktur:
- `value`: Der ursprüngliche Wert
- `timestamp`: Zeitstempel (Unix-Zeit in Millisekunden)

[🌐 RxJS Offizielle Dokumentation - timestamp](https://rxjs.dev/api/index/function/timestamp)

## 💡 Typische Anwendungsfälle

- **Leistungsmessung**: Zeitmessung der Verarbeitung
- **Ereignis-Timing-Analyse**: Messung von Benutzeraktionsintervallen
- **Debugging und Logging**: Aufzeichnung von Emissionstiming
- **Zeitreihendaten-Aufzeichnung**: Speicherung mit Zeitstempel wie Sensordaten

## 🧪 Praktisches Codebeispiel 1: Messung des Klick-Intervalls

Ein Beispiel zur Messung der Klickintervalle von Benutzern.

```ts
import { fromEvent } from 'rxjs';
import { timestamp, pairwise, map } from 'rxjs';

// UI erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'timestamp - Klick-Intervall-Messung';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Bitte klicken';
button.style.marginBottom = '10px';
button.style.padding = '10px 20px';
button.style.fontSize = '16px';
container.appendChild(button);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '250px';
output.style.overflow = 'auto';
container.appendChild(output);

let clickCount = 0;

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.insertBefore(logItem, output.firstChild);  // Neueste oben anzeigen
}

fromEvent(button, 'click')
  .pipe(
    timestamp(),
    pairwise(),
    map(([prev, curr]) => {
      const interval = curr.timestamp - prev.timestamp;
      return {
        clickNumber: clickCount + 1,
        interval: interval,
        timestamp: new Date(curr.timestamp).toLocaleTimeString('de-DE')
      };
    })
  )
  .subscribe(data => {
    clickCount++;
    const color = data.interval < 500 ? '#ffcdd2' :
                  data.interval < 1000 ? '#fff9c4' : '#c8e6c9';

    const speed = data.interval < 500 ? 'Schneller Klick!' :
                  data.interval < 1000 ? 'Normal' : 'Langsam';

    addLog(
      `${data.clickNumber}. Klick: ${data.interval}ms Intervall [${speed}] (${data.timestamp})`,
      color
    );
  });

addLog('Bitte Button klicken (Intervall wird ab 2. Klick gemessen)', '#e3f2fd');
```

- Präzise Messung des Klick-Intervalls
- Farbkodierte Anzeige nach Geschwindigkeit
- Aufzeichnung der Auftretenszeit durch Zeitstempel

## 🧪 Praktisches Codebeispiel 2: Messung der Verarbeitungszeit

Ein Beispiel zur Zeitmessung jeder Verarbeitung.

```ts
import { interval } from 'rxjs';
import { timestamp, map, take } from 'rxjs';

// UI erstellen
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'timestamp - Verarbeitungszeitmessung';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.fontSize = '12px';
  logItem.style.fontFamily = 'monospace';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

addLog2('Verarbeitung gestartet...');

interval(500)
  .pipe(
    take(5),
    timestamp(),  // Zeitstempel vor Verarbeitung
    map(data => {
      const start = data.timestamp;

      // Schwere Verarbeitung simulieren (zufällige Verarbeitungszeit)
      const iterations = Math.floor(Math.random() * 5000000) + 1000000;
      let sum = 0;
      for (let i = 0; i < iterations; i++) {
        sum += i;
      }

      const end = Date.now();
      const duration = end - start;

      return {
        value: data.value,
        startTime: new Date(start).toLocaleTimeString('de-DE', { hour12: false }) +
                   '.' + (start % 1000).toString().padStart(3, '0'),
        duration: duration
      };
    })
  )
  .subscribe({
    next: result => {
      addLog2(
        `Wert${result.value}: Start=${result.startTime}, Verarbeitungszeit=${result.duration}ms`
      );
    },
    complete: () => {
      addLog2('--- Alle Verarbeitungen abgeschlossen ---');
    }
  });
```

- Aufzeichnung der Verarbeitungsstartzeit für jeden Wert
- Messung der Verarbeitungsdauer
- Verwendung für Leistungsanalyse

## 🧪 Praktisches Codebeispiel 3: Ereignisprotokoll

Ein Beispiel für Log-Ausgabe mit Zeitstempel für alle Ereignisse.

```ts
import { merge, fromEvent, interval } from 'rxjs';
import { timestamp, map, take } from 'rxjs';

// UI erstellen
const container3 = document.createElement('div');
container3.style.marginTop = '20px';
document.body.appendChild(container3);

const title3 = document.createElement('h3');
title3.textContent = 'timestamp - Ereignisprotokoll';
container3.appendChild(title3);

const clickButton = document.createElement('button');
clickButton.textContent = 'Klick';
clickButton.style.marginRight = '10px';
container3.appendChild(clickButton);

const hoverDiv = document.createElement('div');
hoverDiv.textContent = 'Maus hier bewegen';
hoverDiv.style.display = 'inline-block';
hoverDiv.style.padding = '10px';
hoverDiv.style.border = '2px solid #4CAF50';
hoverDiv.style.cursor = 'pointer';
container3.appendChild(hoverDiv);

const log3 = document.createElement('div');
log3.style.marginTop = '10px';
log3.style.border = '1px solid #ccc';
log3.style.padding = '10px';
log3.style.maxHeight = '200px';
log3.style.overflow = 'auto';
log3.style.fontFamily = 'monospace';
log3.style.fontSize = '12px';
container3.appendChild(log3);

function addLog3(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.backgroundColor = color;
  logItem.style.padding = '2px';
  logItem.textContent = message;
  log3.insertBefore(logItem, log3.firstChild);
}

// Mehrere Ereignisquellen integrieren
const events$ = merge(
  fromEvent(clickButton, 'click').pipe(map(() => 'CLICK')),
  fromEvent(hoverDiv, 'mouseenter').pipe(map(() => 'HOVER_IN')),
  fromEvent(hoverDiv, 'mouseleave').pipe(map(() => 'HOVER_OUT')),
  interval(3000).pipe(take(5), map(i => `TIMER_${i}`))
);

events$
  .pipe(
    timestamp()
  )
  .subscribe(data => {
    const time = new Date(data.timestamp).toLocaleTimeString('de-DE', { hour12: false }) +
                 '.' + (data.timestamp % 1000).toString().padStart(3, '0');

    const colors: Record<string, string> = {
      'CLICK': '#c8e6c9',
      'HOVER_IN': '#fff9c4',
      'HOVER_OUT': '#ffccbc',
    };

    const color = data.value.startsWith('TIMER') ? '#e1bee7' :
                  (colors[data.value] || '#e3f2fd');

    addLog3(`[${time}] Ereignis: ${data.value}`, color);
  });

addLog3('Ereignisprotokoll läuft...', '#e3f2fd');
```

- Integration mehrerer Ereignisquellen
- Hinzufügen von Zeitstempeln zu allen Ereignissen
- Verfolgung von Ereignissen in Zeitreihenfolge

## Verwendung von Zeitstempeln

```ts
import { of } from 'rxjs';
import { timestamp, map } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    timestamp(),
    map(data => {
      // Verarbeitung mit Zeitstempel
      const date = new Date(data.timestamp);
      return {
        value: data.value,
        time: date.toISOString(),
        unixTime: data.timestamp
      };
    })
  )
  .subscribe(console.log);
// Ausgabe:
// { value: 'A', time: '2024-01-01T00:00:00.000Z', unixTime: 1704067200000 }
// ...
```

## ⚠️ Wichtige Hinweise

### 1. Genauigkeit des Zeitstempels

Da `Date.now()` von JavaScript verwendet wird, ist die Genauigkeit in Millisekunden.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

// Hochfrequenzereignisse (1ms-Intervall)
interval(1)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(data => {
    console.log(`Wert: ${data.value}, Zeitstempel: ${data.timestamp}`);
  });
// Derselbe Zeitstempel kann auftreten
```

Bei Bedarf an höherer Genauigkeit sollten Sie die Verwendung von `performance.now()` in Betracht ziehen.

### 2. Zeitstempel ist zum Emissionszeitpunkt

Es ist der Zeitstempel zum Zeitpunkt der Emission, nicht zum Zeitpunkt der Erzeugung.

```ts
import { of, delay, timestamp } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delay(1000),      // 1 Sekunde Verzögerung
    timestamp()       // Zeitstempel nach Verzögerung
  )
  .subscribe(console.log);
```

### 3. Änderung der Objektstruktur

Bei Verwendung von `timestamp` werden Werte in Objekte eingeschlossen.

```ts
import { of, timestamp, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    timestamp(),
    map(data => data.value * 2)  // Zugriff auf ursprünglichen Wert mit .value
  )
  .subscribe(console.log);
// Ausgabe: 2, 4, 6
```

## 📚 Verwandte Operatoren

- **[tap](./tap)** - Ausführung von Nebenwirkungen (für Debugging)
- **[delay](./delay)** - Feste Zeitverzögerung
- **[timeout](./timeout)** - Timeout-Steuerung

## ✅ Zusammenfassung

Der `timestamp`-Operator fügt jedem Wert einen Zeitstempel hinzu.

- ✅ Präzise Aufzeichnung der Emissionszeit jedes Werts
- ✅ Effektiv für Leistungsmessungen
- ✅ Analyse von Ereignisintervallen möglich
- ✅ Verwendung für Debugging und Logging
- ⚠️ Genauigkeit in Millisekunden
- ⚠️ Werte werden in Objekte eingeschlossen
- ⚠️ Zeitstempel ist zum Emissionszeitpunkt
