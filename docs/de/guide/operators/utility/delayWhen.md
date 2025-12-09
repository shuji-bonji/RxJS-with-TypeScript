---
description: "Der delayWhen-Operator steuert das Verzögerungstiming für jeden Wert dynamisch durch ein individuelles Observable. Flexible Verzögerungsverarbeitung je nach Bedingungen, exponentielles Backoff bei Wiederholungen, Warten auf Benutzeraktionen, Anpassung von API-Aufrufintervallen und weitere praktische Muster werden anhand von TypeScript-Codebeispielen erklärt."
---

# delayWhen - Dynamische Verzögerungssteuerung

Der `delayWhen`-Operator bestimmt die Verzögerungszeit für jeden Wert **dynamisch durch ein individuelles Observable**. Während der `delay`-Operator eine feste Zeitverzögerung bietet, ermöglicht `delayWhen` unterschiedliche Verzögerungen für jeden Wert.

## 🔰 Grundlegende Syntax und Funktionsweise

Sie geben eine Funktion an, die für jeden Wert ein Observable zurückgibt, das die Verzögerung bestimmt.

```ts
import { of, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    delayWhen(value => {
      const delayTime = value === 'B' ? 2000 : 1000;
      return timer(delayTime);
    })
  )
  .subscribe(console.log);
// Ausgabe:
// A (nach 1 Sekunde)
// C (nach 1 Sekunde)
// B (nach 2 Sekunden)
```

In diesem Beispiel wird nur für den Wert `'B'` eine Verzögerung von 2 Sekunden angewendet, während die anderen eine Verzögerung von 1 Sekunde haben.

[🌐 RxJS Offizielle Dokumentation - delayWhen](https://rxjs.dev/api/index/function/delayWhen)

## 💡 Typische Anwendungsfälle

- **Verzögerung je nach Wert**: Ändern der Verzögerungszeit basierend auf Priorität oder Typ
- **Verzögerung durch externe Ereignisse**: Warten auf Benutzeraktionen oder Abschluss anderer Streams
- **Bedingte Verzögerung**: Nur bestimmte Werte verzögern
- **Asynchrone Timing-Steuerung**: Warten auf API-Antworten oder Datenvorbereitung

## 🧪 Praktisches Codebeispiel 1: Verzögerung nach Priorität

Ein Beispiel zur Steuerung des Verarbeitungstimings basierend auf Task-Priorität.

```ts
import { from, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

// UI erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'delayWhen - Verzögerung nach Priorität';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '300px';
output.style.overflow = 'auto';
container.appendChild(output);

interface Task {
  id: number;
  name: string;
  priority: 'high' | 'medium' | 'low';
}

const tasks: Task[] = [
  { id: 1, name: 'Task A', priority: 'low' },
  { id: 2, name: 'Task B', priority: 'high' },
  { id: 3, name: 'Task C', priority: 'medium' },
  { id: 4, name: 'Task D', priority: 'high' },
  { id: 5, name: 'Task E', priority: 'low' }
];

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const time = now.toLocaleTimeString('de-DE', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${time}] ${message}`;
  output.appendChild(logItem);
}

addLog('Verarbeitung gestartet', '#e3f2fd');

from(tasks)
  .pipe(
    delayWhen(task => {
      // Verzögerungszeit je nach Priorität festlegen
      let delayTime: number;
      switch (task.priority) {
        case 'high':
          delayTime = 500;  // Hohe Priorität: 0,5 Sekunden
          break;
        case 'medium':
          delayTime = 1500; // Mittlere Priorität: 1,5 Sekunden
          break;
        case 'low':
          delayTime = 3000; // Niedrige Priorität: 3 Sekunden
          break;
      }
      return timer(delayTime);
    })
  )
  .subscribe({
    next: task => {
      const colors = {
        high: '#c8e6c9',
        medium: '#fff9c4',
        low: '#ffccbc'
      };
      addLog(
        `${task.name} (Priorität: ${task.priority}) wird verarbeitet`,
        colors[task.priority]
      );
    },
    complete: () => {
      addLog('Alle Tasks abgeschlossen', '#e3f2fd');
    }
  });
```

- Tasks mit hoher Priorität werden nach 0,5 Sekunden verarbeitet
- Mittlere Priorität nach 1,5 Sekunden, niedrige Priorität nach 3 Sekunden
- Realisierung der Verarbeitungsreihenfolge entsprechend der Task-Wichtigkeit

## 🧪 Praktisches Codebeispiel 2: Verzögerung durch externe Ereignisse

Ein Beispiel zum Warten auf einen Benutzerklick vor der Wert-Emission.

```ts
import { of, fromEvent } from 'rxjs';
import { delayWhen, take, tap } from 'rxjs';

// UI erstellen
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'delayWhen - Auf Klick warten';
container2.appendChild(title2);

const button = document.createElement('button');
button.textContent = 'Klicken für nächsten Wert';
button.style.marginBottom = '10px';
container2.appendChild(button);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.minHeight = '100px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

let clickCount = 0;

of('Nachricht 1', 'Nachricht 2', 'Nachricht 3')
  .pipe(
    tap(msg => {
      addLog2(`Warten: ${msg} (Bitte Button klicken)`);
      button.textContent = `Klicken zum Anzeigen von "${msg}"`;
    }),
    delayWhen(() => {
      // Verzögerung bis zum Klick-Ereignis
      return fromEvent(button, 'click').pipe(take(1));
    })
  )
  .subscribe({
    next: msg => {
      clickCount++;
      addLog2(`✅ Angezeigt: ${msg}`);
      if (clickCount < 3) {
        button.disabled = false;
      } else {
        button.textContent = 'Abgeschlossen';
        button.disabled = true;
      }
    },
    complete: () => {
      addLog2('--- Alle Nachrichten wurden angezeigt ---');
    }
  });
```

- Jeder Wert wird nach Benutzerklick emittiert
- Verzögerungssteuerung mit externem Ereignis als Trigger ist möglich
- Anwendbar auf interaktive Sequenzverarbeitung

## 🆚 Vergleich mit delay

```ts
import { of, timer } from 'rxjs';
import { delay, delayWhen } from 'rxjs';

// delay - Feste Zeitverzögerung
of(1, 2, 3)
  .pipe(delay(1000))
  .subscribe(console.log);
// Alle Werte werden 1 Sekunde verzögert

// delayWhen - Unterschiedliche Verzögerung pro Wert
of(1, 2, 3)
  .pipe(
    delayWhen(value => timer(value * 1000))
  )
  .subscribe(console.log);
// 1 nach 1 Sekunde, 2 nach 2 Sekunden, 3 nach 3 Sekunden
```

| Operator | Verzögerungssteuerung | Anwendungsfall |
|:---|:---|:---|
| `delay` | Feste Zeit | Einfache einheitliche Verzögerung |
| `delayWhen` | Dynamisch (pro Wert) | Bedingte Verzögerung, Warten auf externe Ereignisse |

## ⚠️ Wichtige Hinweise

### 1. Delay-Observable wird jedes Mal neu erstellt

```ts
// ❌ Schlechtes Beispiel: Wiederverwendung derselben Observable-Instanz
const delayObs$ = timer(1000);
source$.pipe(
  delayWhen(() => delayObs$)  // Funktioniert ab dem zweiten Mal nicht
).subscribe();

// ✅ Gutes Beispiel: Jedes Mal neue Observable erstellen
source$.pipe(
  delayWhen(() => timer(1000))
).subscribe();
```

### 2. Wenn das Delay-Observable nicht abgeschlossen wird

```ts
import { of, NEVER } from 'rxjs';
import { delayWhen } from 'rxjs';

// ❌ Schlechtes Beispiel: NEVER zurückgeben führt zu ewiger Verzögerung
of(1, 2, 3)
  .pipe(
    delayWhen(() => NEVER)  // Werte werden nicht emittiert
  )
  .subscribe(console.log);
// Keine Ausgabe
```

Das Delay-Observable muss unbedingt einen Wert emittieren oder abgeschlossen werden.

### 3. Fehlerbehandlung

Wenn ein Fehler im Delay-Observable auftritt, wird der gesamte Stream fehlerhaft.

```ts
import { of, throwError, timer, delayWhen } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delayWhen(value => {
      if (value === 2) {
        return throwError(() => new Error('Verzögerungsfehler'));
      }
      return timer(1000);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Fehler:', err.message)
  });
// Ausgabe: 1
// Fehler: Verzögerungsfehler
```

## 📚 Verwandte Operatoren

- **[delay](./delay)** - Feste Zeitverzögerung
- **[debounceTime](/de/guide/operators/filtering/debounceTime)** - Verzögerung nach Eingabestopp
- **[throttleTime](/de/guide/operators/filtering/throttleTime)** - Werte in regelmäßigen Abständen durchlassen
- **[timeout](./timeout)** - Timeout-Steuerung

## ✅ Zusammenfassung

Der `delayWhen`-Operator steuert das Verzögerungstiming für jeden Wert dynamisch.

- ✅ Unterschiedliche Verzögerungen pro Wert möglich
- ✅ Verzögerungssteuerung durch externe Ereignisse oder Observables
- ✅ Anpassung des Verarbeitungstimings nach Priorität oder Typ
- ⚠️ Delay-Observable muss jedes Mal neu erstellt werden
- ⚠️ Delay-Observable muss abgeschlossen werden oder einen Wert emittieren
- ⚠️ Auf Fehlerbehandlung achten
