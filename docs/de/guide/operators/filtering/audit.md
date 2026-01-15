---
description: Der audit-Operator ist ein RxJS-Filteroperator, der nur den letzten Wert innerhalb eines durch ein benutzerdefiniertes Observable gesteuerten Zeitraums ausgibt. Ideal für dynamische Timing-Steuerung.
---

# audit - Letzter Wert bei Trigger

Der `audit`-Operator wartet, bis ein benutzerdefiniertes Observable einen Wert ausgibt, und gibt dann den **letzten Wert** aus, der während dieses Zeitraums von der Quelle ausgegeben wurde.
Während `auditTime` mit einer festen Zeit gesteuert wird, kann `audit` den Zeitraum **durch ein dynamisches Observable steuern**.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Click-Ereignis
const clicks$ = fromEvent(document, 'click');

// Zeitraum alle 1 Sekunde abgrenzen
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Klick wurde aufgezeichnet');
});
```

- Wenn ein Klick auftritt, beginnt ein 1-Sekunden-Zeitraum.
- Nur der letzte Klick innerhalb dieser 1 Sekunde wird ausgegeben.
- Nach 1 Sekunde beginnt der nächste Zeitraum.

[🌐 RxJS Offizielle Dokumentation - `audit`](https://rxjs.dev/api/operators/audit)

## 💡 Typische Anwendungsmuster

- **Sampling mit dynamischen Intervallen**: Anpassung des Zeitraums je nach Last
- **Benutzerdefinierte Timing-Steuerung**: Zeitraumsteuerung basierend auf anderen Observables
- **Adaptive Ereignisbegrenzung**: Ausfiltern je nach Situation

## 🔍 Unterschied zu auditTime

| Operator | Zeitraumsteuerung | Anwendungsfall |
|:---|:---|:---|
| `auditTime` | Feste Zeit (Millisekunden) | Einfache zeitbasierte Steuerung |
| `audit` | **Benutzerdefiniertes Observable** | **Dynamische Zeitraumsteuerung** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Feste 1 Sekunde
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Feste 1 Sekunde'));

// audit - Dynamischer Zeitraum
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // Zufälliger Zeitraum 0-2 Sekunden
    return timer(period);
  })
).subscribe(() => console.log(`Dynamischer Zeitraum: ${period}ms`));
```

## 🧠 Praktisches Codebeispiel 1: Dynamisches Sampling nach Last

Ein Beispiel, bei dem das Sampling-Intervall je nach Systemlast angepasst wird.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UI erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>Dynamisches Sampling</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Last ändern';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Laststufe (0: niedrig, 1: mittel, 2: hoch)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Niedrige Last', 'Mittlere Last', 'Hohe Last'];
  statusDiv.textContent = `Aktuelle Last: ${levels[loadLevel]}`;
});

// Mausbewegungsereignis
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Zeitraum je nach Last anpassen
    const periods = [2000, 1000, 500]; // Niedrige Last→längerer Zeitraum, hohe Last→kürzerer Zeitraum
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Mausposition: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Maximal 10 Einträge anzeigen
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Bei niedriger Last wird im 2-Sekunden-Intervall gefiltert (Stromsparmodus)
- Bei hoher Last wird im 500ms-Intervall fein gesampelt
- Der Zeitraum kann dynamisch je nach Last angepasst werden.

## 🎯 Praktisches Codebeispiel 2: Zeitraumsteuerung basierend auf anderen Streams

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map, startWith } from 'rxjs';

// UI erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('Intervall: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// Schiebereglerwert überwachen
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Click-Ereignis
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Schiebereglerwert aktualisieren
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Klicks mit audit steuern
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Klick aufgezeichnet (Intervall: ${currentInterval}ms)`;
  output.insertBefore(log, output.firstChild);
});
```

## ⚠️ Wichtige Hinweise

### 1. Der erste Wert wird nicht sofort ausgegeben

`audit` wartet nach Empfang des ersten Werts, bis der Zeitraum endet.

```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// Ausgabe:
// 9  (nach 1 Sekunde, letzter Wert von 0-9)
// 19 (nach 2 Sekunden, letzter Wert von 10-19)
// 29 (nach 3 Sekunden, letzter Wert von 20-29)
```

### 2. Duration Observable wird jedes Mal neu erstellt

Die an `audit` übergebene Funktion **muss jedes Mal ein neues Observable zurückgeben**.

```ts
// ❌ Schlechtes Beispiel: Wiederverwendung derselben Observable-Instanz
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // Funktioniert ab dem 2. Mal nicht
).subscribe();

// ✅ Gutes Beispiel: Jedes Mal neues Observable erstellen
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. Speicher und Leistung

Bei Verwendung von `audit` in Streams mit häufigen Werten wird Speicher verbraucht.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// Schneller Stream (alle 10ms)
interval(10).pipe(
  audit(() => timer(1000)) // Sampling alle 1 Sekunde
).subscribe();
// 100 Werte akkumulieren sich innerhalb 1 Sekunde im Speicher, nur der letzte wird ausgegeben
```

## 🆚 Vergleich mit ähnlichen Operatoren

| Operator | Ausgabezeitpunkt | Ausgegebener Wert | Anwendungsfall |
|:---|:---|:---|:---|
| `audit` | Bei Zeitraum**ende** | **Letzter** Wert des Zeitraums | Aktuellen Zustand im Zeitraum abrufen |
| `throttle` | Bei Zeitraum**beginn** | **Erster** Wert des Zeitraums | Ersten Wert kontinuierlicher Ereignisse abrufen |
| `debounce` | **Nach Ruhe** | Wert direkt vor Ruhe | Auf Eingabeabschluss warten |
| `sample` | **Bei Auslösung eines anderen Observable** | Aktuellster Wert zu diesem Zeitpunkt | Regelmäßiger Snapshot |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: Letzter Klick in 1 Sekunde
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: letzter'));

// throttle: Erster Klick in 1 Sekunde
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: erster'));

// debounce: 1 Sekunde nach Klick-Stopp
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: nach Stopp'));

// sample: Sampling alle 1 Sekunde
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: regelmäßig'));
```

## 📚 Verwandte Operatoren

- **[auditTime](./auditTime)** - Steuerung mit fester Zeit (vereinfachte Version von `audit`)
- **[throttle](./throttleTime)** - Gibt den ersten Wert zu Beginn des Zeitraums aus
- **[debounce](./debounceTime)** - Gibt Wert nach Ruhe aus
- **[sample](./sampleTime)** - Sampling zum Zeitpunkt eines anderen Observable

## Zusammenfassung

Der `audit`-Operator gibt den letzten Wert innerhalb eines durch ein benutzerdefiniertes Observable dynamisch gesteuerten Zeitraums aus.

- ✅ Dynamische Zeitraumsteuerung möglich
- ✅ Adaptives Sampling je nach Last
- ✅ Steuerung basierend auf anderen Streams
- ⚠️ Muss jedes Mal ein neues Observable erstellen
- ⚠️ Bei häufigen Ausgaben auf Speicher achten
