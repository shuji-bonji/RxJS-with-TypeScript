---
description: "Der windowWhen-Operator teilt Observables durch dynamische Steuerung der Beendigungsbedingungen. Da das nächste Fenster unmittelbar nach dem Ende des vorherigen Fensters beginnt, eignet er sich optimal für kontinuierliche Datensegmentierung."
---

# windowWhen - Dynamisches Fenster mit Beendigungssteuerung

Der `windowWhen`-Operator teilt Observables durch **dynamische Steuerung der Beendigungsbedingungen**. Er realisiert ein kontinuierliches Stream-Verarbeitungsmuster, bei dem unmittelbar nach dem Ende eines Fensters das nächste Fenster geöffnet wird.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500); // Werte alle 0,5 Sekunden ausgeben

// Beendigungsbedingung: nach 1 Sekunde
const closingSelector = () => interval(1000);

source$.pipe(
  windowWhen(closingSelector),
  take(4),
  mergeAll()
).subscribe(value => {
  console.log('Wert im Fenster:', value);
});

// Fenster 1: 0       (Start 0s → Ende 1s)
// Fenster 2: 1, 2    (Start 1s → Ende 2s)
// Fenster 3: 3, 4    (Start 2s → Ende 3s)
// Fenster 4: 5, 6    (Start 3s → Ende 4s)
```

**Ablauf der Funktionsweise**:
1. Das erste Fenster wird automatisch geöffnet
2. Das von `closingSelector()` zurückgegebene Observable gibt einen Wert aus → Fenster wird geschlossen
3. **Unmittelbar danach wird das nächste Fenster geöffnet**
4. Schritte 2-3 werden wiederholt

[🌐 Offizielle RxJS-Dokumentation - `windowWhen`](https://rxjs.dev/api/operators/windowWhen)

## 💡 Typische Anwendungsmuster

- Datenerfassung mit dynamischen Zeitintervallen
- Adaptive Stream-Verarbeitung je nach Last
- Fenstersteuerung basierend auf vorherigen Ergebnissen
- Kontinuierliche Datengruppierung

## 🔍 Unterschied zu bufferWhen

| Operator | Ausgabe | Anwendungsfall |
|:---|:---|:---|
| `bufferWhen` | **Array (T[])** | Gruppierte Werte zusammen verarbeiten |
| `windowWhen` | **Observable\\<T>** | Unterschiedliche Stream-Verarbeitung pro Gruppe |

```ts
import { interval } from 'rxjs';
import { bufferWhen, windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500);
const closing = () => interval(1000);

// bufferWhen - Ausgabe als Array
source$.pipe(
  bufferWhen(closing),
  take(3)
).subscribe(values => {
  console.log('Puffer (Array):', values);
  // Ausgabe: Puffer (Array): [0]
  // Ausgabe: Puffer (Array): [1, 2]
  // Ausgabe: Puffer (Array): [3, 4]
});

// windowWhen - Ausgabe als Observable
source$.pipe(
  windowWhen(closing),
  take(3),
  mergeAll()
).subscribe(value => {
  console.log('Wert im Fenster:', value);
  // Ausgabe: Wert im Fenster: 0
  // Ausgabe: Wert im Fenster: 1
  // Ausgabe: Wert im Fenster: 2
  // ...
});
```

## 🧠 Praxisbeispiel 1: Datenerfassung mit dynamischen Zeitintervallen

Ein Beispiel, bei dem die Dauer des nächsten Fensters basierend auf den Ergebnissen des vorherigen Fensters angepasst wird.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, scan, map } from 'rxjs';

// Sensordaten (kontinuierlich generiert)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10 // 20-30 Grad
  }))
);

let windowNumber = 0;
let previousAvgTemp = 25;

sensorData$.pipe(
  windowWhen(() => {
    const current = ++windowNumber;
    // Je höher die Temperatur, desto kürzer das Intervall
    const duration = previousAvgTemp > 27 ? 500 : 1000;
    console.log(`Fenster ${current} geöffnet (Dauer: ${duration}ms)`);
    return timer(duration);
  }),
  mergeMap(window$ => {
    const currentWindow = windowNumber;  // Aktuelle Fensternummer speichern
    return window$.pipe(
      toArray(),
      map(data => {
        const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
        previousAvgTemp = avgTemp;
        return {
          window: currentWindow,  // Gespeicherte Fensternummer verwenden
          count: data.length,
          avgTemp
        };
      })
    );
  })
).subscribe(stats => {
  console.log(`Fenster ${stats.window}: Durchschnittstemperatur ${stats.avgTemp.toFixed(1)}°C, ${stats.count} Samples`);
});
```

## 🎯 Praxisbeispiel 2: Adaptive Stream-Verarbeitung nach Systemlast

Ein Beispiel, bei dem die Fensterlänge dynamisch entsprechend der Systemlast geändert wird.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { windowWhen, mergeMap, scan, map } from 'rxjs';

// Ausgabebereich erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const loadButton = document.createElement('button');
loadButton.textContent = 'Last erzeugen';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Niedrige Last: Erfassung in 5-Sekunden-Intervallen';
container.appendChild(status);

const logDisplay = document.createElement('div');
logDisplay.style.marginTop = '10px';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// Log-Stream (kontinuierlich generiert)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    timestamp: new Date()
  }))
);

// Lastlevel
let loadLevel = 0;
fromEvent(loadButton, 'click').subscribe(() => {
  loadLevel = Math.min(loadLevel + 1, 5);
  updateStatus();
});

// Last alle 30 Sekunden reduzieren
interval(30000).subscribe(() => {
  loadLevel = Math.max(loadLevel - 1, 0);
  updateStatus();
});

function updateStatus() {
  const interval = getWindowDuration(loadLevel);
  const loadText = loadLevel === 0 ? 'Niedrige Last' :
                   loadLevel <= 2 ? 'Mittlere Last' : 'Hohe Last';
  status.textContent = `${loadText} (Level ${loadLevel}): Erfassung in ${interval / 1000}-Sekunden-Intervallen`;
}

function getWindowDuration(load: number): number {
  // Je höher die Last, desto kürzer das Intervall
  switch (load) {
    case 0: return 5000;
    case 1: return 3000;
    case 2: return 2000;
    case 3: return 1000;
    case 4: return 500;
    default: return 300;
  }
}

let windowNum = 0;

// Adaptive Fensterverarbeitung
logs$.pipe(
  windowWhen(() => {
    windowNum++;
    return timer(getWindowDuration(loadLevel));
  }),
  mergeMap(window$ =>
    window$.pipe(
      scan((stats, log) => ({
        count: stats.count + 1,
        errors: stats.errors + (log.level === 'ERROR' ? 1 : 0),
        window: windowNum
      }), { count: 0, errors: 0, window: windowNum })
    )
  )
).subscribe(stats => {
  const timestamp = new Date().toLocaleTimeString();
  const div = document.createElement('div');
  div.textContent = `[${timestamp}] Fenster ${stats.window}: ${stats.count} Einträge (Fehler: ${stats.errors})`;
  logDisplay.insertBefore(div, logDisplay.firstChild);
});
```

## 🆚 Unterschied zu windowToggle

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowWhen: Nur Beendigung steuern (nächstes startet sofort nach Ende)
source$.pipe(
  windowWhen(() => timer(1000)),
  mergeAll()
).subscribe();

// windowToggle: Start und Ende separat steuern
source$.pipe(
  windowToggle(
    interval(1000),          // Start-Trigger
    () => timer(500)         // End-Trigger (500ms nach Start)
  ),
  mergeAll()
).subscribe();
```

| Operator | Steuerung | Fensterzeitraum | Anwendungsfall |
|:---|:---|:---|:---|
| `windowWhen(closing)` | Nur Beendigung | Kontinuierlich | Einfache periodische Fenster |
| `windowToggle(open$, close)` | Start und Ende separat | Überlappung möglich | Komplexe Start-/End-Bedingungen |

**Entscheidungskriterien**:
- **`windowWhen`**: Alle Daten kontinuierlich und lückenlos verarbeiten (Log-Erfassung, Datenaggregation usw.)
- **`windowToggle`**: Daten nur während bestimmter Zeiträume verarbeiten (Geschäftszeiten, während Tastendruck usw.)

## 🎯 Praktisches Beispiel: Adaptive Fenstergrößensteuerung

Ein Beispiel, bei dem die nächste Fensterdauer automatisch basierend auf den Ergebnissen des vorherigen Fensters angepasst wird.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, map } from 'rxjs';

interface WindowStats {
  count: number;
  nextDuration: number;
}

const data$ = interval(100);

let previousCount = 0;

// Nächste Fensterdauer entsprechend der Datenmenge anpassen
function getNextDuration(count: number): number {
  if (count > 20) {
    return 500;  // Viele Daten → kurzes Intervall
  } else if (count > 10) {
    return 1000; // Mittlere Menge → mittleres Intervall
  } else {
    return 2000; // Wenige Daten → langes Intervall
  }
}

data$.pipe(
  windowWhen(() => timer(getNextDuration(previousCount))),
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(data => {
        previousCount = data.length;
        return {
          count: data.length,
          nextDuration: getNextDuration(data.length)
        } as WindowStats;
      })
    )
  )
).subscribe(stats => {
  console.log(`Fenstergröße: ${stats.count} Einträge, nächste Dauer: ${stats.nextDuration}ms`);
});
```

## ⚠️ Wichtige Hinweise

### 1. Subscription-Verwaltung für Fenster

Jedes Fenster ist ein eigenständiges Observable, daher muss es entweder explizit abonniert oder mit `mergeAll()` o.ä. abgeflacht werden.

```ts
source$.pipe(
  windowWhen(closing)
).subscribe(window$ => {
  // Ohne Abonnieren des Fensters selbst fließen keine Werte
  window$.subscribe(value => {
    console.log('Wert:', value);
  });
});
```

### 2. Jedes Mal ein neues Observable zurückgeben

Die `closingSelector`-Funktion **muss jedes Mal ein neues Observable zurückgeben**. Wenn dieselbe Instanz zurückgegeben wird, funktioniert es nicht richtig.

```ts
// ❌ Schlechtes Beispiel: Dieselbe Observable-Instanz wiederverwenden
const closingObservable = timer(1000);

source$.pipe(
  windowWhen(() => closingObservable) // Funktioniert ab dem zweiten Mal nicht mehr!
).subscribe();

// ✅ Gutes Beispiel: Jedes Mal ein neues Observable erzeugen
source$.pipe(
  windowWhen(() => timer(1000)) // Jedes Mal einen neuen Timer erzeugen
).subscribe();
```

### 3. Vorsicht bei komplexen Beendigungsbedingungen

Wenn die Beendigungsbedingungen zu komplex werden, wird das Debugging schwierig.

```ts
// Zu komplexes Beispiel
let counter = 0;
source$.pipe(
  windowWhen(() => {
    counter++;
    const duration = counter % 3 === 0 ? 500 :
                     counter % 2 === 0 ? 1000 : 1500;
    return timer(duration);
  })
).subscribe();
// Debugging wird schwierig
```

## 🆚 Vergleich der window-Operatoren

| Operator | Steuerung | Fensterzeitraum | Anwendungsfall |
|:---|:---|:---|:---|
| `window` | Ausgabe eines anderen Observable | Kontinuierlich | Ereignisgesteuerte Aufteilung |
| `windowTime` | Feste Zeitdauer | Kontinuierlich | Zeitbasierte Aufteilung |
| `windowCount` | Feste Anzahl | Kontinuierlich | Anzahlbasierte Aufteilung |
| `windowToggle` | Start und Ende separat | Überlappung möglich | Komplexe Start-/End-Bedingungen |
| `windowWhen` | **Nur Ende dynamisch** | **Kontinuierlich** | **Adaptive Fensterverarbeitung** |

## 📚 Verwandte Operatoren

- [`bufferWhen`](./bufferWhen) - Werte als Array sammeln (Array-Version von windowWhen)
- [`window`](./window) - Fensteraufteilung zum Zeitpunkt eines anderen Observable
- [`windowTime`](./windowTime) - Zeitbasierte Fensteraufteilung
- [`windowCount`](./windowCount) - Anzahlbasierte Fensteraufteilung
- [`windowToggle`](./windowToggle) - Fenstersteuerung mit Start- und End-Observables

## Zusammenfassung

Der `windowWhen`-Operator ist ein nützliches Werkzeug zur dynamischen Steuerung von Beendigungsbedingungen und zur Realisierung kontinuierlicher Fensterverarbeitung.

- ✅ Beendigungsbedingungen können dynamisch gesteuert werden
- ✅ Kontinuierliche Fensterverarbeitung (ohne Datenverlust)
- ✅ Nächstes Fenster kann basierend auf vorherigen Ergebnissen angepasst werden
- ⚠️ Subscription-Verwaltung erforderlich
- ⚠️ Jedes Mal muss ein neues Observable zurückgegeben werden
- ⚠️ Beendigungsbedingungen sollten nicht zu komplex werden
