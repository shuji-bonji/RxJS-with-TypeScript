---
description: "bufferWhen ist ein RxJS-Transformationsoperator, der die Endbedingung dynamisch steuert und Werte als Array ausgibt. Er ermöglicht kontinuierliche Pufferung, wobei der nächste Puffer unmittelbar nach dem Ende des vorherigen beginnt - ideal für adaptive Stapelverarbeitung und lastabhängige Datenaggregation."
---

# bufferWhen - Dynamischer Puffer mit Endkontrolle

Der `bufferWhen`-Operator **steuert die Endbedingung dynamisch** und gibt Werte als Array aus. Er realisiert ein kontinuierliches Pufferungsmuster, bei dem sofort nach Ende eines Puffers der nächste beginnt.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(500); // Gibt alle 0,5 Sekunden einen Wert aus

// Endbedingung: nach 1 Sekunde
const closingSelector = () => interval(1000);

source$.pipe(
  bufferWhen(closingSelector),
  take(4)
).subscribe(console.log);
// Ausgabe:
// [0]           (Start 0s → Ende 1s, nur Wert 0)
// [1, 2, 3]     (Start 1s → Ende 2s, Werte 1,2,3)
// [4, 5]        (Start 2s → Ende 3s, Werte 4,5)
// [6, 7]        (Start 3s → Ende 4s, Werte 6,7)
```

**Ablauf der Operation**:
1. Der erste Puffer startet automatisch
2. Das von `closingSelector()` zurückgegebene Observable gibt einen Wert aus → Puffer endet, Array wird ausgegeben
3. **Sofortiger Start des nächsten Puffers** (oft gleichzeitig mit der Ausgabe von source$)
4. Schritte 2-3 wiederholen sich

> [!NOTE]
> Der erste Puffer enthält nur `[0]`, da er für die eine Sekunde gilt, bis `interval(1000)` den ersten Wert ausgibt. Ab dem zweiten Puffer beginnen Pufferstart und `source$`-Ausgabe gleichzeitig, daher enthalten sie mehr Werte.

[🌐 Offizielle RxJS-Dokumentation - `bufferWhen`](https://rxjs.dev/api/operators/bufferWhen)


## 🆚 Unterschied zu bufferToggle

`bufferWhen` und `bufferToggle` sind ähnlich, aber **ihre Kontrollmethoden und Verhaltensmuster unterscheiden sich erheblich**.

### bufferWhen Verhalten

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Gibt 0-11 alle 300ms aus

// bufferWhen: Steuert nur das Ende (nächster startet sofort nach Ende)
source$.pipe(
  bufferWhen(() => interval(1000))
).subscribe(console.log);
// Ausgabe: [0, 1, 2], [3, 4, 5], [6, 7, 8, 9], [10, 11]
//
// Zeitachse:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms 3600ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//  [----------1 Sek----------][----------1 Sek----------][----------1 Sek----------][-----1 Sek-----]
//   Puffer1(0-2)               Puffer2(3-5)               Puffer3(6-9)              Puffer4(10-11)
//   Kontinuierlich, keine Überlappung, nächster startet sofort
```

### bufferToggle Verhalten

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Gibt 0-11 alle 300ms aus

// bufferToggle: Separate Kontrolle von Start und Ende (Überlappung möglich)
const opening$ = interval(1000); // Start jede Sekunde
const closing = () => interval(800); // Ende 800ms nach Start

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Ausgabe: [3, 4, 5], [6, 7, 8], [9, 10, 11]
//
// Zeitachse:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//        ----Start1(1000ms)----[---Ende 800ms später(1800ms)---]
//                        3      4      5
//                        └→ [3,4,5]
//                    ----Start2(2000ms)----[---Ende 800ms später(2800ms)---]
//                                            6      7      8
//                                            └→ [6,7,8]
//                              ----Start3(3000ms)----[---Ende 800ms später(3800ms)---]
//                                                      9      10     11
//                                                      └→ [9,10,11]
//  Wartet auf Start-Trigger, Perioden sind unabhängig (0-2 nicht enthalten da vor Pufferstart)
```

### Hauptunterschiede

| Operator | Startkontrolle | Endkontrolle | Pufferperiode | Merkmal |
|---|---|---|---|---|
| `bufferWhen(closing)` | Automatisch (sofort nach Ende) | Dynamisch | Kontinuierlich | Keine Lücken zwischen Puffern |
| `bufferToggle(open$, close)` | Unabhängiges Observable | Dynamisch | Unabhängig, überlappbar | Lücken zwischen Puffern |

**Verwendungshinweise**:
- **`bufferWhen`**: Alle Daten kontinuierlich und lückenlos puffern (Protokollsammlung, Datenaggregation usw.)
- **`bufferToggle`**: Daten nur für bestimmte Zeiträume sammeln (während Geschäftszeiten, bei Tastendruck usw.)

> [!TIP]
> - **Kontinuierliche Pufferung** (keine Datenverluste) → `bufferWhen`
> - **Zeitlich begrenzte Pufferung** (Start/Ende explizit steuern) → `bufferToggle`


## 💡 Typische Verwendungsmuster

1. **Datenerfassung mit dynamischen Zeitintervallen**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // Sensordaten
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       temperature: 20 + Math.random() * 10
     }))
   );

   // Endbedingung: Dynamisch basierend auf vorheriger Temperatur
   let previousAvgTemp = 25;

   sensorData$.pipe(
     bufferWhen(() => {
       // Je höher die Temperatur, desto kürzere Pufferintervalle
       const duration = previousAvgTemp > 27 ? 500 : 1000;
       return timer(duration);
     })
   ).subscribe(data => {
     const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
     previousAvgTemp = avgTemp;
     console.log(`Durchschnittstemperatur: ${avgTemp.toFixed(1)}°C, Anzahl Samples: ${data.length}`);
   });
   ```

2. **Adaptive Stapelverarbeitung basierend auf Last**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   interface Task {
     id: number;
     timestamp: number;
   }

   // Task-Stream
   let taskCounter = 0;
   const tasks$ = fromEvent(document, 'click').pipe(
     map(() => ({
       id: taskCounter++,
       timestamp: Date.now()
     } as Task))
   );

   // Nächste Pufferperiode basierend auf Puffergröße anpassen
   tasks$.pipe(
     bufferWhen(() => timer(2000))
   ).subscribe(bufferedTasks => {
     if (bufferedTasks.length > 0) {
       console.log(`Stapelverarbeitung: ${bufferedTasks.length} Tasks`);
       console.log('Task-IDs:', bufferedTasks.map(t => t.id));

       // Nächste Pufferperiode dynamisch bestimmen
       // (Diese Logik sollte eigentlich in die bufferWhen-Funktion verschoben werden)
     }
   });
   ```

3. **Sampling mit zufälligen Intervallen**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // Datenstream
   const data$ = interval(100).pipe(
     map(i => ({
       value: Math.sin(i / 10) * 100,
       timestamp: Date.now()
     }))
   );

   // Pufferung mit zufälligen Intervallen (500ms-2000ms)
   data$.pipe(
     bufferWhen(() => {
       const randomDelay = 500 + Math.random() * 1500;
       return timer(randomDelay);
     })
   ).subscribe(samples => {
     const avg = samples.reduce((sum, s) => sum + s.value, 0) / samples.length;
     console.log(`Anzahl Samples: ${samples.length}, Durchschnitt: ${avg.toFixed(2)}`);
   });
   ```


## 🧠 Praktisches Codebeispiel (Lastabhängige Protokollsammlung)

Ein Beispiel, bei dem die Häufigkeit der Protokollsammlung dynamisch basierend auf der Systemlast geändert wird.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { bufferWhen, map, share } from 'rxjs';

// UI-Elemente erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Adaptives Protokollsammelsystem';
container.appendChild(title);

const loadButton = document.createElement('button');
loadButton.textContent = 'Last erzeugen';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#f0f0f0';
status.textContent = 'Niedrige Last: Sammlung alle 5 Sekunden';
container.appendChild(status);

const logDisplay = document.createElement('pre');
logDisplay.style.marginTop = '10px';
logDisplay.style.padding = '10px';
logDisplay.style.backgroundColor = '#f9f9f9';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// Protokollstream (kontinuierlich generiert)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    message: `Log message ${logCounter}`,
    timestamp: new Date()
  })),
  share()
);

// Lastzähler (erhöht durch Buttonklick)
let loadLevel = 0;
fromEvent(loadButton, 'click').subscribe(() => {
  loadLevel = Math.min(loadLevel + 1, 5);
  updateStatus();
});

// Last alle 30 Sekunden verringern
interval(30000).subscribe(() => {
  loadLevel = Math.max(loadLevel - 1, 0);
  updateStatus();
});

function updateStatus() {
  const interval = getBufferInterval(loadLevel);
  const loadText = loadLevel === 0 ? 'Niedrige Last' :
                   loadLevel <= 2 ? 'Mittlere Last' : 'Hohe Last';
  status.textContent = `${loadText} (Level ${loadLevel}): Sammlung alle ${interval / 1000} Sekunden`;
  status.style.backgroundColor =
    loadLevel === 0 ? '#d4edda' :
    loadLevel <= 2 ? '#fff3cd' : '#f8d7da';
}

function getBufferInterval(load: number): number {
  // Je höher die Last, desto kürzere Pufferintervalle
  switch (load) {
    case 0: return 5000;  // 5 Sekunden
    case 1: return 3000;  // 3 Sekunden
    case 2: return 2000;  // 2 Sekunden
    case 3: return 1000;  // 1 Sekunde
    case 4: return 500;   // 0,5 Sekunden
    default: return 300;  // 0,3 Sekunden
  }
}

// Adaptive Pufferung
logs$.pipe(
  bufferWhen(() => timer(getBufferInterval(loadLevel)))
).subscribe(bufferedLogs => {
  if (bufferedLogs.length > 0) {
    const errors = bufferedLogs.filter(log => log.level === 'ERROR').length;
    const timestamp = new Date().toLocaleTimeString();

    const summary = `[${timestamp}] Gesammelt: ${bufferedLogs.length} Einträge (Fehler: ${errors})\n`;
    logDisplay.textContent = summary + logDisplay.textContent;

    console.log('Gesammelte Protokolle:', bufferedLogs);
  }
});
```


## 📋 Typsichere Verwendung

Ein Beispiel für typsichere Implementierung mit TypeScript-Generics.

```ts
import { Observable, interval, timer } from 'rxjs';
import { bufferWhen, map } from 'rxjs';

interface MetricData {
  value: number;
  timestamp: Date;
  source: string;
}

interface BufferConfig {
  minDuration: number;
  maxDuration: number;
  adaptive: boolean;
}

class AdaptiveBuffer<T> {
  constructor(private config: BufferConfig) {}

  private getNextBufferDuration(previousCount: number): number {
    if (!this.config.adaptive) {
      return this.config.minDuration;
    }

    // Nächste Pufferperiode basierend auf Datenmenge anpassen
    const ratio = Math.min(previousCount / 10, 1);
    const duration =
      this.config.minDuration +
      (this.config.maxDuration - this.config.minDuration) * (1 - ratio);

    return Math.floor(duration);
  }

  apply(source$: Observable<T>): Observable<T[]> {
    let previousCount = 0;

    return source$.pipe(
      bufferWhen(() => {
        const duration = this.getNextBufferDuration(previousCount);
        return timer(duration);
      }),
      map(buffer => {
        previousCount = buffer.length;
        return buffer;
      })
    );
  }
}

// Verwendungsbeispiel
const metricsStream$ = interval(300).pipe(
  map(i => ({
    value: Math.random() * 100,
    timestamp: new Date(),
    source: `sensor-${i % 3}`
  } as MetricData))
);

const buffer = new AdaptiveBuffer<MetricData>({
  minDuration: 1000,  // Minimum 1 Sekunde
  maxDuration: 5000,  // Maximum 5 Sekunden
  adaptive: true      // Adaptiv
});

buffer.apply(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avg = metrics.reduce((sum, m) => sum + m.value, 0) / metrics.length;
    console.log(`Puffergröße: ${metrics.length}, Durchschnitt: ${avg.toFixed(2)}`);
  }
});
```


## 🎯 Vergleich mit anderen Puffer-Operatoren

```ts
import { interval, timer, Subject } from 'rxjs';
import { buffer, bufferTime, bufferCount, bufferWhen, bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0-9

// 1. buffer: Fester Trigger
const trigger$ = new Subject<void>();
source$.pipe(buffer(trigger$)).subscribe(console.log);
setInterval(() => trigger$.next(), 1000);
// Ausgabe: [0, 1, 2], [3, 4, 5], ... (beim Trigger-Timing)

// 2. bufferTime: Festes Zeitintervall
source$.pipe(bufferTime(1000)).subscribe(console.log);
// Ausgabe: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 3. bufferCount: Feste Anzahl
source$.pipe(bufferCount(3)).subscribe(console.log);
// Ausgabe: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 4. bufferWhen: Dynamische Endkontrolle (kontinuierlich)
source$.pipe(
  bufferWhen(() => timer(1000))
).subscribe(console.log);
// Ausgabe: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 5. bufferToggle: Unabhängige Kontrolle von Start und Ende (überlappbar)
const opening$ = interval(1000);
const closing = () => timer(800);
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Ausgabe: [3, 4, 5], [6, 7, 8]
```

| Operator | Trigger | Dynamische Kontrolle | Überlappung | Anwendungsfall |
|---|---|---|---|---|
| `buffer` | Externes Observable | ❌ | ❌ | Ereignisgesteuert |
| `bufferTime` | Feste Zeit | ❌ | ❌ | Periodische Aggregation |
| `bufferCount` | Feste Anzahl | ❌ | ❌ | Quantitative Verarbeitung |
| `bufferWhen` | Dynamisch (nur Ende) | ✅ | ❌ | Adaptive Stapelverarbeitung |
| `bufferToggle` | Dynamisch (Start und Ende) | ✅ | ✅ | Komplexe Periodenverwaltung |


## ⚠️ Häufige Fehler

> [!WARNING]
> Die Endbedingungsfunktion in `bufferWhen` **muss jedes Mal ein neues Observable zurückgeben**. Bei Rückgabe derselben Observable-Instanz funktioniert es nicht korrekt.

### Falsch: Gleiche Observable-Instanz zurückgeben

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ❌ Schlechtes Beispiel: Wiederverwendung derselben Observable-Instanz
const closingObservable = timer(1000);

source$.pipe(
  bufferWhen(() => closingObservable) // Funktioniert ab dem zweiten Mal nicht!
).subscribe(console.log);
// Nur der erste Puffer wird ausgegeben, danach keine Ausgabe mehr
```

### Richtig: Jedes Mal neues Observable zurückgeben

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ✅ Gutes Beispiel: Jedes Mal neues Observable generieren
source$.pipe(
  bufferWhen(() => timer(1000)) // Jedes Mal neuer timer
).subscribe(console.log);
// Ausgabe: [0, 1], [2, 3], [4, 5], ...
```

> [!IMPORTANT]
> Die `closingSelector`-Funktion wird **bei jedem Pufferende aufgerufen** und es wird erwartet, dass sie ein neues Observable zurückgibt.


## 🎓 Zusammenfassung

### Wann bufferWhen verwenden
- ✅ Wenn Sie die Endbedingung dynamisch steuern möchten
- ✅ Wenn kontinuierliche Pufferungsperioden benötigt werden
- ✅ Wenn Sie die nächste Periode basierend auf vorherigen Pufferergebnissen anpassen möchten
- ✅ Wenn Sie adaptive Stapelverarbeitung implementieren möchten

### Wann bufferToggle verwenden
- ✅ Wenn Sie Start und Ende unabhängig steuern möchten
- ✅ Wenn Pufferperioden sich überlappen können
- ✅ Wenn klare Start/Ende-Ereignisse vorliegen, z.B. bei Tastendruck

### Wann bufferTime verwenden
- ✅ Wenn Pufferung mit festen Zeitintervallen ausreicht
- ✅ Wenn eine einfache Implementierung erforderlich ist

### Hinweise
- ⚠️ `closingSelector` muss jedes Mal ein neues Observable zurückgeben
- ⚠️ Zu komplexe Endbedingungen erschweren das Debugging
- ⚠️ Bei adaptiver Steuerung sind Tests wichtig, um unerwartetes Verhalten zu vermeiden


## 🚀 Nächste Schritte

- **[buffer](./buffer)** - Grundlegende Pufferung lernen
- **[bufferTime](./bufferTime)** - Zeitbasierte Pufferung lernen
- **[bufferCount](./bufferCount)** - Anzahlbasierte Pufferung lernen
- **[bufferToggle](./bufferToggle)** - Pufferung mit unabhängiger Start/Ende-Kontrolle lernen
- **[Praktische Beispiele für Transformationsoperatoren](./practical-use-cases)** - Reale Anwendungsfälle lernen
