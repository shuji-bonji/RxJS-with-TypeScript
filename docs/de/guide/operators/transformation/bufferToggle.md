---
description: Der bufferToggle-Operator ist ein fortgeschrittener Pufferungsoperator, der Start- und End-Trigger mit separaten Observables steuert und mehrere Pufferungszeiträume unabhängig verwalten kann.
---

# bufferToggle - Puffer mit unabhängiger Start- und End-Steuerung

Der `bufferToggle`-Operator steuert **Start-Trigger** und **End-Trigger** mit separaten Observables und emittiert Werte als Array zusammengefasst. Ein fortgeschrittener Pufferungsoperator, der mehrere Pufferungszeiträume gleichzeitig verwalten kann.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // Werte alle 0,5 Sekunden emittieren

// Start-Trigger: alle 2 Sekunden
const opening$ = interval(2000);

// End-Trigger: 1 Sekunde nach Start
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Ausgabe:
// [3, 4, 5]     (Start bei 2 Sek., Ende bei 3 Sek.)
// [7, 8, 9]     (Start bei 4 Sek., Ende bei 5 Sek.)
// [11, 12, 13]  (Start bei 6 Sek., Ende bei 7 Sek.)
```

**Ablauf**:
1. `opening$` emittiert Wert → Pufferung startet
2. Observable, das `closing()` zurückgibt, emittiert Wert → Pufferung endet, Array wird ausgegeben
3. Mehrere Pufferungszeiträume können sich überlappen

[🌐 RxJS Offizielle Dokumentation - `bufferToggle`](https://rxjs.dev/api/operators/bufferToggle)


## 🆚 Vergleich mit anderen Puffer-Operatoren

`bufferToggle` zeichnet sich durch **unabhängige Steuerung von Start und Ende** im Vergleich zu anderen Puffer-Operatoren aus.

### Vergleich der Operatoren

| Operator | Trigger | Merkmal | Anwendungsfall |
|---|---|---|---|
| `buffer(trigger$)` | Einzelnes Observable | Einfach | Ereignisgesteuertes Puffern |
| `bufferTime(ms)` | Zeit | Periodisch | Datenaggregation in festen Intervallen |
| `bufferCount(n)` | Anzahl | Quantitativ | Verarbeitung in N-Einheiten |
| `bufferToggle(open$, close)` | Start und Ende separat steuerbar | Flexibel | Komplexe Zeitraumverwaltung |

### Vergleich mit Codebeispiel

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0-9 alle 300ms emittieren

// bufferToggle: Start und Ende unabhängig steuern
const opening$ = interval(1000); // Start jede Sekunde
const closing = () => interval(500); // Ende 500ms nach Start

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Ausgabe: [3, 4], [6, 7], [9]
//
// Timeline:
// 0ms  300ms 600ms 900ms 1200ms 1500ms 1800ms 2100ms 2400ms 2700ms
// 0    1     2     3     4      5      6      7      8      9
//                  [Start       Ende]  [Start       Ende]  [Start Ende]
//                  └→ [3,4]           └→ [6,7]           └→ [9]
```

**Unterscheidung von anderen Operatoren**:
- **`buffer`** → Gibt Puffer bei jeder Emission des Trigger-Observables aus
- **`bufferTime`** → Gibt Puffer automatisch in festen Zeitintervallen aus
- **`bufferCount`** → Gibt Puffer aus, wenn angegebene Anzahl erreicht ist
- **`bufferToggle`** → Separate Steuerung von Start und Ende, Überlappung möglich

> [!TIP]
> Details zu jedem Operator finden Sie unter [buffer](./buffer), [bufferTime](./bufferTime), [bufferCount](./bufferCount).


## 💡 Typische Anwendungsmuster

1. **Datensammlung während Geschäftszeiten**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // Sensordaten (kontinuierliche Erfassung)
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       value: Math.random() * 100
     }))
   );

   // Geschäftsbeginn: 9:00 (Simulation: nach 2 Sek.)
   const businessOpen$ = timer(2000, 10000); // Nach 2 Sek., dann alle 10 Sek.

   // Geschäftsende: 5 Sekunden nach Start
   const businessClose = () => timer(5000);

   sensorData$.pipe(
     bufferToggle(businessOpen$, businessClose)
   ).subscribe(data => {
     console.log(`Daten während Geschäftszeit: ${data.length} Stück`);
     console.log(`Durchschnitt: ${(data.reduce((sum, d) => sum + d.value, 0) / data.length).toFixed(2)}`);
   });
   ```

2. **Ereignisaufzeichnung während Button-Druck**
   ```ts
   import { fromEvent, interval } from 'rxjs';
   import { bufferToggle, map, take } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Halten';
   document.body.appendChild(button);

   const display = document.createElement('div');
   display.style.marginTop = '10px';
   document.body.appendChild(display);

   // Datenstrom
   const data$ = interval(100).pipe(
     map(i => ({ id: i, timestamp: Date.now() }))
   );

   // Start: Mousedown
   const mouseDown$ = fromEvent(button, 'mousedown');

   // Ende: Mouseup (bis mouseup nach mousedown auftritt)
   const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

   data$.pipe(
     bufferToggle(mouseDown$, mouseUp)
   ).subscribe(events => {
     display.textContent = `Während Halten aufgezeichnete Ereignisse: ${events.length} Stück`;
     console.log('Aufgezeichnete Ereignisse:', events);
   });
   ```

3. **Aktionsaufzeichnung aktiver Benutzer**
   ```ts
   import { fromEvent, merge, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // Benutzeraktionen
   const clicks$ = fromEvent(document, 'click').pipe(
     map(() => ({ type: 'click' as const, timestamp: Date.now() }))
   );

   const scrolls$ = fromEvent(window, 'scroll').pipe(
     map(() => ({ type: 'scroll' as const, timestamp: Date.now() }))
   );

   const keypresses$ = fromEvent(document, 'keypress').pipe(
     map(() => ({ type: 'keypress' as const, timestamp: Date.now() }))
   );

   const actions$ = merge(clicks$, scrolls$, keypresses$);

   // Aktiv-Status Start: erste Aktion
   const activeStart$ = actions$;

   // Aktiv-Status Ende: 5 Sekunden keine Aktion
   const activeEnd = () => timer(5000);

   actions$.pipe(
     bufferToggle(activeStart$, activeEnd)
   ).subscribe(bufferedActions => {
     console.log(`Aktive Sitzung: ${bufferedActions.length} Aktionen`);
     const summary = bufferedActions.reduce((acc, action) => {
       acc[action.type] = (acc[action.type] || 0) + 1;
       return acc;
     }, {} as Record<string, number>);
     console.log('Aufschlüsselung:', summary);
   });
   ```


## 🧠 Praktisches Codebeispiel (Download-Zeitraum-Verwaltung)

Beispiel zur Verwaltung von Daten-Download-Zeiträumen mit Start- und Stopp-Button.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { bufferToggle, map, take } from 'rxjs';

// UI-Elemente erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Daten-Download-Verwaltung';
container.appendChild(title);

const startButton = document.createElement('button');
startButton.textContent = 'Start';
container.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Stopp';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
container.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Bereit...';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// Datenstrom (Download-Daten jede Sekunde generieren)
const downloadData$ = interval(1000).pipe(
  map(i => ({
    id: i,
    size: Math.floor(Math.random() * 1000) + 100, // 100-1100KB
    timestamp: new Date()
  }))
);

// Start- und End-Trigger
const start$ = fromEvent(startButton, 'click');
const stop$ = new Subject<void>();

fromEvent(stopButton, 'click').subscribe(() => {
  stop$.next();
  status.textContent = 'Gestoppt';
  startButton.disabled = false;
  stopButton.disabled = true;
});

start$.subscribe(() => {
  status.textContent = 'Download läuft...';
  startButton.disabled = true;
  stopButton.disabled = false;
});

// Pufferung
downloadData$.pipe(
  bufferToggle(start$, () => stop$)
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Download abgeschlossen</strong><br>
    Anzahl: ${downloads.length} Stück<br>
    Gesamtgröße: ${(totalSize / 1024).toFixed(2)} MB<br>
    Durchschnittsgröße: ${avgSize.toFixed(0)} KB
  `;

  console.log('Download-Daten:', downloads);
});
```


## 🎯 Überlappende Pufferzeiträume

Ein Merkmal von `bufferToggle` ist, dass mehrere Pufferungszeiträume gleichzeitig verwaltet werden können.

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Start: jede Sekunde
const opening$ = interval(1000);

// Ende: 1,5 Sekunden nach Start
const closing = () => interval(1500);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Ausgabe:
// [4, 5, 6]        (Start 1 Sek. → Ende 2,5 Sek.)
// [9, 10, 11, 12]  (Start 2 Sek. → Ende 3,5 Sek.) ※teilweise Überlappung
// [14, 15, 16, 17] (Start 3 Sek. → Ende 4,5 Sek.)
```

**Timeline**:
```
Quelle:    0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Start:     ----1Sek----2Sek----3Sek----4Sek
Zeitraum1: [------1,5Sek-----]
            └→ Ausgabe: [4,5,6]
Zeitraum2:        [------1,5Sek-----]
                   └→ Ausgabe: [9,10,11,12]
Zeitraum3:               [------1,5Sek-----]
                          └→ Ausgabe: [14,15,16,17]
```


## 📋 Typsichere Verwendung

Beispiel für typsichere Implementierung mit TypeScript-Generics.

```ts
import { Observable, Subject, interval } from 'rxjs';
import { bufferToggle, map } from 'rxjs';

interface MetricData {
  timestamp: Date;
  cpu: number;
  memory: number;
}

interface SessionControl {
  start$: Observable<void>;
  stop$: Observable<void>;
}

class MetricsCollector {
  private startSubject = new Subject<void>();
  private stopSubject = new Subject<void>();

  start(): void {
    this.startSubject.next();
  }

  stop(): void {
    this.stopSubject.next();
  }

  collectMetrics(source$: Observable<MetricData>): Observable<MetricData[]> {
    return source$.pipe(
      bufferToggle(
        this.startSubject,
        () => this.stopSubject
      )
    );
  }
}

// Verwendungsbeispiel
const metricsStream$ = interval(500).pipe(
  map(() => ({
    timestamp: new Date(),
    cpu: Math.random() * 100,
    memory: Math.random() * 100
  } as MetricData))
);

const collector = new MetricsCollector();

collector.collectMetrics(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avgCpu = metrics.reduce((sum, m) => sum + m.cpu, 0) / metrics.length;
    const avgMemory = metrics.reduce((sum, m) => sum + m.memory, 0) / metrics.length;
    console.log(`Sammelzeitraum: ${metrics.length} Stück`);
    console.log(`Durchschnittliche CPU: ${avgCpu.toFixed(1)}%`);
    console.log(`Durchschnittlicher Speicher: ${avgMemory.toFixed(1)}%`);
  }
});

// Nach 3 Sekunden starten
setTimeout(() => {
  console.log('Sammlung gestartet');
  collector.start();
}, 3000);

// Nach 6 Sekunden stoppen
setTimeout(() => {
  console.log('Sammlung gestoppt');
  collector.stop();
}, 6000);
```


## 🔄 Unterschied zu bufferWhen

`bufferToggle` und `bufferWhen` sind ähnlich, haben aber unterschiedliche Steuerungsmethoden.

```ts
import { interval, timer } from 'rxjs';
import { bufferToggle, bufferWhen } from 'rxjs';

const source$ = interval(200);

// bufferToggle: Start und Ende separat steuern
source$.pipe(
  bufferToggle(
    interval(1000),          // Start-Trigger
    () => timer(500)         // End-Trigger (500ms nach Start)
  )
).subscribe(console.log);

// bufferWhen: Nur End-Timing steuern (nächster Start sofort nach Ende)
source$.pipe(
  bufferWhen(() => timer(1000)) // Puffer jede Sekunde
).subscribe(console.log);
```

| Operator | Steuerung | Pufferzeitraum | Anwendungsfall |
|---|---|---|---|
| `bufferToggle(open$, close)` | Start und Ende separat | Überlappung möglich | Komplexe Start-/End-Bedingungen |
| `bufferWhen(closing)` | Nur Ende-Steuerung | Kontinuierlich | Einfache periodische Pufferung |


## ⚠️ Häufige Fehler

> [!WARNING]
> `bufferToggle` kann mehrere Pufferzeiträume gleichzeitig verwalten, aber wenn der Start-Trigger häufig ausgelöst wird, existieren viele Puffer gleichzeitig und verbrauchen Speicher.

### Falsch: Start-Trigger zu häufig

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ❌ Schlechtes Beispiel: Start alle 100ms, Ende nach 5 Sekunden
const opening$ = interval(100); // Zu häufig
const closing = () => interval(5000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Möglicherweise 50 Puffer gleichzeitig → Speicherrisiko
```

### Richtig: Angemessene Intervalle setzen

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ✅ Gutes Beispiel: Start in angemessenen Intervallen
const opening$ = interval(2000); // Alle 2 Sekunden
const closing = () => interval(1000); // 1 Sekunde Pufferung

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Maximal nur 1-2 Puffer gleichzeitig
```


## 🎓 Zusammenfassung

### Wann bufferToggle verwenden
- ✅ Wenn Start und Ende unabhängig gesteuert werden sollen
- ✅ Datensammlung für begrenzte Zeiträume wie beim Button-Druck
- ✅ Gleichzeitige Verwaltung mehrerer Pufferungszeiträume
- ✅ Datensammlung mit komplexen Bedingungen wie nur während Geschäftszeiten

### Wann buffer/bufferTime/bufferCount verwenden
- ✅ Wenn einfache periodische Pufferung ausreicht
- ✅ Wenn mit einzelnem Trigger steuerbar

### Wann bufferWhen verwenden
- ✅ Wenn nur End-Bedingung dynamisch gesteuert werden soll
- ✅ Wenn kontinuierliche Pufferungszeiträume benötigt werden

### Achtung
- ⚠️ Häufige Start-Trigger führen zu vielen gleichzeitigen Puffern und Speicherverbrauch
- ⚠️ Pufferungszeiträume können sich überlappen
- ⚠️ Komplexe Steuerung kann Debugging erschweren


## 🚀 Nächste Schritte

- **[buffer](./buffer)** - Grundlegende Pufferung lernen
- **[bufferTime](./bufferTime)** - Zeitbasierte Pufferung lernen
- **[bufferCount](./bufferCount)** - Anzahlbasierte Pufferung lernen
- **[bufferWhen](https://rxjs.dev/api/operators/bufferWhen)** - Dynamische End-Steuerung lernen (offizielle Dokumentation)
- **[Praxisbeispiele für Transformationsoperatoren](./practical-use-cases)** - Reale Anwendungsfälle lernen
