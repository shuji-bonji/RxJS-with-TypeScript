---
description: windowToggle ist ein erweiterter RxJS-Transformationsoperator, der Start- und End-Trigger über separate Observables steuert und mehrere Fensterperioden unabhängig verwaltet. Ideal für dynamische Zeitraumverwaltung wie Datenerfassung während Geschäftszeiten oder Ereignisaufzeichnung beim Drücken von Schaltflächen. Ermöglicht typsichere Fensteraufteilung durch TypeScript-Typinferenz.
---

# windowToggle - Fenster mit Trigger

Der Operator `windowToggle` steuert den **Start-Trigger** und den **End-Trigger** über separate Observables und gibt jeden Zeitraum als neues Observable aus. Es handelt sich um einen erweiterten Fensteroperator, der mehrere Fensterperioden gleichzeitig verwalten kann.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500); // Gibt alle 0,5 Sekunden Werte aus

// Start-Trigger: alle 2 Sekunden
const opening$ = interval(2000);

// End-Trigger: 1 Sekunde nach dem Start
const closing = () => interval(1000);

source$.pipe(
  windowToggle(opening$, closing),
  mergeAll()
).subscribe(value => {
  console.log('Wert innerhalb des Fensters:', value);
});

// Start bei 2 Sekunden, Ende bei 3 Sekunden → Werte: 4, 5
// Start bei 4 Sekunden, Ende bei 5 Sekunden → Werte: 8, 9
// Start bei 6 Sekunden, Ende bei 7 Sekunden → Werte: 12, 13
```

**Ablauf der Funktionsweise**:
1. `opening$` gibt einen Wert aus → Fenster startet
2. Das von `closing()` zurückgegebene Observable gibt einen Wert aus → Fenster endet
3. Es ist möglich, dass sich mehrere Fensterperioden überlappen

[🌐 Offizielle RxJS-Dokumentation - `windowToggle`](https://rxjs.dev/api/operators/windowToggle)

## 💡 Typische Anwendungsmuster

- Datenerfassung während der Geschäftszeiten
- Ereignisaufzeichnung beim Drücken von Schaltflächen
- Verfolgung von Aktionen während aktiver Sitzungen
- Stream-Verarbeitung mit dynamischer Zeitraumverwaltung

## 🔍 Unterschied zu bufferToggle

| Operator | Ausgabe | Anwendungsfall |
|:---|:---|:---|
| `bufferToggle` | **Array (T[])** | Verarbeitung gruppierter Werte zusammen |
| `windowToggle` | **Observable\<T>** | Unterschiedliche Stream-Verarbeitung für jede Gruppe |

```ts
import { interval } from 'rxjs';
import { bufferToggle, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500);
const opening$ = interval(2000);
const closing = () => interval(1000);

// bufferToggle - Ausgabe als Array
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(values => {
  console.log('Puffer (Array):', values);
  // Ausgabe: Puffer (Array): [4, 5]
});

// windowToggle - Ausgabe als Observable
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  console.log('Fenster (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Wert innerhalb des Fensters:', value);
  });
});
```

## 🧠 Praktisches Codebeispiel 1: Ereignisaufzeichnung beim Drücken von Schaltflächen

Ein Beispiel für die Aufzeichnung von Daten zwischen Maus-Down und Maus-Up.

```ts
import { fromEvent, interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

// Schaltfläche erstellen
const button = document.createElement('button');
button.textContent = 'Halten';
document.body.appendChild(button);

// Ausgabebereich
const display = document.createElement('div');
display.style.marginTop = '10px';
document.body.appendChild(display);

// Daten-Stream (alle 100 ms)
const data$ = interval(100);

// Start: Maus-Down
const mouseDown$ = fromEvent(button, 'mousedown');

// Ende: Maus-Up
const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

data$.pipe(
  windowToggle(mouseDown$, mouseUp),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(events => {
  display.textContent = `Während des Haltens aufgezeichnete Ereignisse: ${events.length} Stück`;
  console.log('Aufgezeichnete Daten:', events);
});
```

## 🎯 Praktisches Codebeispiel 2: Datenerfassung während der Geschäftszeiten

Ein Beispiel für das Sammeln von Sensordaten von der Geschäftseröffnung bis zum Geschäftsschluss.

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, mergeMap, scan, map } from 'rxjs';

// Sensordaten (kontinuierliche Erfassung)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10, // 20-30 Grad
    humidity: 40 + Math.random() * 20     // 40-60%
  }))
);

// Geschäftseröffnung: nach 2 Sekunden, dann alle 10 Sekunden
const businessOpen$ = timer(2000, 10000);

// Geschäftsschluss: 5 Sekunden nach dem Start
const businessClose = () => timer(5000);

let sessionNumber = 0;

sensorData$.pipe(
  windowToggle(businessOpen$, businessClose),
  mergeMap(window$ => {
    const current = ++sessionNumber;
    console.log(`Geschäftssitzung ${current} gestartet`);

    // Statistische Informationen für jedes Fenster berechnen
    return window$.pipe(
      scan((stats, data) => ({
        count: stats.count + 1,
        totalTemp: stats.totalTemp + data.temperature,
        totalHumidity: stats.totalHumidity + data.humidity
      }), { count: 0, totalTemp: 0, totalHumidity: 0 }),
      map(stats => ({
        session: current,
        count: stats.count,
        avgTemp: stats.totalTemp / stats.count,
        avgHumidity: stats.totalHumidity / stats.count
      }))
    );
  })
).subscribe(stats => {
  console.log(`Sitzung ${stats.session}: ${stats.count} Samples`);
  console.log(`  Durchschnittstemperatur: ${stats.avgTemp.toFixed(1)}°C`);
  console.log(`  Durchschnittliche Luftfeuchtigkeit: ${stats.avgHumidity.toFixed(1)}%`);
});
```

## 🎯 Praktisches Beispiel: Verwaltung des Download-Zeitraums

Ein Beispiel für die Verwaltung des Daten-Download-Zeitraums mit Start- und Stopp-Schaltflächen.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { windowToggle, mergeMap, toArray, map } from 'rxjs';

// UI-Elemente erstellen
const startButton = document.createElement('button');
startButton.textContent = 'Start';
document.body.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Stopp';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
document.body.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Bereit...';
document.body.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
document.body.appendChild(result);

// Daten-Stream (generiert alle 1 Sekunde Download-Daten)
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

// Fensterverwaltung
downloadData$.pipe(
  windowToggle(start$, () => stop$),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Download abgeschlossen</strong><br>
    Anzahl: ${downloads.length} Stück<br>
    Gesamtgröße: ${(totalSize / 1024).toFixed(2)} MB<br>
    Durchschnittsgröße: ${avgSize.toFixed(0)} KB
  `;
});
```

## 🎯 Überlappende Fensterperioden

Ein Merkmal von `windowToggle` ist, dass es mehrere Fensterperioden gleichzeitig verwalten kann.

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Start: jede Sekunde
const opening$ = interval(1000);

// Ende: 1,5 Sekunden nach dem Start
const closing = () => interval(1500);

source$.pipe(
  windowToggle(opening$, closing),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Fenster:', values);
});

// Ausgabe:
// Fenster: [4, 5, 6, 7]       (Start bei 1 Sekunde → Ende bei 2,5 Sekunden)
// Fenster: [9, 10, 11, 12]    (Start bei 2 Sekunden → Ende bei 3,5 Sekunden)
// Fenster: [14, 15, 16, 17]   (Start bei 3 Sekunden → Ende bei 4,5 Sekunden)
```

**Zeitachse**:
```
Quelle:    0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Start:     ----1Sek---2Sek---3Sek---4Sek
Periode1:  [------1,5Sek-----]
            └→ Fenster1: [4,5,6,7]
Periode2:         [------1,5Sek-----]
                   └→ Fenster2: [9,10,11,12]
Periode3:                [------1,5Sek-----]
                          └→ Fenster3: [14,15,16,17]
```

## ⚠️ Hinweise

### 1. Verwaltung von Fenster-Subscriptions

Da jedes Fenster ein unabhängiges Observable ist, müssen Sie es explizit abonnieren oder mit `mergeAll()` o.ä. abflachen.

```ts
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  // Werte fließen nicht, wenn das Fenster selbst nicht abonniert wird
  window$.subscribe(value => {
    console.log('Wert:', value);
  });
});
```

### 2. Achtung vor Speicherlecks

Wenn Start-Trigger zu häufig sind, existieren viele Fenster gleichzeitig und verbrauchen Speicher.

```ts
// ❌ Schlechtes Beispiel: Start alle 100 ms, Ende nach 5 Sekunden
const opening$ = interval(100); // Zu häufig
const closing = () => interval(5000);

source$.pipe(
  windowToggle(opening$, closing)
).subscribe();
// Es können bis zu 50 Fenster gleichzeitig existieren → Speicherrisiko

// ✅ Gutes Beispiel: Angemessenes Intervall festlegen
const opening$ = interval(2000); // Alle 2 Sekunden
const closing = () => interval(1000); // 1 Sekunde lang
```

### 3. Überlappung von Fensterperioden

Wenn sich Fensterperioden überlappen, sind dieselben Werte in mehreren Fenstern enthalten. Überprüfen Sie, ob dies das beabsichtigte Verhalten ist.

```ts
// Mit Überlappung
opening$ = interval(1000);    // Start jede Sekunde
closing = () => interval(1500); // 1,5 Sekunden lang

// Ohne Überlappung
opening$ = interval(2000);    // Start alle 2 Sekunden
closing = () => interval(1000); // 1 Sekunde lang
```

## 🆚 Vergleich der window-Operatoren

| Operator | Steuerung | Fensterperiode | Anwendungsfall |
|:---|:---|:---|:---|
| `window` | Ausgabe eines anderen Observables | Kontinuierlich | Ereignisgesteuerte Aufteilung |
| `windowTime` | Feste Zeitspanne | Kontinuierlich | Zeitbasierte Aufteilung |
| `windowCount` | Feste Anzahl | Kontinuierlich | Anzahlbasierte Aufteilung |
| `windowToggle` | **Separate Start-/Endsteuerung** | **Überlappung möglich** | **Komplexe Start-/Endbedingungen** |
| `windowWhen` | Nur Ende gesteuert | Kontinuierlich | Einfache periodische Steuerung |

## 🔄 Unterschied zu windowWhen

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, windowWhen, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowToggle: Start und Ende separat steuern
source$.pipe(
  windowToggle(
    interval(1000),          // Start-Trigger
    () => timer(500)         // End-Trigger (500 ms nach dem Start)
  ),
  mergeAll()
).subscribe();

// windowWhen: Nur den Endzeitpunkt steuern (nächstes startet sofort nach Ende)
source$.pipe(
  windowWhen(() => timer(1000)), // Fenster alle 1 Sekunde
  mergeAll()
).subscribe();
```

| Operator | Steuerung | Fensterperiode | Anwendungsfall |
|:---|:---|:---|:---|
| `windowToggle(open$, close)` | Separate Start-/Endsteuerung | Überlappung möglich | Komplexe Start-/Endbedingungen |
| `windowWhen(closing)` | Nur Ende gesteuert | Kontinuierlich | Einfaches periodisches Fenster |

## 📚 Verwandte Operatoren

- [`bufferToggle`](./bufferToggle) - Werte als Array sammeln (Array-Version von windowToggle)
- [`window`](./window) - Fensteraufteilung beim Timing eines anderen Observables
- [`windowTime`](./windowTime) - Zeitbasierte Fensteraufteilung
- [`windowCount`](./windowCount) - Anzahlbasierte Fensteraufteilung
- [`windowWhen`](./windowWhen) - Fensteraufteilung mit dynamischer Schließbedingung

## Zusammenfassung

Der Operator `windowToggle` ist ein erweitertes Tool, das Start und Ende unabhängig steuert und jeden Zeitraum als unabhängiges Observable verarbeiten kann.

- ✅ Start und Ende können separat gesteuert werden
- ✅ Mehrere Fenster können gleichzeitig verwaltet werden
- ✅ Unterschiedliche Verarbeitung kann auf jedes Fenster angewendet werden
- ⚠️ Subscription-Verwaltung erforderlich
- ⚠️ Häufige Start-Trigger verbrauchen Speicher
- ⚠️ Achtung bei Überlappung von Fensterperioden
