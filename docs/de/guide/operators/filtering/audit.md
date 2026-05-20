---
description: "Der audit-Operator ist ein RxJS-Filteroperator, der nur den letzten Wert innerhalb des durch das benutzerdefinierte Observable kontrollierten Zeitraums ausgibt. Er ist ideal für die dynamische Zeitsteuerung."
---

# audit - letzter Wert des ausgegebenen Kontrollzeitraums

Der Operator "audit" wartet, bis ein benutzerdefiniertes Observable einen Wert ausgibt und gibt den **letzten Wert** aus, der von der Quelle innerhalb dieses Zeitraums ausgegeben wurde.
Während `auditTime` durch eine feste Zeit gesteuert wird, erlaubt `audit` die **Kontrolle des Zeitraums** mit einem dynamischen Observable.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Klick-Ereignis
const clicks$ = fromEvent(document, 'click');

// 1Getrennte Zeitabschnitte pro Sekunde
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Klick wurde aufgezeichnet');
});
```

- Wenn ein Klick erfolgt, beginnt ein Zeitraum von einer Sekunde.
- Nur der letzte Klick dieser 1-Sekunden-Periode wird ausgegeben.
- Nach einer Sekunde beginnt der nächste Zeitraum.

[🌐 Offizielle RxJS Dokumentation - `audit`](https://rxjs.dev/api/operators/audit)

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Klick-Ereignis
const clicks$ = fromEvent(document, 'click');

// 1Getrennte Zeitabschnitte pro Sekunde
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Klick wurde aufgezeichnet');
});
```

> Das obige Beispiel lässt der Einfachheit halber das Abbestellen von `fromEvent` weg. In echtem Code sollten Sie `takeUntil(destroy$)`, `take(N)` oder `Subscription.unsubscribe()` verwenden, um den Lebenszyklus explizit zu verwalten. Weitere Informationen: [Schwierigkeiten überwinden: Lebenszyklus-Management](/de/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Typische Nutzungsmuster

- **Dynamische Intervallabtastung**: Anpassung der Dauer je nach Auslastung.
- **Benutzerdefinierte Zeitsteuerung**: Steuerung der Dauer auf der Grundlage anderer Observable.
- **Adaptive Ereignisbegrenzung**: kontextabhängige Ausdünnung

## 🔍 Unterschiede zu auditTime

| Bediener. | Periodenkontrolle | Anwendungsfall. |
|---|---|---|
| auditTime". | Feste Zeit (Millisekunden) | Einfache zeitbasierte Kontrolle |
| audit". | **Benutzerdefiniertes Observable. | **Dynamische Zeitsteuerung**. |


```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Festgelegt1Sekunden
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Festgelegt1Sekunden'));

// audit - Dynamischer Zeitraum
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~.2Zufälliger Zeitraum von Sekunden
    return timer(period);
  })
).subscribe(() => console.log(`Dynamischer Zeitraum: ${period}ms`));
```

## 🧠 Praktisches Codebeispiel 1: Lastabhängige dynamische Probenahme

Dies ist ein Beispiel für die Anpassung des Abtastintervalls an die Systemlast.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UIErstellung
const output = document.createElement('div');
output.innerHTML = '<h3>Dynamische Probenahme</h3>';
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

// Belastungsgrad (0: Niedrige Last,1: Mittlere Belastung,2: Hohe Last)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Niedrige Last', 'Mittlere Belastung', 'Hohe Last'];
  statusDiv.textContent = `Aktuelle Last: ${levels[loadLevel]}`;
});

// Ereignis Mausbewegung
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Dauer abhängig von der Last
    const periods = [2000, 1000, 500]; // Niedrige Last→Lange Dauer, hohe Last→Kurze Dauer
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Position der Maus: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Max.10Anzeige bis zu
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Ausgedünnte Intervalle von 2 s bei geringer Last (Energiesparmodus)
- Feinabtastung in 500 ms-Intervallen, wenn die Last hoch ist.
- Der Zeitraum kann dynamisch an die Last angepasst werden.

## 🎯 Praktisches Codebeispiel 2: Periodensteuerung auf der Grundlage anderer Ströme

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map, startWith } from 'rxjs';

// UIErstellung
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

// Schiebereglerwerte überwachen
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Klick-Ereignis
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Schiebereglerwerte aktualisieren
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Klicken Sie aufauditGesteuert durch
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Klick auf Datensatz (Intervall: ${currentInterval}ms)`;
  output.insertBefore(log, output.firstChild);
});
```

## ⚠️ Anmerkungen.

### 1. Der erste Wert wird nicht sofort ausgegeben.

Nachdem das `audit` den ersten Wert erhalten hat, wartet es bis zum Ende der Periode.

{```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// Ausgabe:
// 9  (1Sekunden später,0~.9Letzter Wert von)
// 19 (2Sekunden später,10~.19Letzter Wert von)
// 29 (3Sekunden später,20~.29Letzter Wert von)
```

### (2) Das Observable für die Dauer wird jedes Mal neu erzeugt.

An `audit` übergebene Funktionen **müssen jedes Mal ein neues Observable zurückgeben**.

```

```ts
// ❌ Schlechtes Beispiel: Wenn die gleicheObservableInstanz verwendet wird und erneut verwendet wird
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2Funktioniert nach dem zweiten Mal nicht mehr
).subscribe();

// ✅ Gutes Beispiel: Jedes Mal wird eine neue Instanz erstelltObservableErzeugen Sie einen
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. Speicher und Leistung

Die Verwendung von `audit` auf Streams, in denen häufig Werte ausgegeben werden, verbraucht Speicher.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// schnellen Strom (10mspro Sekunde)
interval(10).pipe(
  audit(() => timer(1000)) // 1Abtastung jede Sekunde
).subscribe();
// 1pro Sekunde100Werte werden im Speicher abgelegt und nur der letzte1Nur der letzte Wert wird ausgegeben
```

## 🆚 Vergleich mit ähnlichen Operatoren

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Klick-Ereignis
const clicks$ = fromEvent(document, 'click');

// 1Getrennte Zeitabschnitte pro Sekunde
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Klick wurde aufgezeichnet');
});
```


```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Letzter Klick in Sekunden
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Letzter Klick'));

// throttle: 1Erster Klick in Sekunden
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Erster'));

// debounce: Nachdem der Klick aufhört1Sekunden nach
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Nach dem Stopp'));

// sample: 1Abtastung jede Sekunde
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Regelmäßig'));
```

## 📚 Verwandte Operatoren.

- **[auditTime](./auditTime)** - kontrolliert durch feste Zeit (vereinfachte Version von `audit`).
- **[throttle](./throttleTime)** - erster Wert, der zu Beginn der Periode ausgegeben wird.
- **[debounce](./debounceTime)** - gibt einen Wert nach einer Periode der Inaktivität aus.
- **[sample](./sampleTime)** - Stichprobe zum Zeitpunkt eines anderen Observable

## Zusammenfassung.

Der Operator "audit" gibt den letzten Wert innerhalb eines Zeitraums aus, der dynamisch durch ein benutzerdefiniertes Observable gesteuert wird.

- ✅ Dynamische Periodensteuerung ist möglich.
- ✅ Adaptives Sampling basierend auf der Last
- ✅ Steuerung auf der Grundlage anderer Ströme
- ⚠️ Jedes Mal muss ein neues Observable erzeugt werden
- ⚠️ Speicherempfindlich bei häufiger Ausgabe
