---
description: "window ist ein RxJS-Operator, der ein Quell-Observable in verschachtelte Observables aufteilt, wenn ein anderes Observable Werte ausgibt - ideal für erweiterte ereignisgesteuerte Stream-Verarbeitung."
---

# window - Trigger-Basiertes Fenster

Der `window`-Operator gruppiert Werte eines Quell-Observable, **bis ein anderes Observable einen Wert ausgibt**, und gibt diese Gruppe als **neues Observable** aus.
Während `buffer` ein Array zurückgibt, gibt `window` **Observable\<T>** zurück, sodass weitere Operatoren auf jedes Fenster angewendet werden können.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

// Gibt alle 100ms einen Wert aus
const source$ = interval(100);

// Klickereignis als Trigger verwenden
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  window(clicks$),
  mergeAll() // Jedes Fenster abflachen
).subscribe(value => {
  console.log('Wert im Fenster:', value);
});

// Bei jedem Klick wird ein neues Fenster gestartet
```

- Jedes Mal, wenn `clicks$` einen Wert ausgibt, wird ein neues Fenster (Observable) erstellt.
- Jedes Fenster kann als unabhängiges Observable verarbeitet werden.

[🌐 Offizielle RxJS-Dokumentation - `window`](https://rxjs.dev/api/operators/window)

## 💡 Typische Verwendungsmuster

- Ereignisgesteuerte Stream-Aufteilung
- Unterschiedliche Verarbeitung pro Fenster anwenden
- Datengruppierung mit dynamischen Trennzeichen
- Aggregationsverarbeitung pro Fenster

## 🔍 Unterschied zu buffer

| Operator | Ausgabe | Anwendungsfall |
|:---|:---|:---|
| `buffer` | **Array (T[])** | Gruppierte Werte zusammen verarbeiten |
| `window` | **Observable\<T>** | Unterschiedliche Stream-Verarbeitung pro Gruppe |

```ts
import { interval, timer } from 'rxjs';
import { buffer, window, mergeAll } from 'rxjs';

const source$ = interval(100);
const trigger$ = timer(1000, 1000);

// buffer - Ausgabe als Array
source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Puffer (Array):', values);
  // Ausgabe: Puffer (Array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// window - Ausgabe als Observable
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  console.log('Fenster (Observable):', window$);
  // Verschachtelte Subscription: erforderliches Pattern der window-Operatoren-Spezifikation
  // (notwendig zum Konsumieren der inneren Observable, die von window emittiert wird)
  window$.subscribe(value => {
    console.log('  Wert im Fenster:', value);
  });
});
```

## 🧠 Praktisches Codebeispiel 1: Zählung pro Fenster

Ein Beispiel, bei dem Buttonklicks als Trigger die bisherige Ereignisanzahl zählen.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, map, mergeAll, scan } from 'rxjs';

// Button erstellen
const button = document.createElement('button');
button.textContent = 'Fenster trennen';
document.body.appendChild(button);

// Ausgabebereich
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Gibt alle 100ms einen Wert aus
const source$ = interval(100);

// Buttonklick als Trigger
const clicks$ = fromEvent(button, 'click');

let windowCount = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const currentWindow = ++windowCount;
    console.log(`Fenster ${currentWindow} gestartet`);

    // Werte in jedem Fenster zählen
    return window$.pipe(
      scan((count) => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  output.textContent = `Aktuelles Fenster: ${windowCount}, Zähler: ${count}`;
});
```

- Bei jedem Buttonklick wird ein neues Fenster erstellt.
- Die Anzahl der Werte in jedem Fenster wird in Echtzeit gezählt.

## 🎯 Praktisches Codebeispiel 2: Unterschiedliche Verarbeitung pro Fenster

Ein fortgeschrittenes Beispiel für die Anwendung unterschiedlicher Verarbeitung auf jedes Fenster.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, take, mergeAll, map } from 'rxjs';

const source$ = interval(200);
const clicks$ = fromEvent(document, 'click');

let windowNumber = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Gerade Fenster: Nur erste 3 nehmen
      console.log(`Fenster ${current}: Nur erste 3 nehmen`);
      return window$.pipe(take(3));
    } else {
      // Ungerade Fenster: Alle nehmen
      console.log(`Fenster ${current}: Alle nehmen`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Wert: ${value} (Fenster ${windowNumber})`);
});
```

- Bedingte Verzweigung pro Fenster ermöglicht unterschiedliche Verarbeitung.
- Jedes Fenster ist ein unabhängiges Observable, sodass Operatoren frei kombiniert werden können.

## 🎯 Praktisches Beispiel: Steuerung mit mehreren Triggern

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { window, mergeAll, scan, map } from 'rxjs';

const source$ = interval(100);

// Mehrere Trigger: Klick oder 3 Sekunden vergangen
const clicks$ = fromEvent(document, 'click');
const threeSeconds$ = timer(3000, 3000);
const trigger$ = merge(clicks$, threeSeconds$);

source$.pipe(
  window(trigger$),
  map((window$, index) => {
    console.log(`Fenster ${index + 1} gestartet`);

    // Summe der Werte in jedem Fenster berechnen
    return window$.pipe(
      scan((sum, value) => sum + value, 0)
    );
  }),
  mergeAll()
).subscribe(sum => {
  console.log('Aktuelle Summe:', sum);
});
```

## ⚠️ Hinweise

### 1. Fenster-Subscription-Verwaltung

Jedes Fenster ist ein unabhängiges Observable und muss explizit abonniert werden.

```ts
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  // Verschachtelte Subscription: erforderliches Pattern der window-Operatoren-Spezifikation
  // Fenster selbst muss abonniert werden, sonst fließen keine Werte
  window$.subscribe(value => {
    console.log('Wert:', value);
  });
});
```

Alternativ mit `mergeAll()`, `concatAll()`, `switchAll()` usw. abflachen.

```ts
source$.pipe(
  window(trigger$),
  mergeAll() // Alle Fenster zusammenführen
).subscribe(value => {
  console.log('Wert:', value);
});
```

### 2. Vorsicht vor Speicherlecks

**Problem**: Wenn das Trigger-Observable keinen Wert ausgibt, bleibt das erste Fenster für immer offen und Werte akkumulieren sich unendlich.

#### ❌ Schlechtes Beispiel: Trigger wird nicht ausgelöst

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100); // Gibt alle 100ms Werte aus

// Button existiert nicht oder Benutzer klickt nicht
const button = document.querySelector('#start-button'); // Möglicherweise null
const clicks$ = fromEvent(button, 'click'); // Fehler oder löst nie aus

source$.pipe(
  window(clicks$), // Wenn clicks$ nicht auslöst, schließt erstes Fenster nicht
  mergeAll()
).subscribe();

// Probleme:
// - Wenn clicks$ keinen Wert ausgibt, bleibt erstes Fenster offen
// - source$-Werte (0, 1, 2, 3...) akkumulieren sich im Speicher
// - Verursacht Speicherleck
```

#### ✅ Gutes Beispiel 1: Timeout setzen

Timeout setzen, um zu verhindern, dass das erste Fenster zu lange offen bleibt.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = button ? fromEvent(button, 'click') : interval(0); // Fallback zu Dummy-Observable wenn Button null

// Klick oder 5 Sekunden vergangen - was zuerst kommt, schließt Fenster
const autoClose$ = timer(5000); // Gibt nach 5 Sekunden automatisch Wert aus
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // Fenster schließt garantiert innerhalb von 5 Sekunden
  mergeAll()
).subscribe();
```

#### ✅ Gutes Beispiel 2: Fenster periodisch schließen

Fenster periodisch schließen und neues starten, auch ohne Klicks.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = fromEvent(button, 'click');

// Fenster bei Klick oder alle 3 Sekunden schließen
const autoClose$ = timer(3000, 3000); // Nach ersten 3 Sekunden, dann alle 3 Sekunden
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // Fenster schließt auch ohne Klick alle 3 Sekunden
  mergeAll()
).subscribe();

// Ergebnis:
// - Auch wenn Benutzer nicht klickt, schließt Fenster automatisch alle 3 Sekunden
// - Verhindert unendliche Wertakkumulation im Speicher
```

### 3. Fensterüberlappung

Standardmäßig überlappen sich Fenster nicht (nächstes startet nach Schließen des vorherigen).
Wenn Überlappung benötigt wird, verwenden Sie `windowToggle` oder `windowWhen`.

## 🆚 Vergleich der window-Operatoren

| Operator | Trennungszeitpunkt | Anwendungsfall |
|:---|:---|:---|
| `window` | Ausgabe eines anderen Observable | Ereignisgesteuerte Aufteilung |
| `windowTime` | Feste Zeit | Zeitbasierte Aufteilung |
| `windowCount` | Feste Anzahl | Anzahlbasierte Aufteilung |
| `windowToggle` | Start-/Ende-Observable | Dynamische Start-/Ende-Steuerung |
| `windowWhen` | Dynamische Schließbedingung | Unterschiedliche Endbedingung pro Fenster |

## 📚 Verwandte Operatoren

- [`buffer`](./buffer) - Werte als Array zusammenfassen (Array-Version von window)
- [`windowTime`](./windowTime) - Zeitbasierte Fensteraufteilung
- [`windowCount`](./windowCount) - Anzahlbasierte Fensteraufteilung
- [`windowToggle`](./windowToggle) - Fenstersteuerung mit Start-/Ende-Observable
- [`windowWhen`](./windowWhen) - Fensteraufteilung mit dynamischer Schließbedingung
- [`groupBy`](./groupBy) - Observable nach Schlüssel gruppieren

## Zusammenfassung

Der `window`-Operator ist ein leistungsstarkes Werkzeug, das externe Observables als Trigger verwendet, um Streams aufzuteilen, wobei jede Gruppe als unabhängiges Observable verarbeitet werden kann.

- ✅ Unterschiedliche Verarbeitung pro Fenster anwendbar
- ✅ Flexible ereignisgesteuerte Steuerung
- ✅ Unterstützt erweiterte Stream-Operationen
- ⚠️ Subscription-Verwaltung erforderlich
- ⚠️ Vorsicht vor Speicherlecks
