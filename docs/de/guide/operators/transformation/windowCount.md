---
description: "windowCount teilt Observable-Werte in Fenster fester Anzahl. Optimal für Batch-Verarbeitung, Aggregation und Paginierung mit TypeScript-Typsicherheit."
---

# windowCount - Observable in bestimmte Anzahlen aufteilen

Der `windowCount`-Operator teilt ausgegebene Werte in **neue Observables** nach einer bestimmten Anzahl auf.
Während `bufferCount` ein Array zurückgibt, gibt `windowCount` **Observable\<T>** zurück, sodass auf jedes Fenster weitere Operatoren angewendet werden können.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { windowCount, mergeAll } from 'rxjs';

// Gibt alle 100ms einen Wert aus
const source$ = interval(100);

source$.pipe(
  windowCount(5),
  mergeAll() // Flacht jedes Fenster ab
).subscribe(value => {
  console.log('Wert im Fenster:', value);
});

// Ausgabe:
// Wert im Fenster: 0
// Wert im Fenster: 1
// Wert im Fenster: 2
// Wert im Fenster: 3
// Wert im Fenster: 4
// (neues Fenster beginnt)
// Wert im Fenster: 5
// ...
```

- Alle 5 Werte wird ein neues Fenster (Observable) erstellt.
- Charakteristisch ist die Aufteilung nach Anzahl.

[🌐 RxJS Offizielle Dokumentation - `windowCount`](https://rxjs.dev/api/operators/windowCount)

## 💡 Typische Anwendungsmuster

- Aggregationsverarbeitung in festen Intervallen
- Batch-Übertragung von Daten (unterschiedliche Verarbeitung pro Fenster)
- Paginierungsverarbeitung
- Berechnung statistischer Informationen pro Fenster

## 🔍 Unterschied zu bufferCount

| Operator | Ausgabe | Anwendungsfall |
|:---|:---|:---|
| `bufferCount` | **Array (T[])** | Gesammelte Verarbeitung gruppierter Werte |
| `windowCount` | **Observable\<T>** | Unterschiedliche Stream-Verarbeitung pro Gruppe |

```ts
import { interval } from 'rxjs';
import { bufferCount, windowCount, mergeAll } from 'rxjs';

const source$ = interval(100);

// bufferCount - Ausgabe als Array
source$.pipe(
  bufferCount(5)
).subscribe(values => {
  console.log('Puffer (Array):', values);
  // Ausgabe: Puffer (Array): [0, 1, 2, 3, 4]
});

// windowCount - Ausgabe als Observable
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  console.log('Fenster (Observable):', window$);
  // Verschachtelte Subscription: erforderliches Pattern der window-Operatoren-Spezifikation
  // (notwendig zum Konsumieren der inneren Observable, die von windowCount emittiert wird)
  window$.subscribe(value => {
    console.log('  Wert im Fenster:', value);
  });
});
```

## 🧠 Praktisches Codebeispiel 1: Summenwerte pro Fenster

Beispiel zur Berechnung der Summe von jeweils 5 Werten.

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, reduce } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>Summenwerte von je 5</h3>';
document.body.appendChild(output);

const source$ = interval(200);

let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;
    console.log(`Fenster ${current} beginnt`);

    // Berechne die Summe jedes Fensters
    return window$.pipe(
      reduce((sum, value) => sum + value, 0),
      map(sum => ({ windowNum: current, sum }))  // Fensternummer einschließen
    );
  }),
  mergeAll()
).subscribe(result => {
  const div = document.createElement('div');
  div.textContent = `Summe von Fenster ${result.windowNum}: ${result.sum}`;
  output.appendChild(div);
});

// Ausgabe:
// Summe von Fenster 1: 10  (0+1+2+3+4)
// Summe von Fenster 2: 35  (5+6+7+8+9)
// Summe von Fenster 3: 60  (10+11+12+13+14)
```

## 🎯 Praktisches Codebeispiel 2: Angabe des Startindex

Mit dem zweiten Argument kann der Startindex angegeben werden. Es können überlappende Fenster erstellt werden.

```ts
import { range } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// Gibt Werte von 0 bis 9 aus
range(0, 10).pipe(
  windowCount(3, 2), // Jeweils 3, verschoben um jeweils 2
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Fenster:', values);
});

// Ausgabe:
// Fenster: [0, 1, 2]
// Fenster: [2, 3, 4]    ← Verschoben um 2 (ab 2)
// Fenster: [4, 5, 6]    ← Verschoben um 2 (ab 4)
// Fenster: [6, 7, 8]
// Fenster: [8, 9]       ← Letztes hat 2
```

### Verhaltensmuster des Startindex

```ts
// windowCount(bufferSize, startBufferEvery)

windowCount(3, 3) // Aufeinanderfolgend (Standard): [0,1,2], [3,4,5], [6,7,8]
windowCount(3, 2) // Überlappend: [0,1,2], [2,3,4], [4,5,6]
windowCount(3, 4) // Mit Lücke: [0,1,2], [4,5,6], [8,9,10]
```

## 🎯 Praktisches Beispiel: Unterschiedliche Verarbeitung pro Fenster

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, take } from 'rxjs';

const source$ = interval(100);
let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Gerade Fenster: Nur die ersten 2 erhalten
      console.log(`Fenster ${current}: Erste 2 erhalten`);
      return window$.pipe(take(2));
    } else {
      // Ungerade Fenster: Alle erhalten
      console.log(`Fenster ${current}: Alle erhalten`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Wert: ${value} (Fenster ${windowNumber})`);
});
```

## 🧠 Praktisches Codebeispiel 3: Paginierungsartige Verarbeitung

```ts
import { from } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// Daten von 1-20
const data$ = from(Array.from({ length: 20 }, (_, i) => i + 1));

// Seitenaufteilung in 5er-Gruppen
data$.pipe(
  windowCount(5),
  mergeMap((window$, index) => {
    const pageNumber = index + 1;
    return window$.pipe(
      toArray(),
      map(items => ({ page: pageNumber, items }))
    );
  })
).subscribe(page => {
  console.log(`Seite ${page.page}:`, page.items);
});

// Ausgabe:
// Seite 1: [1, 2, 3, 4, 5]
// Seite 2: [6, 7, 8, 9, 10]
// Seite 3: [11, 12, 13, 14, 15]
// Seite 4: [16, 17, 18, 19, 20]
```

## ⚠️ Hinweise

### 1. Verwaltung der Fenster-Subscriptions

Jedes Fenster ist ein unabhängiges Observable, daher muss explizit abonniert werden.

```ts
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  // Ohne Abonnement des Fensters selbst fließen keine Werte
  window$.subscribe(value => {
    console.log('Wert:', value);
  });
});
```

Oder verwenden Sie `mergeAll()`, `concatAll()`, `switchAll()` usw. zum Abflachen.

### 2. Letztes Fenster

Bei Abschluss des Quell-Observable wird das letzte Fenster auch ausgegeben, wenn es weniger als die angegebene Anzahl hat.

```ts
import { of } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

of(1, 2, 3, 4, 5, 6, 7).pipe(
  windowCount(3),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Fenster:', values);
});

// Ausgabe:
// Fenster: [1, 2, 3]
// Fenster: [4, 5, 6]
// Fenster: [7]  ← Nur 1
```

### 3. Speichernutzung durch Startindex

Wenn `startBufferEvery` kleiner als `bufferSize` ist (Überlappung), sind mehrere Fenster gleichzeitig aktiv, wodurch die Speichernutzung steigt.

```ts
// Überlappung: Maximal 2 Fenster gleichzeitig aktiv
windowCount(5, 3)

// Gegenmaßnahme: Bei Bedarf mit take() begrenzen
source$.pipe(
  take(100), // Maximal 100
  windowCount(5, 3)
)
```

## 🆚 Vergleich der window-Operatoren

| Operator | Aufteilungszeitpunkt | Anwendungsfall |
|:---|:---|:---|
| `window` | Ausgabe eines anderen Observable | Ereignisgesteuerte Aufteilung |
| `windowTime` | Feste Zeitintervalle | Zeitbasierte Aufteilung |
| `windowCount` | **Feste Anzahl** | **Zahlenbasierte Aufteilung** |
| `windowToggle` | Start-/End-Observable | Dynamische Start-/End-Steuerung |
| `windowWhen` | Dynamische Abschlussbedingung | Unterschiedliche Abschlussbedingungen pro Fenster |

## 📚 Verwandte Operatoren

- [`bufferCount`](./bufferCount) - Werte als Array zusammenfassen (Array-Version von windowCount)
- [`window`](./window) - Fensteraufteilung nach Zeitpunkt eines anderen Observable
- [`windowTime`](./windowTime) - Zeitbasierte Fensteraufteilung
- [`windowToggle`](./windowToggle) - Fenstersteuerung mit Start-/End-Observable
- [`windowWhen`](./windowWhen) - Fensteraufteilung mit dynamischer Abschlussbedingung

## Zusammenfassung

Der `windowCount`-Operator ist ein nützliches Werkzeug, das Streams zahlenbasiert aufteilt und jede Gruppe als unabhängiges Observable verarbeiten kann.

- ✅ Optimal für Aggregation/Verarbeitung in festen Intervallen
- ✅ Unterschiedliche Verarbeitung auf jedes Fenster anwendbar
- ✅ Überlappung mit Startindex möglich
- ⚠️ Subscription-Verwaltung erforderlich
- ⚠️ Achten Sie auf Speichernutzung bei Überlappung
