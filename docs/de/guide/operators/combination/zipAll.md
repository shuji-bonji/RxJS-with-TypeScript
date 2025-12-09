---
description: zipAll ist ein Operator, der ein Higher-order Observable (Observable of Observables) empfängt und entsprechende Werte jedes inneren Observables in der Reihenfolge paart und als Array ausgibt.
---

# zipAll - Entsprechende Werte jedes inneren Observables paaren

Der `zipAll`-Operator empfängt ein **Higher-order Observable** (Observable of Observables),
**paart entsprechende Werte jedes inneren Observables in der Reihenfolge** und gibt sie als Array aus.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval, of } from 'rxjs';
import { zipAll, take } from 'rxjs';

// Higher-order Observable mit 3 inneren Observables
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Entsprechende Werte jedes inneren Observables in der Reihenfolge paaren
higherOrder$
  .pipe(zipAll())
  .subscribe(values => console.log(values));

// Ausgabe:
// [0, 0, 0] ← Alle 1. Werte
// [1, 1, 1] ← Alle 2. Werte
// (Hier abgeschlossen: Da 3. Observable nur 2 Werte ausgibt)
```

- Sammelt innere Observables, wenn das Higher-order Observable **abgeschlossen** ist
- **Paart Werte mit demselben Index** jedes inneren Observables
- Abgeschlossen, wenn **das kürzeste innere Observable abgeschlossen** ist

[🌐 RxJS Official Documentation - `zipAll`](https://rxjs.dev/api/index/function/zipAll)

## 💡 Typische Anwendungsmuster

- **Mehrere API-Antworten der Reihe nach zuordnen**
- **Werte mehrerer Streams zum gleichen Zeitpunkt vergleichen**
- **Ergebnisse paralleler Verarbeitung der Reihe nach kombinieren**

## 🧠 Praktisches Codebeispiel

Ein Beispiel, das entsprechende Werte mehrerer Zähler paart

```ts
import { interval, of } from 'rxjs';
import { zipAll, take, map } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// 3 Zähler mit unterschiedlichen Geschwindigkeiten erstellen
const counters$ = of(
  interval(1000).pipe(take(4), map(n => `Langsam: ${n}`)),
  interval(500).pipe(take(5), map(n => `Mittel: ${n}`)),
  interval(300).pipe(take(6), map(n => `Schnell: ${n}`))
);

// Entsprechende Werte jedes Zählers in der Reihenfolge paaren
counters$
  .pipe(zipAll())
  .subscribe(values => {
    const item = document.createElement('div');
    item.textContent = `[${values.join(', ')}]`;
    output.appendChild(item);
  });

// Ausgabe:
// [Langsam: 0, Mittel: 0, Schnell: 0]
// [Langsam: 1, Mittel: 1, Schnell: 1]
// [Langsam: 2, Mittel: 2, Schnell: 2]
// [Langsam: 3, Mittel: 3, Schnell: 3]
// (Hier abgeschlossen: "Langsamer" Zähler gibt nur 4 Werte aus)
```

## 🔄 Verwandte Creation Function

`zipAll` wird hauptsächlich zur Vereinfachung von Higher-order Observables verwendet,
aber für normale Paarung mehrerer Observables verwenden Sie die **Creation Function** `zip`.

```ts
import { zip, interval } from 'rxjs';
import { take } from 'rxjs';

// Creation Function-Version (häufigere Verwendung)
const zipped$ = zip(
  interval(1000).pipe(take(3)),
  interval(500).pipe(take(4)),
  interval(2000).pipe(take(2))
);

zipped$.subscribe(console.log);
```

Siehe [Kapitel 3 Creation Functions - zip](/de/guide/creation-functions/combination/zip).

## 🔄 Verwandte Operatoren

| Operator | Beschreibung |
|---|---|
| [combineLatestAll](./combineLatestAll) | Neueste Werte aller inneren Observables kombinieren |
| [mergeAll](./mergeAll) | Alle inneren Observables parallel abonnieren |
| [concatAll](./concatAll) | Innere Observables nacheinander abonnieren |
| [switchAll](./switchAll) | Zu neuem inneren Observable wechseln |

## 🔄 zipAll vs combineLatestAll

| Operator | Kombinationsmethode | Abschlusszeitpunkt |
|---|---|---|
| `zipAll` | **Gleicher Index** paarweise | Wenn **das kürzeste** innere Observable abgeschlossen ist |
| `combineLatestAll` | **Neueste Werte** kombinieren | Wenn **alle** inneren Observables abgeschlossen sind |

```ts
// zipAll: [0., 0., 0.], [1., 1., 1.], ...
// combineLatestAll: [Neueste, Neueste, Neueste], [Neueste, Neueste, Neueste], ...
```

## ⚠️ Wichtige Hinweise

### Abschluss des Higher-order Observable erforderlich

`zipAll` wartet, bis das Higher-order Observable (äußeres Observable) **abgeschlossen** ist, bevor es innere Observables sammelt.

#### ❌ Higher-order Observable wird nicht abgeschlossen, daher keine Ausgabe
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log); // Keine Ausgabe
```

#### ✅ Mit take abschließen
```ts
interval(1000).pipe(
  take(3), // Mit 3 abschließen
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log);
```

### Abschluss beim kürzesten inneren Observable

Abgeschlossen, wenn **das kürzeste innere Observable abgeschlossen** ist.

```ts
import { of, zipAll } from "rxjs";

of(
  of(1, 2, 3, 4, 5), // 5 Stück
  of(1, 2)           // 2 Stück ← Kürzestes
).pipe(
  zipAll()
).subscribe(console.log);

// Ausgabe: [1, 1], [2, 2]
// (Mit 2 abgeschlossen. 3, 4, 5 werden nicht verwendet)
```

### Backpressure (Speicherverbrauch)

Wenn die Ausgabegeschwindigkeiten der inneren Observables unterschiedlich sind, **sammeln sich Werte schneller innerer Observables im Speicher**.

```ts
import { interval, of, take, zipAll } from "rxjs";

// Werte des schnellen Zählers (100ms) sammeln sich im Speicher, während auf den langsamen Zähler (10000ms) gewartet wird
of(
  interval(10000).pipe(take(3)), // Langsam
  interval(100).pipe(take(100))  // Schnell
).pipe(
  zipAll()
).subscribe(console.log);
```

Achten Sie bei großem Geschwindigkeitsunterschied auf den Speicherverbrauch.
