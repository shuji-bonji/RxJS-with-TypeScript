---
description: reduce ist ein Transformationsoperator von RxJS, der alle Werte des Streams kumuliert und nur das Endergebnis bei Abschluss ausgibt. Ideal für Situationen, in denen nur das endgültige Aggregationsergebnis benötigt wird, wie Berechnung von Summe/Durchschnitt/Maximum/Minimum von Zahlen, Objektaggregation oder Array-Konstruktion. Anders als scan werden Zwischenergebnisse nicht ausgegeben, und Stream-Abschluss ist erforderlich, daher nicht für unendliche Streams verwendbar.
---

# reduce - Nur das endgültige kumulative Ergebnis ausgeben

Der `reduce`-Operator wendet eine kumulative Funktion auf jeden Wert des Streams an und gibt **nur das endgültige kumulative Ergebnis bei Stream-Abschluss** aus.
Funktioniert wie `Array.prototype.reduce` bei Arrays, Zwischenergebnisse werden nicht ausgegeben.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Ausgabe: 15 (nur Endergebnis)
```

- `acc` ist der kumulative Wert, `curr` ist der aktuelle Wert.
- Beginnt mit dem Anfangswert (hier `0`) und kumuliert schrittweise.
- Gibt bis zum Abschluss des Streams keine Werte aus, **nur das Endergebnis bei Abschluss**.

[🌐 RxJS Offizielle Dokumentation - `reduce`](https://rxjs.dev/api/operators/reduce)

## 💡 Typische Anwendungsmuster

- Berechnung von Summe, Durchschnitt, Maximum, Minimum von Zahlen
- Objektaggregation oder -transformation
- Array-Konstruktion oder -Verbindung
- Wenn nur das endgültige Aggregationsergebnis benötigt wird

## 🔍 Unterschied zu scan

| Operator | Ausgabe-Timing | Ausgabeinhalt | Verwendungszweck |
|:---|:---|:---|:---|
| `reduce` | **Nur einmal bei Abschluss** | Endgültiges kumulatives Ergebnis | Aggregation, wenn nur Endergebnis benötigt wird |
| `scan` | **Jedes Mal bei jedem Wert** | Alle einschließlich Zwischenergebnisse | Echtzeit-Aggregation, Zustandsverwaltung |

```ts
import { of } from 'rxjs';
import { reduce, scan } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== reduce ===');
source$.pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Ausgabe: 15

console.log('=== scan ===');
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Ausgabe: 1, 3, 6, 10, 15
```

## 🧠 Praktisches Codebeispiel (mit UI)

Beispiel, das Werte mehrerer Eingabefelder summiert und das Endergebnis bei Button-Klick anzeigt.

```ts
import { fromEvent, from } from 'rxjs';
import { map, reduce, switchMap } from 'rxjs';

// Eingabefelder erstellen
const inputs: HTMLInputElement[] = [];
for (let i = 1; i <= 3; i++) {
  const label = document.createElement('label');
  label.textContent = `Wert${i}: `;
  const input = document.createElement('input');
  input.type = 'number';
  input.value = '0';
  label.appendChild(input);
  document.body.appendChild(label);
  document.body.appendChild(document.createElement('br'));
  inputs.push(input);
}

// Berechnungs-Button
const button = document.createElement('button');
button.textContent = 'Summe berechnen';
document.body.appendChild(button);

// Ergebnisanzeigebereich
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Summe bei Button-Klick berechnen
fromEvent(button, 'click').pipe(
  switchMap(() => {
    // Alle Eingabewerte abrufen
    const values = inputs.map(input => Number(input.value) || 0);
    return from(values).pipe(
      reduce((acc, curr) => acc + curr, 0)
    );
  })
).subscribe(total => {
  output.textContent = `Summe: ${total}`;
  console.log('Summe:', total);
});
```

- Bei Button-Klick werden alle Eingabewerte aggregiert und nur die endgültige Summe angezeigt.
- Zwischenergebnisse werden nicht ausgegeben.

## 🎯 Beispiel für Objektaggregation

Praktisches Beispiel zum Zusammenfassen mehrerer Werte in ein Objekt.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface Product {
  category: string;
  price: number;
}

const products: Product[] = [
  { category: 'Lebensmittel', price: 500 },
  { category: 'Getränke', price: 200 },
  { category: 'Lebensmittel', price: 800 },
  { category: 'Getränke', price: 150 },
  { category: 'Lebensmittel', price: 300 },
];

// Gesamtbetrag nach Kategorie aggregieren
from(products).pipe(
  reduce((acc, product) => {
    acc[product.category] = (acc[product.category] || 0) + product.price;
    return acc;
  }, {} as Record<string, number>)
).subscribe(result => {
  console.log('Summe nach Kategorie:', result);
});

// Ausgabe:
// Summe nach Kategorie: { Lebensmittel: 1600, Getränke: 350 }
```

## 🎯 Beispiel für Array-Konstruktion

Beispiel zum Zusammenfassen von Stream-Werten in ein Array.

```ts
import { interval } from 'rxjs';
import { take, reduce } from 'rxjs';

interval(100).pipe(
  take(5),
  reduce((acc, value) => {
    acc.push(value);
    return acc;
  }, [] as number[])
).subscribe(array => {
  console.log('Gesammeltes Array:', array);
});

// Ausgabe:
// Gesammeltes Array: [0, 1, 2, 3, 4]
```

::: tip
Beim Konstruieren von Arrays erwägen Sie die Verwendung des prägnanter [`toArray`](../utility/toArray)-Operators.
```ts
interval(100).pipe(
  take(5),
  toArray()
).subscribe(console.log);
// Ausgabe: [0, 1, 2, 3, 4]
```
:::

## 💡 Typsichere Nutzung von reduce

Beispiel mit TypeScript-Typinferenz.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface UserAction {
  type: 'click' | 'scroll' | 'input';
  timestamp: number;
}

const actions: UserAction[] = [
  { type: 'click', timestamp: 100 },
  { type: 'scroll', timestamp: 200 },
  { type: 'click', timestamp: 300 },
  { type: 'input', timestamp: 400 },
];

const actions$ = from(actions);

// Anzahl nach Aktionstyp aggregieren
actions$.pipe(
  reduce((acc, action) => {
    acc[action.type] = (acc[action.type] || 0) + 1;
    return acc;
  }, {} as Record<UserAction['type'], number>)
).subscribe(result => {
  console.log('Aktionsaggregation:', result);
});

// Ausgabe:
// Aktionsaggregation: { click: 2, scroll: 1, input: 1 }
```

## ⚠️ Achtung

### ❌ Wird bei unendlichen Streams nicht abgeschlossen (wichtig)

> [!WARNING]
> **`reduce` gibt keinen einzigen Wert aus, bis `complete()` aufgerufen wird.** Bei unendlichen Streams (`interval`, `fromEvent` usw.) wird niemals ein Wert erhalten, was eine Ursache für Probleme in der Praxis ist.

```ts
import { interval } from 'rxjs';
import { reduce } from 'rxjs';

// ❌ Schlechtes Beispiel: Unendlicher Stream, daher keine Ausgabe
interval(1000).pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Keine Ausgabe (weil Stream nicht abschließt)
```

**Gegenmaßnahme 1: Bei rollierender Aggregation `scan` verwenden**

```ts
import { interval, scan, take } from 'rxjs';

// ✅ Gutes Beispiel: Zwischenergebnisse in Echtzeit erhalten
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Ausgabe: 0, 1, 3, 6, 10 (kumulative Werte jedes Mal)
```

**Gegenmaßnahme 2: Bei Bedarf nur Endwert `scan` + `takeLast(1)`**

```ts
import { interval, scan, take, takeLast } from 'rxjs';

// ✅ Gutes Beispiel: Mit scan kumulieren, nur Endwert abrufen
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0),
  takeLast(1)
).subscribe(console.log);
// Ausgabe: 10 (nur Endergebnis)
```

**Gegenmaßnahme 3: Abschlussbedingung mit `take` explizit angeben**

```ts
import { interval, take, reduce } from 'rxjs';

// ✅ Gutes Beispiel: Abschlussbedingung mit take setzen
interval(1000).pipe(
  take(5),
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Ausgabe: 10
```

> [!TIP]
> **Auswahlkriterien**:
> - Zwischenergebnisse benötigt → `scan`
> - Nur Endergebnis benötigt & Stream-Abschluss garantiert → `reduce`
> - Nur Endergebnis benötigt & unendlicher Stream → `scan` + `takeLast(1)` oder `take` + `reduce`

### Speichernutzung

Bei kumulativen Werten, die große Objekte oder Arrays werden, ist auf Speicherverbrauch zu achten.

```ts
// Beispiel mit Speicherachtung
from(largeDataArray).pipe(
  reduce((acc, item) => {
    acc.push(item); // Große Datenmengen akkumulieren
    return acc;
  }, [])
).subscribe();
```

## 📚 Verwandte Operatoren

- [`scan`](./scan) - Gibt Zwischenergebnisse bei jedem Wert aus
- [`toArray`](../utility/toArray) - Fasst alle Werte in ein Array zusammen
- [`count`](https://rxjs.dev/api/operators/count) - Zählt Anzahl der Werte
- [`min`](https://rxjs.dev/api/operators/min) / [`max`](https://rxjs.dev/api/operators/max) - Minimum/Maximum abrufen

## Zusammenfassung

Der `reduce`-Operator kumuliert alle Werte des Streams und gibt **nur das Endergebnis bei Abschluss** aus. Geeignet, wenn Zwischenergebnisse nicht benötigt werden und nur das endgültige Aggregationsergebnis erforderlich ist. Da jedoch kein Ergebnis erhalten wird, wenn der Stream nicht abschließt, muss bei unendlichen Streams `scan` verwendet oder eine Abschlussbedingung mit `take` usw. gesetzt werden.
