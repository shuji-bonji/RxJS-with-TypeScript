---
description: Der findIndex-Operator ist ein RxJS-Filteroperator, der den Index des ersten Werts zurückgibt, der eine Bedingung erfüllt. Gibt -1 zurück, wenn nicht gefunden.
---

# findIndex - Passenden Index Finden

Der `findIndex`-Operator gibt den **Index des ersten Werts zurück, der eine Bedingung erfüllt**, und beendet den Stream sofort. Wenn kein Wert gefunden wird, gibt er `-1` zurück.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Ausgabe: 4 (Index der ersten geraden Zahl 8)
```

**Ablauf**:
1. 1 (Index 0) → Ungerade, überspringen
2. 3 (Index 1) → Ungerade, überspringen
3. 5 (Index 2) → Ungerade, überspringen
4. 7 (Index 3) → Ungerade, überspringen
5. 8 (Index 4) → Gerade, Index 4 ausgeben und beenden

[🌐 RxJS Offizielle Dokumentation - `findIndex`](https://rxjs.dev/api/operators/findIndex)

## 💡 Typische Anwendungsmuster

- **Position in Array ermitteln**: Position eines Elements abrufen, das bestimmte Bedingung erfüllt
- **Reihenfolge prüfen**: An welcher Stelle erscheint ein Element, das eine Bedingung erfüllt
- **Datensortierung**: Verarbeitung mit Index-Informationen
- **Existenzprüfung**: Existenz durch Prüfung auf -1 bestätigen

[Code-Beispiele mit vollständiger deutscher Übersetzung]

## 🆚 Vergleich mit ähnlichen Operatoren

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Gibt Index des ersten Werts zurück, der Bedingung erfüllt
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Ausgabe: 2 (Index von 30)

// find: Gibt ersten Wert zurück, der Bedingung erfüllt
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Ausgabe: 30

// elementAt: Gibt Wert am angegebenen Index zurück
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Ausgabe: 30
```

| Operator | Argument | Rückgabewert | Wenn nicht gefunden |
|:---|:---|:---|:---|
| `findIndex(predicate)` | Bedingungsfunktion | Index (Zahl) | `-1` |
| `find(predicate)` | Bedingungsfunktion | Wert selbst | `undefined` |
| `elementAt(index)` | Index | Wert selbst | Fehler (ohne Standardwert) |

## 📚 Verwandte Operatoren

- **[find](./find)** - Ersten Wert abrufen, der Bedingung erfüllt
- **[elementAt](./elementAt)** - Wert am angegebenen Index abrufen
- **[first](./first)** - Ersten Wert abrufen
- **[filter](./filter)** - Alle Werte abrufen, die Bedingung erfüllen

## Zusammenfassung

Der `findIndex`-Operator gibt den Index des ersten Werts zurück, der eine Bedingung erfüllt.

- ✅ Ähnliche Funktionsweise wie JavaScript `Array.findIndex()`
- ✅ Ideal wenn Index-Information benötigt wird
- ✅ Gibt `-1` zurück wenn nicht gefunden (kein Fehler)
- ✅ Wird sofort beendet sobald gefunden
- ⚠️ Rückgabewert ist immer `number`-Typ (-1 oder ganzzahl ≥ 0)
- ⚠️ Verwenden Sie `find` wenn Wert selbst benötigt wird
