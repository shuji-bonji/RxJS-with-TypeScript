---
description: Der ignoreElements-Operator ist ein RxJS-Filteroperator, der alle Werte ignoriert und nur Abschluss und Fehler durchlässt. Praktisch beim Warten auf Verarbeitungsabschluss.
---

# ignoreElements - Nur Abschluss

Der `ignoreElements`-Operator ignoriert **alle vom Quell-Observable ausgegebenen Werte** und lässt nur **Abschlussbenachrichtigung und Fehlerbenachrichtigung** nach unten durch.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Wert:', value), // Wird nicht aufgerufen
  complete: () => console.log('Abgeschlossen')
});
// Ausgabe: Abgeschlossen
```

**Ablauf**:
1. 1, 2, 3, 4, 5 werden alle ignoriert
2. Nur Abschlussbenachrichtigung wird nach unten weitergegeben

[🌐 RxJS Offizielle Dokumentation - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Typische Anwendungsmuster

- **Auf Verarbeitungsabschluss warten**: Werte nicht benötigt, nur Abschluss wichtig
- **Nur Seiteneffekte ausführen**: Seiteneffekte mit tap ausführen, Werte ignorieren
- **Fehlerbehandlung**: Nur Fehler erfassen
- **Sequenz-Synchronisation**: Auf Abschluss mehrerer Verarbeitungen warten

## 🆚 Vergleich mit ähnlichen Operatoren

### ignoreElements vs filter(() => false) vs take(0)

```ts
import { of } from 'rxjs';
import { ignoreElements, filter, take } from 'rxjs';

const source$ = of(1, 2, 3);

// ignoreElements: Alle Werte ignorieren, Abschluss durchlassen
source$.pipe(
  ignoreElements()
).subscribe({
  next: v => console.log('Wert:', v),
  complete: () => console.log('ignoreElements: Abgeschlossen')
});
// Ausgabe: ignoreElements: Abgeschlossen

// filter(() => false): Alle Werte filtern, Abschluss durchlassen
source$.pipe(
  filter(() => false)
).subscribe({
  next: v => console.log('Wert:', v),
  complete: () => console.log('filter: Abgeschlossen')
});
// Ausgabe: filter: Abgeschlossen

// take(0): Sofort abschließen
source$.pipe(
  take(0)
).subscribe({
  next: v => console.log('Wert:', v),
  complete: () => console.log('take(0): Abgeschlossen')
});
// Ausgabe: take(0): Abgeschlossen
```

| Operator | Werteverarbeitung | Abschlussbenachrichtigung | Anwendungsfall |
|:---|:---|:---|:---|
| `ignoreElements()` | Alle ignorieren | Durchlassen | **Nur Abschluss benötigt** (empfohlen) |
| `filter(() => false)` | Alle filtern | Durchlassen | Bedingungsfilterung (zufällig alle ausgeschlossen) |
| `take(0)` | Sofort abschließen | Durchlassen | Sofort abschließen |

**Empfehlung**: Verwenden Sie `ignoreElements()` wenn Sie absichtlich alle Werte ignorieren möchten. Die Code-Absicht wird klarer.

## 📚 Verwandte Operatoren

- **[filter](./filter)** - Werte basierend auf Bedingungen filtern
- **[take](./take)** - Nur erste N Werte abrufen
- **[skip](./skip)** - Erste N Werte überspringen
- **[tap](../utility/tap)** - Seiteneffekte ausführen

## Zusammenfassung

Der `ignoreElements`-Operator ignoriert alle Werte und lässt nur Abschluss und Fehler durch.

- ✅ Ideal wenn nur Abschlussbenachrichtigung benötigt wird
- ✅ Seiteneffekte (tap) werden ausgeführt
- ✅ Lässt auch Fehlerbenachrichtigung durch
- ✅ Absicht klarer als `filter(() => false)`
- ⚠️ Beendet nicht bei unendlichen Observables
- ⚠️ Rückgabetyp ist `Observable<never>`
- ⚠️ Werte werden vollständig ignoriert, aber Seiteneffekte werden ausgeführt
