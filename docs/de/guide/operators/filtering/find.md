---
description: find ist ein RxJS-Filteroperator, der den ersten Wert findet, der eine Bedingung erfüllt, ihn ausgibt und den Stream sofort beendet. Ideal für Benutzersuche, Bestandsprüfung, Fehlerprotokoll-Erkennung usw. - Szenarien, in denen Sie ein bestimmtes Element aus einem Array oder einer Liste suchen möchten. Wenn kein Wert gefunden wird, wird undefined ausgegeben, und in TypeScript ist der Rückgabetyp T | undefined.
---

# find - Ersten Wert finden, der eine Bedingung erfüllt

Der `find`-Operator findet den **ersten Wert, der eine Bedingung erfüllt**, gibt ihn aus und beendet den Stream sofort. Wenn kein Wert gefunden wird, gibt er `undefined` aus.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Ausgabe: 8 (erste gerade Zahl)
```

**Ablauf**:
1. 1, 3, 5, 7 prüfen → Erfüllen Bedingung nicht
2. 8 prüfen → Erfüllt Bedingung → 8 ausgeben und beenden
3. 9, 10 werden nicht ausgewertet

[🌐 RxJS Offizielle Dokumentation - `find`](https://rxjs.dev/api/operators/find)


## 🆚 Vergleich mit first

`find` und `first` sind ähnlich, werden aber unterschiedlich verwendet.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: Erster Wert, der Bedingung erfüllt (Bedingung optional)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Ausgabe: 7

// find: Erster Wert, der Bedingung erfüllt (Bedingung erforderlich)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Ausgabe: 7
```

| Operator | Bedingungsangabe | Wenn Wert nicht gefunden | Anwendungsfall |
|---|---|---|---|
| `first()` | Optional | Fehler (`EmptyError`) | Ersten Wert abrufen |
| `first(predicate)` | Optional | Fehler (`EmptyError`) | Abruf mit Bedingung |
| `find(predicate)` | Erforderlich | `undefined` ausgeben | Suche/Existenzprüfung |


## 💡 Typische Anwendungsmuster

[Code-Beispiele für Benutzersuche, Bestandsprüfung, Fehlerprotokollsuche]

## 📚 Verwandte Operatoren

- **[first](./first)** - Methode zum Abrufen des ersten Werts lernen
- **[filter](./filter)** - Methode zum Filtern basierend auf Bedingungen lernen
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - Methode zum Abrufen des Index des ersten Werts, der eine Bedingung erfüllt (Offizielle Dokumentation)
- **[Praktische Beispiele für Filteroperatoren](./practical-use-cases)** - Reale Anwendungsfälle lernen

## Zusammenfassung

Der `find`-Operator findet den ersten Wert, der eine Bedingung erfüllt.

- ✅ Wenn Sie den ersten Wert suchen möchten, der eine Bedingung erfüllt
- ✅ Wenn Sie Existenz prüfen möchten
- ✅ Wenn Sie mit `undefined` umgehen möchten, falls nicht gefunden
- ✅ Wenn Sie ein bestimmtes Element aus Array oder Liste suchen möchten
- ⚠️ `find` gibt `undefined` aus, wenn nicht gefunden (kein Fehler)
- ⚠️ Wird sofort beendet beim ersten Wert, der Bedingung erfüllt
- ⚠️ In TypeScript ist Rückgabetyp `T | undefined`
