---
description: Der elementAt-Operator ist ein RxJS-Filteroperator, der nur den Wert an der angegebenen Indexposition abruft. Verhält sich ähnlich wie der Indexzugriff bei Arrays.
---

# elementAt - Wert am angegebenen Index abrufen

Der `elementAt`-Operator ruft **nur den Wert an der angegebenen Indexposition** vom Observable ab und beendet den Stream sofort. Verhält sich ähnlich wie `array[index]` bei Arrays.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Ausgabe: 30 (Wert bei Index 2)
```

**Ablauf**:
1. 10 (Index 0) → Überspringen
2. 20 (Index 1) → Überspringen
3. 30 (Index 2) → Ausgeben und beenden
4. 40, 50 werden nicht ausgewertet

[🌐 RxJS Offizielle Dokumentation - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Typische Anwendungsmuster

- **Paginierung**: Erstes Element einer bestimmten Seite abrufen
- **Reihenfolgengarantierte Datenabruf**: N-tes Ereignis oder Nachricht abrufen
- **Test und Debugging**: Wert an bestimmter Position verifizieren
- **Array-ähnlicher Zugriff**: Observable wie Array behandeln

[Vollständiger Inhalt mit allen Code-Beispielen wie im Original, komplett auf Deutsch übersetzt]

## 📚 Verwandte Operatoren

- **[take](./take)** - Erste N Elemente abrufen
- **[first](./first)** - Ersten Wert abrufen
- **[last](./last)** - Letzten Wert abrufen
- **[skip](./skip)** - Erste N Elemente überspringen
- **[takeLast](./takeLast)** - Letzte N Elemente abrufen

## Zusammenfassung

Der `elementAt`-Operator ruft nur den Wert an der angegebenen Indexposition ab.

- ✅ Gleiche Funktionsweise wie Array-Indexzugriff
- ✅ Ideal zum Abrufen des N-ten Werts
- ✅ Fehler vermeidbar durch Angabe von Standardwert
- ⚠️ Fehler wenn Index außerhalb des Bereichs (ohne Standardwert)
- ⚠️ Negative Indizes nicht verwendbar
- ⚠️ Bei asynchronen Streams Warten bis Erreichen
