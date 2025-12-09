---
description: Der skipLast-Operator ist ein RxJS-Filteroperator, der die letzten N Werte eines Observable-Streams überspringt und nur die vorherigen Werte ausgibt.
---

# skipLast - Die letzten N Werte überspringen

Der `skipLast`-Operator **überspringt die letzten N Werte** eines Quell-Observables und gibt nur die vorherigen Werte aus. Werte werden im Puffer gehalten, bis der Stream abgeschlossen wird, und nur die nicht-letzten N Werte werden ausgegeben.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 bis 9

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 werden übersprungen)
```

**Ablauf**:
1. Stream gibt 0, 1, 2, ... aus
2. Die letzten 3 Werte (7, 8, 9) werden im Puffer gehalten
3. Werte, die die Puffergröße überschreiten (0~6), werden ausgegeben
4. Bei Stream-Abschluss werden die gepufferten Werte (7, 8, 9) verworfen, ohne ausgegeben zu werden

[🌐 Offizielle RxJS-Dokumentation - `skipLast`](https://rxjs.dev/api/operators/skipLast)

## 💡 Typische Anwendungsmuster

- **Ausschluss neuester Daten**: Ausschluss unbestätigter neuester Daten
- **Batch-Verarbeitung**: Ausschluss unbestätigter Daten vor Verarbeitungsabschluss
- **Datenvalidierung**: Wenn Validierung mit nachfolgenden Werten erforderlich ist
- **Verarbeitung verzögert bestätigter Daten**: Wenn die letzten N Werte nicht bestätigt sind

## 🆚 Vergleich mit ähnlichen Operatoren

### skipLast vs takeLast vs skip

```ts
import { range } from 'rxjs';
import { skipLast, takeLast, skip } from 'rxjs';

const numbers$ = range(0, 10); // 0 bis 9

// skipLast: Die letzten N überspringen
numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5, 6

// takeLast: Nur die letzten N abrufen
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Ausgabe: 7, 8, 9

// skip: Die ersten N überspringen
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Ausgabe: 3, 4, 5, 6, 7, 8, 9
```

| Operator | Überspringposition | Ausgabe-Timing | Warten auf Abschluss |
|:---|:---|:---|:---|
| `skipLast(n)` | Letzte n Werte | Ausgabe, sobald Puffer überschritten | Erforderlich |
| `takeLast(n)` | Alle außer letzten n | Zusammen nach Abschluss ausgegeben | Erforderlich |
| `skip(n)` | Erste n Werte | Sofort ausgegeben | Nicht erforderlich |

**Visueller Unterschied**:

```
Eingabe: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

skipLast(3): 0, 1, 2, 3, 4, 5, 6 | [7, 8, 9 überspringen]
                                  ^Letzte 3

takeLast(3): [0~6 überspringen] | 7, 8, 9
                                 ^Nur letzte 3

skip(3): [0, 1, 2 überspringen] | 3, 4, 5, 6, 7, 8, 9
          ^Erste 3
```

## ⚠️ Hinweise

### 1. Verhalten bei unendlichen Streams

Da `skipLast` die letzten N nicht identifizieren kann, bis der Stream abgeschlossen ist, funktioniert es bei unendlichen Streams nicht wie beabsichtigt.

```ts
import { interval } from 'rxjs';
import { skipLast } from 'rxjs';

// ❌ Schlechtes Beispiel: skipLast mit unendlichem Stream verwenden
interval(1000).pipe(
  skipLast(3)
).subscribe(console.log);
// Ausgabe: 0 (nach 3 Sek.), 1 (nach 4 Sek.), 2 (nach 5 Sek.), ...
// Wird mit N-Verzögerung unendlich ausgegeben
// Die letzten 3 bleiben ewig im Puffer und werden nie ausgegeben
```

Bei unendlichen Streams werden die letzten N Werte nicht bestätigt, daher werden alle Werte mit N-Verzögerung weiter ausgegeben. Da es keine wahren "letzten N" gibt, kann der ursprüngliche Zweck von `skipLast` nicht erreicht werden.

**Lösung**: Mit `take` zu einem endlichen Stream machen

```ts
import { interval } from 'rxjs';
import { take, skipLast } from 'rxjs';

// ✅ Gutes Beispiel: Endlichen Stream erstellen, dann skipLast verwenden
interval(1000).pipe(
  take(10),      // Mit ersten 10 abschließen
  skipLast(3)    // Letzte 3 überspringen
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5, 6
// (7, 8, 9 werden übersprungen)
```

### 2. Auf Puffergröße achten

`skipLast(n)` hält immer n Werte im Puffer.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

// ⚠️ 1000 Werte im Puffer halten
range(0, 1000000).pipe(
  skipLast(1000)
).subscribe(console.log);
```

### 3. Verzögerung bei Ausgabe

`skipLast(n)` gibt nichts aus, bis der Puffer n Werte enthält.

```ts
import { interval } from 'rxjs';
import { take, skipLast, tap } from 'rxjs';

interval(1000).pipe(
  take(5),
  tap(val => console.log('Eingabe:', val)),
  skipLast(2)
).subscribe(val => console.log('Ausgabe:', val));
// Eingabe: 0
// Eingabe: 1
// Eingabe: 2
// Ausgabe: 0  ← Ausgabe beginnt, sobald Puffer 2 Werte hat
// Eingabe: 3
// Ausgabe: 1
// Eingabe: 4
// Ausgabe: 2
// Abgeschlossen (3, 4 übersprungen)
```

### 4. Verhalten von skipLast(0)

`skipLast(0)` überspringt nichts.

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

range(0, 5).pipe(
  skipLast(0)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4 (alle ausgegeben)
```

## 💡 Praktische Kombinationsmuster

### Muster 1: Nur mittleren Teil abrufen

Erste und letzte überspringen, nur mittleren Teil abrufen

```ts
import { range } from 'rxjs';
import { skip, skipLast } from 'rxjs';

range(0, 10).pipe(
  skip(2),      // Erste 2 überspringen
  skipLast(2)   // Letzte 2 überspringen
).subscribe(console.log);
// Ausgabe: 2, 3, 4, 5, 6, 7
```

### Muster 2: Datenvalidierung

Wenn Validierung mit nachfolgenden Werten erforderlich ist

```ts
import { from } from 'rxjs';
import { skipLast, map } from 'rxjs';

interface Transaction {
  id: number;
  amount: number;
  pending: boolean;
}

const transactions$ = from([
  { id: 1, amount: 100, pending: false },
  { id: 2, amount: 200, pending: false },
  { id: 3, amount: 150, pending: false },
  { id: 4, amount: 300, pending: true },  // Unbestätigt
  { id: 5, amount: 250, pending: true }   // Unbestätigt
]);

// Unbestätigte Transaktionen (letzte 2) überspringen
transactions$.pipe(
  skipLast(2)
).subscribe(tx => {
  console.log(`Bestätigt: ID ${tx.id}, Betrag ${tx.amount}€`);
});
// Ausgabe:
// Bestätigt: ID 1, Betrag 100€
// Bestätigt: ID 2, Betrag 200€
// Bestätigt: ID 3, Betrag 150€
```

## 📚 Verwandte Operatoren

- **[skip](./skip)** - Die ersten N Werte überspringen
- **[takeLast](./takeLast)** - Nur die letzten N Werte abrufen
- **[take](./take)** - Nur die ersten N Werte abrufen
- **[skipUntil](./skipUntil)** - Überspringen, bis ein anderes Observable ausgelöst wird
- **[skipWhile](./skipWhile)** - Überspringen, solange Bedingung erfüllt ist

## Zusammenfassung

Der `skipLast`-Operator überspringt die letzten N Werte des Streams.

- ✅ Optimal, wenn die letzten N Daten nicht benötigt werden
- ✅ Nützlich zum Ausschluss unbestätigter Daten
- ✅ Gute Speichereffizienz (Puffergröße nur N)
- ✅ Erfordert Stream-Abschluss
- ⚠️ Nicht mit unendlichen Streams verwendbar
- ⚠️ Keine Ausgabe, bis Puffer N Werte hat
- ⚠️ Oft muss mit `take` ein endlicher Stream erstellt werden
