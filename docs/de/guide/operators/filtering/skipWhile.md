---
description: Der skipWhile-Operator überspringt Werte, solange eine angegebene Bedingung erfüllt ist, und gibt alle Werte aus, sobald die Bedingung false wird. Nützlich, wenn Sie Streams mit dynamischen Startbedingungen steuern möchten.
---

# skipWhile - Überspringen Solange Wahr

Der `skipWhile`-Operator **überspringt Werte weiterhin, solange die angegebene Bedingung erfüllt ist**, und gibt **alle Werte aus**, sobald die Bedingung `false` wird.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0 bis 9

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9
```

**Ablauf**:
1. 0 wird ausgegeben → `0 < 5` ist `true` → Überspringen
2. 1 wird ausgegeben → `1 < 5` ist `true` → Überspringen
3. 2 wird ausgegeben → `2 < 5` ist `true` → Überspringen
4. 3 wird ausgegeben → `3 < 5` ist `true` → Überspringen
5. 4 wird ausgegeben → `4 < 5` ist `true` → Überspringen
6. 5 wird ausgegeben → `5 < 5` ist `false` → Ausgabe beginnt
7. 6 und später → Alle ausgeben (Bedingung wird nicht neu bewertet)

[🌐 Offizielle RxJS-Dokumentation - `skipWhile`](https://rxjs.dev/api/operators/skipWhile)

## 💡 Typische Anwendungsmuster

- **Überspringen von anfänglichen unnötigen Daten**: Ausschluss von Daten aus der Aufwärmphase
- **Überspringen bis Schwellenwert**: Warten bis eine bestimmte Bedingung erfüllt ist
- **Überspringen von Kopfzeilen**: Ausschluss von Kopfzeilen wie in CSV-Dateien
- **Überspringen der Vorbereitungszeit**: Warten bis die Systemvorbereitung abgeschlossen ist

## 🆚 Vergleich mit ähnlichen Operatoren

### skipWhile vs takeWhile vs skip vs filter

```ts
import { range } from 'rxjs';
import { skipWhile, takeWhile, skip, filter } from 'rxjs';

const numbers$ = range(0, 10); // 0 bis 9

// skipWhile: Überspringen, solange Bedingung erfüllt, danach alle ausgeben
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9

// takeWhile: Nur abrufen, solange Bedingung erfüllt
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4

// skip: Die ersten N überspringen
numbers$.pipe(
  skip(5)
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9

// filter: Nur Werte durchlassen, die Bedingung erfüllen (gesamte Bewertung)
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9
```

| Operator | Verhalten | Erneute Bewertung der Bedingung | Abschluss-Timing |
|:---|:---|:---|:---|
| `skipWhile(predicate)` | Überspringen, solange Bedingung erfüllt | Nein (endet, sobald einmal false) | Bei Abschluss des ursprünglichen Streams |
| `takeWhile(predicate)` | Abrufen, solange Bedingung erfüllt | Jedes Mal bewerten | Wenn Bedingung false wird |
| `skip(n)` | Die ersten n überspringen | Nein (anzahlbasiert) | Bei Abschluss des ursprünglichen Streams |
| `filter(predicate)` | Nur Werte, die Bedingung erfüllen | **Jedes Mal bewerten** | Bei Abschluss des ursprünglichen Streams |

**Visueller Unterschied**:

```
Eingabe: 0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0

skipWhile(n => n < 5):
[0,1,2,3,4 überspringen] | 5, 4, 3, 2, 1, 0
                         ^Nach false werden alle ausgegeben

filter(n => n >= 5):
[0,1,2,3,4 ausschließen] 5 [4,3,2,1,0 ausschließen]
                         ^Nur Werte ausgeben, die Bedingung erfüllen (jedes Mal bewerten)

takeWhile(n => n < 5):
0, 1, 2, 3, 4 | [5 und später alle ignorieren und abschließen]
```

## ⚠️ Hinweise

### 1. Bedingung wird nicht neu bewertet, sobald einmal false

Dies ist der größte Unterschied zu `filter`.

```ts
import { from } from 'rxjs';
import { skipWhile, filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 4, 3, 2, 1]);

// skipWhile: Sobald Bedingung false wird, danach alle ausgeben
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(val => console.log('skipWhile:', val));
// Ausgabe: skipWhile: 5, 4, 3, 2, 1 (alle nach 5 ausgegeben)

// filter: Bedingung jedes Mal bewerten
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(val => console.log('filter:', val));
// Ausgabe: filter: 5 (nur 5 ausgegeben)
```

### 2. Wenn Bedingung von Anfang an false

Wenn die Bedingung von Anfang an `false` ist, werden alle Werte ausgegeben.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(5, 5).pipe( // 5 bis 9
  skipWhile(n => n < 3) // Bedingung von Anfang an false
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9 (alle ausgegeben)
```

### 3. Wenn alle Werte die Bedingung erfüllen

Wenn alle Werte die Bedingung erfüllen, wird nichts ausgegeben.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(0, 5).pipe( // 0 bis 4
  skipWhile(n => n < 10) // Alle Werte erfüllen Bedingung
).subscribe({
  next: console.log,
  complete: () => console.log('Abgeschlossen (nichts ausgegeben)')
});
// Ausgabe: Abgeschlossen (nichts ausgegeben)
```

### 4. Typ in TypeScript

`skipWhile` ändert den Typ nicht.

```ts
import { Observable, from } from 'rxjs';
import { skipWhile } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

const users$: Observable<User> = from([
  { id: 1, name: 'Alice', isActive: false },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true },
  { id: 4, name: 'Dave', isActive: true }
]);

// Typ bleibt Observable<User>
const activeUsers$: Observable<User> = users$.pipe(
  skipWhile(user => !user.isActive)
);

activeUsers$.subscribe(user => {
  console.log(`${user.name} (ID: ${user.id})`);
});
// Ausgabe: Charlie (ID: 3), Dave (ID: 4)
```

## 🎓 Zusammenfassung

Der `skipWhile`-Operator überspringt Werte, solange eine angegebene Bedingung erfüllt ist, und gibt alle Werte aus, sobald die Bedingung false wird.

- ✅ Optimal zum Überspringen anfänglicher unnötiger Daten
- ✅ Bedingung wird nicht neu bewertet, sobald einmal false
- ✅ Nützlich zum Überspringen von Aufwärm- oder Vorbereitungszeiten
- ✅ Verwendbar zum Überspringen von Kopfzeilen
- ⚠️ Anders als `filter` wird Bedingung nur einmal bewertet
- ⚠️ Wenn alle Werte die Bedingung erfüllen, wird nichts ausgegeben
- ⚠️ Läuft weiter bis der ursprüngliche Stream abgeschlossen wird

## 🚀 Nächste Schritte

- **[takeWhile](./takeWhile)** - Lernen Sie, wie man Werte abruft, solange Bedingung erfüllt ist
- **[skip](./skip)** - Lernen Sie, wie man die ersten N Werte überspringt
- **[skipLast](./skipLast)** - Lernen Sie, wie man die letzten N Werte überspringt
- **[skipUntil](./skipUntil)** - Lernen Sie, wie man überspringt, bis ein anderes Observable ausgelöst wird
- **[filter](./filter)** - Lernen Sie, wie man Werte durchlässt, die Bedingung erfüllen
