---
description: "takeWhile ist ein RxJS-Filteroperator, der Werte abruft, solange eine angegebene Bedingung erfüllt ist, und den Stream abschließt, sobald die Bedingung false wird. Ideal für Szenarien wie Datenabruf bis zu einem Schwellenwert, prioritätsbasierte Verarbeitung oder Paging, bei denen Sie Streams mit dynamischen Bedingungen steuern möchten. Mit der Option inclusive können auch Werte einbezogen werden, bei denen die Bedingung false wird."
---

# takeWhile - Werte abrufen, solange Bedingung erfüllt ist

Der `takeWhile`-Operator ruft Werte ab, **solange die angegebene Bedingung erfüllt ist**, und schließt den Stream ab, sobald die Bedingung `false` wird.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { takeWhile } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('Abgeschlossen')
});
// Ausgabe: 0, 1, 2, 3, 4, Abgeschlossen
```

**Ablauf**:
1. 0 wird ausgegeben → `0 < 5` ist `true` → Ausgabe
2. 1 wird ausgegeben → `1 < 5` ist `true` → Ausgabe
3. 2 wird ausgegeben → `2 < 5` ist `true` → Ausgabe
4. 3 wird ausgegeben → `3 < 5` ist `true` → Ausgabe
5. 4 wird ausgegeben → `4 < 5` ist `true` → Ausgabe
6. 5 wird ausgegeben → `5 < 5` ist `false` → Abgeschlossen (5 wird nicht ausgegeben)

[🌐 Offizielle RxJS-Dokumentation - `takeWhile`](https://rxjs.dev/api/operators/takeWhile)


## 🆚 Vergleich mit take

`take` und `takeWhile` unterscheiden sich in ihren Abrufbedingungen.

```ts
import { interval } from 'rxjs';
import { take, takeWhile } from 'rxjs';

const source$ = interval(1000);

// take: Steuerung nach Anzahl
source$.pipe(
  take(5)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4

// takeWhile: Steuerung nach Bedingung
source$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4
```

| Operator | Steuerungsmethode | Abschlussbedingung | Letzter Wert |
|---|---|---|---|
| `take(n)` | Anzahl | Nach Abruf von n Werten | Einschließlich n-tem Wert |
| `takeWhile(predicate)` | Bedingungsfunktion | Wenn Bedingung `false` wird | Ohne Wert, bei dem `false` wird* |

\* Standardmäßig wird der Wert, bei dem `false` wird, nicht ausgegeben, kann aber mit `inclusive: true` einbezogen werden


## 🎯 Option inclusive

Um auch den Wert einzubeziehen, bei dem die Bedingung `false` wird, geben Sie `inclusive: true` an.

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

const numbers$ = range(0, 10);

// Standard (inclusive: false)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4

// inclusive: true
numbers$.pipe(
  takeWhile(n => n < 5, true)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5 (einschließlich 5, wo Bedingung false wird)
```


## 💡 Typische Anwendungsmuster

1. **Datenabruf bis zu einem Schwellenwert**
   ```ts
   import { interval } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   // Temperatursensor-Simulation
   const temperature$ = interval(100).pipe(
     map(() => 20 + Math.random() * 15)
   );

   // Nur aufzeichnen, solange unter 30 Grad
   temperature$.pipe(
     takeWhile(temp => temp < 30)
   ).subscribe({
     next: temp => console.log(`Temperatur: ${temp.toFixed(1)}°C`),
     complete: () => console.log('Warnung: Temperatur hat 30 Grad überschritten!')
   });
   ```

2. **Bedingte Verarbeitung von Arrays**
   ```ts
   import { from } from 'rxjs';
   import { takeWhile } from 'rxjs';

   interface Task {
     id: number;
     priority: 'high' | 'medium' | 'low';
     completed: boolean;
   }

   const tasks$ = from([
     { id: 1, priority: 'high' as const, completed: false },
     { id: 2, priority: 'high' as const, completed: false },
     { id: 3, priority: 'medium' as const, completed: false },
     { id: 4, priority: 'low' as const, completed: false },
   ] as Task[]);

   // Nur verarbeiten, solange Priorität high ist
   tasks$.pipe(
     takeWhile(task => task.priority === 'high')
   ).subscribe(task => {
     console.log(`Verarbeite Aufgabe ${task.id}`);
   });
   // Ausgabe: Verarbeite Aufgabe 1, Verarbeite Aufgabe 2
   ```

3. **Paging-Verarbeitung**
   ```ts
   import { range } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   interface Page {
     pageNumber: number;
     hasMore: boolean;
   }

   const pages$ = range(1, 10).pipe(
     map(pageNum => ({
       pageNumber: pageNum,
       hasMore: pageNum < 5
     } as Page))
   );

   // Seiten nur laden, solange hasMore true ist
   pages$.pipe(
     takeWhile(page => page.hasMore, true) // inclusive: true
   ).subscribe(page => {
     console.log(`Lade Seite ${page.pageNumber}`);
   });
   // Ausgabe: Lade Seite 1~5
   ```


## 🧠 Praxisbeispiel (Count-up mit Limit)

Beispiel, das das Hochzählen fortsetzt, bis eine bestimmte Bedingung erreicht ist.

```ts
import { fromEvent, interval } from 'rxjs';
import { takeWhile, scan, switchMap } from 'rxjs';

// UI-Elemente erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const startButton = document.createElement('button');
startButton.textContent = 'Zählen starten';
container.appendChild(startButton);

const counter = document.createElement('div');
counter.style.fontSize = '24px';
counter.style.marginTop = '10px';
counter.textContent = 'Zähler: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = 'Zählt weiter, solange unter 10';
container.appendChild(message);

// Zählen bei Buttonklick starten
fromEvent(startButton, 'click').pipe(
  switchMap(() =>
    interval(500).pipe(
      scan(count => count + 1, 0),
      takeWhile(count => count < 10)
    )
  )
).subscribe({
  next: (count) => {
    counter.textContent = `Zähler: ${count}`;
    startButton.disabled = true;
  },
  complete: () => {
    message.textContent = 'Bei 10 angekommen, abgeschlossen!';
    message.style.color = 'green';
    startButton.disabled = false;
  }
});
```

Dieser Code zählt von 0 bis 9 hoch und wird automatisch abgeschlossen, kurz bevor 10 erreicht wird.


## 🎯 Vergleich mit skipWhile

`takeWhile` und `skipWhile` verhalten sich gegensätzlich.

```ts
import { range } from 'rxjs';
import { takeWhile, skipWhile } from 'rxjs';

const numbers$ = range(0, 10);

// takeWhile: Abrufen, solange Bedingung erfüllt
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4

// skipWhile: Überspringen, solange Bedingung erfüllt
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 5, 6, 7, 8, 9
```

| Operator | Verhalten | Abschluss-Timing |
|---|---|---|
| `takeWhile(predicate)` | **Abrufen**, solange Bedingung erfüllt | Wenn Bedingung `false` wird |
| `skipWhile(predicate)` | **Überspringen**, solange Bedingung erfüllt | Bei Abschluss des ursprünglichen Streams |


## 📋 Typsichere Verwendung

Beispiel für typsichere Implementierung mit TypeScript-Generics.

```ts
import { Observable, from } from 'rxjs';
import { takeWhile } from 'rxjs';

interface SensorReading {
  timestamp: Date;
  value: number;
  unit: string;
  status: 'normal' | 'warning' | 'critical';
}

function getReadingsUntilWarning(
  readings$: Observable<SensorReading>
): Observable<SensorReading> {
  return readings$.pipe(
    takeWhile(reading => reading.status === 'normal')
  );
}

// Verwendungsbeispiel
const readings$ = from([
  { timestamp: new Date(), value: 25, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 28, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 32, unit: '°C', status: 'warning' as const },
  { timestamp: new Date(), value: 35, unit: '°C', status: 'critical' as const },
] as SensorReading[]);

getReadingsUntilWarning(readings$).subscribe(reading => {
  console.log(`${reading.value}${reading.unit} - ${reading.status}`);
});
// Ausgabe:
// 25°C - normal
// 28°C - normal
```


## 🔄 Unterschied zwischen takeWhile und filter

`takeWhile` unterscheidet sich von `filter` dadurch, dass es abschließt.

```ts
import { range } from 'rxjs';
import { takeWhile, filter } from 'rxjs';

const numbers$ = range(0, 10);

// filter: Nur Werte, die Bedingung erfüllen, durchlassen (Stream läuft weiter)
numbers$.pipe(
  filter(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter abgeschlossen')
});
// Ausgabe: 0, 1, 2, 3, 4, filter abgeschlossen

// takeWhile: Nur solange Bedingung erfüllt (Abschluss bei false)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('takeWhile abgeschlossen')
});
// Ausgabe: 0, 1, 2, 3, 4, takeWhile abgeschlossen
```

| Operator | Verhalten | Stream-Abschluss |
|---|---|---|
| `filter(predicate)` | Nur Werte durchlassen, die Bedingung erfüllen | Bei Abschluss des ursprünglichen Streams |
| `takeWhile(predicate)` | Abrufen, solange Bedingung erfüllt | Wenn Bedingung `false` wird |


## ⚠️ Häufige Fehler

> [!NOTE]
> Wenn die Bedingung bei `takeWhile` von Anfang an `false` ist, wird nichts ausgegeben und der Stream wird abgeschlossen. Überprüfen Sie, ob die Bedingung richtig eingestellt ist.

### Falsch: Bedingung von Anfang an false

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ❌ Schlechtes Beispiel: Bedingung ist beim ersten Wert false
range(5, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Keine Ausgabe (Bedingung ist beim ersten Wert 5 false)
```

### Richtig: Bedingung überprüfen

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ✅ Gutes Beispiel: Bedingung richtig einstellen
range(0, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4
```


## 🎓 Zusammenfassung

### Wann takeWhile verwenden
- ✅ Wenn Sie Streams mit dynamischen Bedingungen steuern möchten
- ✅ Wenn Sie Daten bis zu einem Schwellenwert abrufen möchten
- ✅ Wenn Sie nur während eines bestimmten Zustands verarbeiten möchten
- ✅ Wenn Sie einen bedingungsbasierten frühen Abschluss benötigen

### Wann take verwenden
- ✅ Wenn die Anzahl der abzurufenden Werte feststeht
- ✅ Wenn Sie eine einfache Anzahlbegrenzung benötigen

### Wann filter verwenden
- ✅ Wenn Sie nur Werte extrahieren möchten, die eine Bedingung erfüllen
- ✅ Wenn Sie den Stream nicht abschließen möchten

### Vorsicht
- ⚠️ Wenn die Bedingung von Anfang an `false` ist, wird nichts ausgegeben und abgeschlossen
- ⚠️ Standardmäßig wird der Wert, bei dem die Bedingung `false` wird, nicht ausgegeben (kann mit `inclusive: true` einbezogen werden)
- ⚠️ Bei unendlichen Streams läuft es ewig weiter, wenn die Bedingung immer `true` ist


## 🚀 Nächste Schritte

- **[take](./take)** - Lernen Sie, wie man die ersten N Werte abruft
- **[takeLast](./takeLast)** - Lernen Sie, wie man die letzten N Werte abruft
- **[filter](./filter)** - Lernen Sie, wie man basierend auf Bedingungen filtert
- **[Praktische Beispiele für Filteroperatoren](./practical-use-cases)** - Lernen Sie reale Anwendungsfälle
