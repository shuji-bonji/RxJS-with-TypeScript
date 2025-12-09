---
description: Der skip-Operator überspringt die angegebene Anzahl der ersten Werte eines Observable-Streams und gibt nur die nachfolgenden Werte aus. Nützlich, wenn Sie Ausgangsdaten ignorieren oder eine Aufwärmphase überspringen möchten.
---

# skip - Die ersten N Werte überspringen

Der `skip`-Operator überspringt **die angegebene Anzahl** der ersten Werte eines Streams und gibt nur die nachfolgenden Werte aus.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  skip(3)
).subscribe(console.log);
// Ausgabe: 3, 4, 5, 6, 7, ...
```

- Die ersten 3 Werte (0, 1, 2) werden übersprungen
- Ab dem 4. Wert (3, 4, 5, ...) werden alle ausgegeben
- Der Stream wird zum ursprünglichen Abschlusszeitpunkt abgeschlossen

[🌐 Offizielle RxJS-Dokumentation - `skip`](https://rxjs.dev/api/operators/skip)


## 🆚 Vergleich mit take

`skip` und `take` verhalten sich gegensätzlich.

```ts
import { range } from 'rxjs';
import { skip, take } from 'rxjs';

const numbers$ = range(0, 10); // 0 bis 9

// take: Die ersten N abrufen
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Ausgabe: 0, 1, 2

// skip: Die ersten N überspringen
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Ausgabe: 3, 4, 5, 6, 7, 8, 9

// Kombination: Die ersten 3 überspringen, dann die nächsten 3 abrufen
numbers$.pipe(
  skip(3),
  take(3)
).subscribe(console.log);
// Ausgabe: 3, 4, 5
```

| Operator | Verhalten | Abschluss-Timing |
|---|---|---|
| `take(n)` | Die ersten n abrufen | Automatischer Abschluss nach n Werten |
| `skip(n)` | Die ersten n überspringen | Bei Abschluss des ursprünglichen Streams |


## 💡 Typische Anwendungsmuster

1. **Anfangswert überspringen**
   ```ts
   import { BehaviorSubject } from 'rxjs';
   import { skip } from 'rxjs';

   const state$ = new BehaviorSubject<number>(0);

   // Anfangswert überspringen, nur Änderungen überwachen
   state$.pipe(
     skip(1)
   ).subscribe(value => {
     console.log(`Status geändert: ${value}`);
   });

   state$.next(1); // Ausgabe: Status geändert: 1
   state$.next(2); // Ausgabe: Status geändert: 2
   ```

2. **Aufwärmphase überspringen**
   ```ts
   import { interval } from 'rxjs';
   import { skip, map } from 'rxjs';

   // Sensordaten-Simulation
   const sensorData$ = interval(100).pipe(
     map(() => Math.random() * 100)
   );

   // Die ersten 10 Einträge (1 Sekunde) als Kalibrierungszeit überspringen
   sensorData$.pipe(
     skip(10)
   ).subscribe(data => {
     console.log(`Sensorwert: ${data.toFixed(2)}`);
   });
   ```

3. **Paginierung**
   ```ts
   import { from } from 'rxjs';
   import { skip, take } from 'rxjs';

   interface Item {
     id: number;
     name: string;
   }

   const allItems$ = from([
     { id: 1, name: 'Item 1' },
     { id: 2, name: 'Item 2' },
     { id: 3, name: 'Item 3' },
     { id: 4, name: 'Item 4' },
     { id: 5, name: 'Item 5' },
     { id: 6, name: 'Item 6' },
   ] as Item[]);

   const pageSize = 2;
   const pageNumber = 2; // 0-indiziert

   // Elemente von Seite 2 abrufen (Items 5 und 6)
   allItems$.pipe(
     skip(pageNumber * pageSize),
     take(pageSize)
   ).subscribe(item => {
     console.log(item);
   });
   // Ausgabe: { id: 5, name: 'Item 5' }, { id: 6, name: 'Item 6' }
   ```


## 🧠 Praxisbeispiel (Zähler)

Beispiel, bei dem die ersten 3 Klicks übersprungen und erst ab dem 4. Klick gezählt wird.

```ts
import { fromEvent } from 'rxjs';
import { skip, scan } from 'rxjs';

// UI-Elemente erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const button = document.createElement('button');
button.textContent = 'Klicken';
container.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'Zähler: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = 'Die ersten 3 Klicks werden übersprungen';
container.appendChild(message);

// Klickereignis
fromEvent(button, 'click').pipe(
  skip(3), // Die ersten 3 überspringen
  scan((count) => count + 1, 0)
).subscribe(count => {
  counter.textContent = `Zähler: ${count}`;
  if (count === 1) {
    message.textContent = 'Ab dem 4. Klick wird gezählt!';
    message.style.color = 'green';
  }
});
```

Dieser Code ignoriert die ersten 3 Klicks und beginnt ab dem 4. Klick mit der Zählung bei "1".


## 🎯 Unterschied zwischen skip und skipWhile

```ts
import { of } from 'rxjs';
import { skip, skipWhile } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6);

// skip: Die ersten N nach Anzahl angeben
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Ausgabe: 4, 5, 6

// skipWhile: Überspringen, solange Bedingung erfüllt
numbers$.pipe(
  skipWhile(n => n < 4)
).subscribe(console.log);
// Ausgabe: 4, 5, 6
```

| Operator | Überspringbedingung | Anwendungsfall |
|---|---|---|
| `skip(n)` | Die ersten n nach Anzahl überspringen | Feste Anzahl überspringen |
| `skipWhile(predicate)` | Überspringen, solange Bedingung erfüllt | Bedingungsbasiertes Überspringen |
| `skipUntil(notifier$)` | Überspringen, bis ein anderes Observable ausgelöst wird | Zeitbasiertes Überspringen |


## 📋 Typsichere Verwendung

Beispiel für typsichere Implementierung mit TypeScript-Generics.

```ts
import { Observable, from } from 'rxjs';
import { skip, take } from 'rxjs';

interface User {
  id: number;
  name: string;
  role: 'admin' | 'user';
}

function getPaginatedUsers(
  users$: Observable<User>,
  page: number,
  pageSize: number
): Observable<User> {
  return users$.pipe(
    skip(page * pageSize),
    take(pageSize)
  );
}

// Verwendungsbeispiel
const users$ = from([
  { id: 1, name: 'Alice', role: 'admin' as const },
  { id: 2, name: 'Bob', role: 'user' as const },
  { id: 3, name: 'Charlie', role: 'user' as const },
  { id: 4, name: 'Dave', role: 'admin' as const },
  { id: 5, name: 'Eve', role: 'user' as const },
] as User[]);

// Seite 1 (zweite Seite, 0-indiziert) abrufen
getPaginatedUsers(users$, 1, 2).subscribe(user => {
  console.log(`${user.name} (${user.role})`);
});
// Ausgabe: Charlie (user), Dave (admin)
```


## ⚠️ Häufige Fehler

> [!NOTE]
> `skip` überspringt nur die ersten N Werte und schließt den Stream nicht ab. Bei unendlichen Streams verwenden Sie `take` in Kombination, um eine Abschlussbedingung festzulegen.

### Falsch: Nur skip bei unendlichem Stream verwenden

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

// ❌ Schlechtes Beispiel: Unendlicher Stream läuft weiter
interval(1000).pipe(
  skip(5)
).subscribe(console.log);
// 5, 6, 7, 8, ... läuft ewig weiter
```

### Richtig: Mit take eine Abschlussbedingung festlegen

```ts
import { interval } from 'rxjs';
import { skip, take } from 'rxjs';

// ✅ Gutes Beispiel: Nach dem Überspringen die Anzahl begrenzen
interval(1000).pipe(
  skip(5),
  take(3)
).subscribe({
  next: console.log,
  complete: () => console.log('Abgeschlossen')
});
// 5, 6, 7, Abgeschlossen
```


## 🎓 Zusammenfassung

### Wann skip verwenden
- ✅ Wenn Anfangswerte oder die ersten N Daten ignoriert werden sollen
- ✅ Wenn der Anfangswert von BehaviorSubject übersprungen werden soll
- ✅ Wenn Daten einer bestimmten Seite für Paginierung abgerufen werden sollen
- ✅ Wenn die Kalibrierungszeit von Sensoren übersprungen werden soll

### Mit take kombinieren, wenn
- ✅ Nur Daten in einem bestimmten Bereich abgerufen werden sollen
- ✅ Mittlere Daten aus einem unendlichen Stream abgerufen werden sollen

### Vorsicht
- ⚠️ Bei unendlichen Streams mit `take` eine Abschlussbedingung festlegen
- ⚠️ `skip(0)` verhält sich wie der ursprüngliche Stream (überspringt nichts)
- ⚠️ Wenn die Anzahl der zu überspringenden Werte größer als die Gesamtanzahl ist, wird nichts ausgegeben und abgeschlossen


## 🚀 Nächste Schritte

- **[take](./take)** - Lernen Sie, wie man die ersten N Werte abruft
- **[first](./first)** - Lernen Sie, wie man den ersten Wert oder den ersten Wert, der eine Bedingung erfüllt, abruft
- **[last](./last)** - Lernen Sie, wie man den letzten Wert abruft
- **[filter](./filter)** - Lernen Sie, wie man basierend auf Bedingungen filtert
- **[Praktische Beispiele für Filteroperatoren](./practical-use-cases)** - Lernen Sie reale Anwendungsfälle
