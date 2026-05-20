---
description: "Der findIndex-Operator ist ein RxJS-Filterungsoperator, der den Index des ersten Wertes zurückgibt, der die Bedingung erfüllt. Wenn er nicht gefunden wird, gibt er -1 zurück."
---

# findIndex - liefert den Index, der die Bedingung erfüllt

Der Operator `findIndex` gibt **den Index des ersten Wertes zurück, der die Bedingung erfüllt**, und schließt den Stream sofort ab. Gibt `-1` zurück, wenn kein Wert gefunden wird.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Ausgabe.: 4(erste Gerade8Index der ersten Geraden)
```

**Fluss der Operation**:.
1. 1 (Index 0) → Ungerade, Überspringen
2. 3 (Index 1) → Ungerade, überspringen
3. 5 (Index 2) → Ungerade, überspringen
4. 7 (Index 3) → Ungerade, überspringen
5. 8 (Index 4) → gerade Zahl, Ausgabe von Index 4 und Abschluss

[🌐 Offizielle RxJS Dokumentation - `findIndex`](https://rxjs.dev/api/operators/findIndex)

## 💡 Typisches Nutzungsmuster.

- **Positionierung in einem Array**: Ermitteln der Position eines Elements, das eine bestimmte Bedingung erfüllt.
- **Prüfen der Reihenfolge**: wie oft ein Element, das eine bestimmte Bedingung erfüllt, vorkommt
- **Daten neu anordnen**: Verarbeitung anhand von Indexinformationen.
- **Existenzprüfung**: prüft die Existenz eines Elements, indem es überprüft, ob es -1 ist oder nicht.

## 🧠 Praktisches Codebeispiel 1: Durchsuchen eines Arbeitsplans

Dies ist ein Beispiel für die Suche nach dem Ort einer Aufgabe mit bestimmten Bedingungen in einer Aufgabenliste.

```ts
import { from, fromEvent } from 'rxjs';
import { findIndex } from 'rxjs';

interface Task {
  id: number;
  title: string;
  priority: 'high' | 'medium' | 'low';
  completed: boolean;
}

const tasks: Task[] = [
  { id: 1, title: 'E-Mail-Antwort', priority: 'low', completed: true },
  { id: 2, title: 'Vorbereitung von Dokumenten', priority: 'medium', completed: true },
  { id: 3, title: 'Vorbereitung einer Besprechung', priority: 'high', completed: false },
  { id: 4, title: 'Überprüfung des Codes', priority: 'high', completed: false },
  { id: 5, title: 'Aktualisierung des Dokuments', priority: 'low', completed: false }
];

// UIerstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Aufgabe suchen';
container.appendChild(title);

// Anzeige der Aufgabenliste
const taskList = document.createElement('ul');
taskList.style.listStyle = 'none';
taskList.style.padding = '0';
tasks.forEach((task, index) => {
  const li = document.createElement('li');
  li.style.padding = '5px';
  li.style.borderBottom = '1px solid #eee';
  const status = task.completed ? '✅' : '⬜';
  const priorityBadge = task.priority === 'high' ? '🔴' : task.priority === 'medium' ? '🟡' : '🟢';
  li.textContent = `[${index}] ${status} ${priorityBadge} ${task.title}`;
  taskList.appendChild(li);
});
container.appendChild(taskList);

// Schaltfläche Suchen
const buttonContainer = document.createElement('div');
buttonContainer.style.marginTop = '10px';
container.appendChild(buttonContainer);

const button1 = document.createElement('button');
button1.textContent = 'Suche nach der ersten nicht erledigten Aufgabe';
button1.style.marginRight = '10px';
buttonContainer.appendChild(button1);

const button2 = document.createElement('button');
button2.textContent = 'Suche nach der ersten Aufgabe mit hoher Priorität';
buttonContainer.appendChild(button2);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Suche nach der ersten nicht erledigten Aufgabe
// NB.: Ursprünglich war das empfohlene Muster, mit switchMap Das empfohlene Muster ist die Verflachung mit
// Hier wird der Lesbarkeit Priorität eingeräumt subscribe verschachtelt (im Produktionscode switchMap empfohlen).
fromEvent(button1, 'click').subscribe(() => {
  // Verschachtelung subscribe: Ursprünglich war das empfohlene Muster, mit switchMap Flattening mit wird empfohlen
  from(tasks).pipe(
    findIndex(task => !task.completed)
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Gefunden in</strong><br>
        Position: Index ${index}<br>
        Aufgabe: ${task.title}<br>
        Priorität: ${task.priority}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Nicht beendete Aufgabe nicht gefunden';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});

// Suche nach der ersten Aufgabe mit hoher Priorität
// NB.: Ursprünglich war das empfohlene Muster, mit switchMap Das empfohlene Muster (im produktiven Code) ist die Verflachung mit switchMap empfohlen).
fromEvent(button2, 'click').subscribe(() => {
  // Verschachtelung subscribe: Ursprünglich war das empfohlene Muster, mit switchMap Flattening mit wird empfohlen
  from(tasks).pipe(
    findIndex(task => task.priority === 'high')
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Gefunden in</strong><br>
        Position: Index ${index}<br>
        Aufgabe: ${task.title}<br>
        Status der Fertigstellung: ${task.completed ? 'Erledigt' : 'Unvollendet'}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Es wurden keine Aufgaben mit hoher Priorität gefunden';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});
```

- Findet die Position der ersten Aufgabe in der Aufgabenliste, die die Bedingung erfüllt.
- Wenn nicht gefunden, wird `-1` zurückgegeben.

## 🎯 Praktisches Codebeispiel 2: Erkennung des Datenorts in Echtzeit

In diesem Beispiel wird die Position des ersten Wertes aus dem Datenstrom ermittelt, der die Bedingung erfüllt.

```ts
import { interval } from 'rxjs';
import { findIndex, map, take } from 'rxjs';

// UIerstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Datensuche in Echtzeit';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = '50Suche nach Positionen, bei denen ein Wert größer oder gleich...';
container.appendChild(status);

const dataDisplay = document.createElement('div');
dataDisplay.style.marginTop = '10px';
dataDisplay.style.padding = '10px';
dataDisplay.style.border = '1px solid #ccc';
dataDisplay.style.maxHeight = '150px';
dataDisplay.style.overflow = 'auto';
container.appendChild(dataDisplay);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.fontWeight = 'bold';
container.appendChild(result);

// Generierung von Zufallswerten (0~100)
const data$ = interval(500).pipe(
  take(20),
  map(i => ({ index: i, value: Math.floor(Math.random() * 100) }))
);

// Anzeige der Daten
data$.subscribe(data => {
  const div = document.createElement('div');
  const highlight = data.value >= 50 ? 'background-color: #fff9c4;' : '';
  div.style.cssText = `padding: 5px; ${highlight}`;
  div.textContent = `[${data.index}] Wert: ${data.value}`;
  dataDisplay.appendChild(div);
  dataDisplay.scrollTop = dataDisplay.scrollHeight;
});

// 50Suche im Index nach dem ersten Wert von mehr als
data$.pipe(
  findIndex(data => data.value >= 50)
).subscribe(index => {
  status.textContent = '';
  if (index !== -1) {
    result.innerHTML = `
      ✅ 50Mehr als oder gleich Wert gefunden<br>
      Position: Index ${index}
    `;
    result.style.color = 'green';
  } else {
    result.textContent = '❌ 50Es wurden keine Werte größer als oder gleich gefunden';
    result.style.color = 'orange';
  }
});
```

- Ermittelt die Position des ersten Wertes über 50 aus Zufallswerten, die alle 0,5 Sekunden erzeugt werden.
- Die Hervorhebung dient der visuellen Klarheit.

## 🆚 Vergleich mit ähnlichen Operatoren

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Gibt den Index des ersten Wertes zurück, der die Bedingung erfüllt
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Ausgabe.: 2Gibt den Index des ersten Wertes zurück, der die Bedingung erfüllt30Index der ersten Geraden)

// find: Gibt den ersten Wert zurück, der die Bedingung erfüllt
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Ausgabe.: 30

// elementAt: Gibt den Wert am angegebenen Index zurück
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Ausgabe.: 30
```

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Ausgabe.: 4(erste Gerade8Index der ersten Geraden)
```

## 🔄 Vergleich mit JavaScript's Array.findIndex()

RxJS `findIndex` verhält sich ähnlich wie die JavaScript Array-Methode `Array.prototype.findIndex()`.


```ts
// JavaScript Array von
const numbers = [10, 20, 30, 40, 50];
const index1 = numbers.findIndex(n => n > 25);
console.log(index1); // 2

// RxJS (gibt den ersten Wert am angegebenen Index zurück, der die Bedingung erfüllt) Observable
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log); // 2
```

**Hauptunterschiede**.
- **Array**: gibt das Ergebnis synchron und sofort zurück.
- Observable**: asynchron, wartet auf Werte, die aus dem Stream fließen.

## ⚠️ Hinweise.

### 1. gibt -1 zurück, wenn nicht gefunden

Wenn kein Wert die Bedingung erfüllt, wird anstelle eines Fehlers `-1` zurückgegeben.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('Es wurde kein Wert gefunden, der die Bedingung erfüllt');
  } else {
    console.log(`Index: ${index}`);
  }
});
// Ausgabe.: Es wurde kein Wert gefunden, der die Bedingung erfüllt
```

### 2. vollständig, wenn zuerst gefunden.

Der Stream wird abgeschlossen, sobald der erste Wert, der die Bedingung erfüllt, gefunden wird.

```ts
import { interval } from 'rxjs';
import { findIndex, tap } from 'rxjs';

interval(1000).pipe(
  tap(val => console.log(`Wert: ${val}`)),
  findIndex(n => n >= 3)
).subscribe(index => {
  console.log(`Index: ${index}`);
});
// Ausgabe.:
// Wert: 0
// Wert: 1
// Wert: 2
// Wert: 3
// Index: 3
```

### 3. typensicherheit in TypeScript

findIndex" gibt immer den Typ "Zahl" zurück.

```ts
import { Observable, from } from 'rxjs';
import { findIndex } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

function findFirstInactiveUserIndex(
  users$: Observable<User>
): Observable<number> {
  return users$.pipe(
    findIndex(user => !user.isActive)
  );
}

const users$ = from([
  { id: 1, name: 'Alice', isActive: true },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true }
]);

findFirstInactiveUserIndex(users$).subscribe(index => {
  // index ist ein Array von number Typ
  if (index !== -1) {
    console.log(`Der erste inaktive Benutzer ist index ${index} ist.`);
  }
});
// Ausgabe.: Der erste inaktive Benutzer ist index 1 ist.
```

### 4. index beginnt bei 0

Wie bei Arrays beginnen die Indizes bei 0.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// Ausgabe.: 0(erstes Element)
```

## 📚 Verwandte Operatoren.

- **[find](. /find)** - Ermittelt den ersten Wert, der die Bedingung erfüllt.
- **[elementAt](. /elementAt)** - Ermittelt den Wert am angegebenen Index.
- **[first](. /first)** - Ermittelt den ersten Wert.
- **[filter](. /filter)** - liefert alle Werte, die die Bedingung erfüllen

## Zusammenfassung.

Der Operator `findIndex` gibt den Index des ersten Wertes zurück, der die Bedingung erfüllt.

- ✅ Ähnliches Verhalten wie JavaScript's `Array.findIndex()`.
- ✅ Ideal, wenn Indexinformationen benötigt werden
- ✅ Gibt `-1` zurück, wenn nicht gefunden (kein Fehler)
- ✅ Wird sofort abgeschlossen, wenn gefunden
- ⚠️ Rückgabewert ist immer vom Typ `Zahl` (-1 oder eine ganze Zahl größer oder gleich 0)
- ⚠️ Use `find` if the value itself is needed
