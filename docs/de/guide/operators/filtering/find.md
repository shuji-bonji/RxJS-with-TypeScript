---
description: "find ist ein RxJS-Filteroperator, der den ersten Wert findet, der eine Bedingung erfüllt, und diesen ausgibt, wodurch der Stream sofort vervollständigt wird. Er eignet sich ideal für Situationen, in denen Sie ein bestimmtes Element aus einem Array oder einer Liste finden möchten, z. B. bei der Suche nach Benutzern, der Überprüfung des Inventars oder der Erkennung von Fehlerprotokollen. Wenn kein Wert gefunden wird, wird ein undefinierter Wert ausgegeben, und in TypeScript ist der Rückgabewert vom Typ T | undefiniert."
---

# find - findet den ersten Wert, der die Bedingung erfüllt

Der Operator "find" findet und gibt den **ersten Wert aus, der die Bedingung erfüllt** und schließt den Stream sofort ab. Wenn kein Wert gefunden wird, gibt er `undefined` aus.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Ausgabe.: 8(erste gerade Zahl)
```

**Ablauf der Operation**:.
1. check 1, 3, 5, 7 → Bedingung nicht erfüllt
2. check 8 → Bedingung erfüllt → Ausgabe 8 und vollständig
3. 9, 10 nicht ausgewertet

[🌐 Offizielle RxJS Dokumentation - `find`](https://rxjs.dev/api/operators/find)

## 🆚 Kontrast zu first

`find` und `first` sind ähnlich, aber ihre Verwendung ist unterschiedlich.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: Erster Wert, der die Bedingung erfüllt (Bedingung ist optional)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Ausgabe.: 7

// find: Erster Wert, der die Bedingung erfüllt (Bedingung ist obligatorisch)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Ausgabe.: 7
```

| Bediener. | Angabe der Bedingung | Wenn kein Wert gefunden wird | Anwendungsfall. |
|---|---|---|---|
| `first()` | Option | Fehler (`EmptyError`) | Ermittelt den ersten Wert |
| first(Prädikat)` | Optional | Fehler (`EmptyError`) | Bedingtes Erhalten. |
| find(Prädikat)` | Erforderlich. | Ausgabe `Undefiniert`. | Suche und Existenzprüfung |

## 💡 Typisches Nutzungsmuster

1. **Benutzersuche**.

```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface User {
     id: number;
     name: string;
     email: string;
   }

   const users$ = from([
     { id: 1, name: 'Alice', email: 'alice@example.com' },
     { id: 2, name: 'Bob', email: 'bob@example.com' },
     { id: 3, name: 'Charlie', email: 'charlie@example.com' }
   ] as User[]);

   // ID(Bedingung ist optional)2Suche nach Benutzern mit
   users$.pipe(
     find(user => user.id === 2)
   ).subscribe(user => {
     if (user) {
       console.log(`Gefunden: ${user.name}`);
     } else {
       console.log('Benutzer nicht gefunden');
     }
   });
   // Ausgabe.: Gefunden: Bob
   ```

2. **Inventar prüfen**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface Product {
     id: string;
     name: string;
     stock: number;
   }

   const products$ = from([
     { id: 'A1', name: 'NotebookPC', stock: 0 },
     { id: 'A2', name: 'Maus', stock: 15 },
     { id: 'A3', name: 'Tastaturen', stock: 8 }
   ] as Product[]);

   // Finde heraus, was nicht vorrätig ist
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`Nicht auf Lager: ${product.name}`);
     } else {
       console.log('Alle auf Lager');
     }
   });
   // Ausgabe.: Nicht auf Lager: NotebookPC
   ```

3. **Suche im Fehlerprotokoll**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 4, level: 'info' as const, message: 'Retry successful' }
   ] as LogEntry[]);

   // Suche nach dem ersten Fehler
   logs$.pipe(
     find(log => log.level === 'error')
   ).subscribe(log => {
     if (log) {
       console.log(`Fehlererkennung: ${log.message} (Zeit: ${log.timestamp})`);
     }
   });
   // Ausgabe.: Fehlererkennung: Connection failed (Zeit: 3)
   ```

## 🧠 Praktisches Code-Beispiel (Produktsuche)

Dies ist ein Beispiel für die Suche nach Produkten aus dem Bestand, die bestimmten Kriterien entsprechen.

```

ts.
import { from, fromEvent } from 'rxjs';
importiere { find } von 'rxjs';

Schnittstelle Product {
  id: string;
  name: string;
  Preis: Zahl;
  Kategorie: string;
}

const products: Product[] = [
  { id: 'P1', Name: 'Kabellose Maus', Preis: 2980, Kategorie: 'PC-Peripheriegeräte' }
  { id: 'P2', name: 'Mechanische Tastatur', preis: 8980, kategorie: 'PC-Peripheriegeräte' }
  { id: 'P3', name: 'USB-Speicherstick 64GB', preis: 1480, kategorie: 'Speicher' }
  { id: 'P4', name: 'Monitor 27-Zoll', preis: 29800, kategorie: 'Bildschirme' }
  { id: 'P5', name: 'Laptop-Ständer', preis: 3980, kategorie: 'PC-Peripheriegeräte' }
];

// Erstellen von UI-Elementen
const container = document.createElement('div');.
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Produktsuche';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'Zahl';
input.placeholder = 'Höchstpreis eingeben';
input.style.marginRight = '10px';
container.appendChild(input);

const searchButton = document.createElement('button');
searchButton.textContent = 'suche';
container.appendChild(searchButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// Suchverarbeitung
// Hinweis: Ursprünglich ist das empfohlene Muster, mit einer switchMap zu flatten, aber,
// Hinweis: Obwohl das empfohlene Muster die Verflachung mit einer switchMap ist, // verschachteln wir hier das subscribe aus Gründen der Lesbarkeit, // weil es eine UI-Validierung (frühe Rückkehr) enthält.
// Betrachten Sie eine flache Implementierung mit `switchMap` im Produktionscode.
fromEvent(searchButton, 'click').subscribe(() => {
  const maxPrice = parseInt(input.value);.

  if (isNaN(maxPreis)) {
    result.textContent = 'Bitte geben Sie einen Preis ein';
    result.style.color = 'rot';
    zurück;
  }

  // Nest subscribe: ursprünglich empfohlen, mit switchMap zu flatten
  from(Produkte).pipe(
    find(produkt => produkt.preis <= maxPreis)
  ).subscribe(produkt => {
    if (produkt) {
      result.innerHTML = `
        <strong>Gefunden! </strong><br>
        Produktname: ${Produkt.Name}<br>
        Preis: ${Produkt.Preis.toLocaleString()}<br>
        Kategorie: ${product.category}
      `;
      result.style.color = 'grün';
    } else {
      result.textContent = `¥${maxPrice.toLocaleString()} oder weniger Produkt nicht gefunden `;
      result.style.color = 'orange'; }
    }
  });
});

```

Dieser Code sucht nach dem ersten Produkt unter dem vom Benutzer eingegebenen Preis und zeigt es an.

## 🎯 filter Der Unterschied zwischen

`find` und `filter` werden für unterschiedliche Zwecke verwendet.

```

ts.
import { from } from 'rxjs';
importiere { find, filter } von 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// filter: Ausgabe aller Werte, die die Bedingung erfüllen
numbers$.pipe(
  filter(n => n > 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('filter complete')
});
// Ausgabe: 7, 8, 9, 10, filter complete

// find: Ausgabe nur des ersten Wertes, der die Bedingung erfüllt
numbers$.pipe(
  find(n => n > 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('find complete')
});
// Ausgabe: 7, find complete

```

| Operator | Anzahl der Ausgaben | Zeitpunkt der Fertigstellung | Anwendungsfall |
|---|---|---|---|
| `filter(predicate)` | Alle Werte, die die Bedingung erfüllen | Bei Abschluss des ursprünglichen Datenstroms | Verfeinerung der Daten |
| `find(predicate)` | Nur der erste Wert, der die Kriterien erfüllt | Unmittelbar nach der Entdeckung | Suche und Existenzprüfung |

## 📋 Typsichere Verwendung

TypeScript Dies ist ein Beispiel für eine typsichere Implementierung, die die Generika in

```

ts.
import { Observable, from } from 'rxjs';
importiere { find } von 'rxjs';

Schnittstelle Task {
  id: Zahl;
  title: string;
  completed: boolescher Wert;
  Priorität: 'hoch' | 'mittel' | 'niedrig'; }
}

function findTaskById(
  tasks$: Observable,.
  id: Zahl
): Observable | undefined> {
  return tasks$.pipe(
    find(aufgabe => aufgabe.id === id)
  );
}

function findFirstIncompleteTask(
  tasks$: Observable
): Observable | undefined> {
  return tasks$.pipe(
    find(task => !task.completed)
  );
}

// Beispiel für die Verwendung
const tasks$ = from([.
  { id: 1, title: 'Aufgabe A', completed: true, priority: 'high' as const }
  { id: 2, title: 'Aufgabe B', completed: false, priority: 'medium' as const }
  { id: 3, title: 'Aufgabe C', completed: false, priority: 'low' as const }
] as Task[]);.

// Suche nach ID
findTaskById(tasks$, 2).subscribe(task => {
  if (Aufgabe) {
    console.log(`Gefunden: ${task.title}`);
  } else {
    console.log('Aufgabe nicht gefunden'); }
  }
});
// Ausgabe: gefunden: Aufgabe B

// Unerledigte Aufgaben finden
findFirstIncompleteTask(tasks$).subscribe(task => {
  if (Aufgabe) {
    console.log(`Nächste Aufgabe: ${task.title} (Priorität: ${task.priority})`);
  }
});
// Ausgabe: nächste Aufgabe: Aufgabe B (Priorität: mittel)

```

## 🔄 find und findIndex Der Unterschied zwischen

RxJSin den `findIndex` Operatoren sind ebenfalls verfügbar.

```

ts
import { from } from 'rxjs';
importiere { find, findIndex } von 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// find: Rückgabe eines Wertes
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);.
// Ausgabe: 30

// findIndex: Rückgabe index
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);.
// Ausgabe: 2 (Index von 30)

```

| Operator | Rückgabewert | wenn der Wert nicht gefunden wird |
|---|---|---|
| `find(predicate)` | Wert selbst | `undefined` |
| `findIndex(predicate)` | Index (numerischer Wert) | `-1` |

## ⚠️ Häufige Fehler

> [!NOTE]
> `find` wenn der Wert nicht gefunden wird. `undefined` ausgegeben wird. Dies führt nicht zu einem Fehler. Wenn ein Fehler erforderlich ist, muss `first` verwendet werden.

### Fehler.: Erwartete Fehlerbehandlung, wenn der Wert nicht gefunden wird.

```

ts.
import { from } from 'rxjs';
importiere { find } von 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ❌ Schlechtes Beispiel: Fehlerbehandlung erwartet, aber nicht aufgerufen
numbers$.pipe(
  find(n => n > 10)
).subscribe({
  next: console.log,.
  error: err => console.log('Error:', err) // nicht aufgerufen
});
// Ausgabe: undefiniert

```

### Positiv: undefined Prüfen Sie auf oder first Verwendung des

```

ts.
import { from } from 'rxjs';
importiere { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ✅ Gutes Beispiel 1: Prüfung auf undefiniert
numbers$.pipe(
  find(n => n > 10)
).subscribe(result => {
  if (result ! == undefiniert) {
    console.log('Gefunden:', Ergebnis);
  } else {
    console.log('Nicht gefunden:'); }
  }
});
// Ausgabe: nicht gefunden

// ✅ Gutes Beispiel 2: Verwenden Sie das erste, wenn Sie einen Fehler benötigen
numbers$.pipe(
  first(n => n > 10, 0) // Standardwert angeben
).subscribe({
  next: console.log,.
  error: err => console.log('Fehler:', err.message)
});
// Ausgabe: 0
```

## 🎓 Zusammenfassung

### Wann Sie find verwenden sollten.
- ✅ Wenn Sie den ersten Wert finden wollen, der eine Bedingung erfüllt
- ✅ Wenn Sie das Vorhandensein eines Wertes prüfen wollen
- ✅ Wenn ein Wert als "undefiniert" behandelt werden soll, wenn er nicht gefunden wird.
- ✅ Wenn Sie ein bestimmtes Element in einem Array oder einer Liste suchen wollen

### Wenn Sie first verwenden sollten
- ✅ Wenn Sie den ersten Wert erhalten wollen
- ✅ Wenn Sie einen Fehler ausgeben wollen, wenn der Wert nicht gefunden wird

### Wann sollte filter verwendet werden?
- ✅ Wenn Sie alle Werte benötigen, die eine Bedingung erfüllen
- ✅ Wenn Sie die Daten filtern wollen

### Anmerkungen.
- ⚠️ `find` gibt `undefined` aus, wenn nicht gefunden (kein Fehler)
- ⚠️ Wird sofort mit dem ersten Wert abgeschlossen, der die Bedingung erfüllt
- ⚠️ TypeScript liefert einen Rückgabewert vom Typ `T | undefined`.

## 🚀 Nächster Schritt.

- **[first](. /first)** - lernen Sie, wie man den ersten Wert erhält.
- **[filter](. /filter)** - lernen Sie, wie man auf der Grundlage von Bedingungen filtert.
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - lernen Sie, wie man den Index des ersten Wertes, der eine Bedingung erfüllt, ermittelt (offizielle Dokumentation)
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - lernen Sie echte Anwendungsfälle kennen
