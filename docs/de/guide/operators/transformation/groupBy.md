---
description: "Der groupBy-Operator gruppiert Stream-Werte basierend auf einem angegebenen Schlüssel und erstellt für jede Gruppe ein separates Observable. Typsichere Implementierungen und praktische Anwendungsfälle in TypeScript wie Aggregation nach Kategorie, Verarbeitung nach Benutzer und Datenklassifizierung."
---

# groupBy - Werte basierend auf Schlüssel gruppieren

Der `groupBy`-Operator **gruppiert Werte aus einem Stream basierend auf einem angegebenen Schlüssel** und gibt jede Gruppe als separates Observable aus.
Dies ist nützlich, wenn Sie Daten nach Kategorien klassifizieren oder auf jede Gruppe unterschiedliche Verarbeitung anwenden möchten.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

interface Person {
  name: string;
  age: number;
}

const people: Person[] = [
  { name: 'Taro', age: 25 },
  { name: 'Hanako', age: 30 },
  { name: 'Jiro', age: 25 },
  { name: 'Misaki', age: 30 },
  { name: 'Kenta', age: 35 },
];

from(people).pipe(
  groupBy(person => person.age), // Nach Alter gruppieren
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(arr => ({ age: group.key, people: arr }))
    )
  )
).subscribe(result => {
  console.log(`Alter ${result.age}:`, result.people);
});

// Ausgabe:
// Alter 25: [{name: 'Taro', age: 25}, {name: 'Jiro', age: 25}]
// Alter 30: [{name: 'Hanako', age: 30}, {name: 'Misaki', age: 30}]
// Alter 35: [{name: 'Kenta', age: 35}]
```

- `groupBy(person => person.age)` gruppiert nach Alter als Schlüssel
- Jede Gruppe wird als `GroupedObservable` behandelt, mit Zugriff auf den Gruppenschlüssel über die `key`-Eigenschaft
- `mergeMap` verarbeitet das Observable jeder Gruppe

[🌐 Offizielle RxJS-Dokumentation - `groupBy`](https://rxjs.dev/api/operators/groupBy)

## 💡 Typische Verwendungsmuster

- Datenklassifizierung nach Kategorien
- Aggregationsverarbeitung pro Gruppe
- Verarbeitung von Logs und Events nach Typ
- Datengruppierung und -transformation

## 🧠 Praktisches Codebeispiel (mit UI)

Ein Beispiel, bei dem beim Klicken auf Buttons nach Farbe gruppiert und die Anzahl angezeigt wird.

```ts
import { fromEvent, from } from 'rxjs';
import { groupBy, mergeMap, toArray, switchMap, map } from 'rxjs';

// Buttons erstellen
const colors = ['Rot', 'Blau', 'Grün', 'Gelb'];
colors.forEach(color => {
  const button = document.createElement('button');
  button.textContent = color;
  button.style.margin = '5px';
  button.style.padding = '10px';
  button.dataset.color = color;
  document.body.appendChild(button);
});

const calculateButton = document.createElement('button');
calculateButton.textContent = 'Aggregieren';
calculateButton.style.margin = '5px';
calculateButton.style.padding = '10px';
document.body.appendChild(calculateButton);

// Ausgabebereich erstellen
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Angeklickte Farben aufzeichnen
const clicks: string[] = [];

// Farbbutton-Klickereignisse
fromEvent(document, 'click').subscribe((event: Event) => {
  const target = event.target as HTMLElement;
  const color = target.dataset.color;
  if (color) {
    clicks.push(color);
    output.innerHTML = `Ausgewählte Farben: ${clicks.join(', ')}`;
  }
});

// Bei Klick auf Aggregieren-Button gruppieren und anzeigen
fromEvent(calculateButton, 'click').pipe(
  switchMap(() =>
    from(clicks).pipe(
      groupBy(color => color),
      mergeMap(group =>
        group.pipe(
          toArray(),
          map(items => ({ color: group.key, count: items.length }))
        )
      ),
      toArray()
    )
  )
).subscribe(results => {
  if (results.length === 0) {
    output.innerHTML = '<p>Noch keine Farben ausgewählt</p>';
    return;
  }
  const resultText = results
    .map(r => `${r.color}: ${r.count} mal`)
    .join('<br>');
  output.innerHTML = `<h3>Aggregationsergebnis</h3>${resultText}`;
});
```

- Farbbuttons klicken, um Farben auszuwählen
- "Aggregieren"-Button klicken, um nach Farbe zu gruppieren und Anzahl anzuzeigen
- `groupBy` gruppiert nach Farbe und zählt die Elemente jeder Gruppe

## 🎯 Aggregationsbeispiel nach Kategorie

Ein Beispiel für die Klassifizierung von Produkten nach Kategorie und Berechnung des Gesamtbetrags pro Kategorie.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, reduce, map } from 'rxjs';

interface Product {
  name: string;
  category: string;
  price: number;
}

const products: Product[] = [
  { name: 'Apfel', category: 'Obst', price: 150 },
  { name: 'Orange', category: 'Obst', price: 100 },
  { name: 'Karotte', category: 'Gemüse', price: 80 },
  { name: 'Tomate', category: 'Gemüse', price: 120 },
  { name: 'Milch', category: 'Milchprodukte', price: 200 },
  { name: 'Käse', category: 'Milchprodukte', price: 300 },
];

from(products).pipe(
  groupBy(product => product.category),
  mergeMap(group =>
    group.pipe(
      reduce((total, product) => total + product.price, 0),
      map(total => ({ category: group.key, total }))
    )
  )
).subscribe(result => {
  console.log(`${result.category}: ${result.total}€`);
});

// Ausgabe:
// Obst: 250€
// Gemüse: 200€
// Milchprodukte: 500€
```

## 🎯 Beispiel für Elementselektor

Bei der Gruppierung können Werte auch transformiert werden.

```ts
import { from } from 'rxjs';
import { groupBy, map, mergeMap, toArray } from 'rxjs';

interface Student {
  name: string;
  grade: number;
  score: number;
}

const students: Student[] = [
  { name: 'Taro', grade: 1, score: 85 },
  { name: 'Hanako', grade: 2, score: 92 },
  { name: 'Jiro', grade: 1, score: 78 },
  { name: 'Misaki', grade: 2, score: 88 },
];

from(students).pipe(
  groupBy(
    student => student.grade,           // Schlüsselselektor
    student => student.name             // Elementselektor (nur Namen behalten)
  ),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(names => ({ grade: group.key, students: names }))
    )
  )
).subscribe(result => {
  console.log(`${result.grade}. Klasse:`, result.students.join(', '));
});

// Ausgabe:
// 1. Klasse: Taro, Jiro
// 2. Klasse: Hanako, Misaki
```

- 1. Argument: Schlüsselselektor (Gruppierungskriterium)
- 2. Argument: Elementselektor (in der Gruppe zu speichernder Wert)

## 🎯 Typsichere Verwendung von groupBy

Ein Beispiel mit TypeScript-Typinferenz.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

type LogLevel = 'info' | 'warning' | 'error';

interface LogEntry {
  level: LogLevel;
  message: string;
  timestamp: number;
}

const logs: LogEntry[] = [
  { level: 'info', message: 'App gestartet', timestamp: 1000 },
  { level: 'warning', message: 'Warnmeldung', timestamp: 2000 },
  { level: 'error', message: 'Fehler aufgetreten', timestamp: 3000 },
  { level: 'info', message: 'Verarbeitung abgeschlossen', timestamp: 4000 },
  { level: 'error', message: 'Verbindungsfehler', timestamp: 5000 },
];

from(logs).pipe(
  groupBy(log => log.level),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(entries => ({
        level: group.key,
        count: entries.length,
        messages: entries.map(e => e.message)
      }))
    )
  )
).subscribe(result => {
  console.log(`[${result.level.toUpperCase()}] ${result.count} Einträge`);
  result.messages.forEach(msg => console.log(`  - ${msg}`));
});

// Ausgabe:
// [INFO] 2 Einträge
//   - App gestartet
//   - Verarbeitung abgeschlossen
// [WARNING] 1 Einträge
//   - Warnmeldung
// [ERROR] 2 Einträge
//   - Fehler aufgetreten
//   - Verbindungsfehler
```

## 🎯 Unterschiedliche Verarbeitung pro Gruppe anwenden

Ein Beispiel für die Anwendung unterschiedlicher Verarbeitung auf jede Gruppe.

```ts
import { from, of } from 'rxjs';
import { groupBy, mergeMap, delay, map } from 'rxjs';

interface Task {
  id: number;
  priority: 'high' | 'medium' | 'low';
  name: string;
}

const tasks: Task[] = [
  { id: 1, priority: 'high', name: 'Dringende Aufgabe' },
  { id: 2, priority: 'low', name: 'Aufgeschobene Aufgabe' },
  { id: 3, priority: 'high', name: 'Wichtige Aufgabe' },
  { id: 4, priority: 'medium', name: 'Normale Aufgabe' },
];

from(tasks).pipe(
  groupBy(task => task.priority),
  mergeMap(group => {
    // Verzögerungszeit je nach Priorität setzen
    const delayTime =
      group.key === 'high' ? 0 :
      group.key === 'medium' ? 1000 :
      2000;

    return group.pipe(
      delay(delayTime),
      map(task => ({ ...task, processedAt: Date.now() }))
    );
  })
).subscribe(task => {
  console.log(`[${task.priority}] ${task.name} verarbeitet`);
});

// Ausgabe (nach Priorität):
// [high] Dringende Aufgabe verarbeitet
// [high] Wichtige Aufgabe verarbeitet
// (nach 1 Sekunde)
// [medium] Normale Aufgabe verarbeitet
// (nach weiterer 1 Sekunde)
// [low] Aufgeschobene Aufgabe verarbeitet
```

## ⚠️ Hinweise

### Subscription-Verwaltung für Gruppen-Observable

`groupBy` erstellt ein Observable für jede Gruppe. Diese Observables können Speicherlecks verursachen, wenn sie nicht korrekt abonniert werden.

```ts
// ❌ Schlechtes Beispiel: Gruppen-Observable nicht abonniert
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd')
).subscribe(group => {
  // Gruppen-Observable wird nicht abonniert
  console.log('Gruppe:', group.key);
});
```

**Gegenmaßnahme**: Verarbeiten Sie jede Gruppe immer mit `mergeMap`, `concatMap`, `switchMap` usw.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray } from 'rxjs';

// ✅ Gutes Beispiel: Jede Gruppe korrekt verarbeiten
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd'),
  mergeMap(group =>
    group.pipe(toArray())
  )
).subscribe(console.log);
```

### Dynamische Gruppenerstellung

Jedes Mal, wenn ein neuer Schlüssel erscheint, wird ein neues Gruppen-Observable erstellt. Vorsicht ist geboten, wenn es viele verschiedene Schlüssel gibt.

```ts
// Beispiel, bei dem Schlüsseltypen unendlich wachsen können
fromEvent(document, 'click').pipe(
  groupBy(() => Math.random()) // Jedes Mal anderer Schlüssel
).subscribe(); // Speicherleckgefahr
```

## 📚 Verwandte Operatoren

- [`partition`](https://rxjs.dev/api/index/function/partition) - In zwei Observables nach Bedingung aufteilen
- [`reduce`](./reduce) - Endergebnis der Aggregation abrufen
- [`scan`](./scan) - Kumulative Aggregation
- [`toArray`](../utility/toArray) - Alle Werte in Array zusammenfassen

## Zusammenfassung

Der `groupBy`-Operator ermöglicht es, Stream-Werte basierend auf Schlüsseln zu gruppieren und **jede Gruppe als separates Observable zu behandeln**. Er ist sehr nützlich für komplexe Datenverarbeitung wie Datenklassifizierung, Aggregation nach Kategorie und unterschiedliche Verarbeitung pro Gruppe. Allerdings muss jedes Gruppen-Observable korrekt abonniert werden und wird normalerweise in Kombination mit `mergeMap` usw. verwendet.
