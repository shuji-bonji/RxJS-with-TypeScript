---
description: De groupBy operator is een RxJS operator die streamwaarden groepeert op basis van een gespecificeerde sleutel en een aparte Observable maakt voor elke groep, wat wordt gebruikt voor dataclassificatie en aggregatieprocessen.
---

# groupBy - Groepeer waarden op basis van een sleutel

De `groupBy` operator **groepeert** waarden die van een stream worden uitgegeven op basis van een gespecificeerde sleutel en voert elke groep uit als een aparte Observable.
Dit is nuttig voor het categoriseren van data of het toepassen van verschillende verwerking op elke groep.

## 🔰 Basissyntax en gebruik

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
  groupBy(person => person.age), // Groepeer op leeftijd
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(arr => ({ age: group.key, people: arr }))
    )
  )
).subscribe(result => {
  console.log(`Leeftijd ${result.age}:`, result.people);
});

// Output:
// Leeftijd 25: [{name: 'Taro', age: 25}, {name: 'Jiro', age: 25}]
// Leeftijd 30: [{name: 'Hanako', age: 30}, {name: 'Misaki', age: 30}]
// Leeftijd 35: [{name: 'Kenta', age: 35}]
```

- `groupBy(person => person.age)` groepeert data op leeftijd als sleutel
- Elke groep wordt behandeld als een `GroupedObservable` en de sleutel van de groep is toegankelijk via de `key` eigenschap
- `mergeMap` behandelt elke gegroepeerde Observable

[🌐 RxJS Officiële Documentatie - `groupBy`](https://rxjs.dev/api/operators/groupBy)

## 💡 Typische gebruikspatronen

- Categorisatie van data per categorie
- Aggregatieverwerking per groep
- Verwerking van logs en gebeurtenissen per type
- Datagroepering en transformatie

## 🧠 Praktisch codevoorbeeld (met UI)

Dit voorbeeld toont hoe het aantal stukken gegroepeerd op kleur wordt weergegeven wanneer een knop wordt geklikt.

```ts
import { fromEvent, from } from 'rxjs';
import { groupBy, mergeMap, toArray, switchMap, map } from 'rxjs';

// Maak knoppen
const colors = ['Rood', 'Blauw', 'Groen', 'Geel'];
colors.forEach(color => {
  const button = document.createElement('button');
  button.textContent = color;
  button.style.margin = '5px';
  button.style.padding = '10px';
  button.dataset.color = color;
  document.body.appendChild(button);
});

const calculateButton = document.createElement('button');
calculateButton.textContent = 'Aggregeer';
calculateButton.style.margin = '5px';
calculateButton.style.padding = '10px';
document.body.appendChild(calculateButton);

// Maak uitvoergebied
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Registreer geklikte kleuren
const clicks: string[] = [];

// Klikgebeurtenissen voor kleurknoppen
fromEvent(document, 'click').subscribe((event: Event) => {
  const target = event.target as HTMLElement;
  const color = target.dataset.color;
  if (color) {
    clicks.push(color);
    output.innerHTML = `Geselecteerde kleuren: ${clicks.join(', ')}`;
  }
});

// Groepeer en toon wanneer aggregeerknop wordt geklikt
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
    output.innerHTML = '<p>Nog geen kleuren geselecteerd</p>';
    return;
  }
  const resultText = results
    .map(r => `${r.color}: ${r.count} keer`)
    .join('<br>');
  output.innerHTML = `<h3>Aggregatieresultaten</h3>${resultText}`;
});
```

- Klik op kleurknoppen om kleuren te selecteren
- Klik op de "Aggregeer" knop om per kleur te groeperen en het aantal stukken weer te geven
- Groepeer op kleur met `groupBy` en tel het aantal elementen in elke groep

## 🎯 Voorbeeld van aggregatie per categorie

Hier is een voorbeeld van het groeperen van producten per categorie en het berekenen van het totaalbedrag voor elke categorie.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, reduce, map } from 'rxjs';

interface Product {
  name: string;
  category: string;
  price: number;
}

const products: Product[] = [
  { name: 'Appel', category: 'Fruit', price: 150 },
  { name: 'Sinaasappel', category: 'Fruit', price: 100 },
  { name: 'Wortel', category: 'Groente', price: 80 },
  { name: 'Tomaat', category: 'Groente', price: 120 },
  { name: 'Melk', category: 'Zuivel', price: 200 },
  { name: 'Kaas', category: 'Zuivel', price: 300 },
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
  console.log(`${result.category}: €${result.total}`);
});

// Output:
// Fruit: €250
// Groente: €200
// Zuivel: €500
```

## 🎯 Voorbeeld van element selector gebruik

Bij groepering kunnen waarden ook worden geconverteerd.

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
    student => student.grade,           // Sleutel selector
    student => student.name             // Element selector (bewaar alleen namen)
  ),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(names => ({ grade: group.key, students: names }))
    )
  )
).subscribe(result => {
  console.log(`Klas ${result.grade}:`, result.students.join(', '));
});

// Output:
// Klas 1: Taro, Jiro
// Klas 2: Hanako, Misaki
```

- 1e argument: Sleutel selector (criteria voor groepering)
- 2e argument: Element selector (waarde om in de groep op te slaan)

## 🎯 Type-veilig groupBy gebruiken

Dit is een voorbeeld van het benutten van TypeScript's type-inferentie.

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
  { level: 'info', message: 'App gestart', timestamp: 1000 },
  { level: 'warning', message: 'Waarschuwingsbericht', timestamp: 2000 },
  { level: 'error', message: 'Fout opgetreden', timestamp: 3000 },
  { level: 'info', message: 'Proces voltooid', timestamp: 4000 },
  { level: 'error', message: 'Verbindingsfout', timestamp: 5000 },
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
  console.log(`[${result.level.toUpperCase()}] ${result.count} items`);
  result.messages.forEach(msg => console.log(`  - ${msg}`));
});

// Output:
// [INFO] 2 items
//   - App gestart
//   - Proces voltooid
// [WARNING] 1 items
//   - Waarschuwingsbericht
// [ERROR] 2 items
//   - Fout opgetreden
//   - Verbindingsfout
```

## 🎯 Verschillende verwerking toepassen op elke groep

Hier is een voorbeeld van het toepassen van verschillende verwerking op elke groep.

```ts
import { from, of } from 'rxjs';
import { groupBy, mergeMap, delay, map } from 'rxjs';

interface Task {
  id: number;
  priority: 'high' | 'medium' | 'low';
  name: string;
}

const tasks: Task[] = [
  { id: 1, priority: 'high', name: 'Urgente taak' },
  { id: 2, priority: 'low', name: 'Lage prioriteit taak' },
  { id: 3, priority: 'high', name: 'Belangrijke taak' },
  { id: 4, priority: 'medium', name: 'Normale taak' },
];

from(tasks).pipe(
  groupBy(task => task.priority),
  mergeMap(group => {
    // Stel vertragingstijd in volgens prioriteit
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
  console.log(`[${task.priority}] Verwerken ${task.name}`);
});

// Output (in prioriteitsvolgorde):
// [high] Verwerken Urgente taak
// [high] Verwerken Belangrijke taak
// (Na 1 seconde)
// [medium] Verwerken Normale taak
// (Na nog 1 seconde)
// [low] Verwerken Lage prioriteit taak
```

## ⚠️ Opmerkingen

### Abonnementsbeheer voor Group Observable

`groupBy` maakt een Observable voor elke groep. Deze Observables kunnen geheugenlekken veroorzaken als er niet correct op wordt geabonneerd.

```ts
// ❌ Slecht voorbeeld: Niet abonneren op group Observables
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd')
).subscribe(group => {
  // Niet abonneren op group Observable
  console.log('Groep:', group.key);
});
```

**Tegenmaatregel**: Gebruik altijd `mergeMap`, `concatMap`, `switchMap`, etc. om elke groep te behandelen.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray } from 'rxjs';

// ✅ Goed voorbeeld: Behandel elke groep correct
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd'),
  mergeMap(group =>
    group.pipe(toArray())
  )
).subscribe(console.log);
```

### Dynamisch aanmaken van groepen

Een nieuwe group Observable wordt gemaakt elke keer dat een nieuwe sleutel verschijnt. Wees voorzichtig wanneer er veel sleuteltypes zijn.

```ts
// Voorbeeld waar het aantal sleuteltypes oneindig kan toenemen
fromEvent(document, 'click').pipe(
  groupBy(() => Math.random()) // Verschillende sleutel elke keer
).subscribe(); // Geheugenlek risico
```

## 📚 Gerelateerde operators

- [`partition`](https://rxjs.dev/api/index/function/partition) - Splits in twee Observables op voorwaarde
- [`reduce`](/nl/guide/operators/transformation/reduce) - Krijg het eindaggregaatresultaat
- [`scan`](/nl/guide/operators/transformation/scan) - Cumulatieve aggregatie
- [`toArray`](/nl/guide/operators/utility/toArray) - Combineer alle waarden in een array

## Samenvatting

De `groupBy` operator stelt u in staat om waarden in een stream te groeperen op basis van sleutels en **elke groep als een aparte Observable te behandelen**. Dit is zeer nuttig voor complexe dataverwerking, zoals het classificeren van data, aggregeren per categorie en elke groep anders verwerken. Echter, elke group Observable moet correct worden geabonneerd en wordt meestal gebruikt in combinatie met `mergeMap` of vergelijkbaar.
