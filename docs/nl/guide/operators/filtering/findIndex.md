---
description: "De operator findIndex is een filteroperator van RxJS die de index van de eerste waarde die aan de voorwaarde voldoet retourneert. Als deze niet wordt gevonden, wordt -1 geretourneerd."
---

# findIndex - de index krijgen die overeenkomt met de voorwaarde

De operator ` findIndex` geeft **de index van de eerste waarde die overeenkomt met de voorwaarde** en voltooit de stream onmiddellijk. Geeft `-1` als er geen waarde wordt gevonden.

## 🔰 Basissyntaxis en gebruik

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Uitgang.: 4(eerste even8index van de eerste even)
```

**Bewerkingsstroom**:.
1. 1 (index 0) → oneven, overslaan
2. 3 (index 1) → oneven, overslaan
3. 5 (index 2) → oneven, overslaan
4. 7 (index 3) → oneven, overslaan
5. 8 (index 4) → even getal, uitvoer index 4 en voltooien

[🌐 Officiële RxJS documentatie - findIndex](https://rxjs.dev/api/operators/findIndex)

## 💡 Typisch gebruikspatroon.

- **Positioneren in een array**: het verkrijgen van de positie van een element dat voldoet aan een specifieke voorwaarde.
- **De volgorde controleren**: hoe vaak verschijnt een element dat aan een bepaalde voorwaarde voldoet?
- Gegevens ordenen**: verwerken met behulp van indexinformatie.
- **Existentiecontrole**: controleert of een element bestaat door te controleren of het -1 is of niet.

## Praktisch codevoorbeeld 1: Een takenlijst doorzoeken

Dit is een voorbeeld van het vinden van de locatie van een taak met specifieke voorwaarden uit een takenlijst.

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
  { id: 1, title: 'Antwoord per e-mail', priority: 'low', completed: true },
  { id: 2, title: 'Document voorbereiding', priority: 'medium', completed: true },
  { id: 3, title: 'Vergadering voorbereiden', priority: 'high', completed: false },
  { id: 4, title: 'Code nakijken', priority: 'high', completed: false },
  { id: 5, title: 'Document bijwerken', priority: 'low', completed: false }
];

// UIaanmaken
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Taken zoeken';
container.appendChild(title);

// Takenlijst weergeven
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

// Zoekknop
const buttonContainer = document.createElement('div');
buttonContainer.style.marginTop = '10px';
container.appendChild(buttonContainer);

const button1 = document.createElement('button');
button1.textContent = 'De eerste onafgeronde taak zoeken';
button1.style.marginRight = '10px';
buttonContainer.appendChild(button1);

const button2 = document.createElement('button');
button2.textContent = 'De eerste taak met hoge prioriteit zoeken';
buttonContainer.appendChild(button2);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// De eerste onafgeronde taak zoeken
// NB.: Oorspronkelijk was het aanbevolen patroon om af te vlakken met switchMap Het aanbevolen patroon is af te vlakken met
// Hier wordt leesbaarheid prioriteit gegeven aan subscribe genest (in productiecode switchMap aanbevolen).
fromEvent(button1, 'click').subscribe(() => {
  // Nesting subscribe: Oorspronkelijk was het aanbevolen patroon om af te vlakken met switchMap Afvlakken met wordt aanbevolen
  from(tasks).pipe(
    findIndex(task => !task.completed)
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Gevonden in</strong><br>
        Positie: Index ${index}<br>
        Taak: ${task.title}<br>
        Prioriteit: ${task.priority}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Onvoltooide taak niet gevonden';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});

// De eerste taak met hoge prioriteit zoeken
// NB.: Oorspronkelijk was het aanbevolen patroon om af te vlakken met switchMap Het aanbevolen patroon (in productiecode) is om af te vlakken met switchMap aanbevolen).
fromEvent(button2, 'click').subscribe(() => {
  // Nesting subscribe: Oorspronkelijk was het aanbevolen patroon om af te vlakken met switchMap Afvlakken met wordt aanbevolen
  from(tasks).pipe(
    findIndex(task => task.priority === 'high')
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Gevonden in</strong><br>
        Positie: Index ${index}<br>
        Taak: ${task.title}<br>
        Voltooiingsstatus: ${task.completed ? 'Voltooid' : 'Onvoltooid'}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Geen taken met hoge prioriteit gevonden';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});
```

- Vindt de positie van de eerste taak in de takenlijst die aan de voorwaarde voldoet.
- Als deze niet wordt gevonden, wordt `-1` geretourneerd.

## Praktisch codevoorbeeld 2: Locatiedetectie van gegevens in realtime

Dit voorbeeld detecteert de positie van de eerste waarde uit de stroom die aan de voorwaarde voldoet.

```ts
import { interval } from 'rxjs';
import { findIndex, map, take } from 'rxjs';

// UIaanmaken
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Real-time gegevens zoeken';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = '50Zoeken naar posities met een waarde groter dan of gelijk aan...';
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

// Willekeurige waarden genereren (0~100)
const data$ = interval(500).pipe(
  take(20),
  map(i => ({ index: i, value: Math.floor(Math.random() * 100) }))
);

// Gegevensweergave
data$.subscribe(data => {
  const div = document.createElement('div');
  const highlight = data.value >= 50 ? 'background-color: #fff9c4;' : '';
  div.style.cssText = `padding: 5px; ${highlight}`;
  div.textContent = `[${data.index}] Waarde: ${data.value}`;
  dataDisplay.appendChild(div);
  dataDisplay.scrollTop = dataDisplay.scrollHeight;
});

// 50Zoek in de index naar de eerste waarde van meer dan
data$.pipe(
  findIndex(data => data.value >= 50)
).subscribe(index => {
  status.textContent = '';
  if (index !== -1) {
    result.innerHTML = `
      ✅ 50Waarde groter dan of gelijk aan gevonden<br>
      Positie: Index ${index}
    `;
    result.style.color = 'green';
  } else {
    result.textContent = '❌ 50Er zijn geen waarden groter dan of gelijk aan gevonden';
    result.style.color = 'orange';
  }
});
```

- Detecteert de positie van de eerste waarde boven 50 uit willekeurige waarden die elke 0,5 seconden worden gegenereerd.
- Markering wordt gebruikt voor visuele duidelijkheid.

## Vergelijking met vergelijkbare operatoren

### FindIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Geeft als resultaat de index van de eerste waarde die voldoet aan de voorwaarde
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Uitgang.: 2Geeft als resultaat de index van de eerste waarde die voldoet aan de voorwaarde30index van de eerste even)

// find: Geeft als resultaat de eerste waarde die aan de voorwaarde voldoet
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Uitgang.: 30

// elementAt: Geeft als resultaat de waarde op de opgegeven index
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Uitgang.: 30
```

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Uitgang.: 4(eerste even8index van de eerste even)
```

## Vergelijking met Array.findIndex() in JavaScript

RxJS `findIndex` gedraagt zich vergelijkbaar met de array-methode `Array.prototype.findIndex()` van JavaScript.


```ts
// JavaScript Matrix van
const numbers = [10, 20, 30, 40, 50];
const index1 = numbers.findIndex(n => n > 25);
console.log(index1); // 2

// RxJS (geeft de eerste waarde op de opgegeven index terug die aan de voorwaarde voldoet) Observable
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log); // 2
```

**Belangrijkste verschillen**.
- **Array**: geeft het resultaat synchroon en onmiddellijk terug.
- **Observable**: asynchroon, wacht tot er waarden uit de stream stromen.

## ⚠️ Opmerkingen.

### 1. retourneert -1 indien niet gevonden

Als geen waarde aan de voorwaarde voldoet, retourneert `-1` in plaats van een foutmelding.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('Er is geen waarde gevonden die aan de voorwaarde voldoet');
  } else {
    console.log(`Index: ${index}`);
  }
});
// Uitgang.: Er is geen waarde gevonden die aan de voorwaarde voldoet
```

### 2. voltooien wanneer voor het eerst gevonden.

De stroom wordt voltooid zodra de eerste waarde die aan de voorwaarde voldoet is gevonden.

```ts
import { interval } from 'rxjs';
import { findIndex, tap } from 'rxjs';

interval(1000).pipe(
  tap(val => console.log(`Waarde: ${val}`)),
  findIndex(n => n >= 3)
).subscribe(index => {
  console.log(`Index: ${index}`);
});
// Uitgang.:
// Waarde: 0
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Index: 3
```

### 3. Typeveiligheid in TypeScript

`findIndex` geeft altijd het type `getal`.

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
  // index is een matrix van number type
  if (index !== -1) {
    console.log(`De eerste inactieve gebruiker is index ${index} is.`);
  }
});
// Uitgang.: De eerste inactieve gebruiker is index 1 is.
```

### 4. index begint bij 0

Net als bij arrays beginnen indexen bij 0.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// Uitgang.: 0(eerste element)
```

## 📚 Verwante operatoren.

- **[find](. /find)** - Verkrijg de eerste waarde die aan de voorwaarde voldoet.
- **[elementAt](. /elementAt)** - Verkrijg de waarde op de opgegeven index.
- **[first](. /first)** - Verkrijg de eerste waarde.
- Filter](. /filter)** - verkrijg alle waarden die voldoen aan de voorwaarde

## Samenvatting.

De ` findIndex` operator geeft de index van de eerste waarde die aan de voorwaarde voldoet.

- Vergelijkbaar met `Array.findIndex()` in JavaScript.
- Ideaal wanneer indexinformatie nodig is
- Geeft `-1` als het niet gevonden wordt (geen fout)
- ✅ Wordt onmiddellijk voltooid als het wordt gevonden
- ⚠️ De teruggegeven waarde is altijd van het type `getal` (-1 of een geheel getal groter dan of gelijk aan 0)
- ⚠️ Gebruik `find` als de waarde zelf nodig is
