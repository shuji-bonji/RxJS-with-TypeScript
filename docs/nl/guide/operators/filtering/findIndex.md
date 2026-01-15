---
description: De findIndex operator is een RxJS filteroperator die de index retourneert van de eerste waarde die aan de voorwaarde voldoet. Als niet gevonden, retourneert het -1.
titleTemplate: ':title | RxJS'
---

# findIndex - Index van Eerste Match

De `findIndex` operator retourneert **de index van de eerste waarde die aan de voorwaarde voldoet** en voltooit de stream onmiddellijk. Als de waarde niet wordt gevonden, retourneert het `-1`.

## 🔰 Basissyntax en gebruik

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Output: 4 (index van eerste even getal 8)
```

**Werkingsstroom**:
1. 1 (index 0) → Oneven, overslaan
2. 3 (index 1) → Oneven, overslaan
3. 5 (index 2) → Oneven, overslaan
4. 7 (index 3) → Oneven, overslaan
5. 8 (index 4) → Even, geef index 4 uit en voltooi

[🌐 RxJS Officiële Documentatie - `findIndex`](https://rxjs.dev/api/operators/findIndex)

## 💡 Typische gebruikspatronen

- **Positie in array lokaliseren**: Positie ophalen van element dat aan specifieke voorwaarde voldoet
- **Volgorde controleren**: Bepalen op welke positie een element dat aan een voorwaarde voldoet verschijnt
- **Datasortering**: Verwerking met indexinformatie
- **Bestaanscontrole**: Bestaan controleren door of het -1 is of niet

## 🆚 Vergelijking met vergelijkbare operators

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Retourneer index van eerste waarde die aan voorwaarde voldoet
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Output: 2 (index van 30)

// find: Retourneer eerste waarde die aan voorwaarde voldoet
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Output: 30

// elementAt: Retourneer waarde op gespecificeerde index
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Output: 30
```

| Operator | Argument | Retourwaarde | Wanneer niet gevonden |
|:---|:---|:---|:---|
| `findIndex(predicate)` | Voorwaardefunctie | Index (number) | `-1` |
| `find(predicate)` | Voorwaardefunctie | Waarde zelf | `undefined` |
| `elementAt(index)` | Index | Waarde zelf | Fout (geen standaardwaarde) |

## ⚠️ Belangrijke opmerkingen

### 1. Retourneert -1 als niet gevonden

Als geen waarde aan de voorwaarde voldoet, retourneert het `-1` in plaats van een fout.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('Geen waarde gevonden die aan voorwaarde voldoet');
  } else {
    console.log(`Index: ${index}`);
  }
});
// Output: Geen waarde gevonden die aan voorwaarde voldoet
```

### 2. Voltooit wanneer eerste match gevonden

Stream voltooit onmiddellijk bij het vinden van de eerste waarde die aan de voorwaarde voldoet.

```ts
import { interval } from 'rxjs';
import { findIndex, tap } from 'rxjs';

interval(1000).pipe(
  tap(val => console.log(`Waarde: ${val}`)),
  findIndex(n => n >= 3)
).subscribe(index => {
  console.log(`Index: ${index}`);
});
// Output:
// Waarde: 0
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Index: 3
```

### 3. Typeveiligheid in TypeScript

`findIndex` retourneert altijd een `number` type.

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
  // index is number type
  if (index !== -1) {
    console.log(`Eerste inactieve gebruiker is op index ${index}`);
  }
});
// Output: Eerste inactieve gebruiker is op index 1
```

### 4. Indexen beginnen bij 0

Net als arrays beginnen indexen bij 0.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// Output: 0 (eerste element)
```

## 📚 Gerelateerde operators

- **[find](/nl/guide/operators/filtering/find)** - Haal eerste waarde die aan voorwaarde voldoet
- **[elementAt](/nl/guide/operators/filtering/elementAt)** - Haal waarde op gespecificeerde index
- **[first](/nl/guide/operators/filtering/first)** - Haal eerste waarde
- **[filter](/nl/guide/operators/filtering/filter)** - Haal alle waarden die aan voorwaarde voldoen

## Samenvatting

De `findIndex` operator retourneert de index van de eerste waarde die aan de voorwaarde voldoet.

- ✅ Vergelijkbaar gedrag als JavaScript's `Array.findIndex()`
- ✅ Ideaal wanneer indexinformatie nodig is
- ✅ Retourneert `-1` als niet gevonden (geen fout)
- ✅ Voltooit onmiddellijk wanneer gevonden
- ⚠️ Retourwaarde is altijd `number` type (-1 of integer ≥ 0)
- ⚠️ Gebruik `find` als u de waarde zelf nodig hebt
