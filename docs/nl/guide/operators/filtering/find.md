---
description: find is een RxJS filteroperator die de eerste waarde vindt en uitgeeft die aan een voorwaarde voldoet en de stream onmiddellijk voltooit. Het is ideaal voor scenario's waar u een specifiek element wilt zoeken uit een array of lijst, zoals gebruikerszoeken, voorraadcontrole en foutlogdetectie. Als er geen waarde wordt gevonden, geeft het undefined uit.
titleTemplate: ':title | RxJS'
---

# find - Vind de eerste waarde die aan een voorwaarde voldoet

De `find` operator vindt en geeft de **eerste waarde die aan een voorwaarde voldoet** uit en voltooit de stream onmiddellijk. Als er geen waarde wordt gevonden, geeft het `undefined` uit.


## 🔰 Basissyntax en gebruik

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Output: 8 (eerste even getal)
```

**Werkingsstroom**:
1. Controleer 1, 3, 5, 7 → Voldoen niet aan voorwaarde
2. Controleer 8 → Voldoet aan voorwaarde → Geef 8 uit en voltooi
3. 9, 10 worden niet geëvalueerd

[🌐 RxJS Officiële Documentatie - `find`](https://rxjs.dev/api/operators/find)


## 🆚 Contrast met first

`find` en `first` zijn vergelijkbaar maar worden anders gebruikt.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: Eerste waarde die aan voorwaarde voldoet (voorwaarde is optioneel)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Output: 7

// find: Eerste waarde die aan voorwaarde voldoet (voorwaarde is vereist)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Output: 7
```

| Operator | Voorwaarde specificatie | Wanneer waarde niet gevonden | Gebruiksscenario |
|---|---|---|---|
| `first()` | Optioneel | Fout (`EmptyError`) | Eerste waarde ophalen |
| `first(predicate)` | Optioneel | Fout (`EmptyError`) | Voorwaardelijk ophalen |
| `find(predicate)` | Vereist | Geef `undefined` uit | Zoeken/bestaanscontrole |


## 💡 Typische gebruikspatronen

1. **Gebruikerszoeken**
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

   // Zoek gebruiker met ID 2
   users$.pipe(
     find(user => user.id === 2)
   ).subscribe(user => {
     if (user) {
       console.log(`Gevonden: ${user.name}`);
     } else {
       console.log('Gebruiker niet gevonden');
     }
   });
   // Output: Gevonden: Bob
   ```

2. **Voorraadcontrole**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface Product {
     id: string;
     name: string;
     stock: number;
   }

   const products$ = from([
     { id: 'A1', name: 'Laptop', stock: 0 },
     { id: 'A2', name: 'Muis', stock: 15 },
     { id: 'A3', name: 'Toetsenbord', stock: 8 }
   ] as Product[]);

   // Vind product zonder voorraad
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`Niet op voorraad: ${product.name}`);
     } else {
       console.log('Alles op voorraad');
     }
   });
   // Output: Niet op voorraad: Laptop
   ```


## 🎯 Verschil met filter

`find` en `filter` worden voor verschillende doeleinden gebruikt.

```ts
import { from } from 'rxjs';
import { find, filter } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// filter: Geef alle waarden uit die aan voorwaarde voldoen
numbers$.pipe(
  filter(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter voltooid')
});
// Output: 7, 8, 9, 10, filter voltooid

// find: Geef alleen eerste waarde uit die aan voorwaarde voldoet
numbers$.pipe(
  find(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('find voltooid')
});
// Output: 7, find voltooid
```

| Operator | Aantal uitvoer | Voltooiingstiming | Gebruiksscenario |
|---|---|---|---|
| `filter(predicate)` | Alle waarden die aan voorwaarde voldoen | Wanneer originele stream voltooit | Datafiltering |
| `find(predicate)` | Alleen eerste waarde die aan voorwaarde voldoet | Onmiddellijk wanneer gevonden | Zoeken/bestaanscontrole |


## ⚠️ Veelgemaakte fouten

> [!NOTE]
> `find` geeft `undefined` uit wanneer waarde niet wordt gevonden. Het geeft geen fout. Gebruik `first` als u een fout nodig hebt.

### Fout: Foutafhandeling verwachten wanneer waarde niet gevonden

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ❌ Slecht voorbeeld: Foutafhandeling verwachten maar niet aangeroepen
numbers$.pipe(
  find(n => n > 10)
).subscribe({
  next: console.log,
  error: err => console.log('Fout:', err) // Wordt niet aangeroepen
});
// Output: undefined
```

### Correct: Controleer undefined of gebruik first

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ✅ Goed voorbeeld 1: Controleer undefined
numbers$.pipe(
  find(n => n > 10)
).subscribe(result => {
  if (result !== undefined) {
    console.log('Gevonden:', result);
  } else {
    console.log('Niet gevonden');
  }
});
// Output: Niet gevonden

// ✅ Goed voorbeeld 2: Gebruik first als fout nodig is
numbers$.pipe(
  first(n => n > 10, 0) // Specificeer standaardwaarde
).subscribe({
  next: console.log,
  error: err => console.log('Fout:', err.message)
});
// Output: 0
```


## 🎓 Samenvatting

### Wanneer find gebruiken
- ✅ Wanneer u wilt zoeken naar de eerste waarde die aan een voorwaarde voldoet
- ✅ Wanneer u wilt controleren op waarde-bestaan
- ✅ Wanneer u niet-gevonden geval wilt afhandelen met `undefined`
- ✅ Wanneer u wilt zoeken naar een specifiek element uit een array of lijst

### Wanneer first gebruiken
- ✅ Wanneer u de eerste waarde wilt ophalen
- ✅ Wanneer u een fout wilt emitteren als waarde niet wordt gevonden

### Wanneer filter gebruiken
- ✅ Wanneer u alle waarden nodig hebt die aan de voorwaarde voldoen
- ✅ Wanneer het doel datafiltering is

### Opmerkingen
- ⚠️ `find` geeft `undefined` uit wanneer niet gevonden (geen fout)
- ⚠️ Voltooit onmiddellijk met eerste waarde die aan voorwaarde voldoet
- ⚠️ In TypeScript is retourwaarde van type `T | undefined`


## 🚀 Volgende stappen

- **[first](/nl/guide/operators/filtering/first)** - Leer hoe u de eerste waarde ophaalt
- **[filter](/nl/guide/operators/filtering/filter)** - Leer hoe u filtert op basis van voorwaarden
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - Leer hoe u de index ophaalt van de eerste waarde die aan voorwaarde voldoet (officiële documentatie)
- **[Filteroperator praktische voorbeelden](/nl/guide/operators/filtering/practical-use-cases)** - Leer echte use cases
