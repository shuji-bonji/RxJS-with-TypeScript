---
description: "find is een RxJS filteroperator die de eerste waarde vindt die aan een voorwaarde voldoet en deze uitvoert, waardoor de stroom onmiddellijk wordt voltooid. Het is ideaal voor situaties waarin je een specifiek element uit een array of lijst wilt vinden, zoals het zoeken naar gebruikers, het controleren van de inventaris of het opsporen van foutlogs. Als er geen waarde wordt gevonden, wordt er een ongedefinieerde uitvoer gegeven en in TypeScript is de retourwaarde van het type T | ongedefinieerd."
---

# find - vind de eerste waarde die aan de voorwaarde voldoet

De `find` operator vindt en voert de **eerste waarde uit die aan de voorwaarde voldoet** en voltooit de stroom onmiddellijk. Als er geen waarde wordt gevonden, wordt `undefined` uitgevoerd.

## 🔰 Basis syntaxis en gebruik

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Uitgang.: 8(eerste even getal)
```

**Bewerkingsstroom**:.
1. controleer 1, 3, 5, 7 → voorwaarde niet vervuld
2. controleer 8 → voorwaarde vervuld → uitvoer 8 en compleet
3. 9, 10 niet geëvalueerd

[🌐 Officiële RxJS documentatie - find](https://rxjs.dev/api/operators/find)

## 🆚 Contrast met first

`find` en `first` lijken op elkaar, maar ze worden anders gebruikt.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: Eerste waarde die aan de voorwaarde voldoet (voorwaarde is optioneel)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Uitgang.: 7

// find: Eerste waarde die aan de voorwaarde voldoet (voorwaarde is verplicht)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Uitgang.: 7
```

TABEL 9

## 💡 Typisch gebruikspatroon

1. **Gebruiker zoeken**.

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

   // ID(voorwaarde is optioneel)2Zoeken naar gebruikers met
   users$.pipe(
     find(user => user.id === 2)
   ).subscribe(user => {
     if (user) {
       console.log(`Gevonden: ${user.name}`);
     } else {
       console.log('Gebruiker niet gevonden');
     }
   });
   // Uitgang.: Gevonden: Bob
   ```

2. **Inventaris controleren**
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
     { id: 'A2', name: 'Muis', stock: 15 },
     { id: 'A3', name: 'Toetsenborden', stock: 8 }
   ] as Product[]);

   // Zoek uit wat niet op voorraad is
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`Niet op voorraad: ${product.name}`);
     } else {
       console.log('Alles op voorraad');
     }
   });
   // Uitgang.: Niet op voorraad: NotebookPC
   ```

3. **Zoeken in foutenlogboek**
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

   // Eerste fout zoeken
   logs$.pipe(
     find(log => log.level === 'error')
   ).subscribe(log => {
     if (log) {
       console.log(`Foutdetectie: ${log.message} (Tijd: ${log.timestamp})`);
     }
   });
   // Uitgang.: Foutdetectie: Connection failed (Tijd: 3)
   ```

## 🧠 Praktijkvoorbeeld code (product zoeken)

Dit is een voorbeeld van het zoeken naar producten uit de voorraad die voldoen aan specifieke criteria.

```

ts.
import { from, fromEvent } from 'rxjs';
import { find } from 'rxjs';

interface Product {
  id: string;
  naam: string;
  prijs: getal;
  categorie: string;
}

const products: Product[] = [
  { id: 'P1', naam: 'Draadloze muis', prijs: 2980, categorie: 'PC-randapparatuur' }
  { id: 'P2', naam: 'Mechanisch toetsenbord', prijs: 8980, categorie: 'PC-randapparatuur' }
  { id: 'P3', naam: 'USB-geheugenstick 64GB', prijs: 1480, categorie: 'Opslag' }
  { id: 'P4', naam: 'Monitor 27-inch', prijs: 29800, categorie: 'Displays' }
  { id: 'P5', naam: 'laptopstandaard', prijs: 3980, categorie: 'PC-randapparatuur' }
];

// UI-elementen maken
const container = document.createElement('div');.
document.body.appendChild(container);

const titel = document.createElement('h3');
title.textContent = "Product zoeken";
container.appendChild(title);

const input = document.createElement('input');
input.type = 'getal';
input.placeholder = "Maximale prijs invoeren";
input.style.marginRight = "10px";
container.appendChild(input);

const zoekknop = document.createElement('knop');
searchButton.textContent = 'search';
container.appendChild(searchButton);

const resultaat = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// Verwerking zoeken
// Opmerking: hoewel het aanbevolen patroon oorspronkelijk is om af te vlakken met een switchMap, maar,
// Opmerking: Hoewel het aanbevolen patroon is om af te vlakken met een switchMap, // nestelen we hier de subscribe voor de leesbaarheid, // omdat het UI-validatie bevat (vroege terugkeer).
// Overweeg een platte implementatie met gebruik van `switchMap` in productiecode.
fromEvent(searchButton, 'click').subscribe() => {
  const maxPrice = parseInt(input.value);.

  Als (isNaN(maxPrice)) {
    result.textContent = 'Voer een prijs in';
    resultaat.style.kleur = 'rood';
    return;
  }

  // Nest subscribe: oorspronkelijk aanbevolen om af te vlakken met switchMap
  from(products).pipe(
    find(product => product.price <= maxPrice)
  ).subscribe(product => {
    if (product) {
      result.innerHTML = `
        <strong>Gevonden! </strong><br>
        Productnaam: ${product.name}<br>
        Prijs: ${product.price.toLocaleString()}<br>
        Categorie: ${product.category}
      `;
      resultaat.style.kleur = 'groen';
    } anders {
      result.textContent = `¥${maxPrice.toLocaleString()} of minder product niet gevonden `;
      resultaat.style.kleur = 'oranje'; }
    }
  });
});

```

Deze code zoekt en toont het eerste product onder de door de gebruiker ingevoerde prijs.

## 🎯 filter Het verschil tussen

`find` en `filter` worden voor verschillende doeleinden gebruikt.

```

ts.
import { from } from 'rxjs';
import { find, filter } from 'rxjs';

const. nummers$ = from([1, 3, 5, 7, 8, 9, 10]);

// filter: uitvoer van alle waarden die aan de voorwaarde voldoen
getallen$.pipe(
  filter(n => n > 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('filter compleet')
});
// Uitvoer: 7, 8, 9, 10, filter voltooid

// find: voer alleen de eerste waarde uit die overeenkomt met de voorwaarde
getallen$.pipe(
  find(n => n > 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('find compleet')
});
// uitvoer: 7, find voltooid

```

| Operator | Aantal uitgangen | Tijdstip van voltooiing | Gebruik |
|---|---|---|---|
| `filter(predicate)` | Alle waarden die aan de voorwaarde voldoen | Bij voltooiing van oorspronkelijke stroom | Verfijning van gegevens |
| `find(predicate)` | Alleen de eerste waarde die aan de criteria voldoet | Onmiddellijk na ontdekking | Zoeken en bestaanscontrole |

## 📋 Type-veilig gebruik

TypeScript Dit is een voorbeeld van een typeveilige implementatie die gebruik maakt van generics in

```

ts.
import { Observable, from } from 'rxjs';
import { find } from 'rxjs';

interface Taak {
  id: getal;
  title: string;
  complete: booleaans;
  priority: 'high' | 'medium' | 'low'; }
}

functie findTaskById(
  taken$: Observable,.
  id: getal
): Observable | undefined> {
  return taken$.pipe(
    find(task => task.id === id)
  );
}

functie findFirstIncompleteTask(
  taken$: Observable
): Observable | undefined> {
  return tasks$.pipe(
    find(task => !.task.complete)
  );
}

// Gebruiksvoorbeeld
const tasks$ = from([.
  { id: 1, titel: 'Taak A', compleet: true, prioriteit: 'hoog' als const }
  { id: 2, titel: 'Taak B', compleet: false, prioriteit: 'medium' als const }
  { id: 3, titel: 'Taak C', compleet: false, prioriteit: 'laag' als const }.
] als Taak[]);.

// Zoeken op ID
findTaskById(tasks$, 2).subscribe(task => {
  if (taak) {
    console.log(`gevonden: ${task.title}`);
  } anders {
    console.log('Taak niet gevonden'); }
  }
});
// Uitvoer: gevonden: taak B

// Onvoltooide taken vinden
findFirstIncompleteTask(tasks$).subscribe(task => {
  if (taak) {
    console.log(`Volgende taak: ${task.title} (prioriteit: ${task.priority})`);
  }
});
// Uitvoer: volgende taak: taak B (prioriteit: gemiddeld)

```

## 🔄 find en findIndex Het verschil tussen

RxJSin de `findIndex` operatoren zijn ook beschikbaar.

```

ts
import { from } from 'rxjs';
import { find, findIndex } from 'rxjs';

const. nummers$ = from([10, 20, 30, 40, 50]);

// find: retourneer een waarde
getallen$.pipe(
  find(n => n > 25)
).subscribe(console.log);.
// uitvoer: 30

// findIndex: index teruggeven
getallen$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);.
// Uitvoer: 2 (index van 30)

```

| Operator | Waarde retourneren | als de waarde niet is gevonden |
|---|---|---|
| `find(predicate)` | Waarde zelf | `undefined` |
| `findIndex(predicate)` | Index (numerieke waarde) | `-1` |

## ⚠️ Een veelgemaakte fout

> [!NOTE]
> `find` als de waarde niet wordt gevonden. `undefined` wordt uitgevoerd. Dit resulteert niet in een fout. Als een fout vereist is, moet `first` worden gebruikt.

### Fout.: Verwachte foutafhandeling als de waarde niet wordt gevonden.

```

ts.
import { from } from 'rxjs';
import { find } from 'rxjs';

const. nummers$ = from([1, 3, 5, 7]);

// ❌ Slecht voorbeeld: foutafhandeling verwacht maar niet aangeroepen
getallen$.pipe(
  find(n => n > 10)
).subscribe({
  next: console.log,.
  error: err => console.log('Error:', err) // niet aangeroepen
});
// uitvoer: ongedefinieerd

```

### Positief: undefined Controleer of first gebruik de

```

ts.
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const. nummers$ = from([1, 3, 5, 7]);

// ✅ Goed voorbeeld 1: controleer op ongedefinieerd
getallen$.pipe(
  find(n => n > 10)
).subscribe(resultaat => {
  als (resultaat ! == undefined) {
    console.log('Gevonden:', resultaat);
  } anders {
    console.log('Niet gevonden:'); }
  }
});
// Uitvoer: niet gevonden

// ✅ Goed voorbeeld 2: gebruik first als je een foutmelding nodig hebt
getallen$.pipe(
  first(n => n > 10, 0) // standaardwaarde opgeven
).subscribe({
  next: console.log,.
  error: err => console.log('Error:', err.message)
});
// Uitvoer: 0
```

## Samenvatting

### Wanneer je find moet gebruiken.
- ✅ Als je de eerste waarde wilt vinden die aan een voorwaarde voldoet
- ✅ Als je het bestaan van een waarde wilt controleren
- ✅ Wanneer je een waarde als `undefined` wilt behandelen als deze niet wordt gevonden.
- ✅ Wanneer je een specifiek element in een array of lijst wilt vinden

### Wanneer je first moet gebruiken
- ✅ Als je de eerste waarde wilt krijgen
- ✅ Als je een foutmelding wilt geven als de waarde niet wordt gevonden

### Wanneer moet filter worden gebruikt?
- ✅ Als u alle waarden nodig hebt die aan een voorwaarde voldoen
- ✅ Als u de gegevens wilt filteren

### Opmerkingen.
- ⚠️ `find` geeft `undefined` als het niet gevonden wordt (geen fout)
- ⚠️ Beëindigt onmiddellijk met de eerste waarde die aan de voorwaarde voldoet
- ⚠️ TypeScript geeft een retourwaarde van het type `T | undefined`.

## Volgende stap.

- **[first](. /first)** - leer hoe je de eerste waarde krijgt.
- **[filter](. /filter)** - leer filteren op basis van voorwaarden.
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - leer hoe je de index krijgt van de eerste waarde die voldoet aan een voorwaarde (officiële documentatie)
- **[filtering-operator-praktische-gebruik-cases](. /practical-use-cases)** - leer echte gebruikssituaties
