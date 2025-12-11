---
description: De distinct operator verwijdert alle dubbele waarden en geeft alleen unieke waarden uit die nog nooit zijn uitgegeven. Let op bij oneindige streams, aangezien het intern Set gebruikt om eerder uitgegeven waarden op te slaan.
---

# distinct - Verwijder alle dubbele waarden

De `distinct` operator monitort alle waarden uitgegeven door Observable en geeft **alleen waarden uit die nog nooit eerder zijn uitgegeven**. Intern gebruikt het Set om eerder uitgegeven waarden te onthouden.


## 🔰 Basissyntax en gebruik

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

const numbers$ = of(1, 2, 1, 3, 2, 4, 1, 5);

numbers$.pipe(
  distinct()
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5
```

- Verwijdert duplicaten door de gehele stream
- Zodra een waarde is uitgegeven, wordt deze genegeerd ongeacht hoe vaak deze daarna verschijnt
- `distinctUntilChanged` verwijdert alleen **opeenvolgende** duplicaten, terwijl `distinct` **alle** duplicaten verwijdert

[🌐 RxJS Officiële Documentatie - `distinct`](https://rxjs.dev/api/operators/distinct)


## 🆚 Verschil met distinctUntilChanged

```ts
import { of } from 'rxjs';
import { distinct, distinctUntilChanged } from 'rxjs';

const values$ = of(1, 2, 1, 2, 3, 1, 2, 3);

// distinctUntilChanged: Verwijder alleen opeenvolgende duplicaten
values$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Output: 1, 2, 1, 2, 3, 1, 2, 3

// distinct: Verwijder alle duplicaten
values$.pipe(
  distinct()
).subscribe(console.log);
// Output: 1, 2, 3
```

| Operator | Verwijderingsdoel | Gebruiksscenario |
|---|---|---|
| `distinctUntilChanged` | Alleen opeenvolgende duplicaten | Invoervelden, sensordata |
| `distinct` | Alle duplicaten | Lijst van unieke waarden, ID-lijst |


## 🎯 Vergelijkingsaanpassing met keySelector

Gebruik de `keySelector` functie om duplicaten te bepalen voor een specifieke eigenschap van een object.

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const users$ = of(
  { id: 1, name: 'Alice' } as User,
  { id: 2, name: 'Bob' } as User,
  { id: 1, name: 'Alice (bijgewerkt)' } as User, // Zelfde ID
  { id: 3, name: 'Charlie' } as User
);

users$.pipe(
  distinct(user => user.id) // Bepaal duplicaten op ID
).subscribe(console.log);
// Output:
// { id: 1, name: 'Alice' }
// { id: 2, name: 'Bob' }
// { id: 3, name: 'Charlie' }
```


## 💡 Typische gebruikspatronen

1. **Haal lijst van unieke ID's op**
   ```ts
   import { from } from 'rxjs';
   import { distinct, map } from 'rxjs';

   interface Order {
     orderId: string;
     userId: number;
     amount: number;
   }

   const orders$ = from([
     { orderId: 'A1', userId: 1, amount: 100 },
     { orderId: 'A2', userId: 2, amount: 200 },
     { orderId: 'A3', userId: 1, amount: 150 },
     { orderId: 'A4', userId: 3, amount: 300 }
   ] as Order[]);

   // Haal alleen unieke gebruikers-ID's op
   orders$.pipe(
     map(order => order.userId),
     distinct()
   ).subscribe(userId => {
     console.log(`Gebruikers-ID: ${userId}`);
   });
   // Output: 1, 2, 3
   ```

2. **Extraheer unieke gebeurtenistypen uit gebeurtenislog**
   ```ts
   import { fromEvent, merge } from 'rxjs';
   import { map, distinct, take } from 'rxjs';

   // Maak UI-elementen dynamisch
   const container = document.createElement('div');
   document.body.appendChild(container);

   const button1 = document.createElement('button');
   button1.textContent = 'Knop 1';
   container.appendChild(button1);

   const button2 = document.createElement('button');
   button2.textContent = 'Knop 2';
   container.appendChild(button2);

   const input = document.createElement('input');
   input.placeholder = 'Voer iets in';
   container.appendChild(input);

   const log = document.createElement('div');
   log.style.marginTop = '10px';
   container.appendChild(log);

   // Voeg meerdere gebeurtenisstreams samen om unieke gebeurtenistypen te extraheren
   const events$ = merge(
     fromEvent(button1, 'click').pipe(map(() => 'knop1-klik')),
     fromEvent(button2, 'click').pipe(map(() => 'knop2-klik')),
     fromEvent(input, 'input').pipe(map(() => 'invoer-wijziging'))
   );

   events$.pipe(
     distinct(),
     take(3) // Voltooi wanneer alle 3 types gebeurtenissen aanwezig zijn
   ).subscribe({
     next: (eventType) => {
       log.textContent += `Unieke gebeurtenis: ${eventType}\n`;
       console.log(`Unieke gebeurtenis: ${eventType}`);
     },
     complete: () => {
       log.textContent += 'Alle gebeurtenistypen gedetecteerd';
     }
   });
   ```


## 🧠 Praktisch codevoorbeeld (Tag-invoer)

Hier is een voorbeeld van een UI die automatisch duplicaten verwijdert uit tags ingevoerd door de gebruiker.

```ts
import { fromEvent, Subject } from 'rxjs';
import { map, distinct, scan } from 'rxjs';

// Maak UI-elementen
const container = document.createElement('div');
document.body.appendChild(container);

const tagInput = document.createElement('input');
tagInput.type = 'text';
tagInput.placeholder = 'Voer tag in en druk Enter';
container.appendChild(tagInput);

const tagList = document.createElement('ul');
tagList.style.marginTop = '10px';
container.appendChild(tagList);

// Tag toevoegingsstream
const tagSubject$ = new Subject<string>();

tagSubject$.pipe(
  map(tag => tag.trim().toLowerCase()),
  distinct() // Verwijder dubbele tags
).subscribe(tag => {
  const li = document.createElement('li');
  li.textContent = tag;
  tagList.appendChild(li);
});

// Voeg een tag toe met de Enter-toets
fromEvent<KeyboardEvent>(tagInput, 'keydown').subscribe(event => {
  if (event.key === 'Enter') {
    const value = tagInput.value.trim();
    if (value) {
      tagSubject$.next(value);
      tagInput.value = '';
    }
  }
});
```

Deze code zorgt ervoor dat dezelfde tag slechts één keer aan de lijst wordt toegevoegd, zelfs als deze meerdere keren wordt ingevoerd.


## ⚠️ Opmerking over geheugengebruik

> [!WARNING]
> De `distinct` operator gebruikt **Set** intern om alle eerder uitgegeven waarden op te slaan. Gebruik met een oneindige stream kan geheugenlekken veroorzaken.

### Probleem: Geheugenlek in oneindige streams

```ts
import { interval } from 'rxjs';
import { distinct, map } from 'rxjs';

// ❌ Slecht voorbeeld: distinct gebruiken met oneindige streams
interval(100).pipe(
  map(n => n % 10), // 0-9 cyclus
  distinct() // Geeft alleen de eerste 10 uit, houdt ze daarna in geheugen
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// Niets wordt daarna uitgegeven, maar de Set blijft opgeslagen
```

### Oplossing: Wis Set met flushes parameter

```ts
import { interval, timer } from 'rxjs';
import { distinct, map } from 'rxjs';

// ✅ Goed voorbeeld: Periodiek Set wissen
interval(100).pipe(
  map(n => n % 5),
  distinct(
    value => value,
    timer(1000) // Wis Set elke 1 seconde
  )
).subscribe(console.log);
// Elke 1 seconde worden 0, 1, 2, 3, 4 opnieuw uitgegeven
```

### Best practices

1. **Gebruik met eindige streams**: HTTP-responses, conversie van arrays, etc.
2. **Gebruik flushes**: Wis periodiek voor oneindige streams
3. **Overweeg distinctUntilChanged**: Gebruik dit om alleen opeenvolgende duplicaten te verwijderen


## 📋 Type-veilig gebruik

Hier is een voorbeeld van een type-veilige implementatie die gebruikmaakt van generics in TypeScript.

```ts
import { Observable } from 'rxjs';
import { distinct, map } from 'rxjs';

interface Product {
  id: number;
  name: string;
  categoryId: number;
}

function getUniqueCategories(
  products$: Observable<Product>
): Observable<number> {
  return products$.pipe(
    distinct(product => product.categoryId)
  ).pipe(
    map(product => product.categoryId)
  );
}

// Voorbeeldgebruik
import { of } from 'rxjs';

const products$ = of(
  { id: 1, name: 'Laptop', categoryId: 10 } as Product,
  { id: 2, name: 'Muis', categoryId: 10 } as Product,
  { id: 3, name: 'Boek', categoryId: 20 } as Product
);

getUniqueCategories(products$).subscribe(categoryId => {
  console.log(`Categorie-ID: ${categoryId}`);
});
// Output: 10, 20
```


## 🎓 Samenvatting

### Wanneer distinct gebruiken
- ✅ Wanneer u een lijst van unieke waarden nodig hebt
- ✅ Wanneer u duplicaten in een eindige stream wilt verwijderen
- ✅ Een lijst van ID's of categorieën maken

### Wanneer distinctUntilChanged gebruiken
- ✅ Wanneer u alleen opeenvolgende duplicaten wilt verwijderen
- ✅ Invoerveld wijzigingsdetectie
- ✅ Wanneer u geheugen wilt besparen met oneindige streams

### Opmerkingen
- ⚠️ Gebruik `flushes` parameter voor oneindige streams om geheugenlekken te voorkomen
- ⚠️ Let op geheugengebruik wanneer grote aantallen unieke waarden worden gestreamd
- ⚠️ Als prestatie kritiek is, monitor de grootte van de Set


## 🚀 Volgende stappen

- **[distinctUntilChanged](/nl/guide/operators/filtering/distinctUntilChanged)** - Leer hoe u alleen opeenvolgende duplicaten verwijdert
- **[distinctUntilKeyChanged](/nl/guide/operators/filtering/distinctUntilKeyChanged)** - Leer hoe u objecten vergelijkt op sleutel
- **[filter](/nl/guide/operators/filtering/filter)** - Leer hoe u filtert op basis van voorwaarden
- **[Filteroperator praktische voorbeelden](/nl/guide/operators/filtering/practical-use-cases)** - Leer echte use cases
