---
description: De skip operator slaat het eerste gespecificeerde aantal waarden over van de Observable-stream en geeft alleen volgende waarden uit. Dit is nuttig wanneer u initiële data wilt negeren of een opwarmperiode wilt overslaan.
---

# skip - Sla de eerste N waarden over

De `skip` operator slaat het **eerste gespecificeerde aantal** waarden over van de stream en geeft alleen de volgende waarden uit.


## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, 7, ...
```

- Slaat de eerste 3 waarden over (0, 1, 2)
- De vierde en volgende waarden (3, 4, 5, ...) worden allemaal uitgegeven
- De stream voltooit op de originele voltooiingstijd

[🌐 RxJS Officiële Documentatie - `skip`](https://rxjs.dev/api/operators/skip)


## 🆚 Contrast met take

`skip` en `take` hebben contrasterend gedrag.

```ts
import { range } from 'rxjs';
import { skip, take } from 'rxjs';

const numbers$ = range(0, 10); // 0 tot 9

// take: Haal de eerste N waarden
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Output: 0, 1, 2

// skip: Sla de eerste N waarden over
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, 7, 8, 9

// Combinatie: Sla de eerste 3 over en haal de volgende 3
numbers$.pipe(
  skip(3),
  take(3)
).subscribe(console.log);
// Output: 3, 4, 5
```

| Operator | Gedrag | Voltooiingstiming |
|---|---|---|
| `take(n)` | Haal de eerste n waarden | Voltooit automatisch na n waarden |
| `skip(n)` | Sla de eerste n waarden over | Wanneer de originele stream voltooit |


## 💡 Typische gebruikspatronen

1. **Initiële waarden overslaan**
   ```ts
   import { BehaviorSubject } from 'rxjs';
   import { skip } from 'rxjs';

   const state$ = new BehaviorSubject<number>(0);

   // Sla initiële waarde over en monitor alleen wijzigingen
   state$.pipe(
     skip(1)
   ).subscribe(value => {
     console.log(`Status veranderd: ${value}`);
   });

   state$.next(1); // Output: Status veranderd: 1
   state$.next(2); // Output: Status veranderd: 2
   ```

2. **Opwarmperiode overslaan**
   ```ts
   import { interval } from 'rxjs';
   import { skip, map } from 'rxjs';

   // Simuleer sensordata
   const sensorData$ = interval(100).pipe(
     map(() => Math.random() * 100)
   );

   // Sla de eerste 10 waarden over (1 seconde) als kalibratieperiode
   sensorData$.pipe(
     skip(10)
   ).subscribe(data => {
     console.log(`Sensorwaarde: ${data.toFixed(2)}`);
   });
   ```

3. **Paginering**
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
   const pageNumber = 2; // 0-geïndexeerd

   // Haal de items op pagina 2 (items 5 en 6)
   allItems$.pipe(
     skip(pageNumber * pageSize),
     take(pageSize)
   ).subscribe(item => {
     console.log(item);
   });
   // Output: { id: 5, name: 'Item 5' }, { id: 6, name: 'Item 6' }
   ```


## 🧠 Praktisch codevoorbeeld (Teller)

Dit voorbeeld slaat de eerste 3 klikken over en telt alleen de 4e en volgende klikken.

```ts
import { fromEvent } from 'rxjs';
import { skip, scan } from 'rxjs';

// Maak UI-elementen
const container = document.createElement('div');
document.body.appendChild(container);

const button = document.createElement('button');
button.textContent = 'Klik';
container.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'Teller: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = 'De eerste 3 klikken worden overgeslagen';
container.appendChild(message);

// Klikgebeurtenis
fromEvent(button, 'click').pipe(
  skip(3), // Sla eerste 3 klikken over
  scan((count) => count + 1, 0)
).subscribe(count => {
  counter.textContent = `Teller: ${count}`;
  if (count === 1) {
    message.textContent = 'Tellen begint na 4e klik!';
    message.style.color = 'green';
  }
});
```

Deze code negeert de eerste 3 klikken en begint te tellen als "1" vanaf de 4e klik.


## 🎯 Verschil tussen skip en skipWhile

```ts
import { of } from 'rxjs';
import { skip, skipWhile } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6);

// skip: Specificeer eerste N waarden op aantal
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 4, 5, 6

// skipWhile: Sla over terwijl voorwaarde voldaan is
numbers$.pipe(
  skipWhile(n => n < 4)
).subscribe(console.log);
// Output: 4, 5, 6
```

| Operator | Overslaanvoorwaarde | Gebruiksscenario |
|---|---|---|
| `skip(n)` | Sla de eerste n waarden over op aantal | Vast aantal overslaan |
| `skipWhile(predicate)` | Sla over terwijl voorwaarde voldaan is | Voorwaarde-gebaseerd overslaan |
| `skipUntil(notifier$)` | Sla over tot andere Observable vuurt | Tijdgebaseerd overslaan |


## 📋 Type-veilig gebruik

Hier is een voorbeeld van een type-veilige implementatie die gebruikmaakt van generics in TypeScript.

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

// Voorbeeldgebruik
const users$ = from([
  { id: 1, name: 'Alice', role: 'admin' as const },
  { id: 2, name: 'Bob', role: 'user' as const },
  { id: 3, name: 'Charlie', role: 'user' as const },
  { id: 4, name: 'Dave', role: 'admin' as const },
  { id: 5, name: 'Eve', role: 'user' as const },
] as User[]);

// Haal pagina 1 (tweede pagina, 0-geïndexeerd)
getPaginatedUsers(users$, 1, 2).subscribe(user => {
  console.log(`${user.name} (${user.role})`);
});
// Output: Charlie (user), Dave (admin)
```


## ⚠️ Veelgemaakte fouten

> [!NOTE]
> `skip` slaat alleen de eerste N waarden over en voltooit de stream niet. Combineer bij oneindige streams met `take` om de beëindigingsvoorwaarde in te stellen.

### Fout: Gebruik skip alleen met oneindige streams

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

// ❌ Slecht voorbeeld: Oneindige stream gaat door zoals het is
interval(1000).pipe(
  skip(5)
).subscribe(console.log);
// 5, 6, 7, 8, ... gaat voor altijd door
```

### Correct: Combineer met take om beëindigingsvoorwaarde in te stellen

```ts
import { interval } from 'rxjs';
import { skip, take } from 'rxjs';

// ✅ Goed voorbeeld: Beperk het aantal waarden na skip
interval(1000).pipe(
  skip(5),
  take(3)
).subscribe({
  next: console.log,
  complete: () => console.log('Voltooid')
});
// 5, 6, 7, Voltooid
```


## 🎓 Samenvatting

### Wanneer skip gebruiken
- ✅ Wanneer u de initiële waarde of de eerste N data wilt negeren
- ✅ Wanneer u de initiële waarde van BehaviorSubject wilt overslaan
- ✅ Wanneer u data voor een specifieke pagina wilt ophalen bij paginering
- ✅ Wanneer u de sensorkalibratiperiode wilt overslaan

### Wanneer te combineren met take
- ✅ Wanneer u alleen een specifiek bereik van data wilt ophalen
- ✅ Wanneer u het middelste gedeelte van data uit een oneindige stream wilt ophalen

### Opmerkingen
- ⚠️ Combineer bij oneindige streams met `take` om de beëindigingsvoorwaarde in te stellen
- ⚠️ `skip(0)` werkt hetzelfde als de originele stream (slaat niets over)
- ⚠️ Als het overslaanaantal groter is dan het totale datacount, voltooit het zonder iets uit te geven


## 🚀 Volgende stappen

- **[take](/nl/guide/operators/filtering/take)** - Leer hoe u de eerste N waarden ophaalt
- **[first](/nl/guide/operators/filtering/first)** - Leer hoe u de eerste waarde of de eerste waarde die aan een voorwaarde voldoet ophaalt
- **[last](/nl/guide/operators/filtering/last)** - Leer hoe u de laatste waarde ophaalt
- **[filter](/nl/guide/operators/filtering/filter)** - Leer hoe u filtert op basis van voorwaarden
- **[Filteroperator praktische voorbeelden](/nl/guide/operators/filtering/practical-use-cases)** - Leer echte use cases
