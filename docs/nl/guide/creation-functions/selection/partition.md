---
description: partition is een RxJS Creation Function die één Observable splitst in twee Observables op basis van voorwaarden. Het is ideaal voor splitsingsverwerking zoals succes/falen, geldig/ongeldig, enz.
---

# partition - splits in twee streams op basis van voorwaarde

`partition` is een Creation Function die een Observable **verdeelt** in twee Observables op basis van een voorwaarde.
Je kunt de voorwaarde specificeren met een predicate-functie (predicate) en de waarden die aan de voorwaarde voldoen en de waarden die niet aan de voorwaarde voldoen als afzonderlijke streams krijgen.

## Basissyntaxis en gebruik

```ts
import { partition, of } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Splits in even en oneven getallen
const [evens$, odds$] = partition(source$, (value) => value % 2 === 0);

evens$.subscribe((value) => console.log('Even:', value));
// Output: Even: 2, Even: 4, Even: 6

odds$.subscribe((value) => console.log('Odd:', value));
// Output: Odd: 1, Odd: 3, Odd: 5
```

- `partition` retourneert een **array met twee Observables**.
- `[0]`: een stream van waarden die aan de voorwaarde voldoen.
- `[1]`: een stream van waarden die niet aan de voorwaarde voldoen.

[🌐 RxJS Officiële Documentatie - `partition`](https://rxjs.dev/api/index/function/partition)

## Typische gebruikspatronen

- **Gesplitste verwerking van succes/falen** (sorteren op HTTP-statuscode)
- **Gebeurtenisclassificatie** (linksklik/rechtsklik)
- **Dataclassificatie** (geldig/ongeldig, volwassene/kind, enz.)
- **Stream-splitsing op basis van voorwaarden**.

## Praktisch codevoorbeeld (met UI)

Wanneer op een knop wordt geklikt, wordt het proces vertakt afhankelijk van of de klikcoördinaten voor de linker- of rechterhelft van het scherm zijn.

```ts
import { partition, fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Maak output gebied
const leftArea = document.createElement('div');
leftArea.innerHTML = '<h3>Linksklik</h3><ul id="left-list"></ul>';
leftArea.style.float = 'left';
leftArea.style.width = '45%';
leftArea.style.padding = '10px';
leftArea.style.background = '#e3f2fd';
document.body.appendChild(leftArea);

const rightArea = document.createElement('div');
rightArea.innerHTML = '<h3>Rechtsklik</h3><ul id="right-list"></ul>';
rightArea.style.float = 'right';
rightArea.style.width = '45%';
rightArea.style.padding = '10px';
rightArea.style.background = '#fce4ec';
document.body.appendChild(rightArea);

// Klik gebeurtenissen
const clicks$ = fromEvent<MouseEvent>(document, 'click');

// Midden X-coördinaat van scherm
const centerX = window.innerWidth / 2;

// Splits in linker- en rechterhelft
const [leftClicks$, rightClicks$] = partition(
  clicks$,
  (event) => event.clientX < centerX
);

// Verwerk linkse klikken
leftClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const leftList = document.getElementById('left-list')!;
  const li = document.createElement('li');
  li.textContent = `Positie: (${pos.x}, ${pos.y})`;
  leftList.appendChild(li);
});

// Verwerk rechtse klikken
rightClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const rightList = document.getElementById('right-list')!;
  const li = document.createElement('li');
  li.textContent = `Positie: (${pos.x}, ${pos.y})`;
  rightList.appendChild(li);
});
```

- Klikken op het scherm worden geregistreerd in de linker- en rechterlijst volgens de klikpositie.
- Twee onafhankelijke streams kunnen worden gemaakt van één enkele bron.

## Praktisch voorbeeld: Vertakkingsverwerking van API-responses

Voorbeeld van het splitsen van succes en falen op HTTP-statuscode

```ts
import { partition, from, of } from 'rxjs';
import { mergeMap, map, catchError, share } from 'rxjs';

interface ApiResponse {
  status: number;
  data?: any;
  error?: string;
}

// Dummy API-aanroepen
const apiCalls$ = from([
  fetch('/api/users/1'),
  fetch('/api/users/999'), // Niet-bestaande gebruiker
  fetch('/api/users/2'),
]);

// Verwerk Response en converteer naar ApiResponse
const responses$ = apiCalls$.pipe(
  mergeMap(fetchPromise => from(fetchPromise)),
  mergeMap(response =>
    from(response.json()).pipe(
      map(data => ({
        status: response.status,
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : (data.message || 'Error')
      } as ApiResponse)),
      catchError(err => of({
        status: response.status,
        data: undefined,
        error: err.message || 'Failed to parse response'
      } as ApiResponse))
    )
  ),
  share() // Behandel 2 subscriptions van partition
);

// Splits in succes (200s) en falen (anderen)
const [success$, failure$] = partition(
  responses$,
  (response: ApiResponse) => response.status >= 200 && response.status < 300
);

// Behandel succesvolle responses
success$.subscribe((response) => {
  console.log('✅ Succes:', response.data);
  // Toon succesdata in UI
});

// Behandel gefaalde responses
failure$.subscribe((response) => {
  console.error('❌ Falen:', response.error);
  // Toon foutmelding
});
```

## Vergelijking met filter

### Basisverschillen

| Methode | Beschrijving | Output | Use Case |
|--------|-------------|--------|----------|
| `partition` | Splits één bron in twee streams | 2 Observables | Wanneer je beide streams **tegelijkertijd** wilt gebruiken |
| `filter` | Laat alleen waarden door die aan de voorwaarde voldoen | 1 Observable | Wanneer slechts één stream nodig is |

### Voorbeelden van gebruik

**Gebruik partition om beide streams tegelijkertijd te verwerken**

```ts
import { partition, interval } from 'rxjs';
import { map, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Succes</h4><ul id="success-list"></ul>';
successArea.style.float = 'left';
successArea.style.width = '45%';
output.appendChild(successArea);

const failureArea = document.createElement('div');
failureArea.innerHTML = '<h4 style="color: red;">❌ Falen</h4><ul id="failure-list"></ul>';
failureArea.style.float = 'right';
failureArea.style.width = '45%';
output.appendChild(failureArea);

// Willekeurige succes/falen stream
const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Taak ${i + 1}`
  }))
);

// ✅ partition - behandel succes en falen tegelijkertijd
const [success$, failure$] = partition(tasks$, task => task.success);

success$.subscribe(task => {
  const successList = document.getElementById('success-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  successList.appendChild(li);
});

failure$.subscribe(task => {
  const failureList = document.getElementById('failure-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  failureList.appendChild(li);
});
```

**Gebruik filter als slechts één stream nodig is**

```ts
import { interval } from 'rxjs';
import { map, take, filter } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Toon alleen succes</h4><ul id="success-only"></ul>';
output.appendChild(successArea);

const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Taak ${i + 1}`
  }))
);

// ✅ filter - verwerk alleen succes (negeer fouten)
tasks$
  .pipe(filter(task => task.success))
  .subscribe(task => {
    const successList = document.getElementById('success-only')!;
    const li = document.createElement('li');
    li.textContent = task.message;
    successList.appendChild(li);
  });
```

**Gebruik filter twee keer vs. partition**

```ts
import { of } from 'rxjs';
import { filter } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// ❌ Gebruik filter twee keer - bron kan twee keer worden uitgevoerd
const evens1$ = numbers$.pipe(filter(n => n % 2 === 0));
const odds1$ = numbers$.pipe(filter(n => n % 2 !== 0));

evens1$.subscribe(n => console.log('Even:', n));
odds1$.subscribe(n => console.log('Oneven:', n));
// Probleem: als numbers$ een cold observable is, wordt het twee keer uitgevoerd

// ✅ Gebruik partition - maak beide streams in één uitvoering
const [evens2$, odds2$] = partition(numbers$, n => n % 2 === 0);

evens2$.subscribe(n => console.log('Even:', n));
odds2$.subscribe(n => console.log('Oneven:', n));
// Voordeel: efficiënt twee streams maken van één bron
```

**Gebruik filter als je in de pipeline wilt vertakken**

```ts
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

interface User {
  id: number;
  name: string;
  age: number;
  isActive: boolean;
}

const users$ = from([
  { id: 1, name: 'Alice', age: 25, isActive: true },
  { id: 2, name: 'Bob', age: 30, isActive: false },
  { id: 3, name: 'Carol', age: 35, isActive: true }
]);

// ❌ partition is een Creation Function, dus het kan niet in een pipeline worden gebruikt
// users$.pipe(
//   map(user => user.name),
//   partition(name => name.startsWith('A')) // Fout
// );

// ✅ Gebruik filter - beschikbaar in pipeline
users$
  .pipe(
    filter(user => user.isActive),  // Alleen actieve gebruikers
    map(user => user.name)           // Extraheer naam
  )
  .subscribe(console.log);
// Output: Alice, Carol
```

### Samenvatting

| Situatie | Aanbevolen Methode | Reden |
|-----------|-------------------|--------|
| Wil **beide** succes en falen verwerken | `partition` | Kan twee streams maken in één bronuitvoering |
| Wil **alleen** succes verwerken | `filter` | Eenvoudig en duidelijk |
| Wil voorwaarden vertakken in pipeline | `filter` | `partition` kan niet worden gebruikt omdat het een Creation Function is |
| Wil vertakken in 3 of meer met complexe voorwaarden | `groupBy` | Kan splitsen in meerdere groepen |

## Opmerkingen

### 1. Subscribe op beide streams

De twee Observables die in een `partition` worden gemaakt **delen** de oorspronkelijke bron.
Als je niet op beide subscribet, wordt de oorspronkelijke stream mogelijk niet volledig verwerkt.

```ts
const [success$, failure$] = partition(source$, predicate);

// Subscribe op beide
success$.subscribe(handleSuccess);
failure$.subscribe(handleFailure);
```

### 2. De bron wordt twee keer uitgevoerd

De `partition` subscribet intern twee keer op de oorspronkelijke bron.
Wees je bewust van eventuele neveneffecten.

```ts
let callCount = 0;
const source$ = new Observable(observer => {
  callCount++;
  console.log(`Subscription aantal: ${callCount}`);
  observer.next(1);
  observer.complete();
});

const [a$, b$] = partition(source$, n => n > 0);
a$.subscribe(); // Subscription aantal: 1
b$.subscribe(); // Subscription aantal: 2
```

Om neveneffecten te voorkomen, gebruik `share()`.

```ts
import { share } from 'rxjs';

const shared$ = source$.pipe(share());
const [a$, b$] = partition(shared$, n => n > 0);
```

### 3. Niet aangeboden als Pipeable Operator

Sinds RxJS 7 wordt `partition` alleen als **Creation Function** aangeboden.
Het kan niet binnen een pipeline worden gebruikt.

```ts
// ❌ Niet mogelijk
source$.pipe(
  partition(n => n % 2 === 0) // Fout
);

// ✅ Correct gebruik
const [evens$, odds$] = partition(source$, n => n % 2 === 0);
```

## Alternatieve Patronen

Als je binnen een pipeline wilt vertakken, gebruik `filter`.

```ts
const source$ = of(1, 2, 3, 4, 5, 6);

const evens$ = source$.pipe(filter(n => n % 2 === 0));
const odds$ = source$.pipe(filter(n => n % 2 !== 0));

// Of deel de bron met share
const shared$ = source$.pipe(share());
const evens$ = shared$.pipe(filter(n => n % 2 === 0));
const odds$ = shared$.pipe(filter(n => n % 2 !== 0));
```

## Gerelateerde operators

- [`filter`](/nl/guide/operators/filtering/filter.md) - laat alleen waarden door die aan een voorwaarde voldoen
- [`groupBy`](/nl/guide/operators/transformation/groupBy.md) - Splits in meerdere groepen
- [`share`](/nl/guide/operators/multicasting/share.md) - Deel een bron

## Samenvatting

`partition` is een krachtige tool om één Observable te splitsen in twee op basis van een voorwaarde.

- ✅ Ideaal voor succes/falen splitsingsverwerking
- ✅ Maakt twee onafhankelijke streams
- ⚠️ Bronnen worden twee keer ge-subscribed (let op neveneffecten)
- ⚠️ Niet aangeboden als Pipeable Operator
