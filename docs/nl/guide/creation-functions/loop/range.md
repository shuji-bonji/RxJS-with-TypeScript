---
description: "range() - Creation Function die opeenvolgende gehele getallen declaratief genereert: Geheugenefficiënt alternatief voor for-lussen voor batchverwerking en paginering"
---

# range() - Genereert een reeks getallen

`range()` is een for-achtige Creation Function die een opgegeven aantal opeenvolgende gehele getallen uitgeeft vanaf een opgegeven startwaarde.

## Overzicht

`range()` geeft een reeks opeenvolgende gehele getallen uit als Observable door een startwaarde en het aantal gehele getallen op te geven. Het wordt gebruikt voor het genereren van opeenvolgende nummers en batchverwerking als een declaratieve manier om de traditionele `for`-statement te vervangen.

**Handtekening**:
```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

**Parameters**:
- `start`: De startwaarde (vanaf waar te beginnen met uitgeven)
- `count`: het aantal uit te geven waarden (weggelaten: van 0 tot minder dan `start`)
- `scheduler`: de scheduler om de waarden uit te geven (weggelaten: synchroon uitgeven)

**Officiële Documentatie**: [📘 RxJS Officieel: range()](https://rxjs.dev/api/index/function/range)

## Basisgebruik

### Patroon 1: Startwaarde en aantal specificeren

Dit is het meest voorkomende gebruik.

```typescript
import { range } from 'rxjs';

// Genereer 5 opeenvolgende nummers vanaf 1 (1, 2, 3, 4, 5)
range(1, 5).subscribe({
  next: value => console.log('Waarde:', value),
  complete: () => console.log('Voltooid')
});

// Output:
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Waarde: 4
// Waarde: 5
// Voltooid
```

### Patroon 2: Opeenvolgende nummers vanaf 0

Door de startwaarde op 0 te zetten, kan een reeks nummers zoals een array-index worden gegenereerd.

```typescript
import { range } from 'rxjs';

// 0 tot 10 opeenvolgende nummers (0, 1, 2, ..., 9)
range(0, 10).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

### Patroon 3: Starten met een negatief getal

Negatieve getallen kunnen ook worden gegenereerd.

```typescript
import { range } from 'rxjs';

// 5 opeenvolgende nummers vanaf -3 (-3, -2, -1, 0, 1)
range(-3, 5).subscribe(console.log);
// Output: -3, -2, -1, 0, 1
```

## Belangrijke kenmerken

### 1. Synchrone emissie

Standaard geeft `range()` alle waarden **synchroon** uit bij abonnement.

```typescript
import { range } from 'rxjs';

console.log('Voor abonnement');

range(1, 3).subscribe(value => console.log('Waarde:', value));

console.log('Na abonnement');

// Output:
// Voor abonnement
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Na abonnement
```

### 2. Voltooit onmiddellijk

Meldt `complete` onmiddellijk na het uitgeven van alle waarden.

```typescript
import { range } from 'rxjs';

range(1, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Voltooid!')
});

// Output: 1, 2, 3, Voltooid!
```

### 3. Equivalentie met for-statement

`range(start, count)` is equivalent aan de volgende for-statement.

```typescript
// Imperatieve for-statement
for (let i = start; i < start + count; i++) {
  console.log(i);
}

// Declaratieve range()
range(start, count).subscribe(console.log);
```

## Praktische use cases

### 1. Batchverwerking

Gebruikt om meerdere taken achtereenvolgens uit te voeren.

```typescript
import { range, of, Observable, concatMap, delay, map } from 'rxjs';
// Functie om dataverwerking te simuleren
function processItem(index: number): Observable<string> {
  return of(index).pipe(
    delay(100), // Simuleer 100ms verwerkingstijd
    map(i => `Resultaat van verwerking item ${i}`)
  );
}

// Verwerk 10 items achtereenvolgens (1 seconde vertraging tussen elke verwerking)
range(1, 10).pipe(
  concatMap(index =>
    processItem(index).pipe(delay(1000))
  )
).subscribe({
  next: result => console.log(`Verwerking voltooid: ${result}`),
  complete: () => console.log('Alle verwerking voltooid')
});

// Output:
// Verwerking voltooid: Resultaat van verwerking item 1 (na ongeveer 1,1 seconden)
// Verwerking voltooid: Resultaat van verwerking item 2 (na ongeveer 2,1 seconden)
// ...
// Verwerking voltooid: Resultaat van verwerking item 10 (na ongeveer 10,1 sec.)
// Alle verwerking voltooid
```

### 2. Paginering

Haal meerdere pagina's met gegevens achtereenvolgens op.

```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Functie om het ophalen van paginagegevens te simuleren
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Item${page}-1`, `Item${page}-2`, `Item${page}-3`]
  }).pipe(
    delay(500) // Simuleer API-aanroep
  );
}

function fetchAllPages(totalPages: number) {
  return range(1, totalPages).pipe(
    concatMap(page => fetchPage(page))
  );
}

fetchAllPages(5).subscribe({
  next: (data: PageData) => console.log(`Pagina ${data.page}:`, data.items),
  complete: () => console.log('Alle pagina\'s opgehaald')
});

// Output:
// Pagina 1: ['Item1-1', 'Item1-2', 'Item1-3']
// Pagina 2: ['Item2-1', 'Item2-2', 'Item2-3']
// Pagina 3: ['Item3-1', 'Item3-2', 'Item3-3']
// Pagina 4: ['Item4-1', 'Item4-2', 'Item4-3']
// Pagina 5: ['Item5-1', 'Item5-2', 'Item5-3']
// Alle pagina's opgehaald
```

### 3. Array-indexen verwerken

Gebruik als een indexgebaseerde lus bij het verwerken van elk element van een array.

```typescript
import { range, map } from 'rxjs';
const items = ['Appel', 'Banaan', 'Kers', 'Dadel', 'Vlierbes'];

range(0, items.length).pipe(
  map(index => ({ index, item: items[index] }))
).subscribe(({ index, item }) => {
  console.log(`[${index}] ${item}`);
});

// Output:
// [0] Appel
// [1] Banaan
// [2] Kers
// [3] Dadel
// [4] Vlierbes
```

### 4. Testgegevens genereren

Handig voor het genereren van mock-gegevens voor unit tests.

```typescript
import { range, map, toArray } from 'rxjs';
// Mock-gebruikersgegevens genereren
range(1, 100).pipe(
  map(id => ({
    id,
    name: `Gebruiker${id}`,
    email: `gebruiker${id}@example.com`
  })),
  toArray()
).subscribe(users => {
  console.log(`${users.length} gebruikers gegenereerd`);
  // Gebruik in testen
});
```

### 5. Teller voor retry-verwerking

Controleert het aantal herhalingen bij een fout.

```typescript
import { range, throwError, concat, of, Observable, mergeMap, delay, catchError, map, toArray } from 'rxjs';
// Functie om het ophalen van gegevens te simuleren (faalt willekeurig)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.7; // 30% kans op succes

  return of(shouldFail).pipe(
    delay(300),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Gegevens ophalen mislukt'))
        : of('Gegevens ophalen geslaagd')
    )
  );
}

function fetchWithRetry(maxRetries: number = 3) {
  return concat(
    range(0, maxRetries).pipe(
      map(attempt => {
        console.log(`Poging ${attempt + 1}/${maxRetries}`);
        return fetchData().pipe(
          catchError(error => {
            if (attempt === maxRetries - 1) {
              return throwError(() => new Error('Maximum aantal pogingen bereikt'));
            }
            return throwError(() => error);
          }),
          delay(Math.pow(2, attempt) * 1000) // Exponentiële backoff
        );
      }),
      toArray()
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Resultaat:', result),
  error: err => console.error('Fout:', err.message)
});

// Voorbeeldoutput:
// Poging 1/3
// Poging 2/3
// Resultaat: Gegevens ophalen geslaagd
```

## Asynchronisatie met Scheduler

Bij het verwerken van grote hoeveelheden gegevens is asynchrone uitvoering mogelijk door een scheduler op te geven.

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
console.log('Start');

// Asynchroon 1.000.000 nummers uitgeven
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe({
  next: val => {
    if (val % 100000 === 0) {
      console.log(`Voortgang: ${val}`);
    }
  },
  complete: () => console.log('Voltooid')
});

console.log('Na abonnement (asynchroon, dus wordt onmiddellijk uitgevoerd)');

// Output:
// Start
// Na abonnement (asynchroon, dus wordt onmiddellijk uitgevoerd)
// Voortgang: 100000
// Voortgang: 200000
// ...
// Voltooid
```

> [!TIP]
> **De Scheduler gebruiken**:
> - Blokkeer de UI niet bij het verwerken van grote hoeveelheden gegevens
> - Tijdcontrole in testen (TestScheduler)
> - Event loop controle in Node.js-omgeving

Voor meer informatie, zie [Soorten Schedulers en hoe ze te gebruiken](/nl/guide/schedulers/types).

## Vergelijking met andere Creation Functions

### range() vs of()

```typescript
import { range, of } from 'rxjs';

// range() - opeenvolgende gehele getallen
range(1, 3).subscribe(console.log);
// Output: 1, 2, 3

// of() - willekeurige waarden opsommen
of(1, 2, 3).subscribe(console.log);
// Output: 1, 2, 3

// Verschil: range() accepteert alleen opeenvolgende nummers, of() accepteert willekeurige waarden
of(1, 10, 100).subscribe(console.log);
// Output: 1, 10, 100 (niet mogelijk met range())
```

### range() vs from()

```typescript
import { range, from } from 'rxjs';

// range() - opeenvolgende nummers genereren
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// from() - genereren uit een array (moet array van tevoren maken)
from([1, 2, 3, 4, 5]).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// Voordeel van range(): geen voorafgaande toewijzing van arrays in geheugen
range(1, 1000000); // Geheugenefficiënt
from(Array.from({ length: 1000000 }, (_, i) => i + 1)); // Array gaat in geheugen
```

### range() vs generate()

```typescript
import { range, generate } from 'rxjs';

// range() - eenvoudige opeenvolgende nummering
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// generate() - een complex voorbeeld van hetzelfde
generate(
  1,                    // Initiële waarde
  x => x <= 5,          // Voortzettingsvoorwaarde
  x => x + 1            // Iteratie
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// Voordelen van generate(): complexe voorwaarde en statusbeheer
generate(
  1,
  x => x <= 100,
  x => x * 2  // Verhoogt met factor 2
).subscribe(console.log);
// Output: 1, 2, 4, 8, 16, 32, 64
// (niet mogelijk met range())
```

> [!TIP]
> **Selectiecriteria**:
> - **Opeenvolgende nummers nodig** → `range()`
> - **Willekeurige waarden opsommen** → `of()`
> - **Bestaande array/Promise** → `from()`
> - **Complexe voorwaarde/stap** → `generate()`

## Prestatieoverwegingen

Omdat `range()` waarden synchroon uitgeeft, moet rekening worden gehouden met prestaties bij het genereren van grote aantallen waarden.

> [!WARNING]
> **Verwerken van grote hoeveelheden gegevens**:
> ```typescript
> // ❌ Slecht voorbeeld: 1 miljoen waarden synchroon uitgeven (UI blokkeert)
> range(1, 1000000).subscribe(console.log);
>
> // ✅ Goed voorbeeld 1: asynchroon met scheduler
> range(1, 1000000).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Goed voorbeeld 2: Splitsen door buffering
> range(1, 1000000).pipe(
>   bufferCount(1000)
> ).subscribe(batch => console.log(`${batch.length} items verwerkt`));
> ```

## Kiezen tussen from() Array

```typescript
import { range, from } from 'rxjs';

// Als u opeenvolgende nummers nodig heeft → range() is beknopter
range(0, 10).subscribe(console.log);

// Geen noodzaak om een array te maken en dan te converteren (inefficiënt)
from(Array.from({ length: 10 }, (_, i) => i)).subscribe(console.log);

// Als er een bestaande array is → gebruik from()
const existingArray = [5, 10, 15, 20];
from(existingArray).subscribe(console.log);
```

## Foutafhandeling

Hoewel `range()` zelf geen fouten uitgeeft, kunnen fouten optreden in de pipeline.

```typescript
import { range, of, map, catchError } from 'rxjs';
range(1, 10).pipe(
  map(n => {
    if (n === 5) {
      throw new Error('Fout bij 5');
    }
    return n * 2;
  }),
  catchError(error => {
    console.error('Fout opgetreden:', error.message);
    return of(-1); // Standaardwaarde retourneren
  })
).subscribe(console.log);

// Output: 2, 4, 6, 8, -1
```

## Samenvatting

`range()` is een eenvoudige maar krachtige Creation Function die een reeks opeenvolgende gehele getallen produceert.

> [!IMPORTANT]
> **range() kenmerken**:
> - ✅ Ideaal voor het genereren van opeenvolgende nummers (alternatief voor for-statement)
> - ✅ Handig voor batchverwerking, paginering, testgegevensgeneratie
> - ✅ Geheugenefficiënt (geen voorafgaande creatie van arrays)
> - ⚠️ Overweeg asynchroon voor grote hoeveelheden gegevens
> - ⚠️ Gebruik `generate()` voor complexe voorwaarden

## Gerelateerde onderwerpen

- [generate()](/nl/guide/creation-functions/loop/generate) - Generieke lusgeneratie
- [of()](/nl/guide/creation-functions/basic/of) - Willekeurige waarden opsommen
- [from()](/nl/guide/creation-functions/basic/from) - Converteren van array of Promise
- [interval()](/nl/guide/creation-functions/basic/interval) - Periodiek waarden uitgeven

## Referenties

- [RxJS Officieel: range()](https://rxjs.dev/api/index/function/range)
- [Learn RxJS: range](https://www.learnrxjs.io/learn-rxjs/operators/creation/range)
