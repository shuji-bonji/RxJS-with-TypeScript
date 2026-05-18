---
description: "Creation Function die waarden genereren op een lus-achtige manier worden uitgelegd: leer hoe u range en generate kunt gebruiken om iteratieve processen zoals for- en while-statements te implementeren als Observable streams. Van het genereren van opeenvolgende getallen tot complexe toestandsovergangen gebaseerd op aangepaste voorwaarden, u kunt declaratieve lusverwerking realiseren die gebruik maakt van TypeScript type-inferentie."
---

# Loop generatie systeem Creation Function

Creation Function voor het weergeven van lusverwerking zoals for- en while-verklaringen als Observable.

## Lusgeneratiesysteem Wat zijn Creation Function?

Creation Functions voor lusgeneratiesystemen realiseren reactief herhalende processen. Door traditionele imperatieve lussen (`for` en `while` statements) te vervangen door declaratieve Observable streams, maken ze flexibele verwerking mogelijk in combinatie met de RxJS operatorketen.

Bekijk de onderstaande tabel voor de kenmerken en het gebruik van elke Creation Function.

## Systemen voor het genereren van hoofdlussen Creation Function

| Functie | Beschrijving | Gebruikscasus. |
|---|---|---|
| **[range](/nl/guide/creation-functions/loop/range)** | Een bereik van getallen genereren (voor statement-achtige) | Genereren van opeenvolgende getallen, batchverwerking. |
| **[generate](/nl/guide/creation-functions/loop/generate)** | Generieke lussen genereren (while-statement-achtig) | Voorwaardelijke herhaling, complexe toestandsovergangen |

## Gebruikscriteria

De keuze van de lusgenererende Creation Function wordt bepaald door de volgende aspecten.

### 1. generatiepatroon

- **Getallenreeks**: `range()` - eenvoudige opeenvolgende getallengeneratie met begin- en eindwaarden.
- Complexe voorwaarden**: `generate()` - vrije controle over beginwaarden, voorwaarden, iteraties en selectie van resultaten.

### 2. Loop types

- **for-instructie-achtige lus**: `range()` - `for (laat i = start; i <= einde; i++)`
- **while-achtige lussen**: `generate()` - `while (voorwaarde) { ... }`

### 3. flexibiliteit

- **Simpel genoeg**: `range()` - als een reeks getallen nodig is
- **Nodig geavanceerde controle**: `generate()` - aangepast statusbeheer, voorwaardelijke vertakking, stapcontrole

## Praktische gebruikssituaties

### Range() - opeenvolgende getallen genereren

Voor het eenvoudig genereren van opeenvolgende getallen is `range()` de beste keuze.

```typescript
import { range, map } from 'rxjs';
// 1naar5Opeenvolgende nummers genereren uit
range(1, 5).subscribe(console.log);
// Uitvoer: 1, 2, 3, 4, 5

// Gebruiken in batchverwerking
range(0, 10).pipe(
  map(i => `Verwerking${i + 1}`)
).subscribe(console.log);
// Uitvoer: Verwerking1, Verwerking2, ..., Verwerking10
```

### generate() - voorwaardelijke lus

Gebruik `generate()` als complexe voorwaarden of aangepast toestandsbeheer vereist is.

```typescript
import { generate } from 'rxjs';

// Fibonacci-reeks genereren (eerste10term)
generate(
  { current: 0, next: 1, count: 0 },  // Beginvoorwaarde
  state => state.count < 10,           // Voortzetting voorwaarde
  state => ({                          // Toestand bijwerken
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // Resultaat selectie
).subscribe(console.log);
// Uitvoer: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## Vergeleken met imperatieve lus

Dit is een vergelijking tussen conventionele imperatieve lussen en het RxJS lusgeneratiesysteem Creation Function.

### Imperatief voor-statement

```typescript
// ConventioneelforZin
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### Declaratief range()

```typescript
import { range, map, toArray } from 'rxjs';
// RxJSvanrange()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

__OPROEP_11___

> **Voordelen van de declaratieve aanpak**:.
> - Verbeterde leesbaarheid door pijplijnverwerking
> - Uniforme foutafhandeling
> - Gemakkelijk te combineren met asynchrone verwerking
> - Gemakkelijk te annuleren en af te breken (bijv. `takeUntil()`)

## Koude naar warme conversie

Zoals in de bovenstaande tabel te zien is, genereren **alle loop-genererende Creation Function een Cold Observable. Elke inschrijving start een onafhankelijke uitvoering.

Echter, **Koude Observable kunnen worden geconverteerd** naar Hot Observable door gebruik te maken van multicast operators (`share()`, `shareReplay()`, enz.)

### Praktisch voorbeeld: het delen van de resultaten van een berekening.

```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - Onafhankelijke berekening per abonnement
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('Lopende berekening:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('Abonnee1:', val));
cold$.subscribe(val => console.log('Abonnee2:', val));
// → Een berekening wordt2eenmalig uitgevoerd (in2000Berekening eenmalig uitgevoerd)

// 🔥 Hot - Berekeningsresultaten worden gedeeld tussen abonnees
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('Lopende berekening:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('Abonnee1:', val));
hot$.subscribe(val => console.log('Abonnee2:', val));
// → Berekening wordt1slechts eenmaal uitgevoerd (1000Berekening eenmalig uitgevoerd)
```

```typescript
import { range, map } from 'rxjs';
// 1naar5Opeenvolgende nummers genereren uit
range(1, 5).subscribe(console.log);
// Uitvoer: 1, 2, 3, 4, 5

// Gebruiken in batchverwerking
range(0, 10).pipe(
  map(i => `Verwerking${i + 1}`)
).subscribe(console.log);
// Uitvoer: Verwerking1, Verwerking2, ..., Verwerking10
```

> **Gevallen waarin Hotting vereist is**:.
> - Berekeningen met hoge kosten worden op meerdere locaties gebruikt
> - Resultaten van batchverwerking gedeeld door meerdere componenten
> - Resultaten van paginering worden weergegeven in meerdere UI-componenten.
>
> Voor meer informatie, zie [Basissystemen voor creatie - Conversie van koud naar warm](/nl/guide/creation-functions/basic/#cold- to -hot-).

## Gecombineerd met asynchrone verwerking

Het lusgeneratiesysteem Creation Function kan gecombineerd worden met asynchrone verwerking om krachtige functionaliteit te bieden.

### Sequentiële uitvoering van API-oproepen


```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Functie om het verwerven van paginagegevens te simuleren
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Gegevens${page}-1`, `Gegevens${page}-2`, `Gegevens${page}-3`]
  }).pipe(
    delay(300) // APISimuleer een oproep naar
  );
}

// Pagina1naar10Achtereenvolgens opvragen tot (met een1seconden vertraging tussen elke aanvraag)
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe({
  next: data => console.log(`Pagina ${data.page} ophalen:`, data.items),
  error: err => console.error('Fout:', err)
});
```

### Gebruik bij opnieuw proberen

```typescript
import { range, throwError, of, Observable, mergeMap, retry, delay } from 'rxjs';
// Functie om gegevensverwerving te simuleren (mislukt willekeurig)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // 40%Succes met een waarschijnlijkheid van

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Falen van gegevensverwerving'))
        : of('Succesvolle gegevensverwerving')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    // RxJS 7.3+ Aanbeveling: retry({ count, delay }) Formaat
    retry({
      count: 3, // Max.3Herhalingen
      delay: (error, retryCount) => {
        console.log(`Herhalingen ${retryCount}/3`);
        // Exponentiële back-off: 1Seconden,2Seconden,4sec.
        return range(0, 1).pipe(delay(Math.pow(2, retryCount - 1) * 1000));
      }
    })
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Resultaat:', result),
  error: err => console.error('Fout:', err.message)
});

// Voorbeeld uitvoer:
// Herhalingen 1/3
// Herhalingen 2/3
// Resultaat: Succesvolle gegevensverwerving
```

## Relatie met Pipeable Operator

Lus-genererende Creation Function hebben geen directe tegenhanger Pipeable Operator. Ze worden altijd gebruikt als Creation Function.

Ze kunnen echter worden gecombineerd met de volgende operatoren voor meer geavanceerde verwerking.

```typescript
import { range, map } from 'rxjs';
// 1naar5Opeenvolgende nummers genereren uit
range(1, 5).subscribe(console.log);
// Uitvoer: 1, 2, 3, 4, 5

// Gebruiken in batchverwerking
range(0, 10).pipe(
  map(i => `Verwerking${i + 1}`)
).subscribe(console.log);
// Uitvoer: Verwerking1, Verwerking2, ..., Verwerking10
```

## Opmerkingen over de prestaties.

Lus-genererende Creation Function geven waarden synchroon uit, dus er moet rekening worden gehouden met de prestaties als er grote aantallen waarden worden gegenereerd.


```typescript
import { range, map } from 'rxjs';
// 1naar5Opeenvolgende nummers genereren uit
range(1, 5).subscribe(console.log);
// Uitvoer: 1, 2, 3, 4, 5

// Gebruiken in batchverwerking
range(0, 10).pipe(
  map(i => `Verwerking${i + 1}`)
).subscribe(console.log);
// Uitvoer: Verwerking1, Verwerking2, ..., Verwerking10
```

> **Handelen van grote hoeveelheden gegevens**:.
> - Grote hoeveelheden gegevens, zoals `range(1, 1000000)`, worden allemaal synchroon uitgegeven en verbruiken geheugen.
> - Buffer met `bufferCount()` of `windowCount()` zoals vereist
> - of verander naar asynchrone uitvoering door een scheduler op te geven met `scheduled()`.


```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// Uitgevoerd door asynchrone planner
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## Volgende stappen.

Klik op de links in de bovenstaande tabel voor meer informatie over de werking van elke Creation Function en praktische voorbeelden.

U kunt ook meer te weten komen over [basis Creation Function](/nl/guide/creation-functions/basic/), [combined Creation Function](/nl/guide/creation-functions/combination/), [selection and Creation Function](/nl/guide/creation-functions/selection/) en [Creation Function](/nl/guide/creation-functions/conditional/). Dit zal je helpen om het hele plaatje van Creation Function te begrijpen.
