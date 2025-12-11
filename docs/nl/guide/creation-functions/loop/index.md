---
description: Deze sectie beschrijft Creation Functions die waarden genereren op een lusachtige manier, waarbij range en generate worden gebruikt om te leren hoe iteratieve verwerking zoals for- en while-statements als Observable streams te implementeren. Van sequentiële nummergeneratie tot complexe statusovergangen op basis van aangepaste voorwaarden, u kunt declaratieve lusverwerking realiseren met gebruikmaking van TypeScript's type-inferentie.
---

# Lus Generatie Creation Functions

Creation Functions voor het uitdrukken van lusverwerking zoals for- en while-statements als Observable.

## Wat zijn Lus Generatie Creation Functions?

Lus Generatie Creation Functions realiseren reactief repetitieve verwerking. Door conventionele imperatieve lussen (`for`- en `while`-statements) te vervangen door declaratieve Observable streams, is flexibele verwerking mogelijk in combinatie met de RxJS operator chain.

Bekijk de onderstaande tabel om de kenmerken en het gebruik van elke Creation Function te zien.

## Belangrijke Lus Generatie Creation Functions

| Function | Beschrijving | Gebruikssituaties |
|----------|--------------|-------------------|
| **[range](/nl/guide/creation-functions/loop/range)** | Genereer een reeks getallen (zoals for-statement) | Sequentiële nummergeneratie, batchverwerking |
| **[generate](/nl/guide/creation-functions/loop/generate)** | Algemene lusgeneratie (zoals while-statement) | Voorwaardelijke herhaling, complexe statusovergangen |

## Gebruikscriteria

De selectie van Lus Generatie Creation Functions wordt bepaald vanuit de volgende perspectieven.

### 1. Generatiepatroon

- **Numerieke reeks**: `range()` - Eenvoudige sequentiële nummergeneratie met start- en eindwaarden
- **Complexe voorwaarden**: `generate()` - Vrije controle over initiële waarden, voorwaarden, iteratie en resultaatselectie

### 2. Lustypes

- **for-statement-achtige lus**: `range()` - `for (let i = start; i <= end; i++)`
- **while-statement-achtige lus**: `generate()` - `while (condition) { ... }`

### 3. Flexibiliteit

- **Eenvoudig is voldoende**: `range()` - Wanneer u een reeks getallen nodig hebt
- **Geavanceerde controle nodig**: `generate()` - Aangepast statusbeheer, voorwaardelijke vertakkingen, stapcontrole

## Praktische Gebruiksvoorbeelden

### range() - Sequentiële Nummergeneratie

Voor eenvoudige sequentiële nummergeneratie is `range()` de beste keuze.

```typescript
import { range, map } from 'rxjs';
// Genereer sequentiële nummers van 1 tot 5
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// Gebruik in batchverwerking
range(0, 10).pipe(
  map(i => `Proces ${i + 1}`)
).subscribe(console.log);
// Output: Proces 1, Proces 2, ..., Proces 10
```

### generate() - Voorwaardelijke Lus

Gebruik `generate()` voor complexe voorwaarden of aangepast statusbeheer.

```typescript
import { generate } from 'rxjs';

// Genereer Fibonacci-reeks (eerste 10 termen)
generate(
  { current: 0, next: 1, count: 0 },  // Initiële status
  state => state.count < 10,           // Doorgaande voorwaarde
  state => ({                          // Status update
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // Resultaatselector
).subscribe(console.log);
// Output: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## Vergelijking met Imperatieve Lus

Dit is een vergelijking tussen de conventionele imperatieve lus en RxJS's Lus Generatie Creation Functions.

### Imperatief for-Statement

```typescript
// Conventioneel for-statement
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### Declaratieve range()

```typescript
import { range, map, toArray } from 'rxjs';
// RxJS range()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

> [!TIP]
> **Voordelen van de declaratieve benadering**:
> - Verbeterde leesbaarheid met pipelineverwerking
> - Uniforme foutafhandeling
> - Gemakkelijk te combineren met asynchrone verwerking
> - Gemakkelijk te annuleren en af te breken (bijv. `takeUntil()`)

## Conversie van Cold naar Hot

Zoals weergegeven in de bovenstaande tabel, **genereren alle Lus Generatie Creation Functions Cold Observables**. Elk abonnement initieert een onafhankelijke uitvoering.

Door gebruik te maken van multicast operators (`share()`, `shareReplay()`, etc.) kunt u echter **een Cold Observable converteren naar een Hot Observable**.

### Praktisch Voorbeeld: Berekeningsresultaten Delen

```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - Onafhankelijke berekening voor elk abonnement
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('Berekenen:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('Abonnee 1:', val));
cold$.subscribe(val => console.log('Abonnee 2:', val));
// → Berekening twee keer uitgevoerd (2000 berekeningen)

// 🔥 Hot - Deel berekeningsresultaten tussen abonnees
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('Berekenen:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('Abonnee 1:', val));
hot$.subscribe(val => console.log('Abonnee 2:', val));
// → Berekening slechts één keer uitgevoerd (1000 berekeningen)
```

> [!TIP]
> **Gevallen waarin Hot-conversie vereist is**:
> - Gebruik kostbare berekeningen op meerdere locaties
> - Deel batchverwerkingsresultaten met meerdere componenten
> - Toon pagineringsresultaten in meerdere UI-componenten
>
> Voor meer informatie, zie [Basis Creatie - Conversie van Cold naar Hot](/nl/guide/creation-functions/basic/#conversie-van-cold-naar-hot).

## Gecombineerd met Asynchrone Verwerking

Lus Generatie Creation Functions tonen krachtige functionaliteit wanneer gecombineerd met asynchrone verwerking.

### Sequentiële Uitvoering van API-aanroepen

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
    items: [`Gegevens${page}-1`, `Gegevens${page}-2`, `Gegevens${page}-3`]
  }).pipe(
    delay(300) // Simuleer API-aanroep
  );
}

// Haal sequentieel pagina's 1 tot 10 op (met 1 seconde vertraging tussen elk verzoek)
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe(
  data => console.log(`Pagina ${data.page} opgehaald:`, data.items),
  err => console.error('Fout:', err)
);
```

### Gebruik in Retry-verwerking

```typescript
import { range, throwError, of, Observable, mergeMap, retryWhen, delay } from 'rxjs';
// Functie om het ophalen van gegevens te simuleren (faalt willekeurig)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // 40% succespercentage

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Gegevens ophalen mislukt'))
        : of('Gegevens succesvol opgehaald')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          // Probeer tot 3 keer opnieuw
          if (index >= 3) {
            return throwError(() => error);
          }
          console.log(`Opnieuw proberen ${index + 1}/3`);
          // Exponentiële backoff: 1s, 2s, 4s
          return range(0, 1).pipe(delay(Math.pow(2, index) * 1000));
        })
      )
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Resultaat:', result),
  error: err => console.error('Fout:', err.message)
});

// Voorbeelduitvoer:
// Opnieuw proberen 1/3
// Opnieuw proberen 2/3
// Resultaat: Gegevens succesvol opgehaald
```

## Relatie met Pipeable Operator

Lus Generatie Creation Functions hebben geen directe Pipeable Operator tegenhanger. Ze worden altijd gebruikt als Creation Functions.

Geavanceerdere verwerking is echter mogelijk door ze te combineren met de volgende operators:

| Te Combineren Operators | Doel |
|-------------------------|------|
| `map()` | Transformeer elke waarde |
| `filter()` | Geef alleen waarden door die aan de voorwaarde voldoen |
| `take()`, `skip()` | Controleer het aantal waarden |
| `concatMap()`, `mergeMap()` | Voer asynchrone verwerking uit voor elke waarde |
| `toArray()` | Verzamel alle waarden in een array |

## Prestatie-opmerkingen

Lus Generatie Creation Functions geven waarden synchroon uit, dus wees voorzichtig met de prestaties bij het genereren van een groot aantal waarden.

> [!WARNING]
> **Omgaan met grote hoeveelheden gegevens**:
> - Grote hoeveelheden gegevens, zoals `range(1, 1000000)`, worden allemaal synchroon uitgegeven en verbruiken geheugen
> - Buffer indien nodig met `bufferCount()` of `windowCount()`
> - Of verander naar asynchrone uitvoering door een scheduler op te geven met `scheduled()`

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// Voer uit met asynchrone scheduler
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## Volgende Stappen

Om meer te leren over het gedetailleerde gedrag en praktische voorbeelden van elke Creation Function, klik op de links in de bovenstaande tabel.

U kunt ook het volledige beeld van Creation Functions begrijpen door [Basis Creation Functions](/nl/guide/creation-functions/basic/), [Combinatie Creation Functions](/nl/guide/creation-functions/combination/), [Selectie/Partitie Creation Functions](/nl/guide/creation-functions/selection/) en [Voorwaardelijke Creation Functions](/nl/guide/creation-functions/conditional/) te leren.
