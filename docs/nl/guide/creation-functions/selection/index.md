---
description: Deze sectie biedt een overzicht van Selection en Partition Creation Functions die één Observable selecteren uit meerdere Observables of één Observable opsplitsen in meerdere Observables. Het legt uit hoe race en partition te gebruiken, evenals praktische voorbeelden.
---

# Selection/Partition Creation Functions

Dit zijn Creation Functions voor het selecteren van één Observable uit meerdere Observables of het opsplitsen van één Observable in meerdere Observables.

## Wat zijn Selection/Partition Creation Functions?

Selection/Partition Creation Functions zijn een verzameling functies die concurreren tussen meerdere Observables om de snelste te selecteren, of een Observable in twee streams opsplitsen op basis van voorwaarden. Dit is nuttig voor concurrerende gegevensbronnen of het toewijzen van verwerking op basis van voorwaarden.

Bekijk de onderstaande tabel om de kenmerken en het gebruik van elke Creation Function te zien.

## Belangrijkste Selection/Partition Creation Functions

| Functie | Beschrijving | Gebruikssituaties |
|----------|------|-------------|
| **[race](/nl/guide/creation-functions/selection/race)** | Selecteer de snelste Observable (degene die als eerste emiteert) | Concurrentie tussen meerdere gegevensbronnen, fallback-verwerking |
| **[partition](/nl/guide/creation-functions/selection/partition)** | Splits op in twee Observables op basis van een voorwaarde | Succes/fout-afhandeling, vertakking op basis van voorwaarden |

## Gebruikscriteria

De selectie van Selection/Partition Creation Functions wordt bepaald vanuit de volgende perspectieven.

### 1. Doel

- **Selecteer snelste uit meerdere bronnen**: `race` - Selecteer de eerste die een waarde emiteert tussen meerdere gegevensbronnen
- **Splits op voorwaarde**: `partition` - Splits één Observable in twee streams op basis van een voorwaarde

### 2. Emissietiming

- **Alleen de snelste**: `race` - Eenmaal geselecteerd, worden andere Observable-waarden genegeerd
- **Classificeer alle waarden**: `partition` - Alle waarden worden volgens voorwaarden in twee streams gesorteerd

### 3. Timing van voltooiing

- **Afhankelijk van geselecteerde Observable**: `race` - Volgt voltooiing van de Observable die als eerste emiteerde
- **Afhankelijk van originele Observable**: `partition` - Beide streams voltooien wanneer de originele Observable voltooit

## Praktische gebruiksvoorbeelden

### race() - Selecteer de snelste uit meerdere gegevensbronnen

Als u meerdere gegevensbronnen heeft en de snelst reagerende wilt gebruiken, gebruik dan `race()`.

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Simuleer meerdere API's
const api1$ = timer(1000).pipe(map(() => 'API1 Response'));
const api2$ = timer(500).pipe(map(() => 'API2 Response'));
const api3$ = timer(1500).pipe(map(() => 'API3 Response'));

// Gebruik de snelste respons
race(api1$, api2$, api3$).subscribe(console.log);
// Output: API2 Response (snelste bij 500ms)
```

### partition() - Splits in twee op basis van voorwaarde

Als u één Observable in twee streams wilt splitsen op basis van een voorwaarde, gebruik dan `partition()`.

```typescript
import { of } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// Splits in even en oneven getallen
const [evens$, odds$] = partition(numbers$, n => n % 2 === 0);

evens$.subscribe(n => console.log('Even:', n));
// Output: Even: 2, Even: 4, Even: 6, Even: 8, Even: 10

odds$.subscribe(n => console.log('Oneven:', n));
// Output: Oneven: 1, Oneven: 3, Oneven: 5, Oneven: 7, Oneven: 9
```

## Converteren van Cold naar Hot

Zoals in de bovenstaande tabel getoond, **genereren alle Selection/Partition Creation Functions Cold Observables**. Onafhankelijke uitvoering wordt gestart voor elk abonnement.

Door echter multicast-operators (`share()`, `shareReplay()`, enz.) te gebruiken, kunt u **een Cold Observable naar een Hot Observable converteren**.

### Praktisch voorbeeld: Uitvoering delen

```typescript
import { race, timer, share } from 'rxjs';
import { map } from 'rxjs';

// ❄️ Cold - Onafhankelijke uitvoering voor elk abonnement
const coldRace$ = race(
  timer(1000).pipe(map(() => 'API1')),
  timer(500).pipe(map(() => 'API2'))
);

coldRace$.subscribe(val => console.log('Abonnee 1:', val));
coldRace$.subscribe(val => console.log('Abonnee 2:', val));
// → Elke abonnee voert onafhankelijke race uit (2x verzoeken)

// 🔥 Hot - Deel uitvoering tussen abonnees
const hotRace$ = race(
  timer(1000).pipe(map(() => 'API1')),
  timer(500).pipe(map(() => 'API2'))
).pipe(share());

hotRace$.subscribe(val => console.log('Abonnee 1:', val));
hotRace$.subscribe(val => console.log('Abonnee 2:', val));
// → Deel race-uitvoering (verzoekt slechts één keer)
```

> [!TIP]
> **Gevallen waarin Hot-conversie vereist is**:
> - Deel het resultaat van `race()` tussen meerdere componenten
> - Gebruik het resultaat van `partition()` op meerdere locaties
> - Voer kostbare verwerking slechts één keer uit
>
> Voor meer informatie, zie [Basiscreatie - Converteren van Cold naar Hot](/nl/guide/creation-functions/basic/#converting-cold-to-hot).

## Correspondentie met Pipeable Operator

Voor Selection/Partition Creation Functions is er een corresponderende Pipeable Operator. Bij gebruik in een pipeline wordt de `~With` type operator gebruikt.

| Creation Function | Pipeable Operator |
|-------------------|-------------------|
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(source$, predicate)` | Geen directe correspondentie (gebruik als Creation Function) |

> [!NOTE]
> `partition()` wordt doorgaans gebruikt als Creation Function. Om streamsplitsing binnen een pipeline uit te voeren, gebruikt u operators zoals `filter()` in combinatie.

## Volgende stappen

Om het gedetailleerde gedrag en praktische voorbeelden van elke Creation Function te leren, klik op de links uit de bovenstaande tabel.

Ook door [Basis Creation Functions](/nl/guide/creation-functions/basic/), [Combinatie Creation Functions](/nl/guide/creation-functions/combination/) en [Voorwaardelijke Creation Functions](/nl/guide/creation-functions/conditional/) te leren, kunt u het totaalbeeld van Creation Functions begrijpen.
