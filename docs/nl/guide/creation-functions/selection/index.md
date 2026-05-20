---
description: "Creation Function (race en partition) die een van meerdere Observable selecteren of een Observable opsplitsen in meerdere worden uitgelegd. Praktische gebruikssituaties en type-veilige implementaties in TypeScript, zoals conflictafhandeling, snelste reactieverwerving en stream-partitionering door voorwaardelijke vertakking worden gepresenteerd."
---

# Selection and partitioning systems Creation Function

Creation Function die een Observable selecteren uit meerdere Observables of een Observable opsplitsen in meerdere Observables volgens voorwaarden.

## Wat zijn Creation Function?

Creation Functions voor selectie- en splitsystemen verschillen van die voor combinatie-systemen en hebben de volgende rollen.

- **Selecteren**: selecteert uit meerdere Observables een Observable die aan bepaalde voorwaarden voldoet.
- **Splitsen**: splitst een Observable op in meerdere Observables volgens een voorwaarde.

Deze werken in de tegenovergestelde richting of vanuit een ander perspectief dan de join 'combineer meerdere in één'.

## Belangrijkste selectie- en verdeelsystemen Creation Function

| Functie | Beschrijving | Gebruikscasus. |
|---|---|---|
| **Race](/nl/guide/creation-functions/selection/race)** | Eerst gepubliceerde aannemen | Wedstrijd met meerdere gegevensbronnen |
| **[partition](/nl/guide/creation-functions/selection/partition)** | In tweeën splitsen met voorwaarden | Vertakkingsproces voor succes/falen |

## Criteria voor gebruik

### race - selecteer de snelste Observable

race` abonneert zich op meerdere Observables tegelijk en neemt de **eerste Observable aan die een waarde afgeeft. Observables die niet worden overgenomen, worden automatisch unsubscribe.

**Gebruiksgeval**:.
- Het snelste antwoord van meerdere API-eindpunten overnemen
- Time-out afhandeling (origineel proces vs. timer)
- Concurrentie tussen cache en daadwerkelijke API-aanroepen

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Neem de snelste uit meerdere gegevensbronnen over
const fast$ = timer(1000).pipe(map(() => 'Fast API'));
const slow$ = timer(3000).pipe(map(() => 'Slow API'));

race(fast$, slow$).subscribe(console.log);
// Uitvoer.: 'Fast API' (1De uitvoer wordt na een secondeslow$wordt geannuleerd)
```

### Partition - gesplitst per voorwaarde

De `partition` splitst een Observable in **twee Observables** op basis van een voorwaardelijke functie. De retourwaarde is een array `[indien waar, indien onwaar]`.

**Gebruiksgeval**:.
- Scheiding van succes en mislukking.
- Scheiding van even en oneven getallen.
- Scheiding van geldige en ongeldige gegevens.

```typescript
import { of, partition } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Opsplitsen in even en oneven nummers
const [even$, odd$] = partition(source$, n => n % 2 === 0);

even$.subscribe(val => console.log('Even:', val));
// Uitvoer.: Even: 2, Even: 4, Even: 6

odd$.subscribe(val => console.log('Odd:', val));
// Uitvoer.: Odd: 1, Odd: 3, Odd: 5
```

## Conversie van koud naar warm

Zoals de bovenstaande tabel laat zien, genereren **Alle Selection en Split System Creation Function een Cold Observable. Elke inschrijving initieert een onafhankelijke uitvoering.

Cold Observable kunnen worden geconverteerd naar Hot Observable door gebruik te maken van multicast-gebaseerde operatoren (`share()`, `shareReplay()`, etc.).

### Praktisch voorbeeld: delen van conflicterende API-verzoeken

```typescript
import { race, timer } from 'rxjs';
import { map, share } from 'rxjs';

// ❄️ Cold - Concurrentie herhalen voor elk abonnement
const coldRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
);

coldRace$.subscribe(val => console.log('Abonnee1:', val));
coldRace$.subscribe(val => console.log('Abonnee2:', val));
// → Elke abonnee voert een onafhankelijke competitie uit (een2competitie)

// 🔥 Hot - Wedstrijdresultaten delen tussen abonnees
const hotRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
).pipe(share());

hotRace$.subscribe(val => console.log('Abonnee1:', val));
hotRace$.subscribe(val => console.log('Abonnee2:', val));
// → 1Deel de resultaten van één competitie
```

> [!TIP]

> Zie voor meer informatie [Basissysteem voor creatie - Conversie van koud naar warm](/nl/guide/creation-functions/basic/#cold- to -hot-).

## Correspondentie met Pipeable Operator

Selection en division Creation Functions hebben ook een overeenkomstige Pipeable Operator.

| Creation Function | Pipeable Operator |
|---|---|
| race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(bron$, predikaat)` | Kan niet worden gebruikt binnen een pijplijn (Creation Function). |

__oproep_7___

> Er is geen Pipeable Operator versie van `partition`. Als een partition nodig is, kan deze worden gebruikt als een Creation Function of twee keer handmatig worden gesplitst met `filter`.

## Volgende stappen.

Klik op de links in de bovenstaande tabel voor meer informatie over de gedetailleerde werking en praktische voorbeelden van elke Creation Function.

U kunt ook de [Combinatie Creation Functies](/nl/guide/creation-functions/combination/) en [Voorwaardelijke Creation Functies](/nl/guide/creation-functions/conditional/) leren. Samen bieden ze een holistisch begrip van Creation Function.
