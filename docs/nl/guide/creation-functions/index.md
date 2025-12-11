---
description: Een uitgebreide uitleg van RxJS Creation Functions (Observable-creatiefuncties), inclusief de verschillen met Pipeable Operators, basisgebruik en zeven categorieën (basis creatie, lus generatie, HTTP-communicatie, combinatie, selectie/partitie, conditionele vertakking en controlesystemen).
---

# Creation Functions

In RxJS zijn er twee verschillende vormen: **Creation Functions** voor het maken van Observables en **Pipeable Operators** voor het transformeren van bestaande Observables.

Deze pagina beschrijft de basisconcepten van Creation Functions en de zeven hoofdcategorieën.

## Wat zijn Creation Functions?

**Creation Functions** zijn functies voor het maken van nieuwe Observables.

```typescript
import { of, from, interval } from 'rxjs';

// Gebruiken als Creation Functions
const obs1$ = of(1, 2, 3);
const obs2$ = from([4, 5, 6]);
const obs3$ = interval(1000);
```

Ze worden rechtstreeks geïmporteerd uit het `rxjs`-pakket en als functies aangeroepen om Observables te maken.

## Verschil met Pipeable Operators

Creation Functions en Pipeable Operators hebben verschillende toepassingen en gebruiksmogelijkheden. Zie de onderstaande tabel voor de verschillen tussen beide.

| Kenmerk | Creation Function | Pipeable Operator |
|---------|-------------------|-------------------|
| **Doel** | Nieuwe Observable maken | Bestaande Observable transformeren |
| **Importeren van** | `rxjs` | `rxjs/operators` |
| **Gebruik** | Direct aanroepen als functie | Gebruiken binnen `.pipe()` |
| **Voorbeeld** | `concat(obs1$, obs2$)` | `obs1$.pipe(concatWith(obs2$))` |

### Voorbeeld van Creation Function

Creation Functions worden gebruikt om meerdere Observables direct te combineren.

```typescript
import { concat, of } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Gebruiken als Creation Function
concat(obs1$, obs2$).subscribe(console.log);
// Output: 1, 2, 3, 4, 5, 6
```

### Voorbeeld van Pipeable Operator

De Pipeable Operator wordt gebruikt om een transformatieproces toe te voegen aan een bestaande Observable.

```typescript
import { of } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Gebruiken als Pipeable Operator
obs1$.pipe(
  concatWith(obs2$)
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5, 6
```

## Gebruikscriteria

De keuze tussen Creation Function en Pipeable Operator wordt bepaald door de volgende criteria.

### Wanneer Creation Function te gebruiken

De Creation Function is geschikt wanneer meerdere Observables op hetzelfde niveau moeten worden uitgevoerd of wanneer een Observable vanaf nul moet worden gemaakt.

- **Bij het combineren van meerdere Observables op hetzelfde niveau**
  ```typescript
  concat(obs1$, obs2$, obs3$)
  merge(click$, hover$, scroll$)
  ```

- **Bij het maken van een Observable vanaf nul**
  ```typescript
  of(1, 2, 3)
  from([1, 2, 3])
  interval(1000)
  ```

### Wanneer Pipeable Operator te gebruiken

De Pipeable Operator is geschikt voor het toevoegen van verwerking aan een bestaande Observable of voor het koppelen van meerdere bewerkingen.

- **Bij het toevoegen van bewerkingen aan een bestaande Observable**
  ```typescript
  obs1$.pipe(
    map(x => x * 2),
    concatWith(obs2$),
    filter(x => x > 5)
  )
  ```

- **Bij het koppelen van meerdere bewerkingen als een pipeline**

## Categorieën van Creation Functions

In dit hoofdstuk worden Creation Functions in zeven categorieën verdeeld.

### Overzicht van alle categorieën

In de onderstaande tabel kunt u alle categorieën en de functies die ze bevatten zien. Klik op elke functienaam om naar de detailpagina te gaan.

| Categorie | Beschrijving | Belangrijkste functies | Typische use cases |
|-----------|--------------|------------------------|-------------------|
| **[Basis Creatie](/nl/guide/creation-functions/basic/)** | Meest basale en frequent gebruikte functies. Maak data-, array-, event- en tijdgebaseerde Observables | [of](/nl/guide/creation-functions/basic/of), [from](/nl/guide/creation-functions/basic/from), [fromEvent](/nl/guide/creation-functions/basic/fromEvent), [interval](/nl/guide/creation-functions/basic/interval), [timer](/nl/guide/creation-functions/basic/timer) | Testen met vaste waarden, streamen van bestaande data, DOM event handling, polling, vertraagde uitvoering |
| **[Lus Generatie](/nl/guide/creation-functions/loop/)** | Druk lusverwerking zoals for/while-statements uit in Observable | [range](/nl/guide/creation-functions/loop/range), [generate](/nl/guide/creation-functions/loop/generate) | Sequentiële nummergeneratie, batchverwerking, complexe statusovergangen, wiskundige berekeningen |
| **[HTTP-communicatie](/nl/guide/creation-functions/http-communication/)** | Behandel HTTP-communicatie als Observable | [ajax](/nl/guide/creation-functions/http-communication/ajax), [fromFetch](/nl/guide/creation-functions/http-communication/fromFetch) | XMLHttpRequest-gebaseerde HTTP-communicatie, Fetch API-gebaseerde HTTP-communicatie, REST API-aanroepen |
| **[Combinatie](/nl/guide/creation-functions/combination/)** | Combineer meerdere Observables tot één. Emissietiming en volgorde verschillen afhankelijk van de combinatiemethode | [concat](/nl/guide/creation-functions/combination/concat), [merge](/nl/guide/creation-functions/combination/merge), [combineLatest](/nl/guide/creation-functions/combination/combineLatest), [zip](/nl/guide/creation-functions/combination/zip), [forkJoin](/nl/guide/creation-functions/combination/forkJoin) | Stapsgewijze verwerking, integratie van meerdere events, synchronisatie van formulierinvoer, wachten op voltooiing van parallelle API-aanroepen |
| **[Selectie/Partitie](/nl/guide/creation-functions/selection/)** | Selecteer één uit meerdere Observables of partitioneer één Observable in meerdere | [race](/nl/guide/creation-functions/selection/race), [partition](/nl/guide/creation-functions/selection/partition) | Competitie tussen meerdere databronnen, succes/faalvertakking |
| **[Conditioneel](/nl/guide/creation-functions/conditional/)** | Selecteer Observable op basis van voorwaarden of genereer dynamisch tijdens abonnement | [iif](/nl/guide/creation-functions/conditional/iif), [defer](/nl/guide/creation-functions/conditional/defer) | Verwerkingsvertakking op basis van inlogstatus, dynamische Observable-creatie, lazy evaluatie |
| **[Controle](/nl/guide/creation-functions/control/)** | Controleer Observable-uitvoeringstiming en resourcebeheer | [scheduled](/nl/guide/creation-functions/control/scheduled), [using](/nl/guide/creation-functions/control/using) | Uitvoeringstimingcontrole met scheduler, resource lifecycle management, geheugenlek preventie |

> [!TIP]
> **Leervolgorde**
>
> We raden beginners aan om in de volgende volgorde te leren:
> 1. **Basis Creatie** - Fundamentele RxJS-functies
> 2. **Combinatie** - Basis van het werken met meerdere streams
> 3. **HTTP-communicatie** - Praktische API-integratie
> 4. Andere categorieën - Leer indien nodig

## Correspondentie met Pipeable Operators

Veel Creation Functions hebben een corresponderende Pipeable Operator. Bij gebruik in een pipeline, gebruik een operator van de `~With`-familie.

| Creation Function | Pipeable Operator | Opmerkingen |
|-------------------|-------------------|-------------|
| `concat(a$, b$)` | `a$.pipe(`**[concatWith](/nl/guide/operators/combination/concatWith)**`(b$))` | RxJS 7+ |
| `merge(a$, b$)` | `a$.pipe(`**[mergeWith](/nl/guide/operators/combination/mergeWith)**`(b$))` | RxJS 7+ |
| `zip(a$, b$)` | `a$.pipe(`**[zipWith](/nl/guide/operators/combination/zipWith)**`(b$))` | RxJS 7+ |
| `combineLatest([a$, b$])` | `a$.pipe(`**[combineLatestWith](/nl/guide/operators/combination/combineLatestWith)**`(b$))` | RxJS 7+ |
| `race(a$, b$)` | `a$.pipe(`**[raceWith](/nl/guide/operators/combination/raceWith)**`(b$))` | RxJS 7+ |

> [!NOTE]
> Sinds RxJS 7 zijn **[concatWith](/nl/guide/operators/combination/concatWith)**, **[mergeWith](/nl/guide/operators/combination/mergeWith)**, **[zipWith](/nl/guide/operators/combination/zipWith)**, **[combineLatestWith](/nl/guide/operators/combination/combineLatestWith)**, **[raceWith](/nl/guide/operators/combination/raceWith)** en andere `~With`-type operators toegevoegd, waardoor ze gemakkelijker te gebruiken zijn als Pipeable Operators.

## Welke moet ik gebruiken?

De keuze tussen Creation Function en Pipeable Operator hangt af van de context.

### Creation Function wordt aanbevolen

Als meerdere Observables op hetzelfde niveau moeten worden uitgevoerd, vereenvoudigt de Creation Function de code.

```typescript
// ✅ Combineer meerdere Observables op hetzelfde niveau
const combined$ = merge(
  fromEvent(button1, 'click'),
  fromEvent(button2, 'click'),
  fromEvent(button3, 'click')
);
```

### Pipeable Operator wordt aanbevolen

Bij het toevoegen van bewerkingen als onderdeel van een pipeline, gebruik Pipeable Operator om de verwerkingsstroom te verduidelijken.

```typescript
// ✅ Combineren als onderdeel van pipeline
const result$ = source$.pipe(
  map(x => x * 2),
  mergeWith(other$),
  filter(x => x > 10)
);
```

## Samenvatting

- **Creation Functions**: Functies om Observables te maken en te combineren
- **Pipeable Operators**: Functies om bestaande Observables te transformeren
- Creation Functions vallen in 7 categorieën:
  1. **Basis Creatie**: Maak data-, array-, event- en tijdgebaseerde Observables
  2. **Lus Generatie**: Druk iteratieve verwerking uit in Observable
  3. **HTTP-communicatie**: Behandel HTTP-communicatie als Observable
  4. **Combinatie**: Combineer meerdere tot één
  5. **Selectie/Partitie**: Selecteer of partitioneer
  6. **Conditioneel**: Genereer dynamisch volgens voorwaarden
  7. **Controle**: Controleer uitvoeringstiming en resourcebeheer
- Gebruik `~With`-familie Pipeable Operators in pipelines
- Elke categorie bevat meerdere functies en kan op verschillende manieren worden gebruikt afhankelijk van de toepassing

## Volgende stappen

Om meer te leren over elke categorie, volg de onderstaande links:

1. **[Basis Creatie Functies](/nl/guide/creation-functions/basic/)** - of, from, fromEvent, interval, timer
2. **[Lus Generatie Functies](/nl/guide/creation-functions/loop/)** - range, generate
3. **[HTTP-communicatie Functies](/nl/guide/creation-functions/http-communication/)** - ajax, fromFetch
4. **[Combinatie Functies](/nl/guide/creation-functions/combination/)** - concat, merge, combineLatest, zip, forkJoin
5. **[Selectie/Partitie Functies](/nl/guide/creation-functions/selection/)** - race, partition
6. **[Conditionele Functies](/nl/guide/creation-functions/conditional/)** - iif, defer
7. **[Controle Functies](/nl/guide/creation-functions/control/)** - scheduled, using

Op elke pagina leert u meer over hoe Creation Functions werken en praktische voorbeelden.

## Referentiebronnen

- [RxJS Officiële Documentatie - Creation Functions](https://rxjs.dev/guide/operators#creation-operators-list)
- [Learn RxJS - Creation Operators](https://www.learnrxjs.io/learn-rxjs/operators/creation)
