---
description: "Uitleg over Creation Functions die meerdere Observables samenvoegen tot één. Leer de verschillen en toepassingen van concat, merge, combineLatest, zip, forkJoin en race, en typeveilige implementatiepatronen in TypeScript aan de hand van praktische codevoorbeelden."
---

# Combinatie Creation Functions

Belangrijke Creation Functions voor het samenvoegen van meerdere Observables tot één Observable.

## Wat zijn Combinatie Creation Functions

Combinatie Creation Functions accepteren meerdere Observables en voegen deze samen tot één Observable stream. Afhankelijk van de combinatiemethode verschillen de timing en volgorde van waarde-emissies.

Bekijk onderstaande tabel voor de kenmerken en toepassingen van elke Creation Function.

## Belangrijkste Combinatie Creation Functions

| Function | Beschrijving | Gebruikscasus |
|----------|--------------|---------------|
| **[concat](/nl/guide/creation-functions/combination/concat)** | Sequentiële combinatie (volgende start na voltooiing vorige) | Stapsgewijze verwerking |
| **[merge](/nl/guide/creation-functions/combination/merge)** | Parallelle combinatie (gelijktijdige subscriptie, output in emissievolgorde) | Integratie van meerdere events |
| **[combineLatest](/nl/guide/creation-functions/combination/combineLatest)** | Combineert laatste waarden | Synchronisatie van formulierinvoer |
| **[zip](/nl/guide/creation-functions/combination/zip)** | Koppelt corresponderende waarden | Koppeling van request en response |
| **[forkJoin](/nl/guide/creation-functions/combination/forkJoin)** | Wacht op voltooiing van alle en combineert eindwaarden | Wachten op parallelle API-aanroepen |

## Criteria voor Keuze

De selectie van Combinatie Creation Functions wordt bepaald op basis van de volgende aspecten.

### 1. Uitvoeringstiming

- **Sequentiële uitvoering**: `concat` - Start volgende Observable na voltooiing van de vorige
- **Parallelle uitvoering**: `merge`, `combineLatest`, `zip`, `forkJoin` - Alle Observables worden gelijktijdig gesubscribeerd

### 2. Waarde-emissiemethode

- **Alle waarden emitteren**: `concat`, `merge` - Output van alle waarden die door elke Observable worden uitgezonden
- **Laatste waarden combineren**: `combineLatest` - Combineert en emit alle laatste waarden telkens wanneer een van hen een waarde emit
- **Corresponderende waarden koppelen**: `zip` - Koppelt waarden op corresponderende posities van elke Observable
- **Alleen eindwaarden**: `forkJoin` - Emit een array van eindwaarden wanneer alle Observables zijn voltooid

### 3. Voltooi-timing

- **Na voltooiing van alle**: `concat`, `forkJoin` - Wacht tot alle Observables zijn voltooid
- **Voltooid bij kortste stream**: `zip` - Voltooid wanneer één Observable voltooid is, omdat resterende waarden niet kunnen worden gekoppeld
- **Voltooit niet**: `merge`, `combineLatest` - Zelfs als één voltooid is, voltooit het niet als anderen doorgaan

## Conversie van Cold naar Hot

Zoals aangegeven in bovenstaande tabel, **genereren alle Combinatie Creation Functions Cold Observables**. Bij elke subscriptie wordt een onafhankelijke uitvoering gestart.

Door gebruik te maken van multicasting operators (`share()`, `shareReplay()`, `publish()`, etc.) kun je echter **Cold Observables omzetten naar Hot Observables**.

### Praktisch voorbeeld: HTTP-request delen

```typescript
import { merge, interval } from 'rxjs';
import { map, take, share } from 'rxjs';

// ❄️ Cold - Onafhankelijk HTTP-request per subscriptie
const coldApi$ = merge(
  interval(1000).pipe(map(() => 'Bron A'), take(3)),
  interval(1500).pipe(map(() => 'Bron B'), take(2))
);

coldApi$.subscribe(val => console.log('Subscriber 1:', val));
coldApi$.subscribe(val => console.log('Subscriber 2:', val));
// → Elke subscriber voert onafhankelijk interval uit (dubbele requests)

// 🔥 Hot - Uitvoering gedeeld tussen subscribers
const hotApi$ = merge(
  interval(1000).pipe(map(() => 'Bron A'), take(3)),
  interval(1500).pipe(map(() => 'Bron B'), take(2))
).pipe(share());

hotApi$.subscribe(val => console.log('Subscriber 1:', val));
hotApi$.subscribe(val => console.log('Subscriber 2:', val));
// → Deelt één interval (slechts één request)
```

> [!TIP]
> **Gevallen waarin Hot-conversie nodig is**:
> - Dezelfde API-resultaten delen tussen meerdere componenten
> - Parallelle request-resultaten met `forkJoin` gebruiken op meerdere locaties
> - State beheren met `combineLatest` en distribueren naar meerdere subscribers
>
> Zie [Basis Creation - Conversie van Cold naar Hot](/nl/guide/creation-functions/basic/#cold-naar-hot-conversie) voor meer details.

## Correspondentie met Pipeable Operators

Voor Combinatie Creation Functions bestaan corresponderende Pipeable Operators. Bij gebruik binnen een pipeline gebruik je de `~With` operators.

| Creation Function | Pipeable Operator |
|-------------------|-------------------|
| `concat(a$, b$)` | `a$.pipe(concatWith(b$))` |
| `merge(a$, b$)` | `a$.pipe(mergeWith(b$))` |
| `zip(a$, b$)` | `a$.pipe(zipWith(b$))` |
| `combineLatest([a$, b$])` | `a$.pipe(combineLatestWith(b$))` |

## Volgende stappen

Klik op de links in bovenstaande tabel om de gedetailleerde werking en praktische voorbeelden van elke Creation Function te leren.

Daarnaast kun je door ook [Selectie & Partitie Creation Functions](/nl/guide/creation-functions/selection/) en [Conditionele Creation Functions](/nl/guide/creation-functions/conditional/) te bestuderen het volledige overzicht van Creation Functions begrijpen.
