---
description: Deze sectie beschrijft Creation Functions die Observables selecteren en creëren op basis van voorwaarden. Leer hoe je iif en defer gebruikt, evenals praktische voorbeelden.
---

# Voorwaardelijke Creation Functions

Creation Functions selecteren een Observable op basis van een voorwaarde of genereren dynamisch een Observable bij het abonneren.

## Wat zijn Voorwaardelijke Creation Functions?

Voorwaardelijke Creation Functions hebben de volgende rollen:

- **Voorwaardelijke Selectie**: Selecteer verschillende Observables volgens voorwaarden
- **Vertraagde Generatie**: Creëer dynamisch een Observable bij abonnement

In tegenstelling tot andere Creation Functions, die statisch Observables creëren en combineren, kunnen deze hun gedrag veranderen op basis van **runtime-voorwaarden en -toestanden**.

> [!NOTE]
> Hoewel `iif` en `defer` voorheen werden geclassificeerd als "voorwaardelijke operators", zijn het **Creation Functions** (Observable creatiefuncties), geen Pipeable Operators.

## Belangrijkste Voorwaardelijke Creation Functions

| Function | Beschrijving | Gebruikssituaties |
|----------|--------------|-------------------|
| **[iif](/nl/guide/creation-functions/conditional/iif)** | Selecteer een van twee Observables op basis van een voorwaarde | Verwerking vertakking op basis van login-status |
| **[defer](/nl/guide/creation-functions/conditional/defer)** | Vertraag generatie van Observable op abonnementstijdstip | Dynamische Observable creatie |

## Gebruikscriteria

### iif - Twee Takken op Basis van Voorwaarde

`iif` selecteert een van twee Observables afhankelijk van het resultaat van een voorwaardelijke functie. De voorwaarde wordt geëvalueerd **op het moment van abonneren**.

**Syntaxis**:
```typescript
iif(
  () => condition,  // Voorwaardelijke functie (geëvalueerd op abonnementstijdstip)
  trueObservable,   // Observable als waar
  falseObservable   // Observable als onwaar
)
```

**Gebruikssituaties**:
- Verwerking vertakking op basis van login-status
- Verwerking wisselen op basis van of cache bestaat
- Gedragsverandering door omgevingsvariabelen

```typescript
import { iif, of } from 'rxjs';

const isAuthenticated = () => Math.random() > 0.5;

const data$ = iif(
  isAuthenticated,
  of('Authenticated data'),
  of('Public data')
);

data$.subscribe(console.log);
// Output: 'Authenticated data' of 'Public data' (afhankelijk van voorwaarde op abonnementstijdstip)
```

### defer - Vertraagde Generatie op Abonnementstijdstip

`defer` genereert een Observable elke keer dat een abonnement plaatsvindt. Hierdoor kan de Observable zijn gedrag veranderen op basis van zijn toestand op het moment van abonneren.

**Syntaxis**:
```typescript
defer(() => {
  // Uitgevoerd op abonnementstijdstip
  return someObservable;
})
```

**Gebruikssituaties**:
- Genereer Observable die de laatste status weerspiegelt op het moment van abonneren
- Genereer elke keer een andere willekeurige waarde
- Voer verschillende verwerking uit voor elk abonnement

```typescript
import { defer, of } from 'rxjs';

// Krijg huidige tijd bij abonnement
const timestamp$ = defer(() => of(new Date().toISOString()));

setTimeout(() => {
  timestamp$.subscribe(time => console.log('First:', time));
}, 1000);

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Second:', time));
}, 2000);

// Output:
// First: 2024-10-21T01:00:00.000Z
// Second: 2024-10-21T01:00:01.000Z
// ※Verschillende tijden worden uitgevoerd omdat abonnementstijden verschillen
```

## Verschil Tussen iif en defer

| Kenmerk | iif | defer |
|---------|-----|-------|
| **Keuze** | Selecteer uit twee Observables | Genereer elke Observable |
| **Evaluatietiming** | Evalueer voorwaarde op abonnementstijdstip | Voer functie uit op abonnementstijdstip |
| **Doel** | Voorwaardelijke vertakking | Dynamische generatie |

## Gebruiken in Pipeline

Voorwaardelijke Creation Functions kunnen worden gebruikt in combinatie met andere operators.

```typescript
import { defer, of } from 'rxjs';
import { switchMap } from 'rxjs';

// Krijg gebruikersinformatie van gebruikers-ID
const userId$ = of(123);

userId$.pipe(
  switchMap(id =>
    defer(() => {
      // Controleer laatste cache op abonnementstijdstip
      const cached = cache.get(id);
      return cached ? of(cached) : fetchUser(id);
    })
  )
).subscribe(console.log);
```

## Converteren van Cold naar Hot

Zoals weergegeven in de tabel hierboven, **genereren alle Voorwaardelijke Creation Functions Cold Observables**. Voorwaardelijke evaluaties en generatiefuncties worden uitgevoerd elke keer dat een abonnement wordt gemaakt.

Je kunt een Cold Observable converteren naar een Hot Observable door multicast operators (`share()`, `shareReplay()`, etc.) te gebruiken.

### Praktisch Voorbeeld: Delen van Voorwaardelijke Vertakkingsresultaten

```typescript
import { iif, of, interval } from 'rxjs';
import { take, share } from 'rxjs';

const condition = () => Math.random() > 0.5;

// ❄️ Cold - Herevalueer voorwaarde voor elk abonnement
const coldIif$ = iif(
  condition,
  of('Condition is true'),
  interval(1000).pipe(take(3))
);

coldIif$.subscribe(val => console.log('Subscriber 1:', val));
coldIif$.subscribe(val => console.log('Subscriber 2:', val));
// → Elke abonnee evalueert onafhankelijk voorwaarde (mogelijkheid van verschillende resultaten)

// 🔥 Hot - Deel voorwaarde-evaluatieresultaten tussen abonnees
const hotIif$ = iif(
  condition,
  of('Condition is true'),
  interval(1000).pipe(take(3))
).pipe(share());

hotIif$.subscribe(val => console.log('Subscriber 1:', val));
hotIif$.subscribe(val => console.log('Subscriber 2:', val));
// → Voorwaarde slechts één keer geëvalueerd, resultaten gedeeld
```

> [!TIP]
> Voor meer informatie, zie [Basis Creatie - Converteren van Cold naar Hot](/nl/guide/creation-functions/basic/#converting-cold-to-hot).

## Volgende Stappen

Om het gedetailleerde gedrag en praktische voorbeelden van elke Creation Function te leren, klik op de links in de tabel hierboven.

Ook door het leren van [Combinatie Creation Functions](/nl/guide/creation-functions/combination/) en [Selectie/Partitie Creation Functions](/nl/guide/creation-functions/selection/), kun je het totaalbeeld van Creation Functions begrijpen.
