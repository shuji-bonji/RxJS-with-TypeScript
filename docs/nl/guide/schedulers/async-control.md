---
description: Uitleg over het gebruik van schedulers in RxJS en hoe je asynchrone verwerking kunt controleren met observeOn en subscribeOn. Ontdek praktische technieken voor uitvoeringstiming-controle, uitvoeringscontext-beheer, taakprioritering, prestatie-optimalisatie en het voorkomen van UI-blokkering met TypeScript codevoorbeelden.
---
# Controle van asynchrone verwerking

Schedulers in RxJS zijn een belangrijk mechanisme voor het controleren van de timing en uitvoeringscontext van asynchrone verwerking. In dit hoofdstuk leggen we uit hoe je asynchrone verwerking kunt controleren met behulp van schedulers.

## De rol van schedulers

Schedulers vervullen drie belangrijke rollen:

|Rol|Beschrijving|
|---|---|
|Controle van uitvoeringstiming|Bepalen wanneer taken worden uitgevoerd|
|Beheer van uitvoeringscontext|Bepalen in welke thread of uitvoeringsomgeving taken worden uitgevoerd|
|Taakprioritering|Beheren van de uitvoeringsvolgorde van meerdere taken|

## Begrip van synchrone en asynchrone verwerking

### Standaardgedrag (synchrone uitvoering)

RxJS-operators worden standaard waar mogelijk synchron uitgevoerd.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

console.log('Start uitvoering');

of(1, 2, 3)
  .pipe(
    map((x) => {
      console.log(`map: ${x}`);
      return x * 2;
    })
  )
  .subscribe((x) => console.log(`subscribe: ${x}`));

console.log('Einde uitvoering');

// Output:
// Start uitvoering
// map: 1
// subscribe: 2
// map: 2
// subscribe: 4
// map: 3
// subscribe: 6
// Einde uitvoering
```

### Asynchroon maken met schedulers

Door schedulers te gebruiken, kun je verwerking asynchroon maken.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Start uitvoering');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(x => console.log(`subscribe: ${x}`));

console.log('Einde uitvoering');

// Output:
// Start uitvoering
// Einde uitvoering
// subscribe: 1
// subscribe: 2
// subscribe: 3
```

## Operators die schedulers gebruiken

### observeOn operator

`observeOn` wijzigt de uitvoeringscontext van de stream. Het laat waarden uitgeven met de opgegeven scheduler.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { take, observeOn } from 'rxjs';

// Gebruiksvoorbeeld voor animatie
interval(16)
  .pipe(
    take(10),
    observeOn(animationFrameScheduler)
  )
  .subscribe(() => {
    // Uitvoeren gesynchroniseerd met animatieframe
    updateAnimation();
  });

function updateAnimation() {
  // Animatie-update verwerking
}
```

> [!TIP]
> Voor een gedetailleerde uitleg, praktische voorbeelden en belangrijke aandachtspunten van de `observeOn` operator, zie de [observeOn](/nl/guide/operators/utility/observeOn.md) operator pagina.

### subscribeOn operator

`subscribeOn` controleert de timing van het starten van de stream-subscriptie.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, tap } from 'rxjs';

console.log('Voor subscribe start');

of('Taak uitvoeren')
  .pipe(
    tap(() => console.log('Taak start')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(value => console.log(value));

console.log('Na subscribe start');

// Output:
// Voor subscribe start
// Na subscribe start
// Taak start
// Taak uitvoeren
```

> [!TIP]
> Voor een gedetailleerde uitleg, praktische voorbeelden en het verschil met `observeOn`, zie de [subscribeOn](/nl/guide/operators/utility/subscribeOn.md) operator pagina.

## Praktische voorbeelden van asynchrone verwerking

### API-verzoek controle

```ts
import { from, queueScheduler } from 'rxjs';
import { mergeMap, observeOn, tap } from 'rxjs';

interface ApiRequest {
  endpoint: string;
  id: number;
}

const requests: ApiRequest[] = [
  { endpoint: '/users', id: 1 },
  { endpoint: '/posts', id: 1 },
  { endpoint: '/comments', id: 1 },
];

// Verzoeken in wachtrij plaatsen en op volgorde verwerken
from(requests)
  .pipe(
    observeOn(queueScheduler),
    tap((req) => console.log(`Toegevoegd aan wachtrij: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // Simulatie van daadwerkelijk API-verzoek
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`Resultaat van ${req.endpoint}/${req.id}`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`Voltooid: ${result}`));

// Output:
// Toegevoegd aan wachtrij: /users
// Toegevoegd aan wachtrij: /posts
// Toegevoegd aan wachtrij: /comments
// Voltooid: Resultaat van /users/1
// Voltooid: Resultaat van /posts/1
// Voltooid: Resultaat van /comments/1
```

### Voorkomen van UI-thread blokkering

Bij het verwerken van grote hoeveelheden data gebruik je schedulers om te voorkomen dat de UI-thread wordt geblokkeerd.

```ts
import { from, asapScheduler } from 'rxjs';
import { observeOn, bufferCount } from 'rxjs';

const largeDataSet = Array.from({ length: 10000 }, (_, i) => i);

// Batch-grootte
const batchSize = 100;
// Totaal aantal batches berekenen
const totalBatches = Math.ceil(largeDataSet.length / batchSize);
// Batch teller
let batchIndex = 0;

from(largeDataSet)
  .pipe(
    bufferCount(100), // Groepeer per 100 items
    observeOn(asapScheduler) // Zo snel mogelijk, maar zonder UI te blokkeren
  )
  .subscribe((batch) => {
    batchIndex++;
    processBatch(batch, batchIndex, totalBatches);
  });

function processBatch(
  batch: number[],
  batchIndex: number,
  totalBatches: number
) {
  // Batch data verwerking
  const processed = batch.map((n) => n * 2);
  console.log(
    `Batch ${batchIndex} van ${totalBatches} voltooid: ${processed.length} items verwerkt.`
  );
}

// Output:
// Batch 1 van 100 voltooid: 100 items verwerkt.
// Batch 2 van 100 voltooid: 100 items verwerkt.
// ...
// ...
// Batch 100 van 100 voltooid: 100 items verwerkt.
```

## Prestatie-optimalisatie en debugging

### Testen met schedulers

```ts
import { TestScheduler } from 'rxjs/testing';
import { delay } from 'rxjs';
import { beforeEach, describe, expect, it } from 'vitest';

describe('Test van asynchrone verwerking', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Test van delay operator', () => {
    scheduler.run(({ cold, expectObservable }) => {
      const source = cold('a-b-c|');
      const expected =    '1000ms a-b-(c|)';

      const result = source.pipe(delay(1000, scheduler));

      expectObservable(result).toBe(expected);
    });
  });
});
```

### Log-output voor debugging

```ts
import { of, asyncScheduler } from 'rxjs';
import { tap, observeOn } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    tap(value => console.log(`[Voor scheduler・synchroon] Waarde: ${value}`)),
    observeOn(asyncScheduler),  // Gebruik asyncScheduler
    tap(value => console.log(`[Na scheduler・asynchroon] Waarde: ${value}`))
  )
  .subscribe();

console.log('Einde');

// Daadwerkelijke output:
// Start
// [Voor scheduler・synchroon] Waarde: 1
// [Voor scheduler・synchroon] Waarde: 2
// [Voor scheduler・synchroon] Waarde: 3
// Einde
// [Na scheduler・asynchroon] Waarde: 1
// [Na scheduler・asynchroon] Waarde: 2
// [Na scheduler・asynchroon] Waarde: 3
```

Bij gebruik van `asyncScheduler` kun je het verwachte asynchrone gedrag bevestigen. `queueScheduler` gebruikt de microtask queue en wordt daarom uitgevoerd tijdens de uitvoering van synchrone code, terwijl `asyncScheduler` intern setTimeout gebruikt en dus volledig asynchroon wordt uitgevoerd.

## Voorbeeld dat verschillen in scheduler-gedrag toont
Dit voorbeeld toont de verschillen in uitvoeringstiming tussen verschillende schedulers.

```ts
import { of, queueScheduler, asyncScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

// Synchrone verwerking
of('sync').subscribe(value => console.log(`2: ${value}`));

// queueScheduler (microtask)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`3: ${value}`));

// asapScheduler (microtask)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`4: ${value}`));

// asyncScheduler (macrotask)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`5: ${value}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Einde');

// Daadwerkelijke uitvoervolgorde:
// 1: Start
// 2: sync
// 7: Einde
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```


## Best practices

1. **Gebruik schedulers alleen wanneer nodig**: Gebruik geen schedulers als het standaard synchrone gedrag voldoende is

2. **Kies de juiste scheduler**: Kies de optimale scheduler voor het gebruik
   - Animatie: `animationFrameScheduler`
   - Voorkomen UI-blokkering: `asapScheduler`
   - Wachtrij-verwerking: `queueScheduler`
   - Asynchrone verwerking: `asyncScheduler`

3. **Prestaties monitoren**: Monitor altijd de impact van scheduler-gebruik op prestaties

4. **Zorg voor testbaarheid**: Schrijf tests voor asynchrone verwerking met `TestScheduler`

## Veelgemaakte fouten en oplossingen

### Overmatig asynchroon maken

```ts
// ❌ Onnodige asynchronisatie
of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Dubbele asynchronisatie
    filter(x => x > 3)
  )
  .subscribe();

// ✅ Alleen asynchroon maken waar nodig
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    filter(x => x > 3),
    observeOn(asyncScheduler)  // Samen asynchroon maken op het einde
  )
  .subscribe();
```

### Verkeerd gebruik van schedulers

```ts
// ❌ Verkeerd gebruik
interval(1000)
  .pipe(
    subscribeOn(animationFrameScheduler)  // Heeft geen effect op interval
  )
  .subscribe();

// ✅ Correct gebruik
interval(1000, animationFrameScheduler)  // Specificeer scheduler bij creatie
  .subscribe();
```

## Samenvatting

Schedulers zijn een krachtig hulpmiddel voor het nauwkeurig controleren van asynchrone verwerking in RxJS. Door ze correct te gebruiken, kun je prestatie-optimalisatie, het voorkomen van UI-thread blokkering en eenvoudiger testen realiseren. Echter, overmatig asynchroon maken kan prestaties juist verslechteren, dus het is belangrijk om ze alleen te gebruiken wanneer nodig.

In de volgende sectie leggen we de specifieke soorten schedulers en hun gebruik in detail uit.
