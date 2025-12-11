---
description: "TestScheduler is een krachtig hulpmiddel om tijdgebaseerde operators van RxJS te testen met virtuele tijd. Dit artikel legt uit over marble notatie, het maken van Cold en Hot Observables, het vooruitschuiven van tijd, het testen van debounceTime en delay, en type-veilige implementatie in TypeScript."
---

# Testen met TestScheduler

RxJS's `TestScheduler` is een krachtig hulpmiddel voor het nauwkeurig testen van tijdgebaseerde operators. Dit hoofdstuk legt systematisch uit hoe je TestScheduler kunt gebruiken.

## Wat is TestScheduler?

Normaal gesproken werken Observables tijdafhankelijk. Bijvoorbeeld, `delay()` en `debounceTime()` zijn operators die een bepaalde tijd wachten.
Omdat het inefficiënt is om daadwerkelijk te wachten in tests, biedt `TestScheduler` een mechanisme om direct te testen met virtuele tijd.

> [!TIP]
> TestScheduler gebruikt "virtuele tijd", waardoor wachten in echte tijd niet nodig is.

## Basis configuratie van TestScheduler

```ts
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('TestScheduler basis', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('eenvoudige test', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b--c|');
      const expected =  '--a--b--c|';

      expectObservable(source$).toBe(expected);
    });
  });
});
```

- `cold()`: Creëert een Cold Observable waarbij de stream onafhankelijk start bij elke subscriptie
- `hot()`: Creëert een Hot Observable waarbij de stream al bezig is
- `expectObservable()`: Verifieert de output van de Observable met marble notatie


## Cold Observable en Hot Observable

|Type|Kenmerken|Gebruik|
|:---|:---|:---|
|Cold Observable|Stroomt data vanaf het begin bij elke subscriptie|HTTP requests, etc.|
|Hot Observable|De datastroom is al begonnen en wordt gedeeld met subscribers|Gebruikersevents, WebSocket, etc.|


## Basis van Marble Notatie

Marble notatie is een techniek om het tijdsverloop van Observables weer te geven met strings.

|Symbool|Betekenis|
|:---|:---|
|`-`|Tijdsverloop (1 frame)|
|`a`, `b`, `c`|Uitgegeven waarden|
|`|`|Voltooiing|
|`#`|Fout|
|`() `|Meerdere waarden tegelijk uitgeven (meerdere events)|

#### Voorbeeld

```
--a--b--c|    // Na 2 frames a, daarna b, c, voltooid
```


## Testvoorbeeld met virtuele tijd

### Test van debounceTime

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { debounceTime, map } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Tests met virtuele tijd', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test van debounceTime operator', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20),
        map(x => x.toUpperCase())
      );
      const expected =    '-----------(C|)';  // ← Dit!

      expectObservable(result$).toBe(expected, { B: 'B', C: 'C' });
    });
  });
});
```


## Testen van error handling

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { catchError} from 'rxjs';
import { TestScheduler } from 'rxjs/testing';
import { of } from 'rxjs';

describe('Error handling tests', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('fout opvangen met catchError', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--#');
      const result$ = source$.pipe(
        catchError(() => of('X'))
      );

      const expected =    '--a--(X|)';

      expectObservable(result$).toBe(expected);
    });
  });
});
```


## Samenvatting

- Met TestScheduler kun je testen zonder te wachten op echte tijd
- Begrijp het verschil tussen Cold/Hot Observables en gebruik ze correct
- Gebruik marble notatie om tijdsverloop te visualiseren
- Ook complexe asynchrone streams kunnen nauwkeurig worden getest

> [!NEXT]
> Vervolgens leer je over meer geavanceerde marble testing (aanpassen van marble strings en combinaties van meerdere streams).
