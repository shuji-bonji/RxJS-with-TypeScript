---
description: "Marble testing is een methode om asynchrone streams van RxJS visueel te testen met behulp van strings. Dit artikel legt uit wat het verschil is tussen Cold en Hot Observables, de regels van marble notatie, het gebruik van TestScheduler, en hoe je complexe asynchrone verwerking kunt verifiëren met TypeScript codevoorbeelden."
---

# Introductie tot Marble Testing

RxJS biedt een methode genaamd "Marble Testing" waarmee je het gedrag van asynchrone streams **visueel kunt weergeven** tijdens het testen.

Hier leer je de basis van marble testing door eenvoudige voorbeelden.

## Wat is Marble Notatie?

Marble notatie is een methode om **tijdsverloop en gebeurtenissen** weer te geven met strings.

### Basisregels

| Symbool | Betekenis |
|:----|:----|
| `-` | Tijdsverloop (1 frame vooruit) |
| `a`, `b`, `c` | Uitgegeven waarden (willekeurige karakters) |
| `|` | Voltooiing (complete) |
| `#` | Fout (error) |

Bijvoorbeeld

```text
--a-b--c-|
```
Dit betekent:
- Wacht 2 frames en geef `a` uit
- 1 frame later `b`
- 2 frames later `c`
- Nog 1 frame later voltooid
wordt weergegeven.

## Verschil tussen Cold en Hot

### Cold Observable

Cold Observables worden "vanaf het begin afgespeeld bij elke subscriptie".

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

      expectObservable(source$).toBe('--a--b--c|');
    });
  });
});

```

### Hot Observable

Hot Observables zijn streams die "al bezig zijn".
Als je halverwege subscribet, ontvang je alleen de waarden vanaf dat moment.

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
    testScheduler.run(({ hot, expectObservable }) => {
      const source$ = hot('--a--b--c|');

      expectObservable(source$, '----^').toBe('-----b--c|');
    });
  });
});

```

## Eenvoudig voorbeeld van Marble Testing

Bijvoorbeeld bij het testen van de `debounceTime` operator

```ts
import { debounceTime } from 'rxjs';
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
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20)
      );

      const expected =    '-----------(c|)';

      expectObservable(result$).toBe(expected, { c: 'c' });
    });
  });
});

```

Hier wordt geverifieerd dat alleen de laatst uitgegeven `c` wordt uitgevoerd.

## Aandachtspunten

- Elk teken in marble notatie vertegenwoordigt standaard **1 frame (10ms)** (kan ingesteld worden afhankelijk van de omgeving)
- Tijdafhankelijke operators zoals `debounceTime`, `delay`, `interval` **werken goed samen met marble testing**
- Gebruik `expectObservable` om de output van de stream te verifiëren
- `expectSubscriptions` is een geavanceerde functie om het timing van subscripties te verifiëren, maar wordt hier niet behandeld

## Samenvatting

Marble testing is een zeer krachtige methode om RxJS-code tests te **visualiseren en intuïtief** te begrijpen.

- **Let op het verschil tussen Cold en Hot**
- **Druk tijdsverloop en gebeurtenissen uit met strings**
- **Zelfs complexe asynchrone streams kunnen duidelijk worden getest**

Begin met het oefenen van eenvoudige marble tests!
