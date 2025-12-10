---
description: "Marble testing per stream asincroni RxJS con rappresentazione visuale. Cold vs Hot Observable, notazione marble, TestScheduler ed esempi TypeScript."
---

# Introduzione al Marble Testing

In RxJS è disponibile una tecnica chiamata "marble testing" che permette di testare il comportamento degli stream asincroni **rappresentandoli visivamente**.

Qui impariamo le basi del marble testing attraverso semplici esempi.

## Cos'è la Notazione Marble?

La notazione marble è un metodo per rappresentare il **passare del tempo e il verificarsi di eventi** tramite stringhe.

### Regole di Base

| Simbolo | Significato |
|:----|:----|
| `-` | Passare del tempo (avanza di 1 frame) |
| `a`, `b`, `c` | Valori emessi (caratteri arbitrari) |
| `|` | Completamento (complete) |
| `#` | Errore (error) |

Ad esempio:

```text
--a-b--c-|
```

Questo significa:
- Dopo 2 frame viene emesso `a`
- Dopo 1 frame viene emesso `b`
- Dopo 2 frame viene emesso `c`
- Dopo 1 frame ulteriore, completamento

## Differenza tra Cold e Hot

### Cold Observable

I Cold Observable vengono "riprodotti dall'inizio per ogni sottoscrizione".

```ts
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Basi di TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test semplice', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b--c|');

      expectObservable(source$).toBe('--a--b--c|');
    });
  });
});
```

### Hot Observable

Gli Hot Observable sono stream "già in corso".
Se si sottoscrive a metà, si ricevono solo i valori da quel momento in poi.

```ts
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Basi di TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test semplice', () => {
    testScheduler.run(({ hot, expectObservable }) => {
      const source$ = hot('--a--b--c|');

      expectObservable(source$, '----^').toBe('-----b--c|');
    });
  });
});
```

## Esempio Semplice di Marble Testing

Ad esempio, quando si testa l'operatore `debounceTime`:

```ts
import { debounceTime } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Basi di TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test semplice', () => {
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

Qui si verifica che venga emesso solo l'ultimo valore `c`.

## Punti di Attenzione

- Nella notazione marble, 1 carattere rappresenta di default **1 frame (10ms)** (configurabile in base all'ambiente)
- Gli operatori dipendenti dal tempo come `debounceTime`, `delay`, `interval` **si integrano bene con il marble testing**
- Si usa `expectObservable` per verificare l'output dello stream
- `expectSubscriptions` è una funzione avanzata per verificare il timing delle sottoscrizioni, ma non viene trattata qui

## Riepilogo

Il marble testing è una tecnica molto potente che permette di **visualizzare e comprendere intuitivamente** i test del codice RxJS.

- **Essere consapevoli della differenza tra Cold e Hot**
- **Rappresentare il passare del tempo e gli eventi con stringhe**
- **Rendere possibili test chiari anche per stream asincroni complessi**

Iniziamo a esercitarci con semplici marble test!
