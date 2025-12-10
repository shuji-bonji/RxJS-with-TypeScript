---
description: "TestScheduler per operatori temporali RxJS con tempo virtuale. Notazione marble, Cold/Hot Observable, debounceTime, delay e implementazione TypeScript."
---

# Test con TestScheduler

Il `TestScheduler` di RxJS è uno strumento potente per testare accuratamente gli operatori basati sul tempo. Questo capitolo spiega sistematicamente come testare utilizzando TestScheduler.

## Cos'è TestScheduler?

Normalmente, gli Observable funzionano in base al tempo. Ad esempio, `delay()` e `debounceTime()` sono operatori che attendono un certo tempo.
Nei test, attendere effettivamente è inefficiente, quindi `TestScheduler` è un meccanismo che permette di testare immediatamente usando il tempo virtuale.

> [!TIP]
> Con TestScheduler si usa il "tempo virtuale", quindi non è necessario attendere il tempo reale.

## Struttura Base di TestScheduler

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
      const expected =  '--a--b--c|';

      expectObservable(source$).toBe(expected);
    });
  });
});
```

- `cold()`: Crea un Cold Observable che inizia lo stream in modo indipendente per ogni sottoscrizione
- `hot()`: Crea un Hot Observable con uno stream già in corso
- `expectObservable()`: Verifica l'output dell'Observable con la notazione marble

## Cold Observable e Hot Observable

|Tipo|Caratteristica|Utilizzo|
|:---|:---|:---|
|Cold Observable|Fa fluire i dati dall'inizio ogni volta che viene sottoscritto|Richieste HTTP, ecc.|
|Hot Observable|Il flusso di dati è già iniziato e viene condiviso con i sottoscrittori|Eventi utente, WebSocket, ecc.|

## Basi della Notazione Marble

La notazione marble è una tecnica per rappresentare il passare del tempo di un Observable con stringhe.

|Simbolo|Significato|
|:---|:---|
|`-`|Passare del tempo (1 frame)|
|`a`, `b`, `c`|Valori emessi|
|`|`|Completamento|
|`#`|Errore|
|`() `|Emissione simultanea di più valori (eventi multipli)|

#### Esempio

```
--a--b--c|    // Dopo 2 frame a, poi b, c, completamento
```

## Esempi di Test con Tempo Virtuale

### Test di debounceTime

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { debounceTime, map } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Test usando tempo virtuale', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test dell\'operatore debounceTime', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20),
        map(x => x.toUpperCase())
      );
      const expected =    '-----------(C|)';  // ←Questo!

      expectObservable(result$).toBe(expected, { B: 'B', C: 'C' });
    });
  });
});
```

## Test di Gestione degli Errori

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { catchError} from 'rxjs';
import { TestScheduler } from 'rxjs/testing';
import { of } from 'rxjs';

describe('Test di gestione degli errori', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('cattura errore con catchError', () => {
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

## Riepilogo

- Con TestScheduler è possibile testare senza attendere il tempo reale
- Comprendere e utilizzare correttamente la differenza tra Observable Cold/Hot
- Utilizzare la notazione marble per visualizzare il passare del tempo
- È possibile testare con precisione anche stream asincroni complessi

> [!NEXT]
> Successivamente, impareremo tecniche più avanzate di marble testing (personalizzazione delle stringhe marble e combinazione di più stream).
