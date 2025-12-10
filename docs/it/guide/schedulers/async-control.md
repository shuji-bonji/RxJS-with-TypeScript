---
description: Spiega come utilizzare gli scheduler in RxJS e come controllare l'elaborazione asincrona attraverso observeOn e subscribeOn. Introduce tecniche pratiche per l'ottimizzazione delle prestazioni e l'evitamento del blocco dell'UI, come il controllo del timing di esecuzione, la gestione del contesto di esecuzione e la prioritizzazione dei task, con esempi di codice TypeScript.
---
# Controllo dell'Elaborazione Asincrona

Gli scheduler in RxJS sono un meccanismo importante per controllare il timing e il contesto di esecuzione dell'elaborazione asincrona. Questo capitolo spiega come controllare l'elaborazione asincrona utilizzando gli scheduler.

## Ruolo degli Scheduler

Gli scheduler svolgono tre ruoli importanti:

|Ruolo|Descrizione|
|---|---|
|Controllo del timing di esecuzione|Determina quando eseguire i task|
|Gestione del contesto di esecuzione|Determina in quale thread o ambiente di esecuzione far funzionare i task|
|Prioritizzazione dei task|Gestisce l'ordine di esecuzione di più task|

## Comprensione dell'Elaborazione Sincrona e Asincrona

### Comportamento Predefinito (Esecuzione Sincrona)

Gli operatori di RxJS vengono eseguiti in modo sincrono il più possibile per impostazione predefinita.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

console.log('Inizio esecuzione');

of(1, 2, 3)
  .pipe(
    map((x) => {
      console.log(`map: ${x}`);
      return x * 2;
    })
  )
  .subscribe((x) => console.log(`subscribe: ${x}`));

console.log('Fine esecuzione');

// Output:
// Inizio esecuzione
// map: 1
// subscribe: 2
// map: 2
// subscribe: 4
// map: 3
// subscribe: 6
// Fine esecuzione
```

### Asincronia Tramite Scheduler

Utilizzando uno scheduler, è possibile rendere asincrona l'elaborazione.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Inizio esecuzione');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(x => console.log(`subscribe: ${x}`));

console.log('Fine esecuzione');

// Output:
// Inizio esecuzione
// Fine esecuzione
// subscribe: 1
// subscribe: 2
// subscribe: 3
```

## Operatori Che Utilizzano gli Scheduler

### Operatore observeOn

`observeOn` modifica il contesto di esecuzione dello stream. Emette valori con lo scheduler specificato.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { take, observeOn } from 'rxjs';

// Esempio di utilizzo per animazioni
interval(16)
  .pipe(
    take(10),
    observeOn(animationFrameScheduler)
  )
  .subscribe(() => {
    // Esegue sincronizzandosi con i frame di animazione
    updateAnimation();
  });

function updateAnimation() {
  // Elaborazione aggiornamento animazione
}
```

> [!TIP]
> Per spiegazioni dettagliate, esempi pratici e note sull'operatore `observeOn`, consulta la pagina dell'operatore [observeOn](../operators/utility/observeOn.md).

### Operatore subscribeOn

`subscribeOn` controlla il timing di inizio della sottoscrizione dello stream.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, tap } from 'rxjs';

console.log('Prima dell\'inizio sottoscrizione');

of('Esecuzione task')
  .pipe(
    tap(() => console.log('Inizio task')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(value => console.log(value));

console.log('Dopo l\'inizio sottoscrizione');

// Output:
// Prima dell'inizio sottoscrizione
// Dopo l'inizio sottoscrizione
// Inizio task
// Esecuzione task
```

> [!TIP]
> Per spiegazioni dettagliate, esempi pratici e differenze con `observeOn` dell'operatore `subscribeOn`, consulta la pagina dell'operatore [subscribeOn](../operators/utility/subscribeOn.md).

## Esempi Pratici di Elaborazione Asincrona

### Controllo delle Richieste API

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

// Mette le richieste in coda ed elabora in ordine
from(requests)
  .pipe(
    observeOn(queueScheduler),
    tap((req) => console.log(`Aggiunto alla coda: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // Simulazione richiesta API reale
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`Risultato di ${req.endpoint}/${req.id}`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`Completato: ${result}`));

// Output:
// Aggiunto alla coda: /users
// Aggiunto alla coda: /posts
// Aggiunto alla coda: /comments
// Completato: Risultato di /users/1
// Completato: Risultato di /posts/1
// Completato: Risultato di /comments/1
```

### Evitare il Blocco del Thread UI

Quando si elaborano grandi quantità di dati, utilizza gli scheduler per evitare di bloccare il thread UI.

```ts
import { from, asapScheduler } from 'rxjs';
import { observeOn, bufferCount } from 'rxjs';

const largeDataSet = Array.from({ length: 10000 }, (_, i) => i);

// Dimensione batch
const batchSize = 100;
// Calcola numero totale di batch
const totalBatches = Math.ceil(largeDataSet.length / batchSize);
// Contatore batch
let batchIndex = 0;

from(largeDataSet)
  .pipe(
    bufferCount(100), // Raggruppa per 100 elementi
    observeOn(asapScheduler) // Il prima possibile, ma senza bloccare l'UI
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
  // Elaborazione dati batch
  const processed = batch.map((n) => n * 2);
  console.log(
    `Batch ${batchIndex} di ${totalBatches} completato: elaborati ${processed.length} elementi.`
  );
}

// Output:
// Batch 1 di 100 completato: elaborati 100 elementi.
// Batch 2 di 100 completato: elaborati 100 elementi.
// ...
// ...
// Batch 100 di 100 completato: elaborati 100 elementi.
```

## Ottimizzazione delle Prestazioni e Debug

### Test Utilizzando gli Scheduler

```ts
import { TestScheduler } from 'rxjs/testing';
import { delay } from 'rxjs';
import { beforeEach, describe, expect, it } from 'vitest';

describe('Test elaborazione asincrona', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Test operatore delay', () => {
    scheduler.run(({ cold, expectObservable }) => {
      const source = cold('a-b-c|');
      const expected =    '1000ms a-b-(c|)';

      const result = source.pipe(delay(1000, scheduler));

      expectObservable(result).toBe(expected);
    });
  });
});
```

### Output di Log per Debug

```ts
import { of, asyncScheduler } from 'rxjs';
import { tap, observeOn } from 'rxjs';

console.log('Inizio');

of(1, 2, 3)
  .pipe(
    tap(value => console.log(`[Prima scheduler・sincrono] Valore: ${value}`)),
    observeOn(asyncScheduler),  // Utilizza asyncScheduler
    tap(value => console.log(`[Dopo scheduler・asincrono] Valore: ${value}`))
  )
  .subscribe();

console.log('Fine');

// Output effettivo:
// Inizio
// [Prima scheduler・sincrono] Valore: 1
// [Prima scheduler・sincrono] Valore: 2
// [Prima scheduler・sincrono] Valore: 3
// Fine
// [Dopo scheduler・asincrono] Valore: 1
// [Dopo scheduler・asincrono] Valore: 2
// [Dopo scheduler・asincrono] Valore: 3
```

Utilizzando `asyncScheduler`, è possibile confermare il comportamento asincrono atteso. Mentre `queueScheduler` utilizza la coda dei micro task e quindi viene elaborato durante l'esecuzione del codice sincrono, `asyncScheduler` utilizza setTimeout internamente e quindi viene eseguito in modo completamente asincrono.

## Esempio che Mostra le Differenze di Comportamento degli Scheduler
Questo esempio mostra le differenze nel timing di esecuzione dei diversi scheduler.

```ts
import { of, queueScheduler, asyncScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inizio');

// Elaborazione sincrona
of('sync').subscribe(value => console.log(`2: ${value}`));

// queueScheduler (micro task)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`3: ${value}`));

// asapScheduler (micro task)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`4: ${value}`));

// asyncScheduler (macro task)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`5: ${value}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Fine');

// Ordine di output effettivo:
// 1: Inizio
// 2: sync
// 7: Fine
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```


## Best Practice

1. **Utilizza gli scheduler solo quando necessario**: Se il comportamento sincrono predefinito è sufficiente, non forzare l'uso degli scheduler

2. **Scegli lo scheduler appropriato**: Seleziona lo scheduler ottimale in base allo scopo
   - Animazioni: `animationFrameScheduler`
   - Evitare blocco UI: `asapScheduler`
   - Elaborazione code: `queueScheduler`
   - Elaborazione asincrona: `asyncScheduler`

3. **Monitora le prestazioni**: Controlla sempre l'impatto sulle prestazioni derivante dall'uso degli scheduler

4. **Garantisci la testabilità**: Scrivi test per l'elaborazione asincrona utilizzando `TestScheduler`

## Errori Comuni e Contromisure

### Asincronia Eccessiva

```ts
// ❌ Asincronia non necessaria
of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Asincronia duplicata
    filter(x => x > 3)
  )
  .subscribe();

// ✅ Rendi asincrono solo dove necessario
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    filter(x => x > 3),
    observeOn(asyncScheduler)  // Rendi asincrono tutto insieme alla fine
  )
  .subscribe();
```

### Uso Improprio degli Scheduler

```ts
// ❌ Uso errato
interval(1000)
  .pipe(
    subscribeOn(animationFrameScheduler)  // Non influisce su interval
  )
  .subscribe();

// ✅ Uso corretto
interval(1000, animationFrameScheduler)  // Specifica lo scheduler alla creazione
  .subscribe();
```

## Riepilogo

Gli scheduler sono strumenti potenti per controllare finemente l'elaborazione asincrona in RxJS. Utilizzandoli appropriatamente, è possibile realizzare l'ottimizzazione delle prestazioni, l'evitamento del blocco del thread UI e la facilitazione dei test. Tuttavia, poiché un'asincronia eccessiva può invece peggiorare le prestazioni, è importante utilizzarli solo quando necessario.

La prossima sezione spiegherà in dettaglio i tipi specifici di scheduler e come utilizzarli.
