---
description: Questa sezione descrive scheduled e using, che sono Funzioni di Creazione di controllo RxJS. scheduled controlla il timing di esecuzione dell'Observable specificando uno scheduler, e using gestisce automaticamente le risorse come WebSocket e file handle secondo il ciclo di vita dell'Observable. Può essere usato anche per testing e ottimizzazione delle performance.
---

# Funzioni di Creazione di Controllo

RxJS fornisce Funzioni di Creazione per controllare in dettaglio il timing di esecuzione e la gestione delle risorse dell'Observable. Questa sezione descrive due funzioni, `scheduled()` e `using()`, in dettaglio.

## Cosa Sono le Funzioni di Creazione di Controllo?

Le Funzioni di Creazione di Controllo sono un insieme di funzioni per un controllo più fine del comportamento dell'Observable. Supportano casi d'uso avanzati come il controllo del timing di esecuzione (scheduler) e la gestione del ciclo di vita delle risorse.

### Caratteristiche Principali

- **Controllo del timing di esecuzione**: Usa lo scheduler per passare tra esecuzione sincrona e asincrona
- **Gestione delle risorse**: Rilascio automatico delle risorse secondo il ciclo di vita dell'Observable
- **Facilità di testing**: Cambia tra scheduler per facilitare il testing
- **Ottimizzazione delle performance**: Controlla il timing di esecuzione per evitare il blocco della UI

## Lista delle Funzioni di Creazione di Controllo

| Funzione | Descrizione | Usi Principali |
|------|------|---------|
| [scheduled()](/it/guide/creation-functions/control/scheduled) | Genera Observable con scheduler specificato | Controllo timing esecuzione, testing |
| [using()](/it/guide/creation-functions/control/using) | Observable con controllo delle risorse | Gestione risorse per WebSocket, file handle, ecc. |

## Nozioni Base di scheduled()

`scheduled()` è una funzione che ti permette di specificare esplicitamente uno scheduler quando generi un Observable da una sorgente dati esistente (array, Promise, Iterable, ecc.).

### Uso Base

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Emetti array asincronamente
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Inizio subscription');
observable$.subscribe({
  next: val => console.log('Valore:', val),
  complete: () => console.log('Completo')
});
console.log('Fine subscription');

// Output:
// Inizio subscription
// Fine subscription
// Valore: 1
// Valore: 2
// Valore: 3
// Completo
```

> [!NOTE]
> Con `asyncScheduler`, l'emissione dei valori diventa asincrona. Questo permette al processo di subscription di eseguire senza bloccare il thread principale.

## Nozioni Base di using()

`using()` è una funzione che crea e rilascia automaticamente le risorse secondo il ciclo di vita dell'Observable. Crea una risorsa all'inizio di una subscription e la rilascia automaticamente quando la subscription termina (`complete` o `unsubscribe`).

### Uso Base

```typescript
import { using, interval, Subscription, take } from 'rxjs';

const resource$ = using(
  // Factory delle risorse: eseguita all'inizio della subscription
  () => {
    console.log('Risorsa creata');
    return new Subscription(() => console.log('Risorsa rilasciata'));
  },
  // Factory dell'Observable: crea Observable usando la risorsa
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Valore:', value),
  complete: () => console.log('Completo')
});

// Output:
// Risorsa creata
// Valore: 0
// Valore: 1
// Valore: 2
// Completo
// Risorsa rilasciata
```

> [!IMPORTANT]
> `using()` rilascia automaticamente le risorse alla fine di una subscription, prevenendo così le perdite di memoria.

## Confronto: scheduled() vs using()

| Caratteristica | scheduled() | using() |
|------|-------------|---------|
| Scopo Principale | Controllo timing esecuzione | Gestione ciclo di vita risorse |
| Scheduler | ✅ Può specificare esplicitamente | ❌ Non può specificare |
| Gestione Risorse | ❌ Gestione manuale richiesta | ✅ Gestione automatica |
| Casi d'Uso | Testing, ottimizzazione UI | WebSocket, file handle |
| Complessità | Semplice | Un po' complessa |

## Linee Guida all'Uso

### Quando Scegliere scheduled()

1. **Vuoi controllare il timing di esecuzione**
   - Vuoi cambiare elaborazione sincrona in asincrona
   - Vuoi evitare il blocco della UI

2. **Serve controllo del tempo per il testing**
   - Combina con TestScheduler per controllare il tempo
   - Vuoi testare elaborazione asincrona sincronamente

3. **Converti sorgenti dati esistenti in Observable**
   - Converti Array, Promise, Iterable in Observable
   - Vuoi specificare esplicitamente uno scheduler

### Quando Scegliere using()

1. **È richiesto rilascio automatico delle risorse**
   - Gestione connessioni WebSocket
   - Gestione file handle
   - Pulizia automatica dei timer

2. **Vuoi prevenire perdite di memoria**
   - Prevenire dimenticanza nel rilascio risorse
   - Pulizia affidabile alla fine della subscription

3. **Gestione risorse complessa**
   - Gestire più risorse contemporaneamente
   - Gestire dipendenze tra risorse

## Esempi di Utilizzo Pratico

### Esempio di Utilizzo di scheduled()

```typescript
import { scheduled, asyncScheduler, queueScheduler } from 'rxjs';

// Elabora grandi quantità di dati asincronamente (non blocca UI)
const largeArray = Array.from({ length: 10000 }, (_, i) => i);
const async$ = scheduled(largeArray, asyncScheduler);

async$.subscribe(value => {
  // Esegui elaborazione pesante qui
  // La UI non viene bloccata
});

// Esegui sincronamente nei test
const sync$ = scheduled(largeArray, queueScheduler);
```

### Esempio di Utilizzo di using()

```typescript
import { using, timer } from 'rxjs';

// Gestisci automaticamente la connessione WebSocket
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    console.log('Connessione WebSocket iniziata');
    return {
      unsubscribe: () => {
        ws.close();
        console.log('Connessione WebSocket terminata');
      }
    };
  },
  () => timer(0, 1000) // Ricevi messaggi ogni 1 secondo
);
```

## Tipi di Scheduler (per scheduled())

| Scheduler | Descrizione | Casi d'Uso |
|---------------|------|---------|
| `queueScheduler` | Esecuzione sincrona (metodo coda) | Default, elaborazione sincrona |
| `asyncScheduler` | Esecuzione asincrona (setTimeout) | Ottimizzazione UI, elaborazione lunga |
| `asapScheduler` | Esecuzione asincrona più veloce (Promise) | Elaborazione asincrona ad alta priorità |
| `animationFrameScheduler` | Frame di animazione | Animazione, rendering UI |

> [!TIP]
> Per maggiori informazioni sugli scheduler, vedi [Tipi di Scheduler](/it/guide/schedulers/types).

## Domande Frequenti

### D1: Qual è la differenza tra scheduled() e from()?

**R:** `from()` usa internamente lo scheduler di default (sincrono). `scheduled()` permette di specificare esplicitamente lo scheduler, consentendo così un controllo fine del timing di esecuzione.

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - esegui sincronamente
const sync$ = from([1, 2, 3]);

// scheduled() - esegui asincronamente
const async$ = scheduled([1, 2, 3], asyncScheduler);
```

### D2: Quando dovrei usare using()?

**R:** Usa quando vuoi evitare di dimenticare il rilascio delle risorse. È particolarmente utile nei seguenti casi:
- Connessioni di rete come WebSocket, EventSource, ecc.
- File handle, connessioni database
- Processi che richiedono manualmente `clearInterval()` o `clearTimeout()`

### D3: Perché scheduled() è più facile da testare?

**R:** TestScheduler ti permette di controllare virtualmente il passare del tempo. I processi asincroni possono essere testati sincronamente, riducendo notevolmente il tempo di esecuzione dei test.

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled, asyncScheduler } from 'rxjs';

const testScheduler = new TestScheduler((actual, expected) => {
  expect(actual).toEqual(expected);
});

testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

## Best Practice

### 1. Evita il Blocco della UI con scheduled()

```typescript
// ❌ Cattivo esempio: Elabora grandi quantità di dati sincronamente
from(largeArray).subscribe(processHeavyTask);

// ✅ Buon esempio: Elaborazione asincrona con asyncScheduler
scheduled(largeArray, asyncScheduler).subscribe(processHeavyTask);
```

### 2. Assicura il Rilascio delle Risorse con using()

```typescript
// ❌ Cattivo esempio: Gestione manuale delle risorse
const ws = new WebSocket('wss://example.com');
const source$ = interval(1000);
source$.subscribe(() => ws.send('ping'));
// Perdita di risorse se si dimentica unsubscribe

// ✅ Buon esempio: Gestione automatica con using()
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return { unsubscribe: () => ws.close() };
  },
  () => interval(1000).pipe(tap(() => ws.send('ping')))
);
```

### 3. Usa lo Scheduler Appropriato per il Testing

```typescript
// ✅ Buon esempio: TestScheduler per il testing
const testScheduler = new TestScheduler(...);
const source$ = scheduled([1, 2, 3], testScheduler);

// ✅ Buon esempio: asyncScheduler per la produzione
const source$ = scheduled([1, 2, 3], asyncScheduler);
```

## Riepilogo

Le Funzioni di Creazione di Controllo sono funzioni avanzate per la messa a punto fine del comportamento dell'Observable.

**scheduled():**
- Controlla esplicitamente il timing di esecuzione (sincrono/asincrono)
- Utile per il controllo del tempo nel testing
- Efficace per evitare il blocco della UI

**using():**
- Gestione automatica del ciclo di vita delle risorse
- Previene le perdite di memoria
- Ideale per gestire connessioni come WebSocket

Usate appropriatamente, puoi costruire applicazioni RxJS più robuste e performanti.

## Prossimi Passi

Per l'utilizzo dettagliato di ogni funzione, consulta le seguenti pagine:

- [scheduled() in dettaglio](/it/guide/creation-functions/control/scheduled) - Genera Observable con scheduler
- [using() in dettaglio](/it/guide/creation-functions/control/using) - Observable con controllo delle risorse

## Risorse di Riferimento

- [Documentazione Ufficiale RxJS - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [Documentazione Ufficiale RxJS - using](https://rxjs.dev/api/index/function/using)
- [Documentazione Ufficiale RxJS - Scheduler](https://rxjs.dev/guide/scheduler)
- [Tipi di Scheduler](/it/guide/schedulers/types)
