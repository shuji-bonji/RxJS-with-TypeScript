---
description: "Observable è un concetto fondamentale in RxJS che rappresenta un flusso di dati che si verifica nel tempo. Spieghiamo in dettaglio le differenze con Promise, i meccanismi di sottoscrizione (subscribe) e cancellazione (unsubscribe), Cold e Hot Observable, e le definizioni di tipo in TypeScript."
---

# Cos'è Observable

[📘 RxJS Ufficiale: Observable](https://rxjs.dev/api/index/class/Observable)

Observable in RxJS è un costrutto fondamentale che rappresenta "un flusso di dati (stream) che si verifica nel tempo". È progettato sulla base del pattern Observer e può gestire l'elaborazione asincrona e guidata dagli eventi in modo unificato.

## Ruolo di Observable

Un Observable agisce come "produttore di dati" che emette valori multipli nel tempo. Al contrario, un Observer agisce come "consumatore" e sottoscrive i valori tramite `subscribe()`.

Nell'esempio seguente, viene creato un **Observable (produttore)** chiamato `observable$` e l'**Observer (consumatore)** sottoscrive e riceve i valori.

```ts
import { Observable } from 'rxjs';

// Crea un Observable (produttore)
const observable$ = new Observable<number>(subscriber => {
  // Logica eseguita alla sottoscrizione
  subscriber.next(1);
  subscriber.next(2);
  subscriber.complete();
});

// L'Observer (consumatore) sottoscrive
observable$.subscribe({
  next: value => console.log('Valore successivo:', value),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});

// Output:
// Valore successivo: 1
// Valore successivo: 2
// Completato
```

> [!NOTE]
> La funzione passata come argomento a `new Observable(funzione)` definisce la **logica da eseguire quando l'Observable viene sottoscritto**. La funzione stessa non è il produttore, ma l'Observable nel suo complesso.

## Tipi di notifica

Observable invia i seguenti tre tipi di notifiche all'Observer:

- `next`: Notifica di un valore
- `error`: Notifica di un errore (non vengono inviate altre notifiche)
- `complete`: Notifica di completamento con successo

Per dettagli, vedere la sezione Observer in ["Ciclo di vita di Observable"](./observable-lifecycle.md#_2-observer-オブザーバー).

## Differenze tra Observable e Promise

| Caratteristica | Observable | Promise |
|---|---|---|
| Valori multipli | ◯ | × (solo uno) |
| Cancellabile | ◯ (`unsubscribe()`) | × |
| Esecuzione ritardata | ◯ | ◯ |
| Sincrono/Asincrono | Entrambi | Solo asincrono |

La differenza principale tra Observable e Promise è "la possibilità di gestire valori multipli" e "la possibilità di cancellare a metà".
Promise è adatto per elaborazioni asincrone una tantum, mentre Observable eccelle con "dati asincroni che si verificano continuamente" come i flussi di eventi.

Observable è importante anche per la gestione delle risorse, come la prevenzione di memory leak e l'interruzione di comunicazioni non necessarie, poiché le sottoscrizioni possono essere cancellate a metà con `unsubscribe()`.

D'altra parte, Promise è ampiamente adottato nelle API standard e può essere scritto in modo intuitivo in combinazione con `async/await`. È consigliabile usarli in base all'applicazione.

## Distinzione tra Cold e Hot

In RxJS esistono due tipi di Observable: "cold" e "hot".

- **Cold Observable**: Ogni sottoscrittore ha il proprio flusso di dati, che inizia l'esecuzione quando viene sottoscritto. (es: `of()`, `from()`, `fromEvent()`, `ajax()`)
- **Hot Observable**: I sottoscrittori condividono lo stesso flusso di dati, e i dati continuano a fluire indipendentemente dalla sottoscrizione. (es: `Subject`, Observable multicast con `share()`)

Questa distinzione ha un impatto significativo sulla condivisione dei dati e sull'efficienza delle risorse.
Per dettagli, vedere la sezione ["Cold Observable e Hot Observable"](./cold-and-hot-observables.md).

## Observable e Pipeline

Il vero valore di Observable si realizza quando viene combinato con gli operatori usando il metodo `pipe()`.

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5);
numbers$.pipe(
  filter(n => n % 2 === 0), // Passa solo i numeri pari
  map(n => n * 10)          // Moltiplica per 10
).subscribe(value => console.log(value));
// Output: 20, 40
```

## Ciclo di vita di Observable

Un Observable ha il seguente ciclo di vita:

1. **Creazione** - Generazione dell'istanza Observable
2. **Sottoscrizione** - Inizio della ricezione dati con `subscribe()`
3. **Esecuzione** - Emissione dati (`next`), errore (`error`), o completamento (`complete`)
4. **Cancellazione** - Fine della sottoscrizione con `unsubscribe()`

Per prevenire perdite di risorse, è importante cancellare le sottoscrizioni di Observable che non sono più necessari.
Per dettagli, vedere la sezione ["Ciclo di vita di Observable"](./observable-lifecycle.md).

## Quando usare Observable

- Eventi UI (click, scroll, azioni tastiera, ecc.)
- Richieste HTTP
- Elaborazione basata sul tempo (intervalli e timer)
- WebSocket e comunicazione in tempo reale
- Gestione dello stato dell'applicazione

## Riepilogo

Observable è la base per gestire i dati asincroni in modo flessibile e unificato. Come concetto centrale di ReactiveX (RxJS), permette di esprimere in modo conciso elaborazioni asincrone complesse e flussi di eventi.
