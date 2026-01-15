---
description: "Descrive come gestire efficacemente il completamento dello stream e il rilascio delle risorse in RxJS usando finalize e complete. Include pattern pratici per prevenire le perdite di memoria, rilasciare gli handle dei file, pulire le connessioni WebSocket, ripristinare lo stato dell'interfaccia utente, ecc."
---

# finalize e complete - Cleanup risorse

In RxJS è importante gestire correttamente il completamento degli stream e il rilascio delle risorse. Questa pagina spiega come funzionano l'operatore `finalize` e le notifiche `complete`.

## finalize - Operatore per il rilascio delle risorse

L'operatore `finalize` è l'operatore che esegue il codice di pulizia specificato quando l'Observable termina con **completamento, errore o unsubscribe**.
Finalize viene chiamato **solo una volta** alla fine dello stream e mai più di una volta.

[🌐 Documentazione ufficiale RxJS - finalize](https://rxjs.dev/api/index/function/finalize)

### Utilizzo base di finalize

```ts
import { of } from 'rxjs';
import { finalize, tap } from 'rxjs';

// Variabile per gestire lo stato di caricamento
let isLoading = true;

// Stream di successo
of('dati')
  .pipe(
    tap((data) => console.log('Elaborazione dati:', data)),
    // Eseguito in caso di successo, fallimento o cancellazione
    finalize(() => {
      isLoading = false;
      console.log('Stato di caricamento resettato:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Valore:', value),
    complete: () => console.log('Completato'),
  });

// Output:
// Elaborazione dati: dati
// Valore: dati
// Completato
// Stato di caricamento resettato: false
```

### finalize in caso di errore

```ts
import { throwError } from 'rxjs';
import { finalize, catchError } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Errore nel recupero dati'))
  .pipe(
    catchError((err) => {
      console.error('Gestione errore:', err.message);
      throw err; // Rilancia l'errore
    }),
    finalize(() => {
      isLoading = false;
      console.log('Rilascio risorse dopo errore:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Valore:', value),
    error: (err) => console.error('Errore nel subscriber:', err.message),
    complete: () => console.log('Completato'), // Non chiamato in caso di errore
  });

// Output:
// Gestione errore: Errore nel recupero dati
// Errore nel subscriber: Errore nel recupero dati
// Rilascio risorse dopo errore: false
```

### finalize al momento dell'unsubscribe

```ts
import { interval } from 'rxjs';
import { finalize } from 'rxjs';

let resource = 'attiva';

// Conta ogni secondo
const subscription = interval(1000)
  .pipe(
    finalize(() => {
      resource = 'rilasciata';
      console.log('Stato risorsa:', resource);
    })
  )
  .subscribe((count) => {
    console.log('Conteggio:', count);

    // Unsubscribe manuale dopo 3 conteggi
    if (count >= 2) {
      subscription.unsubscribe();
    }
  });

// Output:
// Conteggio: 0
// Conteggio: 1
// Conteggio: 2
// Stato risorsa: rilasciata
```

Finalize è utile quando si vuole garantire l'elaborazione della pulizia non solo in caso di errore, ma anche in caso di completamento positivo o di unsubscribe manuale.

## complete - Notifica del completamento normale dello stream

Quando un Observable viene completato con successo, viene invocato il callback `complete` dell'Observer. Questo è l'ultimo passo del ciclo di vita dell'Observable.

### Completamento automatico

Alcuni Observable si completano automaticamente quando vengono soddisfatte determinate condizioni.

```ts
import { of, interval } from 'rxjs';
import { take } from 'rxjs';

// Una sequenza finita si completa automaticamente
of(1, 2, 3).subscribe({
  next: (value) => console.log('Valore:', value),
  complete: () => console.log('Stream finito completato'),
});

// Stream limitato con interval + take
interval(1000)
  .pipe(
    take(3) // Completato dopo 3 valori
  )
  .subscribe({
    next: (value) => console.log('Conteggio:', value),
    complete: () => console.log('Stream limitato completato'),
  });

// Output:
// Valore: 1
// Valore: 2
// Valore: 3
// Stream finito completato
// Conteggio: 0
// Conteggio: 1
// Conteggio: 2
// Stream limitato completato
```

### Completamento manuale

Con Subject o Observable personalizzati è possibile chiamare complete manualmente.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.subscribe({
  next: (value) => console.log('Valore:', value),
  complete: () => console.log('Subject completato'),
});

subject.next(1);
subject.next(2);
subject.complete(); // Completamento manuale
subject.next(3); // Ignorato dopo il completamento

// Output:
// Valore: 1
// Valore: 2
// Subject completato
```

## Differenza tra finalize e complete

È importante capire le differenze principali.

1. **Tempistica di esecuzione**
   - `complete`: viene chiamato solo quando un Observable è **completato con successo**
   - `finalize`: viene chiamato quando l'Observable termina con **completamento, errore o unsubscribe**

2. **Utilizzo**
   - `complete`: riceve la notifica del completamento con successo (elaborazione in caso di successo)
   - `finalize`: garantisce il rilascio o la pulizia delle risorse (elaborazione che deve essere eseguita indipendentemente dal successo o dal fallimento)

## Casi d'uso pratici

### Chiamate API e gestione dello stato di caricamento

```ts
import { ajax } from 'rxjs/ajax';
import { finalize, catchError } from 'rxjs';
import { of } from 'rxjs';

// Stato di caricamento
let isLoading = false;

function fetchData(id: string) {
  // Inizio caricamento
  isLoading = true;
  const loading = document.createElement('p');
  loading.style.display = 'block';
  document.body.appendChild(loading);
  // document.getElementById('loading')!.style.display = 'block';

  // Richiesta API
  return ajax.getJSON(`https://jsonplaceholder.typicode.com/posts/${id}`).pipe(
    catchError((error) => {
      console.error('Errore API:', error);
      return of({ error: true, message: 'Recupero dati fallito' });
    }),
    // Termina il caricamento indipendentemente dal successo o dal fallimento
    finalize(() => {
      isLoading = false;
      loading!.style.display = 'none';
      console.log('Reset stato di caricamento completato');
    })
  );
}

// Esempio di utilizzo
fetchData('123').subscribe({
  next: (data) => console.log('Dati:', data),
  complete: () => console.log('Recupero dati completato'),
});

// Output:
// Errore API: AjaxErrorImpl {message: 'ajax error', name: 'AjaxError', xhr: XMLHttpRequest, request: {...}, status: 0, ...}
// Dati: {error: true, message: 'Recupero dati fallito'}
// Recupero dati completato
// Reset stato di caricamento completato
// GET https://jsonplaceholder.typicode.com/posts/123 net::ERR_NAME_NOT_RESOLVED
```

### Pulizia delle risorse

```ts
import { interval } from 'rxjs';
import { finalize, takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

class ResourceManager {
  private destroy$ = new Subject<void>();
  private timerId: number | null = null;

  constructor() {
    // Inizializzazione di una risorsa
    this.timerId = window.setTimeout(() => console.log('Timer eseguito'), 10000);

    // Elaborazione periodica
    interval(1000)
      .pipe(
        // Si ferma quando il componente viene distrutto
        takeUntil(this.destroy$),
        // Rilascia le risorse in modo affidabile
        finalize(() => {
          console.log('Intervallo fermato');
        })
      )
      .subscribe((count) => {
        console.log('In esecuzione...', count);
      });
  }

  dispose() {
    // Processo di distruzione
    if (this.timerId) {
      window.clearTimeout(this.timerId);
      this.timerId = null;
    }

    // Segnale di arresto dello stream
    this.destroy$.next();
    this.destroy$.complete();

    console.log('Distruzione ResourceManager completata');
  }
}

// Esempio di utilizzo
const manager = new ResourceManager();

// Distrugge dopo 5 secondi
setTimeout(() => {
  manager.dispose();
}, 5000);

// Output:
// In esecuzione... 0
// In esecuzione... 1
// In esecuzione... 2
// In esecuzione... 3
// In esecuzione... 4
// Intervallo fermato
// Distruzione ResourceManager completata
```

[📘 RxJS ufficiale: takeUntil()](https://rxjs.dev/api/index/function/takeUntil)

## Best Practice

1. **Rilasciare sempre le risorse**: usare `finalize` per garantire la pulizia quando lo stream termina
2. **Gestione dello stato di caricamento**: resettare sempre lo stato di caricamento usando `finalize`
3. **Gestione del ciclo di vita dei componenti**: usare una combinazione di `takeUntil` e `finalize` per pulire le risorse quando i componenti vengono distrutti (questo pattern è particolarmente raccomandato in Angular)
4. **Uso con la gestione degli errori**: combinare `catchError` e `finalize` per fornire una gestione di fallback e una pulizia affidabile dopo gli errori
5. **Conoscere lo stato di completamento**: utilizzare il callback `complete` per determinare se lo stream è stato completato con successo

## Riepilogo

`finalize` e `complete` sono strumenti importanti per la gestione delle risorse e il completamento dell'elaborazione in RxJS. `finalize` è ideale per il rilascio delle risorse, in quanto garantisce l'esecuzione indipendentemente da come lo stream termina. D'altra parte, `complete` è usato quando si vuole eseguire elaborazioni al completamento normale. Combinandoli in modo appropriato, si possono prevenire le perdite di memoria e costruire applicazioni affidabili.
