---
description: "Spiega in dettaglio come funziona il multicasting di RxJS, compresi i pattern di base che prevedono l'uso di Subject, l'uso degli operatori share e shareReplay, l'evitare la duplicazione delle richieste API, le strategie di caching e la condivisione dello stato dell'applicazione. Verranno presentati pattern di progettazione pratici con esempi di codice TypeScript."
---

# Come funziona il multicasting

Il multicasting è un metodo per distribuire in modo efficiente un flusso di dati da un singolo Observable a più sottoscrittori (Observer).
In RxJS, questo può essere ottenuto tramite Subject e operatori.

## Cos'è il multicasting

Un normale Observable (Cold Observable) crea un nuovo flusso di dati ogni volta che viene sottoscritto. Ciò significa che se ci sono più sottoscrittori, lo stesso processo viene eseguito più volte.

Il multicasting consente di eseguire una fonte dati una sola volta e di distribuire i risultati a più sottoscrittori. Questo è particolarmente importante quando:

- Non si vogliono invocare richieste HTTP/API duplicate
- Si vuole eseguire un'operazione costosa (calcolo o effetto collaterale) una sola volta
- Si vuole condividere lo stato dell'applicazione con più componenti

## Pattern di multicasting di base

### Multicasting di base con Subject

```ts
import { Observable, Subject } from 'rxjs';
import { tap } from 'rxjs';

// Fonte dati (Cold Observable)
function createDataSource(): Observable<number> {
  return new Observable<number>(observer => {
    console.log('Fonte dati: connessione');
    // Logica di generazione dati (operazione ad alto costo)
    const id = setInterval(() => {
      const value = Math.round(Math.random() * 100);
      console.log(`Fonte dati: genera valore -> ${value}`);
      observer.next(value);
    }, 1000);

    // Funzione di cleanup
    return () => {
      console.log('Fonte dati: disconnessa');
      clearInterval(id);
    };
  });
}

// Implementazione multicast
function multicast() {
  // Fonte dati originale
  const source$ = createDataSource().pipe(
    tap(value => console.log(`Elaborazione sorgente: ${value}`))
  );

  // Subject per il multicast
  const subject = new Subject<number>();

  // Collegare la sorgente al Subject
  const subscription = source$.subscribe(subject);

  // Più sottoscrittori sottoscrivono il Subject
  console.log('Observer 1 sottoscrizione avviata');
  const subscription1 = subject.subscribe(value => console.log(`Observer 1: ${value}`));

  // Aggiungere un altro sottoscrittore dopo 3 secondi
  setTimeout(() => {
    console.log('Observer 2 sottoscrizione avviata');
    const subscription2 = subject.subscribe(value => console.log(`Observer 2: ${value}`));

    // Terminare tutte le sottoscrizioni dopo 5 secondi
    setTimeout(() => {
      console.log('Termina tutte le sottoscrizioni');
      subscription.unsubscribe();
      subscription1.unsubscribe();
      subscription2.unsubscribe();
    }, 5000);
  }, 3000);
}

// Esecuzione
multicast();
```

#### Risultato dell'esecuzione
```
Fonte dati: connessione
Observer 1 sottoscrizione avviata
Fonte dati: genera valore -> 71
Elaborazione sorgente: 71
Observer 1: 71
Fonte dati: genera valore -> 79
Elaborazione sorgente: 79
Observer 1: 79
Fonte dati: genera valore -> 63
Elaborazione sorgente: 63
Observer 1: 63
Observer 2 sottoscrizione avviata
Fonte dati: genera valore -> 49
Elaborazione sorgente: 49
Observer 1: 49
Observer 2: 49
Fonte dati: genera valore -> 94
Elaborazione sorgente: 94
Observer 1: 94
Observer 2: 94
Fonte dati: genera valore -> 89
Elaborazione sorgente: 89
Observer 1: 89
Observer 2: 89
Fonte dati: genera valore -> 10
Elaborazione sorgente: 10
Observer 1: 10
Observer 2: 10
Fonte dati: genera valore -> 68
Elaborazione sorgente: 68
Observer 1: 68
Observer 2: 68
Termina tutte le sottoscrizioni
Fonte dati: disconnessa
```

## Operatori multicast

RxJS fornisce operatori dedicati per implementare il multicasting.

### Operatore `share()`
[📘 RxJS Official: share()](https://rxjs.dev/api/index/function/share)

L'operatore più semplice per implementare il multicast.
Internamente è una combinazione di `multicast()` e `refCount()`.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

// Observable per contare con intervallo
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Sorgente: ${value}`)),
  share() // abilita il multicast
);

// Primo sottoscrittore
console.log('Observer 1 sottoscrizione avviata');
const subscription1 = source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Aggiungere il secondo sottoscrittore dopo 2,5 secondi
setTimeout(() => {
  console.log('Observer 2 sottoscrizione avviata');
  const subscription2 = source$.subscribe(value => console.log(`Observer 2: ${value}`));

  // Annullare la sottoscrizione dell'Observer 1 dopo 5 secondi
  setTimeout(() => {
    console.log('Observer 1 disiscrizione');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

#### Risultato dell'esecuzione
```
Observer 1 sottoscrizione avviata
Sorgente: 0
Observer 1: 0
Observer 2 sottoscrizione avviata
Sorgente: 1
Observer 1: 1
Observer 2: 1
Sorgente: 2
Observer 1: 2
Observer 2: 2
Sorgente: 3
Observer 1: 3
Observer 2: 3
Observer 1 disiscrizione
Sorgente: 4
Observer 2: 4
```

### Controllo dettagliato di `share()`

A partire da RxJS 7, si possono passare opzioni a `share()` per controllarne il comportamento in modo più esplicito, invece di usare `refCount()`.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Sorgente: ${value}`)),
  share({
    resetOnError: true,
    resetOnComplete: true,
    resetOnRefCountZero: true,
  })
);

// Primo sottoscrittore
console.log('Observer 1 sottoscrizione avviata');
const subscription1 = source$.subscribe((value) =>
  console.log(`Observer 1: ${value}`)
);

// Aggiungere il secondo sottoscrittore dopo 2,5 secondi
setTimeout(() => {
  console.log('Observer 2 sottoscrizione avviata');
  const subscription2 = source$.subscribe((value) =>
    console.log(`Observer 2: ${value}`)
  );

  setTimeout(() => {
    console.log('Observer 1 disiscrizione');
    subscription1.unsubscribe();
  }, 1500);
}, 2500);
```

#### Risultato dell'esecuzione
```
Observer 1 sottoscrizione avviata
Sorgente: 0
Observer 1: 0
Sorgente: 1
Observer 1: 1
Observer 2 sottoscrizione avviata
Sorgente: 2
Observer 1: 2
Observer 2: 2
Sorgente: 3
Observer 1: 3
Observer 2: 3
Observer 1 disiscrizione
Sorgente: 4
Observer 2: 4
Sorgente: 5
Observer 2: 5
```

In questo modo, è possibile controllare chiaramente il comportamento dello stream quando termina o i sottoscrittori raggiungono lo zero.

### Operatore `shareReplay()`

[📘 RxJS Official: shareReplay()](https://rxjs.dev/api/index/function/shareReplay)

Simile a `share()`, ma ricorda un numero specificato di valori precedenti e li rende disponibili ai sottoscrittori che si aggiungono successivamente.

```ts
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// Utilizzare shareReplay (dimensione del buffer 2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Sorgente: ${value}`)),
  shareReplay(2) // bufferizza gli ultimi due valori
);

// Primo sottoscrittore
console.log('Observer 1 sottoscrizione avviata');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Aggiungere il secondo sottoscrittore dopo 3,5 secondi
setTimeout(() => {
  console.log('Observer 2 sottoscrizione avviata - riceve gli ultimi due valori');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

#### Risultato dell'esecuzione
```
Observer 1 sottoscrizione avviata
Sorgente: 0
Observer 1: 0
Sorgente: 1
Observer 1: 1
Observer 2 sottoscrizione avviata - riceve gli ultimi due valori
Observer 2: 0
Observer 2: 1
Sorgente: 2
Observer 1: 2
Observer 2: 2
Sorgente: 3
Observer 1: 3
Observer 2: 3
Sorgente: 4
Observer 1: 4
Observer 2: 4
```

## Tempistica e ciclo di vita nel multicasting

È importante capire il ciclo di vita di un flusso multicast. In particolare, quando si usa l'operatore `share()`, occorre tenere presente il seguente comportamento:

1. Primo sottoscrittore: `share()` avvia una connessione all'Observable di origine nel momento in cui viene effettuata la prima sottoscrizione.
2. Tutti i sottoscrittori vengono cancellati: se `share({ resetOnRefCountZero: true })` è impostato, la connessione alla sorgente viene cancellata quando il numero di sottoscrittori raggiunge lo zero.
3. Completamento o errore: per impostazione predefinita, `share()` ripristina lo stato interno al completamento o all'errore (se resetOnComplete/resetOnError è true).
4. Nuova sottoscrizione: se lo stream viene resettato e poi sottoscritto di nuovo, verrà ricostruito come un nuovo Observable.

Pertanto, le opzioni di `share()` controllano quando lo stream si avvia, si ferma e si rigenera, a seconda del numero di sottoscrizioni e dello stato di completamento.

## Casi d'uso pratici

### Condivisione delle richieste API

Esempio di come evitare richieste duplicate allo stesso endpoint API.

```ts
import { Observable, of, throwError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, catchError, shareReplay, tap } from 'rxjs';

// Simulazione di un servizio API
class UserService {
  private cache = new Map<string, Observable<any>>();

  getUser(id: string): Observable<any> {
    // Restituisce dalla cache se presente
    if (this.cache.has(id)) {
      console.log(`Recupero ID utente ${id} dalla cache`);
      return this.cache.get(id)!;
    }

    // Crea una nuova richiesta
    console.log(`Recupero ID utente ${id} dall'API`);
    const request$ = ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${id}`).pipe(
      tap(response => console.log('Risposta API:', response)),
      catchError(error => {
        console.error('Errore API:', error);
        // Rimuovi dalla cache
        this.cache.delete(id);
        return throwError(() => new Error('Recupero utente fallito'));
      }),
      // shareReplay per condividere (valore ancora in cache dopo il completamento)
      shareReplay(1)
    );

    // Memorizza nella cache
    this.cache.set(id, request$);
    return request$;
  }
}

// Esempio di utilizzo
const userService = new UserService();

// Più componenti richiedono gli stessi dati utente
console.log('Componente 1: richiesta dati utente');
userService.getUser('1').subscribe(user => {
  console.log('Componente 1: dati utente ricevuti', user);
});

// Dopo un leggero ritardo, un altro componente richiede gli stessi dati
setTimeout(() => {
  console.log('Componente 2: richiesta degli stessi dati utente');
  userService.getUser('1').subscribe(user => {
    console.log('Componente 2: dati utente ricevuti', user);
  });
}, 1000);

// Richiesta di un altro utente
setTimeout(() => {
  console.log('Componente 3: richiesta dati di un altro utente');
  userService.getUser('2').subscribe(user => {
    console.log('Componente 3: dati utente ricevuti', user);
  });
}, 2000);
```

#### Risultato dell'esecuzione
```
Componente 1: richiesta dati utente
Recupero ID utente 1 dall'API
Risposta API: {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Componente 1: dati utente ricevuti {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Componente 2: richiesta degli stessi dati utente
Recupero ID utente 1 dalla cache
Componente 2: dati utente ricevuti {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Componente 3: richiesta dati di un altro utente
Recupero ID utente 2 dall'API
Risposta API: {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
Componente 3: dati utente ricevuti {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
```

## Pattern di progettazione per il multicasting

### Observable Singleton

Un pattern in cui un singolo Observable è condiviso per l'intera applicazione.

```ts
import { Subject } from 'rxjs';

// Gestione dello stato globale dell'applicazione
class AppState {
  // Istanza singleton
  private static instance: AppState;

  // Flusso di notifiche globali
  private notificationsSubject = new Subject<string>();

  // Observable per la pubblicazione (sola lettura)
  readonly notifications$ = this.notificationsSubject.asObservable();

  // Accesso singleton
  static getInstance(): AppState {
    if (!AppState.instance) {
      AppState.instance = new AppState();
    }
    return AppState.instance;
  }

  // Metodo per l'invio di notifiche
  notify(message: string): void {
    this.notificationsSubject.next(message);
  }
}

// Esempio di utilizzo
const appState = AppState.getInstance();

// Monitorare le notifiche (da più componenti)
appState.notifications$.subscribe((msg) =>
  console.log('Componente A:', msg)
);
appState.notifications$.subscribe((msg) =>
  console.log('Componente B:', msg)
);

// Inviare la notifica
appState.notify('Aggiornamento del sistema disponibile');
```

#### Risultato dell'esecuzione
```ts
Componente A: Aggiornamento del sistema disponibile
Componente B: Aggiornamento del sistema disponibile
```

## Riepilogo

Il multicasting è una tecnica importante per migliorare l'efficienza e le prestazioni delle applicazioni RxJS. I punti principali sono i seguenti:

- Il multicasting consente a una singola fonte di dati di essere condivisa da più sottoscrittori
- Può essere implementato utilizzando operatori come `share()`, `shareReplay()` e `publish()`
- Può evitare richieste API duplicate e ottimizzare processi computazionalmente costosi
- Utile per la gestione dello stato e la comunicazione tra componenti

La scelta della giusta strategia di multicast può ridurre la quantità di codice e migliorare la manutenibilità, aumentando la reattività e l'efficienza dell'applicazione.
