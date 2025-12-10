---
description: "Subject è una classe speciale di RxJS con proprietà sia Observable che Observer. Può pubblicare e sottoscrivere dati allo stesso tempo e può fornire lo stesso valore a più sottoscrittori tramite multicast. Può implementare pattern pratici come event bus e gestione dello stato, mantenendo la type safety con i parametri di tipo TypeScript."
---

# Cos'è un Subject

[📘 RxJS Official: Subject](https://rxjs.dev/api/index/class/Subject)

Il Subject è un tipo speciale di Observable in RxJS. Mentre un normale Observable fornisce un flusso di dati unidirezionale, un Subject è un'entità ibrida con proprietà sia di "Observable" che di "Observer".

I Subject hanno le seguenti caratteristiche:

- Può pubblicare dati (funzione Observable)
- Può sottoscrivere dati (funzione Observer)
- Può fornire lo stesso valore a più sottoscrittori (Multicast)
- Riceve solo valori che si verificano dopo la sottoscrizione (proprietà Hot Observable)


## Utilizzo di base del Subject

```ts
import { Subject } from 'rxjs';

// Creare il Subject
const subject = new Subject<number>();

// Sottoscrivere come Observer
subject.subscribe(value => console.log('Observer A:', value));
subject.subscribe(value => console.log('Observer B:', value));

// Pubblicare il valore come Observable
subject.next(1); // Pubblica il valore a entrambi i sottoscrittori
subject.next(2); // Pubblica il valore a entrambi i sottoscrittori

// Aggiungere un nuovo sottoscrittore (sottoscrizione ritardata)
subject.subscribe(value => console.log('Observer C:', value));

subject.next(3); // Pubblica il valore a tutti i sottoscrittori

// Notifica il completamento
subject.complete();
```

#### Risultato dell'esecuzione
```
Observer A: 1
Observer B: 1
Observer A: 2
Observer B: 2
Observer A: 3
Observer B: 3
Observer C: 3
```

### Differenze rispetto al normale Observable

Il Subject è un **Hot Observable** e differisce da un normale Cold Observable nei seguenti modi:

- I dati vengono pubblicati con o senza sottoscrizioni
- Lo stesso valore può essere condiviso da più sottoscrittori (multicast)
- I valori possono essere pubblicati esternamente con `.next()`
- I valori passati non vengono conservati, ma vengono ricevuti solo quelli successivi alla sottoscrizione


## Subject e Multicasting

Una delle caratteristiche principali di Subject è il "multicasting".
Si tratta della capacità di distribuire in modo efficiente una fonte di dati a più sottoscrittori.

```ts
import { Subject, interval } from 'rxjs';
import { take } from 'rxjs';

// Fonte dei dati
const source$ = interval(1000).pipe(take(3));

// Subject per il multicast
const subject = new Subject<number>();

// Collegare la sorgente al Subject
source$.subscribe(subject); // Il Subject agisce da sottoscrittore

// Più osservatori si iscrivono al Subject
subject.subscribe(value => console.log('Observer 1:', value));
subject.subscribe(value => console.log('Observer 2:', value));
```

#### Risultato dell'esecuzione
```
Observer 1: 0
Observer 2: 0
Observer 1: 1
Observer 2: 1
Observer 1: 2
Observer 2: 2
```

Questo pattern, noto anche come multicast a sorgente singola, viene utilizzato per distribuire in modo efficiente una singola fonte di dati a più sottoscrittori.


## Due usi del Subject

Esistono due usi principali del Subject. Ognuno di essi ha un uso e un comportamento diverso.

### 1. Pattern di chiamata `.next()` direttamente

Il Subject viene usato come **entità che pubblica dati (Observable)**.
Questo pattern è adatto per "inviare valori espliciti", come le notifiche di eventi e gli aggiornamenti di stato.

```ts
const subject = new Subject<string>();

subject.subscribe(val => console.log('Observer A:', val));
subject.next('Hello');
subject.next('World');

// Output:
// Observer A: Hello
// Observer A: World
```

---

### 2. Pattern per la trasmissione di Observable (multicast)

Il Subject agisce come **Observer**, ricevendo e trasmettendo i valori dell'Observable.
Questo utilizzo è utile per la **conversione di Cold Observable in Hot e per il multicasting**.

```ts
const source$ = interval(1000).pipe(take(3));
const subject = new Subject<number>();

// Observable → Subject (relay)
source$.subscribe(subject);

// Subject → Distribuzione a più sottoscrittori
subject.subscribe(val => console.log('Observer 1:', val));
subject.subscribe(val => console.log('Observer 2:', val));

// Output:
// Observer 1: 0
// Observer 2: 0
// Observer 1: 1
// Observer 2: 1
// Observer 1: 2
// Observer 2: 2
```



> [!TIP]
> È più facile capire se si immagina che chiamare `.next()` direttamente sia come "una persona che parla da sola", mentre ricevere e trasmettere da un Observable sia come "una persona che usa un microfono per amplificare il discorso di qualcun altro".


## Casi d'uso pratici per Subject

Subject è particolarmente utile nei seguenti scenari:

1. **Gestione dello stato** - condivisione e aggiornamento dello stato dell'applicazione
2. **Event Bus** - comunicazione tra i componenti
3. **Condivisione delle risposte HTTP** - condivisione dei risultati di una stessa chiamata API tra più componenti
4. **Gestione centralizzata degli eventi UI** - gestione di varie operazioni dell'interfaccia utente in un unico punto

#### Esempio: Implementazione di un Event Bus
```ts
import { Subject } from 'rxjs';
import { filter } from 'rxjs';

interface AppEvent {
  type: string;
  payload: any;
}

// Event bus a livello di applicazione
const eventBus = new Subject<AppEvent>();

// Sottoscrivere un tipo di evento specifico
eventBus.pipe(
  filter(event => event.type === 'USER_LOGGED_IN')
).subscribe(event => {
  console.log('Utente connesso:', event.payload);
});

// Sottoscrivere un altro tipo di evento
eventBus.pipe(
  filter(event => event.type === 'DATA_UPDATED')
).subscribe(event => {
  console.log('Dati aggiornati:', event.payload);
});

// Emissione evento
eventBus.next({ type: 'USER_LOGGED_IN', payload: { userId: '123', username: 'test_user' } });
eventBus.next({ type: 'DATA_UPDATED', payload: { items: [1, 2, 3] } });
```

#### Risultato dell'esecuzione
```
Utente connesso: {userId: '123', username: 'test_user'}
Dati aggiornati: {items: Array(3)}
```

## Riepilogo

Il Subject è un importante elemento dell'ecosistema RxJS, che svolge i seguenti ruoli:

- Ha le proprietà sia dell'Observer (osservatore) che dell'Observable (osservabile)
- Fornisce un mezzo per convertire un Cold Observable in Hot
- Fornisce in modo efficiente lo stesso flusso di dati a più sottoscrittori
- Facilita la comunicazione tra componenti e servizi
- Fornisce una base per la gestione dello stato e degli eventi

## 🔗 Sezioni correlate

- **[Errori comuni e rimedi](/it/guide/anti-patterns/common-mistakes#1-subject-の外部公開)** - Best practice per evitare l'uso improprio dei Subject
- **[Tipi di Subject](./types-of-subject)** - BehaviorSubject, ReplaySubject, AsyncSubject, ecc.
