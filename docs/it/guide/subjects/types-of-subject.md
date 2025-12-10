---
description: "Spiega le caratteristiche e gli scenari di utilizzo di ciascuno dei quattro tipi di Subject (Subject, BehaviorSubject, ReplaySubject e AsyncSubject). Imparerete con esempi di codice TypeScript come utilizzarli in base alla loro destinazione d'uso, come ad esempio con o senza un valore iniziale, il numero di volte che il valore viene riprodotto e l'ottenimento del valore dopo il completamento."
---

# Tipi di Subject

Oltre al `Subject` di base, RxJS fornisce diverse classi derivate, specifiche per particolari casi d'uso. Ciascuna ha caratteristiche comportamentali diverse e può essere utilizzata in situazioni appropriate per consentire una programmazione reattiva più efficace.

Questa sezione descrive in dettaglio i quattro tipi principali di Subject, le loro caratteristiche e gli scenari di utilizzo.

## Quattro tipi fondamentali di Subject

| Tipi | Caratteristiche | Principali casi d'uso |
|------|------|----------------|
| [`Subject`](#subject) | Subject più semplice<br>riceve solo valori dopo la sottoscrizione | notifica di eventi, multicast |
| [`BehaviorSubject`](#behaviorsubject) | Mantiene il valore più recente e lo fornisce immediatamente dopo una nuova sottoscrizione | Gestione dello stato, valore corrente del componente UI |
| [`ReplaySubject`](#replaysubject) | Riproduce il numero specificato di valori precedenti a un nuovo sottoscrittore | Cronologia delle operazioni, aggiornamenti recenti |
| [`AsyncSubject`](#asyncsubject) | Pubblica solo l'ultimo valore al completamento | Risultato di una richiesta HTTP/API |

## Subject standard {#subject}

[📘 RxJS Official: Subject](https://rxjs.dev/api/index/class/Subject)

Il tipo più semplice di Subject, che riceve solo valori che si verificano dopo la sottoscrizione.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

// Nessun valore iniziale, non viene ricevuto nulla al momento della sottoscrizione
subject.subscribe(value => console.log('Observer 1:', value));

subject.next(1);
subject.next(2);

// Sottoscrivere una seconda volta (riceve il valore solo dopo la sottoscrizione)
subject.subscribe(value => console.log('Observer 2:', value));

subject.next(3);
subject.complete();
```

#### Risultato dell'esecuzione
```
Observer 1: 1
Observer 1: 2
Observer 1: 3
Observer 2: 3
```

## BehaviorSubject {#behaviorsubject}

[📘 RxJS Official: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Richiede un valore iniziale e mantiene sempre il valore più recente.
I nuovi iscritti ricevono l'ultimo valore immediatamente dopo l'iscrizione.

```ts
import { BehaviorSubject } from 'rxjs';

// Crearlo con un valore iniziale di 0
const behaviorSubject = new BehaviorSubject<number>(0);

// Ricevere immediatamente il valore iniziale
behaviorSubject.subscribe(value => console.log('Observer 1:', value));

behaviorSubject.next(1);
behaviorSubject.next(2);

// Sottoscrivere una seconda volta (riceve immediatamente l'ultimo valore 2)
behaviorSubject.subscribe(value => console.log('Observer 2:', value));

behaviorSubject.next(3);
behaviorSubject.complete();
```

#### Risultato dell'esecuzione
```
Observer 1: 0
Observer 1: 1
Observer 1: 2
Observer 2: 2
Observer 1: 3
Observer 2: 3
```

### Esempi di utilizzo di BehaviorSubject

#### Gestione dello stato di autenticazione dell'utente

```ts
import { BehaviorSubject } from 'rxjs';

interface User {
  id: string;
  name: string;
}

// Il valore iniziale è null (stato non loggato)
const currentUser$ = new BehaviorSubject<User | null>(null);

// Monitorare lo stato di login con i componenti, ecc.
currentUser$.subscribe(user => {
  if (user) {
    console.log(`Loggato: ${user.name}`);
  } else {
    console.log('Non loggato');
  }
});

// Elaborazione del login
function login(user: User) {
  currentUser$.next(user);
}

// Elaborazione del logout
function logout() {
  currentUser$.next(null);
}

// Esempio di utilizzo
console.log('Applicazione avviata');
// → Non loggato

login({ id: 'user123', name: 'Mario Rossi' });
// → Loggato: Mario Rossi

logout();
// → Non loggato
```

#### Risultato dell'esecuzione
```sh
Non loggato
Applicazione avviata
Loggato: Mario Rossi
Non loggato
```

## `ReplaySubject` {#replaysubject}
[📘 RxJS Official: ReplaySubject](https://rxjs.dev/api/index/class/ReplaySubject)

Memorizza il numero specificato di valori precedenti e li ripropone ai nuovi abbonati.
La dimensione del buffer e la finestra temporale sono configurabili.

```ts
import { ReplaySubject } from 'rxjs';

// Memorizza gli ultimi 3 valori
const replaySubject = new ReplaySubject<number>(3);

replaySubject.next(1);
replaySubject.next(2);
replaySubject.next(3);
replaySubject.next(4);

// Avvia la sottoscrizione (riceve gli ultimi 3 valori 2, 3, 4)
replaySubject.subscribe(value => console.log('Observer 1:', value));

replaySubject.next(5);

// Sottoscrivere una seconda volta (riceve gli ultimi tre valori 3, 4, 5)
replaySubject.subscribe(value => console.log('Observer 2:', value));

replaySubject.complete();
```

#### Risultato dell'esecuzione
```
Observer 1: 2
Observer 1: 3
Observer 1: 4
Observer 1: 5
Observer 2: 3
Observer 2: 4
Observer 2: 5
```

### ReplaySubject con finestra temporale

È possibile anche un buffering basato sul tempo.

```ts
import { ReplaySubject } from 'rxjs';

// Buffer fino a 5 valori ed entro 500ms
const timeWindowSubject = new ReplaySubject<number>(5, 500);

timeWindowSubject.next(1);

setTimeout(() => {
  timeWindowSubject.next(2);

  // Sottoscrizione dopo 1000 ms (1 non viene ricevuto perché è stato superato l'intervallo di tempo di 500 ms)
  setTimeout(() => {
    timeWindowSubject.subscribe(value => console.log('Ricevuto:', value));
  }, 1000);
}, 100);
```

#### Risultato dell'esecuzione
```
Ricevuto: 2
```

### Esempi di utilizzo di ReplaySubject

#### Gestione della cronologia delle ricerche recenti

```ts
import { ReplaySubject } from 'rxjs';

// Mantenere le ultime 5 query di ricerca
const searchHistory$ = new ReplaySubject<string>(5);

// Funzione per eseguire la ricerca
function search(query: string) {
  console.log(`Ricerca eseguita: ${query}`);
  searchHistory$.next(query);
  // processo di ricerca effettivo...
}

// Componente per mostrare la cronologia delle ricerche
function showSearchHistory() {
  console.log('--- Cronologia ricerche ---');
  searchHistory$.subscribe(query => {
    console.log(query);
  });
}

// Esempio di utilizzo
search('TypeScript');
search('RxJS');
search('Angular');
search('React');

showSearchHistory();
// Mostra la cronologia delle ultime 5 (in questo caso 4) ricerche
```

#### Risultato dell'esecuzione
```sh
Ricerca eseguita: TypeScript
Ricerca eseguita: RxJS
Ricerca eseguita: Angular
Ricerca eseguita: React
--- Cronologia ricerche ---
TypeScript
RxJS
Angular
React
```

## `AsyncSubject` {#asyncsubject}
[📘 RxJS Official: AsyncSubject](https://rxjs.dev/api/index/class/AsyncSubject)

Un Subject in cui solo l'ultimo valore viene emesso al completamento. I valori precedenti al completamento non vengono emessi.

```ts
import { AsyncSubject } from 'rxjs';

const asyncSubject = new AsyncSubject<number>();

asyncSubject.subscribe(value => console.log('Observer 1:', value));

asyncSubject.next(1);
asyncSubject.next(2);
asyncSubject.next(3);

// Riceve solo l'ultimo valore, indipendentemente dalla tempistica di sottoscrizione
asyncSubject.subscribe(value => console.log('Observer 2:', value));

asyncSubject.next(4);
asyncSubject.complete(); // l'ultimo valore (4) viene emesso al completamento
```

#### Risultato dell'esecuzione
```
Observer 1: 4
Observer 2: 4
```

### Esempi di utilizzo di AsyncSubject

#### Condivisione dei risultati delle richieste API

```ts
import { AsyncSubject } from 'rxjs';

interface ApiResponse {
  data: any;
  status: number;
}

function fetchData(url: string) {
  const subject = new AsyncSubject<ApiResponse>();

  // Simulare una richiesta API
  console.log(`Richiesta API: ${url}`);
  setTimeout(() => {
    const response = {
      data: { id: 1, name: 'dati di esempio' },
      status: 200
    };

    subject.next(response);
    subject.complete();
  }, 1000);

  return subject;
}

// Esempio di utilizzo
const data$ = fetchData('/api/users/1');

// Più componenti possono condividere lo stesso risultato della richiesta
data$.subscribe(response => {
  console.log('Componente 1:', response.data);
});

setTimeout(() => {
  data$.subscribe(response => {
    console.log('Componente 2:', response.data);
  });
}, 1500); // riceve ancora valori dopo il completamento
```

#### Risultato dell'esecuzione
```sh
Richiesta API: /api/users/1
Componente 1: {id: 1, name: 'dati di esempio'}
Componente 2: {id: 1, name: 'dati di esempio'}
```

## Guida al confronto e alla selezione per ogni Subject

Questa sezione riassume i punti chiave per aiutarvi a scegliere tra i vari tipi di Subject.

### Come scegliere un Subject

| Tipo | Criteri di selezione |
|---|---|
| `Subject` | Utilizzato per semplici notifiche di eventi e per l'invio multicast |
| `BehaviorSubject` | <li>Casi in cui è sempre richiesto un valore iniziale</li><li>Dati sullo stato corrente (stato dell'utente, impostazioni, flag, ecc.)</li><li>Valore corrente di un componente UI</li> |
| `ReplaySubject` | <li>Casi in cui è necessario mantenere una cronologia delle operazioni recenti</li><li>Se si vogliono fornire dati storici a sottoscrittori che si sono aggiunti in seguito</li><li>Flussi di dati in buffering</li> |
| `AsyncSubject` | <li>Quando è importante solo il risultato finale (ad esempio, la risposta dell'API)</li><li>Quando non sono necessarie fasi intermedie e si vuole condividere solo il valore al completamento</li> |

### Flusso decisionale di selezione

1. È necessario solo l'ultimo valore al completamento ⇨ `AsyncSubject`
2. Sono necessari gli ultimi N valori ⇨ `ReplaySubject`
3. Stato/valore corrente sempre richiesto ⇨ `BehaviorSubject`
4. Altro (ad esempio notifica di eventi puri) ⇨ `Subject`

## Pattern di utilizzo nella progettazione delle applicazioni

### Esempi di comunicazione tra moduli

```ts
// Servizio di gestione dello stato dell'applicazione
class AppStateService {
  // Utente corrente (è richiesto BehaviorSubject come valore iniziale)
  private userSubject = new BehaviorSubject<User | null>(null);
  // Esporre come Observable di sola lettura
  readonly user$ = this.userSubject.asObservable();

  // Notifica (Subject, poiché si tratta di una semplice notifica di evento)
  private notificationSubject = new Subject<Notification>();
  readonly notifications$ = this.notificationSubject.asObservable();

  // Ricerche recenti (ReplaySubject perché è necessaria la cronologia)
  private searchHistorySubject = new ReplaySubject<string>(10);
  readonly searchHistory$ = this.searchHistorySubject.asObservable();

  // Cache dei risultati delle chiamate API (AsyncSubject, poiché sono richiesti solo i risultati finali)
  private readonly apiCaches = new Map<string, AsyncSubject<any>>();

  // Esempio di metodo
  setUser(user: User | null) {
    this.userSubject.next(user);
  }

  notify(notification: Notification) {
    this.notificationSubject.next(notification);
  }

  addSearch(query: string) {
    this.searchHistorySubject.next(query);
  }

  // Cache dei risultati API
  fetchData(url: string): Observable<any> {
    if (!this.apiCaches.has(url)) {
      const subject = new AsyncSubject<any>();
      this.apiCaches.set(url, subject);

      // Chiamata API effettiva
      fetch(url)
        .then(res => res.json())
        .then(data => {
          subject.next(data);
          subject.complete();
        })
        .catch(err => {
          subject.error(err);
        });
    }

    return this.apiCaches.get(url)!.asObservable();
  }
}
```

## Riepilogo

Il Subject RxJS è uno strumento potente per un'ampia gamma di casi d'uso. Comprendendo le caratteristiche di ciascun tipo e utilizzandole in modo appropriato, è possibile realizzare applicazioni reattive efficienti e manutenibili.

- `Subject`: il più semplice, che fornisce funzionalità multicast di base
- `BehaviorSubject`: mantiene sempre lo stato corrente e lo offre immediatamente ai nuovi sottoscrittori
- `ReplaySubject`: mantiene una cronologia dei valori più recenti e la offre ai sottoscrittori che si aggiungono successivamente
- `AsyncSubject`: pubblica solo l'ultimo valore al termine

La scelta del giusto `Subject` per ogni situazione, compresa la gestione degli stati, la notifica degli eventi e la condivisione dei dati, è la chiave per una programmazione reattiva efficiente.
