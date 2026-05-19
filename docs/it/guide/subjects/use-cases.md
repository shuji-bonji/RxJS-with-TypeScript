---
description: "Casi d'uso pratici come la gestione degli stati, il bus degli eventi, il sistema di notifiche globali, la cache dei dati, la gestione reattiva dei moduli e così via, utilizzando la famiglia Subject (Subject, BehaviorSubject, ReplaySubject, AsyncSubject) con ricchi esempi di codice TypeScript. Spiegato con ricchi esempi di codice TypeScript. Comprenderete le caratteristiche di ciascun tipo di Subject e sarete in grado di utilizzarli in situazioni appropriate."
---

# Caso d'uso del Subject.

I Subject RxJS possono essere utilizzati in diversi scenari pratici. Questa sezione presenta casi d'uso pratici per la famiglia Subject (Subject, BehaviorSubject, ReplaySubject e AsyncSubject) e descrive le situazioni in cui ciascuno di essi è più adatto.

## Modelli di gestione dello stato

### Implementazione di un semplice negozio.

Usare BehaviorSubject per implementare un semplice negozio che può contenere, aggiornare e sottoscrivere lo stato dell'applicazione.

```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// Stato iniziale
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // BehaviorSubjectGestione dello stato con
  private state$ = new BehaviorSubject<AppState>(initialState);
  
  // Metodi per leggere lo stato
  getState() {
    return this.state$.getValue();
  }
  
  // Recupera la proprietà specificata comeObservableRecupera come
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }
  
  // Aggiornamento dello stato
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }
  
  // Recupera lo stato comeObservablePubblica come
  get state() {
    return this.state$.asObservable();
  }
}

// Esempio di utilizzo
const store = new Store();

// Monitoraggio dello stato
store.select('user').subscribe(user => {
  console.log('Cambiamenti di stato dell'utente:', user?.name, user?.role);
});

// Monitoraggio delle modifiche al tema
store.select('theme').subscribe(theme => {
  console.log('Modifiche al tema:', theme);
  document.body.className = theme; // UIRiflesso in
});

// Aggiornamento dello stato
store.setState({ user: { name: 'Taro Yamada', role: 'admin' } });
store.setState({ theme: 'dark' });
```

#### Risultato dell'esecuzione

```sh
Cambiamenti di stato dell'utente: undefined undefined
Modifiche al tema: light
Cambiamenti di stato dell'utente: Taro Yamada admin
Modifiche al tema: light
Cambiamenti di stato dell'utente: Taro Yamada admin
Modifiche al tema: dark
```

Questo pattern è utile per le applicazioni di piccole dimensioni o quando non si utilizzano librerie di gestione degli stati di grandi dimensioni, come NgRx o Redux.

## Comunicazione da componente a componente

### Implementazione del bus degli eventi.

Implementare un bus di eventi basato su Subject, che può gestire diversi tipi di dati per diversi tipi di notifica, per la comunicazione tra componenti.

```ts
import { Subject } from 'rxjs';
import { filter, map } from 'rxjs';

type EventPayloadMap = {
  USER_LOGIN: { username: string; timestamp: number };
  DATA_UPDATED: any;
  NOTIFICATION: string;
};

// Definizione del tipo di evento
type EventType = keyof EventPayloadMap;

interface AppEvent<K extends EventType> {
  type: K;
  payload: EventPayloadMap[K];
}

// Servizio bus eventi
class EventBusService {
  private eventSubject = new Subject<AppEvent<unknown>>();

  emit<K extends EventType>(type: K, payload: EventPayloadMap[K]): void {
    this.eventSubject.next({ type, payload });
  }

  // Sottoscrizione di eventi di un tipo specifico
  on<K extends EventType>(type: K) {
    return this.eventSubject.pipe(
      filter((event): event is AppEvent<K> => event.type === type),
      map((event) => event.payload)
    );
  }
}
// Esempio di utilizzo) Comunicazione da componente a componente
const eventBus = new EventBusService();

// Componente Header (visualizza le notifiche)
eventBus.on('NOTIFICATION').subscribe((message) => {
  console.log('Intestazione: Visualizza le notifiche:', message);
});

// Componente utente (controlla lo stato di accesso)
eventBus.on('USER_LOGIN').subscribe((user) => {
  console.log('Componente utente: Rilevamento dell'accesso:', user.username);
});

// Componente di configurazione (controlla gli aggiornamenti dei dati)
eventBus.on('DATA_UPDATED').subscribe((data) => {
  console.log('Componente di configurazione: Aggiornamento dati:', data);
});

// Evento emesso
eventBus.emit('USER_LOGIN', { username: 'user123', timestamp: Date.now() });
eventBus.emit('NOTIFICATION', 'C'è un nuovo messaggio');
```

#### Risultati dell'esecuzione

```sh
Componente utente: Rilevamento dell'accesso: user123
Intestazione: Visualizza le notifiche: C'è un nuovo messaggio
```

Il pattern Event Bus è un modo eccellente per ottenere una comunicazione intercomponente non accoppiata. È particolarmente adatto per la comunicazione tra componenti distanti tra loro nella gerarchia.

> [!CAUTION]

> 💡 Nelle applicazioni reali, la mancata unsubscribe (`unsubscribe()`) può portare a perdite di memoria. Si consideri anche il processo di disiscrizione tramite l'uso di takeUntil()` o simili.

## Caching dei dati API

### Condivisione e cache dei risultati delle richieste

Usare AsyncSubject per condividere e mettere in cache i dati che vengono emessi una sola volta, come le richieste HTTP.

```ts
import { Observable, AsyncSubject, of, throwError } from 'rxjs';
import { tap, catchError, delay } from 'rxjs';

class ApiCacheService {
  private cache = new Map<string, AsyncSubject<any>>();

  fetchData<T>(url: string): Observable<T> {
    // Lo restituisce se esiste nella cache
    if (this.cache.has(url)) {
      console.log(`Recupera i dati dalla cache: ${url}`);
      return this.cache.get(url)!.asObservable() as Observable<T>;
    }

    // Se non è presente nella cache, crea una nuova richiesta
    console.log(`APIEseguire la richiesta: ${url}`);
    const subject = new AsyncSubject<T>();
    this.cache.set(url, subject);

    // APISimula la richiesta
    this.makeRequest<T>(url)
      .pipe(
        tap((data) => {
          subject.next(data);
          subject.complete();
        }),
        catchError((error: unknown) => {
          // Cancella dalla cache in caso di errore
          this.cache.delete(url);
          subject.error(error);
          return throwError(() => error);
        })
      )
      .subscribe();

    return subject.asObservable();
  }

  // Simulare l'effettivaAPIElaborazione della richiesta
  private makeRequest<T>(url: string): Observable<T> {
    // Nelle applicazioni realifetcheHTTPUtilizzare il client
    return of({
      data: 'Dati di esempio',
      timestamp: Date.now(),
    } as unknown as T).pipe(
      tap(() => console.log('APIRicezione della risposta')),
      // Simulare ritardi casuali
      delay(Math.random() * 1000 + 500)
    );
  }

  // Cancellare la cache
  clearCache(url?: string): void {
    if (url) {
      this.cache.delete(url);
    } else {
      this.cache.clear();
    }
    console.log('Cache cancellata');
  }
}

// Esempio di utilizzo
const apiCache = new ApiCacheService();

// Più componenti richiedono lo stesso datoAPIRichiesta di dati
apiCache.fetchData('/api/products').subscribe((data) => {
  console.log('Componente1: Dati ricevuti', data);
});

// Poco dopo anche un altro componente richiede gli stessi dati (recuperati dalla cache)
setTimeout(() => {
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Componente2: Dati ricevuti', data);
  });
}, 1000);

// Richiesta di nuovo dopo aver cancellato la cache
setTimeout(() => {
  apiCache.clearCache();
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Componente3: Dati ricevuti (dopo aver cancellato la cache)', data);
  });
}, 2000);
```

#### Risultati dell'esecuzione

```sh
APIEseguire la richiesta: /api/products
APIRicezione della risposta
Componente1: Dati ricevuti {data: 'Dati di esempio', timestamp: 1745405703582}
Recupera i dati dalla cache: /api/products
Componente2: Dati ricevuti {data: 'Dati di esempio', timestamp: 1745405703582}
Cache cancellata
APIEseguire la richiesta: /api/products
APIRicezione della risposta
Componente3: Dati ricevuti (dopo aver cancellato la cache) {data: 'Dati di esempio', timestamp: 1745405705585}
```

Questo schema con AsyncSubject è ideale per le richieste API in cui è importante solo l'ultimo valore al completamento. Inoltre, impedisce la duplicazione della stessa richiesta.

```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// Stato iniziale
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // BehaviorSubjectGestione dello stato con
  private state$ = new BehaviorSubject<AppState>(initialState);
  
  // Metodi per leggere lo stato
  getState() {
    return this.state$.getValue();
  }
  
  // Recupera la proprietà specificata comeObservableRecupera come
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }
  
  // Aggiornamento dello stato
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }
  
  // Recupera lo stato comeObservablePubblica come
  get state() {
    return this.state$.asObservable();
  }
}

// Esempio di utilizzo
const store = new Store();

// Monitoraggio dello stato
store.select('user').subscribe(user => {
  console.log('Cambiamenti di stato dell'utente:', user?.name, user?.role);
});

// Monitoraggio delle modifiche al tema
store.select('theme').subscribe(theme => {
  console.log('Modifiche al tema:', theme);
  document.body.className = theme; // UIRiflesso in
});

// Aggiornamento dello stato
store.setState({ user: { name: 'Taro Yamada', role: 'admin' } });
store.setState({ theme: 'dark' });
```

> Si noti che AsyncSubject notificherà solo un `errore', non un valore, se viene chiamato `error()`.

## Gestione dei moduli.

Usare BehaviorSubject per gestire il valore corrente e lo stato di convalida di un modulo reattivo.
### Legame bidirezionale dei valori dei form.


```ts
import { BehaviorSubject } from 'rxjs';
import { debounceTime, distinctUntilChanged } from 'rxjs';

interface UserForm {
  name: string;
  email: string;
  age: number;
}

class ReactiveForm {
  // Con valori inizialiBehaviorSubject
  private formSubject = new BehaviorSubject<UserForm>({
    name: '',
    email: '',
    age: 0
  });
  
  // Per la pubblicazioneObservable
  formValues$ = this.formSubject.asObservable();
  
  // Risultato della convalida
  private validSubject = new BehaviorSubject<boolean>(false);
  valid$ = this.validSubject.asObservable();
  
  constructor() {
    // Convalida eseguita quando si modificano i valori
    this.formValues$.pipe(
      debounceTime(300),
      distinctUntilChanged((prev, curr) => JSON.stringify(prev) === JSON.stringify(curr))
    ).subscribe(form => {
      this.validateForm(form);
    });
  }
  
  // Aggiornamento dei valori dei campi
  updateField<K extends keyof UserForm>(field: K, value: UserForm[K]) {
    const currentForm = this.formSubject.getValue();
    this.formSubject.next({
      ...currentForm,
      [field]: value
    });
  }
  
  // Recupero del modulo
  getForm(): UserForm {
    return this.formSubject.getValue();
  }
  
  // Convalida
  private validateForm(form: UserForm) {
    const isValid = 
      form.name.length > 0 && 
      form.email.includes('@') &&
      form.age > 0;
      
    this.validSubject.next(isValid);
  }
  
  // Invio del modulo
  submit() {
    if (this.validSubject.getValue()) {
      console.log('Invio del modulo:', this.getForm());
      // APIRichieste ecc.
    } else {
      console.error('Il modulo non è valido');
    }
  }
}

// Esempio di utilizzo
const form = new ReactiveForm();

// Monitoraggio dei valori del modulo
form.formValues$.subscribe(values => {
  console.log('Modifiche del valore del modulo:', values);
  // UIprocesso di aggiornamento, ecc.
});

// Monitoraggio dello stato di convalida
form.valid$.subscribe(isValid => {
  console.log('Validità del modulo:', isValid);
  // Abilitazione/disabilitazione del pulsante di invio/Disabilitazione del pulsante di invio, ecc.
});

// Simulazione dell'input dell'utente
form.updateField('name', 'Taro Yamada');
form.updateField('email', 'yamada@example.com');
form.updateField('age', 30);

// Invio del modulo
form.submit();
```

#### Risultato dell'esecuzione

```sh
Modifiche del valore del modulo: {name: '', email: '', age: 0}
Validità del modulo: false
Modifiche del valore del modulo: {name: 'Taro Yamada', email: '', age: 0}
Modifiche del valore del modulo: {name: 'Taro Yamada', email: 'yamada@example.com', age: 0}
Modifiche del valore del modulo: {name: 'Taro Yamada', email: 'yamada@example.com', age: 30}
Il modulo non è valido
submit @ 
(anonimo) @ Analizzare l'errore
Validità del modulo: true
```

Questo pattern è particolarmente utile per le implementazioni reattive dei form, in quanto BehaviorSubject mantiene sempre il valore corrente, rendendolo ideale per la gestione dello stato del form.

## Registrazione e cronologia

Usare ReplaySubject per creare un meccanismo di gestione dei log, in grado di conservare e visualizzare la cronologia delle operazioni passate.
### Gestione della cronologia delle operazioni.

```ts
import { Observable, ReplaySubject } from 'rxjs';
import { tap } from 'rxjs';

interface LogEntry {
  action: string;
  timestamp: number;
  data?: any;
}

class ActivityLogger {
  // Ultimo10Conserva un registro degli errori più recenti
  private logSubject = new ReplaySubject<LogEntry>(10);
  logs$ = this.logSubject.asObservable();
  
  // Aggiungere una voce di registro
  log(action: string, data?: any) {
    const entry: LogEntry = {
      action,
      timestamp: Date.now(),
      data
    };
    
    this.logSubject.next(entry);
    console.log(`Registrato: ${action}`, data);
  }
  
  // Avvolge un altroObservableAvvolge e registra un registro
  wrapWithLogging<T>(source$: Observable<T>, actionName: string): Observable<T> {
    return source$.pipe(
      tap(data => this.log(actionName, data))
    );
  }
}

// Esempio di utilizzo
const logger = new ActivityLogger();

// Monitorare i registri (ad es.UI(ad esempio, per visualizzarli)
logger.logs$.subscribe(log => {
  const time = new Date(log.timestamp).toLocaleTimeString();
  console.log(`[${time}] ${log.action}`);
});

// Registra vari々Registra operazioni come
logger.log('Avvio dell'applicazione');
logger.log('Accesso dell'utente', { userId: 'user123' });

// Poco dopo, si avvia la sottoscrizione di un nuovo componente, che include i registri passati.
setTimeout(() => {
  console.log('--- Il visualizzatore della cronologia include i registri passati. ---');
  logger.logs$.subscribe(log => {
    const time = new Date(log.timestamp).toLocaleTimeString();
    console.log(`La storia: [${time}] ${log.action}`);
  });
  
  // Ulteriori registri aggiunti
  logger.log('Aggiornamento dati', { itemId: 456 });
}, 1000);
```

#### Risultato dell'esecuzione

```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// Stato iniziale
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // BehaviorSubjectGestione dello stato con
  private state$ = new BehaviorSubject<AppState>(initialState);
  
  // Metodi per leggere lo stato
  getState() {
    return this.state$.getValue();
  }
  
  // Recupera la proprietà specificata comeObservableRecupera come
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }
  
  // Aggiornamento dello stato
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }
  
  // Recupera lo stato comeObservablePubblica come
  get state() {
    return this.state$.asObservable();
  }
}

// Esempio di utilizzo
const store = new Store();

// Monitoraggio dello stato
store.select('user').subscribe(user => {
  console.log('Cambiamenti di stato dell'utente:', user?.name, user?.role);
});

// Monitoraggio delle modifiche al tema
store.select('theme').subscribe(theme => {
  console.log('Modifiche al tema:', theme);
  document.body.className = theme; // UIRiflesso in
});

// Aggiornamento dello stato
store.setState({ user: { name: 'Taro Yamada', role: 'admin' } });
store.setState({ theme: 'dark' });
```

ReplaySubject può essere usato per fornire ai nuovi sottoscrittori le voci di registro passate, ideale per la gestione della cronologia. È utile per tracciare le operazioni dell'utente e raccogliere informazioni di debug.

> [!IMPORTANT]

> ⚠️ Se non si specifica una dimensione del buffer per ReplaySubject, tutti i valori continueranno a essere conservati in memoria, quindi occorre prestare attenzione a grandi quantità di dati o ad applicazioni di lunga durata.

## Gestione dell'elaborazione asincrona

Utilizzate `Subject` e `BehaviourSubject` per gestire l'avanzamento e lo stato attivo di più attività in tempo reale.
## Gestire l'avanzamento di task di lunga durata.


```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// Stato iniziale
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // BehaviorSubjectGestione dello stato con
  private state$ = new BehaviorSubject<AppState>(initialState);
  
  // Metodi per leggere lo stato
  getState() {
    return this.state$.getValue();
  }
  
  // Recupera la proprietà specificata comeObservableRecupera come
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }
  
  // Aggiornamento dello stato
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }
  
  // Recupera lo stato comeObservablePubblica come
  get state() {
    return this.state$.asObservable();
  }
}

// Esempio di utilizzo
const store = new Store();

// Monitoraggio dello stato
store.select('user').subscribe(user => {
  console.log('Cambiamenti di stato dell'utente:', user?.name, user?.role);
});

// Monitoraggio delle modifiche al tema
store.select('theme').subscribe(theme => {
  console.log('Modifiche al tema:', theme);
  document.body.className = theme; // UIRiflesso in
});

// Aggiornamento dello stato
store.setState({ user: { name: 'Taro Yamada', role: 'admin' } });
store.setState({ theme: 'dark' });
```

#### Risultati dell'esecuzione


Questo schema fornisce una notifica in tempo reale dell'avanzamento di un'attività in corso da tempo, utilizzando Subject. È adatto per visualizzare l'avanzamento del caricamento dei file, dell'elaborazione dei dati e delle operazioni in background.

## Aggiornamenti in tempo reale

Gestire lo stato della connessione WebSocket, i messaggi in arrivo e il controllo della riconnessione utilizzando più Subject.
### Gestione dei flussi WebSocket.


```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// Stato iniziale
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // BehaviorSubjectGestione dello stato con
  private state$ = new BehaviorSubject<AppState>(initialState);
  
  // Metodi per leggere lo stato
  getState() {
    return this.state$.getValue();
  }
  
  // Recupera la proprietà specificata comeObservableRecupera come
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }
  
  // Aggiornamento dello stato
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }
  
  // Recupera lo stato comeObservablePubblica come
  get state() {
    return this.state$.asObservable();
  }
}

// Esempio di utilizzo
const store = new Store();

// Monitoraggio dello stato
store.select('user').subscribe(user => {
  console.log('Cambiamenti di stato dell'utente:', user?.name, user?.role);
});

// Monitoraggio delle modifiche al tema
store.select('theme').subscribe(theme => {
  console.log('Modifiche al tema:', theme);
  document.body.className = theme; // UIRiflesso in
});

// Aggiornamento dello stato
store.setState({ user: { name: 'Taro Yamada', role: 'admin' } });
store.setState({ theme: 'dark' });
```

#### Risultati dell'esecuzione


Questo pattern di gestione WebSocket è ideale per le applicazioni che richiedono una comunicazione in tempo reale; utilizza Subject per gestire lo stato della connessione e il flusso dei messaggi, che possono essere condivisi da più componenti.

##Linee guida per la scelta di un Subject.


```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// Stato iniziale
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // BehaviorSubjectGestione dello stato con
  private state$ = new BehaviorSubject<AppState>(initialState);
  
  // Metodi per leggere lo stato
  getState() {
    return this.state$.getValue();
  }
  
  // Recupera la proprietà specificata comeObservableRecupera come
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }
  
  // Aggiornamento dello stato
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }
  
  // Recupera lo stato comeObservablePubblica come
  get state() {
    return this.state$.asObservable();
  }
}

// Esempio di utilizzo
const store = new Store();

// Monitoraggio dello stato
store.select('user').subscribe(user => {
  console.log('Cambiamenti di stato dell'utente:', user?.name, user?.role);
});

// Monitoraggio delle modifiche al tema
store.select('theme').subscribe(theme => {
  console.log('Modifiche al tema:', theme);
  document.body.className = theme; // UIRiflesso in
});

// Aggiornamento dello stato
store.setState({ user: { name: 'Taro Yamada', role: 'admin' } });
store.setState({ theme: 'dark' });
```

> 💡 È convenzione comune di RxJS terminare i nomi delle variabili con `$` per indicare che sono Observable.

## Riepilogo.

La famiglia Subject di RxJS è uno strumento potente per una varietà di casi d'uso, tra cui.

- BehaviorSubject**: gestione degli stati, gestione dei moduli, visualizzazione dei valori correnti.
- Subject**: notifica di eventi, comunicazione tra componenti.
- ReplaySubject**: gestione della cronologia, registri delle operazioni, partecipazione ritardata dei componenti.
- AsyncSubject**: caching delle risposte API, condivisione dei risultati dei calcoli.

Combinando opportunamente questi pattern, si possono costruire applicazioni reattive e manutenibili. In particolare, bisogna fare attenzione a non dimenticare di disiscriversi al momento giusto, per evitare perdite di memoria.