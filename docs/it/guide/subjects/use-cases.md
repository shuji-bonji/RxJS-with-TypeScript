---
description: "Imparerete i pattern pratici per utilizzare RxJS Subject nelle vostre applicazioni: gestione dello stato, comunicazione tra componenti, caching dei dati API, gestione dei form, gestione della cronologia, gestione delle code asincrone e gestione degli stream WebSocket, con esempi di codice TypeScript."
---

# Casi d'uso pratici di Subject

Questa sezione presenta pattern pratici per i casi d'uso dei Subject nelle applicazioni reali.

## Gestione dello stato

BehaviorSubject è adatto a contenere lo stato più recente e a fornirlo ai nuovi sottoscrittori.

### Pattern di store semplice

```ts
import { BehaviorSubject, Observable } from 'rxjs';
import { map, distinctUntilChanged } from 'rxjs';

// Definizione dell'interfaccia di stato
interface AppState {
  user: { id: string; name: string } | null;
  isLoading: boolean;
  items: string[];
}

// Stato iniziale
const initialState: AppState = {
  user: null,
  isLoading: false,
  items: []
};

// Implementazione di uno store semplice
class SimpleStore {
  private state$: BehaviorSubject<AppState>;

  constructor(initialState: AppState) {
    this.state$ = new BehaviorSubject<AppState>(initialState);
  }

  // Ottenere l'intero stato
  getState(): AppState {
    return this.state$.getValue();
  }

  // Ottenere lo stato come Observable
  select<T>(selector: (state: AppState) => T): Observable<T> {
    return this.state$.pipe(
      map(selector),
      distinctUntilChanged()
    );
  }

  // Aggiornare lo stato
  setState(partial: Partial<AppState>): void {
    this.state$.next({
      ...this.getState(),
      ...partial
    });
  }
}

// Esempio di utilizzo
const store = new SimpleStore(initialState);

// Monitorare lo stato del login
store.select(state => state.user).subscribe(user => {
  console.log('Utente:', user);
});

// Aggiornamento dello stato
store.setState({ user: { id: '1', name: 'Mario Rossi' } });
store.setState({ isLoading: true });
```

#### Risultato dell'esecuzione
```
Utente: null
Utente: {id: '1', name: 'Mario Rossi'}
```

### Pattern Redux-like

Per applicazioni più grandi, è possibile incorporare i concetti di azioni e reducer.

```ts
import { BehaviorSubject, Subject, Observable, scan, startWith } from 'rxjs';

// Definizione delle azioni
type Action =
  | { type: 'SET_USER'; payload: { id: string; name: string } }
  | { type: 'LOGOUT' }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'ADD_ITEM'; payload: string };

// Tipo dello stato
interface State {
  user: { id: string; name: string } | null;
  isLoading: boolean;
  items: string[];
}

// Stato iniziale
const initialState: State = {
  user: null,
  isLoading: false,
  items: []
};

// Reducer
function reducer(state: State, action: Action): State {
  switch (action.type) {
    case 'SET_USER':
      return { ...state, user: action.payload };
    case 'LOGOUT':
      return { ...state, user: null };
    case 'SET_LOADING':
      return { ...state, isLoading: action.payload };
    case 'ADD_ITEM':
      return { ...state, items: [...state.items, action.payload] };
    default:
      return state;
  }
}

// Store in stile Redux
class ReduxLikeStore {
  private actions$ = new Subject<Action>();
  private state$: Observable<State>;

  constructor() {
    this.state$ = this.actions$.pipe(
      scan(reducer, initialState),
      startWith(initialState)
    );
  }

  dispatch(action: Action): void {
    this.actions$.next(action);
  }

  select<T>(selector: (state: State) => T): Observable<T> {
    return this.state$.pipe(
      // @ts-ignore - per semplicità
      map(selector)
    );
  }

  getState$(): Observable<State> {
    return this.state$;
  }
}

// Esempio di utilizzo
const reduxStore = new ReduxLikeStore();

reduxStore.getState$().subscribe(state => {
  console.log('Stato:', state);
});

reduxStore.dispatch({ type: 'SET_USER', payload: { id: '1', name: 'Mario Rossi' } });
reduxStore.dispatch({ type: 'ADD_ITEM', payload: 'Nuovo elemento' });
```

#### Risultato dell'esecuzione
```
Stato: {user: null, isLoading: false, items: Array(0)}
Stato: {user: {…}, isLoading: false, items: Array(0)}
Stato: {user: {…}, isLoading: false, items: Array(1)}
```

## Comunicazione tra componenti

Subject è utile per la comunicazione tra componenti non in relazione genitore-figlio (ad esempio, componenti fratelli o distanti).

### Event Bus

```ts
import { Subject, filter, Observable } from 'rxjs';

// Definizione dell'interfaccia evento
interface AppEvent {
  type: string;
  payload?: any;
  source?: string;
}

// Implementazione dell'Event Bus
class EventBus {
  private events$ = new Subject<AppEvent>();

  // Emettere un evento
  emit(event: AppEvent): void {
    this.events$.next(event);
  }

  // Sottoscrivere tutti gli eventi
  on(): Observable<AppEvent> {
    return this.events$.asObservable();
  }

  // Sottoscrivere un tipo di evento specifico
  onType(type: string): Observable<AppEvent> {
    return this.events$.pipe(
      filter(event => event.type === type)
    );
  }

  // Sottoscrivere eventi da una specifica origine
  fromSource(source: string): Observable<AppEvent> {
    return this.events$.pipe(
      filter(event => event.source === source)
    );
  }
}

// Esempio di utilizzo
const eventBus = new EventBus();

// Sottoscrizione del Componente A
eventBus.onType('USER_ACTION').subscribe(event => {
  console.log('Componente A ha ricevuto:', event);
});

// Sottoscrizione del Componente B
eventBus.onType('DATA_UPDATE').subscribe(event => {
  console.log('Componente B ha ricevuto:', event);
});

// Emissione di eventi
eventBus.emit({ type: 'USER_ACTION', payload: { action: 'click' }, source: 'Header' });
eventBus.emit({ type: 'DATA_UPDATE', payload: { items: [1, 2, 3] }, source: 'API' });
```

#### Risultato dell'esecuzione
```
Componente A ha ricevuto: {type: 'USER_ACTION', payload: {…}, source: 'Header'}
Componente B ha ricevuto: {type: 'DATA_UPDATE', payload: {…}, source: 'API'}
```

### Pattern Pub/Sub per argomento

```ts
import { Subject, Observable } from 'rxjs';

// Implementazione del Pub/Sub per argomento
class TopicPubSub<T = any> {
  private topics = new Map<string, Subject<T>>();

  // Ottenere o creare un argomento
  private getTopic(topic: string): Subject<T> {
    if (!this.topics.has(topic)) {
      this.topics.set(topic, new Subject<T>());
    }
    return this.topics.get(topic)!;
  }

  // Pubblicare
  publish(topic: string, message: T): void {
    this.getTopic(topic).next(message);
  }

  // Sottoscrivere
  subscribe(topic: string): Observable<T> {
    return this.getTopic(topic).asObservable();
  }

  // Eliminare un argomento
  removeTopic(topic: string): void {
    const subject = this.topics.get(topic);
    if (subject) {
      subject.complete();
      this.topics.delete(topic);
    }
  }
}

// Esempio di utilizzo
const pubsub = new TopicPubSub<string>();

// Sottoscrivere a più argomenti
pubsub.subscribe('chat').subscribe(msg => console.log('Chat:', msg));
pubsub.subscribe('notification').subscribe(msg => console.log('Notifica:', msg));

// Pubblicare messaggi
pubsub.publish('chat', 'Ciao a tutti!');
pubsub.publish('notification', 'Nuovo messaggio ricevuto');
```

#### Risultato dell'esecuzione
```
Chat: Ciao a tutti!
Notifica: Nuovo messaggio ricevuto
```

## Caching dei dati API

AsyncSubject è utile per memorizzare nella cache i risultati delle chiamate API.

### Servizio API con cache

```ts
import { AsyncSubject, Observable, of, throwError, timer } from 'rxjs';
import { catchError, switchMap, tap } from 'rxjs';

interface User {
  id: string;
  name: string;
  email: string;
}

// Servizio API con cache
class UserApiService {
  private cache = new Map<string, AsyncSubject<User>>();
  private cacheExpiry = new Map<string, number>();
  private readonly CACHE_DURATION = 5 * 60 * 1000; // 5 minuti

  // Ottenere i dati utente (con cache)
  getUser(id: string): Observable<User> {
    // Controllare la scadenza della cache
    if (this.isCacheValid(id)) {
      console.log(`Cache hit: utente ${id}`);
      return this.cache.get(id)!.asObservable();
    }

    // Creare una nuova richiesta
    console.log(`Cache miss: recupero utente ${id} dall'API`);
    const subject = new AsyncSubject<User>();
    this.cache.set(id, subject);
    this.cacheExpiry.set(id, Date.now() + this.CACHE_DURATION);

    // Simulare una chiamata API
    this.fetchUser(id).pipe(
      tap(user => {
        subject.next(user);
        subject.complete();
      }),
      catchError(error => {
        this.cache.delete(id);
        this.cacheExpiry.delete(id);
        subject.error(error);
        return throwError(() => error);
      })
    ).subscribe();

    return subject.asObservable();
  }

  // Controllare la validità della cache
  private isCacheValid(id: string): boolean {
    const expiry = this.cacheExpiry.get(id);
    return !!expiry && Date.now() < expiry && this.cache.has(id);
  }

  // Simulazione di chiamata API
  private fetchUser(id: string): Observable<User> {
    return timer(500).pipe(
      switchMap(() => of({
        id,
        name: `Utente ${id}`,
        email: `user${id}@example.com`
      }))
    );
  }

  // Invalidare la cache
  invalidateCache(id?: string): void {
    if (id) {
      this.cache.delete(id);
      this.cacheExpiry.delete(id);
    } else {
      this.cache.clear();
      this.cacheExpiry.clear();
    }
  }
}

// Esempio di utilizzo
const userService = new UserApiService();

// Prima richiesta (cache miss)
userService.getUser('1').subscribe(user => {
  console.log('Prima richiesta:', user);
});

// Seconda richiesta (cache hit)
setTimeout(() => {
  userService.getUser('1').subscribe(user => {
    console.log('Seconda richiesta:', user);
  });
}, 1000);
```

#### Risultato dell'esecuzione
```
Cache miss: recupero utente 1 dall'API
Prima richiesta: {id: '1', name: 'Utente 1', email: 'user1@example.com'}
Cache hit: utente 1
Seconda richiesta: {id: '1', name: 'Utente 1', email: 'user1@example.com'}
```

## Gestione dei form

BehaviorSubject è adatto per la gestione del valore e dello stato del form.

### Form reattivo

```ts
import { BehaviorSubject, combineLatest, Observable, map } from 'rxjs';

// Definizione dei campi del form
interface FormFields {
  username: string;
  email: string;
  password: string;
}

// Stato del form
interface FormState {
  values: FormFields;
  touched: Record<keyof FormFields, boolean>;
  errors: Record<keyof FormFields, string | null>;
  isValid: boolean;
  isSubmitting: boolean;
}

// Gestione del form reattivo
class ReactiveForm {
  private state$: BehaviorSubject<FormState>;

  constructor(initialValues: FormFields) {
    this.state$ = new BehaviorSubject<FormState>({
      values: initialValues,
      touched: { username: false, email: false, password: false },
      errors: { username: null, email: null, password: null },
      isValid: false,
      isSubmitting: false
    });
  }

  // Ottenere lo stato
  getState(): Observable<FormState> {
    return this.state$.asObservable();
  }

  // Aggiornare il valore di un campo
  setValue<K extends keyof FormFields>(field: K, value: FormFields[K]): void {
    const current = this.state$.getValue();
    const newValues = { ...current.values, [field]: value };
    const errors = this.validate(newValues);

    this.state$.next({
      ...current,
      values: newValues,
      errors,
      isValid: Object.values(errors).every(e => e === null)
    });
  }

  // Impostare lo stato di touched
  setTouched(field: keyof FormFields): void {
    const current = this.state$.getValue();
    this.state$.next({
      ...current,
      touched: { ...current.touched, [field]: true }
    });
  }

  // Validazione
  private validate(values: FormFields): Record<keyof FormFields, string | null> {
    return {
      username: values.username.length < 3 ? 'Il nome utente deve avere almeno 3 caratteri' : null,
      email: !values.email.includes('@') ? 'Inserisci un\'email valida' : null,
      password: values.password.length < 6 ? 'La password deve avere almeno 6 caratteri' : null
    };
  }

  // Invio del form
  async submit(): Promise<void> {
    const current = this.state$.getValue();

    if (!current.isValid) {
      // Impostare tutti i campi come touched
      this.state$.next({
        ...current,
        touched: { username: true, email: true, password: true }
      });
      return;
    }

    this.state$.next({ ...current, isSubmitting: true });

    try {
      // Simulazione dell'invio API
      await new Promise(resolve => setTimeout(resolve, 1000));
      console.log('Invio completato:', current.values);
    } finally {
      this.state$.next({
        ...this.state$.getValue(),
        isSubmitting: false
      });
    }
  }
}

// Esempio di utilizzo
const form = new ReactiveForm({
  username: '',
  email: '',
  password: ''
});

form.getState().subscribe(state => {
  console.log('Stato del form:', {
    isValid: state.isValid,
    errors: state.errors
  });
});

// Input dell'utente
form.setValue('username', 'ab'); // Troppo corto
form.setValue('username', 'admin'); // OK
form.setValue('email', 'test@example.com');
form.setValue('password', 'password123');
```

#### Risultato dell'esecuzione
```
Stato del form: {isValid: false, errors: {…}}
Stato del form: {isValid: false, errors: {…}}
Stato del form: {isValid: false, errors: {…}}
Stato del form: {isValid: false, errors: {…}}
Stato del form: {isValid: true, errors: {…}}
```

## Gestione dei log e della cronologia

ReplaySubject è utile per conservare la cronologia recente.

### Servizio di logging

```ts
import { ReplaySubject, Observable } from 'rxjs';
import { map, filter } from 'rxjs';

// Livelli di log
type LogLevel = 'debug' | 'info' | 'warn' | 'error';

// Voce di log
interface LogEntry {
  level: LogLevel;
  message: string;
  timestamp: Date;
  context?: Record<string, any>;
}

// Servizio di logging
class LoggingService {
  // Conserva le ultime 100 voci di log
  private logs$ = new ReplaySubject<LogEntry>(100);

  // Metodo di log generico
  private log(level: LogLevel, message: string, context?: Record<string, any>): void {
    this.logs$.next({
      level,
      message,
      timestamp: new Date(),
      context
    });
  }

  // Metodi per ogni livello
  debug(message: string, context?: Record<string, any>): void {
    this.log('debug', message, context);
  }

  info(message: string, context?: Record<string, any>): void {
    this.log('info', message, context);
  }

  warn(message: string, context?: Record<string, any>): void {
    this.log('warn', message, context);
  }

  error(message: string, context?: Record<string, any>): void {
    this.log('error', message, context);
  }

  // Ottenere tutti i log
  getLogs(): Observable<LogEntry> {
    return this.logs$.asObservable();
  }

  // Filtrare per livello
  getLogsByLevel(level: LogLevel): Observable<LogEntry> {
    return this.logs$.pipe(
      filter(log => log.level === level)
    );
  }

  // Ottenere solo gli errori recenti
  getRecentErrors(): Observable<LogEntry[]> {
    const errors: LogEntry[] = [];
    return new Observable<LogEntry[]>(subscriber => {
      this.logs$.pipe(
        filter(log => log.level === 'error')
      ).subscribe(log => {
        errors.push(log);
        subscriber.next([...errors]);
      });
    });
  }
}

// Esempio di utilizzo
const logger = new LoggingService();

// Sottoscrivere ai log degli errori
logger.getLogsByLevel('error').subscribe(log => {
  console.log('ERRORE:', log.message);
});

// Generare log
logger.info('Applicazione avviata');
logger.debug('Inizializzazione della configurazione', { env: 'production' });
logger.warn('Utilizzo memoria elevato');
logger.error('Connessione al database fallita', { host: 'localhost', port: 5432 });

// Un nuovo sottoscrittore riceve la cronologia passata
setTimeout(() => {
  console.log('--- Nuovo sottoscrittore ---');
  logger.getLogs().subscribe(log => {
    console.log(`[${log.level}] ${log.message}`);
  });
}, 100);
```

#### Risultato dell'esecuzione
```
ERRORE: Connessione al database fallita
--- Nuovo sottoscrittore ---
[info] Applicazione avviata
[debug] Inizializzazione della configurazione
[warn] Utilizzo memoria elevato
[error] Connessione al database fallita
```

## Gestione delle code asincrone

La combinazione di Subject e BehaviorSubject è utile per la gestione delle code di attività asincrone.

### Coda di attività

```ts
import { Subject, BehaviorSubject, Observable, from, concatMap, tap, catchError, of, finalize } from 'rxjs';

// Definizione dell'attività
interface Task {
  id: string;
  name: string;
  execute: () => Promise<any>;
}

// Risultato dell'attività
interface TaskResult {
  taskId: string;
  success: boolean;
  result?: any;
  error?: any;
}

// Gestione della coda di attività
class TaskQueue {
  private queue$ = new Subject<Task>();
  private processing$ = new BehaviorSubject<boolean>(false);
  private results$ = new Subject<TaskResult>();

  constructor() {
    this.processQueue();
  }

  // Elaborazione della coda
  private processQueue(): void {
    this.queue$.pipe(
      tap(() => this.processing$.next(true)),
      concatMap(task =>
        from(task.execute()).pipe(
          tap(result => {
            this.results$.next({
              taskId: task.id,
              success: true,
              result
            });
          }),
          catchError(error => {
            this.results$.next({
              taskId: task.id,
              success: false,
              error
            });
            return of(null);
          }),
          finalize(() => {
            // Controllare le attività rimanenti
            // (omesso nella versione semplificata)
          })
        )
      )
    ).subscribe({
      complete: () => this.processing$.next(false)
    });
  }

  // Aggiungere un'attività
  enqueue(task: Task): void {
    console.log(`Attività aggiunta: ${task.name}`);
    this.queue$.next(task);
  }

  // Ottenere lo stato di elaborazione
  isProcessing(): Observable<boolean> {
    return this.processing$.asObservable();
  }

  // Ottenere i risultati
  getResults(): Observable<TaskResult> {
    return this.results$.asObservable();
  }
}

// Esempio di utilizzo
const taskQueue = new TaskQueue();

// Monitorare i risultati
taskQueue.getResults().subscribe(result => {
  console.log('Risultato attività:', result);
});

// Aggiungere attività
taskQueue.enqueue({
  id: '1',
  name: 'Recupera dati utente',
  execute: async () => {
    await new Promise(resolve => setTimeout(resolve, 500));
    return { user: 'admin' };
  }
});

taskQueue.enqueue({
  id: '2',
  name: 'Aggiorna cache',
  execute: async () => {
    await new Promise(resolve => setTimeout(resolve, 300));
    return { cached: true };
  }
});

taskQueue.enqueue({
  id: '3',
  name: 'Invia notifica',
  execute: async () => {
    await new Promise(resolve => setTimeout(resolve, 200));
    throw new Error('Invio notifica fallito');
  }
});
```

#### Risultato dell'esecuzione
```
Attività aggiunta: Recupera dati utente
Attività aggiunta: Aggiorna cache
Attività aggiunta: Invia notifica
Risultato attività: {taskId: '1', success: true, result: {…}}
Risultato attività: {taskId: '2', success: true, result: {…}}
Risultato attività: {taskId: '3', success: false, error: Error: Invio notifica fallito…}
```

## Gestione degli stream WebSocket

Subject è utile per avvolgere le comunicazioni bidirezionali come WebSocket.

### Wrapper WebSocket

```ts
import { Subject, Observable, BehaviorSubject, timer, retry, tap } from 'rxjs';

// Tipi di messaggio
interface WebSocketMessage {
  type: string;
  payload: any;
}

// Stato della connessione
type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error';

// Wrapper WebSocket
class WebSocketWrapper {
  private socket: WebSocket | null = null;
  private messages$ = new Subject<WebSocketMessage>();
  private connectionState$ = new BehaviorSubject<ConnectionState>('disconnected');
  private outgoing$ = new Subject<WebSocketMessage>();

  constructor(private url: string) {}

  // Connetti
  connect(): Observable<ConnectionState> {
    if (this.socket) {
      return this.connectionState$.asObservable();
    }

    this.connectionState$.next('connecting');

    this.socket = new WebSocket(this.url);

    this.socket.onopen = () => {
      this.connectionState$.next('connected');
      console.log('WebSocket connesso');
    };

    this.socket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        this.messages$.next(message);
      } catch (e) {
        console.error('Errore di parsing del messaggio:', e);
      }
    };

    this.socket.onerror = () => {
      this.connectionState$.next('error');
    };

    this.socket.onclose = () => {
      this.connectionState$.next('disconnected');
      this.socket = null;
    };

    // Gestione dei messaggi in uscita
    this.outgoing$.subscribe(message => {
      if (this.socket?.readyState === WebSocket.OPEN) {
        this.socket.send(JSON.stringify(message));
      }
    });

    return this.connectionState$.asObservable();
  }

  // Disconnetti
  disconnect(): void {
    this.socket?.close();
    this.socket = null;
  }

  // Invia un messaggio
  send(message: WebSocketMessage): void {
    this.outgoing$.next(message);
  }

  // Ricevi messaggi
  getMessages(): Observable<WebSocketMessage> {
    return this.messages$.asObservable();
  }

  // Ricevi messaggi di un tipo specifico
  getMessagesByType(type: string): Observable<WebSocketMessage> {
    return this.messages$.pipe(
      // @ts-ignore
      filter(msg => msg.type === type)
    );
  }

  // Ottenere lo stato della connessione
  getConnectionState(): Observable<ConnectionState> {
    return this.connectionState$.asObservable();
  }
}

// Esempio di utilizzo (pseudocodice - richiede un server WebSocket reale)
/*
const ws = new WebSocketWrapper('wss://example.com/socket');

// Monitorare lo stato della connessione
ws.getConnectionState().subscribe(state => {
  console.log('Stato della connessione:', state);
});

// Ricevere messaggi
ws.getMessages().subscribe(message => {
  console.log('Messaggio ricevuto:', message);
});

// Connetti
ws.connect();

// Inviare un messaggio
ws.send({ type: 'chat', payload: { text: 'Ciao!' } });
*/
```

## Guida alla selezione dei Subject

Infine, riassumiamo i criteri di selezione di ogni Subject.

| Caso d'uso | Subject consigliato | Motivo |
|---|---|---|
| Notifica di eventi | `Subject` | Non è necessario conservare i valori precedenti |
| Gestione dello stato | `BehaviorSubject` | È necessario conservare lo stato corrente |
| Cronologia/Registro | `ReplaySubject` | È necessario conservare i valori passati |
| Cache dei risultati API | `AsyncSubject` | È necessario solo il risultato finale |
| Form reattivo | `BehaviorSubject` | È necessario tenere traccia del valore corrente |
| Event Bus | `Subject` | Notifica di eventi semplice |
| WebSocket | `Subject` | Per lo streaming di dati bidirezionale |

## Riepilogo

In questa sezione abbiamo presentato casi d'uso pratici dei Subject, tra cui:

1. **Gestione dello stato** - Store semplice e pattern Redux-like con BehaviorSubject
2. **Comunicazione tra componenti** - Event Bus e Pub/Sub con Subject
3. **Caching dei dati API** - Servizio API con cache usando AsyncSubject
4. **Gestione dei form** - Form reattivo con BehaviorSubject
5. **Gestione dei log** - Servizio di logging con ReplaySubject
6. **Coda di attività asincrone** - Gestione della coda con Subject e BehaviorSubject
7. **Gestione WebSocket** - Wrapper per streaming bidirezionale con Subject

L'utilizzo appropriato dei Subject può migliorare significativamente la manutenibilità e l'efficienza delle applicazioni reattive.

## 🔗 Sezioni correlate

- **[Cos'è un Subject](./what-is-subject)** - Concetti base e caratteristiche dei Subject
- **[Tipi di Subject](./types-of-subject)** - BehaviorSubject, ReplaySubject, AsyncSubject, ecc.
- **[Come funziona il multicasting](./multicasting)** - Pattern di condivisione degli stream
