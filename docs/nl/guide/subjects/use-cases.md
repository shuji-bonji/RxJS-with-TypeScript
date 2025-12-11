---
description: Uitleg over state management, event busses, globale notificatiesystemen, data caching, reactive form management en andere praktische use cases met de Subject-familie (Subject, BehaviorSubject, ReplaySubject, AsyncSubject) met uitgebreide TypeScript-codevoorbeelden. Begrijp de kenmerken van elk Subject-type en leer hoe je ze in de juiste situaties kunt gebruiken.
---

# Use Cases van Subject

Subject in RxJS kan worden toegepast in verschillende praktische scenario's. Hier introduceren we praktische gebruiksvoorbeelden van de Subject-familie (Subject, BehaviorSubject, ReplaySubject, AsyncSubject) en leggen we uit welke het meest geschikt is voor welke situatie.

## State Management Patronen

### Implementatie van eenvoudige store

Met `BehaviorSubject` implementeren we een eenvoudige store die de applicatiestatus kan behouden, bijwerken en erop kan abonneren.

```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// Initiële staat
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // Beheer staat met BehaviorSubject
  private state$ = new BehaviorSubject<AppState>(initialState);

  // Methode om staat te lezen
  getState() {
    return this.state$.getValue();
  }

  // Verkrijg gespecificeerde property als Observable
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }

  // Update staat
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }

  // Publiceer staat als Observable
  get state() {
    return this.state$.asObservable();
  }
}

// Gebruiksvoorbeeld
const store = new Store();

// Monitor staat
store.select('user').subscribe(user => {
  console.log('Gebruikersstatus gewijzigd:', user?.name, user?.role);
});

// Monitor themawijziging
store.select('theme').subscribe(theme => {
  console.log('Thema gewijzigd:', theme);
  document.body.className = theme; // Reflecteer in UI
});

// Update staat
store.setState({ user: { name: 'Yamada Taro', role: 'admin' } });
store.setState({ theme: 'dark' });
```

#### Uitvoerresultaat
```sh
Gebruikersstatus gewijzigd: undefined undefined
Thema gewijzigd: light
Gebruikersstatus gewijzigd: Yamada Taro admin
Thema gewijzigd: light
Gebruikersstatus gewijzigd: Yamada Taro admin
Thema gewijzigd: dark
```

Dit patroon is handig voor kleine applicaties of wanneer je geen grote state management libraries zoals NgRx of Redux gebruikt.

## Communicatie tussen Componenten

### Implementatie van Event Bus

Implementeer een `Subject`-gebaseerde event bus die verschillende datatypes per notificatietype kan verwerken om communicatie tussen componenten te realiseren.

```ts
import { Subject } from 'rxjs';
import { filter, map } from 'rxjs';

type EventPayloadMap = {
  USER_LOGIN: { username: string; timestamp: number };
  DATA_UPDATED: any;
  NOTIFICATION: string;
};

// Event type definitie
type EventType = keyof EventPayloadMap;

interface AppEvent<K extends EventType> {
  type: K;
  payload: EventPayloadMap[K];
}

// Event bus service
class EventBusService {
  private eventSubject = new Subject<AppEvent<unknown>>();

  emit<K extends EventType>(type: K, payload: EventPayloadMap[K]): void {
    this.eventSubject.next({ type, payload });
  }

  // Abonneer op specifiek type event
  on<K extends EventType>(type: K) {
    return this.eventSubject.pipe(
      filter((event): event is AppEvent<K> => event.type === type),
      map((event) => event.payload)
    );
  }
}
// Gebruiksvoorbeeld) Communicatie tussen componenten
const eventBus = new EventBusService();

// Header component (toont notificatie)
eventBus.on('NOTIFICATION').subscribe((message) => {
  console.log('Header: toon notificatie:', message);
});

// User component (monitort login status)
eventBus.on('USER_LOGIN').subscribe((user) => {
  console.log('User component: login gedetecteerd:', user.username);
});

// Settings component (monitort data updates)
eventBus.on('DATA_UPDATED').subscribe((data) => {
  console.log('Settings component: data update:', data);
});

// Verstuur events
eventBus.emit('USER_LOGIN', { username: 'user123', timestamp: Date.now() });
eventBus.emit('NOTIFICATION', 'Je hebt nieuwe berichten');
```

#### Uitvoerresultaat
```sh
User component: login gedetecteerd: user123
Header: toon notificatie: Je hebt nieuwe berichten
```

Het event bus patroon is een uitstekende manier om losgekoppelde communicatie tussen componenten te realiseren. Het is vooral geschikt voor communicatie tussen componenten die ver van elkaar verwijderd zijn in de hiërarchie.

> [!CAUTION]
> 💡 In echte applicaties kan het niet uitvoeren van unsubscribe (`unsubscribe()`) leiden tot geheugenlekken. Overweeg ook een unsubscribe-proces met bijvoorbeeld `takeUntil()`.

## API Data Caching

### Delen en cachen van request resultaten

Met `AsyncSubject` realiseren we het delen en cachen van data die slechts één keer wordt uitgezonden, zoals HTTP-requests.

```ts
import { Observable, AsyncSubject, of, throwError } from 'rxjs';
import { tap, catchError, delay } from 'rxjs';

class ApiCacheService {
  private cache = new Map<string, AsyncSubject<any>>();

  fetchData<T>(url: string): Observable<T> {
    // Als in cache bestaat, retourneer die
    if (this.cache.has(url)) {
      console.log(`Data ophalen uit cache: ${url}`);
      return this.cache.get(url)!.asObservable() as Observable<T>;
    }

    // Als geen cache bestaat, maak nieuwe request
    console.log(`API request uitvoeren: ${url}`);
    const subject = new AsyncSubject<T>();
    this.cache.set(url, subject);

    // Simuleer API request
    this.makeRequest<T>(url)
      .pipe(
        tap((data) => {
          subject.next(data);
          subject.complete();
        }),
        catchError((error) => {
          // Bij error, verwijder uit cache
          this.cache.delete(url);
          subject.error(error);
          return throwError(() => error);
        })
      )
      .subscribe();

    return subject.asObservable();
  }

  // Werkelijke API request verwerking
  private makeRequest<T>(url: string): Observable<T> {
    // In echte app gebruik fetch of HTTP client
    return of({
      data: 'Voorbeelddata',
      timestamp: Date.now(),
    } as unknown as T).pipe(
      tap(() => console.log('API response ontvangen')),
      // Simuleer willekeurige vertraging
      delay(Math.random() * 1000 + 500)
    );
  }

  // Wis cache
  clearCache(url?: string): void {
    if (url) {
      this.cache.delete(url);
    } else {
      this.cache.clear();
    }
    console.log('Cache gewist');
  }
}

// Gebruiksvoorbeeld
const apiCache = new ApiCacheService();

// Meerdere componenten vragen dezelfde API data
apiCache.fetchData('/api/products').subscribe((data) => {
  console.log('Component 1: data ontvangen', data);
});

// Iets later vraagt ander component dezelfde data (ophalen uit cache)
setTimeout(() => {
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Component 2: data ontvangen', data);
  });
}, 1000);

// Na cache wissen opnieuw request
setTimeout(() => {
  apiCache.clearCache();
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Component 3: data ontvangen (na cache wissen)', data);
  });
}, 2000);
```

#### Uitvoerresultaat
```sh
API request uitvoeren: /api/products
API response ontvangen
Component 1: data ontvangen {data: 'Voorbeelddata', timestamp: 1745405703582}
Data ophalen uit cache: /api/products
Component 2: data ontvangen {data: 'Voorbeelddata', timestamp: 1745405703582}
Cache gewist
API request uitvoeren: /api/products
API response ontvangen
Component 3: data ontvangen (na cache wissen) {data: 'Voorbeelddata', timestamp: 1745405705585}
```

Dit patroon met AsyncSubject is optimaal voor API-requests waar alleen de laatste waarde bij voltooiing belangrijk is. Het voorkomt ook dubbele uitgifte van dezelfde request.

> [!TIP]
> 💡 Let op: als `error()` wordt aangeroepen bij `AsyncSubject`, wordt er geen waarde uitgezonden maar alleen `error`.


## Form Management

Met `BehaviorSubject` beheren we de huidige waarde en validatiestatus van reactive forms.
### Bidirectionele binding van form waarden

```ts
import { BehaviorSubject } from 'rxjs';
import { debounceTime, distinctUntilChanged } from 'rxjs';

interface UserForm {
  name: string;
  email: string;
  age: number;
}

class ReactiveForm {
  // BehaviorSubject met initiële waarde
  private formSubject = new BehaviorSubject<UserForm>({
    name: '',
    email: '',
    age: 0
  });

  // Publieke Observable
  formValues$ = this.formSubject.asObservable();

  // Validatieresultaat
  private validSubject = new BehaviorSubject<boolean>(false);
  valid$ = this.validSubject.asObservable();

  constructor() {
    // Voer validatie uit bij waardewijziging
    this.formValues$.pipe(
      debounceTime(300),
      distinctUntilChanged((prev, curr) => JSON.stringify(prev) === JSON.stringify(curr))
    ).subscribe(form => {
      this.validateForm(form);
    });
  }

  // Update veldwaarde
  updateField<K extends keyof UserForm>(field: K, value: UserForm[K]) {
    const currentForm = this.formSubject.getValue();
    this.formSubject.next({
      ...currentForm,
      [field]: value
    });
  }

  // Verkrijg form
  getForm(): UserForm {
    return this.formSubject.getValue();
  }

  // Validatie
  private validateForm(form: UserForm) {
    const isValid =
      form.name.length > 0 &&
      form.email.includes('@') &&
      form.age > 0;

    this.validSubject.next(isValid);
  }

  // Form verzenden
  submit() {
    if (this.validSubject.getValue()) {
      console.log('Form verzenden:', this.getForm());
      // API request etc.
    } else {
      console.error('Form is ongeldig');
    }
  }
}

// Gebruiksvoorbeeld
const form = new ReactiveForm();

// Monitor form waarden
form.formValues$.subscribe(values => {
  console.log('Form waarde gewijzigd:', values);
  // UI update verwerking etc.
});

// Monitor validatiestatus
form.valid$.subscribe(isValid => {
  console.log('Form geldigheid:', isValid);
  // Schakel submit knop in/uit etc.
});

// Simuleer gebruikersinvoer
form.updateField('name', 'Yamada Taro');
form.updateField('email', 'yamada@example.com');
form.updateField('age', 30);

// Form verzenden
form.submit();
```

#### Uitvoerresultaat
```sh
Form waarde gewijzigd: {name: '', email: '', age: 0}
Form geldigheid: false
Form waarde gewijzigd: {name: 'Yamada Taro', email: '', age: 0}
Form waarde gewijzigd: {name: 'Yamada Taro', email: 'yamada@example.com', age: 0}
Form waarde gewijzigd: {name: 'Yamada Taro', email: 'yamada@example.com', age: 30}
Form is ongeldig
submit @
（匿名） @ Analyseer deze fout
Form geldigheid: true
```


Dit patroon is bijzonder nuttig voor implementatie van reactive forms. BehaviorSubject behoudt altijd de huidige waarde, waardoor het optimaal is voor form state management.

## Logging en Geschiedenis

Met `ReplaySubject` bouwen we een log management mechanisme dat operationele geschiedenis kan behouden en opnieuw kan weergeven.
### Beheer van operationele geschiedenis

```ts
import { Observable, ReplaySubject } from 'rxjs';
import { tap } from 'rxjs';

interface LogEntry {
  action: string;
  timestamp: number;
  data?: any;
}

class ActivityLogger {
  // Behoud laatste 10 logs
  private logSubject = new ReplaySubject<LogEntry>(10);
  logs$ = this.logSubject.asObservable();

  // Voeg log entry toe
  log(action: string, data?: any) {
    const entry: LogEntry = {
      action,
      timestamp: Date.now(),
      data
    };

    this.logSubject.next(entry);
    console.log(`Log vastgelegd: ${action}`, data);
  }

  // Wrap andere Observable en leg log vast
  wrapWithLogging<T>(source$: Observable<T>, actionName: string): Observable<T> {
    return source$.pipe(
      tap(data => this.log(actionName, data))
    );
  }
}

// Gebruiksvoorbeeld
const logger = new ActivityLogger();

// Monitor logs (weergeven in UI etc.)
logger.logs$.subscribe(log => {
  const time = new Date(log.timestamp).toLocaleTimeString();
  console.log(`[${time}] ${log.action}`);
});

// Leg verschillende operaties vast in log
logger.log('Applicatie opgestart');
logger.log('Gebruiker ingelogd', { userId: 'user123' });

// Iets later start nieuw component abonnement inclusief eerdere logs
setTimeout(() => {
  console.log('--- Geschiedenis viewer toont eerdere logs ---');
  logger.logs$.subscribe(log => {
    const time = new Date(log.timestamp).toLocaleTimeString();
    console.log(`Geschiedenis: [${time}] ${log.action}`);
  });

  // Voeg meer logs toe
  logger.log('Data update', { itemId: 456 });
}, 1000);
```
#### Uitvoerresultaat
```sh
[19:58:40] Applicatie opgestart
Log vastgelegd: Applicatie opgestart undefined
[19:58:40] Gebruiker ingelogd
Log vastgelegd: Gebruiker ingelogd {userId: 'user123'}
--- Geschiedenis viewer toont eerdere logs ---
Geschiedenis: [19:58:40] Applicatie opgestart
Geschiedenis: [19:58:40] Gebruiker ingelogd
[19:58:41] Data update
Geschiedenis: [19:58:41] Data update
Log vastgelegd: Data update {itemId: 456}
```

Met ReplaySubject kunnen nieuwe abonnees eerdere log entries krijgen, waardoor het optimaal is voor geschiedenisbeheer. Het is nuttig voor het traceren van gebruikersoperaties en verzamelen van debug-informatie.

> [!IMPORTANT]
> ⚠️ Als je geen buffergrootte specificeert voor `ReplaySubject`, blijven alle waarden in het geheugen bewaard, dus wees voorzichtig bij grote hoeveelheden data of langlopende apps.

## Beheer van Asynchrone Verwerking

Met `Subject` en `BehaviorSubject` beheren we de voortgangsstatus en actieve staat van meerdere taken in real-time.
### Voortgangsbeheer van langlopende taken

```ts
import { Subject, BehaviorSubject } from 'rxjs';

interface TaskProgress {
  taskId: string;
  progress: number; // 0-100
  status: 'pending' | 'running' | 'completed' | 'error';
  message?: string;
}

class TaskManager {
  // Notificatie van taakvoortgang
  private progressSubject = new Subject<TaskProgress>();
  progress$ = this.progressSubject.asObservable();

  // Momenteel actieve taken
  private activeTasksSubject = new BehaviorSubject<string[]>([]);
  activeTasks$ = this.activeTasksSubject.asObservable();

  // Start taak
  startTask(taskId: string, taskFn: (update: (progress: number) => void) => Promise<any>) {
    // Voeg toe aan actieve taken lijst
    const currentTasks = this.activeTasksSubject.getValue();
    this.activeTasksSubject.next([...currentTasks, taskId]);

    // Initiële voortgangsnotificatie
    this.progressSubject.next({
      taskId,
      progress: 0,
      status: 'running'
    });

    // Functie voor voortgang update
    const updateProgress = (progress: number) => {
      this.progressSubject.next({
        taskId,
        progress,
        status: 'running'
      });
    };

    // Voer taak uit
    return taskFn(updateProgress)
      .then(result => {
        // Voltooiingsnotificatie
        this.progressSubject.next({
          taskId,
          progress: 100,
          status: 'completed'
        });
        return result;
      })
      .catch(error => {
        // Foutnotificatie
        this.progressSubject.next({
          taskId,
          progress: 0,
          status: 'error',
          message: error.message
        });
        throw error;
      })
      .finally(() => {
        // Verwijder uit actieve taken lijst
        const tasks = this.activeTasksSubject.getValue();
        this.activeTasksSubject.next(tasks.filter(id => id !== taskId));
      });
  }
}

// Gebruiksvoorbeeld
const taskManager = new TaskManager();

// Toon voortgang in voortgangsbalk UI etc.
taskManager.progress$.subscribe(progress => {
  console.log(`Taak ${progress.taskId}: ${progress.progress}% - ${progress.status}`);

  // UI update code
  // progressBar.setValue(progress.progress);
  // statusLabel.setText(progress.status);
});

// Toon aantal actieve taken
taskManager.activeTasks$.subscribe(tasks => {
  console.log(`Aantal actieve taken: ${tasks.length}`);
});

// Simulatie van langlopende taak
taskManager.startTask('file-upload', (update) => {
  return new Promise((resolve) => {
    let progress = 0;

    // Voortgang simulatie
    const interval = setInterval(() => {
      progress += 10;
      update(progress);

      if (progress >= 100) {
        clearInterval(interval);
        resolve('Upload voltooid');
      }
    }, 500);
  });
});
```

#### Uitvoerresultaat
```sh
Aantal actieve taken: 0
Aantal actieve taken: 1
Taak file-upload: 0% - running
Taak file-upload: 10% - running
Taak file-upload: 20% - running
Taak file-upload: 30% - running
Taak file-upload: 40% - running
Taak file-upload: 50% - running
Taak file-upload: 60% - running
Taak file-upload: 70% - running
Taak file-upload: 80% - running
Taak file-upload: 90% - running
Taak file-upload: 100% - running
Taak file-upload: 100% - completed
Aantal actieve taken: 0
```

Met dit patroon worden Subject gebruikt om de voortgangsstatus van langlopende taken in real-time te melden. Geschikt voor voortgangsweergave van bestandsuploads, dataverwerking, achtergrondoperaties etc.

## Real-time Updates

Beheer WebSocket-verbindingsstatus, ontvangen berichten en herverbindingscontrole met meerdere Subjects.
### Beheer van WebSocket streams

```ts
import { Subject, BehaviorSubject, timer, Observable } from 'rxjs';
import { takeUntil, filter, map } from 'rxjs';

interface WebSocketMessage {
  type: string;
  data: any;
}

class WebSocketService {
  private socket: WebSocket | null = null;
  private url: string;

  // Verbindingsstatus
  private connectionStatusSubject = new BehaviorSubject<boolean>(false);
  connectionStatus$ = this.connectionStatusSubject.asObservable();

  // Berichtenstroom
  private messagesSubject = new Subject<WebSocketMessage>();
  messages$ = this.messagesSubject.asObservable();

  // Subject voor verbinding beëindigen
  private destroySubject = new Subject<void>();

  constructor(url: string) {
    this.url = url;
  }

  // Start WebSocket verbinding
  connect(): void {
    if (this.socket) {
      return; // Al verbonden
    }

    this.socket = new WebSocket(this.url);

    // Stel event handlers in
    this.socket.addEventListener('open', () => {
      console.log('WebSocket verbinding tot stand gebracht');
      this.connectionStatusSubject.next(true);
    });

    this.socket.addEventListener('message', (event) => {
      try {
        const message = JSON.parse(event.data) as WebSocketMessage;
        this.messagesSubject.next(message);
      } catch (e) {
        console.error('Bericht parse fout:', e);
      }
    });

    this.socket.addEventListener('close', () => {
      console.log('WebSocket verbinding beëindigd');
      this.connectionStatusSubject.next(false);
      this.socket = null;

      // Automatische herverbinding
      this.reconnect();
    });

    this.socket.addEventListener('error', (error) => {
      console.error('WebSocket fout:', error);
      this.connectionStatusSubject.next(false);
    });
  }

  // Herverbinding logica
  private reconnect(): void {
    // Herverbind als destroy niet is aangeroepen
    timer(3000)
      .pipe(takeUntil(this.destroySubject))
      .subscribe(() => {
        console.log('WebSocket herverbinding poging...');
        this.connect();
      });
  }

  // Verzend bericht
  send(type: string, data: any): void {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      const message: WebSocketMessage = { type, data };
      this.socket.send(JSON.stringify(message));
    } else {
      console.error('WebSocket is niet verbonden');
    }
  }

  // Verkrijg alleen berichten van specifiek type
  getMessagesOfType<T>(type: string): Observable<T> {
    return this.messages$.pipe(
      filter((msg) => msg.type === type),
      map((msg) => msg.data as T)
    );
  }

  // Verbreek verbinding
  disconnect(): void {
    this.destroySubject.next();
    this.destroySubject.complete();

    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
  }
}

// Gebruiksvoorbeeld
const wsService = new WebSocketService('wss://echo.websocket.org');

// Monitor verbindingsstatus
wsService.connectionStatus$.subscribe((isConnected) => {
  console.log('Verbindingsstatus:', isConnected ? 'Online' : 'Offline');
  // UI update etc.
});

// Monitor alle berichten
wsService.messages$.subscribe((message) => {
  console.log('Ontvangen bericht:', message);
});

// Monitor alleen berichten van specifiek type
wsService
  .getMessagesOfType<{ price: number }>('stock-update')
  .subscribe((stockData) => {
    console.log(`Aandelenkoers update: ${stockData.price}`);
  });

// Start verbinding
wsService.connect();

// Verzend bericht
setTimeout(() => {
  wsService.send('chat-message', { text: 'Hallo!' });
}, 1000);

// Bij beëindigen van applicatie
// wsService.disconnect();
```

#### Uitvoerresultaat
```sh
Verbindingsstatus: Offline
WebSocket verbinding tot stand gebracht
Verbindingsstatus: Online
Bericht parse fout: SyntaxError: Unexpected token 'R', "Request se"... is not valid JSON
  at JSON.parse (<anonymous>)
  at WebSocket.<anonymous> (:30)
（匿名） @ Analyseer deze fout
Ontvangen bericht: {type: 'chat-message', data: {…}}
```

Dit WebSocket-beheerpatroon is optimaal voor applicaties die real-time communicatie nodig hebben. Met Subjects worden verbindingsstatus en berichtenstroom beheerd en gedeeld tussen meerdere componenten.

## Richtlijnen voor het kiezen van Subject

| Use Case | Aanbevolen Subject | Uitleg |
|--------------|-------------|------|
| Event notificatie/communicatie | `Subject` | Geschikt voor eenvoudige unidirectionele communicatie |
| Behouden huidige waarde/state management | `BehaviorSubject` | Initiële waarde nodig, laatste waarde altijd verkrijgbaar |
| Stream met geschiedenis/logs | `ReplaySubject` | Kan ook eerdere waarden aan abonnees leveren |
| Verzameld leveren laatste waarde/response delen | `AsyncSubject` | Meldt alleen laatste waarde bij voltooiing |

> 💡 Het toevoegen van `$` aan het einde van variabelenamen is een veelvoorkomende naamgevingsconventie in RxJS om aan te geven dat het een Observable is.

## Samenvatting

De Subject-familie van RxJS is een krachtig hulpmiddel dat verschillende use cases dekt:

- **BehaviorSubject**: State management, form management, weergave huidige waarde
- **Subject**: Event notificatie, communicatie tussen componenten
- **ReplaySubject**: Geschiedenisbeheer, operatie logs, late-join componenten
- **AsyncSubject**: API response caching, delen berekeningsresultaten

Door deze patronen op de juiste manier te combineren, kun je reactieve en onderhoubare applicaties bouwen. Let vooral op om geheugenlekken te voorkomen door op het juiste moment af te melden.
