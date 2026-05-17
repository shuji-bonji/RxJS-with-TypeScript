---
description: State Management, Event Bus, globales Benachrichtigungssystem, Daten-Caching, reaktive Formularverwaltung und andere praktische Anwendungsfälle unter Verwendung der Subject-Familie (Subject, BehaviorSubject, ReplaySubject, AsyncSubject) werden mit umfangreichen TypeScript-Codebeispielen erklärt. Verstehen Sie die Eigenschaften jedes Subject-Typs und lernen Sie, wie Sie sie in geeigneten Situationen einsetzen können.
---

# Anwendungsfälle von Subject

RxJS Subject kann in verschiedenen praktischen Szenarien eingesetzt werden. Hier werden praktische Verwendungsbeispiele der Subject-Familie (Subject, BehaviorSubject, ReplaySubject, AsyncSubject) vorgestellt und erklärt, für welche Situationen jeweils welcher Typ am besten geeignet ist.

## State Management Patterns

### Implementierung eines einfachen Stores

Mit `BehaviorSubject` kann ein einfacher Store implementiert werden, der den Zustand der Anwendung speichert, aktualisiert und abonniert.

```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// Anfangszustand
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // Zustandsverwaltung mit BehaviorSubject
  private state$ = new BehaviorSubject<AppState>(initialState);

  // Methode zum Lesen des Zustands
  getState() {
    return this.state$.getValue();
  }

  // Bestimmte Eigenschaft als Observable abrufen
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }

  // Zustand aktualisieren
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }

  // Zustand als Observable veröffentlichen
  get state() {
    return this.state$.asObservable();
  }
}

// Verwendungsbeispiel
const store = new Store();

// Zustand überwachen
store.select('user').subscribe(user => {
  console.log('Benutzerzustandsänderung:', user?.name, user?.role);
});

// Theme-Änderung überwachen
store.select('theme').subscribe(theme => {
  console.log('Theme-Änderung:', theme);
  document.body.className = theme; // In UI reflektieren
});

// Zustand aktualisieren
store.setState({ user: { name: 'Taro Yamada', role: 'admin' } });
store.setState({ theme: 'dark' });
```

#### Ausführungsergebnis
```sh
Benutzerzustandsänderung: undefined undefined
Theme-Änderung: light
Benutzerzustandsänderung: Taro Yamada admin
Theme-Änderung: light
Benutzerzustandsänderung: Taro Yamada admin
Theme-Änderung: dark
```

Dieses Pattern ist nützlich für kleine Anwendungen oder wenn große State-Management-Bibliotheken wie NgRx oder Redux nicht verwendet werden.

## Kommunikation zwischen Komponenten

### Implementierung eines Event Bus

Implementierung eines `Subject`-basierten Event Bus, der verschiedene Datentypen für jeden Benachrichtigungstyp verarbeiten kann, zur Kommunikation zwischen Komponenten.

```ts
import { Subject } from 'rxjs';
import { filter, map } from 'rxjs';

type EventPayloadMap = {
  USER_LOGIN: { username: string; timestamp: number };
  DATA_UPDATED: any;
  NOTIFICATION: string;
};

// Event-Typ-Definition
type EventType = keyof EventPayloadMap;

interface AppEvent<K extends EventType> {
  type: K;
  payload: EventPayloadMap[K];
}

// Event-Bus-Service
class EventBusService {
  private eventSubject = new Subject<AppEvent<unknown>>();

  emit<K extends EventType>(type: K, payload: EventPayloadMap[K]): void {
    this.eventSubject.next({ type, payload });
  }

  // Bestimmten Event-Typ abonnieren
  on<K extends EventType>(type: K) {
    return this.eventSubject.pipe(
      filter((event): event is AppEvent<K> => event.type === type),
      map((event) => event.payload)
    );
  }
}
// Verwendungsbeispiel) Kommunikation zwischen Komponenten
const eventBus = new EventBusService();

// Header-Komponente (Benachrichtigungen anzeigen)
eventBus.on('NOTIFICATION').subscribe((message) => {
  console.log('Header: Benachrichtigung anzeigen:', message);
});

// Benutzerkomponente (Login-Status überwachen)
eventBus.on('USER_LOGIN').subscribe((user) => {
  console.log('Benutzerkomponente: Login erkannt:', user.username);
});

// Einstellungskomponente (Datenaktualisierung überwachen)
eventBus.on('DATA_UPDATED').subscribe((data) => {
  console.log('Einstellungskomponente: Datenaktualisierung:', data);
});

// Events veröffentlichen
eventBus.emit('USER_LOGIN', { username: 'user123', timestamp: Date.now() });
eventBus.emit('NOTIFICATION', 'Sie haben eine neue Nachricht');
```

#### Ausführungsergebnis
```sh
Benutzerkomponente: Login erkannt: user123
Header: Benachrichtigung anzeigen: Sie haben eine neue Nachricht
```

Das Event-Bus-Pattern ist eine ausgezeichnete Methode zur Realisierung locker gekoppelter Kommunikation zwischen Komponenten. Besonders geeignet für Kommunikation zwischen Komponenten, die hierarchisch weit voneinander entfernt sind.

> [!CAUTION]
> 💡 In realen Anwendungen kann das Nichtdurchführen von Abonnement-Beendigungen (`unsubscribe()`) zu Speicherlecks führen. Erwägen Sie auch Beendigungsverarbeitungen mit `takeUntil()` usw.

## API-Daten-Caching

### Teilen und Cachen von Request-Ergebnissen

Mit `AsyncSubject` kann das Teilen und Cachen von Daten realisiert werden, die nur einmal wie bei HTTP-Anfragen veröffentlicht werden.

```ts
import { Observable, AsyncSubject, of, throwError } from 'rxjs';
import { tap, catchError, delay } from 'rxjs';

class ApiCacheService {
  private cache = new Map<string, AsyncSubject<any>>();

  fetchData<T>(url: string): Observable<T> {
    // Wenn im Cache vorhanden, zurückgeben
    if (this.cache.has(url)) {
      console.log(`Daten aus Cache abrufen: ${url}`);
      return this.cache.get(url)!.asObservable() as Observable<T>;
    }

    // Wenn nicht im Cache, neue Anfrage erstellen
    console.log(`API-Anfrage ausführen: ${url}`);
    const subject = new AsyncSubject<T>();
    this.cache.set(url, subject);

    // API-Anfrage simulieren
    this.makeRequest<T>(url)
      .pipe(
        tap((data) => {
          subject.next(data);
          subject.complete();
        }),
        catchError((error: unknown) => {
          // Bei Fehler aus Cache entfernen
          this.cache.delete(url);
          subject.error(error);
          return throwError(() => error);
        })
      )
      .subscribe();

    return subject.asObservable();
  }

  // Tatsächliche API-Anfrageverarbeitung
  private makeRequest<T>(url: string): Observable<T> {
    // In realen Anwendungen fetch oder HTTP-Client verwenden
    return of({
      data: 'Beispieldaten',
      timestamp: Date.now(),
    } as unknown as T).pipe(
      tap(() => console.log('API-Antwort empfangen')),
      // Zufällige Verzögerung simulieren
      delay(Math.random() * 1000 + 500)
    );
  }

  // Cache löschen
  clearCache(url?: string): void {
    if (url) {
      this.cache.delete(url);
    } else {
      this.cache.clear();
    }
    console.log('Cache wurde gelöscht');
  }
}

// Verwendungsbeispiel
const apiCache = new ApiCacheService();

// Mehrere Komponenten fordern dieselben API-Daten an
apiCache.fetchData('/api/products').subscribe((data) => {
  console.log('Komponente 1: Daten empfangen', data);
});

// Etwas später fordert eine andere Komponente dieselben Daten an (aus Cache abrufen)
setTimeout(() => {
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Komponente 2: Daten empfangen', data);
  });
}, 1000);

// Nach Cache-Leerung erneut anfordern
setTimeout(() => {
  apiCache.clearCache();
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Komponente 3: Daten empfangen (nach Cache-Leerung)', data);
  });
}, 2000);
```

#### Ausführungsergebnis
```sh
API-Anfrage ausführen: /api/products
API-Antwort empfangen
Komponente 1: Daten empfangen {data: 'Beispieldaten', timestamp: 1745405703582}
Daten aus Cache abrufen: /api/products
Komponente 2: Daten empfangen {data: 'Beispieldaten', timestamp: 1745405703582}
Cache wurde gelöscht
API-Anfrage ausführen: /api/products
API-Antwort empfangen
Komponente 3: Daten empfangen (nach Cache-Leerung) {data: 'Beispieldaten', timestamp: 1745405705585}
```

Dieses Pattern mit AsyncSubject ist optimal für API-Anfragen, bei denen nur der Wert beim Abschluss wichtig ist. Es verhindert auch doppelte Ausführungen derselben Anfrage.

> [!TIP]
> 💡 Wenn bei `AsyncSubject` `error()` aufgerufen wird, wird kein Wert veröffentlicht, sondern nur der `error` benachrichtigt, daher Vorsicht geboten.


## Formularverwaltung

Mit `BehaviorSubject` werden der aktuelle Wert und der Validierungsstatus reaktiver Formulare verwaltet.
### Bidirektionale Bindung von Formularwerten

```ts
import { BehaviorSubject } from 'rxjs';
import { debounceTime, distinctUntilChanged } from 'rxjs';

interface UserForm {
  name: string;
  email: string;
  age: number;
}

class ReactiveForm {
  // BehaviorSubject mit Anfangswert
  private formSubject = new BehaviorSubject<UserForm>({
    name: '',
    email: '',
    age: 0
  });

  // Observable zur Veröffentlichung
  formValues$ = this.formSubject.asObservable();

  // Validierungsergebnis
  private validSubject = new BehaviorSubject<boolean>(false);
  valid$ = this.validSubject.asObservable();

  constructor() {
    // Validierung bei Wertänderung ausführen
    this.formValues$.pipe(
      debounceTime(300),
      distinctUntilChanged((prev, curr) => JSON.stringify(prev) === JSON.stringify(curr))
    ).subscribe(form => {
      this.validateForm(form);
    });
  }

  // Feldwert aktualisieren
  updateField<K extends keyof UserForm>(field: K, value: UserForm[K]) {
    const currentForm = this.formSubject.getValue();
    this.formSubject.next({
      ...currentForm,
      [field]: value
    });
  }

  // Formular abrufen
  getForm(): UserForm {
    return this.formSubject.getValue();
  }

  // Validierung
  private validateForm(form: UserForm) {
    const isValid =
      form.name.length > 0 &&
      form.email.includes('@') &&
      form.age > 0;

    this.validSubject.next(isValid);
  }

  // Formular absenden
  submit() {
    if (this.validSubject.getValue()) {
      console.log('Formular absenden:', this.getForm());
      // API-Anfrage usw.
    } else {
      console.error('Formular ist ungültig');
    }
  }
}

// Verwendungsbeispiel
const form = new ReactiveForm();

// Formularwerte überwachen
form.formValues$.subscribe(values => {
  console.log('Formularwertänderung:', values);
  // UI-Aktualisierungsverarbeitung usw.
});

// Validierungsstatus überwachen
form.valid$.subscribe(isValid => {
  console.log('Formulargültigkeit:', isValid);
  // Senden-Button aktivieren/deaktivieren usw.
});

// Benutzereingabe simulieren
form.updateField('name', 'Taro Yamada');
form.updateField('email', 'yamada@example.com');
form.updateField('age', 30);

// Formular absenden
form.submit();
```

#### Ausführungsergebnis
```sh
Formularwertänderung: {name: '', email: '', age: 0}
Formulargültigkeit: false
Formularwertänderung: {name: 'Taro Yamada', email: '', age: 0}
Formularwertänderung: {name: 'Taro Yamada', email: 'yamada@example.com', age: 0}
Formularwertänderung: {name: 'Taro Yamada', email: 'yamada@example.com', age: 30}
Formular ist ungültig
submit @
(anonym) @ Diesen Fehler analysieren
Formulargültigkeit: true
```


Dieses Pattern ist besonders nützlich für die Implementierung reaktiver Formulare. Da BehaviorSubject den aktuellen Wert immer speichert, ist es optimal für die Zustandsverwaltung von Formularen.

## Logging und Historie

Mit `ReplaySubject` kann eine Log-Management-Struktur aufgebaut werden, die vergangene Operationshistorien speichert und erneut anzeigt.
### Verwaltung der Operationshistorie

```ts
import { Observable, ReplaySubject } from 'rxjs';
import { tap } from 'rxjs';

interface LogEntry {
  action: string;
  timestamp: number;
  data?: any;
}

class ActivityLogger {
  // Speichert die letzten 10 Logs
  private logSubject = new ReplaySubject<LogEntry>(10);
  logs$ = this.logSubject.asObservable();

  // Log-Eintrag hinzufügen
  log(action: string, data?: any) {
    const entry: LogEntry = {
      action,
      timestamp: Date.now(),
      data
    };

    this.logSubject.next(entry);
    console.log(`Log aufgezeichnet: ${action}`, data);
  }

  // Anderes Observable wrappen und Log aufzeichnen
  wrapWithLogging<T>(source$: Observable<T>, actionName: string): Observable<T> {
    return source$.pipe(
      tap(data => this.log(actionName, data))
    );
  }
}

// Verwendungsbeispiel
const logger = new ActivityLogger();

// Logs überwachen (in UI anzeigen usw.)
logger.logs$.subscribe(log => {
  const time = new Date(log.timestamp).toLocaleTimeString();
  console.log(`[${time}] ${log.action}`);
});

// Verschiedene Operationen im Log aufzeichnen
logger.log('Anwendungsstart');
logger.log('Benutzer-Login', { userId: 'user123' });

// Etwas später startet eine neue Komponente das Abonnement einschließlich vergangener Logs
setTimeout(() => {
  console.log('--- Historie-Viewer zeigt vergangene Logs einschließlich an ---');
  logger.logs$.subscribe(log => {
    const time = new Date(log.timestamp).toLocaleTimeString();
    console.log(`Historie: [${time}] ${log.action}`);
  });

  // Weitere Logs hinzufügen
  logger.log('Datenaktualisierung', { itemId: 456 });
}, 1000);
```
#### Ausführungsergebnis
```sh
[19:58:40] Anwendungsstart
Log aufgezeichnet: Anwendungsstart undefined
[19:58:40] Benutzer-Login
Log aufgezeichnet: Benutzer-Login {userId: 'user123'}
--- Historie-Viewer zeigt vergangene Logs einschließlich an ---
Historie: [19:58:40] Anwendungsstart
Historie: [19:58:40] Benutzer-Login
[19:58:41] Datenaktualisierung
Historie: [19:58:41] Datenaktualisierung
Log aufgezeichnet: Datenaktualisierung {itemId: 456}
```

Mit ReplaySubject können neuen Abonnenten auch vergangene Log-Einträge bereitgestellt werden, was es optimal für Historieverwaltung macht. Hilfreich für Verfolgung von Benutzeroperationen und Sammlung von Debug-Informationen.

> [!IMPORTANT]
> ⚠️ Wenn bei `ReplaySubject` keine Puffergröße angegeben wird, werden alle Werte dauerhaft im Speicher gehalten, daher ist bei großen Datenmengen oder lange laufenden Anwendungen Vorsicht geboten.

## Verwaltung asynchroner Verarbeitung

Mit `Subject` und `BehaviorSubject` werden Fortschrittsstatus und Aktivzustand mehrerer Tasks in Echtzeit verwaltet.
### Fortschrittsverwaltung für langandauernde Tasks

```ts
import { Subject, BehaviorSubject } from 'rxjs';

interface TaskProgress {
  taskId: string;
  progress: number; // 0-100
  status: 'pending' | 'running' | 'completed' | 'error';
  message?: string;
}

class TaskManager {
  // Task-Fortschrittsbenachrichtigung
  private progressSubject = new Subject<TaskProgress>();
  progress$ = this.progressSubject.asObservable();

  // Aktuell laufende Tasks
  private activeTasksSubject = new BehaviorSubject<string[]>([]);
  activeTasks$ = this.activeTasksSubject.asObservable();

  // Task starten
  startTask(taskId: string, taskFn: (update: (progress: number) => void) => Promise<any>) {
    // Zur Liste aktiver Tasks hinzufügen
    const currentTasks = this.activeTasksSubject.getValue();
    this.activeTasksSubject.next([...currentTasks, taskId]);

    // Anfangsfortschrittsbenachrichtigung
    this.progressSubject.next({
      taskId,
      progress: 0,
      status: 'running'
    });

    // Funktion zur Fortschrittsaktualisierung
    const updateProgress = (progress: number) => {
      this.progressSubject.next({
        taskId,
        progress,
        status: 'running'
      });
    };

    // Task ausführen
    return taskFn(updateProgress)
      .then(result => {
        // Abschlussbenachrichtigung
        this.progressSubject.next({
          taskId,
          progress: 100,
          status: 'completed'
        });
        return result;
      })
      .catch(error => {
        // Fehlerbenachrichtigung
        this.progressSubject.next({
          taskId,
          progress: 0,
          status: 'error',
          message: error.message
        });
        throw error;
      })
      .finally(() => {
        // Aus Liste aktiver Tasks entfernen
        const tasks = this.activeTasksSubject.getValue();
        this.activeTasksSubject.next(tasks.filter(id => id !== taskId));
      });
  }
}

// Verwendungsbeispiel
const taskManager = new TaskManager();

// Fortschritt in Fortschrittsbalken-UI usw. anzeigen
taskManager.progress$.subscribe(progress => {
  console.log(`Task ${progress.taskId}: ${progress.progress}% - ${progress.status}`);

  // UI-Aktualisierungscode
  // progressBar.setValue(progress.progress);
  // statusLabel.setText(progress.status);
});

// Anzahl aktiver Tasks anzeigen
taskManager.activeTasks$.subscribe(tasks => {
  console.log(`Anzahl laufender Tasks: ${tasks.length}`);
});

// Langandauernden Task simulieren
taskManager.startTask('file-upload', (update) => {
  return new Promise((resolve) => {
    let progress = 0;

    // Fortschritt simulieren
    const interval = setInterval(() => {
      progress += 10;
      update(progress);

      if (progress >= 100) {
        clearInterval(interval);
        resolve('Upload abgeschlossen');
      }
    }, 500);
  });
});
```

#### Ausführungsergebnis
```sh
Anzahl laufender Tasks: 0
Anzahl laufender Tasks: 1
Task file-upload: 0% - running
Task file-upload: 10% - running
Task file-upload: 20% - running
Task file-upload: 30% - running
Task file-upload: 40% - running
Task file-upload: 50% - running
Task file-upload: 60% - running
Task file-upload: 70% - running
Task file-upload: 80% - running
Task file-upload: 90% - running
Task file-upload: 100% - running
Task file-upload: 100% - completed
Anzahl laufender Tasks: 0
```

Dieses Pattern verwendet Subject zur Echtzeitbenachrichtigung des Fortschrittsstatus langandauernder Tasks. Geeignet für Fortschrittsanzeigen bei Datei-Uploads, Datenverarbeitung, Hintergrundoperationen usw.

## Echtzeit-Updates

WebSocket-Verbindungsstatus, empfangene Nachrichten und Wiederverbindungssteuerung werden mit mehreren Subjects verwaltet.
### Verwaltung von WebSocket-Streams

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

  // Verbindungsstatus
  private connectionStatusSubject = new BehaviorSubject<boolean>(false);
  connectionStatus$ = this.connectionStatusSubject.asObservable();

  // Nachrichtenstream
  private messagesSubject = new Subject<WebSocketMessage>();
  messages$ = this.messagesSubject.asObservable();

  // Subject zur Verbindungsbeendigung
  private destroySubject = new Subject<void>();

  constructor(url: string) {
    this.url = url;
  }

  // WebSocket-Verbindung starten
  connect(): void {
    if (this.socket) {
      return; // Bereits verbunden
    }

    this.socket = new WebSocket(this.url);

    // Event-Handler einrichten
    this.socket.addEventListener('open', () => {
      console.log('WebSocket-Verbindung hergestellt');
      this.connectionStatusSubject.next(true);
    });

    this.socket.addEventListener('message', (event) => {
      try {
        const message = JSON.parse(event.data) as WebSocketMessage;
        this.messagesSubject.next(message);
      } catch (e) {
        console.error('Nachrichtenanalyse-Fehler:', e);
      }
    });

    this.socket.addEventListener('close', () => {
      console.log('WebSocket-Verbindung beendet');
      this.connectionStatusSubject.next(false);
      this.socket = null;

      // Automatische Wiederverbindung
      this.reconnect();
    });

    this.socket.addEventListener('error', (error) => {
      console.error('WebSocket-Fehler:', error);
      this.connectionStatusSubject.next(false);
    });
  }

  // Wiederverbindungslogik
  private reconnect(): void {
    // Wiederverbinden wenn destroy nicht aufgerufen wurde
    timer(3000)
      .pipe(takeUntil(this.destroySubject))
      .subscribe(() => {
        console.log('WebSocket-Wiederverbindungsversuch...');
        this.connect();
      });
  }

  // Nachricht senden
  send(type: string, data: any): void {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      const message: WebSocketMessage = { type, data };
      this.socket.send(JSON.stringify(message));
    } else {
      console.error('WebSocket ist nicht verbunden');
    }
  }

  // Nur Nachrichten eines bestimmten Typs abrufen
  getMessagesOfType<T>(type: string): Observable<T> {
    return this.messages$.pipe(
      filter((msg) => msg.type === type),
      map((msg) => msg.data as T)
    );
  }

  // Verbindung trennen
  disconnect(): void {
    this.destroySubject.next();
    this.destroySubject.complete();

    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
  }
}

// Verwendungsbeispiel
const wsService = new WebSocketService('wss://echo.websocket.org');

// Verbindungsstatus überwachen
wsService.connectionStatus$.subscribe((isConnected) => {
  console.log('Verbindungsstatus:', isConnected ? 'Online' : 'Offline');
  // UI-Aktualisierung usw.
});

// Alle Nachrichten überwachen
wsService.messages$.subscribe((message) => {
  console.log('Empfangene Nachricht:', message);
});

// Nur Nachrichten eines bestimmten Typs überwachen
wsService
  .getMessagesOfType<{ price: number }>('stock-update')
  .subscribe((stockData) => {
    console.log(`Aktienkurs-Update: ${stockData.price}`);
  });

// Verbindung starten
wsService.connect();

// Nachricht senden
setTimeout(() => {
  wsService.send('chat-message', { text: 'Hallo!' });
}, 1000);

// Bei Anwendungsbeendigung
// wsService.disconnect();
```

#### Ausführungsergebnis
```sh
Verbindungsstatus: Offline
WebSocket-Verbindung hergestellt
Verbindungsstatus: Online
Nachrichtenanalyse-Fehler: SyntaxError: Unexpected token 'R', "Request se"... is not valid JSON
  at JSON.parse (<anonymous>)
  at WebSocket.<anonymous> (:30)
(anonym) @ Diesen Fehler analysieren
Empfangene Nachricht: {type: 'chat-message', data: {…}}
```

Dieses WebSocket-Verwaltungspattern ist optimal für Anwendungen, die Echtzeitkommunikation erfordern. Subject wird verwendet, um Verbindungsstatus und Nachrichtenfluss zu verwalten und über mehrere Komponenten zu teilen.

## Richtlinien zur Auswahl von Subject

| Anwendungsfall | Empfohlenes Subject | Erklärung |
|--------------|-------------|------|
| Event-Benachrichtigung・Kommunikation | `Subject` | Geeignet für einfache unidirektionale Kommunikation |
| Aktuellen Wert halten・State Management | `BehaviorSubject` | Anfangswert erforderlich, aktuellster Wert immer abrufbar |
| Stream mit Historie・Log | `ReplaySubject` | Kann auch vergangene Werte an Abonnenten bereitstellen |
| Endwert gesammelt bereitstellen・Antworten teilen | `AsyncSubject` | Benachrichtigt nur den letzten Wert beim Abschluss |

> 💡 Das Anhängen von `$` am Ende von Variablennamen ist eine gängige RxJS-Namenskonvention, die anzeigt, dass es sich um ein Observable handelt.

## Zusammenfassung

Die RxJS Subject-Familie ist ein leistungsstarkes Tool, das verschiedene Anwendungsfälle abdeckt.

- **BehaviorSubject**: State Management, Formularverwaltung, Anzeige aktueller Werte
- **Subject**: Event-Benachrichtigung, Kommunikation zwischen Komponenten
- **ReplaySubject**: Historieverwaltung, Operationslogs, verspätet beitretende Komponenten
- **AsyncSubject**: Caching von API-Antworten, Teilen von Berechnungsergebnissen

Durch angemessene Kombination dieser Patterns können reaktive, wartbare Anwendungen erstellt werden. Besonders wichtig ist es, Speicherlecks zu vermeiden, indem Abonnements zur richtigen Zeit beendet werden.
