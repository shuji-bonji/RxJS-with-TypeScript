---
description: Practical use cases such as state management utilizing the Subject family (Subject, BehaviorSubject, ReplaySubject, AsyncSubject), event bus, global notification system, data cache, and reactive form management are explained with TypeScript code examples. You will understand the characteristics of each Subject type and be able to utilize them in appropriate situations.
---

# Use Cases for Subject

RxJS's Subject can be used in a variety of practical scenarios. This section introduces practical use cases for the Subject family (Subject, BehaviorSubject, ReplaySubject, and AsyncSubject) and explains when each is best suited.

## State Management Pattern

### Simple Store Implementation

Use `BehaviorSubject` to implement a simple store that can hold, update, and subscribe to application state.

```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// Initial state
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // Manage state with BehaviorSubject
  private state$ = new BehaviorSubject<AppState>(initialState);

  // Method for reading state
  getState() {
    return this.state$.getValue();
  }

  // Get specified property as Observable
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }

  // Update state
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }

  // Expose state as Observable
  get state() {
    return this.state$.asObservable();
  }
}

// Usage example
const store = new Store();

// Monitor state
store.select('user').subscribe(user => {
  console.log('User state changed:', user?.name, user?.role);
});

// Monitor theme changes
store.select('theme').subscribe(theme => {
  console.log('Theme changed:', theme);
  document.body.className = theme; // Reflect in UI
});

// Update state
store.setState({ user: { name: 'Taro Yamada', role: 'admin' } });
store.setState({ theme: 'dark' });
```

#### Execution Result
```sh
User state changed: undefined undefined
Theme changed: light
User state changed: Taro Yamada admin
Theme changed: light
User state changed: Taro Yamada admin
Theme changed: dark
```

This pattern is useful for small applications or when large state management libraries such as NgRx or Redux are not used.

## Inter-Component Communication

### Event Bus Implementation

Implement a `Subject`-based event bus that can handle different data types for each notification type for inter-component communication.

```ts
import { Subject } from 'rxjs';
import { filter, map } from 'rxjs';

type EventPayloadMap = {
  USER_LOGIN: { username: string; timestamp: number };
  DATA_UPDATED: any;
  NOTIFICATION: string;
};

// Event type definition
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

  // Subscribe to specific type of events
  on<K extends EventType>(type: K) {
    return this.eventSubject.pipe(
      filter((event): event is AppEvent<K> => event.type === type),
      map((event) => event.payload)
    );
  }
}

// Usage example) Inter-component communication
const eventBus = new EventBusService();

// Header component (display notifications)
eventBus.on('NOTIFICATION').subscribe((message) => {
  console.log('Header: Display notification:', message);
});

// User component (monitor login state)
eventBus.on('USER_LOGIN').subscribe((user) => {
  console.log('User component: Login detected:', user.username);
});

// Settings component (monitor data updates)
eventBus.on('DATA_UPDATED').subscribe((data) => {
  console.log('Settings component: Data updated:', data);
});

// Emit events
eventBus.emit('USER_LOGIN', { username: 'user123', timestamp: Date.now() });
eventBus.emit('NOTIFICATION', 'You have a new message');
```

#### Execution Result
```sh
User component: Login detected: user123
Header: Display notification: You have a new message
```

The Event Bus Pattern is an excellent way to achieve loosely coupled inter-component communication. It is particularly suited for communication between components that are far apart in the hierarchy.

> [!CAUTION]
> 💡 In real applications, not unsubscribing (`unsubscribe()`) may lead to memory leaks. Consider using `takeUntil()` or other unsubscribe processes.

## API Data Caching

### Request Result Sharing and Caching

Use `AsyncSubject` to share and cache data that is issued only once, such as HTTP requests.

```ts
import { Observable, AsyncSubject, of, throwError } from 'rxjs';
import { tap, catchError, delay } from 'rxjs';

class ApiCacheService {
  private cache = new Map<string, AsyncSubject<any>>();

  fetchData<T>(url: string): Observable<T> {
    // Return from cache if it exists
    if (this.cache.has(url)) {
      console.log(`Getting data from cache: ${url}`);
      return this.cache.get(url)!.asObservable() as Observable<T>;
    }

    // Create new request if no cache
    console.log(`Executing API request: ${url}`);
    const subject = new AsyncSubject<T>();
    this.cache.set(url, subject);

    // Simulate API request
    this.makeRequest<T>(url)
      .pipe(
        tap((data) => {
          subject.next(data);
          subject.complete();
        }),
        catchError((error) => {
          // Remove from cache on error
          this.cache.delete(url);
          subject.error(error);
          return throwError(() => error);
        })
      )
      .subscribe();

    return subject.asObservable();
  }

  // Actual API request processing
  private makeRequest<T>(url: string): Observable<T> {
    // In real apps, use fetch or HTTP client
    return of({
      data: 'Sample data',
      timestamp: Date.now(),
    } as unknown as T).pipe(
      tap(() => console.log('API response received')),
      // Simulate random delay
      delay(Math.random() * 1000 + 500)
    );
  }

  // Clear cache
  clearCache(url?: string): void {
    if (url) {
      this.cache.delete(url);
    } else {
      this.cache.clear();
    }
    console.log('Cache cleared');
  }
}

// Usage example
const apiCache = new ApiCacheService();

// Multiple components request the same API data
apiCache.fetchData('/api/products').subscribe((data) => {
  console.log('Component 1: Data received', data);
});

// Slightly later, another component requests the same data (retrieved from cache)
setTimeout(() => {
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Component 2: Data received', data);
  });
}, 1000);

// Request again after cache clear
setTimeout(() => {
  apiCache.clearCache();
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Component 3: Data received (after cache clear)', data);
  });
}, 2000);
```

#### Execution Result
```sh
Executing API request: /api/products
API response received
Component 1: Data received {data: 'Sample data', timestamp: 1745405703582}
Getting data from cache: /api/products
Component 2: Data received {data: 'Sample data', timestamp: 1745405703582}
Cache cleared
Executing API request: /api/products
API response received
Component 3: Data received (after cache clear) {data: 'Sample data', timestamp: 1745405705585}
```

This pattern with AsyncSubject is ideal for API requests where only the last value at completion is important. It also prevents duplicate issuance of the same request.

> [!TIP]
> 💡 Note that AsyncSubject does not issue a value when `error()` is called, only an error is reported.


## Form Management

Use `BehaviorSubject` to manage the current values and validation state of reactive forms.

### Bidirectional Binding of Form Values

```ts
import { BehaviorSubject } from 'rxjs';
import { debounceTime, distinctUntilChanged } from 'rxjs';

interface UserForm {
  name: string;
  email: string;
  age: number;
}

class ReactiveForm {
  // BehaviorSubject with initial value
  private formSubject = new BehaviorSubject<UserForm>({
    name: '',
    email: '',
    age: 0
  });

  // Public Observable
  formValues$ = this.formSubject.asObservable();

  // Validation result
  private validSubject = new BehaviorSubject<boolean>(false);
  valid$ = this.validSubject.asObservable();

  constructor() {
    // Execute validation on value change
    this.formValues$.pipe(
      debounceTime(300),
      distinctUntilChanged((prev, curr) => JSON.stringify(prev) === JSON.stringify(curr))
    ).subscribe(form => {
      this.validateForm(form);
    });
  }

  // Update field value
  updateField<K extends keyof UserForm>(field: K, value: UserForm[K]) {
    const currentForm = this.formSubject.getValue();
    this.formSubject.next({
      ...currentForm,
      [field]: value
    });
  }

  // Get form
  getForm(): UserForm {
    return this.formSubject.getValue();
  }

  // Validation
  private validateForm(form: UserForm) {
    const isValid =
      form.name.length > 0 &&
      form.email.includes('@') &&
      form.age > 0;

    this.validSubject.next(isValid);
  }

  // Form submission
  submit() {
    if (this.validSubject.getValue()) {
      console.log('Form submitted:', this.getForm());
      // API request, etc.
    } else {
      console.error('Form is invalid');
    }
  }
}

// Usage example
const form = new ReactiveForm();

// Monitor form values
form.formValues$.subscribe(values => {
  console.log('Form values changed:', values);
  // UI update processing, etc.
});

// Monitor validation state
form.valid$.subscribe(isValid => {
  console.log('Form validity:', isValid);
  // Enable/disable submit button, etc.
});

// Simulate user input
form.updateField('name', 'Taro Yamada');
form.updateField('email', 'yamada@example.com');
form.updateField('age', 30);

// Form submission
form.submit();
```

#### Execution Result
```sh
Form values changed: {name: '', email: '', age: 0}
Form validity: false
Form values changed: {name: 'Taro Yamada', email: '', age: 0}
Form values changed: {name: 'Taro Yamada', email: 'yamada@example.com', age: 0}
Form values changed: {name: 'Taro Yamada', email: 'yamada@example.com', age: 30}
Form is invalid
Form validity: true
```

This pattern is particularly useful for reactive form implementations. The BehaviorSubject always retains the current value, making it ideal for form state management.

## Logging and History

Use `ReplaySubject` to create a log management mechanism that can retain and redisplay the history of past operations.

### Operation History Management

```ts
import { Observable, ReplaySubject } from 'rxjs';
import { tap } from 'rxjs';

interface LogEntry {
  action: string;
  timestamp: number;
  data?: any;
}

class ActivityLogger {
  // Keep latest 10 logs
  private logSubject = new ReplaySubject<LogEntry>(10);
  logs$ = this.logSubject.asObservable();

  // Add log entry
  log(action: string, data?: any) {
    const entry: LogEntry = {
      action,
      timestamp: Date.now(),
      data
    };

    this.logSubject.next(entry);
    console.log(`Log recorded: ${action}`, data);
  }

  // Wrap another Observable to record logs
  wrapWithLogging<T>(source$: Observable<T>, actionName: string): Observable<T> {
    return source$.pipe(
      tap(data => this.log(actionName, data))
    );
  }
}

// Usage example
const logger = new ActivityLogger();

// Monitor logs (display in UI, etc.)
logger.logs$.subscribe(log => {
  const time = new Date(log.timestamp).toLocaleTimeString();
  console.log(`[${time}] ${log.action}`);
});

// Record various operations to log
logger.log('Application started');
logger.log('User logged in', { userId: 'user123' });

// Slightly later, new component starts subscription including past logs
setTimeout(() => {
  console.log('--- History viewer displays including past logs ---');
  logger.logs$.subscribe(log => {
    const time = new Date(log.timestamp).toLocaleTimeString();
    console.log(`History: [${time}] ${log.action}`);
  });

  // Add more logs
  logger.log('Data updated', { itemId: 456 });
}, 1000);
```

#### Execution Result
```sh
[19:58:40] Application started
Log recorded: Application started undefined
[19:58:40] User logged in
Log recorded: User logged in {userId: 'user123'}
--- History viewer displays including past logs ---
History: [19:58:40] Application started
History: [19:58:40] User logged in
[19:58:41] Data updated
History: [19:58:41] Data updated
Log recorded: Data updated {itemId: 456}
```

ReplaySubject allows you to provide past log entries to new subscribers, making it ideal for history management. It is useful for tracking user operations and collecting debugging information.

> [!IMPORTANT]
> ⚠️ If no buffer size is specified for `ReplaySubject`, all values will continue to be held in memory, so be careful with large amounts of data or long-running applications.

## Asynchronous Processing Management

Use `Subject` and `BehaviorSubject` to manage the progress and active state of multiple tasks in real time.

### Progress Management of Long-Running Tasks

```ts
import { Subject, BehaviorSubject } from 'rxjs';

interface TaskProgress {
  taskId: string;
  progress: number; // 0-100
  status: 'pending' | 'running' | 'completed' | 'error';
  message?: string;
}

class TaskManager {
  // Task progress notification
  private progressSubject = new Subject<TaskProgress>();
  progress$ = this.progressSubject.asObservable();

  // Currently executing tasks
  private activeTasksSubject = new BehaviorSubject<string[]>([]);
  activeTasks$ = this.activeTasksSubject.asObservable();

  // Start task
  startTask(taskId: string, taskFn: (update: (progress: number) => void) => Promise<any>) {
    // Add to active tasks list
    const currentTasks = this.activeTasksSubject.getValue();
    this.activeTasksSubject.next([...currentTasks, taskId]);

    // Initial progress notification
    this.progressSubject.next({
      taskId,
      progress: 0,
      status: 'running'
    });

    // Function for updating progress
    const updateProgress = (progress: number) => {
      this.progressSubject.next({
        taskId,
        progress,
        status: 'running'
      });
    };

    // Execute task
    return taskFn(updateProgress)
      .then(result => {
        // Completion notification
        this.progressSubject.next({
          taskId,
          progress: 100,
          status: 'completed'
        });
        return result;
      })
      .catch(error => {
        // Error notification
        this.progressSubject.next({
          taskId,
          progress: 0,
          status: 'error',
          message: error.message
        });
        throw error;
      })
      .finally(() => {
        // Remove from active tasks list
        const tasks = this.activeTasksSubject.getValue();
        this.activeTasksSubject.next(tasks.filter(id => id !== taskId));
      });
  }
}

// Usage example
const taskManager = new TaskManager();

// Display progress in progress bar UI, etc.
taskManager.progress$.subscribe(progress => {
  console.log(`Task ${progress.taskId}: ${progress.progress}% - ${progress.status}`);

  // UI update code
  // progressBar.setValue(progress.progress);
  // statusLabel.setText(progress.status);
});

// Display active task count
taskManager.activeTasks$.subscribe(tasks => {
  console.log(`Running tasks: ${tasks.length}`);
});

// Simulate long-running task
taskManager.startTask('file-upload', (update) => {
  return new Promise((resolve) => {
    let progress = 0;

    // Simulate progress
    const interval = setInterval(() => {
      progress += 10;
      update(progress);

      if (progress >= 100) {
        clearInterval(interval);
        resolve('Upload complete');
      }
    }, 500);
  });
});
```

#### Execution Result
```sh
Running tasks: 0
Running tasks: 1
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
Running tasks: 0
```

This pattern uses Subject to provide real-time notification of the progress of a long-running task. It is suitable for displaying the progress of file uploads, data processing, background operations, etc.

## Real-Time Updates

Manage WebSocket connection status, incoming messages, and reconnection control using multiple Subjects.

### WebSocket Stream Management

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

  // Connection status
  private connectionStatusSubject = new BehaviorSubject<boolean>(false);
  connectionStatus$ = this.connectionStatusSubject.asObservable();

  // Message stream
  private messagesSubject = new Subject<WebSocketMessage>();
  messages$ = this.messagesSubject.asObservable();

  // Subject for connection termination
  private destroySubject = new Subject<void>();

  constructor(url: string) {
    this.url = url;
  }

  // Start WebSocket connection
  connect(): void {
    if (this.socket) {
      return; // Already connected
    }

    this.socket = new WebSocket(this.url);

    // Set event handlers
    this.socket.addEventListener('open', () => {
      console.log('WebSocket connection established');
      this.connectionStatusSubject.next(true);
    });

    this.socket.addEventListener('message', (event) => {
      try {
        const message = JSON.parse(event.data) as WebSocketMessage;
        this.messagesSubject.next(message);
      } catch (e) {
        console.error('Message parsing error:', e);
      }
    });

    this.socket.addEventListener('close', () => {
      console.log('WebSocket connection closed');
      this.connectionStatusSubject.next(false);
      this.socket = null;

      // Auto-reconnect
      this.reconnect();
    });

    this.socket.addEventListener('error', (error) => {
      console.error('WebSocket error:', error);
      this.connectionStatusSubject.next(false);
    });
  }

  // Reconnection logic
  private reconnect(): void {
    // Reconnect if destroy has not been called
    timer(3000)
      .pipe(takeUntil(this.destroySubject))
      .subscribe(() => {
        console.log('Attempting WebSocket reconnection...');
        this.connect();
      });
  }

  // Send message
  send(type: string, data: any): void {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      const message: WebSocketMessage = { type, data };
      this.socket.send(JSON.stringify(message));
    } else {
      console.error('WebSocket is not connected');
    }
  }

  // Get only messages of specific type
  getMessagesOfType<T>(type: string): Observable<T> {
    return this.messages$.pipe(
      filter((msg) => msg.type === type),
      map((msg) => msg.data as T)
    );
  }

  // Disconnect
  disconnect(): void {
    this.destroySubject.next();
    this.destroySubject.complete();

    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
  }
}

// Usage example
const wsService = new WebSocketService('wss://echo.websocket.org');

// Monitor connection status
wsService.connectionStatus$.subscribe((isConnected) => {
  console.log('Connection status:', isConnected ? 'Online' : 'Offline');
  // UI update, etc.
});

// Monitor all messages
wsService.messages$.subscribe((message) => {
  console.log('Received message:', message);
});

// Monitor only messages of specific type
wsService
  .getMessagesOfType<{ price: number }>('stock-update')
  .subscribe((stockData) => {
    console.log(`Stock price update: ${stockData.price}`);
  });

// Start connection
wsService.connect();

// Send message
setTimeout(() => {
  wsService.send('chat-message', { text: 'Hello!' });
}, 1000);

// On application termination
// wsService.disconnect();
```

#### Execution Result
```sh
Connection status: Offline
WebSocket connection established
Connection status: Online
Message parsing error: SyntaxError: Unexpected token 'R', "Request se"... is not valid JSON
  at JSON.parse (<anonymous>)
  at WebSocket.<anonymous> (:30)
Received message: {type: 'chat-message', data: {…}}
```

This WebSocket management pattern is ideal for applications that require real-time communication. It uses Subject to manage connection state and message flow, which can be shared by multiple components.

## Guidelines for Choosing a Subject

| Use Case | Recommended Subject | Explanation |
|--------------|-------------|------|
| Event notification and communication | `Subject` | Suitable for simple one-way communication |
| Retention of current value and state management | `BehaviorSubject` | Requires an initial value, and the latest value can always be retrieved |
| Stream log with history | `ReplaySubject` | Past values can also be provided to subscribers |
| Bulk provision of final values and response sharing | `AsyncSubject` | Notifies only the last value upon completion |

> 💡 The `$` at the end of a variable name is a common RxJS naming convention to indicate that it is Observable.

## Summary

The Subject family of RxJS is a powerful tool for a variety of use cases:

- **BehaviorSubject**: State management, form management, display of current values
- **Subject**: Event notification, communication between components
- **ReplaySubject**: History management, operation log, delayed participation component
- **AsyncSubject**: Caching of API responses, sharing of calculation results

By appropriately combining these patterns, reactive and maintainable applications can be built. Be especially careful not to forget to unsubscribe at the appropriate time to prevent memory leaks.
