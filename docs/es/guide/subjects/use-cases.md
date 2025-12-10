---
description: Casos de uso prácticos como gestión de estado utilizando la familia Subject (Subject, BehaviorSubject, ReplaySubject, AsyncSubject), event bus, sistema de notificación global, caché de datos y gestión de formularios reactivos se explican con ejemplos de código TypeScript. Comprenderá las características de cada tipo de Subject y podrá utilizarlos en situaciones apropiadas.
---

# Casos de Uso para Subject

El Subject de RxJS puede utilizarse en una variedad de escenarios prácticos. Esta sección introduce casos de uso prácticos para la familia Subject (Subject, BehaviorSubject, ReplaySubject y AsyncSubject) y explica cuándo cada uno es más adecuado.

## Patrón de Gestión de Estado

### Implementación de Store Simple

Use `BehaviorSubject` para implementar un store simple que puede mantener, actualizar y suscribirse al estado de la aplicación.

```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// Estado inicial
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // Gestionar estado con BehaviorSubject
  private state$ = new BehaviorSubject<AppState>(initialState);

  // Método para leer estado
  getState() {
    return this.state$.getValue();
  }

  // Obtener propiedad especificada como Observable
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }

  // Actualizar estado
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }

  // Exponer estado como Observable
  get state() {
    return this.state$.asObservable();
  }
}

// Ejemplo de uso
const store = new Store();

// Monitorear estado
store.select('user').subscribe(user => {
  console.log('User state changed:', user?.name, user?.role);
});

// Monitorear cambios de tema
store.select('theme').subscribe(theme => {
  console.log('Theme changed:', theme);
  document.body.className = theme; // Reflejar en UI
});

// Actualizar estado
store.setState({ user: { name: 'Taro Yamada', role: 'admin' } });
store.setState({ theme: 'dark' });
```

#### Resultado de Ejecución
```sh
User state changed: undefined undefined
Theme changed: light
User state changed: Taro Yamada admin
Theme changed: light
User state changed: Taro Yamada admin
Theme changed: dark
```

Este patrón es útil para aplicaciones pequeñas o cuando no se utilizan bibliotecas grandes de gestión de estado como NgRx o Redux.

## Comunicación entre Componentes

### Implementación de Event Bus

Implemente un event bus basado en `Subject` que pueda manejar diferentes tipos de datos para cada tipo de notificación para comunicación entre componentes.

```ts
import { Subject } from 'rxjs';
import { filter, map } from 'rxjs';

type EventPayloadMap = {
  USER_LOGIN: { username: string; timestamp: number };
  DATA_UPDATED: any;
  NOTIFICATION: string;
};

// Definición de tipo de evento
type EventType = keyof EventPayloadMap;

interface AppEvent<K extends EventType> {
  type: K;
  payload: EventPayloadMap[K];
}

// Servicio de event bus
class EventBusService {
  private eventSubject = new Subject<AppEvent<unknown>>();

  emit<K extends EventType>(type: K, payload: EventPayloadMap[K]): void {
    this.eventSubject.next({ type, payload });
  }

  // Suscribirse a tipo específico de eventos
  on<K extends EventType>(type: K) {
    return this.eventSubject.pipe(
      filter((event): event is AppEvent<K> => event.type === type),
      map((event) => event.payload)
    );
  }
}

// Ejemplo de uso) Comunicación entre componentes
const eventBus = new EventBusService();

// Componente de encabezado (mostrar notificaciones)
eventBus.on('NOTIFICATION').subscribe((message) => {
  console.log('Header: Display notification:', message);
});

// Componente de usuario (monitorear estado de inicio de sesión)
eventBus.on('USER_LOGIN').subscribe((user) => {
  console.log('User component: Login detected:', user.username);
});

// Componente de configuración (monitorear actualizaciones de datos)
eventBus.on('DATA_UPDATED').subscribe((data) => {
  console.log('Settings component: Data updated:', data);
});

// Emitir eventos
eventBus.emit('USER_LOGIN', { username: 'user123', timestamp: Date.now() });
eventBus.emit('NOTIFICATION', 'You have a new message');
```

#### Resultado de Ejecución
```sh
User component: Login detected: user123
Header: Display notification: You have a new message
```

El Patrón Event Bus es una excelente manera de lograr comunicación débilmente acoplada entre componentes. Es particularmente adecuado para comunicación entre componentes que están lejos en la jerarquía.

> [!CAUTION]
> 💡 En aplicaciones reales, no cancelar la suscripción (`unsubscribe()`) puede llevar a fugas de memoria. Considere usar `takeUntil()` u otros procesos de cancelación de suscripción.

## Caché de Datos de API

### Compartir y Cachear Resultados de Solicitud

Use `AsyncSubject` para compartir y cachear datos que se emiten solo una vez, como solicitudes HTTP.

```ts
import { Observable, AsyncSubject, of, throwError } from 'rxjs';
import { tap, catchError, delay } from 'rxjs';

class ApiCacheService {
  private cache = new Map<string, AsyncSubject<any>>();

  fetchData<T>(url: string): Observable<T> {
    // Devolver desde caché si existe
    if (this.cache.has(url)) {
      console.log(`Getting data from cache: ${url}`);
      return this.cache.get(url)!.asObservable() as Observable<T>;
    }

    // Crear nueva solicitud si no hay caché
    console.log(`Executing API request: ${url}`);
    const subject = new AsyncSubject<T>();
    this.cache.set(url, subject);

    // Simular solicitud API
    this.makeRequest<T>(url)
      .pipe(
        tap((data) => {
          subject.next(data);
          subject.complete();
        }),
        catchError((error) => {
          // Eliminar del caché en caso de error
          this.cache.delete(url);
          subject.error(error);
          return throwError(() => error);
        })
      )
      .subscribe();

    return subject.asObservable();
  }

  // Procesamiento de solicitud API real
  private makeRequest<T>(url: string): Observable<T> {
    // En aplicaciones reales, use fetch o cliente HTTP
    return of({
      data: 'Sample data',
      timestamp: Date.now(),
    } as unknown as T).pipe(
      tap(() => console.log('API response received')),
      // Simular retraso aleatorio
      delay(Math.random() * 1000 + 500)
    );
  }

  // Limpiar caché
  clearCache(url?: string): void {
    if (url) {
      this.cache.delete(url);
    } else {
      this.cache.clear();
    }
    console.log('Cache cleared');
  }
}

// Ejemplo de uso
const apiCache = new ApiCacheService();

// Múltiples componentes solicitan los mismos datos de API
apiCache.fetchData('/api/products').subscribe((data) => {
  console.log('Component 1: Data received', data);
});

// Un poco más tarde, otro componente solicita los mismos datos (recuperados del caché)
setTimeout(() => {
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Component 2: Data received', data);
  });
}, 1000);

// Solicitar nuevamente después de limpiar caché
setTimeout(() => {
  apiCache.clearCache();
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Component 3: Data received (after cache clear)', data);
  });
}, 2000);
```

#### Resultado de Ejecución
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

Este patrón con AsyncSubject es ideal para solicitudes API donde solo es importante el último valor al completarse. También previene la emisión duplicada de la misma solicitud.

> [!TIP]
> 💡 Tenga en cuenta que AsyncSubject no emite un valor cuando se llama `error()`, solo se reporta un error.


## Gestión de Formularios

Use `BehaviorSubject` para gestionar los valores actuales y el estado de validación de formularios reactivos.

### Enlace Bidireccional de Valores de Formulario

```ts
import { BehaviorSubject } from 'rxjs';
import { debounceTime, distinctUntilChanged } from 'rxjs';

interface UserForm {
  name: string;
  email: string;
  age: number;
}

class ReactiveForm {
  // BehaviorSubject con valor inicial
  private formSubject = new BehaviorSubject<UserForm>({
    name: '',
    email: '',
    age: 0
  });

  // Observable público
  formValues$ = this.formSubject.asObservable();

  // Resultado de validación
  private validSubject = new BehaviorSubject<boolean>(false);
  valid$ = this.validSubject.asObservable();

  constructor() {
    // Ejecutar validación en cambio de valor
    this.formValues$.pipe(
      debounceTime(300),
      distinctUntilChanged((prev, curr) => JSON.stringify(prev) === JSON.stringify(curr))
    ).subscribe(form => {
      this.validateForm(form);
    });
  }

  // Actualizar valor de campo
  updateField<K extends keyof UserForm>(field: K, value: UserForm[K]) {
    const currentForm = this.formSubject.getValue();
    this.formSubject.next({
      ...currentForm,
      [field]: value
    });
  }

  // Obtener formulario
  getForm(): UserForm {
    return this.formSubject.getValue();
  }

  // Validación
  private validateForm(form: UserForm) {
    const isValid =
      form.name.length > 0 &&
      form.email.includes('@') &&
      form.age > 0;

    this.validSubject.next(isValid);
  }

  // Envío de formulario
  submit() {
    if (this.validSubject.getValue()) {
      console.log('Form submitted:', this.getForm());
      // Solicitud API, etc.
    } else {
      console.error('Form is invalid');
    }
  }
}

// Ejemplo de uso
const form = new ReactiveForm();

// Monitorear valores de formulario
form.formValues$.subscribe(values => {
  console.log('Form values changed:', values);
  // Procesamiento de actualización de UI, etc.
});

// Monitorear estado de validación
form.valid$.subscribe(isValid => {
  console.log('Form validity:', isValid);
  // Habilitar/deshabilitar botón de envío, etc.
});

// Simular entrada de usuario
form.updateField('name', 'Taro Yamada');
form.updateField('email', 'yamada@example.com');
form.updateField('age', 30);

// Envío de formulario
form.submit();
```

#### Resultado de Ejecución
```sh
Form values changed: {name: '', email: '', age: 0}
Form validity: false
Form values changed: {name: 'Taro Yamada', email: '', age: 0}
Form values changed: {name: 'Taro Yamada', email: 'yamada@example.com', age: 0}
Form values changed: {name: 'Taro Yamada', email: 'yamada@example.com', age: 30}
Form is invalid
Form validity: true
```

Este patrón es particularmente útil para implementaciones de formularios reactivos. El BehaviorSubject siempre retiene el valor actual, haciéndolo ideal para gestión de estado de formularios.

## Registro e Historial

Use `ReplaySubject` para crear un mecanismo de gestión de registros que pueda retener y volver a mostrar el historial de operaciones pasadas.

### Gestión de Historial de Operaciones

```ts
import { Observable, ReplaySubject } from 'rxjs';
import { tap } from 'rxjs';

interface LogEntry {
  action: string;
  timestamp: number;
  data?: any;
}

class ActivityLogger {
  // Mantener los últimos 10 registros
  private logSubject = new ReplaySubject<LogEntry>(10);
  logs$ = this.logSubject.asObservable();

  // Agregar entrada de registro
  log(action: string, data?: any) {
    const entry: LogEntry = {
      action,
      timestamp: Date.now(),
      data
    };

    this.logSubject.next(entry);
    console.log(`Log recorded: ${action}`, data);
  }

  // Envolver otro Observable para registrar logs
  wrapWithLogging<T>(source$: Observable<T>, actionName: string): Observable<T> {
    return source$.pipe(
      tap(data => this.log(actionName, data))
    );
  }
}

// Ejemplo de uso
const logger = new ActivityLogger();

// Monitorear registros (mostrar en UI, etc.)
logger.logs$.subscribe(log => {
  const time = new Date(log.timestamp).toLocaleTimeString();
  console.log(`[${time}] ${log.action}`);
});

// Registrar varias operaciones
logger.log('Application started');
logger.log('User logged in', { userId: 'user123' });

// Un poco más tarde, nuevo componente inicia suscripción incluyendo registros pasados
setTimeout(() => {
  console.log('--- History viewer displays including past logs ---');
  logger.logs$.subscribe(log => {
    const time = new Date(log.timestamp).toLocaleTimeString();
    console.log(`History: [${time}] ${log.action}`);
  });

  // Agregar más registros
  logger.log('Data updated', { itemId: 456 });
}, 1000);
```

#### Resultado de Ejecución
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

ReplaySubject le permite proporcionar entradas de registro pasadas a nuevos suscriptores, haciéndolo ideal para gestión de historial. Es útil para rastrear operaciones de usuario y recopilar información de depuración.

> [!IMPORTANT]
> ⚠️ Si no se especifica un tamaño de búfer para `ReplaySubject`, todos los valores continuarán manteniéndose en memoria, así que tenga cuidado con grandes cantidades de datos o aplicaciones de larga ejecución.

## Gestión de Procesamiento Asíncrono

Use `Subject` y `BehaviorSubject` para gestionar el progreso y el estado activo de múltiples tareas en tiempo real.

### Gestión de Progreso de Tareas de Larga Duración

```ts
import { Subject, BehaviorSubject } from 'rxjs';

interface TaskProgress {
  taskId: string;
  progress: number; // 0-100
  status: 'pending' | 'running' | 'completed' | 'error';
  message?: string;
}

class TaskManager {
  // Notificación de progreso de tarea
  private progressSubject = new Subject<TaskProgress>();
  progress$ = this.progressSubject.asObservable();

  // Tareas actualmente en ejecución
  private activeTasksSubject = new BehaviorSubject<string[]>([]);
  activeTasks$ = this.activeTasksSubject.asObservable();

  // Iniciar tarea
  startTask(taskId: string, taskFn: (update: (progress: number) => void) => Promise<any>) {
    // Agregar a lista de tareas activas
    const currentTasks = this.activeTasksSubject.getValue();
    this.activeTasksSubject.next([...currentTasks, taskId]);

    // Notificación de progreso inicial
    this.progressSubject.next({
      taskId,
      progress: 0,
      status: 'running'
    });

    // Función para actualizar progreso
    const updateProgress = (progress: number) => {
      this.progressSubject.next({
        taskId,
        progress,
        status: 'running'
      });
    };

    // Ejecutar tarea
    return taskFn(updateProgress)
      .then(result => {
        // Notificación de finalización
        this.progressSubject.next({
          taskId,
          progress: 100,
          status: 'completed'
        });
        return result;
      })
      .catch(error => {
        // Notificación de error
        this.progressSubject.next({
          taskId,
          progress: 0,
          status: 'error',
          message: error.message
        });
        throw error;
      })
      .finally(() => {
        // Eliminar de lista de tareas activas
        const tasks = this.activeTasksSubject.getValue();
        this.activeTasksSubject.next(tasks.filter(id => id !== taskId));
      });
  }
}

// Ejemplo de uso
const taskManager = new TaskManager();

// Mostrar progreso en barra de progreso de UI, etc.
taskManager.progress$.subscribe(progress => {
  console.log(`Task ${progress.taskId}: ${progress.progress}% - ${progress.status}`);

  // Código de actualización de UI
  // progressBar.setValue(progress.progress);
  // statusLabel.setText(progress.status);
});

// Mostrar recuento de tareas activas
taskManager.activeTasks$.subscribe(tasks => {
  console.log(`Running tasks: ${tasks.length}`);
});

// Simular tarea de larga duración
taskManager.startTask('file-upload', (update) => {
  return new Promise((resolve) => {
    let progress = 0;

    // Simular progreso
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

#### Resultado de Ejecución
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

Este patrón usa Subject para proporcionar notificación en tiempo real del progreso de una tarea de larga duración. Es adecuado para mostrar el progreso de cargas de archivos, procesamiento de datos, operaciones en segundo plano, etc.

## Actualizaciones en Tiempo Real

Gestione el estado de conexión WebSocket, mensajes entrantes y control de reconexión usando múltiples Subjects.

### Gestión de Flujo WebSocket

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

  // Estado de conexión
  private connectionStatusSubject = new BehaviorSubject<boolean>(false);
  connectionStatus$ = this.connectionStatusSubject.asObservable();

  // Flujo de mensajes
  private messagesSubject = new Subject<WebSocketMessage>();
  messages$ = this.messagesSubject.asObservable();

  // Subject para terminación de conexión
  private destroySubject = new Subject<void>();

  constructor(url: string) {
    this.url = url;
  }

  // Iniciar conexión WebSocket
  connect(): void {
    if (this.socket) {
      return; // Ya conectado
    }

    this.socket = new WebSocket(this.url);

    // Configurar manejadores de eventos
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

      // Auto-reconectar
      this.reconnect();
    });

    this.socket.addEventListener('error', (error) => {
      console.error('WebSocket error:', error);
      this.connectionStatusSubject.next(false);
    });
  }

  // Lógica de reconexión
  private reconnect(): void {
    // Reconectar si no se ha llamado destroy
    timer(3000)
      .pipe(takeUntil(this.destroySubject))
      .subscribe(() => {
        console.log('Attempting WebSocket reconnection...');
        this.connect();
      });
  }

  // Enviar mensaje
  send(type: string, data: any): void {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      const message: WebSocketMessage = { type, data };
      this.socket.send(JSON.stringify(message));
    } else {
      console.error('WebSocket is not connected');
    }
  }

  // Obtener solo mensajes de tipo específico
  getMessagesOfType<T>(type: string): Observable<T> {
    return this.messages$.pipe(
      filter((msg) => msg.type === type),
      map((msg) => msg.data as T)
    );
  }

  // Desconectar
  disconnect(): void {
    this.destroySubject.next();
    this.destroySubject.complete();

    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
  }
}

// Ejemplo de uso
const wsService = new WebSocketService('wss://echo.websocket.org');

// Monitorear estado de conexión
wsService.connectionStatus$.subscribe((isConnected) => {
  console.log('Connection status:', isConnected ? 'Online' : 'Offline');
  // Actualización de UI, etc.
});

// Monitorear todos los mensajes
wsService.messages$.subscribe((message) => {
  console.log('Received message:', message);
});

// Monitorear solo mensajes de tipo específico
wsService
  .getMessagesOfType<{ price: number }>('stock-update')
  .subscribe((stockData) => {
    console.log(`Stock price update: ${stockData.price}`);
  });

// Iniciar conexión
wsService.connect();

// Enviar mensaje
setTimeout(() => {
  wsService.send('chat-message', { text: 'Hello!' });
}, 1000);

// Al terminar la aplicación
// wsService.disconnect();
```

#### Resultado de Ejecución
```sh
Connection status: Offline
WebSocket connection established
Connection status: Online
Message parsing error: SyntaxError: Unexpected token 'R', "Request se"... is not valid JSON
  at JSON.parse (<anonymous>)
  at WebSocket.<anonymous> (:30)
Received message: {type: 'chat-message', data: {…}}
```

Este patrón de gestión WebSocket es ideal para aplicaciones que requieren comunicación en tiempo real. Usa Subject para gestionar el estado de conexión y el flujo de mensajes, que puede ser compartido por múltiples componentes.

## Directrices para Elegir un Subject

| Caso de Uso | Subject Recomendado | Explicación |
|--------------|-------------|------|
| Notificación de eventos y comunicación | `Subject` | Adecuado para comunicación unidireccional simple |
| Retención de valor actual y gestión de estado | `BehaviorSubject` | Requiere un valor inicial, y el último valor siempre puede recuperarse |
| Flujo con historial de logs | `ReplaySubject` | Los valores pasados también pueden proporcionarse a suscriptores |
| Provisión masiva de valores finales y compartir respuestas | `AsyncSubject` | Notifica solo el último valor al completarse |

> 💡 El `$` al final de un nombre de variable es una convención de nomenclatura común de RxJS para indicar que es Observable.

## Resumen

La familia Subject de RxJS es una herramienta poderosa para una variedad de casos de uso:

- **BehaviorSubject**: Gestión de estado, gestión de formularios, visualización de valores actuales
- **Subject**: Notificación de eventos, comunicación entre componentes
- **ReplaySubject**: Gestión de historial, registro de operaciones, componente de participación retrasada
- **AsyncSubject**: Caché de respuestas API, compartir resultados de cálculo

Al combinar apropiadamente estos patrones, se pueden construir aplicaciones reactivas y mantenibles. Tenga especial cuidado de no olvidar cancelar la suscripción en el momento apropiado para prevenir fugas de memoria.
