---
description: Esta sección explica en detalle las características de cada uno de los cuatro tipos de Subject (Subject, BehaviorSubject, ReplaySubject y AsyncSubject), las situaciones en las que se utilizan, y una comparación de cómo elegir uno.
---

# Tipos de Subject

Además del `Subject` básico, RxJS proporciona varias clases derivadas especializadas para casos de uso específicos. Cada una de ellas tiene diferentes características de comportamiento y puede utilizarse en situaciones apropiadas para una programación reactiva más efectiva.

Aquí se explican en detalle los cuatro tipos principales de Subject, sus características y escenarios de uso.

## Cuatro Tipos Básicos de Subjects

| Tipo | Características | Casos de Uso Principales |
|------|------|----------------|
| [`Subject`](#subject) | Subject más simple<br>Recibe solo valores después de la suscripción | Notificación de eventos, distribución multicast |
| [`BehaviorSubject`](#behaviorsubject) | Mantiene el último valor y lo proporciona inmediatamente en una nueva suscripción | Gestión de estado, valor actual de componentes UI |
| [`ReplaySubject`](#replaysubject) | Reproduce un número específico de valores pasados para nuevos suscriptores | Historial de operaciones, actualizaciones recientes |
| [`AsyncSubject`](#asyncsubject) | Publica solo el último valor al completarse | Resultado de solicitud HTTP/API |

## `Subject` Estándar {#subject}

[📘 RxJS Official: Subject](https://rxjs.dev/api/index/class/Subject)

Este es el tipo más simple de Subject y recibe solo valores que ocurren después de la suscripción.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

// Sin valor inicial, no se recibe nada en la suscripción
subject.subscribe(value => console.log('Observer 1:', value));

subject.next(1);
subject.next(2);

// Segunda suscripción (solo recibe valores después de la suscripción)
subject.subscribe(value => console.log('Observer 2:', value));

subject.next(3);
subject.complete();
```

#### Resultado de Ejecución
```
Observer 1: 1
Observer 1: 2
Observer 1: 3
Observer 2: 3
```

## BehaviorSubject  {#behaviorsubject}

[📘 RxJS Official: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Requiere un valor inicial y siempre mantiene el último valor.
Los nuevos suscriptores reciben el último valor inmediatamente al suscribirse.

```ts
import { BehaviorSubject } from 'rxjs';

// Crear con valor inicial 0
const behaviorSubject = new BehaviorSubject<number>(0);

// Recibe inmediatamente el valor inicial
behaviorSubject.subscribe(value => console.log('Observer 1:', value));

behaviorSubject.next(1);
behaviorSubject.next(2);

// Segunda suscripción (recibe inmediatamente el último valor 2)
behaviorSubject.subscribe(value => console.log('Observer 2:', value));

behaviorSubject.next(3);
behaviorSubject.complete();
```

#### Resultado de Ejecución
```
Observer 1: 0
Observer 1: 1
Observer 1: 2
Observer 2: 2
Observer 1: 3
Observer 2: 3
```

### Ejemplos de Uso de BehaviorSubject

#### Gestión de Estado de Autenticación de Usuario

```ts
import { BehaviorSubject } from 'rxjs';

interface User {
  id: string;
  name: string;
}

// El valor inicial es null (no autenticado)
const currentUser$ = new BehaviorSubject<User | null>(null);

// Monitorear estado de autenticación en componentes, etc.
currentUser$.subscribe(user => {
  if (user) {
    console.log(`Logged in: ${user.name}`);
  } else {
    console.log('Not logged in');
  }
});

// Proceso de inicio de sesión
function login(user: User) {
  currentUser$.next(user);
}

// Proceso de cierre de sesión
function logout() {
  currentUser$.next(null);
}

// Ejemplo de uso
console.log('Application started');
// → Not logged in

login({ id: 'user123', name: 'Taro Yamada' });
// → Logged in: Taro Yamada

logout();
// → Not logged in
```

#### Resultado de Ejecución
```sh
Not logged in
Application started
Logged in: Taro Yamada
Not logged in
```

## `ReplaySubject` {#replaysubject}
[📘 RxJS Official: ReplaySubject](https://rxjs.dev/api/index/class/ReplaySubject)

Memoriza un número específico de valores anteriores y los reenvía a nuevos suscriptores.
Tamaño de búfer y ventana de tiempo configurables.

```ts
import { ReplaySubject } from 'rxjs';

// Almacenar los últimos 3 valores
const replaySubject = new ReplaySubject<number>(3);

replaySubject.next(1);
replaySubject.next(2);
replaySubject.next(3);
replaySubject.next(4);

// Iniciar suscripción (recibe los últimos 3 valores: 2, 3, 4)
replaySubject.subscribe(value => console.log('Observer 1:', value));

replaySubject.next(5);

// Segunda suscripción (recibe los últimos 3 valores: 3, 4, 5)
replaySubject.subscribe(value => console.log('Observer 2:', value));

replaySubject.complete();
```

#### Resultado de Ejecución
```
Observer 1: 2
Observer 1: 3
Observer 1: 4
Observer 1: 5
Observer 2: 3
Observer 2: 4
Observer 2: 5
```

### ReplaySubject con Ventana de Tiempo

El almacenamiento basado en tiempo también está disponible.

```ts
import { ReplaySubject } from 'rxjs';

// Almacenar hasta 5 valores dentro de 500ms
const timeWindowSubject = new ReplaySubject<number>(5, 500);

timeWindowSubject.next(1);

setTimeout(() => {
  timeWindowSubject.next(2);

  // Suscribirse después de 1000ms (1 no se recibe porque excedió la ventana de tiempo de 500ms)
  setTimeout(() => {
    timeWindowSubject.subscribe(value => console.log('Received:', value));
  }, 1000);
}, 100);
```

#### Resultado de Ejecución
```
Received: 2
```

### Ejemplos de Uso de ReplaySubject

#### Gestionar Historial de Búsqueda Reciente

```ts
import { ReplaySubject } from 'rxjs';

// Mantener las últimas 5 consultas de búsqueda
const searchHistory$ = new ReplaySubject<string>(5);

// Función de ejecución de búsqueda
function search(query: string) {
  console.log(`Search executed: ${query}`);
  searchHistory$.next(query);
  // Proceso de búsqueda real...
}

// Componente que muestra el historial de búsqueda
function showSearchHistory() {
  console.log('--- Search History ---');
  searchHistory$.subscribe(query => {
    console.log(query);
  });
}

// Ejemplo de uso
search('TypeScript');
search('RxJS');
search('Angular');
search('React');

showSearchHistory();
// Mostrar los últimos 5 elementos (4 elementos en este caso)
```

#### Resultado de Ejecución
```sh
Search executed: TypeScript
Search executed: RxJS
Search executed: Angular
Search executed: React
--- Search History ---
TypeScript
RxJS
Angular
React
```

## `AsyncSubject` {#asyncsubject}
[📘 RxJS Official: AsyncSubject](https://rxjs.dev/api/index/class/AsyncSubject)

Solo se emite el último valor al completarse. El valor antes de la finalización no se emitirá.

```ts
import { AsyncSubject } from 'rxjs';

const asyncSubject = new AsyncSubject<number>();

asyncSubject.subscribe(value => console.log('Observer 1:', value));

asyncSubject.next(1);
asyncSubject.next(2);
asyncSubject.next(3);

// Recibe solo el último valor, independientemente del tiempo de suscripción
asyncSubject.subscribe(value => console.log('Observer 2:', value));

asyncSubject.next(4);
asyncSubject.complete(); // El último valor (4) se emite al completarse
```

#### Resultado de Ejecución
```
Observer 1: 4
Observer 2: 4
```

### Ejemplos de Uso de AsyncSubject

#### Compartir Resultado de Solicitud API

```ts
import { AsyncSubject } from 'rxjs';

interface ApiResponse {
  data: any;
  status: number;
}

function fetchData(url: string) {
  const subject = new AsyncSubject<ApiResponse>();

  // Simular solicitud API
  console.log(`API request: ${url}`);
  setTimeout(() => {
    const response = {
      data: { id: 1, name: 'Sample data' },
      status: 200
    };

    subject.next(response);
    subject.complete();
  }, 1000);

  return subject;
}

// Ejemplo de uso
const data$ = fetchData('/api/users/1');

// Múltiples componentes pueden compartir el mismo resultado de solicitud
data$.subscribe(response => {
  console.log('Component 1:', response.data);
});

setTimeout(() => {
  data$.subscribe(response => {
    console.log('Component 2:', response.data);
  });
}, 1500); // Puede recibir valor incluso después de la finalización
```

#### Resultado de Ejecución
```sh
API request: /api/users/1
Component 1: {id: 1, name: 'Sample data'}
Component 2: {id: 1, name: 'Sample data'}
```

## Comparación y Guía de Selección para Cada Subject

Aquí hay un resumen de puntos para ayudarlo a elegir cada tipo de Subject.

### Cómo Elegir un Subject

|Tipo|Criterios de Selección|
|---|---|
|`Subject`|Utilizado para notificación simple de eventos y distribución multicast|
|`BehaviorSubject`|<li>Casos donde siempre se necesitan valores iniciales </li><li>Datos que representan el estado actual (estado de usuario, configuraciones, banderas, etc.) </li><li>Valores actuales de componentes UI</li>|
|`ReplaySubject`|<li>Casos donde es necesario mantener el historial de operaciones más reciente </li><li>Casos donde es necesario proporcionar datos históricos para suscriptores que se unieron más tarde </li><li>Flujos de datos almacenados en búfer</li>|
|`AsyncSubject`|<li>Cuando solo es importante el resultado final (por ejemplo, respuestas API) </li><li>Cuando no se necesitan pasos intermedios y solo se debe compartir el valor al completarse</li>|

### Flujo de Decisión de Selección

1. Solo se requiere el último valor al completarse ⇨ `AsyncSubject`
2. Se requieren los últimos N valores ⇨ `ReplaySubject`
3. Siempre se necesita el estado/valor actual ⇨ `BehaviorSubject`
4. Otro (por ejemplo, notificación pura de eventos) ⇨ `Subject`

## Patrones de Utilización en el Diseño de Aplicaciones

### Ejemplo de Comunicación entre Módulos

```ts
// Servicio de gestión de estado a nivel de aplicación
class AppStateService {
  // Usuario actual (BehaviorSubject porque se requiere valor inicial)
  private userSubject = new BehaviorSubject<User | null>(null);
  // Exponer como Observable de solo lectura
  readonly user$ = this.userSubject.asObservable();

  // Notificación (Subject porque es notificación simple de eventos)
  private notificationSubject = new Subject<Notification>();
  readonly notifications$ = this.notificationSubject.asObservable();

  // Búsquedas recientes (ReplaySubject porque se necesita historial)
  private searchHistorySubject = new ReplaySubject<string>(10);
  readonly searchHistory$ = this.searchHistorySubject.asObservable();

  // Caché de resultados de llamadas API (AsyncSubject porque solo se necesita el resultado final)
  private readonly apiCaches = new Map<string, AsyncSubject<any>>();

  // Ejemplos de métodos
  setUser(user: User | null) {
    this.userSubject.next(user);
  }

  notify(notification: Notification) {
    this.notificationSubject.next(notification);
  }

  addSearch(query: string) {
    this.searchHistorySubject.next(query);
  }

  // Cachear resultados de API
  fetchData(url: string): Observable<any> {
    if (!this.apiCaches.has(url)) {
      const subject = new AsyncSubject<any>();
      this.apiCaches.set(url, subject);

      // Llamada API real
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

## Resumen

RxJS Subject es una herramienta poderosa que puede utilizarse para una amplia variedad de casos de uso. Al comprender las características de cada tipo y utilizarlas apropiadamente, puede construir aplicaciones reactivas eficientes y mantenibles.

- `Subject`: Proporciona la funcionalidad multicast más simple y básica
- `BehaviorSubject`: mantiene el estado actual en todo momento y lo proporciona inmediatamente a nuevos suscriptores
- `ReplaySubject`: mantiene un historial de los valores más recientes y los ofrece a suscriptores que se unen más tarde
- `AsyncSubject`: Publica solo el último valor al completarse

Elegir el `Subject` adecuado para cada situación, incluyendo gestión de estado, notificación de eventos y compartir datos, es la clave para una programación reactiva eficiente.
