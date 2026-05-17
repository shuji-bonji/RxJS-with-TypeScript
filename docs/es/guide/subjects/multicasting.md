---
description: Explica en detalle cómo funciona el multicasting de RxJS, incluyendo patrones básicos usando Subject, uso de operadores share y shareReplay, evitar llamadas duplicadas de solicitudes API, estrategias de caché, compartir estado de aplicación y otros patrones de diseño prácticos con ejemplos de código TypeScript.
---

# Mecanismo de Multicasting

Multicasting es un método para distribuir eficientemente un flujo de datos de un único Observable a múltiples suscriptores (Observer).
En RxJS, esto puede lograrse mediante Subject y operadores.

## ¿Qué es Multicasting?

Un Observable normal (Cold Observable) crea un nuevo flujo de datos cada vez que se suscribe a él. Esto significa que si hay múltiples suscriptores, el mismo proceso se ejecutará múltiples veces.

Con multicasting, una fuente de datos puede ejecutarse solo una vez y los resultados distribuirse a múltiples suscriptores. Esto es especialmente importante cuando:

- No desea invocar solicitudes HTTP/API duplicadas
- Desea realizar una operación de alto costo (computación o efecto secundario) solo una vez
- Compartir el estado de la aplicación con múltiples componentes

## Patrón Básico de Multicasting

### Multicast Básico con Subject

```ts
import { Observable, Subject } from 'rxjs';
import { tap } from 'rxjs';

// Fuente de datos (Cold Observable)
function createDataSource(): Observable<number> {
  return new Observable<number>(observer => {
    console.log('Data source: Connected');
    // Lógica de generación de datos (asumiendo operación de alto costo)
    const id = setInterval(() => {
      const value = Math.round(Math.random() * 100);
      console.log(`Data source: Generated value -> ${value}`);
      observer.next(value);
    }, 1000);

    // Función de limpieza
    return () => {
      console.log('Data source: Disconnected');
      clearInterval(id);
    };
  });
}

// Implementación de multicast
function multicast() {
  // Fuente de datos original
  const source$ = createDataSource().pipe(
    tap(value => console.log(`Source processing: ${value}`))
  );

  // Subject para multicasting
  const subject = new Subject<number>();

  // Conectar fuente a Subject
  const subscription = source$.subscribe(subject);

  // Múltiples suscriptores se suscriben al Subject
  console.log('Observer 1 subscription started');
  const subscription1 = subject.subscribe(value => console.log(`Observer 1: ${value}`));

  // Agregar otro suscriptor después de 3 segundos
  setTimeout(() => {
    console.log('Observer 2 subscription started');
    const subscription2 = subject.subscribe(value => console.log(`Observer 2: ${value}`));

    // Cancelar todas las suscripciones después de 5 segundos
    setTimeout(() => {
      console.log('All subscriptions terminated');
      subscription.unsubscribe();
      subscription1.unsubscribe();
      subscription2.unsubscribe();
    }, 5000);
  }, 3000);
}

// Ejecutar
multicast();
```

#### Resultado de Ejecución
```
Data source: Connected
Observer 1 subscription started
Data source: Generated value -> 71
Source processing: 71
Observer 1: 71
Data source: Generated value -> 79
Source processing: 79
Observer 1: 79
Data source: Generated value -> 63
Source processing: 63
Observer 1: 63
Observer 2 subscription started
Data source: Generated value -> 49
Source processing: 49
Observer 1: 49
Observer 2: 49
Data source: Generated value -> 94
Source processing: 94
Observer 1: 94
Observer 2: 94
Data source: Generated value -> 89
Source processing: 89
Observer 1: 89
Observer 2: 89
Data source: Generated value -> 10
Source processing: 10
Observer 1: 10
Observer 2: 10
Data source: Generated value -> 68
Source processing: 68
Observer 1: 68
Observer 2: 68
All subscriptions terminated
Data source: Disconnected
```

## Operadores de Multicast

RxJS proporciona operadores dedicados para implementar multicasting.

### Operador `share()`
[📘 RxJS Official: share()](https://rxjs.dev/api/index/function/share)

Este es el operador más fácil para implementar multicast.
Internamente, combina `multicast()` y `refCount()`.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

// Observable contando a intervalos
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source: ${value}`)),
  share() // Habilitar multicasting
);

// Primer suscriptor
console.log('Observer 1 subscription started');
const subscription1 = source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Agregar segundo suscriptor después de 2.5 segundos
setTimeout(() => {
  console.log('Observer 2 subscription started');
  const subscription2 = source$.subscribe(value => console.log(`Observer 2: ${value}`));

  // Cancelar suscripción del suscriptor 1 después de 5 segundos
  setTimeout(() => {
    console.log('Observer 1 unsubscribed');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

#### Resultado de Ejecución
```
Observer 1 subscription started
Source: 0
Observer 1: 0
Observer 2 subscription started
Source: 1
Observer 1: 1
Observer 2: 1
Source: 2
Observer 1: 2
Observer 2: 2
Source: 3
Observer 1: 3
Observer 2: 3
Observer 1 unsubscribed
Source: 4
Observer 2: 4
```

### Control Detallado de `share()`

En lugar de `refCount()`, puede controlar el comportamiento más explícitamente pasando opciones a `share()` en RxJS 7 y posterior.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Source: ${value}`)),
  share({
    resetOnError: true,
    resetOnComplete: true,
    resetOnRefCountZero: true,
  })
);

// Primer suscriptor
console.log('Observer 1 subscription started');
const subscription1 = source$.subscribe((value) =>
  console.log(`Observer 1: ${value}`)
);

// Agregar segundo suscriptor después de 2.5 segundos
setTimeout(() => {
  console.log('Observer 2 subscription started');
  const subscription2 = source$.subscribe((value) =>
    console.log(`Observer 2: ${value}`)
  );

  setTimeout(() => {
    console.log('Observer 1 unsubscribed');
    subscription1.unsubscribe();
  }, 1500);
}, 2500);
```

#### Resultado de Ejecución
```
Observer 1 subscription started
Source: 0
Observer 1: 0
Source: 1
Observer 1: 1
Observer 2 subscription started
Source: 2
Observer 1: 2
Observer 2: 2
Source: 3
Observer 1: 3
Observer 2: 3
Observer 1 unsubscribed
Source: 4
Observer 2: 4
Source: 5
Observer 2: 5
```

De esta manera, puede controlar claramente el comportamiento cuando el flujo termina o cuando el número de suscriptores llega a cero.

### Operador `shareReplay()`

[📘 RxJS Official: shareReplay()](https://rxjs.dev/api/index/function/shareReplay)

Similar a `share()`, pero almacena un número específico de valores históricos y los hace disponibles para suscriptores que se unen más tarde.

```ts
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// Usando shareReplay (tamaño de búfer 2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source: ${value}`)),
  shareReplay({ bufferSize: 2, refCount: true }) // Almacenar los últimos 2 valores
);

// Primer suscriptor
console.log('Observer 1 subscription started');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Agregar segundo suscriptor después de 3.5 segundos
setTimeout(() => {
  console.log('Observer 2 subscription started - receives latest 2 values');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

#### Resultado de Ejecución
```
Observer 1 subscription started
Source: 0
Observer 1: 0
Source: 1
Observer 1: 1
Observer 2 subscription started - receives latest 2 values
Observer 2: 0
Observer 2: 1
Source: 2
Observer 1: 2
Observer 2: 2
Source: 3
Observer 1: 3
Observer 2: 3
Source: 4
Observer 1: 4
Observer 2: 4
```

## Temporización y Ciclo de Vida en Multicasting

Es importante entender el ciclo de vida de un flujo multicast. En particular, al usar el operador `share()`, se debe notar el siguiente comportamiento:

1. **Primer suscriptor**: `share()` inicia una conexión al Observable fuente en el momento en que se realiza la primera suscripción.
2. **Todos los suscriptores cancelan la suscripción**: Si se establece `share({ resetOnRefCountZero: true })`, la conexión a la fuente se desconectará cuando el número de suscriptores llegue a cero.
3. **Finalización o error**: Por defecto, `share()` restablece su estado interno al `complete` o `error` (si `resetOnComplete`/`resetOnError` es `true`).
4. **Resuscripción**: Si el flujo se restablece y luego se resuscribe, se reconstruirá como un nuevo Observable.

Por lo tanto, las opciones de `share()` controlan cuándo el flujo comienza, se detiene y se regenera dependiendo del número de suscripciones y el estado de finalización.

## Casos de Uso Prácticos

### Compartir Solicitud API

Ejemplo de evitar solicitudes duplicadas al mismo endpoint de API.

```ts
import { Observable, of, throwError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, catchError, shareReplay, tap } from 'rxjs';

// Simulación de servicio API
class UserService {
  private cache = new Map<string, Observable<any>>();

  getUser(id: string): Observable<any> {
    // Devolver desde caché si está disponible
    if (this.cache.has(id)) {
      console.log(`Getting user ID ${id} from cache`);
      return this.cache.get(id)!;
    }

    // Crear nueva solicitud
    console.log(`Getting user ID ${id} from API`);
    const request$ = ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${id}`).pipe(
      tap(response => console.log('API response:', response)),
      catchError((error: unknown) => {
        console.error('API error:', error);
        // Eliminar del caché
        this.cache.delete(id);
        return throwError(() => new Error('Failed to retrieve user'));
      }),
      // Compartir con shareReplay (cachear valor incluso después de la finalización)
      shareReplay({ bufferSize: 1, refCount: true })
    );

    // Guardar en caché
    this.cache.set(id, request$);
    return request$;
  }
}

// Ejemplo de uso
const userService = new UserService();

// Múltiples componentes solicitan los mismos datos de usuario
console.log('Component 1: Request user data');
userService.getUser('1').subscribe(user => {
  console.log('Component 1: Received user data', user);
});

// Otro componente solicita los mismos datos un poco después
setTimeout(() => {
  console.log('Component 2: Request same user data');
  userService.getUser('1').subscribe(user => {
    console.log('Component 2: Received user data', user);
  });
}, 1000);

// Solicitar otro usuario
setTimeout(() => {
  console.log('Component 3: Request different user data');
  userService.getUser('2').subscribe(user => {
    console.log('Component 3: Received user data', user);
  });
}, 2000);
```

#### Resultado de Ejecución
```
Component 1: Request user data
Getting user ID 1 from API
API response: {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Component 1: Received user data {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Component 2: Request same user data
Getting user ID 1 from cache
Component 2: Received user data {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Component 3: Request different user data
Getting user ID 2 from API
API response: {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
Component 3: Received user data {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
```

## Patrones de Diseño para Multicasting

### Observable Singleton

Un patrón en el cual un único Observable se comparte en toda la aplicación.

```ts
import { Subject } from 'rxjs';

// Gestión de estado global a nivel de aplicación
class AppState {
  // Instancia singleton
  private static instance: AppState;

  // Flujo de notificaciones global
  private notificationsSubject = new Subject<string>();

  // Observable público (solo lectura)
  readonly notifications$ = this.notificationsSubject.asObservable();

  // Acceso singleton
  static getInstance(): AppState {
    if (!AppState.instance) {
      AppState.instance = new AppState();
    }
    return AppState.instance;
  }

  // Método para enviar notificaciones
  notify(message: string): void {
    this.notificationsSubject.next(message);
  }
}

// Ejemplo de uso
const appState = AppState.getInstance();

// Monitorear notificaciones (desde múltiples componentes)
appState.notifications$.subscribe((msg) =>
  console.log('Component A:', msg)
);
appState.notifications$.subscribe((msg) =>
  console.log('Component B:', msg)
);

// Enviar notificación
appState.notify('System update available');
```

#### Resultado de Ejecución
```ts
Component A: System update available
Component B: System update available
```

## Resumen

Multicasting es una técnica importante para mejorar la eficiencia y el rendimiento de las aplicaciones RxJS. Los puntos principales son los siguientes:

- Multicasting permite que una única fuente de datos sea compartida por múltiples suscriptores
- Puede implementarse usando operadores como `share()`, `shareReplay()` y `publish()`
- Puede evitar solicitudes API duplicadas y optimizar procesos computacionalmente costosos
- Útil para gestión de estado y comunicación entre componentes

Elegir la estrategia de multicast adecuada puede reducir la cantidad de código y mejorar la mantenibilidad mientras aumenta la capacidad de respuesta y eficiencia de la aplicación.
