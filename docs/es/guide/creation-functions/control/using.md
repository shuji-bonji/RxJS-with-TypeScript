---
description: using() es una Función de Creación de RxJS que crea y libera recursos automáticamente de acuerdo con el ciclo de vida del Observable. Gestiona de forma segura recursos que requieren limpieza manual como WebSocket, manejadores de archivos y temporizadores, previniendo fugas de memoria. Realiza una gestión confiable de recursos con creación de recursos al inicio de la suscripción y liberación automática al terminar la suscripción.
---

# using()

[📘 Documentación Oficial RxJS - using](https://rxjs.dev/api/index/function/using)

`using()` es una Función de Creación que crea y libera recursos automáticamente de acuerdo con el ciclo de vida del Observable. Gestiona de forma segura recursos que requieren limpieza manual como WebSocket, manejadores de archivos y temporizadores, previniendo fugas de memoria.

## Uso Básico

### Gestión simple de recursos

```typescript
import { using, interval, Subscription, take } from 'rxjs';
const resource$ = using(
  // Resource factory: Se ejecuta al inicio de la suscripción
  () => {
    console.log('Creación de recurso');
    return new Subscription(() => console.log('Liberación de recurso'));
  },
  // Observable factory: Crea Observable usando el recurso
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Valor:', value),
  complete: () => console.log('Completado')
});

// Salida:
// Creación de recurso
// Valor: 0
// Valor: 1
// Valor: 2
// Completado
// Liberación de recurso
```

> [!IMPORTANT]
> **Liberación automática de recursos**
>
> `using()` libera automáticamente los recursos cuando el Observable completa (`complete`) o se desuscribe (`unsubscribe`).

## Mecanismo de using()

`using()` recibe las siguientes dos funciones:

```typescript
function using<T>(
  resourceFactory: () => Unsubscribable | void,
  observableFactory: (resource: Unsubscribable | void) => ObservableInput<T>
): Observable<T>
```

### 1. resourceFactory (fábrica de recursos)

Se ejecuta al inicio de la suscripción y crea el recurso. Lo que debe devolver es un objeto con un método `unsubscribe()`.

```typescript
// Devolver Subscription
() => new Subscription(() => {
  console.log('Procesamiento de limpieza');
});

// O devolver un objeto con método unsubscribe
() => ({
  unsubscribe: () => {
    console.log('Procesamiento de limpieza');
  }
});
```

### 2. observableFactory (fábrica de Observable)

Crea un Observable usando el recurso.

```typescript
(resource) => interval(1000);
```

## Patrones Prácticos

### Gestión de conexión WebSocket

```typescript
import { using, interval, Subject, map, takeUntil } from 'rxjs';
function createWebSocketStream(url: string) {
  return using(
    // Crear conexión WebSocket
    () => {
      const ws = new WebSocket(url);
      console.log('Conexión WebSocket iniciada:', url);

      ws.onopen = () => console.log('Conexión completada');
      ws.onerror = (error) => console.error('Error de conexión:', error);

      return {
        unsubscribe: () => {
          console.log('Conexión WebSocket terminada');
          ws.close();
        }
      };
    },
    // Crear stream de mensajes
    () => {
      const messages$ = new Subject<MessageEvent>();
      const ws = new WebSocket(url);

      ws.onmessage = (event) => messages$.next(event);
      ws.onerror = (error) => messages$.error(error);
      ws.onclose = () => messages$.complete();

      return messages$;
    }
  );
}

// Ejemplo de uso
const websocket$ = createWebSocketStream('wss://echo.websocket.org');

const subscription = websocket$.subscribe({
  next: message => console.log('Recibido:', message.data),
  error: error => console.error('Error:', error),
  complete: () => console.log('Completado')
});

// Cerrar WebSocket automáticamente después de 10 segundos
setTimeout(() => subscription.unsubscribe(), 10000);
```

### Limpieza automática de temporizador

```typescript
import { using, Observable, Subscription } from 'rxjs';

function createTimerStream(intervalMs: number) {
  return using(
    // Crear recurso de temporizador
    () => {
      let timerId: number | null = null;
      console.log('Temporizador iniciado');

      return new Subscription(() => {
        if (timerId !== null) {
          clearInterval(timerId);
          console.log('Temporizador detenido');
        }
      });
    },
    // Crear stream de temporizador
    () => new Observable(subscriber => {
      const timerId = setInterval(() => {
        subscriber.next(Date.now());
      }, intervalMs);

      return () => clearInterval(timerId);
    })
  );
}

// Ejemplo de uso
const timer$ = createTimerStream(1000);

const subscription = timer$.subscribe({
  next: time => console.log('Hora actual:', new Date(time).toLocaleTimeString())
});

// Detener después de 5 segundos
setTimeout(() => subscription.unsubscribe(), 5000);
```

### Operación de archivos (Node.js)

```typescript
import { using, Observable } from 'rxjs';
import * as fs from 'fs';

function readFileStream(filePath: string) {
  return using(
    // Abrir manejador de archivo
    () => {
      const fd = fs.openSync(filePath, 'r');
      console.log('Archivo abierto:', filePath);

      return {
        unsubscribe: () => {
          fs.closeSync(fd);
          console.log('Archivo cerrado');
        }
      };
    },
    // Crear stream de lectura de archivo
    () => new Observable<string>(subscriber => {
      const stream = fs.createReadStream(filePath, { encoding: 'utf8' });

      stream.on('data', (chunk) => subscriber.next(chunk));
      stream.on('error', (error) => subscriber.error(error));
      stream.on('end', () => subscriber.complete());

      return () => stream.destroy();
    })
  );
}

// Ejemplo de uso
const file$ = readFileStream('./data.txt');

file$.subscribe({
  next: chunk => console.log('Leído:', chunk),
  error: error => console.error('Error:', error),
  complete: () => console.log('Lectura completada')
});
```

### Gestión de event listeners

```typescript
import { using, Observable } from 'rxjs';

function createClickStream(element: HTMLElement) {
  return using(
    // Registrar event listener
    () => {
      console.log('Event listener registrado');

      return {
        unsubscribe: () => {
          console.log('Event listener eliminado');
          // La eliminación real se hace dentro del Observable factory
        }
      };
    },
    // Crear stream de eventos de clic
    () => new Observable<MouseEvent>(subscriber => {
      const handler = (event: MouseEvent) => subscriber.next(event);

      element.addEventListener('click', handler);

      return () => {
        element.removeEventListener('click', handler);
      };
    })
  );
}

// Ejemplo de uso
const button = document.querySelector('#myButton') as HTMLElement;
const clicks$ = createClickStream(button);

const subscription = clicks$.subscribe({
  next: event => console.log('Posición del clic:', event.clientX, event.clientY)
});

// Eliminar automáticamente después de 30 segundos
setTimeout(() => subscription.unsubscribe(), 30000);
```

## Ejemplos de Uso Común

### 1. Gestión de conexión de base de datos

```typescript
import { using, from, mergeMap } from 'rxjs';
interface DbConnection {
  query: (sql: string) => Promise<any[]>;
  close: () => Promise<void>;
}

function queryWithConnection(sql: string) {
  return using(
    // Establecer conexión de base de datos
    () => {
      const connection = createDbConnection();
      console.log('Conexión DB establecida');

      return {
        unsubscribe: async () => {
          await connection.close();
          console.log('Conexión DB cerrada');
        }
      };
    },
    // Ejecutar consulta
    () => {
      const connection = createDbConnection();
      return from(connection.query(sql));
    }
  );
}

// Ejemplo de uso
const users$ = queryWithConnection('SELECT * FROM users');

users$.subscribe({
  next: rows => console.log('Obtenido:', rows),
  error: error => console.error('Error:', error),
  complete: () => console.log('Consulta completada')
});

function createDbConnection(): DbConnection {
  // Procesamiento de conexión real
  return {
    query: async (sql) => [],
    close: async () => {}
  };
}
```

### 2. Gestión de pool de recursos

```typescript
import { using, Observable, defer } from 'rxjs';

class ResourcePool<T> {
  private available: T[] = [];
  private inUse = new Set<T>();

  constructor(private factory: () => T, size: number) {
    for (let i = 0; i < size; i++) {
      this.available.push(factory());
    }
  }

  acquire(): T | null {
    const resource = this.available.pop();
    if (resource) {
      this.inUse.add(resource);
      return resource;
    }
    return null;
  }

  release(resource: T): void {
    if (this.inUse.has(resource)) {
      this.inUse.delete(resource);
      this.available.push(resource);
    }
  }
}

// Ejemplo de uso
const pool = new ResourcePool(() => ({ id: Math.random() }), 5);

function usePooledResource<T>(
  pool: ResourcePool<T>,
  work: (resource: T) => Observable<any>
) {
  return using(
    () => {
      const resource = pool.acquire();
      if (!resource) {
        throw new Error('Pool de recursos agotado');
      }
      console.log('Recurso adquirido:', resource);

      return {
        unsubscribe: () => {
          pool.release(resource);
          console.log('Recurso devuelto:', resource);
        }
      };
    },
    (subscription) => {
      const resource = pool.acquire();
      return resource ? work(resource) : defer(() => {
        throw new Error('Error al adquirir recurso');
      });
    }
  );
}

// Procesar usando recurso
const work$ = usePooledResource(pool, (resource) =>
  new Observable(subscriber => {
    subscriber.next(`Procesando: ${resource.id}`);
    setTimeout(() => subscriber.complete(), 1000);
  })
);

work$.subscribe({
  next: result => console.log(result),
  complete: () => console.log('Procesamiento completado')
});
```

### 3. Gestión coordinada de múltiples recursos

```typescript
import { using, merge, Subject } from 'rxjs';

interface MultiResource {
  ws: WebSocket;
  timer: number;
}

function createMultiResourceStream() {
  return using(
    // Crear múltiples recursos
    () => {
      const ws = new WebSocket('wss://echo.websocket.org');
      const timer = setInterval(() => {
        console.log('Ejecución periódica');
      }, 1000);

      console.log('Múltiples recursos creados');

      return {
        unsubscribe: () => {
          ws.close();
          clearInterval(timer);
          console.log('Múltiples recursos liberados');
        }
      };
    },
    // Combinar múltiples streams
    () => {
      const messages$ = new Subject<string>();
      const ticks$ = new Subject<number>();

      return merge(messages$, ticks$);
    }
  );
}

// Ejemplo de uso
const multiStream$ = createMultiResourceStream();

const subscription = multiStream$.subscribe({
  next: value => console.log('Recibido:', value)
});

// Liberar todos los recursos después de 10 segundos
setTimeout(() => subscription.unsubscribe(), 10000);
```

### 4. Gestión condicional de recursos

```typescript
import { using, interval, EMPTY, take } from 'rxjs';
function conditionalResource(shouldCreate: boolean) {
  return using(
    () => {
      if (shouldCreate) {
        console.log('Creación de recurso');
        return {
          unsubscribe: () => console.log('Liberación de recurso')
        };
      } else {
        console.log('Creación de recurso omitida');
        return { unsubscribe: () => {} };
      }
    },
    () => {
      if (shouldCreate) {
        return interval(1000).pipe(take(3));
      } else {
        return EMPTY;
      }
    }
  );
}

// Caso de crear recurso
conditionalResource(true).subscribe({
  next: val => console.log('Valor:', val),
  complete: () => console.log('Completado')
});

// Caso de no crear recurso
conditionalResource(false).subscribe({
  next: val => console.log('Valor:', val),
  complete: () => console.log('Completado')
});
```

## Manejo de Errores

### Liberación de recursos en caso de error

```typescript
import { using, throwError, of, catchError } from 'rxjs';
const errorHandling$ = using(
  () => {
    console.log('Creación de recurso');
    return {
      unsubscribe: () => console.log('Liberación de recurso (también se ejecuta en error)')
    };
  },
  () => throwError(() => new Error('Error intencional'))
);

errorHandling$.pipe(
  catchError(error => {
    console.error('Error capturado:', error.message);
    return of('Valor por defecto');
  })
).subscribe({
  next: val => console.log('Valor:', val),
  complete: () => console.log('Completado')
});

// Salida:
// Creación de recurso
// Liberación de recurso (también se ejecuta en error)
// Error capturado: Error intencional
// Valor: Valor por defecto
// Completado
```

> [!IMPORTANT]
> **Liberación confiable de recursos incluso en errores**
>
> `using()` siempre libera los recursos creados en `resourceFactory` incluso si ocurre un error.

## Errores Comunes y Soluciones

### 1. Olvidar implementar el método unsubscribe

**Ejemplo de error:**
```typescript
// ❌ Error: No hay método unsubscribe
using(
  () => {
    console.log('Creación de recurso');
    return {}; // No hay unsubscribe
  },
  () => interval(1000)
);
```

**Solución:**
```typescript
// ✅ Correcto: Implementar método unsubscribe
using(
  () => {
    console.log('Creación de recurso');
    return {
      unsubscribe: () => console.log('Liberación de recurso')
    };
  },
  () => interval(1000)
);
```

### 2. Creación de recursos asíncronos

**Problema:**
```typescript
// ❌ Problema: resourceFactory no puede ser asíncrono
using(
  async () => { // No se puede usar async
    const resource = await createResourceAsync();
    return resource;
  },
  () => interval(1000)
);
```

**Solución:**
```typescript
import { defer, from, mergeMap } from 'rxjs';
// ✅ Correcto: Procesamiento asíncrono con defer y mergeMap

defer(() =>
  from(createResourceAsync()).pipe(
    mergeMap(resource =>
      using(
        () => resource,
        () => interval(1000)
      )
    )
  )
);
```

### 3. Creación duplicada de recursos

**Problema:**
```typescript
// ❌ Problema: Crear recursos por separado en resourceFactory y observableFactory
let sharedResource: any;

using(
  () => {
    sharedResource = createResource(); // Crear aquí
    return { unsubscribe: () => sharedResource.close() };
  },
  () => {
    const resource = createResource(); // Crear de nuevo
    return from(resource.getData());
  }
);
```

**Solución:**
```typescript
// ✅ Correcto: Compartir recurso
using(
  () => {
    const resource = createResource();
    return {
      resource, // Mantener recurso
      unsubscribe: () => resource.close()
    };
  },
  (subscription: any) => {
    return from(subscription.resource.getData());
  }
);
```

## Mejores Prácticas de using()

### 1. Liberación confiable de recursos

```typescript
// ✅ Buen ejemplo: Patrón try-finally
using(
  () => {
    const resource = createResource();
    return {
      unsubscribe: () => {
        try {
          resource.close();
        } catch (error) {
          console.error('Error al liberar recurso:', error);
        }
      }
    };
  },
  () => interval(1000)
);
```

### 2. Registro de creación de recursos

```typescript
// ✅ Buen ejemplo: Registrar ciclo de vida del recurso en logs
using(
  () => {
    const resourceId = Math.random();
    console.log(`[${resourceId}] Creación de recurso`);

    return {
      unsubscribe: () => {
        console.log(`[${resourceId}] Liberación de recurso`);
      }
    };
  },
  () => interval(1000)
);
```

### 3. Gestión de recursos con seguridad de tipos

```typescript
// ✅ Buen ejemplo: Usar tipos de TypeScript
interface ManagedResource {
  id: string;
  close: () => void;
}

function createManagedStream(resource: ManagedResource) {
  return using(
    () => {
      console.log('Recurso iniciado:', resource.id);
      return {
        unsubscribe: () => {
          resource.close();
          console.log('Recurso terminado:', resource.id);
        }
      };
    },
    () => interval(1000)
  );
}
```

## Comparación con Gestión Manual

### Gestión manual de recursos (❌ No recomendado)

```typescript
// ❌ Mal ejemplo: Gestión manual (riesgo de olvido de liberación)
const ws = new WebSocket('wss://example.com');
const subscription = interval(1000).subscribe(() => {
  ws.send('ping');
});

// Posibilidad de olvidar liberar
// subscription.unsubscribe();
// ws.close();
```

### Gestión de recursos con using() (✅ Recomendado)

```typescript
// ✅ Buen ejemplo: Gestión automática con using()
const stream$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return {
      unsubscribe: () => ws.close()
    };
  },
  () => interval(1000)
);

const subscription = stream$.subscribe(() => {
  // Procesamiento usando WebSocket
});

// WebSocket se cierra automáticamente solo con unsubscribe()
subscription.unsubscribe();
```

## Resumen

`using()` es una Función de Creación que gestiona automáticamente recursos de acuerdo con el ciclo de vida del Observable.

**Características principales:**
- Crea recursos al inicio de la suscripción
- Liberación automática al terminar la suscripción (complete o unsubscribe)
- Previene fugas de memoria
- Liberación confiable de recursos incluso en errores

**Escenarios de uso:**
- Conexiones de red como WebSocket, EventSource
- Manejadores de archivos, conexiones de base de datos
- Limpieza automática de temporizadores, intervalos
- Eliminación automática de event listeners

**Precauciones:**
- `resourceFactory` debe ser una función síncrona
- Siempre implementar el método `unsubscribe`
- Manejar errores apropiadamente

**Uso recomendado:**
- Prevenir olvido de liberación de recursos
- Registrar ciclo de vida en logs
- Usar tipos de TypeScript para gestión con seguridad de tipos

## Páginas Relacionadas

- [scheduled()](/es/guide/creation-functions/control/scheduled) - Generar Observable especificando scheduler
- [Funciones de Creación de Control](/es/guide/creation-functions/control/) - Comparación de scheduled() y using()
- [finalize()](/es/guide/error-handling/finalize) - Operador para agregar procesamiento al terminar la suscripción

## Recursos de Referencia

- [Documentación Oficial RxJS - using](https://rxjs.dev/api/index/function/using)
- [Documentación Oficial RxJS - Subscription](https://rxjs.dev/guide/subscription)
