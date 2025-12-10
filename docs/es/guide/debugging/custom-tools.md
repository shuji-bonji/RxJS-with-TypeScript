---
description: Esta página explica cómo crear herramientas personalizadas para la depuración de RxJS. Proporciona ejemplos de implementaciones prácticas de herramientas de depuración, como seguimiento de streams con nombre, operadores de depuración configurables y operadores de medición de rendimiento.
---

# Herramientas de depuración personalizadas

Crear sus propias herramientas de depuración permite una depuración flexible adaptada a los requisitos de su proyecto.

## Depuración de streams con nombre

Cree un operador personalizado que pueda nombrar y rastrear un Observable.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Mapa para administrar nombres de streams
const namedStreams = new Map<string, any[]>();

/**
 * Nombrar y rastrear un Observable
 */
function tagStream<T>(name: string) {
  if (!namedStreams.has(name)) {
    namedStreams.set(name, []);
  }

  return tap<T>({
    next: value => {
      const log = {
        name,
        type: 'next',
        value,
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.log(`[${name}] next:`, value);
    },
    error: error => {
      const log = {
        name,
        type: 'error',
        error,
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.error(`[${name}] error:`, error);
    },
    complete: () => {
      const log = {
        name,
        type: 'complete',
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.log(`[${name}] complete`);
    }
  });
}

/**
 * Obtener logs para un stream con nombre específico
 */
function getStreamLogs(name: string) {
  return namedStreams.get(name) || [];
}

/**
 * Obtener una lista de todos los streams con nombre
 */
function getAllStreamNames() {
  return Array.from(namedStreams.keys());
}

/**
 * Limpiar logs
 */
function clearStreamLogs(name?: string) {
  if (name) {
    namedStreams.set(name, []);
  } else {
    namedStreams.clear();
  }
}
```

### Ejemplos de uso

Aquí hay un ejemplo de cómo nombrar y rastrear un Observable.

```ts
import { interval } from 'rxjs';
import { map, take } from 'rxjs';

// Nombrar el Observable
interval(1000)
  .pipe(
    tagStream('interval-stream'),
    map(x => x * 2),
    take(5)
  )
  .subscribe();

// Verificar logs después de 3 segundos
setTimeout(() => {
  console.log('Todos los streams:', getAllStreamNames());
  console.log('Logs de interval-stream:', getStreamLogs('interval-stream'));
}, 3000);

// Salida:
// [interval-stream] next: 0
// [interval-stream] next: 1
// [interval-stream] next: 2
// Todos los streams: ['interval-stream']
// Logs de interval-stream: [
//   { name: 'interval-stream', type: 'next', value: 0, timestamp: 1697280000000 },
//   { name: 'interval-stream', type: 'next', value: 1, timestamp: 1697280001000 },
//   { name: 'interval-stream', type: 'next', value: 2, timestamp: 1697280002000 }
// ]
```

### Rastrear múltiples streams

Nombre y administre múltiples streams.

```ts
import { interval, fromEvent } from 'rxjs';
import { map, take } from 'rxjs';

// Nombrar múltiples streams
interval(1000)
  .pipe(
    tagStream('timer-stream'),
    map(x => x * 2),
    take(3)
  )
  .subscribe();

const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tagStream('click-stream'),
      take(5)
    )
    .subscribe();
}

// Verificar todos los streams
console.log('Streams rastreados:', getAllStreamNames());
// Salida: ['timer-stream', 'click-stream']
```

> [!NOTE]
> **Acerca de rxjs-spy**
>
> `rxjs-spy` era una biblioteca útil para depurar Observables, pero ya no se mantiene y tiene problemas de compatibilidad con el último RxJS.
>
> En su lugar, recomendamos usar operadores de depuración personalizados como se muestra arriba. Son más flexibles y se pueden personalizar según los requisitos de su proyecto.

## Crear operador de depuración personalizado

Crear sus propios operadores de depuración permite una mayor flexibilidad de depuración.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

interface DebugOptions {
  enabled?: boolean;
  label?: string;
  logValues?: boolean;
  logErrors?: boolean;
  logComplete?: boolean;
  logTimestamp?: boolean;
}

/**
 * Operador de depuración personalizado
 */
function debug<T>(options: DebugOptions = {}) {
  const {
    enabled = true,
    label = 'Debug',
    logValues = true,
    logErrors = true,
    logComplete = true,
    logTimestamp = false
  } = options;

  if (!enabled) {
    return (source: Observable<T>) => source;
  }

  return tap<T>({
    next: value => {
      if (logValues) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.log(`${timestamp} [${label}] next:`, value);
      }
    },
    error: error => {
      if (logErrors) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.error(`${timestamp} [${label}] error:`, error);
      }
    },
    complete: () => {
      if (logComplete) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.log(`${timestamp} [${label}] complete`);
      }
    }
  });
}

// Ejemplo de uso
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    debug({ label: 'Input', logTimestamp: true }),
    map(x => x * 2),
    debug({ label: 'After Map', logTimestamp: true })
  )
  .subscribe();

// Salida:
// [2025-10-14T12:00:00.000Z] [Input] next: 1
// [2025-10-14T12:00:00.001Z] [After Map] next: 2
// [2025-10-14T12:00:00.001Z] [Input] next: 2
// [2025-10-14T12:00:00.002Z] [After Map] next: 4
// [2025-10-14T12:00:00.002Z] [Input] next: 3
// [2025-10-14T12:00:00.003Z] [After Map] next: 6
// [2025-10-14T12:00:00.003Z] [Input] complete
// [2025-10-14T12:00:00.004Z] [After Map] complete
```

## Operador de depuración para medición de rendimiento

Un operador de medición de rendimiento que registra automáticamente el tiempo de ejecución y el número de valores.

```ts
import { tap } from 'rxjs';

function measure<T>(label: string) {
  let startTime: number;
  let count = 0;

  return tap<T>({
    subscribe: () => {
      startTime = performance.now();
      console.log(`⏱️ [${label}] Inicio`);
    },
    next: value => {
      count++;
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Valor #${count} (${elapsed.toFixed(2)}ms):`, value);
    },
    complete: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Completado (Total: ${elapsed.toFixed(2)}ms, ${count} valores)`);
    },
    error: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Error (${elapsed.toFixed(2)}ms)`);
    }
  });
}

// Ejemplo de uso
import { interval } from 'rxjs';
import { take, delay } from 'rxjs';

interval(100)
  .pipe(
    take(5),
    measure('Interval Stream'),
    delay(50)
  )
  .subscribe();

// Salida:
// ⏱️ [Interval Stream] Inicio
// ⏱️ [Interval Stream] Valor #1 (150.23ms): 0
// ⏱️ [Interval Stream] Valor #2 (250.45ms): 1
// ⏱️ [Interval Stream] Valor #3 (350.67ms): 2
// ⏱️ [Interval Stream] Valor #4 (450.89ms): 3
// ⏱️ [Interval Stream] Valor #5 (551.12ms): 4
// ⏱️ [Interval Stream] Completado (Total: 551.12ms, 5 valores)
```

## Resumen

Al crear herramientas de depuración personalizadas

- ✅ **Streams con nombre** - identificar y rastrear múltiples streams por nombre
- ✅ **Configuración flexible** - Operador de depuración adaptado a los requisitos del proyecto
- ✅ **Medición de rendimiento** - Registrar automáticamente el tiempo de ejecución y el número de valores
- ✅ **Gestión de logs** - Registrar y recuperar logs con marca de tiempo

Se recomienda que estas herramientas estén habilitadas solo en el entorno de desarrollo y deshabilitadas en el entorno de producción.

## Páginas relacionadas

- [Estrategias básicas de depuración](/es/guide/debugging/) - Cómo usar el operador tap y las herramientas de desarrollo
- [Escenarios comunes de depuración](/es/guide/debugging/common-scenarios) - Solución de problemas específicos
- [Depuración de rendimiento](/es/guide/debugging/performance) - Monitoreo de suscripciones, verificación de uso de memoria
