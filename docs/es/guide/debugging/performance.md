---
description: Esta página explica las técnicas de depuración de rendimiento para aplicaciones RxJS. Proporciona técnicas prácticas como el seguimiento del número de suscripciones, la detección de reevaluaciones innecesarias, el monitoreo del uso de memoria, la configuración del entorno de desarrollo, la depuración con seguridad de tipos y el establecimiento de límites de error.
---

# Depuración de rendimiento y mejores prácticas

Esta sesión cubrirá técnicas para optimizar el rendimiento de las aplicaciones RxJS y crear un entorno de depuración eficiente.

## Verificar el número de suscripciones

Verifique si se han creado varias suscripciones sin querer.

```ts
import { Observable, defer } from 'rxjs';
import { finalize } from 'rxjs';

let globalSubscriptionId = 0;
let activeSubscriptions = 0;

/**
 * Operador personalizado para rastrear el número de suscripciones
 */
function tracked<T>(label: string) {
  return (source: Observable<T>) =>
    defer(() => {
      const id = ++globalSubscriptionId;
      activeSubscriptions++;
      console.log(`➕ Suscripción iniciada [${label}] #${id} (Activas: ${activeSubscriptions})`);

      return source.pipe(
        finalize(() => {
          activeSubscriptions--;
          console.log(`➖ Suscripción finalizada [${label}] #${id} (Activas: ${activeSubscriptions})`);
        })
      );
    });
}

// Ejemplo de uso
import { interval } from 'rxjs';
import { take } from 'rxjs';

const stream$ = interval(1000).pipe(
  take(3),
  tracked('Test Stream')
);

const sub1 = stream$.subscribe();
const sub2 = stream$.subscribe();

setTimeout(() => {
  sub1.unsubscribe();
  sub2.unsubscribe();
}, 5000);

// Salida:
// ➕ Suscripción iniciada [Test Stream] #1 (Activas: 1)
// ➕ Suscripción iniciada [Test Stream] #2 (Activas: 2)
// ➖ Suscripción finalizada [Test Stream] #1 (Activas: 1)
// ➖ Suscripción finalizada [Test Stream] #2 (Activas: 0)
```

En esta implementación,
- ✅ `defer` para generar un nuevo ID cada vez que se suscribe
- ✅ `finalize` para garantizar que el proceso de cancelación de suscripción se realice de forma fiable
- ✅ Rastrear el número de suscripciones activas en tiempo real
- ✅ Tipo seguro y funciona con RxJS v8

## Detectar reevaluaciones innecesarias

Verifica si el mismo valor se ha calculado más de una vez.

```ts
import { of } from 'rxjs';
import { map, tap, shareReplay } from 'rxjs';

let computeCount = 0;

function expensiveComputation(value: number): number {
  computeCount++;
  console.log(`💰 Cálculo ejecutado (${computeCount} veces):`, value);
  // Simular cálculo pesado
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result += Math.sin(i);
  }
  return result;
}

// ❌ Sin shareReplay → Calculado para cada suscripción
console.log('=== Sin shareReplay ===');
computeCount = 0;
const withoutShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x))
);

withoutShare$.subscribe(v => console.log('Suscripción 1:', v));
withoutShare$.subscribe(v => console.log('Suscripción 2:', v));
// Salida: El cálculo se ejecuta 6 veces (3 valores × 2 suscripciones)

// ✅ Con shareReplay → Los resultados del cálculo se comparten
console.log('\n=== Con shareReplay ===');
computeCount = 0;
const withShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x)),
  shareReplay(3)
);

withShare$.subscribe(v => console.log('Suscripción 1:', v));
withShare$.subscribe(v => console.log('Suscripción 2:', v));
// Salida: El cálculo se ejecuta solo 3 veces
```

## Monitorear el uso de memoria

Este método de monitoreo se utiliza para detectar fugas de memoria.

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MemoryMonitor {
  private intervals: ReturnType<typeof setInterval>[] = [];

  start(intervalMs: number = 5000) {
    const id = setInterval(() => {
      if (typeof performance !== 'undefined' && (performance as any).memory) {
        const memory = (performance as any).memory;
        console.log('📊 Uso de memoria:', {
          Usado: `${(memory.usedJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          Total: `${(memory.totalJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          Límite: `${(memory.jsHeapSizeLimit / 1024 / 1024).toFixed(2)} MB`
        });
      }
    }, intervalMs);

    this.intervals.push(id);
  }

  stop() {
    this.intervals.forEach(id => clearInterval(id));
    this.intervals = [];
  }
}

// Ejemplo de uso
const monitor = new MemoryMonitor();
monitor.start(5000); // Mostrar uso de memoria cada 5 segundos

// Probar fuga de memoria
const leakyStreams: any[] = [];

for (let i = 0; i < 100; i++) {
  // ❌ Stream sin cancelación de suscripción
  const sub = interval(100).subscribe();
  leakyStreams.push(sub);
}

// Cancelar suscripción después de 10 segundos
setTimeout(() => {
  console.log('Cancelación de suscripción iniciada');
  leakyStreams.forEach(sub => sub.unsubscribe());
  console.log('Cancelación de suscripción completada');

  // Detener monitoreo después de otros 10 segundos
  setTimeout(() => {
    monitor.stop();
  }, 10000);
}, 10000);
```

## Mejores prácticas

### Establecer un entorno de depuración

Cómo habilitar logs de depuración solo en el entorno de desarrollo.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Determinar modo de depuración (ajustar según la herramienta de compilación)
const IS_DEVELOPMENT =
  // Al usar Vite: import.meta.env.DEV
  // Al usar webpack: process.env.NODE_ENV === 'development'
  // Configuración manual: definir variable global
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

function devLog<T>(label: string) {
  if (!IS_DEVELOPMENT) {
    return (source: Observable<T>) => source;
  }

  return tap<T>({
    next: value => console.log(`[${label}]`, value),
    error: error => console.error(`[${label}] Error:`, error),
    complete: () => console.log(`[${label}] Completado`)
  });
}

// Ejemplo de uso
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    devLog('Input'),
    map(x => x * 2),
    devLog('Output')
  )
  .subscribe();
// Sin logs en entorno de producción
```

### Depuración con seguridad de tipos

Este es un método de depuración que aprovecha el sistema de tipos de TypeScript.

```ts
import { tap } from 'rxjs';

type LogLevel = 'debug' | 'info' | 'warn' | 'error';

interface TypedDebugOptions<T> {
  label: string;
  level?: LogLevel;
  transform?: (value: T) => any;
  filter?: (value: T) => boolean;
}

function typedDebug<T>(options: TypedDebugOptions<T>) {
  const { label, level = 'debug', transform, filter } = options;

  const logFn = console[level] || console.log;

  return tap<T>({
    next: value => {
      if (filter && !filter(value)) return;

      const displayValue = transform ? transform(value) : value;
      logFn(`[${label}]`, displayValue);
    }
  });
}

// Ejemplo de uso
interface User {
  id: number;
  name: string;
  email: string;
}

import { of } from 'rxjs';

of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob', email: 'bob@example.com' },
  { id: 3, name: 'Charlie', email: 'charlie@example.com' }
)
  .pipe(
    typedDebug<User>({
      label: 'User Stream',
      level: 'info',
      transform: user => `${user.name} (${user.email})`,
      filter: user => user.id > 1
    })
  )
  .subscribe();

// Salida:
// [User Stream] Bob (bob@example.com)
// [User Stream] Charlie (charlie@example.com)
```

### Establecer límites de error

Aislar adecuadamente los errores para facilitar la depuración.

```ts
import { Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

function errorBoundary<T>(label: string) {
  return (source: Observable<T>) =>
    source.pipe(
      catchError(error => {
        console.error(`🔴 [${label}] Error capturado:`, {
          message: error.message,
          stack: error.stack,
          timestamp: new Date().toISOString()
        });

        // Volver a lanzar error o devolver valor de respaldo
        throw error;
      })
    );
}

// Ejemplo de uso
import { throwError } from 'rxjs';
import { mergeMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    errorBoundary('Proceso principal'),
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Error en valor 2'));
      }
      return of(value);
    }),
    errorBoundary('Proceso asíncrono')
  )
  .subscribe({
    next: value => console.log('Éxito:', value),
    error: error => console.log('Error final:', error.message)
  });
```

## Resumen

Depuración de rendimiento y mejores prácticas

### Monitoreo de rendimiento
- ✅ **Rastrear suscripciones** - administrar suscripciones usando defer y finalize
- ✅ **Detectar reevaluaciones** - evitar cálculos innecesarios con shareReplay
- ✅ **Monitoreo de memoria** - rastrear el uso de memoria con API de performance

### Optimizar el entorno de desarrollo
- ✅ **Configuración específica del entorno** - habilitar logs de depuración solo en entorno de desarrollo
- ✅ **Depuración con seguridad de tipos** - aprovechar el sistema de tipos de TypeScript
- ✅ **Límites de error** - aislar y depurar errores adecuadamente

Juntas, estas técnicas optimizan el rendimiento de las aplicaciones RxJS y crean un entorno de depuración eficiente.

## Páginas relacionadas

- [Estrategias básicas de depuración](/es/guide/debugging/) - Cómo usar el operador tap y las herramientas de desarrollo
- [Escenarios comunes de depuración](/es/guide/debugging/common-scenarios) - Solución de problemas específicos
- [Herramientas de depuración personalizadas](/es/guide/debugging/custom-tools) - Streams con nombre, operadores de depuración
- [Operador - shareReplay](/es/guide/operators/multicasting/shareReplay) - Evitar reevaluaciones innecesarias
