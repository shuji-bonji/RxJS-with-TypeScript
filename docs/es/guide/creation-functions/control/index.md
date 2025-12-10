---
description: Explicamos las Funciones de Creación de control de RxJS scheduled y using. scheduled controla el momento de ejecución del Observable especificando un scheduler, using gestiona automáticamente recursos como WebSocket y manejadores de archivos de acuerdo con el ciclo de vida del Observable. También es útil para pruebas y optimización del rendimiento.
---

# Funciones de Creación de Control

RxJS proporciona Funciones de Creación para controlar finamente el momento de ejecución del Observable y la gestión de recursos. Esta sección explica en detalle las dos funciones `scheduled()` y `using()`.

## ¿Qué son las Funciones de Creación de Control?

Las Funciones de Creación de Control son un grupo de funciones para controlar más finamente el comportamiento del Observable. Cubren casos de uso avanzados como control del momento de ejecución (scheduler) y gestión del ciclo de vida de recursos.

### Características Principales

- **Control del momento de ejecución**: Cambiar entre ejecución síncrona y asíncrona usando schedulers
- **Gestión de recursos**: Liberación automática de recursos de acuerdo con el ciclo de vida del Observable
- **Facilidad de pruebas**: Facilita las pruebas al cambiar schedulers
- **Optimización del rendimiento**: Evita el bloqueo de UI controlando el momento de ejecución

## Lista de Funciones de Creación de Control

| Función | Descripción | Usos Principales |
|------|------|---------|
| [scheduled()](/es/guide/creation-functions/control/scheduled) | Generar Observable especificando un scheduler | Control del momento de ejecución, pruebas |
| [using()](/es/guide/creation-functions/control/using) | Observable con control de recursos | Gestión de recursos como WebSocket, manejadores de archivos |

## Fundamentos de scheduled()

`scheduled()` es una función que puede especificar explícitamente un scheduler al generar un Observable desde fuentes de datos existentes (arrays, Promises, Iterables, etc.).

### Uso Básico

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Emitir array de forma asíncrona
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Inicio de suscripción');
observable$.subscribe({
  next: val => console.log('Valor:', val),
  complete: () => console.log('Completado')
});
console.log('Fin de suscripción');

// Salida:
// Inicio de suscripción
// Fin de suscripción
// Valor: 1
// Valor: 2
// Valor: 3
// Completado
```

> [!NOTE]
> Al usar `asyncScheduler`, la emisión de valores se vuelve asíncrona. Esto permite que el procesamiento de suscripción se ejecute sin bloquear el hilo principal.

## Fundamentos de using()

`using()` es una función que crea y libera automáticamente recursos de acuerdo con el ciclo de vida del Observable. Crea el recurso al inicio de la suscripción y lo libera automáticamente cuando termina (`complete` o `unsubscribe`).

### Uso Básico

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
> `using()` libera automáticamente recursos al terminar la suscripción, lo que puede prevenir fugas de memoria.

## Comparación scheduled() vs using()

| Característica | scheduled() | using() |
|------|-------------|---------|
| Propósito principal | Control del momento de ejecución | Gestión del ciclo de vida de recursos |
| Scheduler | ✅ Se puede especificar explícitamente | ❌ No se puede especificar |
| Gestión de recursos | ❌ Requiere gestión manual | ✅ Gestión automática |
| Escenarios de uso | Pruebas, optimización de UI | WebSocket, manejadores de archivos |
| Complejidad | Simple | Algo complejo |

## Guía de Selección

### Cuándo elegir scheduled()

1. **Quieres controlar el momento de ejecución**
   - Quieres cambiar procesamiento síncrono a asíncrono
   - Quieres evitar bloqueo de UI

2. **Necesitas control de tiempo en pruebas**
   - Combinar con TestScheduler para controlar el tiempo
   - Quieres probar procesamiento asíncrono de forma síncrona

3. **Convertir fuentes de datos existentes a Observable**
   - Convertir arrays, Promises, Iterables a Observable
   - Quieres especificar explícitamente un scheduler

### Cuándo elegir using()

1. **Necesitas liberación automática de recursos**
   - Gestión de conexiones WebSocket
   - Gestión de manejadores de archivos
   - Limpieza automática de temporizadores

2. **Quieres prevenir fugas de memoria**
   - Prevenir olvido de liberación de recursos
   - Limpieza confiable al terminar la suscripción

3. **Gestión de recursos compleja**
   - Gestionar múltiples recursos en bloque
   - Gestionar dependencias de recursos

## Ejemplos de Uso Práctico

### Ejemplo de uso de scheduled()

```typescript
import { scheduled, asyncScheduler, queueScheduler } from 'rxjs';

// Procesar grandes cantidades de datos de forma asíncrona (no bloquea UI)
const largeArray = Array.from({ length: 10000 }, (_, i) => i);
const async$ = scheduled(largeArray, asyncScheduler);

async$.subscribe(value => {
  // Ejecutar procesamiento pesado aquí
  // UI no se bloquea
});

// Ejecutar de forma síncrona en pruebas
const sync$ = scheduled(largeArray, queueScheduler);
```

### Ejemplo de uso de using()

```typescript
import { using, timer } from 'rxjs';

// Gestión automática de conexión WebSocket
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    console.log('Conexión WebSocket iniciada');
    return {
      unsubscribe: () => {
        ws.close();
        console.log('Conexión WebSocket terminada');
      }
    };
  },
  () => timer(0, 1000) // Recibir mensaje cada segundo
);
```

## Tipos de Scheduler (para scheduled())

| Scheduler | Descripción | Escenarios de uso |
|---------------|------|---------|
| `queueScheduler` | Ejecución síncrona (método de cola) | Por defecto, procesamiento síncrono |
| `asyncScheduler` | Ejecución asíncrona (setTimeout) | Optimización de UI, procesamiento largo |
| `asapScheduler` | Ejecución asíncrona más rápida (Promise) | Procesamiento asíncrono de alta prioridad |
| `animationFrameScheduler` | Frame de animación | Animación, renderizado de UI |

> [!TIP]
> Para más información sobre schedulers, consulta [Tipos de Schedulers](/es/guide/schedulers/types).

## Preguntas Frecuentes

### P1: ¿Cuál es la diferencia entre scheduled() y from()?

**R:** `from()` usa internamente el scheduler por defecto (síncrono). `scheduled()` puede especificar el scheduler explícitamente, permitiendo un control fino del momento de ejecución.

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - Ejecuta de forma síncrona
const sync$ = from([1, 2, 3]);

// scheduled() - Ejecuta de forma asíncrona
const async$ = scheduled([1, 2, 3], asyncScheduler);
```

### P2: ¿Cuándo debería usar using()?

**R:** Úsalo cuando quieras prevenir el olvido de liberación de recursos. Es especialmente efectivo en los siguientes casos:
- Conexiones de red como WebSocket, EventSource
- Manejadores de archivos, conexiones de base de datos
- Procesamiento que requiere `clearInterval()` o `clearTimeout()` manual

### P3: ¿Por qué scheduled() facilita las pruebas?

**R:** Usando TestScheduler, puedes controlar virtualmente el paso del tiempo. Puedes probar procesamiento asíncrono de forma síncrona, reduciendo significativamente el tiempo de ejecución de pruebas.

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled, asyncScheduler } from 'rxjs';

const testScheduler = new TestScheduler((actual, expected) => {
  expect(actual).toEqual(expected);
});

testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

## Mejores Prácticas

### 1. Evitar bloqueo de UI con scheduled()

```typescript
// ❌ Mal ejemplo: Procesar grandes cantidades de datos de forma síncrona
from(largeArray).subscribe(processHeavyTask);

// ✅ Buen ejemplo: Procesamiento asíncrono con asyncScheduler
scheduled(largeArray, asyncScheduler).subscribe(processHeavyTask);
```

### 2. Liberar recursos de forma confiable con using()

```typescript
// ❌ Mal ejemplo: Gestión manual de recursos
const ws = new WebSocket('wss://example.com');
const source$ = interval(1000);
source$.subscribe(() => ws.send('ping'));
// Fuga de recursos por olvido de unsubscribe

// ✅ Buen ejemplo: Gestión automática con using()
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return { unsubscribe: () => ws.close() };
  },
  () => interval(1000).pipe(tap(() => ws.send('ping')))
);
```

### 3. Usar schedulers apropiados en pruebas

```typescript
// ✅ Buen ejemplo: TestScheduler para pruebas
const testScheduler = new TestScheduler(...);
const source$ = scheduled([1, 2, 3], testScheduler);

// ✅ Buen ejemplo: asyncScheduler para producción
const source$ = scheduled([1, 2, 3], asyncScheduler);
```

## Resumen

Las Funciones de Creación de Control son funciones avanzadas para controlar finamente el comportamiento del Observable.

**scheduled():**
- Controla explícitamente el momento de ejecución (síncrono/asíncrono)
- Conveniente para control de tiempo en pruebas
- Efectivo para evitar bloqueo de UI

**using():**
- Gestiona automáticamente el ciclo de vida de recursos
- Previene fugas de memoria
- Óptimo para gestión de conexiones como WebSocket

Al usarlos apropiadamente, puedes construir aplicaciones RxJS más robustas y de alto rendimiento.

## Próximos Pasos

Para información detallada sobre cómo usar cada función, consulta las siguientes páginas:

- [Detalles de scheduled()](/es/guide/creation-functions/control/scheduled) - Generar Observable especificando scheduler
- [Detalles de using()](/es/guide/creation-functions/control/using) - Observable con control de recursos

## Recursos de Referencia

- [Documentación Oficial RxJS - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [Documentación Oficial RxJS - using](https://rxjs.dev/api/index/function/using)
- [Documentación Oficial RxJS - Scheduler](https://rxjs.dev/guide/scheduler)
- [Tipos de Schedulers](/es/guide/schedulers/types)
