---
description: Esta página explica en detalle las características, implementaciones y aplicaciones de los principales schedulers en RxJS, como asyncScheduler y queueScheduler. Comprenda las diferencias entre macrotareas, microtareas y procesamiento síncrono, y aprenda el tiempo de ejecución y las características de cada scheduler. Al usarlos correctamente, puede optimizar el rendimiento y comportamiento de su aplicación.
---

# Tipos de Schedulers y Cómo Usarlos

RxJS proporciona múltiples schedulers para diferentes aplicaciones. Cada scheduler tiene su propio tiempo de ejecución y características específicas, y el uso apropiado de cada uno puede optimizar el rendimiento y el comportamiento de su aplicación.

## Clasificación de Schedulers

Los schedulers de RxJS se dividen en tres categorías principales.

1. **Macrotarea**: ejecutada en la siguiente cola de tareas en el event loop
2. **Microtarea**: ejecutada inmediatamente después de que la tarea actual se completa y antes de que comience la siguiente tarea
3. **Procesamiento síncrono**: ejecución inmediata

Para más información, consulte [Fundamentos de Tareas y Schedulers](./task-and-scheduler-basics.md) para más detalles.

## Principales schedulers

### asyncScheduler

#### Características
- **Implementación interna**: usa setTimeout
- **Tiempo de ejecución**: macrotareas
- **Uso**: Procesamiento asíncrono general, procesamiento con transcurso de tiempo

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inicio');

of('Procesamiento asíncrono')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fin');

// Salida:
// 1: Inicio
// 2: Fin
// 3: Procesamiento asíncrono
```

#### Casos de Uso

Este ejemplo simula un proceso de cálculo pesado.

```ts
import { asyncScheduler, map, observeOn, of } from "rxjs";

function heavyComputation(value: number): number {
  // Simular cálculo pesado
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result = Math.sin(result);
  }
  return result;
}

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(value => heavyComputation(value))
  )
  .subscribe(result => {
    console.log(`Resultado del cálculo: ${result}`);
  });
```

### queueScheduler

#### Características
- **Implementación interna**: cola de microtareas
- **Tiempo de ejecución**: dentro de la tarea actual (parece síncrono)
- **Uso**: Puesta en cola de tareas, optimización de recursión

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inicio');

of('Procesamiento en cola')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: Fin');

// Salida:
// 1: Inicio
// 2: Procesamiento en cola
// 3: Fin
```

#### Casos de Uso

Este es un ejemplo de optimizar un proceso recursivo.

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs';

// Optimización del procesamiento recursivo
function fibonacci(n: number): Observable<number> {
  return of([0, 1]).pipe(
    observeOn(queueScheduler),
    expand(([a, b]) => of([b, a + b])),
    map(([a]) => a),
    take(n)
  );
}

fibonacci(10).subscribe(value => console.log(value));
```

### asapScheduler

#### Características
- **Implementación interna**: Promise.resolve().then() o setImmediate
- **Tiempo de ejecución**: microtareas
- **Uso**: Para ejecución asíncrona lo antes posible

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inicio');

of('Procesamiento ASAP')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fin');

// Salida:
// 1: Inicio
// 2: Fin
// 3: Procesamiento ASAP
```

#### Casos de Uso

Este es un ejemplo de optimizar eventos de movimiento del mouse.

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Optimización de eventos de movimiento del mouse
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // Procesamiento de actualización de UI
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Características
- **Implementación interna**: requestAnimationFrame
- **Tiempo de ejecución**: antes del siguiente renderizado de pantalla
- **Uso**: Animación, proceso de dibujo para 60fps

#### Ejemplo de una animación de rotación simple

Este es un ejemplo de rotar un elemento circular en HTML.

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// Crear elemento HTML
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Configuración de animación
let rotation = 0;

// Animar a 60fps durante 2 segundos
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2 segundos = 120 frames
    map(() => {
      rotation += 3;  // Rotar 3 grados por frame
      return rotation;
    })
  )
  .subscribe(angle => {
    // Realmente rotar el elemento DOM
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### ¿Por qué animationFrameScheduler?

El `animationFrameScheduler` se ejecuta de forma síncrona con el ciclo de dibujo del navegador, lo que ofrece las siguientes ventajas

1. **Animación Suave**: Debido a que el procesamiento se realiza en sincronía con el tiempo de renderizado del navegador (típicamente 60 fps), se puede lograr una animación suave sin ningún salto.
2. **Uso eficiente de recursos**: Cuando el navegador desactiva la pestaña, la ejecución de requestAnimationFrame se pausa automáticamente para evitar un uso innecesario de CPU.
3. **Anti-parpadeo**: Asegura que el cálculo se complete antes de que se dibuje la pantalla, evitando parpadeos y mostrando frames incompletos.

Lo siguiente es una comparación de `setInterval` y `animationFrameScheduler`.

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ Animación ineficiente usando setInterval
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // aprox. 60fps

// Problemas:
// - No sincronizado con el tiempo de renderizado del navegador
// - Continúa ejecutándose incluso en pestañas de fondo
// - Incapaz de garantizar 60fps precisos

// ✅ Animación eficiente usando animationFrameScheduler
interval(0, animationFrameScheduler)
  .pipe(
    map(() => {
      position += 1;
      return position;
    })
  )
  .subscribe(pos => {
    element.style.transform = `translateX(${pos}px)`;
  });

// Beneficios
// - Se sincroniza con el tiempo de renderizado del navegador
// - Se pausa automáticamente en pestañas de fondo
// - Logra 60fps estables
```


#### Ejemplo de animación que sigue al mouse

Crear una animación de círculo que sigue el cursor del mouse.

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Crear un círculo que sigue
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Dejar pasar eventos del mouse
document.body.appendChild(circle);

// Posiciones actuales y objetivo
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Monitorear eventos de movimiento del mouse
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove')
  .pipe(
    map(event => ({
      x: event.clientX,
      y: event.clientY
    }))
  );

// Bucle de animación
interval(0, animationFrameScheduler)
  .pipe(
    withLatestFrom(mouseMove$),
    map(([_, mousePos]) => mousePos)
  )
  .subscribe(({ x, y }) => {
    // Establecer posición del mouse como objetivo
    targetX = x;
    targetY = y;

    // Moverse gradualmente de la posición actual a la posición objetivo (easing)
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;

    // Actualizar elemento DOM
    circle.style.left = `${currentX - 15}px`;  // Ajustar para posición central
    circle.style.top = `${currentY - 15}px`;
  });
```

## Guía para usar schedulers

### Comparación por tiempo de ejecución

El siguiente es un ejemplo que compara el orden de ejecución de cada scheduler.

```ts
import { of, asyncScheduler, queueScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inicio');

// Procesamiento síncrono
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler (microtarea)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler (microtarea)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler (macrotarea)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(v => console.log(`5: ${v}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Fin');

// Orden de ejecución:
// 1: Inicio
// 2: sync
// 7: Fin
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```

### Criterios de Selección por Uso

El siguiente es un resumen de las características y aplicaciones adecuadas de cada scheduler.

| Scheduler | Características | Usos Adecuados |
|--------------|------|----------|
| asyncScheduler | Usa setTimeout, totalmente asíncrono | Procesamiento que consume tiempo, ejecución retrasada |
| queueScheduler | Síncrono pero optimiza la recursión | Procesamiento recursivo, gestión de cola de tareas |
| asapScheduler | Ejecución asíncrona lo más rápido posible | Manejo de eventos, procesamiento de respuesta rápida |
| animationFrameScheduler | Sincronizado con el renderizado de pantalla | Animación, actualizaciones de UI, desarrollo de juegos |

## Casos de uso prácticos

### Procesamiento de grandes cantidades de datos

Este es un ejemplo de poner en cola solicitudes y procesarlas en orden.

```ts
import { from, queueScheduler } from 'rxjs';
import { mergeMap, observeOn, tap } from 'rxjs';

interface ApiRequest {
  endpoint: string;
  id: number;
}

const requests: ApiRequest[] = [
  { endpoint: '/users', id: 1 },
  { endpoint: '/posts', id: 1 },
  { endpoint: '/comments', id: 1 },
];

// Poner en cola solicitudes y procesarlas en orden
from(requests)
  .pipe(
    observeOn(queueScheduler),
    tap((req) => console.log(`Agregado a la cola: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // Simular solicitud de API real
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`${req.endpoint}/${req.id} resultado`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`Completado: ${result}`));
```

### Manejo de mensajes WebSocket

Este es un ejemplo de procesamiento de mensajes WebSocket que requiere una respuesta rápida.

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// Nota: Este es pseudo-código para ilustrar el concepto
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // Tratar como string
});

socket$
  .pipe(
    // Procesamiento de mensajes que requiere respuesta rápida
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('Mensaje recibido:', msg);
}
```

### Control de reintentos de errores

Al utilizar el scheduler con el operador `retry`, el tiempo de los reintentos puede controlarse finamente.

#### Control básico de reintentos

La opción `delay` del operador `retry` usa internamente el `asyncScheduler` para controlar el intervalo de reintento.

```ts
import { throwError, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

// Simulación de llamada de API
function fetchData(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.7) {
        return of({ id, data: 'éxito' });
      }
      return throwError(() => new Error('Error de red'));
    })
  );
}

fetchData(1)
  .pipe(
    retry({
      count: 3,
      delay: 1000  // Esperar 1 segundo con asyncScheduler antes de reintentar
    })
  )
  .subscribe({
    next: result => console.log('✅ Éxito:', result),
    error: error => console.log('❌ Error final:', error.message)
  });
```

#### Utilización del scheduler en exponential back-off

Para un control más avanzado, se puede implementar exponential backoff combinando `retryWhen` y `asyncScheduler`.

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, mergeMap, tap } from 'rxjs';

function fetchDataWithBackoff(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.9) {
        return of({ id, data: 'éxito' });
      }
      return throwError(() => new Error('Error temporal'));
    })
  );
}

fetchDataWithBackoff(1)
  .pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;

          // Verificar conteo máximo de reintentos
          if (retryCount > 3) {
            console.log('❌ Conteo máximo de reintentos alcanzado');
            throw error;
          }

          // Exponential backoff: 1 segundo, 2 segundos, 4 segundos...
          const delayTime = Math.pow(2, index) * 1000;
          console.log(`🔄 Reintentar ${retryCount} veces (después de ${delayTime}ms)`);

          // timer usa internamente asyncScheduler
          return timer(delayTime);
        })
      )
    )
  )
  .subscribe({
    next: result => console.log('✅ Éxito:', result),
    error: error => console.log('❌ Error final:', error.message)
  });

// Salida de ejemplo:
// 🔄 Reintentar 1 veces (después de 1000ms)
// 🔄 Reintentar 2 veces (después de 2000ms)
// 🔄 Reintentar 3 veces (después de 4000ms)
// ❌ Conteo máximo de reintentos alcanzado
// ❌ Error final: Error temporal
```

#### Cuando asyncScheduler se especifica explícitamente

Especificar explícitamente un scheduler específico permite un control más flexible, como reemplazarlo con `TestScheduler` durante las pruebas.

```ts
import { throwError, asyncScheduler, of } from 'rxjs';
import { retryWhen, mergeMap, delay } from 'rxjs';

function fetchDataWithScheduler(id: number, scheduler = asyncScheduler) {
  return of(id).pipe(
    mergeMap(() => throwError(() => new Error('Error'))),
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          if (index >= 2) throw error;

          // Especificar scheduler explícitamente
          return of(null).pipe(
            delay(1000, scheduler)
          );
        })
      )
    )
  );
}

// Entorno de producción: usar asyncScheduler
fetchDataWithScheduler(1).subscribe({
  error: err => console.log('Error:', err.message)
});

// Entorno de prueba: puede ser reemplazado con TestScheduler
```

> [!TIP]
> Para patrones de implementación detallados y métodos de depuración para procesamiento de reintentos, consulte la página [retry y catchError](/es/guide/error-handling/retry-catch).
> - Uso detallado del operador retry
> - Patrones de combinación con catchError
> - Técnicas de depuración de reintentos (seguimiento del número de intentos, registro, etc.)

## Impacto en el Rendimiento

### Sobrecarga del scheduler

Este es un ejemplo de cómo evitar el uso excesivo del scheduler y optimizar para procesamiento por lotes.

```ts
import { range, asyncScheduler, pipe } from 'rxjs';
import { bufferCount, map, observeOn, tap } from 'rxjs';

// ❌ Uso excesivo del scheduler
range(1, 1000)
  .pipe(
    observeOn(asyncScheduler),  // 1000 setTimeouts
    map(x => x * 2),
    // tap(console.log)
  )
  .subscribe();

// ✅ Optimizar con procesamiento por lotes
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeouts
    map(batch => batch.map(x => x * 2)),
    // tap(console.log)
  )
  .subscribe();
```

## Resumen

La elección del scheduler tiene un impacto significativo en el rendimiento y la capacidad de respuesta de la aplicación. Comprender las características de cada scheduler y usarlos en situaciones apropiadas asegurará un funcionamiento eficiente y fluido. Como guía general,

- Para procesamiento asíncrono general, use `asyncScheduler`
- `queueScheduler` para procesamiento recursivo y puesta en cola síncrona
- `asapScheduler` para tiempos de respuesta rápidos
- `animationFrameScheduler` para animación

se recomienda.
