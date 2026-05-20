---
description: "Obtenga más información sobre las características, la implementación y el uso de los principales planificadores de RxJS, como asyncScheduler y queueScheduler. Comprenda las diferencias entre macrotareas, microtareas y procesamiento síncrono, y aprenda los tiempos de ejecución y las características de cada planificador. Podrá optimizar el rendimiento y el comportamiento de su aplicación utilizándolos adecuadamente."
---

# Tipos de programadores y cómo utilizarlos

RxJS proporciona varios planificadores para diferentes aplicaciones. Cada planificador tiene su propio tiempo de ejecución y características específicas y puede ser utilizado adecuadamente para optimizar el rendimiento y el comportamiento de la aplicación.

## Clasificación de los planificadores

Los planificadores de RxJS se dividen en tres categorías principales.

1. **Macrotareas**: ejecutadas en la siguiente cola de tareas en el bucle de eventos
2. **Micro-tareas**: ejecutadas inmediatamente después de que la tarea actual se complete y antes de que comience la siguiente tarea
3.**Procesamiento síncrono**: ejecución inmediata

Para más información, consulte [Conceptos básicos de tareas y programadores](./task-and-scheduler-basics.md).

## Planificadores principales.

### asyncScheduler

#### Características.
- Implementación interna**: utiliza setTimeout.
- **Tiempo de ejecución**: macro tareas.
- **Uso**: Procesamiento asíncrono general, procesamiento time-lapse.

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

#### Caso de uso

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
    console.log(`Resultados del cálculo: ${result}`);
  });
```

### queueScheduler

#### Características.
- Implementación interna**: micro cola de tareas
- **Tiempo de ejecución**: dentro de la tarea actual (parece síncrono)
- Usos**: cola de tareas, optimización de la recursividad

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

#### Caso de uso

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

#### Características.
- **Implementación interna**: Promise.resolve().then() o setImmediate
- **Tiempo de ejecución**: microtareas
- **Uso**: Para una ejecución asíncrona lo más rápida posible

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inicio');

of('ASAPProcesamiento')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fin');

// Salida:
// 1: Inicio
// 2: Fin
// 3: ASAPProcesamiento
```

#### Caso de uso

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Optimización de los movimientos del ratón
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // UIProcesamiento de actualización de
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Características.
- Implementación interna**: requestAnimationFrame
- **Tiempo de ejecución**: antes del siguiente renderizado de pantalla
- **Uso**: Animación, proceso de dibujo para 60fps.

#### Ejemplo de una simple animación de rotación

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// HTMLCreación de elementos
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Creación de animaciones
let rotation = 0;

// 60fpsEn los2Segundos de animación
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2Segundos = 120Fotograma
    map(() => {
      rotation += 3;  // 1Cada fotograma3Grado de rotación
      return rotation;
    })
  )
  .subscribe(angle => {
    // DOMRotación real del elemento
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### ¿Por qué necesitamos animationFrameScheduler?

El `animationFrameScheduler` realiza sus operaciones de forma sincronizada con el ciclo de dibujo del navegador, lo que tiene las siguientes ventajas.

1.**Animaciones suaves**: como el procesamiento está sincronizado con el tiempo de renderizado del navegador (normalmente 60 fps), se consiguen animaciones suaves y sin cortes.
2.**Uso eficiente de los recursos**: cuando el navegador desactiva la pestaña, la ejecución del requestAnimationFrame se pausa automáticamente, evitando el uso innecesario de la CPU.
3, **Prevención del parpadeo de la pantalla**: se evita el parpadeo de la pantalla y los fotogramas incompletos porque el cálculo se completa de forma fiable antes de que se dibuje la pantalla.

He aquí una comparación entre `setInterval` y `animationFrameScheduler`.

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ setIntervalAnimación ineficiente utilizando
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // aproximadamente60fps

// Problemas:
// - No se sincroniza con los tiempos de renderizado del navegador
// - Continúa ejecutándose incluso en pestañas en segundo plano
// - No se puede garantizar la precisión de60fpsno se puede garantizar

// ✅ animationFrameSchedulerUso eficiente de la animación
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

// Ventajas
// - Sincronizado con los tiempos de dibujo del navegador
// - Pausa automática en las pestañas de fondo
// - Estable60fpsPermite
```

#### Ejemplo de animación siguiendo al ratón

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Creación de un círculo a seguir
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Transparente a los eventos del ratón
document.body.appendChild(circle);

// Posición actual y de destino
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Monitoriza los eventos de movimiento del ratón
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
    // Establecer la posición del ratón como objetivo
    targetX = x;
    targetY = y;
    
    // Movimiento gradual desde la posición actual hacia la posición de destino々Movimiento gradual (easing) desde la posición actual hacia la posición objetivo
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;
    
    // DOMActualizar elemento
    circle.style.left = `${currentX - 15}px`;  // Ajuste a la posición central
    circle.style.top = `${currentY - 15}px`;
  });
```

## Guía para el uso de diferentes programadores

### Comparación por tiempo de ejecución

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

### Criterios de selección por aplicación


## Casos prácticos de uso

### Procesamiento de grandes cantidades de datos


### Procesamiento de mensajes WebSocket.

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// 注: これは概念を示す疑似コードです
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // 文字列として扱う
});

socket$
  .pipe(
    // 高速な応答が必要なメッセージ処理
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('メッセージ受信:', msg);
}
```

#### Utilización del programador con retardo exponencial

Como control más avanzado, el backoff exponencial puede implementarse combinando `retryWhen` y `asyncScheduler`.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

function fetchDataWithBackoff(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.9) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Temporary error'));
    })
  );
}

fetchDataWithBackoff(1)
  .pipe(
    // RxJS 7.3+ 推奨: retry({ count, delay }) 形式
    retry({
      count: 3, // Máx.3回まで再試行
      delay: (error, retryCount) => {
        // 指数バックオフ: 1秒, 2秒, 4秒...
        const delayTime = Math.pow(2, retryCount - 1) * 1000;
        console.log(`🔄 リトライ ${retryCount}回目 (${delayTime}ms後)`);

        // timer は内部的に asyncScheduler を使用
        return timer(delayTime);
      }
    })
  )
  .subscribe({
    next: result => console.log('✅ 成功:', result),
    error: error => {
      console.log('❌ Máx.リトライ数に到達');
      console.log('❌ 最終Error:', error.message);
    }
  });

// SalidaEj.:
// 🔄 リトライ 1回目 (1000ms後)
// 🔄 リトライ 2回目 (2000ms後)
// 🔄 リトライ 3回目 (4000ms後)
// ❌ Máx.リトライ数に到達
// ❌ 最終Error: Temporary error
```

#### Cuando se especifica explícitamente un Scheduler asíncrono

Especificar explícitamente un Scheduler específico permite un control más flexible, como sustituirlo por `TestScheduler` durante las pruebas.


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


> - En la página [retry y catchError](/es/guide/error-handling/retry-catch) se describen patrones de implementación detallados y métodos de depuración para el manejo de retry.
> - Uso detallado del operador retry.
> - Patrones de combinación con catchError.
> - Técnicas de depuración para reintentos (por ejemplo, seguimiento del número de intentos, registro, etc.).

## Impacto en el rendimiento.

### Sobrecarga del programador


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

## Resumen.

La elección del planificador tiene un impacto significativo en el rendimiento y la capacidad de respuesta de una aplicación. Comprender las características de cada planificador y utilizarlos en las situaciones adecuadas garantizará un funcionamiento eficiente y sin problemas. Como pauta general,

- Scheduler` para el procesamiento asíncrono en general.
- Scheduler para el procesamiento recursivo y las colas síncronas.
- Scheduler` para tiempos de respuesta rápidos.
- `animationFrameScheduler` para animación

para animación.