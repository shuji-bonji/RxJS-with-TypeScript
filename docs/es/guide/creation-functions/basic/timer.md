---
description: "timer() - Función de Creación que comienza a emitir después de un retraso especificado: Perfecta para ejecución retrasada, polling con retraso e implementaciones de timeout"
---

# timer() - Comenzar a Emitir Después de un Retraso

`timer()` es una Función de Creación que comienza a emitir valores después de un tiempo de retraso especificado, soportando tanto emisión única como periódica.

## Resumen

`timer()` es una Función de Creación flexible que te permite controlar el momento de la primera emisión. Su comportamiento cambia dependiendo del número de argumentos, y puede usarse tanto para emisión única como para emisión periódica como `interval()`.

**Firma**:
```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

**Documentación Oficial**: [📘 RxJS Oficial: timer()](https://rxjs.dev/api/index/function/timer)

## Uso Básico

El comportamiento de `timer()` depende del número de argumentos.

### Emisión Única

Si solo se especifica el primer argumento, emite 0 después del tiempo especificado y se completa.

```typescript
import { timer } from 'rxjs';

// Emitir 0 después de 3 segundos y completar
const timer$ = timer(3000);

timer$.subscribe({
  next: value => console.log('Valor:', value),
  complete: () => console.log('Completado')
});

// Salida después de 3 segundos:
// Valor: 0
// Completado
```

### Emisión Periódica

Si se especifica un intervalo para el segundo argumento, continuará emitiendo periódicamente después del retraso inicial.

```typescript
import { timer } from 'rxjs';

// Comenzar después de 3 segundos, luego emitir valores cada 1 segundo
const timer$ = timer(3000, 1000);

timer$.subscribe(value => console.log('Valor:', value));

// Salida:
// Valor: 0  (después de 3 segundos)
// Valor: 1  (después de 4 segundos)
// Valor: 2  (después de 5 segundos)
// ... (continúa infinitamente)
```

## Características Importantes

### 1. Especificación Flexible de Retrasos

El retraso puede especificarse como un número en milisegundos o como un objeto `Date`.

```typescript
import { timer } from 'rxjs';

// Especificar en milisegundos
timer(5000).subscribe(() => console.log('Después de 5 segundos'));

// Especificar con objeto Date (ejecutar en tiempo específico)
const targetTime = new Date(Date.now() + 10000); // 10 segundos después
timer(targetTime).subscribe(() => console.log('Ejecutar en tiempo especificado'));
```

### 2. El Comportamiento Cambia Dependiendo del Segundo Argumento

Si se especifica o no el segundo argumento determina si se completa.

```typescript
import { timer } from 'rxjs';

// Sin segundo argumento - emitir una vez y completar
timer(1000).subscribe({
  next: value => console.log('Una vez:', value),
  complete: () => console.log('Completado')
});

// Con segundo argumento - emitir infinitamente
timer(1000, 1000).subscribe({
  next: value => console.log('Repetir:', value),
  complete: () => console.log('Completado (no se muestra)')
});
```

> [!IMPORTANT]
> **Con Segundo Argumento, No se Completa**
>
> Si especificas el segundo argumento como `timer(1000, 1000)`, seguirá emitiendo indefinidamente, igual que `interval()`. La desuscripción siempre es requerida.

### 3. Cold Observable

`timer()` es un Cold Observable, lo que significa que se crea un temporizador independiente para cada suscripción.

```typescript
import { timer } from 'rxjs';

const timer$ = timer(1000);

console.log('Inicio');

// Suscripción 1
timer$.subscribe(() => console.log('Observador 1'));

// Añadir suscripción 2 después de 500ms
setTimeout(() => {
  timer$.subscribe(() => console.log('Observador 2'));
}, 500);

// Salida:
// Inicio
// Observador 1  (después de 1 segundo)
// Observador 2  (después de 1.5 segundos - temporizador independiente)
```

> [!NOTE]
> **Características de Cold Observable**:
> - Se inicia una ejecución independiente para cada suscripción
> - Cada suscriptor recibe su propio flujo de datos
> - Se inicia un temporizador independiente para cada suscripción; como con `interval()`, usa `share()` si se requiere compartir
>
> Ver [Cold Observable y Hot Observable](/es/guide/observables/cold-and-hot-observables) para más información.

## Diferencia Entre timer() e interval()

La principal diferencia entre los dos es el momento de la primera emisión.

```typescript
import { timer, interval } from 'rxjs';
import { take } from 'rxjs';

console.log('Inicio');

// interval() - comienza inmediatamente (primer valor después de 1 segundo)
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - sin retraso (primer valor inmediatamente)
timer(0, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// timer() - comienza después de retraso de 2 segundos
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer(delay):', value);
});
```

| Función de Creación | Momento de Primera Emisión | Propósito |
|-------------------|----------------------|---------|
| `interval(1000)` | Después de 1 segundo | Iniciar ejecución periódica inmediatamente |
| `timer(0, 1000)` | Inmediatamente | Quiere primera ejecución inmediatamente |
| `timer(2000, 1000)` | Después de 2 segundos | Ejecución periódica después de retraso |
| `timer(2000)` | Después de 2 segundos (solo una vez) | Ejecución retrasada (única) |

## Casos de Uso Prácticos

### 1. Ejecución Retrasada

Ejecutar un proceso solo una vez después de cierto período de tiempo.

```typescript
import { from, timer } from 'rxjs';
import { switchMap } from 'rxjs';

function delayedApiCall() {
  return timer(2000).pipe(
    switchMap(() => from(
      fetch('https://jsonplaceholder.typicode.com/posts/1')
        .then(res => res.json())
    ))
  );
}

delayedApiCall().subscribe(data => {
  console.log('Obtener datos después de 2 segundos:', data);
});
```

### 2. Polling con Retraso

Iniciar polling después de cierto período de tiempo en lugar de ejecutar inmediatamente la primera vez.

```typescript
import { from, timer } from 'rxjs';
import { switchMap, retry } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

// Iniciar polling después de 5 segundos, luego cada 10 segundos
const polling$ = timer(5000, 10000).pipe(
  switchMap(() => from(
    fetch('https://jsonplaceholder.typicode.com/users/1')
      .then(res => res.json() as Promise<Status>)
  )),
  retry(3) // Reintentar hasta 3 veces en caso de error
);

const subscription = polling$.subscribe(data => {
  console.log('Actualización de estado:', data);
});

// Detener según sea necesario
// subscription.unsubscribe();
```

### 3. Procesamiento de Timeout

Timeout ocurre cuando el procesamiento no se completa dentro de cierto período de tiempo.

```typescript
import { timer, race, from } from 'rxjs';
import { map } from 'rxjs';

function fetchWithTimeout(url: string, timeoutMs: number) {
  const request$ = from(fetch(url).then(res => res.json()));
  const timeout$ = timer(timeoutMs).pipe(
    map(() => {
      throw new Error('Timeout');
    })
  );

  // Usar el que llegue primero
  return race(request$, timeout$);
}

fetchWithTimeout('https://jsonplaceholder.typicode.com/posts/1', 3000).subscribe({
  next: data => console.log('Obtener datos:', data),
  error: err => console.error('Error:', err.message)
});
```

### 4. Notificaciones que se Ocultan Automáticamente

Ocultar notificaciones automáticamente después de cierto período de tiempo después de mostrarse.

```typescript
import { timer, Subject, map } from 'rxjs';
import { switchMap, takeUntil } from 'rxjs';

interface Notification {
  id: number;
  message: string;
}

const notifications$ = new Subject<Notification>();
const dismiss$ = new Subject<number>();

notifications$.pipe(
  switchMap(notification => {
    console.log('Mostrar notificación:', notification.message);

    // Auto-ocultar después de 5 segundos
    return timer(5000).pipe(
      takeUntil(dismiss$), // Cancelar si se descarta manualmente
      map(() => notification.id)
    );
  })
).subscribe(id => {
  console.log('Ocultar notificación:', id);
});

// Mostrar notificación
notifications$.next({ id: 1, message: 'Nuevo mensaje recibido' });

// Para descartar manualmente
// dismiss$.next(1);
```

## Uso en Pipeline

`timer()` se usa como punto de partida para procesamiento retrasado o ejecución periódica.

```typescript
import { timer } from 'rxjs';
import { map, take, scan } from 'rxjs';

// Temporizador de cuenta regresiva (de 10 segundos a 0 segundos)
timer(0, 1000).pipe(
  map(count => 10 - count),
  take(11), // De 0 a 10 (11 valores)
  scan((acc, curr) => curr, 0)
).subscribe({
  next: time => console.log(`Restante: ${time} segundos`),
  complete: () => console.log('Temporizador terminado')
});

// Salida:
// Restante: 10 segundos
// Restante: 9 segundos
// ...
// Restante: 0 segundos
// Temporizador terminado
```

## Errores Comunes

### 1. Olvidar Desuscribirse con Segundo Argumento

```typescript
// ❌ Incorrecto - se ejecuta infinitamente con segundo argumento
import { timer } from 'rxjs';

function startTimer() {
  timer(1000, 1000).subscribe(value => {
    console.log('Valor:', value); // Se ejecuta para siempre
  });
}

startTimer();

// ✅ Correcto - mantener suscripción y desuscribirse según sea necesario
import { timer, Subscription } from 'rxjs';
import { take } from 'rxjs';

let subscription: Subscription | null = null;

function startTimer() {
  subscription = timer(1000, 1000).pipe(
    take(10) // Auto-completar después de 10 veces
  ).subscribe(value => {
    console.log('Valor:', value);
  });
}

function stopTimer() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startTimer();
```

### 2. No Entender la Diferencia con interval()

```typescript
// ❌ Confusión - interval() comienza inmediatamente (primer valor después de 1 segundo)
import { interval } from 'rxjs';

interval(1000).subscribe(value => {
  console.log('interval:', value); // 0 emitido después de 1 segundo
});

// ✅ timer() - cuando quieres emitir el primer valor inmediatamente sin retraso
import { timer } from 'rxjs';

timer(0, 1000).subscribe(value => {
  console.log('timer:', value); // 0 emitido inmediatamente
});
```

## Consideraciones de Rendimiento

Aunque `timer()` es ligero, su uso puede afectar el rendimiento.

> [!TIP]
> **Consejos de Optimización**:
> - No especificar segundo argumento para ejecución única
> - Siempre desuscribirse cuando ya no se necesita
> - Si se necesitan múltiples Observers, compartirlos con `share()`
> - Usar intervalos cortos (menos de 100ms) con precaución

```typescript
import { timer } from 'rxjs';
import { share } from 'rxjs';

// ❌ Problema de rendimiento - múltiples temporizadores independientes
const timer$ = timer(0, 1000);

timer$.subscribe(value => console.log('Observador 1:', value));
timer$.subscribe(value => console.log('Observador 2:', value));
// Dos temporizadores se ejecutan en paralelo

// ✅ Optimización - compartir un temporizador
const sharedTimer$ = timer(0, 1000).pipe(share());

sharedTimer$.subscribe(value => console.log('Observador 1:', value));
sharedTimer$.subscribe(value => console.log('Observador 2:', value));
// Se comparte un temporizador
```

## Funciones de Creación Relacionadas

| Función | Diferencia | Uso |
|----------|------|----------|
| **[interval()](/es/guide/creation-functions/basic/interval)** | Comienza inmediatamente (sin retraso) | Ejecución periódica sin retraso |
| **[of()](/es/guide/creation-functions/basic/of)** | Emitir síncronamente e inmediatamente | Cuando no se necesita asíncrono |
| **defer()** | Diferir procesamiento hasta suscripción | Generación dinámica de valores |

## Resumen

- `timer()` es una Función de Creación que comienza a emitir después de un retraso
- Sin segundo argumento: emisión única (se completa)
- Con segundo argumento: emisión periódica (no se completa)
- El tiempo de retraso puede especificarse en milisegundos o como objeto `Date`
- Ideal para ejecución retrasada, polling con retraso, procesamiento de timeout

## Próximos Pasos

- [interval() - Emisión Continua a Intervalos Especificados](/es/guide/creation-functions/basic/interval)
- [defer() - Diferir Generación Hasta Suscripción](/es/guide/creation-functions/conditional/defer)
- [Volver a Funciones de Creación Básicas](/es/guide/creation-functions/basic/)
