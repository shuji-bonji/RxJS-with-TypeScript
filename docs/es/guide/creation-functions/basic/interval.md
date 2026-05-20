---
description: "interval() - Creation Function que emite continuamente valores (números secuenciales empezando por 0) a intervalos especificados, ideal para sondeo, ejecución periódica y control de animación; diferencias con timer(), cómo limitar con take(), implementación segura de tipo en TypeScript, fugas de memoria. Explica el patrón unsubscribe para evitar fugas de memoria."
---

# interval() - emisión continua a intervalos especificados

interval()` es una Creation Function que emite continuamente valores a intervalos de tiempo especificados.

## Visión general.

Interval()` emite continuamente valores continuos a partir de 0 en intervalos de milisegundos especificados. Se utiliza frecuentemente para procesos de sondeo y ejecución periódica de tareas.

**Firma**:.

```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

**Documentación oficial**: [📘 Fórmula RxJS: interval()](https://rxjs.dev/api/index/function/interval)

## Uso básico

`interval()` emite un número que cuenta hacia arriba en un intervalo especificado.

```typescript
import { interval } from 'rxjs';

// 1Valor emitido cada segundo
const interval$ = interval(1000);

interval$.subscribe(value => {
  console.log('Valor:', value);
});

// Salida (en1(cada segundo):
// Valor: 0
// Valor: 1
// Valor: 2
// Valor: 3
// ...(continúa indefinidamente)
```

## Características importantes.

### 1. Números consecutivos empezando por 0

interval()` emite un número entero que siempre empieza en 0 y se incrementa en 1.

```typescript
import { interval } from 'rxjs';
import { take } from 'rxjs';

interval(500).pipe(
  take(5) // Primer5Sólo se recuperan los dos primeros valores
).subscribe(value => console.log(value));

// Salida (en500ms(por segundo):
// 0
// 1
// 2
// 3
// 4
```

### 2. no completado (flujo infinito)

`interval()` no se completa automáticamente y **debe darse de baja**.

```typescript
import { interval } from 'rxjs';

const subscription = interval(1000).subscribe(value => {
  console.log('Valor:', value);
});

// 5Desuscribirse después de 2 segundos
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Desuscrito');
}, 5000);
```

> [!WARNING]

> **Fuga de memoria si olvidas darte de baja**
>
> > Dado que `interval()` sigue emitiendo valores indefinidamente, olvidarse de darse de baja puede causar fugas de memoria y problemas de rendimiento. Asegúrese de llamar a `unsubscribe()` o utilizar operadores como `take()`, `takeUntil()` o `takeWhile()` para completar automáticamente.

### 4. Cold Observable.

Interval()` es un Cold Observable, que crea un temporizador independiente para cada suscripción.

```typescript
import { interval } from 'rxjs';

const interval$ = interval(1000);

// Suscrito1
interval$.subscribe(value => console.log('Observer 1:', value));

// 2Suscrito después de segundos2Añadido
setTimeout(() => {
  interval$.subscribe(value => console.log('Observer 2:', value));
}, 2000);

// Salida:
// Observer 1: 0
// Observer 1: 1
// Observer 2: 0  ← En temporizador independiente0Comienza desde
// Observer 1: 2
// Observer 2: 1
```

> [!NOTE]

> **Características de Cold Observable**.
> - Cada suscripción inicia una ejecución independiente
> - Cada suscriptor recibe su propio flujo de datos
> - Se inicia un temporizador independiente para cada suscripción. Utilice `share()` si necesita compartir datos.
>
> Para más información, véase [Cold Observable and Hot Observable](/es/guide/observables/cold-and-hot-observables).

## Diferencia entre interval() vs timer()

Aunque `interval()` y `timer()` son similares, existen algunas diferencias importantes.

```typescript
import { interval, timer } from 'rxjs';
import { take } from 'rxjs';

// interval() - Inicio inmediato, publicación continua
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - Inicio tras retardo
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// Salida:
// interval: 0  (1(tras un retardo de un segundo)
// interval: 1  (2(tras un retardo de un segundo)
// timer: 0     (2(tras un retardo de un segundo)
// interval: 2  (3(tras un retardo de un segundo)
// timer: 1     (3(tras un retardo de un segundo)
// timer: 2     (4(tras un retardo de un segundo)
```

```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

## Casos prácticos

### 1. API de sondeo

Llame a la API a intervalos regulares para actualizar los datos.


```typescript
import { from, interval } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

function fetchStatus(): Promise<Status> {
  return fetch('https://jsonplaceholder.typicode.com/users/1')
    .then(res => res.json());
}

// 5Cada segundoAPISondeo
const polling$ = interval(5000).pipe(
  switchMap(() => from(fetchStatus())),
  catchError((error: unknown) => {
    console.error('API Error:', error);
    return of({ status: 'error', timestamp: Date.now() });
  })
);

const subscription = polling$.subscribe(data => {
  console.log('Actualización de estado:', data);
});

// Detenido si es necesario
// subscription.unsubscribe();
```

### 2. Temporizador de cuenta atrás

Implementa una cuenta atrás hasta el tiempo límite.

```typescript
import { interval } from 'rxjs';
import { map, takeWhile } from 'rxjs';

const countdown$ = interval(1000).pipe(
  map(count => 10 - count), // 10Cuenta atrás de segundos
  takeWhile(time => time >= 0) // 0Finalización automática con
);

countdown$.subscribe({
  next: time => console.log(`Tiempo restante: ${time}segundos`),
  complete: () => console.log('Tiempo expirado！')
});

// Salida (en1(cada segundo):
// Tiempo restante: 10segundos
// Tiempo restante: 9segundos
// ...
// Tiempo restante: 0segundos
// Tiempo expirado！
```

### 3. función de guardado automático

El contenido del formulario se guarda automáticamente a intervalos regulares.

```typescript
import { fromEvent, from } from 'rxjs';
import { switchMap, debounceTime } from 'rxjs';

// Crear formulario
const form = document.createElement('form');
form.id = 'myForm';
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Rellene el formulario';
form.appendChild(input);
document.body.appendChild(form);

const input$ = fromEvent(form, 'input');

// Después de la entrada se detiene3Segundos después, autoguardado (abreviado para fines de demostración)
input$.pipe(
  debounceTime(3000), // 3Si no hay entradas durante segundos
  switchMap(() => {
    const formData = new FormData(form);
    // Para demostración: ActualAPIen lugar del realPromiseSimulado en
    return from(
      Promise.resolve({ success: true, data: formData.get('text') })
    );
  })
).subscribe(result => {
  console.log('Autoguardado:', result);
});
```

### 4. Visualización del reloj en tiempo real

Actualiza la hora actual en tiempo real.

```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

## Usar en pipeline.

`interval()` se utiliza como punto de partida para pipelines o como disparador para control de tiempo.


## Errores comunes.

### 1. Olvidar darse de baja


### 2. las suscripciones múltiples crean temporizadores independientes


## Consideraciones de rendimiento.

`interval()` es ligero, pero deben tenerse en cuenta consideraciones de rendimiento cuando se ejecuta a intervalos cortos.

> [!TIP]

> **Consejos de optimización**:.
> - No ejecute procesos innecesarios (fíltrelos con `filter()`)
> - Utilice intervalos cortos (menos de 100 ms) con cuidado
> - Asegure la desuscripción.
> - Compartir con `share()` si se necesita más de un Observer.


```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

## Related Creation Function.


## Resumen

- `interval()` emite valores continuos a intervalos especificados
- Emite enteros continuos a partir de 0
- No finaliza automáticamente, debe darse de baja
- Funciona como Cold Observable (temporizador independiente por suscripción)
- Ideal para sondeo, ejecución periódica, cuenta atrás, etc.

## Próximos pasos.

- timer() - iniciar la publicación tras un retardo](/es/guide/creation-functions/basic/timer)
- fromEvent() - convertir evento en Observable](/es/guide/creation-functions/basic/fromEvent)
- Volver al sistema básico de creación](/es/guide/creation-functions/basic/)
