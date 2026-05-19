---
description: "timer() - Creation Function que emite un valor después de un tiempo especificado, usando timer(delay) para una ejecución única retardada y timer(delay, period) para una ejecución periódica retardada; cómo usarlo con interval(); inferencia de tipos en TypeScript, Explicación de su uso como alternativa a setTimeout."
---

# timer() - comienza a publicar después de un retardo

`timer()` es una Creation Function que comienza a publicar valores después de un retardo especificado, tanto una sola vez como periódicamente.

## Visión general.

timer()` es una flexible Creation Function que permite controlar el tiempo de la primera emisión. Su comportamiento depende del número de argumentos y puede ser una emisión única o una emisión periódica como `interval()`.

**Firma**:.

```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

**Documentación oficial**: [📘 Fórmula RxJS: timer()](https://rxjs.dev/api/index/function/timer)

## Uso básico

El comportamiento de `timer()` depende del número de argumentos.

### Problema puntual.

Si sólo se especifica el primer argumento, 0 se emite después de que el tiempo especificado para completar.

```typescript
import { timer } from 'rxjs';

// 3segundos después de0Emitir y completar el
const timer$ = timer(3000);

timer$.subscribe({
  next: value => console.log('Valor:', value),
  complete: () => console.log('Completado')
});

// 3Salida después de segundos:
// Valor: 0
// Completado
```

### Emisión periódica.

Si se especifica un intervalo en el segundo argumento, continuará emitiendo periódicamente después del retardo inicial.

```typescript
import { timer } from 'rxjs';

// 3Emite después de segundos y1Emite valor cada segundo
const timer$ = timer(3000, 1000);

timer$.subscribe(value => console.log('Valor:', value));

// Salida:
// Valor: 0  (3(después de segundos)
// Valor: 1  (4(después de segundos)
// Valor: 2  (5(después de segundos)
// ...(continúa indefinidamente)
```

## Características importantes.

### 1. Especificación flexible de los retrasos.

El retraso puede especificarse como un número en milisegundos o como un objeto `Date`.

```typescript
import { timer } from 'rxjs';

// Especificado en milisegundos
timer(5000).subscribe(() => console.log('5Después de segundos'));

// Date Especificado por objeto (se ejecuta a una hora determinada)
const targetTime = new Date(Date.now() + 10000); // 10Después de segundos
timer(targetTime).subscribe(() => console.log('Ejecutado a la hora especificada'));
```

### 2. El comportamiento cambia con o sin el segundo argumento.

El hecho de especificar o no el segundo argumento determina si se alcanza o no la finalización.

```typescript
import { timer } from 'rxjs';

// Número de segundos2Sin argumento - 1Emitido una vez y completado
timer(1000).subscribe({
  next: value => console.log('12ª:', value),
  complete: () => console.log('Completado')
});

// Número de segundos2Con argumento - Seguir emitiendo indefinidamente
timer(1000, 1000).subscribe({
  next: value => console.log('Repetir:', value),
  complete: () => console.log('Completado (no se muestra)')
});
```

> [!IMPORTANT]

> **No se completa con el segundo argumento**
>
> >Si se especifica un segundo argumento, como `timer(1000, 1000)`, continuará emitiendo indefinidamente, como con `interval()`. Siempre debe darse de baja.

### 5. Cold Observable.

timer()` es un Cold Observable, que crea un timer independiente para cada suscripción.

```typescript
import { timer } from 'rxjs';

const timer$ = timer(1000);

console.log('Iniciar');

// Suscribir1
timer$.subscribe(() => console.log('Observer 1'));

// 500msSuscribir después de2Añadir
setTimeout(() => {
  timer$.subscribe(() => console.log('Observer 2'));
}, 500);

// Salida:
// Iniciar
// Observer 1  (1(después de segundos)
// Observer 2  (1.5Después de segundos - (Temporizador independiente)
```

> [!NOTE]

> **Características de Cold Observable**.
> - Cada suscripción inicia una ejecución independiente
> - Cada suscriptor recibe su propio flujo de datos
> - Se inicia un temporizador independiente para cada suscripción; al igual que con interval()`, utilice `share()` si es necesario compartir.
>
> Para más información, véase [Cold Observable and Hot Observable](/es/guide/observables/cold-and-hot-observables).

## Diferencia entre timer() vs interval()

La principal diferencia entre las dos es la temporización de la primera emisión.

```typescript
import { timer, interval } from 'rxjs';
import { take } from 'rxjs';

console.log('Iniciar');

// interval() - Comienza inmediatamente (después de1(primer valor en segundos)
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - Sin retardo (primer valor inmediatamente)
timer(0, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// timer() - 2Se inicia tras un retardo de segundos
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer(Con retardo (primer valor inmediatamente)):', value);
});
```

```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

## Casos prácticos

### 1. ejecución perezosa

Ejecutar el proceso sólo una vez después de un cierto período de tiempo.


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
  console.log('2Adquisición de datos después de segundos:', data);
});
```

### 2. Sondeo con retardo

La primera vez el sondeo no se realiza inmediatamente, sino que se inicia después de un cierto período de tiempo.

```typescript
import { from, timer } from 'rxjs';
import { switchMap, retry } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

// 5Comienza el sondeo después de segundos, luego10cada segundo
const polling$ = timer(5000, 10000).pipe(
  switchMap(() => from(
    fetch('https://jsonplaceholder.typicode.com/users/1')
      .then(res => res.json() as Promise<Status>)
  )),
  retry(3) // En caso de error3Reintentos hasta tres veces
);

const subscription = polling$.subscribe(data => {
  console.log('Actualización del estado:', data);
});

// Se detiene si es necesario
// subscription.unsubscribe();
```

### 3. Proceso de Time-out

Tiempo de espera si el proceso no se completa en un periodo de tiempo determinado.

```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

### 4. auto-ocultamiento de notificaciones

Oculta automáticamente las notificaciones después de un cierto período de tiempo después de que se muestran.


## Usar en pipeline.

El `timer()` se utiliza como punto de partida para el procesamiento retardado y la ejecución periódica.


```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

## Errores comunes.

### 1. olvidar darse de baja con un segundo argumento


### 2. no entender la diferencia entre interval() y


## Consideraciones de rendimiento.

Aunque `timer()` es ligero, su uso puede afectar al rendimiento.

## consideraciones de rendimiento

> **Consejos de optimización**:.
> - Nunca especifiques un segundo argumento para una ejecución única.
> - Anular siempre la suscripción cuando ya no se necesite.
> - Compartir con `share()` si se necesitan varios Observer
> - Utilice intervalos cortos (menos de 100 ms) con precaución


```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

## Related Creation Function.


## Resumen

- timer()` es una Creation Function que comienza a emitir después de un retardo
- Sin segundo argumento: emisión única (completa)
- Con segundo argumento: emisión periódica (nunca se completa)
- Tiempo de retardo especificado en milisegundos u objeto `Date
- Adecuado para ejecución retardada, sondeo con retardo, procesamiento de tiempo de espera

## Siguiente paso.

- interval() - publicar continuamente a intervalos especificados](/es/guide/creation-functions/basic/interval)
- defer() - retrasa la generación en la suscripción](/es/guide/creation-functions/conditional/defer)
- volver al resumen de funciones básicas de creación](/es/guide/creation-functions/basic/)
