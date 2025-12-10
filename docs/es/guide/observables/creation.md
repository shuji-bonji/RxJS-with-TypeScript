---
description: Este curso proporciona una explicación completa de cómo crear Observables en RxJS, desde funciones generadoras básicas como of y from, hasta la definición de Observables personalizados, comunicación HTTP y flujo de eventos, con ejemplos de código prácticos.
---
# Cómo Crear un Observable

Un Observable define un "flujo de datos", y hay una amplia variedad de formas de crear uno.
RxJS proporciona una variedad de medios para crear Observables personalizados o generar fácilmente Observables desde eventos, arrays, respuestas HTTP, etc.

Esta sección proporciona una descripción completa de cómo crear Observables en RxJS, desde la sintaxis básica hasta las aplicaciones prácticas.

## Clasificación de Métodos de Creación de Observable

La siguiente es una lista de los principales métodos de creación por categoría.

| Categoría | Métodos Principales | Descripción |
|----------|----------|------|
| Creación Personalizada | [`new Observable()`](#new-observable) | Alta flexibilidad pero requiere más código. Limpieza manual requerida |
| Creation Functions | [`of()`](#of), [`from()`](#from), [`fromEvent()`](#fromevent), [`interval()`](#interval-timer), [`timer()`](#interval-timer), [`ajax()`](#ajax), [`fromFetch()`](#fromfetch), [`scheduled()`](#scheduled) | Funciones de generación basadas en datos, eventos y tiempo comúnmente utilizadas |
| Special Creation Functions | [`defer()`](#defer), [`range()`](#range), [`generate()`](#generate), [`iif()`](#iif) | Orientadas a control, generación orientada a bucles, cambio condicional, etc. |
| Special Observables | [`EMPTY`](#empty-never-throwerror), [`NEVER`](#empty-never-throwerror), [`throwError()`](#empty-never-throwerror) | Para finalización, sin acción, y emisión de errores |
| Familia Subject | [`Subject`](#subject-behaviorsubject), [`BehaviorSubject`](#subject-behaviorsubject) | Observable especial que funciona como observador y emisor |
| Conversión de Callback | [`bindCallback()`](#bindcallback), [`bindNodeCallback()`](#bindnodecallback) | Convertir funciones basadas en callback a Observable |
| Control de Recursos | [`using()`](#using) | Realizar control de recursos al mismo tiempo que se suscribe a Observable |
| WebSocket | [`webSocket()`](#websocket) | Manejar comunicación WebSocket como Observable bidireccional |

## Creación Personalizada

### new Observable()
[📘 RxJS Oficial: Observable](https://rxjs.dev/api/index/class/Observable)

El método más básico es usar el constructor `Observable` directamente. Este método es más flexible cuando se desea definir lógica Observable personalizada. Es posible un control de comportamiento de grano fino a través de llamadas explícitas a `next`, `error` y `complete`.

```ts
import { Observable } from 'rxjs';

const observable$ = new Observable<number>(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  setTimeout(() => {
    subscriber.next(4);
    subscriber.complete();
  }, 1000);
});

observable$.subscribe({
  next: value => console.log('Valor:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completo')
});
// Salida:
// Valor: 1
// Valor: 2
// Valor: 3
// Valor: 4
// Completo
```

> [!CAUTION]
> Si usa `new Observable()`, debe escribir la liberación explícita de recursos (proceso de limpieza) usted mismo.
> ```ts
> const obs$ = new Observable(subscriber => {
>   const id = setInterval(() => subscriber.next(Date.now()), 1000);
>   return () => {
>     clearInterval(id); // Liberación explícita de recursos
>   };
> });
> ```
> Por otro lado, las creation functions integradas de RxJS como `fromEvent()` e `interval()` tienen procesos de limpieza apropiados internamente.
> ```ts
> const click$ = fromEvent(document, 'click');
> const timer$ = interval(1000);
> ```
> Usan `addEventListener` o `setInterval` internamente y están diseñadas para que RxJS llame automáticamente a `removeEventListener` o `clearInterval()` cuando se hace `unsubscribe()`.
>
> Tenga en cuenta que incluso si el proceso de limpieza está implementado dentro de RxJS, ese proceso no se ejecutará a menos que se llame a `unsubscribe()`.
> ```ts
>  const subscription = observable$.subscribe({
>  // Omitido...
>  });
>
>  subscription.unsubscribe(); // 👈
> ```
> - No importa qué método use para crear un Observable, asegúrese de adquirir el hábito de `unsubscribe()` cuando ya no lo necesite.
> - Olvidar cancelar la suscripción mantendrá los escuchadores de eventos y temporizadores en ejecución, causando fugas de memoria y efectos secundarios inesperados.

## Creation Functions

Para una creación de Observable más concisa y específica de la aplicación, RxJS proporciona "Creation Functions". Estas se pueden usar para simplificar el código para casos de uso repetidos.

> [!NOTE]
> En la documentación oficial de RxJS, estas se categorizan como "Creation Functions".
> Anteriormente (RxJS 5.x ~ 6) se llamaban "creation operators", pero desde RxJS 7, "Creation Functions" es el término oficial.

### of()
[📘 RxJS Oficial: of()](https://rxjs.dev/api/index/function/of)

La Creation Function de Observable más simple que emite múltiples valores **uno a la vez en secuencia**.

```ts
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Valor:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completo')
});
// Salida: Valor: 1, Valor: 2, Valor: 3, Valor: 4, Valor: 5, Completo
```

> [!IMPORTANT]
> Diferencia entre `of()` y `from()`
> - `of([1, 2, 3])` → emite un solo array.
> - `from([1, 2, 3])` → emite valores individuales `1`, `2`, `3` en secuencia.
>
> Tenga en cuenta que esto a menudo se confunde.

> [!TIP]
> Para uso detallado y ejemplos prácticos, consulte [página de detalle de of()](/es/guide/creation-functions/basic/of).

### from()
[📘 RxJS Oficial: from()](https://rxjs.dev/api/index/function/from)

Genera un Observable a partir de una **estructura de datos existente** como un array, Promise o iterable.

```ts
import { from } from 'rxjs';

// Crear desde array
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Valor del array:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completo')
});

// Crear desde Promise
const promise$ = from(Promise.resolve('Resultado de Promise'));
promise$.subscribe({
  next: value => console.log('Resultado de Promise:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completo')
});

// Crear desde iterable
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Valor iterable:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completo')
});

// Salida:
// Valor del array: 1
// Valor del array: 2
// Valor del array: 3
// Completo
// Valor iterable: 1
// Valor iterable: 2
// Valor iterable: 3
// Completo
// Resultado de Promise: Resultado de Promise
// Completo
```

> [!TIP]
> Para uso detallado y ejemplos prácticos, consulte [página de detalle de from()](/es/guide/creation-functions/basic/from).

### fromEvent()
[📘 RxJS Oficial: fromEvent](https://rxjs.dev/api/index/function/fromEvent)

Función para **manejar fuentes de eventos** como eventos DOM como un Observable.

```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe({
  next: event => console.log('Evento de clic:', event),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completo')
});

// Salida:
// Evento de clic: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!CAUTION]
> Tenga en cuenta los objetivos de eventos admitidos
> - `fromEvent()` admite elementos DOM del navegador (implementación de EventTarget), EventEmitter de Node.js y objetivos de eventos similares a jQuery.
> - Múltiples suscripciones pueden agregar múltiples escuchadores de eventos.

> 👉 Para ejemplos más detallados de utilización de flujos de eventos, consulte [Flujo de Eventos](../observables/events).

> [!TIP]
> Para uso detallado y ejemplos prácticos, consulte [página de detalle de fromEvent()](/es/guide/creation-functions/basic/fromEvent).

### interval(), timer()
[📘 RxJS Oficial: interval](https://rxjs.dev/api/index/function/interval), [📘 RxJS Oficial: timer](https://rxjs.dev/api/index/function/timer)

Esta función se usa cuando se desea emitir valores continuamente a intervalos regulares o cuando se necesita **control de tiempo**.

```ts
import { interval, timer } from 'rxjs';

// Emitir valores cada segundo
const interval$ = interval(1000);
interval$.subscribe({
  next: value => console.log('Intervalo:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completo')
});

// Iniciar después de 3 segundos, luego emitir valores cada segundo
const timer$ = timer(3000, 1000);
timer$.subscribe({
  next: value => console.log('Temporizador:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completo')
});

// Salida:
// Intervalo: 0
// Intervalo: 1
// Intervalo: 2
// Temporizador: 0
// Intervalo: 3
// Temporizador: 1
// Intervalo: 4
// Temporizador: 2
// .
// .
```
`interval()` y `timer()` se usan frecuentemente para procesamiento controlado por tiempo, especialmente adecuados para animación, sondeo y retrasos de eventos asíncronos.

> [!CAUTION]
> Tenga en cuenta que Cold Observable
> - `interval()` y `timer()` son Cold Observable y se ejecutan independientemente para cada suscripción.
> - Puede considerar hacerlos Hot con `share()` u otros métodos si es necesario.
>
> Para detalles, consulte la sección ["Cold Observable y Hot Observable"](./cold-and-hot-observables.md).

> [!TIP]
> Para uso detallado y ejemplos prácticos, consulte [página de detalle de interval()](/es/guide/creation-functions/basic/interval) y [página de detalle de timer()](/es/guide/creation-functions/basic/timer).

### ajax()
[📘 RxJS Oficial: ajax](https://rxjs.dev/api/ajax/ajax)

Función para **manejo asíncrono** de resultados de comunicación HTTP como **Observable**.

```ts
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
api$.subscribe({
  next: response => console.log('Respuesta de API:', response),
  error: error => console.error('Error de API:', error),
  complete: () => console.log('API completa')
});

// Salida:
// Respuesta de API: {userId: 1, id: 1, title: 'delectus aut autem', completed: false}
// API completa
```

> [!NOTE]
> RxJS ajax usa XMLHttpRequest internamente. Por otro lado, RxJS también tiene un operador llamado fromFetch, que usa la API Fetch para hacer solicitudes HTTP.

> [!TIP]
> Para uso detallado y ejemplos prácticos, consulte [página de detalle de ajax()](/es/guide/creation-functions/http-communication/ajax). Para una descripción general de las funciones de comunicación HTTP, consulte [Creation Functions de Comunicación HTTP](/es/guide/creation-functions/http-communication/).

### fromFetch()
[📘 RxJS Oficial: fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` envuelve la API Fetch y le permite tratar solicitudes HTTP como Observables.
Es similar a `ajax()`, pero más moderno y ligero.

```ts
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs';

const api$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1');

api$.pipe(
  switchMap(response => response.json())
).subscribe({
  next: data => console.log('Datos:', data),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completo')
});

// Salida:
// Datos: {completed: false, id: 1, title: "delectus aut autem", userId: 1}
// Completo
```

> [!NOTE]
> Debido a que `fromFetch()` usa la API Fetch, a diferencia de `ajax()`, la inicialización de configuración de solicitudes y la conversión `.json()` de respuestas deben hacerse manualmente.
> También se requiere un manejo de errores apropiado y verificación de estado HTTP.

> [!TIP]
> Para uso detallado y ejemplos prácticos, consulte [página de detalle de fromFetch()](/es/guide/creation-functions/http-communication/fromFetch). Para una descripción general de las funciones de comunicación HTTP, consulte [Creation Functions de Comunicación HTTP](/es/guide/creation-functions/http-communication/).

### scheduled()
[📘 RxJS Oficial: scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` es una función que le permite especificar explícitamente un scheduler para funciones publicadas como `of()` y `from()`.
Use esta función cuando desee controlar el tiempo de ejecución sincrónica o asíncrona en detalle.

```ts
import { scheduled, asyncScheduler } from 'rxjs';

const observable$ = scheduled([1, 2, 3], asyncScheduler);
observable$.subscribe({
  next: val => console.log('Valor:', val),
  complete: () => console.log('Completo')
});

// La ejecución es asíncrona
// Salida:
// Valor: 1
// Valor: 2
// Valor: 3
// Completo
```

> [!NOTE]
> `scheduled()` permite que las funciones sincrónicas existentes (ej. `of()`, `from()`) funcionen de manera asíncrona.
> Esto es útil para pruebas y optimización de rendimiento de UI donde se requiere control de procesamiento asíncrono.

> [!TIP]
> Para uso detallado y ejemplos prácticos, consulte [página de detalle de scheduled()](/es/guide/creation-functions/control/scheduled). Para una descripción general de funciones de control, consulte [Creation Functions de Control](/es/guide/creation-functions/control/).

### defer()
[📘 RxJS Oficial: defer](https://rxjs.dev/api/index/function/defer)

Se usa cuando se desea **retrasar la generación de un Observable hasta el momento de la suscripción**.

```ts
import { defer, of } from 'rxjs';

const random$ = defer(() => of(Math.random()));

random$.subscribe(value => console.log('1ro:', value));
random$.subscribe(value => console.log('2do:', value));

// Salida:
// 1ro: 0.123456789
// 2do: 0.987654321
```

> [!NOTE]
> `defer()` es útil cuando se desea crear un nuevo Observable en cada suscripción. Se puede lograr evaluación perezosa.

> [!TIP]
> Para uso detallado y ejemplos prácticos, consulte [página de detalle de defer()](/es/guide/creation-functions/conditional/defer).

### range()
[📘 RxJS Oficial: range](https://rxjs.dev/api/index/function/range)

Genera un valor entero continuo en el rango especificado como un Observable.

```ts
import { range } from 'rxjs';

const numbers$ = range(1, 5);
numbers$.subscribe({
  next: value => console.log('Número:', value),
  complete: () => console.log('Completo')
});

// Salida:
// Número: 1
// Número: 2
// Número: 3
// Número: 4
// Número: 5
// Completo
```

> [!TIP]
> Para uso detallado y ejemplos prácticos, consulte [página de detalle de range()](/es/guide/creation-functions/loop/range).

### generate()
[📘 RxJS Oficial: generate](https://rxjs.dev/api/index/function/generate)

Genera Observable como una estructura de bucle. Permite un control fino sobre valores iniciales, condiciones, aumentos/disminuciones y salida de valores.

```ts
import { generate } from 'rxjs';

const fibonacci$ = generate({
  initialState: [0, 1],
  condition: ([, b]) => b < 100,
  iterate: ([a, b]) => [b, a + b],
  resultSelector: ([a]) => a
});

fibonacci$.subscribe({
  next: value => console.log('Fibonacci:', value),
  complete: () => console.log('Completo')
});

// Salida:
// Fibonacci: 0
// Fibonacci: 1
// Fibonacci: 1
// Fibonacci: 2
// Fibonacci: 3
// Fibonacci: 5
// Fibonacci: 8
// Fibonacci: 13
// Fibonacci: 21
// Fibonacci: 34
// Fibonacci: 55
// Fibonacci: 89
// Completo
```

> [!TIP]
> Para uso detallado y ejemplos prácticos, consulte [página de detalle de generate()](/es/guide/creation-functions/loop/generate).

### iif()
[📘 RxJS Oficial: iif](https://rxjs.dev/api/index/function/iif)

Use esta función cuando desee **cambiar Observable por ramificación condicional**.

```ts
import { iif, of, EMPTY } from 'rxjs';

const condition = true;
const iif$ = iif(() => condition, of('La condición es verdadera'), EMPTY);

iif$.subscribe({
  next: val => console.log('iif:', val),
  complete: () => console.log('Completo')
});

// Salida:
// iif: La condición es verdadera
// Completo
```

> [!NOTE]
> `iif()` puede cambiar dinámicamente el Observable a devolver dependiendo de las condiciones. Esto es útil para el control de flujo.

## Special Observables

### EMPTY, NEVER, throwError()
[📘 RxJS Oficial: EMPTY](https://rxjs.dev/api/index/const/EMPTY), [📘 RxJS Oficial: NEVER](https://rxjs.dev/api/index/const/NEVER), [📘 RxJS Oficial: throwError](https://rxjs.dev/api/index/function/throwError)

RxJS también proporciona Observables especiales que son útiles para el control de ejecución, manejo de excepciones y aprendizaje.

```ts
import { EMPTY, throwError, NEVER } from 'rxjs';

// Observable que se completa inmediatamente
const empty$ = EMPTY;
empty$.subscribe({
  next: () => console.log('Esto no se muestra'),
  complete: () => console.log('Se completa inmediatamente')
});

// Observable que emite un error
const error$ = throwError(() => new Error('Error ocurrido'));
error$.subscribe({
  next: () => console.log('Esto no se muestra'),
  error: err => console.error('Error:', err.message),
  complete: () => console.log('Completo')
});

// Observable que no emite nada y no se completa
const never$ = NEVER;
never$.subscribe({
  next: () => console.log('Esto no se muestra'),
  complete: () => console.log('Esto tampoco se muestra')
});

// Salida:
// Se completa inmediatamente
// Error: Error ocurrido
```

> [!IMPORTANT]
> Principalmente para propósitos de control, verificación y aprendizaje
> - `EMPTY`, `NEVER` y `throwError()` se usan para **control de flujo, validación de manejo de excepciones**, o propósitos de aprendizaje, no para flujos de datos normales.

## Familia Subject

### Subject, BehaviorSubject, etc. {#subject-behaviorsubject}
[📘 RxJS Oficial: Subject](https://rxjs.dev/api/index/class/Subject), [📘 RxJS Oficial: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Observable que puede emitir su propio valor, adecuado para **multicast y compartir estado**.

```ts
import { Subject } from 'rxjs';

const subject$ = new Subject<number>();

// Usar como Observer
subject$.subscribe(value => console.log('Observador 1:', value));
subject$.subscribe(value => console.log('Observador 2:', value));

// Usar como Observable
subject$.next(1);
subject$.next(2);
subject$.next(3);

// Salida:
// Observador 1: 1
// Observador 2: 1
// Observador 1: 2
// Observador 2: 2
// Observador 1: 3
// Observador 2: 3
```

> [!IMPORTANT]
> Subject tiene las propiedades tanto de Observable como de Observer. Múltiples suscriptores pueden compartir el mismo flujo de datos (multicast).

> [!TIP]
> Para detalles sobre varios tipos de Subject (BehaviorSubject, ReplaySubject, AsyncSubject), consulte [Subject y Multicast](/es/guide/subjects/what-is-subject).

## Conversión de Callback

### bindCallback()
[📘 RxJS Oficial: bindCallback](https://rxjs.dev/api/index/function/bindCallback)

Una función que permite que las funciones asíncronas basadas en callback se traten como Observable.

```ts
import { bindCallback } from 'rxjs';

// Función basada en callback (estilo legado)
function asyncFunction(value: number, callback: (result: number) => void) {
  setTimeout(() => callback(value * 2), 1000);
}

// Convertir a Observable
const asyncFunction$ = bindCallback(asyncFunction);
const observable$ = asyncFunction$(5);

observable$.subscribe({
  next: result => console.log('Resultado:', result),
  complete: () => console.log('Completo')
});

// Salida:
// Resultado: 10
// Completo
```

> [!TIP]
> `bindCallback()` es útil para convertir APIs basadas en callback heredadas a Observable.

### bindNodeCallback()
[📘 RxJS Oficial: bindNodeCallback](https://rxjs.dev/api/index/function/bindNodeCallback)

Una función especializada para convertir funciones basadas en callback en estilo Node.js (error-first callback) a Observable.

```ts
import { bindNodeCallback } from 'rxjs';

// Función de callback estilo Node.js (error-first callback)
function readFile(path: string, callback: (err: Error | null, data: string) => void) {
  if (path === 'valid.txt') {
    callback(null, 'contenido del archivo');
  } else {
    callback(new Error('Archivo no encontrado'), '');
  }
}

// Convertir a Observable
const readFile$ = bindNodeCallback(readFile);

readFile$('valid.txt').subscribe({
  next: data => console.log('Datos:', data),
  error: err => console.error('Error:', err.message),
  complete: () => console.log('Completo')
});

// Salida:
// Datos: contenido del archivo
// Completo
```

### Diferencia entre bindCallback() y bindNodeCallback()

#### Ejemplo: Objetivo de bindCallback()

```ts
// Callback general (solo éxito)
function getData(cb: (data: string) => void) {
  cb('éxito');
}
```
→ Use bindCallback() para callbacks simples de "devolver solo un valor".

#### Ejemplo: Objetivo de bindNodeCallback() (estilo Node.js)

```ts
// Error-first callback
function readFile(path: string, cb: (err: Error | null, data: string) => void) {
  if (path === 'valid.txt') cb(null, 'contenido del archivo');
  else cb(new Error('no encontrado'), '');
}
```
→ Si usa bindNodeCallback(), los errores se notificarán como errores de Observable.

> [!NOTE]
> Cómo usar
> - bindNodeCallback() si el primer argumento del callback es "error o no"
> - bindCallback() para un callback simple de "devolver solo un valor"

## Control de Recursos

### using()
[📘 RxJS Oficial: using](https://rxjs.dev/api/index/function/using)

`using()` funciona para asociar la creación y liberación de recursos con el ciclo de vida del Observable.
Es útil en combinación con **procesos que requieren limpieza manual**, como WebSockets, escuchadores de eventos y recursos externos.

```ts
import { using, interval, Subscription } from 'rxjs';

const resource$ = using(
  () => new Subscription(() => console.log('Recurso liberado')),
  () => interval(1000)
);

const sub = resource$.subscribe(value => console.log('Valor:', value));

// Cancelar suscripción después de unos segundos
setTimeout(() => sub.unsubscribe(), 3500);

// Salida:
// Valor: 0
// Valor: 1
// Valor: 2
// Recurso liberado
```

> [!IMPORTANT]
> `using()` es útil para hacer coincidir el alcance de un recurso con la suscripción del Observable.
> Se llama automáticamente un proceso de limpieza explícito cuando se hace `unsubscribe()`.

> [!TIP]
> Para uso detallado y ejemplos prácticos, consulte [página de detalle de using()](/es/guide/creation-functions/control/using). Para una descripción general de funciones de control, consulte [Creation Functions de Control](/es/guide/creation-functions/control/).

## WebSocket()
[📘 RxJS Oficial: webSocket](https://rxjs.dev/api/webSocket/webSocket)

El módulo `rxjs/webSocket` de RxJS proporciona una función `webSocket()` que permite que WebSocket se trate como un Observable/Observer.

```ts
import { webSocket } from 'rxjs/webSocket';

const socket$ = webSocket('wss://echo.websocket.org');

socket$.subscribe({
  next: msg => console.log('Recibido:', msg),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completo')
});

// Enviar mensaje (como Observer)
socket$.next('¡Hola WebSocket!');
```

> [!IMPORTANT]
> `webSocket()` es un híbrido Observable/Observer que permite comunicación bidireccional.
> Es útil para comunicación en tiempo real porque las conexiones WebSocket, envío y recepción se pueden manejar fácilmente como Observable.

## Resumen

Hay una amplia variedad de formas de crear Observables en RxJS, y es importante elegir el método apropiado para su aplicación.

- Si necesita procesamiento personalizado, use `new Observable()`
- `of()`, `from()`, `fromEvent()`, etc. para manejar datos y eventos existentes
- `ajax()` o `fromFetch()` para comunicación HTTP
- Familia `Subject` para compartir datos entre múltiples suscriptores

Al usarlos apropiadamente, puede aprovechar al máximo la flexibilidad de RxJS.
