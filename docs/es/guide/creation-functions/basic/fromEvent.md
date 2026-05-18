---
description: "fromEvent() - Creation Function para convertir eventos DOM y EventEmitters en Observable, la base de la programación dirigida por eventos, que se puede utilizar para procesar varios eventos como clics, pulsaciones de teclas, movimientos del ratón y desplazamiento. Explica las definiciones de tipos de TypeScript y cómo especificar objetivos de eventos."
---

# fromEvent() - convierte el evento en Observable

fromEvent()` es una Creation Function que convierte fuentes de eventos como eventos DOM y Node.js EventEmitter en flujos Observable.

## Visión general.

fromEvent()` permite que el procesamiento asíncrono basado en eventos sea manejado en el pipeline de RxJS. Registra automáticamente los escuchadores de eventos cuando se suscriben y elimina automáticamente los escuchadores cuando se desuscriben, reduciendo así significativamente el riesgo de fugas de memoria.

**Firma**:.

```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

**Documentación oficial**: [📘 Fórmula RxJS: fromEvent()](https://rxjs.dev/api/index/function/fromEvent)

## Uso básico.

Este es el ejemplo más simple de tratar eventos DOM como Observable.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Botón pulsado.:', event);
});

// Se emite un evento cada vez que se hace clic.
```

## Características importantes.

### 1. alta y baja automática de oyentes

La función `fromEvent()` registra escuchadores de eventos cuando se suscriben y los elimina automáticamente cuando se dan de baja.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent<MouseEvent>(document, 'click');

const subscription = clicks$.subscribe(event => {
  console.log('Posición del clic:', event.clientX, event.clientY);
});

// 5Desuscrito después de unos segundos (el listener del evento también se borra automáticamente)
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Desuscrito');
}, 5000);
```

```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

> **Prevención de fugas de memoria**
>
> Cuando se llama a `unsubscribe()`, se ejecuta automáticamente un `removeEventListener()` interno. Esto elimina la necesidad de eliminar manualmente los oyentes y reduce significativamente el riesgo de fugas de memoria.

### 2. Cold Observable (cada suscripción registra un oyente independiente).

El Observable creado por `fromEvent()` es un **Cold Observable. Cada suscripción registra un receptor de eventos independiente.


```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Suscrito a1 - ListenerASuscrito a
clicks$.subscribe(() => console.log('Observer 1: Haga clic en'));

// 1Suscribirse en segundos2Añadir - ListenerBRegistrar independientemente de
setTimeout(() => {
  clicks$.subscribe(() => console.log('Observer 2: Haga clic en'));
}, 1000);

// 1Ambos receptores se activan con un solo clic
// Esto es una prueba de que cada suscripción tiene oyentes independientes
```

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Botón pulsado.:', event);
});

// Se emite un evento cada vez que se hace clic.
```

> **Prueba de frío Observable
>
> Se registra un nuevo oyente de eventos cada vez que se realiza una suscripción y se elimina cuando se cancela la suscripción. Esta es una característica de Cold Observable. Sin embargo, también tiene la propiedad Hot de que "no se pueden recibir eventos antes de la suscripción" porque la fuente de eventos (por ejemplo, elementos DOM) es externa y compartida.

### 3. Soporte de tipos TypeScript

Los tipos de eventos pueden especificarse explícitamente.


```typescript
import { fromEvent } from 'rxjs';

const input = document.createElement('input');
input.type = 'text';
document.body.appendChild(input);
const input$ = fromEvent<InputEvent>(input, 'input');

input$.subscribe(event => {
  // eventEl tipo deInputEvent
  const target = event.target as HTMLInputElement;
  console.log('Valor de entrada:', target.value);
});
```

### 4. Cold Observable.

El `fromEvent()` es un **Cold Observable. Cada suscripción inicia una ejecución independiente.

```typescript
import { fromEvent } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Suscrito a";
document.body.appendChild(button);

const clicks$ = fromEvent(document, 'click');

// 1La segunda suscripción - Se añaden escuchadores de eventos
clicks$.subscribe(() => console.log('SuscripciónA'));

// 2La segunda suscripción - Se añade otro receptor de eventos
clicks$.subscribe(() => console.log('SuscripciónB'));

// 1Un clic activa ambos escuchadores
// Salida:
// SuscripciónA
// SuscripciónB
```

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Botón pulsado.:', event);
});

// Se emite un evento cada vez que se hace clic.
```

> **Características de Cold Observable**.
> - Cada suscripción inicia una ejecución independiente
> - Cada suscriptor recibe su propio flujo de datos

>
> Para más información, consulte [Cold Observable y Hot Observable](/es/guide/observables/cold-and-hot-observables).

## Casos prácticos de uso.

### 1. manejo de eventos click

Controla los clics de los botones y evita los clics consecutivos.


```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const button = document.createElement('button');
button.innerText = "submit";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  debounceTime(300), // 300msIgnora clics consecutivos dentro de
  map(() => 'durante la transmisión...')
).subscribe(message => {
  console.log(message);
  // APILlamadas y otros procesos
});
```

### 2. validación en tiempo real de las entradas de los formularios

Flujo de eventos de entrada y realizar la validación en tiempo real.

```typescript
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'email: ';
const emailInput = document.createElement('input');
label.appendChild(emailInput);
document.body.appendChild(label);
const email$ = fromEvent<InputEvent>(emailInput, 'input');

email$.pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(500), // después de que se haya detenido la entrada.500msProcesado más tarde
  distinctUntilChanged() // Sólo cuando cambia el valor
).subscribe(email => {
  console.log('Sujeto a validación:', email);
  // Proceso de validación de direcciones de correo electrónico
  validateEmail(email);
});

function validateEmail(email: string): void {
  const isValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  console.log(isValid ? 'Dirección de correo electrónico válida' : 'Dirección de correo electrónico no válida');
}
```

### 3. Implementación de arrastrar y soltar

Combina eventos de ratón para implementar arrastrar y soltar.

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, takeUntil, map } from 'rxjs';

// Crear elementos arrastrables
const element = document.createElement('div');
element.style.width = '100px';
element.style.height = '100px';
element.style.backgroundColor = '#333';
element.style.position = 'absolute'; // Posicionamiento absoluto
element.style.left = '50px'; // Posición inicial
element.style.top = '50px';
element.style.cursor = 'move'; // Cursor arrastrable
document.body.appendChild(element);

const mousedown$ = fromEvent<MouseEvent>(element, 'mousedown');
const mousemove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseup$ = fromEvent<MouseEvent>(document, 'mouseup');

mousedown$.pipe(
  switchMap(startEvent => {
    // Registra la posición del clic en el elemento
    const startX = startEvent.clientX - element.offsetLeft;
    const startY = startEvent.clientY - element.offsetTop;

    return mousemove$.pipe(
      map(moveEvent => ({
        left: moveEvent.clientX - startX,
        top: moveEvent.clientY - startY
      })),
      takeUntil(mouseup$) // Finaliza con ratón arriba
    );
  })
).subscribe(({ left, top }) => {
  // Actualizar posición del elemento
  element.style.left = `${left}px`;
  element.style.top = `${top}px`;
});
```

### 4. monitorizar eventos de desplazamiento

Utilizado para rastrear el desplazamiento infinito y la posición de desplazamiento.

```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Usar en pipeline.

fromEvent()` es ideal para el procesamiento de tuberías a partir de un flujo de eventos.


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Errores comunes.

### 1. Olvidar darse de baja

#### ❌ Error - olvidarse de darse de baja provoca fugas de memoria.


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

#### ✅ Correcto - darse de baja siempre


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

> [!WARNING]

> **Atención fugas de memoria**
>
> En SPA y frameworks basados en componentes, siempre desuscríbete cuando destruyas un componente. Si olvidas desuscribirte, los escuchadores de eventos permanecerán y causarán fugas de memoria.

### 2. Registro duplicado de múltiples escuchadores de eventos

#### ❌ Error - suscribirse al mismo evento varias veces registrará múltiples escuchadores.

CÓDIGO 14

#### ✅ Correcto - multidifusión con share() si es necesario.


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Consideraciones de rendimiento.

Las consideraciones de rendimiento deben tenerse en cuenta cuando se trata de eventos de disparo de alta frecuencia (scroll, mousemove, cambiar el tamaño, etc.).

## consideraciones de rendimiento

> **Optimización de eventos de alta frecuencia**:.
> - `throttleTime()` - sólo se maneja una vez cada cierto tiempo.
> - `debounceTime()` - Procesar después de que la entrada se haya detenido.
> - `distinctUntilChanged()` - Procesar sólo cuando cambia el valor.

#### ❌ Problemas de rendimiento - procesar cada cambio de tamaño.


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

#### ✅ Optimización - sólo se procesa una vez cada 200ms


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Funciones de Creation Function relevantes.


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Resumen

- fromEvent()` convierte eventos DOM y EventEmitter a Observable
- Listener registrado cuando se suscribe, borrado automáticamente cuando se desuscribe (previene fugas de memoria)
- Funciona como un Observable en caliente
- Realiza siempre la desuscripción para evitar fugas de memoria
- Optimiza los eventos de alta frecuencia con `throttleTime()` y `debounceTime()`.

## Próximos pasos.

- interval() - emitir valores a intervalos regulares](/es/guide/creation-functions/basic/interval)
- timer() - empezar a publicar después de un retraso](/es/guide/creation-functions/basic/timer)
- volver al resumen de las funciones básicas de creación](/es/guide/creation-functions/basic/)
