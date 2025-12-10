---
description: "fromEvent() - Función de Creación que convierte eventos DOM y EventEmitter a flujos Observable con gestión automática de listeners y prevención de fugas de memoria"
---

# fromEvent() - Convertir Eventos a Observable

`fromEvent()` es una Función de Creación que convierte fuentes de eventos como eventos DOM y Node.js EventEmitter en flujos Observable.

## Resumen

`fromEvent()` permite que los pipelines de RxJS manejen procesamiento asíncrono basado en eventos. Registra automáticamente los event listeners al suscribirse y los elimina automáticamente al desuscribirse, reduciendo enormemente el riesgo de fugas de memoria.

**Firma**:
```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

**Documentación Oficial**: [📘 RxJS Oficial: fromEvent()](https://rxjs.dev/api/index/function/fromEvent)

## Uso Básico

Este es el ejemplo más simple de tratar eventos DOM como Observable.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Botón clickeado:', event);
});

// El evento se emite cada vez que haces clic
```

## Características Importantes

### 1. Registro y Eliminación Automática de Listeners

`fromEvent()` registra un event listener al suscribirse y elimina automáticamente el listener al desuscribirse.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent<MouseEvent>(document, 'click');

const subscription = clicks$.subscribe(event => {
  console.log('Posición del clic:', event.clientX, event.clientY);
});

// Desuscribirse después de 5 segundos (el event listener se elimina automáticamente)
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Desuscrito');
}, 5000);
```

> [!IMPORTANT]
> **Prevención de Fugas de Memoria**
>
> Cuando se llama `unsubscribe()`, `removeEventListener()` se ejecuta automáticamente internamente. Esto elimina la necesidad de eliminar listeners manualmente y reduce enormemente el riesgo de fugas de memoria.

### 2. Cold Observable (Cada Suscripción Registra Listener Independiente)

El Observable creado por `fromEvent()` es un **Cold Observable**. Cada suscripción registra un event listener independiente.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Suscripción 1 - Registrar listener A
clicks$.subscribe(() => console.log('Observador 1: Clic'));

// Añadir suscripción 2 después de 1 segundo - Registrar listener B independientemente
setTimeout(() => {
  clicks$.subscribe(() => console.log('Observador 2: Clic'));
}, 1000);

// Ambos listeners se disparan con un solo clic
// Esto prueba que cada suscripción tiene un listener independiente
```

> [!NOTE]
> **Prueba de Cold Observable**
>
> Se registra un nuevo event listener cada vez que te suscribes y se elimina cuando te desuscribes. Esta es una característica de Cold Observable. Sin embargo, dado que la fuente del evento (ej., elemento DOM) es externa y compartida, también tiene la propiedad Hot de "no recibir eventos antes de la suscripción".

### 3. Soporte de Tipos TypeScript

Los tipos de eventos pueden especificarse explícitamente.

```typescript
import { fromEvent } from 'rxjs';

const input = document.createElement('input');
input.type = 'text';
document.body.appendChild(input);
const input$ = fromEvent<InputEvent>(input, 'input');

input$.subscribe(event => {
  // el tipo de event es InputEvent
  const target = event.target as HTMLInputElement;
  console.log('Valor de entrada:', target.value);
});
```

### 4. Cold Observable

`fromEvent()` es un **Cold Observable**. Cada suscripción inicia una ejecución independiente.

```typescript
import { fromEvent } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Suscribir";
document.body.appendChild(button);

const clicks$ = fromEvent(document, 'click');

// Primera suscripción - se añade event listener
clicks$.subscribe(() => console.log('Suscriptor A'));

// Segunda suscripción - se añade otro event listener
clicks$.subscribe(() => console.log('Suscriptor B'));

// Ambos listeners se disparan cuando se hace un clic
// Salida:
// Suscriptor A
// Suscriptor B
```

> [!NOTE]
> **Características de Cold Observable**:
> - Se inicia una ejecución independiente para cada suscripción
> - Cada suscriptor recibe su propio flujo de datos
> - Se registra un event listener independiente para cada suscripción; desuscribirse elimina automáticamente el listener
>
> Ver [Cold Observable y Hot Observable](/es/guide/observables/cold-and-hot-observables) para más información.

## Casos de Uso Prácticos

### 1. Procesamiento de Eventos de Clic

Controlar clics de botón y prevenir clics consecutivos.

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const button = document.createElement('button');
button.innerText = "enviar";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  debounceTime(300), // Ignorar clics consecutivos dentro de 300ms
  map(() => 'Enviando...')
).subscribe(message => {
  console.log(message);
  // Procesamiento de llamada API, etc.
});
```

### 2. Validación de Entrada de Formulario en Tiempo Real

Transmitir eventos de entrada y realizar validación en tiempo real.

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
  debounceTime(500), // Procesar 500ms después de que la entrada se detenga
  distinctUntilChanged() // Solo cuando el valor cambia
).subscribe(email => {
  console.log('Objetivo de validación:', email);
  // Procesamiento de validación de email
  validateEmail(email);
});

function validateEmail(email: string): void {
  const isValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  console.log(isValid ? 'Dirección de email válida' : 'Dirección de email inválida');
}
```

### 3. Implementación de Arrastrar y Soltar

Combinar eventos de ratón para implementar arrastrar y soltar.

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, takeUntil, map } from 'rxjs';

// Crear elemento arrastrable
const element = document.createElement('div');
element.style.width = '100px';
element.style.height = '100px';
element.style.backgroundColor = '#333';
element.style.position = 'absolute'; // Establecer posicionamiento absoluto
element.style.left = '50px'; // Posición inicial
element.style.top = '50px';
element.style.cursor = 'move'; // Cursor arrastrable
document.body.appendChild(element);

const mousedown$ = fromEvent<MouseEvent>(element, 'mousedown');
const mousemove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseup$ = fromEvent<MouseEvent>(document, 'mouseup');

mousedown$.pipe(
  switchMap(startEvent => {
    // Registrar posición del clic dentro del elemento
    const startX = startEvent.clientX - element.offsetLeft;
    const startY = startEvent.clientY - element.offsetTop;

    return mousemove$.pipe(
      map(moveEvent => ({
        left: moveEvent.clientX - startX,
        top: moveEvent.clientY - startY
      })),
      takeUntil(mouseup$) // Terminar al soltar el ratón
    );
  })
).subscribe(({ left, top }) => {
  // Actualizar posición del elemento
  element.style.left = `${left}px`;
  element.style.top = `${top}px`;
});
```

### 4. Monitoreo de Eventos de Scroll

Se usa para rastrear scroll infinito y posición de scroll.

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

const scroll$ = fromEvent(window, 'scroll');

scroll$.pipe(
  throttleTime(200), // Procesar solo una vez cada 200ms
  map(() => window.scrollY)
).subscribe(scrollPosition => {
  console.log('Posición de scroll:', scrollPosition);

  // Cargar contenido adicional al llegar al final de la página
  if (scrollPosition + window.innerHeight >= document.body.scrollHeight - 100) {
    console.log('Cargar contenido adicional');
    // loadMoreContent();
  }
});
```

## Uso en Pipeline

`fromEvent()` es ideal para procesamiento de pipeline que comienza desde flujos de eventos.

```typescript
import { fromEvent } from 'rxjs';
import { map, filter, scan } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Contador";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  filter((event: Event) => {
    // Contar solo clics mientras se mantiene presionada la tecla Shift
    return (event as MouseEvent).shiftKey;
  }),
  scan((count, _) => count + 1, 0),
  map(count => `Conteo de clics: ${count}`)
).subscribe(message => console.log(message));
```

## Errores Comunes

### 1. Olvidar Desuscribirse

#### ❌ Incorrecto - Olvidar desuscribirse causa fugas de memoria

```typescript
import { fromEvent } from 'rxjs';

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  clicks$.subscribe(console.log); // ¡No desuscrito!
}

setupEventListener();
```

#### ✅ Correcto - Siempre desuscribirse

```typescript
import { fromEvent } from 'rxjs';
import { Subscription } from 'rxjs';

let subscription: Subscription;

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  subscription = clicks$.subscribe(console.log);
}

function cleanup() {
  if (subscription) {
    subscription.unsubscribe();
  }
}

setupEventListener();
// Llamar cleanup() cuando el componente se destruye, etc.
```

> [!WARNING]
> **Cuidado con las Fugas de Memoria**
>
> En SPA y frameworks basados en componentes, asegúrate de desuscribirte cuando destruyas un componente. Si olvidas desuscribirte, los event listeners permanecerán y causarán fugas de memoria.

### 2. Registro Duplicado de Múltiples Event Listeners

#### ❌ Incorrecto - Suscribirse al mismo evento múltiples veces registra múltiples listeners

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Observador 1'));
clicks$.subscribe(() => console.log('Observador 2'));
// Ambos logs se muestran al hacer clic (se registran dos listeners)
```

#### ✅ Correcto - Multicast con share() según sea necesario

```typescript
import { fromEvent } from 'rxjs';
import { share } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(share());

clicks$.subscribe(() => console.log('Observador 1'));
clicks$.subscribe(() => console.log('Observador 2'));
// Se comparte un listener
```

## Consideraciones de Rendimiento

El rendimiento debe considerarse al manejar eventos que se disparan con alta frecuencia (scroll, mousemove, resize, etc.).

> [!TIP]
> **Optimización de Eventos de Alta Frecuencia**:
> - `throttleTime()` - Procesar solo una vez cada cierto período de tiempo
> - `debounceTime()` - Procesar después de que la entrada se detenga
> - `distinctUntilChanged()` - Procesar solo cuando el valor cambia

#### ❌ Problema de Rendimiento - Procesar en cada resize

```typescript
import { fromEvent } from 'rxjs';

const resize$ = fromEvent(window, 'resize');

resize$.subscribe(() => {
  console.log('Procesamiento de resize'); // Procesamiento pesado
});
```

#### ✅ Optimización - Procesar solo una vez cada 200ms

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

const resize$ = fromEvent(window, 'resize');
resize$.pipe(
  throttleTime(200)
).subscribe(() => {
  console.log('Procesamiento de resize'); // Reducción de carga
});
```

## Funciones de Creación Relacionadas

| Función | Diferencia | Uso |
|----------|------|----------|
| **[from()](/es/guide/creation-functions/basic/from)** | Convertir desde array/Promise | Transmitir datos que no son eventos |
| **[interval()](/es/guide/creation-functions/basic/interval)** | Emitir a intervalos regulares | Procesamiento periódico necesario |
| **fromEventPattern()** | Registro de eventos personalizado | Sistemas de eventos personalizados distintos de EventEmitter |

## Resumen

- `fromEvent()` convierte eventos DOM y EventEmitter a Observable
- Registra listeners al suscribirse, elimina automáticamente al desuscribirse (previene fugas de memoria)
- Funciona como Hot Observable
- Siempre realizar desuscripción para prevenir fugas de memoria
- Optimizar eventos de alta frecuencia con `throttleTime()` y `debounceTime()`

## Próximos Pasos

- [interval() - Emitir Valores a Intervalos Regulares](/es/guide/creation-functions/basic/interval)
- [timer() - Comenzar a Emitir Después de un Retraso](/es/guide/creation-functions/basic/timer)
- [Volver a Funciones de Creación Básicas](/es/guide/creation-functions/basic/)
