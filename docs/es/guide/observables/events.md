---
description: Este curso explica cómo usar fromEvent para manejar eventos DOM como Observable. Se presentarán ejemplos prácticos que van desde el flujo de clics, movimiento del mouse y eventos de teclado hasta la implementación de procesamiento de UI complejo como arrastrar y soltar y validación de formularios.
---

# Flujo de Eventos

Esta sección proporciona una introducción completa a la creación de Observable en RxJS, desde la sintaxis básica hasta las aplicaciones prácticas.

## Manejo de Eventos Tradicional vs. RxJS

### Eventos de Clic
#### ◇ Manejo Tradicional de Eventos DOM

```ts
document.addEventListener('click', (event) => {
  console.log('Clic:', event);
});

// Resultado:
// Clic: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

#### ◆ Manejo de Eventos con RxJS

```ts
import { fromEvent } from 'rxjs';

// Flujo de eventos de clic
const clicks$ = fromEvent(document, 'click');
clicks$.subscribe(event => console.log('Clic RxJS:', event));

// Resultado:
// Clic RxJS: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Eventos de Movimiento del Mouse
#### ◇ Manejo Tradicional de Eventos DOM
```ts
document.addEventListener('mousemove', (event) => {
  console.log('Posición del mouse:', event.clientX, event.clientY);
});

// Resultado:
// Posición del mouse: 4 357
// Posición del mouse: 879 148
// Posición del mouse: 879 148
```

#### ◆ Manejo de Eventos con RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, throttleTime } from 'rxjs';

// Flujo de eventos de movimiento del mouse (con limitación)
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  throttleTime(100), // Limitar a cada 100 milisegundos
  map(event => ({ x: event.clientX, y: event.clientY }))
);
mouseMove$.subscribe(position => console.log('Posición del mouse:', position));

// Resultado:
// Posición del mouse: {x: 177, y: 453}
// Posición del mouse: {x: 1239, y: 297}
```

### Eventos de Teclado
#### ◇ Manejo Tradicional de Eventos DOM
```ts
document.addEventListener('keydown', (event) => {
  console.log('Tecla presionada:', event.key);
});

// Resultado:
// Tecla presionada: h
// Tecla presionada: o
// Tecla presionada: g
// Tecla presionada: e
```

#### ◆ Manejo de Eventos con RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Flujo de eventos de teclado
const keyDown$ = fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  filter(key => key.length === 1) // Solo carácter único
);
keyDown$.subscribe(key => console.log('Tecla presionada:', key));

// Resultado:
// Tecla presionada: h
// Tecla presionada: o
// Tecla presionada: g
// Tecla presionada: e
```

## Cómo Usar fromEvent y Aplicaciones

`fromEvent` es la forma más común de convertir eventos DOM en Observables. `fromEvent` es la función de conversión Evento → Observable más básica y es el punto de partida para el procesamiento de eventos con RxJS.

### Uso Básico
```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe((event) => console.log('Clic RxJS:', event));

// Resultado:
// Clic RxJS: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Especificar Objetivo de Evento y Tipo
```ts
import { fromEvent } from 'rxjs';

const myButton = document.querySelector('#myButton')!;
const buttonClicks$ = fromEvent<MouseEvent>(myButton, 'click');
buttonClicks$.subscribe((event) => console.log('Clic en myButton:', event));

// Resultado:
// Clic en myButton: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Especificar Opciones (Escuchar en Fase de Captura)
```ts
import { fromEvent } from 'rxjs';

const capturedClicks$ = fromEvent(document, 'click', { capture: true });
capturedClicks$.subscribe((event) => console.log('Clic en página:', event));

// Resultado:
// Clic en página: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!NOTE]
> Hay dos fases de propagación de eventos DOM: "captura" y "burbujeo".
> Normalmente es "burbujeo" (los eventos se propagan del elemento hijo al elemento padre), pero si se especifica `capture: true`, escucha en la "fase de captura" (propagación del elemento padre al elemento hijo).
> Esto permite que los eventos sean detectados por el elemento padre antes de ser procesados por el elemento hijo.

## Manejo de Múltiples Fuentes de Eventos

RxJS permite que múltiples fuentes de eventos se fusionen en una lógica común a través de `merge` o `combineLatest`.

```ts
import { fromEvent, merge } from 'rxjs';
import { map } from 'rxjs';

// Integrar clics de múltiples botones
const button1Clicks$ = fromEvent(document.querySelector('#button1')!, 'click')
  .pipe(map(() => 'El botón 1 fue clicado'));

const button2Clicks$ = fromEvent(document.querySelector('#button2')!, 'click')
  .pipe(map(() => 'El botón 2 fue clicado'));

// Fusionar ambos flujos de eventos
const allButtonClicks$ = merge(button1Clicks$, button2Clicks$);
allButtonClicks$.subscribe(message => console.log(message));
```

#### Resultado de Ejecución
```
El botón 1 fue clicado
```
```
El botón 2 fue clicado
```

## Conversión y Manipulación de Flujos de Eventos

La ventaja del flujo de eventos es que se pueden convertir y manipular fácilmente usando operadores de RxJS.

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  debounceTime,
  distinctUntilChanged,
} from 'rxjs';

// Monitorear cambios en valores de campo de entrada
const input$ = fromEvent<InputEvent>(
  document.querySelector('#searchInput')!,
  'input'
).pipe(
  map((event) => (event.target as HTMLInputElement).value),
  filter((text) => text.length > 2), // Procesar solo si hay 3 o más caracteres
  debounceTime(300), // Intervalo de 300ms (no se dispara mientras se escribe)
  distinctUntilChanged() // No se dispara si el valor es el mismo que el anterior
);

input$.subscribe((searchText) => {
  console.log('Texto de búsqueda:', searchText);
  // Llamar a la API de búsqueda aquí, etc.
});
```

#### Resultado de Ejecución
```sh
Texto de búsqueda: abc
Texto de búsqueda: abcd
```
Así, al tratar los eventos de entrada y otros eventos como flujos, la capacidad de respuesta y mantenibilidad de la UI se pueden mejorar significativamente.

## Ejemplo de Implementación de Arrastrar y Soltar

Como ejemplo del uso de una combinación de múltiples eventos, intentemos gestionar operaciones de arrastre del mouse con Observable.

```ts
import { fromEvent } from 'rxjs';
import { map, switchMap, takeUntil, tap } from 'rxjs';

function implementDragAndDrop(element: HTMLElement) {
  // Flujo de eventos de presión del mouse
  const mouseDown$ = fromEvent<MouseEvent>(element, 'mousedown');

  // Flujo de eventos de movimiento del mouse en documento
  const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');

  // Flujo de eventos de liberación del mouse en documento
  const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

  // Procesamiento de arrastre
  const drag$ = mouseDown$.pipe(
    tap(event => {
      // Prevenir procesamiento de arrastre predeterminado del navegador
      event.preventDefault();
    }),
    switchMap(startEvent => {
      // Registrar posición inicial
      const initialX = startEvent.clientX;
      const initialY = startEvent.clientY;
      const elementX = parseInt(element.style.left || '0', 10);
      const elementY = parseInt(element.style.top || '0', 10);

      // Devolver flujo de movimiento del mouse (hasta mouseUp)
      return mouseMove$.pipe(
        map(moveEvent => ({
          x: elementX + (moveEvent.clientX - initialX),
          y: elementY + (moveEvent.clientY - initialY)
        })),
        takeUntil(mouseUp$) // Finalizar al soltar el mouse
      );
    })
  );

  // Suscribirse al flujo de arrastre
  drag$.subscribe(position => {
    element.style.left = `${position.x}px`;
    element.style.top = `${position.y}px`;
    console.log(`${position.x}px, ${position.y}px`);
  });
}

// Ejemplo de uso
const draggableElement = document.querySelector('#draggable') as HTMLElement;
implementDragAndDrop(draggableElement);
```

#### Resultado de Ejecución
```
1px, 0px
1px, -1px
0px, -2px
0px, -3px
0px, -4px
```

## Observar y Validar Entrada de Formulario

Procesos típicos de UI como la validación de formularios se pueden escribir de manera más declarativa y segura con Observable.

```ts
import { fromEvent, combineLatest } from 'rxjs';
import { map, startWith, debounceTime } from 'rxjs';

function validateForm() {
  // Referencias de campo de entrada
  const usernameInput = document.querySelector('#username') as HTMLInputElement;
  const passwordInput = document.querySelector('#password') as HTMLInputElement;
  const submitButton = document.querySelector('#submit') as HTMLButtonElement;

  // Flujo de cambio de campo de entrada
  const username$ = fromEvent<InputEvent>(usernameInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Valor inicial
  );

  const password$ = fromEvent<InputEvent>(passwordInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Valor inicial
  );

  // Validar combinando ambas entradas
  const formValid$ = combineLatest([username$, password$]).pipe(
    debounceTime(300),
    map(([username, password]) => {
      return username.length >= 3 && password.length >= 6;
    })
  );

  // Alternar habilitado/deshabilitado del botón según estado de validación del formulario
  formValid$.subscribe(isValid => {
    submitButton.disabled = !isValid;
  });

  // Procesamiento de envío de formulario
  const submit$ = fromEvent(submitButton, 'click');
  submit$.subscribe(() => {
    console.log('Envío de formulario:', {
      username: usernameInput.value,
      password: passwordInput.value
    });
    // Realizar procesamiento de envío real aquí
  });
}

// Ejemplo de uso
validateForm();
```
#### Resultado de Ejecución
```
Envío de formulario: {username: 'testuser', password: '123456'}
```

## Enlace a Lista de Eventos

Se puede encontrar una lista de todos los eventos y su disponibilidad para `fromEvent` en el siguiente enlace:

➡️ **[Tabla de Lista de Eventos](./events-list.md)**

Esta lista es útil para programación reactiva con RxJS, ya que indica claramente si los eventos JavaScript estándar son compatibles con `fromEvent`.

## Eventos que No se Pueden Usar con fromEvent {#cannot-used-fromEvent}

`fromEvent` depende de la interfaz `EventTarget` del DOM. Por lo tanto, los siguientes eventos no se pueden manejar directamente con `fromEvent`. Estos están vinculados a objetos específicos o tienen sus propios escuchadores de eventos.

| Nombre del Evento | Tipo | Razón |
| ------------------ | --------------------- | ------------------------------------------------ |
| `beforeunload`    | `BeforeUnloadEvent`  | Evento ejecutado antes de cerrar la ventana, depende del comportamiento del navegador en lugar de escuchadores de eventos DOM |
| `unload`          | `Event`              | Cuando la página está completamente cerrada, los escuchadores también se eliminan, por lo que no es válido en Observable de RxJS |
| `message`         | `MessageEvent`       | Los mensajes de ServiceWorker o WebWorker no se pueden capturar directamente con `fromEvent` |
| `popstate`        | `PopStateEvent`      | Los cambios a `history.pushState` o `replaceState` requieren manejo manual |
| `storage`         | `StorageEvent`       | Los cambios a `localStorage` no se pueden monitorear con `fromEvent` (necesita pasar por `window.addEventListener`) |
| `languagechange`  | `Event`              | Los cambios de configuración del navegador dependen del comportamiento del objeto `window` |
| `fetch`           | `Event`              | El progreso de `fetch` (como `onprogress`) no es un evento DOM normal |
| `WebSocket`       | `Event`              | `onmessage`, `onopen`, `onclose` tienen sus propios escuchadores de eventos |
| `ServiceWorker`   | `Event`              | `message`, `install`, `activate`, etc. no se pueden manejar con `fromEvent` |

### Métodos Alternativos
Si desea monitorear estos eventos, use los siguientes métodos:

- `window.addEventListener('message', callback)`
- `window.addEventListener('popstate', callback)`
- `window.addEventListener('storage', callback)`
- Para `WebSocket`, `ws.addEventListener('message', callback)`
- Para `ServiceWorker`, `navigator.serviceWorker.addEventListener('message', callback)`

Cuando se envuelve en RxJS, en lugar de `fromEvent`, puede generar manualmente un Observable de la siguiente manera:

```typescript
import { Observable } from 'rxjs';

const message$ = new Observable<MessageEvent>(observer => {
  const handler = (event: MessageEvent) => observer.next(event);
  window.addEventListener('message', handler);

  // Procesamiento de limpieza
  return () => window.removeEventListener('message', handler);
});

message$.subscribe(event => {
  console.log('Mensaje recibido:', event.data);
});
```

## Resumen y Mejores Prácticas

En este artículo, hemos visto los beneficios y aplicaciones específicas de hacer que los eventos sean Observable.

El procesamiento de eventos con RxJS ofrece las siguientes ventajas:

- Es posible la gestión de eventos declarativa y estructurada
- Filtrado, transformación y aplazamiento fácil de eventos a través de `pipe()` y operadores
- Integración claramente expresada de múltiples fuentes de eventos y control de estados complejos
- Gestión centralizada de efectos secundarios a través de `subscribe`

### Mejores Prácticas

- `fromEvent` para cada componente de UI debe tener un `unsubscribe` apropiado (use `takeUntil`, etc.)
- Estabilizar referencias DOM con verificaciones nulas y `!` explícito
- Los flujos deben dividirse en partes pequeñas, y ser conscientes del uso de `switchMap` y `mergeMap`
- La combinación con comunicación backend puede controlarse mediante `exhaustMap`, `concatMap`, etc.

El flujo de eventos con RxJS es más que solo procesamiento de clic y keydown, es **el concepto de diseño básico de toda la construcción de UI reactiva**.
