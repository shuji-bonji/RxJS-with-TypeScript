---
description: This course explains how to use fromEvent to handle DOM events as Observable. Practical examples will be presented ranging from streaming click, mouse movement, and keyboard events to implementing complex UI processing such as drag & drop and form validation.
---

# Streaming Events

This section provides a comprehensive introduction to creating Observable in RxJS, from basic syntax to practical applications.

## Traditional Event Handling vs. RxJS

### Click Events
#### ◇ Traditional DOM Event Handling

```ts
document.addEventListener('click', (event) => {
  console.log('Clicked:', event);
});

// Result:
// Clicked: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

#### ◆ Event Handling with RxJS

```ts
import { fromEvent } from 'rxjs';

// Streaming click events
const clicks$ = fromEvent(document, 'click');
clicks$.subscribe(event => console.log('RxJS click:', event));

// Result:
// RxJS click: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Mouse Movement Events
#### ◇ Traditional DOM Event Handling
```ts
document.addEventListener('mousemove', (event) => {
  console.log('Mouse position:', event.clientX, event.clientY);
});

// Result:
// Mouse position: 4 357
// Mouse position: 879 148
// Mouse position: 879 148
```

#### ◆ Event Handling with RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, throttleTime } from 'rxjs';

// Mouse move event streaming (with throttling)
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  throttleTime(100), // Limit to every 100 milliseconds
  map(event => ({ x: event.clientX, y: event.clientY }))
);
mouseMove$.subscribe(position => console.log('Mouse position:', position));

// Result:
// Mouse position: {x: 177, y: 453}
// Mouse position: {x: 1239, y: 297}
```

### Keyboard Events
#### ◇ Traditional DOM Event Handling
```ts
document.addEventListener('keydown', (event) => {
  console.log('Key pressed:', event.key);
});

// Result:
// Key pressed: h
// Key pressed: o
// Key pressed: g
// Key pressed: e
```

#### ◆ Event Handling with RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Keyboard event streaming
const keyDown$ = fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  filter(key => key.length === 1) // Single character only
);
keyDown$.subscribe(key => console.log('Key pressed:', key));

// Result:
// Key pressed: h
// Key pressed: o
// Key pressed: g
// Key pressed: e
```

## How to Use fromEvent and Applications

`fromEvent` is the most common way to convert DOM events into Observables. `fromEvent` is the most basic Event → Observable conversion function and is the starting point for event processing with RxJS.

### Basic Usage
```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe((event) => console.log('RxJS click:', event));

// Result:
// RxJS click: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Specifying Event Target and Type
```ts
import { fromEvent } from 'rxjs';

const myButton = document.querySelector('#myButton')!;
const buttonClicks$ = fromEvent<MouseEvent>(myButton, 'click');
buttonClicks$.subscribe((event) => console.log('myButton click:', event));

// Result:
// myButton click: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Specifying Options (Listening in Capture Phase)
```ts
import { fromEvent } from 'rxjs';

const capturedClicks$ = fromEvent(document, 'click', { capture: true });
capturedClicks$.subscribe((event) => console.log('Page click:', event));

// Result:
// Page click: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!NOTE]
> There are two phases of DOM event propagation: "capturing" and "bubbling".
> Normally it is "bubbling" (events propagate from the child element to the parent element), but if `capture: true` is specified, it listens in the "capture phase" (propagation from the parent element to the child element).
> This allows events to be detected by the parent element before they are processed by the child element.

## Handling Multiple Event Sources

RxJS allows multiple event sources to be merged into a common logic via `merge` or `combineLatest`.

```ts
import { fromEvent, merge } from 'rxjs';
import { map } from 'rxjs';

// Integrate clicks from multiple buttons
const button1Clicks$ = fromEvent(document.querySelector('#button1')!, 'click')
  .pipe(map(() => 'Button 1 was clicked'));

const button2Clicks$ = fromEvent(document.querySelector('#button2')!, 'click')
  .pipe(map(() => 'Button 2 was clicked'));

// Merge both event streams
const allButtonClicks$ = merge(button1Clicks$, button2Clicks$);
allButtonClicks$.subscribe(message => console.log(message));
```

#### Execution Result
```
Button 1 was clicked
```
```
Button 2 was clicked
```

## Converting and Manipulating Event Streams

The advantage of streaming events is that they can be easily converted and manipulated using RxJS operators.

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  debounceTime,
  distinctUntilChanged,
} from 'rxjs';

// Monitor changes in input field values
const input$ = fromEvent<InputEvent>(
  document.querySelector('#searchInput')!,
  'input'
).pipe(
  map((event) => (event.target as HTMLInputElement).value),
  filter((text) => text.length > 2), // Process only if 3 or more characters
  debounceTime(300), // Space 300ms interval (doesn't fire while typing)
  distinctUntilChanged() // Doesn't fire if the value is the same as the previous one
);

input$.subscribe((searchText) => {
  console.log('Search text:', searchText);
  // Call search API here, etc.
});
```

#### Execution Result
```sh
Search text: abc
Search text: abcd
```
Thus, by treating input events and other events as streams, the responsiveness and maintainability of the UI can be greatly improved.

## Example of Drag & Drop Implementation

As an example of using a combination of multiple events, let's try managing mouse drag operations with Observable.

```ts
import { fromEvent } from 'rxjs';
import { map, switchMap, takeUntil, tap } from 'rxjs';

function implementDragAndDrop(element: HTMLElement) {
  // Mouse down event stream
  const mouseDown$ = fromEvent<MouseEvent>(element, 'mousedown');

  // Mouse move event stream on document
  const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');

  // Mouse up event stream on document
  const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

  // Drag processing
  const drag$ = mouseDown$.pipe(
    tap(event => {
      // Prevent browser's default drag processing
      event.preventDefault();
    }),
    switchMap(startEvent => {
      // Record initial position
      const initialX = startEvent.clientX;
      const initialY = startEvent.clientY;
      const elementX = parseInt(element.style.left || '0', 10);
      const elementY = parseInt(element.style.top || '0', 10);

      // Return mouse move stream (until mouseUp)
      return mouseMove$.pipe(
        map(moveEvent => ({
          x: elementX + (moveEvent.clientX - initialX),
          y: elementY + (moveEvent.clientY - initialY)
        })),
        takeUntil(mouseUp$) // End on mouse up
      );
    })
  );

  // Subscribe to drag stream
  drag$.subscribe(position => {
    element.style.left = `${position.x}px`;
    element.style.top = `${position.y}px`;
    console.log(`${position.x}px, ${position.y}px`);
  });
}

// Usage example
const draggableElement = document.querySelector('#draggable') as HTMLElement;
implementDragAndDrop(draggableElement);
```

#### Execution Result
```
1px, 0px
1px, -1px
0px, -2px
0px, -3px
0px, -4px
```

## Observe and Validate Form Input

Typical UI processes such as form validation can be written more declaratively and safely with Observable.

```ts
import { fromEvent, combineLatest } from 'rxjs';
import { map, startWith, debounceTime } from 'rxjs';

function validateForm() {
  // Input field references
  const usernameInput = document.querySelector('#username') as HTMLInputElement;
  const passwordInput = document.querySelector('#password') as HTMLInputElement;
  const submitButton = document.querySelector('#submit') as HTMLButtonElement;

  // Input field change stream
  const username$ = fromEvent<InputEvent>(usernameInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Initial value
  );

  const password$ = fromEvent<InputEvent>(passwordInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Initial value
  );

  // Validate by combining both inputs
  const formValid$ = combineLatest([username$, password$]).pipe(
    debounceTime(300),
    map(([username, password]) => {
      return username.length >= 3 && password.length >= 6;
    })
  );

  // Toggle button enabled/disabled based on form validation state
  formValid$.subscribe(isValid => {
    submitButton.disabled = !isValid;
  });

  // Form submission processing
  const submit$ = fromEvent(submitButton, 'click');
  submit$.subscribe(() => {
    console.log('Form submit:', {
      username: usernameInput.value,
      password: passwordInput.value
    });
    // Perform actual submission processing here
  });
}

// Usage example
validateForm();
```
#### Execution Result
```
Form submit: {username: 'testuser', password: '123456'}
```

## Link to Event List

A list of all events and their availability for `fromEvent` can be found at the following link:

➡️ **[Event List Table](./events-list.md)**

This list is useful for reactive programming with RxJS, since it clearly indicates whether standard JavaScript events are supported by `fromEvent`.

## Events That Cannot Be Used with fromEvent {#cannot-used-fromEvent}

`fromEvent` relies on the `EventTarget` interface of the DOM. Therefore, the following events cannot be handled directly by `fromEvent`. These are tied to specific objects or have their own event listeners.

| Event Name         | Type                  | Reason                                             |
| ------------------ | --------------------- | ------------------------------------------------ |
| `beforeunload`    | `BeforeUnloadEvent`  | Event executed before the window is closed, depends on browser behavior rather than DOM event listeners |
| `unload`          | `Event`              | When the page is completely closed, listeners are also deleted, so invalid in RxJS Observable |
| `message`         | `MessageEvent`       | Messages from ServiceWorker or WebWorker cannot be captured directly with `fromEvent` |
| `popstate`        | `PopStateEvent`      | Changes to `history.pushState` or `replaceState` require manual handling |
| `storage`         | `StorageEvent`       | Changes to `localStorage` cannot be monitored with `fromEvent` (need to go through `window.addEventListener`) |
| `languagechange`  | `Event`              | Browser settings changes depend on the behavior of the `window` object |
| `fetch`           | `Event`              | `fetch` progress (such as `onprogress`) is not a normal DOM event |
| `WebSocket`       | `Event`              | `onmessage`, `onopen`, `onclose` have their own event listeners |
| `ServiceWorker`   | `Event`              | `message`, `install`, `activate`, etc. cannot be handled with `fromEvent` |

### Alternative Methods
If you wish to monitor these events, use the following methods:

- `window.addEventListener('message', callback)`
- `window.addEventListener('popstate', callback)`
- `window.addEventListener('storage', callback)`
- For `WebSocket`, `ws.addEventListener('message', callback)`
- For `ServiceWorker`, `navigator.serviceWorker.addEventListener('message', callback)`

When wrapped in RxJS, instead of `fromEvent`, you can manually generate an Observable as follows:

```typescript
import { Observable } from 'rxjs';

const message$ = new Observable<MessageEvent>(observer => {
  const handler = (event: MessageEvent) => observer.next(event);
  window.addEventListener('message', handler);

  // Cleanup processing
  return () => window.removeEventListener('message', handler);
});

message$.subscribe(event => {
  console.log('Message received:', event.data);
});
```

## Summary and Best Practices

In this article, we have seen the benefits and specific applications of making events Observable.

Event processing with RxJS offers the following advantages:

- Declarative and structured event management is possible
- Easy filtering, transformation, and deferral of events through `pipe()` and operators
- Clearly expressed integration of multiple event sources and control of complex states
- Centralized management of side effects via `subscribe`

### Best Practices

- `fromEvent` for each UI component should be properly `unsubscribe` (use `takeUntil`, etc.)
- Stabilize DOM references with null checks and explicit `!`
- Streams should be divided into small parts, and be aware of the use of `switchMap` and `mergeMap`
- Combination with backend communication can be controlled by `exhaustMap`, `concatMap`, etc.

Event streaming with RxJS is more than just click and keydown processing, it is **the basic design concept of the entire reactive UI construction**.
