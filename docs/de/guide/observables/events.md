---
description: "Erklärung, wie DOM-Events mit fromEvent als Observable behandelt werden. Praktische Vorstellung von der Umwandlung von Klicks, Mausbewegungen, Tastatur, Formulareingaben in Streams, über Drag & Drop-Implementierung, Event-Delegation-Muster bis zur typsicheren Event-Verarbeitung in TypeScript."
---

# Event-Umwandlung in Streams

Hier stellen wir die Methoden zur Erstellung von Observables in RxJS umfassend vor, von grundlegender Syntax bis zu praktischen Anwendungsfällen.

## Vergleich zwischen traditioneller Event-Verarbeitung und RxJS

### Click-Event
#### ◇ Traditionelle DOM-Event-Verarbeitung

```ts
document.addEventListener('click', (event) => {
  console.log('Geklickt:', event);
});

// Verarbeitungsergebnis:
// Geklickt: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

#### ◆ Event-Verarbeitung mit RxJS

```ts
import { fromEvent } from 'rxjs';

// Click-Event in Stream umwandeln
const clicks$ = fromEvent(document, 'click');
clicks$.subscribe(event => console.log('RxJS-Klick:', event));

// Verarbeitungsergebnis:
// RxJS-Klick: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Mausbewegung-Event
#### ◇ Traditionelle DOM-Event-Verarbeitung
```ts
document.addEventListener('mousemove', (event) => {
  console.log('Mausposition:', event.clientX, event.clientY);
});

// Verarbeitungsergebnis:
// Mausposition: 4 357
// Mausposition: 879 148
// Mausposition: 879 148
```

#### ◆ Event-Verarbeitung mit RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, throttleTime } from 'rxjs';

// Mausbewegung-Event in Stream umwandeln (mit Throttling)
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  throttleTime(100), // Auf alle 100 Millisekunden beschränken
  map(event => ({ x: event.clientX, y: event.clientY }))
);
mouseMove$.subscribe(position => console.log('Mausposition:', position));

// Verarbeitungsergebnis:
// Mausposition: {x: 177, y: 453}
// Mausposition: {x: 1239, y: 297}
```

### Tastatur-Event
#### ◇ Traditionelle DOM-Event-Verarbeitung
```ts
document.addEventListener('keydown', (event) => {
  console.log('Taste gedrückt:', event.key);
});

// Verarbeitungsergebnis:
// Taste gedrückt: h
// Taste gedrückt: o
// Taste gedrückt: g
// Taste gedrückt: e
```

#### ◆ Event-Verarbeitung mit RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Tastatur-Event in Stream umwandeln
const keyDown$ = fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  filter(key => key.length === 1) // Nur einzelne Zeichen (Modifier-Tasten ausschließen)
);
keyDown$.subscribe(key => console.log('Gedrückte Taste:', key));

// Verarbeitungsergebnis:
// Gedrückte Taste: h
// Gedrückte Taste: o
// Gedrückte Taste: g
// Gedrückte Taste: e
```


## Verwendung und Anwendung von fromEvent

`fromEvent` ist die häufigste Methode, DOM-Events in Observables umzuwandeln. `fromEvent` ist die grundlegendste Event → Observable Konvertierungsfunktion und der Ausgangspunkt für Event-Verarbeitung mit RxJS.

### Grundlegende Verwendung
```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe((event) => console.log('RxJS-Klick:', event));

// Verarbeitungsergebnis:
// RxJS-Klick: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```


### Event-Target und Typ-Spezifikation
```ts
import { fromEvent } from 'rxjs';

const myButton = document.querySelector('#myButton')!;
const buttonClicks$ = fromEvent<MouseEvent>(myButton, 'click');
buttonClicks$.subscribe((event) => console.log('myButton-Klick:', event));

// Verarbeitungsergebnis:
// myButton-Klick: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Optionen angeben (Listening in Capture-Phase)
```ts
import { fromEvent } from 'rxjs';

const capturedClicks$ = fromEvent(document, 'click', { capture: true });
capturedClicks$.subscribe((event) => console.log('Seiten-Klick:', event));

// Verarbeitungsergebnis:
// Seiten-Klick: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!NOTE]
> DOM-Event-Propagation hat zwei Phasen: "Capture" und "Bubbling".
> Normalerweise ist es "Bubbling" (Event propagiert von Kindelement zu Elternelement), aber mit `capture: true` wird in der "Capture-Phase" (Propagation von Elternelement zu Kindelement) gelauscht.
> Dadurch kann das Elternelement das Event erkennen, bevor das Kindelement es verarbeitet.

## Verarbeitung mehrerer Event-Quellen

In RxJS können mehrere Event-Quellen durch `merge` oder `combineLatest` integriert und zu gemeinsamer Logik konsolidiert werden.

```ts
import { fromEvent, merge } from 'rxjs';
import { map } from 'rxjs';

// Klicks von mehreren Buttons integrieren
const button1Clicks$ = fromEvent(document.querySelector('#button1')!, 'click')
  .pipe(map(() => 'Button 1 wurde geklickt'));

const button2Clicks$ = fromEvent(document.querySelector('#button2')!, 'click')
  .pipe(map(() => 'Button 2 wurde geklickt'));

// Beide Event-Streams mergen
const allButtonClicks$ = merge(button1Clicks$, button2Clicks$);
allButtonClicks$.subscribe(message => console.log(message));
```

#### Ausführungsergebnis
```
Button 1 wurde geklickt
```
```
Button 2 wurde geklickt
```


## Transformation und Manipulation von Event-Streams

Der Vorteil der Umwandlung von Events in Streams ist, dass Transformation und Manipulation mit RxJS-Operatoren einfach durchgeführt werden können.

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  debounceTime,
  distinctUntilChanged,
} from 'rxjs';

// Überwachung von Wertänderungen im Eingabefeld
const input$ = fromEvent<InputEvent>(
  document.querySelector('#searchInput')!,
  'input'
).pipe(
  map((event) => (event.target as HTMLInputElement).value),
  filter((text) => text.length > 2), // Nur bei 3 oder mehr Zeichen verarbeiten
  debounceTime(300), // 300ms Abstand lassen (feuert nicht während des Tippens)
  distinctUntilChanged() // Feuert nicht bei gleichem Wert wie zuvor
);

input$.subscribe((searchText) => {
  console.log('Suchtext:', searchText);
  // Hier Such-API aufrufen etc.
});

```

#### Ausführungsergebnis
```sh
Suchtext: abc
Suchtext: abcd
```
Auf diese Weise kann durch die Behandlung von Eingabe-Events etc. als Streams die Reaktionsfähigkeit und Wartbarkeit der UI erheblich verbessert werden.

## Beispiel-Implementierung Drag & Drop

Als Beispiel für die Kombination mehrerer Events verwalten wir Maus-Drag-Operationen mit Observables.

```ts
import { fromEvent } from 'rxjs';
import { map, switchMap, takeUntil, tap } from 'rxjs';

function implementDragAndDrop(element: HTMLElement) {
  // Mousedown-Event-Stream
  const mouseDown$ = fromEvent<MouseEvent>(element, 'mousedown');

  // Mausbewegung-Event-Stream auf Dokument
  const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');

  // Mouseup-Event-Stream auf Dokument
  const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

  // Drag-Verarbeitung
  const drag$ = mouseDown$.pipe(
    tap(event => {
      // Browser-Standard-Drag-Verarbeitung verhindern
      event.preventDefault();
    }),
    switchMap(startEvent => {
      // Anfangsposition aufzeichnen
      const initialX = startEvent.clientX;
      const initialY = startEvent.clientY;
      const elementX = parseInt(element.style.left || '0', 10);
      const elementY = parseInt(element.style.top || '0', 10);

      // Mausbewegung-Stream zurückgeben (bis mouseUp)
      return mouseMove$.pipe(
        map(moveEvent => ({
          x: elementX + (moveEvent.clientX - initialX),
          y: elementY + (moveEvent.clientY - initialY)
        })),
        takeUntil(mouseUp$) // Bei Mouseup beenden
      );
    })
  );

  // Subscriben und Position aktualisieren
  drag$.subscribe(position => {
    element.style.left = `${position.x}px`;
    element.style.top = `${position.y}px`;
    console.log(`${element.style.left}, ${element.style.top}`);
  });
}

// Verwendungsbeispiel
const draggableElement = document.querySelector('#draggable') as HTMLElement;
implementDragAndDrop(draggableElement);
```

#### Ausführungsergebnis
```
1px, 0px
1px, -1px
0px, -2px
0px, -3px
0px, -4px
```

## Formular-Eingabe-Überwachung und -Validierung

Auch typische UI-Verarbeitung wie Formularvalidierung kann mit Observables deklarativer und sicherer beschrieben werden.

```ts
import { fromEvent, combineLatest } from 'rxjs';
import { map, startWith, debounceTime } from 'rxjs';

function validateForm() {
  // Referenz auf Eingabefelder
  const usernameInput = document.querySelector('#username') as HTMLInputElement;
  const passwordInput = document.querySelector('#password') as HTMLInputElement;
  const submitButton = document.querySelector('#submit') as HTMLButtonElement;

  // Änderungs-Streams der Eingabefelder
  const username$ = fromEvent<InputEvent>(usernameInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Anfangswert
  );

  const password$ = fromEvent<InputEvent>(passwordInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Anfangswert
  );

  // Beide Eingaben kombinieren und validieren
  const formValid$ = combineLatest([username$, password$]).pipe(
    debounceTime(300),
    map(([username, password]) => {
      return username.length >= 3 && password.length >= 6;
    })
  );

  // Button aktivieren/deaktivieren basierend auf Formular-Validierungsstatus
  formValid$.subscribe(isValid => {
    submitButton.disabled = !isValid;
  });

  // Formular-Absende-Verarbeitung
  const submit$ = fromEvent(submitButton, 'click');
  submit$.subscribe(() => {
    console.log('Formular abgesendet:', {
      username: usernameInput.value,
      password: passwordInput.value
    });
    // Hier tatsächliche Absende-Verarbeitung durchführen
  });
}

// Verwendungsbeispiel
validateForm();
```
#### Ausführungsergebnis
```
Formular abgesendet: {username: 'testuser', password: '123456'}
```

## Link zur Event-Übersicht

Eine Übersicht aller Events und deren Verfügbarkeit in `fromEvent` kann über folgenden Link eingesehen werden.

➡️ **[Event-Übersichtstabelle](./events-list.md)**

Diese Übersicht zeigt klar, ob JavaScript-Standard-Events mit `fromEvent` kompatibel sind, was bei reaktiver Programmierung mit RxJS praktisch ist.


## Events, die mit fromEvent nicht verwendbar sind {#cannot-used-fromEvent}

`fromEvent` hängt vom DOM-`EventTarget`-Interface ab. Daher können folgende Events nicht direkt mit `fromEvent` behandelt werden. Diese sind an spezifische Objekte gebunden oder haben eigene Event-Listener.

| Event-Name         | Typ                    | Grund                                             |
| ------------------ | --------------------- | ------------------------------------------------ |
| `beforeunload`    | `BeforeUnloadEvent`  | Event vor Schließen des Fensters, abhängig vom Browser-Verhalten, nicht von DOM-Event-Listenern |
| `unload`          | `Event`              | Listener wird auch entfernt, wenn Seite vollständig geschlossen wird, daher in RxJS Observable ungültig |
| `message`         | `MessageEvent`       | Nachrichten von ServiceWorker oder WebWorker können nicht direkt mit `fromEvent` erfasst werden |
| `popstate`        | `PopStateEvent`      | Änderungen durch `history.pushState` oder `replaceState` erfordern manuelles Handling |
| `storage`         | `StorageEvent`       | Änderungen in `localStorage` können nicht mit `fromEvent` überwacht werden (über `window.addEventListener` erforderlich) |
| `languagechange`  | `Event`              | Änderungen der Browser-Einstellungen hängen vom Verhalten des `window`-Objekts ab |
| `fetch`           | `Event`              | `fetch`-Fortschritt (`onprogress` etc.) ist kein normales DOM-Event |
| `WebSocket`       | `Event`              | `onmessage`, `onopen`, `onclose` haben eigene Event-Listener |
| `ServiceWorker`   | `Event`              | `message`, `install`, `activate` etc. können nicht mit `fromEvent` behandelt werden |

### Alternative Methoden
Um diese Events zu überwachen, verwenden Sie folgende Methoden:

- `window.addEventListener('message', callback)`
- `window.addEventListener('popstate', callback)`
- `window.addEventListener('storage', callback)`
- Bei `WebSocket`: `ws.addEventListener('message', callback)`
- Bei `ServiceWorker`: `navigator.serviceWorker.addEventListener('message', callback)`

Zum Wrappen in RxJS können Sie anstelle von `fromEvent` Observables manuell wie folgt erstellen:

```typescript
import { Observable } from 'rxjs';

const message$ = new Observable<MessageEvent>(observer => {
  const handler = (event: MessageEvent) => observer.next(event);
  window.addEventListener('message', handler);

  // Aufhebungsverarbeitung
  return () => window.removeEventListener('message', handler);
});

message$.subscribe(event => {
  console.log('Nachricht empfangen:', event.data);
});
```

## Zusammenfassung und Best Practices

In diesem Artikel haben wir die Vorteile und konkrete Anwendungsmethoden der Umwandlung von Events in Observables gesehen.

Event-Verarbeitung mit RxJS bietet folgende Vorteile:

- Deklaratives und strukturiertes Event-Management möglich
- Filterung, Transformation und verzögerte Verarbeitung von Events durch `pipe()` und Operatoren einfach
- Integration mehrerer Event-Quellen und Steuerung komplexer Zustände klar ausdrückbar
- Zentrale Verwaltung von Seiteneffekten durch `subscribe`

### Best Practices

- Pro UI-Komponente `fromEvent` entsprechend `unsubscribe` (Verwendung von `takeUntil` etc.)
- DOM-Referenzen mit Null-Check oder explizitem `!` stabilisieren
- Streams fein aufteilen, Unterscheidung zwischen `switchMap` und `mergeMap` beachten
- Kombination mit Backend-Kommunikation mit `exhaustMap` oder `concatMap` etc. steuerbar

Event-Streaming mit RxJS wird über einfache Click- oder Keydown-Verarbeitung hinaus zur **grundlegenden Designphilosophie für reaktive UI-Konstruktion insgesamt**.
