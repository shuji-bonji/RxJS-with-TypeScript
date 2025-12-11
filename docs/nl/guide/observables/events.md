---
description: "Uitleg over hoe DOM events als Observable te behandelen met fromEvent. Van het stroomvormig maken van klik-, muisbeweging-, toetsenbord- en formulierinvoer events, tot drag & drop implementatie, event delegatiepatronen en type-veilige event handling in TypeScript."
---

# Streamificatie van Events

Hier introduceren we uitgebreid de manieren om Observables in RxJS te maken, van basale syntaxis tot praktische toepassingen.

## Vergelijking van traditionele event handling en RxJS

### Klik event
#### ◇ Traditionele DOM event handling

```ts
document.addEventListener('click', (event) => {
  console.log('Geklikt:', event);
});

// Verwerkingsresultaat:
// Geklikt: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

#### ◆ Event handling met RxJS

```ts
import { fromEvent } from 'rxjs';

// Streamificatie van klik event
const clicks$ = fromEvent(document, 'click');
clicks$.subscribe(event => console.log('RxJS klik:', event));

// Verwerkingsresultaat:
// RxJS klik: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Muisbeweging event
#### ◇ Traditionele DOM event handling
```ts
document.addEventListener('mousemove', (event) => {
  console.log('Muispositie:', event.clientX, event.clientY);
});

// Verwerkingsresultaat:
// Muispositie: 4 357
// Muispositie: 879 148
// Muispositie: 879 148
```

#### ◆ Event handling met RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, throttleTime } from 'rxjs';

// Streamificatie van muisbeweging event (met throttling)
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  throttleTime(100), // Beperken tot elke 100 milliseconden
  map(event => ({ x: event.clientX, y: event.clientY }))
);
mouseMove$.subscribe(position => console.log('Muispositie:', position));

// Verwerkingsresultaat:
// Muispositie: {x: 177, y: 453}
// Muispositie: {x: 1239, y: 297}
```

### Toetsenbord event
#### ◇ Traditionele DOM event handling
```ts
document.addEventListener('keydown', (event) => {
  console.log('Toets ingedrukt:', event.key);
});

// Verwerkingsresultaat:
// Toets ingedrukt: h
// Toets ingedrukt: o
// Toets ingedrukt: g
// Toets ingedrukt: e
```

#### ◆ Event handling met RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Streamificatie van toetsenbord event
const keyDown$ = fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  filter(key => key.length === 1) // Alleen enkele karakters (modifier toetsen uitsluiten)
);
keyDown$.subscribe(key => console.log('Ingedrukte toets:', key));

// Verwerkingsresultaat:
// Ingedrukte toets: h
// Ingedrukte toets: o
// Ingedrukte toets: g
// Ingedrukte toets: e
```


## Gebruik en toepassingen van fromEvent

`fromEvent` is de meest gebruikelijke manier om DOM events naar Observable te converteren. `fromEvent` is de meest basale event → Observable conversiefunctie en vormt het startpunt voor event handling met RxJS.

### Basisgebruik
```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe((event) => console.log('RxJS klik:', event));

// Verwerkingsresultaat:
// RxJS klik: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```


### Event target en type specificatie
```ts
import { fromEvent } from 'rxjs';

const myButton = document.querySelector('#myButton')!;
const buttonClicks$ = fromEvent<MouseEvent>(myButton, 'click');
buttonClicks$.subscribe((event) => console.log('myButton klik:', event));

// Verwerkingsresultaat:
// myButton klik: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Optie specificatie (listening in capture fase)
```ts
import { fromEvent } from 'rxjs';

const capturedClicks$ = fromEvent(document, 'click', { capture: true });
capturedClicks$.subscribe((event) => console.log('Pagina klik:', event));

// Verwerkingsresultaat:
// Pagina klik: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!NOTE]
> DOM event propagatie heeft twee fasen: "capture" en "bubbling".
> Normaal is het "bubbling" (event propageert van kindelement naar ouder element), maar door `capture: true` te specificeren, kun je in de "capture fase" luisteren (propageert van ouder naar kindelement).
> Hierdoor kun je events detecteren in het ouderelement voordat het kindelement ze verwerkt.

## Verwerking van meerdere event sources

In RxJS is het mogelijk om meerdere event sources te integreren met `merge` of `combineLatest` en te consolideren in gemeenschappelijke logica.

```ts
import { fromEvent, merge } from 'rxjs';
import { map } from 'rxjs';

// Integreren van kliks van meerdere knoppen
const button1Clicks$ = fromEvent(document.querySelector('#button1')!, 'click')
  .pipe(map(() => 'Knop 1 geklikt'));

const button2Clicks$ = fromEvent(document.querySelector('#button2')!, 'click')
  .pipe(map(() => 'Knop 2 geklikt'));

// Merge beide event streams
const allButtonClicks$ = merge(button1Clicks$, button2Clicks$);
allButtonClicks$.subscribe(message => console.log(message));
```

#### Uitvoeringsresultaat
```
Knop 1 geklikt
```
```
Knop 2 geklikt
```


## Transformatie en manipulatie van event streams

Het voordeel van het stroomvormig maken van events is dat transformatie en manipulatie eenvoudig kunnen worden uitgevoerd met RxJS operators.

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  debounceTime,
  distinctUntilChanged,
} from 'rxjs';

// Monitoren van wijzigingen in invoerveldenwaarde
const input$ = fromEvent<InputEvent>(
  document.querySelector('#searchInput')!,
  'input'
).pipe(
  map((event) => (event.target as HTMLInputElement).value),
  filter((text) => text.length > 2), // Alleen verwerken bij 3 of meer karakters
  debounceTime(300), // 300ms interval aanhouden (niet afvuren tijdens typen)
  distinctUntilChanged() // Niet afvuren bij dezelfde waarde als vorige keer
);

input$.subscribe((searchText) => {
  console.log('Zoektekst:', searchText);
  // Hier kun je zoek API aanroep uitvoeren
});

```

#### Uitvoeringsresultaat
```sh
Zoektekst: abc
Zoektekst: abcd
```
Zo kun je de reactiviteit en onderhoudbaarheid van de UI aanzienlijk verbeteren door input events en dergelijke als stream te behandelen.

## Drag&Drop implementatievoorbeeld

Als voorbeeld van het combineren van meerdere events, laten we een muissleeoperatie beheren met Observable.

```ts
import { fromEvent } from 'rxjs';
import { map, switchMap, takeUntil, tap } from 'rxjs';

function implementDragAndDrop(element: HTMLElement) {
  // Stream van mousedown event
  const mouseDown$ = fromEvent<MouseEvent>(element, 'mousedown');

  // Stream van muisbeweging event op document
  const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');

  // Stream van mouseup event op document
  const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

  // Drag processing
  const drag$ = mouseDown$.pipe(
    tap(event => {
      // Voorkom standaard browser drag processing
      event.preventDefault();
    }),
    switchMap(startEvent => {
      // Vastleggen van beginpositie
      const initialX = startEvent.clientX;
      const initialY = startEvent.clientY;
      const elementX = parseInt(element.style.left || '0', 10);
      const elementY = parseInt(element.style.top || '0', 10);

      // Retourneer stream van muisbeweging (tot mouseUp)
      return mouseMove$.pipe(
        map(moveEvent => ({
          x: elementX + (moveEvent.clientX - initialX),
          y: elementY + (moveEvent.clientY - initialY)
        })),
        takeUntil(mouseUp$) // Eindigen bij mouse up
      );
    })
  );

  // Abonneren en positie bijwerken
  drag$.subscribe(position => {
    element.style.left = `${position.x}px`;
    element.style.top = `${position.y}px`;
    console.log(`${element.style.left}, ${element.style.top}`);
  });
}

// Gebruiksvoorbeeld
const draggableElement = document.querySelector('#draggable') as HTMLElement;
implementDragAndDrop(draggableElement);
```

#### Uitvoeringsresultaat
```
1px, 0px
1px, -1px
0px, -2px
0px, -3px
0px, -4px
```

## Monitoring en validatie van formulierinvoer

Ook typische UI-processing zoals formuliervalidatie kan declaratiever en veiliger worden geschreven met Observable.

```ts
import { fromEvent, combineLatest } from 'rxjs';
import { map, startWith, debounceTime } from 'rxjs';

function validateForm() {
  // Referenties naar invoervelden
  const usernameInput = document.querySelector('#username') as HTMLInputElement;
  const passwordInput = document.querySelector('#password') as HTMLInputElement;
  const submitButton = document.querySelector('#submit') as HTMLButtonElement;

  // Wijzigingsstreams van invoervelden
  const username$ = fromEvent<InputEvent>(usernameInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Beginwaarde
  );

  const password$ = fromEvent<InputEvent>(passwordInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Beginwaarde
  );

  // Beide invoer combineren en valideren
  const formValid$ = combineLatest([username$, password$]).pipe(
    debounceTime(300),
    map(([username, password]) => {
      return username.length >= 3 && password.length >= 6;
    })
  );

  // Knop in-/uitschakelen op basis van formulier validatiestatus
  formValid$.subscribe(isValid => {
    submitButton.disabled = !isValid;
  });

  // Formulierverzending processing
  const submit$ = fromEvent(submitButton, 'click');
  submit$.subscribe(() => {
    console.log('Formulier verzending:', {
      username: usernameInput.value,
      password: passwordInput.value
    });
    // Hier kun je de daadwerkelijke verzendprocessing uitvoeren
  });
}

// Gebruiksvoorbeeld
validateForm();
```
#### Uitvoeringsresultaat
```
Formulier verzending: {username: 'testuser', password: '123456'}
```

## Link naar eventlijst

Zie onderstaande link voor een lijst van alle events en hun beschikbaarheid in `fromEvent`.

➡️ **[Lijst van Events](./events-list.md)**

Deze lijst toont duidelijk of JavaScript standaard events compatibel zijn met `fromEvent`, wat handig is bij reactieve programmering met RxJS.


## Events die niet kunnen worden gebruikt in fromEvent {#cannot-used-fromEvent}

`fromEvent` is afhankelijk van de `EventTarget` interface van DOM. Daarom kunnen de volgende events niet direct met `fromEvent` worden behandeld. Deze zijn gebonden aan specifieke objecten of hebben hun eigen event listeners.

| Event Naam         | Type                    | Reden                                             |
| ------------------ | --------------------- | ------------------------------------------------ |
| `beforeunload`    | `BeforeUnloadEvent`  | Event dat wordt uitgevoerd voordat venster wordt gesloten, afhankelijk van browsergedrag, niet van DOM event listener |
| `unload`          | `Event`              | Wanneer pagina volledig sluit, wordt listener ook verwijderd, dus ongeldig in RxJS Observable |
| `message`         | `MessageEvent`       | Berichten van ServiceWorker of WebWorker kunnen niet direct worden gevangen met `fromEvent` |
| `popstate`        | `PopStateEvent`      | Wijzigingen via `history.pushState` of `replaceState` vereisen handmatige handling |
| `storage`         | `StorageEvent`       | Wijzigingen in `localStorage` kunnen niet worden gemonitord met `fromEvent` (via `window.addEventListener` nodig) |
| `languagechange`  | `Event`              | Wijzigingen in browserinstellingen zijn afhankelijk van `window` objectgedrag |
| `fetch`           | `Event`              | Voortgang van `fetch` (zoals `onprogress`) zijn geen normale DOM events |
| `WebSocket`       | `Event`              | `onmessage`, `onopen`, `onclose` hebben hun eigen event listeners |
| `ServiceWorker`   | `Event`              | `message`, `install`, `activate` etc. kunnen niet met `fromEvent` worden behandeld |

### Alternatieve methoden
Om deze events te monitoren, gebruik de volgende methoden.

- `window.addEventListener('message', callback)`
- `window.addEventListener('popstate', callback)`
- `window.addEventListener('storage', callback)`
- Bij `WebSocket`, `ws.addEventListener('message', callback)`
- Bij `ServiceWorker`, `navigator.serviceWorker.addEventListener('message', callback)`

Om te wrappen in RxJS, kun je handmatig Observable genereren in plaats van `fromEvent`.

```typescript
import { Observable } from 'rxjs';

const message$ = new Observable<MessageEvent>(observer => {
  const handler = (event: MessageEvent) => observer.next(event);
  window.addEventListener('message', handler);

  // Afmeldprocessing
  return () => window.removeEventListener('message', handler);
});

message$.subscribe(event => {
  console.log('Bericht ontvangen:', event.data);
});
```

## Samenvatting en best practices

In dit artikel hebben we de voordelen en concrete toepassingsmethoden gezien van het Observable maken van events.

Event handling met RxJS heeft de volgende voordelen:

- Declaratief en gestructureerd event management mogelijk
- Via `pipe()` en operators zijn filtering, transformatie en vertragingsprocessing van events eenvoudig
- Ook integratie van meerdere event sources en complexe status controle kunnen duidelijk worden uitgedrukt
- Geconcentreerd beheer van bijeffecten via `subscribe`

### Best Practices

- Passende `unsubscribe` van `fromEvent` per UI-component (gebruik `takeUntil` etc.)
- Stabiliseer DOM-referenties met null checks of `!` expliciet
- Splits streams fijn op en wees bewust van het onderscheid tussen `switchMap` en `mergeMap`
- Combinaties met backend communicatie kunnen worden gecontroleerd met `exhaustMap` of `concatMap` etc.

Event streamificatie met RxJS gaat verder dan simpele klik- of keydown-processing en wordt de **fundamentele ontwerpfilosofie voor de constructie van gehele reactieve UI**.
