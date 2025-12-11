---
description: "fromEvent() - Creation Function die DOM events en EventEmitters converteert naar Observables. Basis voor event-driven programmering met clicks, keyboard input, mouse movements, scroll, etc. TypeScript type definities en event target specificatie uitgelegd."
---

# fromEvent() - Converteer Events naar Observable

`fromEvent()` is een Creation Function die DOM events en Node.js EventEmitters converteert naar Observable streams.

## Overzicht

`fromEvent()` maakt event-gebaseerde async operaties verwerkbaar in RxJS pipelines. Het registreert automatisch event listeners bij subscription en verwijdert ze bij unsubscription, waardoor het risico op memory leaks aanzienlijk wordt verminderd.

**Signature**:
```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

**Officiële Documentatie**: [📘 RxJS Official: fromEvent()](https://rxjs.dev/api/index/function/fromEvent)

## Basisgebruik

Het eenvoudigste voorbeeld van DOM events als Observable behandelen.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Button geklikt:', event);
});

// Bij elke klik wordt event geëmit
```

## Belangrijke Kenmerken

### 1. Automatische Listener Registratie/Verwijdering

`fromEvent()` registreert event listeners bij subscription en verwijdert ze automatisch bij unsubscription.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent<MouseEvent>(document, 'click');

const subscription = clicks$.subscribe(event => {
  console.log('Klik positie:', event.clientX, event.clientY);
});

// Na 5 seconden unsubscriben (event listener wordt automatisch verwijderd)
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Unsubscribed');
}, 5000);
```

> [!IMPORTANT]
> **Memory Leak Preventie**
>
> Bij `unsubscribe()` wordt intern automatisch `removeEventListener()` uitgevoerd. Hierdoor hoef je listeners niet handmatig te verwijderen, wat het risico op memory leaks aanzienlijk vermindert.

### 2. Cold Observable (Elke Subscription Registreert Onafhankelijke Listener)

De Observable van `fromEvent()` is een **Cold Observable**. Bij elke subscription wordt een onafhankelijke event listener geregistreerd.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Subscription 1 - Registreert listener A
clicks$.subscribe(() => console.log('Observer 1: Klik'));

// Na 1 seconde subscription 2 toevoegen - Registreert onafhankelijke listener B
setTimeout(() => {
  clicks$.subscribe(() => console.log('Observer 2: Klik'));
}, 1000);

// Bij één klik worden beide listeners geactiveerd
// Dit bewijst dat elke subscription een onafhankelijke listener heeft
```

> [!NOTE]
> **Cold Observable Bewijs**
>
> Bij elke subscription wordt een nieuwe event listener geregistreerd en bij unsubscription verwijderd. Dit is typisch Cold Observable gedrag. Echter, omdat de event source (DOM element, etc.) extern is en gedeeld wordt, heeft het ook Hot eigenschappen: "events voor subscription worden niet ontvangen".

### 3. TypeScript Type Support

Event types kunnen expliciet worden gespecificeerd.

```typescript
import { fromEvent } from 'rxjs';

const input = document.createElement('input');
input.type = 'text';
document.body.appendChild(input);
const input$ = fromEvent<InputEvent>(input, 'input');

input$.subscribe(event => {
  // event type is InputEvent
  const target = event.target as HTMLInputElement;
  console.log('Invoerwaarde:', target.value);
});
```

### 4. Cold Observable

`fromEvent()` is een **Cold Observable**. Bij elke subscription start een onafhankelijke uitvoering.

```typescript
import { fromEvent } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Subscribe";
document.body.appendChild(button);

const clicks$ = fromEvent(document, 'click');

// Eerste subscription - Event listener wordt toegevoegd
clicks$.subscribe(() => console.log('Subscriber A'));

// Tweede subscription - Aparte event listener wordt toegevoegd
clicks$.subscribe(() => console.log('Subscriber B'));

// Bij één klik worden beide listeners geactiveerd
// Output:
// Subscriber A
// Subscriber B
```

> [!NOTE]
> **Cold Observable Kenmerken**
> - Bij elke subscription start een onafhankelijke uitvoering
> - Elke subscriber ontvangt zijn eigen datastream
> - Bij elke subscription wordt een onafhankelijke event listener geregistreerd. Bij unsubscribe wordt de listener automatisch verwijderd.
>
> Zie [Cold en Hot Observables](/nl/guide/observables/cold-and-hot-observables) voor meer details.

## Praktische Gebruikssituaties

### 1. Click Event Verwerking

Controleer button clicks en voorkom opeenvolgende clicks.

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const button = document.createElement('button');
button.innerText = "submit";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  debounceTime(300), // Negeer clicks binnen 300ms
  map(() => 'Verzenden...')
).subscribe(message => {
  console.log(message);
  // API aanroep, etc.
});
```

### 2. Realtime Formulier Validatie

Stream input events en voer realtime validatie uit.

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
  debounceTime(500), // Verwerk 500ms na invoer stop
  distinctUntilChanged() // Alleen bij gewijzigde waarde
).subscribe(email => {
  console.log('Validatie doel:', email);
  // E-mail validatie
  validateEmail(email);
});

function validateEmail(email: string): void {
  const isValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  console.log(isValid ? 'Geldig e-mailadres' : 'Ongeldig e-mailadres');
}
```

### 3. Drag & Drop Implementatie

Combineer mouse events voor drag & drop.

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, takeUntil, map } from 'rxjs';

// Maak draggable element
const element = document.createElement('div');
element.style.width = '100px';
element.style.height = '100px';
element.style.backgroundColor = '#333';
element.style.position = 'absolute';
element.style.left = '50px';
element.style.top = '50px';
element.style.cursor = 'move';
document.body.appendChild(element);

const mousedown$ = fromEvent<MouseEvent>(element, 'mousedown');
const mousemove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseup$ = fromEvent<MouseEvent>(document, 'mouseup');

mousedown$.pipe(
  switchMap(startEvent => {
    // Registreer klik positie binnen element
    const startX = startEvent.clientX - element.offsetLeft;
    const startY = startEvent.clientY - element.offsetTop;

    return mousemove$.pipe(
      map(moveEvent => ({
        left: moveEvent.clientX - startX,
        top: moveEvent.clientY - startY
      })),
      takeUntil(mouseup$) // Stop bij mouse up
    );
  })
).subscribe(({ left, top }) => {
  // Update element positie
  element.style.left = `${left}px`;
  element.style.top = `${top}px`;
});
```

### 4. Scroll Event Monitoring

Gebruik voor infinite scroll of scroll positie tracking.

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

const scroll$ = fromEvent(window, 'scroll');

scroll$.pipe(
  throttleTime(200), // Verwerk slechts 1x per 200ms
  map(() => window.scrollY)
).subscribe(scrollPosition => {
  console.log('Scroll positie:', scrollPosition);

  // Laad extra content bij einde pagina
  if (scrollPosition + window.innerHeight >= document.body.scrollHeight - 100) {
    console.log('Laad extra content');
    // loadMoreContent();
  }
});
```

## Gebruik in Pipelines

`fromEvent()` is ideaal als startpunt voor event stream pipeline verwerking.

```typescript
import { fromEvent } from 'rxjs';
import { map, filter, scan } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Teller";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  filter((event: Event) => {
    // Tel alleen clicks met Shift-toets
    return (event as MouseEvent).shiftKey;
  }),
  scan((count, _) => count + 1, 0),
  map(count => `Aantal clicks: ${count}`)
).subscribe(message => console.log(message));
```

## Veelgemaakte Fouten

### 1. Vergeten Unsubscriben

#### ❌ Fout - Vergeten unsubscriben veroorzaakt memory leaks

```typescript
import { fromEvent } from 'rxjs';

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  clicks$.subscribe(console.log); // Geen unsubscribe!
}

setupEventListener();
```

#### ✅ Correct - Altijd unsubscriben

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
// Roep cleanup() aan bij component destruction, etc.
```

> [!WARNING]
> **Let op Memory Leaks**
>
> In SPAs en component-gebaseerde frameworks moet je altijd unsubscriben bij component destruction. Vergeten unsubscriben laat event listeners achter, wat memory leaks veroorzaakt.

### 2. Dubbele Event Listener Registratie

#### ❌ Fout - Meerdere subscriptions op hetzelfde event registreren meerdere listeners

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Observer 1'));
clicks$.subscribe(() => console.log('Observer 2'));
// Bij klik worden beide logs getoond (2 listeners geregistreerd)
```

#### ✅ Correct - Gebruik share() voor multicast indien nodig

```typescript
import { fromEvent } from 'rxjs';
import { share } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(share());

clicks$.subscribe(() => console.log('Observer 1'));
clicks$.subscribe(() => console.log('Observer 2'));
// Eén listener wordt gedeeld
```

## Prestatieoverwegingen

Bij high-frequency events (scroll, mousemove, resize, etc.) is prestatie-aandacht vereist.

> [!TIP]
> **High-frequency Event Optimalisatie**:
> - `throttleTime()` - Verwerk slechts 1x per tijdsinterval
> - `debounceTime()` - Verwerk na invoer stop
> - `distinctUntilChanged()` - Verwerk alleen bij gewijzigde waarde

#### ❌ Prestatieproblemen - Verwerk bij elke resize

```typescript
import { fromEvent } from 'rxjs';

const resize$ = fromEvent(window, 'resize');

resize$.subscribe(() => {
  console.log('Resize verwerking'); // Zware verwerking
});
```

#### ✅ Optimalisatie - Verwerk slechts 1x per 200ms

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

const resize$ = fromEvent(window, 'resize');
resize$.pipe(
  throttleTime(200)
).subscribe(() => {
  console.log('Resize verwerking'); // Verminderde belasting
});
```

## Gerelateerde Creation Functions

| Function | Verschil | Gebruik |
|----------|----------|---------|
| **[from()](/nl/guide/creation-functions/basic/from)** | Converteer arrays/Promises | Stream niet-event data |
| **[interval()](/nl/guide/creation-functions/basic/interval)** | Emit op vast interval | Periodieke verwerking |
| **fromEventPattern()** | Custom event registratie | Custom event systemen (niet EventEmitter) |

## Samenvatting

- `fromEvent()` converteert DOM events en EventEmitters naar Observables
- Registreert listeners bij subscription, verwijdert automatisch bij unsubscription (memory leak preventie)
- Werkt als Hot Observable
- Altijd unsubscriben voor memory leak preventie
- Optimaliseer high-frequency events met `throttleTime()` of `debounceTime()`

## Volgende Stappen

- [interval() - Emit waarden op vast interval](/nl/guide/creation-functions/basic/interval)
- [timer() - Start emitting na vertraging](/nl/guide/creation-functions/basic/timer)
- [Terug naar basis overzicht](/nl/guide/creation-functions/basic/)
