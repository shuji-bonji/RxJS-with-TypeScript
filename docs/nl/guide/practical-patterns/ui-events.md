---
description: "Praktische patronen voor UI event processing met RxJS worden uitgelegd. Specifieke implementatievoorbeelden voor het implementeren van interactieve UI zoals controle over klikgebeurtenissen (throttle, debounce, distinct), verwerking van scrollgebeurtenissen, slepen en neerzetten, toetsenbordinvoer (autocomplete) en ondersteuning voor multi-touch worden samen met TypeScript geïntroduceerd. code. U leert efficiënte patronen voor het afhandelen van hoogfrequente gebeurtenissen."
---

# UI gebeurtenis verwerkingspatroon

UI event handling is een van de meest voorkomende uitdagingen in front-end ontwikkeling, en met RxJS kun je complexe event handling declaratief en intuïtief implementeren.

Dit artikel beschrijft specifieke patronen van UI event handling die in de praktijk nodig zijn, zoals klikken, scrollen, slepen en neerzetten en toetsenbordinvoer.

## Wat je in dit artikel leert.

- Klikgebeurtenissen beheren (throttle, debounce, distinct)
- Efficiënte afhandeling van scroll-events
- Slepen en neerzetten implementeren
- Toetsenbordinvoer en autoaanvullen
- Multi-touch ondersteuning
- Samengestelde events combineren

```
Gebruiker klikt: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Verwerking uitvoeren      Verwerking uitvoeren
```

> Dit artikel gaat uit van kennis van [Hoofdstuk 4: Operatoren](. /operators/index.md) en veronderstelt kennis van het volgende. In het bijzonder is een begrip van `debounceTime`, `throttleTime` en `distinctUntilChanged` belangrijk.

## Afhandeling van klikgebeurtenissen.

### Probleem: overmatige verwerking door een reeks klikken.

Opeenvolgende klikken op een knop kunnen resulteren in herhaalde verwerking, wat prestatieproblemen en bugs veroorzaakt.

### Oplossing 1: Controle met throttleTime

Verwerk alleen de eerste klik binnen een bepaalde periode.


```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1In een seconde.1Slechts eenmaal verwerkt
  ).subscribe(() => {
    console.log('Uitvoering van verzendproces');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Tijdens formulierverzending...');
  // APIGesprekken, enz.
}
```

#### Uitvoeringsstroom

```
Gebruiker klikt: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Verwerking uitvoeren      Verwerking uitvoeren
```

Gebruiker klikt: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Verwerking uitvoeren      Verwerking uitvoeren
```

> - Verwerkt **eerste gebeurtenis** en negeert volgende gebeurtenissen gedurende een bepaalde tijd
> - geschikt wanneer real-time belangrijk is (scrollen, formaat wijzigen, enz.)

### Oplossing 2: Controle met debounceTime

Verwerk gebeurtenissen na een bepaalde periode nadat ze zijn gestopt.


```

```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Zoekwoorden invoeren...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Na invoer stopt300msWacht
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Zoekopdracht uitvoeren:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Zoeken in uitvoering...', query);
  // ZoekenAPIOpvragen
}
```

#### Uitvoeringsstroom

```
Invoer gebruiker:  ●●●●●     ●●        ●●●●
                      |            |      |
debounceTime(300):   300ms       300ms  300msWachten
                      |            |      |
                     Verwerking         Verwerking   Verwerking uitvoeren
```

Gebruiker klikt: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Verwerking uitvoeren      Verwerking uitvoeren
```

> - Wacht een bepaalde tijd na **laatste gebeurtenis** voor verwerking
> - geschikt voor zoeken, autocomplete en real-time validatie

### Hoe throttleTime vs. debounceTime gebruiken

Gebruiker klikt: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Verwerking uitvoeren      Verwerking uitvoeren

### Oplossing 3: ontdubbeling met distinctUntilChanged

Vergelijk met de vorige waarde en sla verwerking over als dezelfde waarde opeenvolgend is.


```

```typescript
import { fromEvent, map, debounceTime, distinctUntilChanged } from 'rxjs';
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Zoekwoorden invoeren...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  debounceTime(300),
  distinctUntilChanged() // Genegeerd als de waarde hetzelfde is als de vorige keer
).subscribe(query => {
  console.log('Zoekopdracht uitvoeren:', query);
  performSearch(query);
});
```

#### Uitvoeringsvoorbeeld

```typescript
// Invoer gebruiker: "RxJS" → Backspace → "RxJS"
// distinctUntilChangedGeen: 2Eén keer zoeken
// distinctUntilChangedJa, als de waarde hetzelfde is als de vorige keer.: 1Zoekactie slechts één keer uitvoeren (dezelfde waarde, tweede zoekactie overslaan)2De tweede zoekopdracht wordt overgeslagen)
```

Gebruiker klikt: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Verwerking uitvoeren      Verwerking uitvoeren
```

> In zoek- en autocomplete-implementaties wordt het aanbevolen om de volgende drie sets te gebruiken.
> 1. `debounceTime()` - wachten op invoerstop.
> 2. `distinctUntilChanged()` - ontdubbeling
> 3. `switchMap()` - annuleer oude verzoeken

## Afhandeling van scroll-evenementen

### Probleem: overmatig afgaan van scroll-events

Scroll-events worden zeer vaak geactiveerd en kunnen prestatieproblemen veroorzaken als ze op deze manier worden afgehandeld.

### Oplossing: Verdun met throttleTime.


```

```typescript
import { fromEvent, throttleTime, map } from 'rxjs';
const scrollContainer = document.createElement('div');
scrollContainer.id = 'scroll-container';
scrollContainer.style.width = '400px';
scrollContainer.style.height = '300px';
scrollContainer.style.overflow = 'auto';
scrollContainer.style.border = '1px solid #ccc';
scrollContainer.style.margin = '10px';
scrollContainer.style.padding = '10px';

// Add content to make it scrollable
scrollContainer.innerHTML = Array.from({ length: 100 }, (_, i) =>
  `<p>Item ${i + 1}</p>`
).join('');

document.body.appendChild(scrollContainer);

fromEvent(scrollContainer, 'scroll').pipe(
  throttleTime(100), // 100msnaar1Slechts eenmaal verwerkt
  map(() => ({
    scrollTop: scrollContainer.scrollTop,
    scrollHeight: scrollContainer.scrollHeight,
    clientHeight: scrollContainer.clientHeight
  }))
).subscribe(({ scrollTop, scrollHeight, clientHeight }) => {
  // Berekening van scrollpositie
  const scrollPercentage = (scrollTop / (scrollHeight - clientHeight)) * 100;
  console.log(`Scrollpositie: ${scrollPercentage.toFixed(1)}%`);

  // Oneindig scrollen: 90%Volgende pagina laden na meer dan scrollen
  if (scrollPercentage > 90) {
    console.log('Volgende pagina laden...');
    loadMoreItems();
  }
});

function loadMoreItems(): void {
  console.log('Extra gegevensverzameling');
}
```

### Praktijkvoorbeeld: scrollrichting detecteren

```typescript
import { fromEvent, BehaviorSubject, throttleTime, map, pairwise, distinctUntilChanged } from 'rxjs';
type ScrollDirection = 'up' | 'down' | 'none';

const scrollDirection$ = new BehaviorSubject<ScrollDirection>('none');

// Create header element dynamically
const header = document.createElement('div');
header.id = 'header';
header.innerText = 'Koptekst (scrollen om te tonen/(verborgen)';
header.style.position = 'fixed';
header.style.top = '0';
header.style.left = '0';
header.style.width = '100%';
header.style.padding = '20px';
header.style.background = '#333';
header.style.color = '#fff';
header.style.transition = 'transform 0.3s';
document.body.appendChild(header);

// Add scroll content
const scrollContent = document.createElement('div');
scrollContent.style.marginTop = '80px';
scrollContent.innerHTML = Array.from({ length: 100 }, (_, i) =>
  `<p>Inhoud ${i + 1}</p>`
).join('');
document.body.appendChild(scrollContent);

fromEvent(window, 'scroll').pipe(
  throttleTime(100),
  map(() => window.scrollY),
  pairwise(), // Vorige en huidige waarden paarsgewijs ophalen
  map(([prev, curr]) => {
    if (curr > prev) return 'down';
    if (curr < prev) return 'up';
    return 'none';
  }),
  distinctUntilChanged() // Melding alleen wanneer de richting verandert
).subscribe(direction => {
  scrollDirection$.next(direction);
  console.log('Scrollrichting:', direction);

  // Toont de koptekst/Schakelen tussen verborgen en zichtbaar
  if (direction === 'down') {
    header.style.transform = 'translateY(-100%)';
  } else if (direction === 'up') {
    header.style.transform = 'translateY(0)';
  }
});
```

```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Zoekwoorden invoeren...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Na invoer stopt300msWacht
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Zoekopdracht uitvoeren:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Zoeken in uitvoering...', query);
  // ZoekenAPIOpvragen
}
```

> `pairwise()` is een handige operator waarmee je de vorige en huidige waarden in paren kunt verkrijgen. Het kan worden gebruikt om de scrollrichting te bepalen, waarden te verhogen/verlagen en verschillen te berekenen.

## Slepen en neerzetten implementeren

### Probleem: complexe combinaties van muisgebeurtenissen

Slepen en neerzetten is een complexe combinatie van mousedown → mousemove → mouseup events.

### Oplossing: combineer verschillende Observable.


```typescript
import { fromEvent, merge, map, switchMap, takeUntil, tap } from 'rxjs';
interface Position {
  x: number;
  y: number;
}

const draggableElement = document.createElement('div');
draggableElement.id = 'draggable';
draggableElement.innerText = 'Slepen.';
draggableElement.style.position = 'absolute';
draggableElement.style.left = '100px';
draggableElement.style.top = '100px';
draggableElement.style.width = '150px';
draggableElement.style.height = '150px';
draggableElement.style.padding = '20px';
draggableElement.style.background = '#4CAF50';
draggableElement.style.color = '#fff';
draggableElement.style.cursor = 'move';
draggableElement.style.userSelect = 'none';
draggableElement.style.display = 'flex';
draggableElement.style.alignItems = 'center';
draggableElement.style.justifyContent = 'center';
document.body.appendChild(draggableElement);

const mouseDown$ = fromEvent<MouseEvent>(draggableElement, 'mousedown');
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

// Geeft de positie van het element bij het begin van het slepen
let initialX = 0;
let initialY = 0;

mouseDown$.pipe(
  tap((event: MouseEvent) => {
    event.preventDefault();
    // Legt de huidige positie van het element vast
    const rect = draggableElement.getBoundingClientRect();
    initialX = rect.left;
    initialY = rect.top;

    // Verschil met de muispositie bij het begin van het slepen
    initialX = rect.left - event.clientX;
    initialY = rect.top - event.clientY;

    draggableElement.style.opacity = '0.7';
  }),
  switchMap(() =>
    // mousedownWanneer,mousemoveBegin met het controleren van de
    mouseMove$.pipe(
      map((event: MouseEvent): Position => ({
        x: event.clientX + initialX,
        y: event.clientY + initialY
      })),
      // mouseupofmouseleaveom de controle te beëindigen
      takeUntil(
        merge(
          mouseUp$,
          fromEvent(document, 'mouseleave')
        ).pipe(
          tap(() => {
            draggableElement.style.opacity = '1';
          })
        )
      )
    )
  )
).subscribe((position: Position) => {
  // Element verplaatsen
  draggableElement.style.left = `${position.x}px`;
  draggableElement.style.top = `${position.y}px`;
});
```

#### Gebeurtenisstroom

```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1In een seconde.1Slechts eenmaal verwerkt
  ).subscribe(() => {
    console.log('Uitvoering van verzendproces');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Tijdens formulierverzending...');
  // APIGesprekken, enz.
}
```


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Zoekwoorden invoeren...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Na invoer stopt300msWacht
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Zoekopdracht uitvoeren:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Zoeken in uitvoering...', query);
  // ZoekenAPIOpvragen
}
```

> - Start de bewaking van mousedown → mousemove met `switchMap`.
> - Beëindig de monitoring bij mouseup met `takeUntil`.
> - Standaard sleepgedrag uitschakelen met `preventDefault()`.
> - Visuele feedback met `classList.add/remove`.

### Ondersteuning voor aanraakapparaten.


```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1In een seconde.1Slechts eenmaal verwerkt
  ).subscribe(() => {
    console.log('Uitvoering van verzendproces');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Tijdens formulierverzending...');
  // APIGesprekken, enz.
}
```


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Zoekwoorden invoeren...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Na invoer stopt300msWacht
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Zoekopdracht uitvoeren:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Zoeken in uitvoering...', query);
  // ZoekenAPIOpvragen
}
```

> Door `merge` te gebruiken om muis- en aanraakgebeurtenissen te integreren, kun je slepen en neerzetten implementeren dat werkt op alle pc's/tablets/smartphones.

#### Vergelijking van gebeurtenissenstromen


```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1In een seconde.1Slechts eenmaal verwerkt
  ).subscribe(() => {
    console.log('Uitvoering van verzendproces');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Tijdens formulierverzending...');
  // APIGesprekken, enz.
}
```

Dit sequentiediagram laat zien dat muis- en aanraakgebeurtenissen in dezelfde pijplijn zijn geïntegreerd en zich op beide apparaten op dezelfde manier gedragen.

## Toetsenbordinvoer en autoaanvullen

### Probleem: overmatige API-aanroepen tijdens invoer

Wanneer API-aanroepen worden gedaan als reactie op toetsenbordinvoer, zoals in het zoekvak, kan het telkens aanroepen van deze aanroepen een prestatieprobleem zijn.

Als een gebruiker bijvoorbeeld `RxJS,
- `R` → API-oproep
- `Rx` → API-aanroep
- `RxJ` → API-oproep
- `RxJS` → API-oproep

Een invoer van vier letters leidt ertoe dat de API vier keer wordt aangeroepen. Dit is verspilling en belast ook de server.

### Oplossing: debounceTime + switchMap

Om autocompletion efficiënt te implementeren, combineer je de volgende drie operatoren.

1. **debounceTime(300)** - wacht 300 ms nadat de gebruiker de invoer heeft gestopt
2. **distinctUntilChanged()** - negeer of de waarde hetzelfde is als de vorige keer (voorkomt verspilde verzoeken)
3.**switchMap()** - annuleer oud verzoek als nieuwe invoer wordt ontvangen

Met deze combinatie wordt de API slechts eenmaal aangeroepen nadat de invoer is gestopt, zelfs als de gebruiker snel "RxJS" invoert.


```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1In een seconde.1Slechts eenmaal verwerkt
  ).subscribe(() => {
    console.log('Uitvoering van verzendproces');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Tijdens formulierverzending...');
  // APIGesprekken, enz.
}
```

#### Gedetailleerde beschrijving van de werking

De volgende concrete voorbeelden illustreren hoe elke stap van deze code werkt.

**Tijdlijn van een gebruiker die snel 'RxJS' typt:***.


```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1In een seconde.1Slechts eenmaal verwerkt
  ).subscribe(() => {
    console.log('Uitvoering van verzendproces');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Tijdens formulierverzending...');
  // APIGesprekken, enz.
}
```

#### Rol van elke operator

1. DebounceTime(300)**
   - Blijft wachten tijdens een reeks invoergebeurtenissen.
   - Spoelt de waarde door nadat 300 ms zijn verstreken sinds de invoer is gestopt.
   - Resultaat: er vinden geen API-oproepen plaats tijdens snel typen.

2.**distinctUntilChanged()**
   - Vergelijk met de laatste waarde en negeer als de waarde hetzelfde is.
   - Voorbeeld: Als "abc" -> (delete) -> "abc" wordt getypt, wordt de tweede "abc" niet verwerkt.
   - Resultaat: voorkomt onnodige API-aanroepen.

3. **switchMap()**
   - Als er een nieuwe zoekopdracht binnenkomt, wordt de oude aanvraag die wordt uitgevoerd geannuleerd.
   - Voorbeeld: Als een zoekopdracht voor "Rx" wordt uitgevoerd terwijl een zoekopdracht voor "RxJS" wordt uitgevoerd, wordt het verzoek voor "Rx" afgebroken.
   - Resultaat: alleen de laatste zoekresultaten worden altijd weergegeven.


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Zoekwoorden invoeren...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Na invoer stopt300msWacht
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Zoekopdracht uitvoeren:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Zoeken in uitvoering...', query);
  // ZoekenAPIOpvragen
}
```

> Als u `mergeMap` gebruikt in plaats van `switchMap`, worden de oudere verzoeken verder uitgevoerd. Hierdoor worden de resultaten van langzamere verzoeken later weergegeven, wat problemen met de UI veroorzaakt.
>
> - ❌ mergeMap**: 'Rx' (langzaam) → 'RxJS' (snel) → 'RxJS' resultaten → 'Rx' resultaten (overschreven door oude resultaten).
> - ✅ **switchMap**: 'Rx' (annuleren) → 'RxJS' (uitvoeren) → alleen 'RxJS'-resultaten worden weergegeven.

#### Uitvoeringsvoorbeeld


```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1In een seconde.1Slechts eenmaal verwerkt
  ).subscribe(() => {
    console.log('Uitvoering van verzendproces');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Tijdens formulierverzending...');
  // APIGesprekken, enz.
}
```

### Praktisch voorbeeld: sneltoets


```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1In een seconde.1Slechts eenmaal verwerkt
  ).subscribe(() => {
    console.log('Uitvoering van verzendproces');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Tijdens formulierverzending...');
  // APIGesprekken, enz.
}
```

### Meerdere toetscombinaties


```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1In een seconde.1Slechts eenmaal verwerkt
  ).subscribe(() => {
    console.log('Uitvoering van verzendproces');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Tijdens formulierverzending...');
  // APIGesprekken, enz.
}
```


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Zoekwoorden invoeren...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Na invoer stopt300msWacht
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Zoekopdracht uitvoeren:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Zoeken in uitvoering...', query);
  // ZoekenAPIOpvragen
}
```

> - Voorkom standaard gedrag met `preventDefault()`.
> Bepaal modificatietoetsen met `event.ctrlKey`, `event.shiftKey`, `event.altKey`.
> - Verwerk alleen bepaalde toetsen met `filter`.
> - Prioriteit wordt gegeven aan gebruiksvriendelijke sneltoetsen (bijv. Ctrl+S)

## Multi-touch ondersteuning

### Probleem: knijp-zoom en multi-touch gebaren

We willen pinch-zoom en multi-touch gebaren implementeren op tablets en smartphones.

### Oplossing: aanraakgebeurtenissen monitoren.


```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1In een seconde.1Slechts eenmaal verwerkt
  ).subscribe(() => {
    console.log('Uitvoering van verzendproces');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Tijdens formulierverzending...');
  // APIGesprekken, enz.
}
```

__oproep_35___

> - Bepaal aanraking met twee vingers met `touches.length === 2`.
> - Sla de eerste afstand op met `touchstart`.
> - `touchmove` om huidige afstand te berekenen en schaal bij te werken
> - Bereken het verschil met de vorige keer met `pairwise()`.
> - Beperk het bereik van de schaal om de bruikbaarheid te verbeteren

## Samengesteld gebeurtenissenpatroon

### Praktisch voorbeeld: lange druk detectie


```typescript
import { fromEvent, throttleTime } from 'rxjs';
const button = document.createElement('button');
button.id = 'submit-button';
button.innerText = 'submit';
document.body.appendChild(button);

if (button) {
  fromEvent(button, 'click').pipe(
    throttleTime(1000) // 1In een seconde.1Slechts eenmaal verwerkt
  ).subscribe(() => {
    console.log('Uitvoering van verzendproces');
    submitForm();
  });
}

function submitForm(): void {
  console.log('Tijdens formulierverzending...');
  // APIGesprekken, enz.
}
```

### Praktisch voorbeeld: detectie van dubbelklikken

Gebruiker klikt: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Verwerking uitvoeren      Verwerking uitvoeren

### Praktisch voorbeeld: hoververtraging weergeven

Gebruiker klikt: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Verwerking uitvoeren      Verwerking uitvoeren

## Gebeurtenis opruimen

### Probleem: geheugenlekken voorkomen

Het niet correct afmelden van event listeners kan geheugenlekken veroorzaken.

### Oplossing: opruimen met takeUntil.

Gebruiker klikt: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Verwerking uitvoeren      Verwerking uitvoeren


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Zoekwoorden invoeren...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Na invoer stopt300msWacht
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Zoekopdracht uitvoeren:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Zoeken in uitvoering...', query);
  // ZoekenAPIOpvragen
}
```

> - **Pas `takeUntil` toe op alle event-abonnementen**
> - **Vuur `destroy$` af bij componentvernietiging**
> - **Globale events (window, document) vereisen speciale aandacht**
> - **Vergeet `unsubscribe()` niet bij het expliciet beheren van abonnementen**

## Praktische UI component voorbeelden

### Oneindig scrollen implementatie

Gebruiker klikt: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Verwerking uitvoeren      Verwerking uitvoeren


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Zoekwoorden invoeren...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Na invoer stopt300msWacht
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Zoekopdracht uitvoeren:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Zoeken in uitvoering...', query);
  // ZoekenAPIOpvragen
}
```

> ` exhaustMap` kan worden gebruikt om nieuwe verzoeken te negeren totdat het vorige verzoek is voltooid. Dit voorkomt dubbele verzoeken als gevolg van een scrollende reeks.

## Testcode.

Voorbeeldtest voor UI-gebeurtenisafhandeling.

Gebruiker klikt: ●    ●●●        ●  ●●
                    |    |          |  |
throttleTime(1000): ●              ●
                    |              |
                   Verwerking uitvoeren      Verwerking uitvoeren

## Samenvatting.

Het beheersen van patronen voor het afhandelen van UI-gebeurtenissen kan zorgen voor een interactieve en prettige gebruikerservaring.


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Zoekwoorden invoeren...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Na invoer stopt300msWacht
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Zoekopdracht uitvoeren:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Zoeken in uitvoering...', query);
  // ZoekenAPIOpvragen
}
```

> - throttleTime**: slechts eenmaal in een bepaalde periode verwerken (scrollen, formaat wijzigen)
> - **debounceTime**: verwerkt nadat de gebeurtenis is gestopt (zoeken, autoaanvullen)
> - **distinctUntilChanged**: ontdubbeling (identieke waarden negeren)
> - **switchMap**: complexe gebeurtenisketen (slepen en neerzetten)
> - **takeUntil**: betrouwbaar opruimen (geheugenlekken voorkomen)


```typescript
import { fromEvent, debounceTime } from 'rxjs';
// Traditional approach (commented for reference)
// const searchInput = document.querySelector<HTMLInputElement>('#search');

// Self-contained: creates input dynamically
const searchInput = document.createElement('input');
searchInput.id = 'search';
searchInput.type = 'text';
searchInput.placeholder = 'Zoekwoorden invoeren...';
searchInput.style.padding = '8px';
searchInput.style.margin = '10px';
searchInput.style.width = '300px';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // Na invoer stopt300msWacht
).subscribe((event) => {
  const value = (event.target as HTMLInputElement).value;
  console.log('Zoekopdracht uitvoeren:', value);
  performSearch(value);
});

function performSearch(query: string): void {
  console.log('Zoeken in uitvoering...', query);
  // ZoekenAPIOpvragen
}
```

> - **Performance**: throttle/debounce om overprocessing te voorkomen
> - **Gebruikbaarheid**: stel geschikte vertragingstijden in (bijv. 300 ms)
> - **Toegankelijkheid**: toetsenbordbediening ondersteunen
> - **Multi-apparaat**: ondersteuning voor zowel aanraken als muis
> - **Schoonmaak**: `takeUntil` zorgt voor geheugenvrijgave

## Volgende stappen.

Als je het UI event handling patroon onder de knie hebt, kun je verder met de volgende patronen.

- form-handling](. /form-handling.md) - real-time validatie, integratie van meerdere velden.
- API-oproepen](. /api-calls.md) - integratie van UI-gebeurtenissen en API-oproepen.
- real-time gegevensverwerking](. /real-time-data.md) - WebSocket, SSE.
- caching-strategieën](. /caching-strategies.md) - event data caching

## Gerelateerde secties.

- Hoofdstuk 4: Operatoren filteren](. /operators/filtering/) - debounceTime, throttleTime details.
- Hoofdstuk 4: Transformatie operatoren](. /operators/transformation/) - meer over switchMap, exhaustMap.
- Hoofdstuk 2: Observable](. /observables/what-is-observable.md)) - basis van fromEvent

## Verwijzingsbronnen

- RxJS officieel: fromEvent](https://rxjs.dev/api/index/function/fromEvent) - meer over fromEvent()
- MDN: Aanraakgebeurtenissen](https://developer.mozilla.org/ja/docs/Web/API/Touch_events) - Hoe aanrakingsgebeurtenissen te gebruiken.
- [Learn RxJS: debounceTime](https://www.learnrxjs.io/learn-rxjs/operators/filtering/debouncetime) - Praktische voorbeelden van debounceTime.
