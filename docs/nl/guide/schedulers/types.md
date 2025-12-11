---
description: Gedetailleerde uitleg over de belangrijkste schedulers in RxJS zoals asyncScheduler en queueScheduler, hun kenmerken, implementatie en toepassingen. Begrijp de verschillen tussen macrotasks, microtasks en synchrone verwerking, en leer de uitvoeringstiming en eigenschappen van elke scheduler. Optimaliseer prestaties en gedrag van je applicatie door ze correct toe te passen.
---

# Soorten schedulers en hun gebruik

RxJS biedt meerdere schedulers voor verschillende toepassingen. Elke scheduler heeft zijn eigen specifieke uitvoeringstiming en kenmerken, en door ze correct toe te passen kun je de prestaties en het gedrag van je applicatie optimaliseren.

## Classificatie van schedulers

RxJS-schedulers zijn grofweg in drie categorieën te verdelen:

1. **Macrotasks**: Uitvoering in de volgende task queue van de event loop
2. **Microtasks**: Uitvoering direct na voltooiing van de huidige taak, vóór het begin van de volgende taak
3. **Synchrone verwerking**: Onmiddellijke uitvoering

Zie ook [Basiskennis van taken en schedulers](./task-and-scheduler-basics.md) voor meer details.

## Belangrijkste schedulers

### asyncScheduler

#### Kenmerken
- **Interne implementatie**: Gebruikt setTimeout
- **Uitvoeringstiming**: Macrotask
- **Toepassing**: Algemene asynchrone verwerking, verwerking met tijdsverloop

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Asynchrone verwerking')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Einde');

// Output:
// 1: Start
// 2: Einde
// 3: Asynchrone verwerking
```

#### Gebruikscases

```ts
import { asyncScheduler, map, observeOn, of } from "rxjs";

function heavyComputation(value: number): number {
  // Simuleer zware berekening
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result = Math.sin(result);
  }
  return result;
}

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(value => heavyComputation(value))
  )
  .subscribe(result => {
    console.log(`Berekeningsresultaat: ${result}`);
  });
```

### queueScheduler

#### Kenmerken
- **Interne implementatie**: Microtask queue
- **Uitvoeringstiming**: Binnen de huidige taak (lijkt synchroon)
- **Toepassing**: Taak-queuing, optimalisatie van recursieve verwerking

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Wachtrij-verwerking')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: Einde');

// Output:
// 1: Start
// 2: Wachtrij-verwerking
// 3: Einde
```

#### Gebruikscases

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs';

// Optimalisatie van recursieve verwerking
function fibonacci(n: number): Observable<number> {
  return of([0, 1]).pipe(
    observeOn(queueScheduler),
    expand(([a, b]) => of([b, a + b])),
    map(([a]) => a),
    take(n)
  );
}

fibonacci(10).subscribe(value => console.log(value));
```

### asapScheduler

#### Kenmerken
- **Interne implementatie**: Promise.resolve().then() of setImmediate
- **Uitvoeringstiming**: Microtask
- **Toepassing**: Wanneer je zo snel mogelijk asynchroon wilt uitvoeren

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('ASAP verwerking')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Einde');

// Output:
// 1: Start
// 2: Einde
// 3: ASAP verwerking
```

#### Gebruikscases

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Optimalisatie van muis bewegingsevents
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // UI update verwerking
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Kenmerken
- **Interne implementatie**: requestAnimationFrame
- **Uitvoeringstiming**: Vóór de volgende schermtekening
- **Toepassing**: Animatie, 60fps render-verwerking

#### Voorbeeld van eenvoudige rotatie-animatie

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// Maak HTML-element
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Animatie-instelling
let rotation = 0;

// 2 seconden animatie op 60fps
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2 seconden = 120 frames
    map(() => {
      rotation += 3;  // Roteer 3 graden per frame
      return rotation;
    })
  )
  .subscribe(angle => {
    // Roteer DOM-element daadwerkelijk
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Waarom animationFrameScheduler nodig is

`animationFrameScheduler` voert verwerking uit gesynchroniseerd met de render-cyclus van de browser, wat de volgende voordelen biedt:

1. **Vloeiende animatie**: Omdat verwerking wordt uitgevoerd gesynchroniseerd met de render-timing van de browser (meestal 60fps), kan vloeiende animatie zonder hapering worden gerealiseerd.
2. **Efficiënt gebruik van resources**: Wanneer de browser een tabblad inactief maakt, wordt de uitvoering van requestAnimationFrame automatisch gepauzeerd, waardoor onnodig CPU-gebruik wordt voorkomen.
3. **Voorkomen van schermflikkering**: Omdat berekeningen zeker worden voltooid vóór de schermtekening, worden schermflikkering en onvolledige frame-weergave voorkomen.

Hieronder staat een vergelijking tussen `setInterval` en `animationFrameScheduler`:

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ Inefficiënte animatie met setInterval
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // Ongeveer 60fps

// Problemen:
// - Niet gesynchroniseerd met browser render-timing
// - Blijft draaien in achtergrond tabs
// - Kan geen nauwkeurige 60fps garanderen

// ✅ Efficiënte animatie met animationFrameScheduler
interval(0, animationFrameScheduler)
  .pipe(
    map(() => {
      position += 1;
      return position;
    })
  )
  .subscribe(pos => {
    element.style.transform = `translateX(${pos}px)`;
  });

// Voordelen:
// - Gesynchroniseerd met browser render-timing
// - Automatisch gepauzeerd in achtergrond tabs
// - Stabiele 60fps gerealiseerd
```


#### Voorbeeld van muis-volganimatie

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Maak volgende cirkel
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Laat muisevents door
document.body.appendChild(circle);

// Huidige positie en doelpositie
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Monitor muisbewegingsevents
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove')
  .pipe(
    map(event => ({
      x: event.clientX,
      y: event.clientY
    }))
  );

// Animatie loop
interval(0, animationFrameScheduler)
  .pipe(
    withLatestFrom(mouseMove$),
    map(([_, mousePos]) => mousePos)
  )
  .subscribe(({ x, y }) => {
    // Stel muispositie in als doel
    targetX = x;
    targetY = y;

    // Beweeg geleidelijk van huidige positie naar doelpositie (easing)
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;

    // Update DOM-element
    circle.style.left = `${currentX - 15}px`;  // Pas aan naar middenpositie
    circle.style.top = `${currentY - 15}px`;
  });
```

## Gids voor scheduler-gebruik

### Vergelijking op basis van uitvoeringstiming

```ts
import { of, asyncScheduler, queueScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

// Synchrone verwerking
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler (microtask)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler (microtask)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler (macrotask)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(v => console.log(`5: ${v}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Einde');

// Uitvoervolgorde:
// 1: Start
// 2: sync
// 7: Einde
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```

### Selectiecriteria per toepassing

| Scheduler | Kenmerken | Geschikt voor |
|--------------|------|----------|
| asyncScheduler | Gebruikt setTimeout, volledig asynchroon | Tijdrovende verwerking, vertraagde uitvoering |
| queueScheduler | Synchroon maar optimaliseert recursie | Recursieve verwerking, taak-queue beheer |
| asapScheduler | Zo snel mogelijke asynchrone uitvoering | Event handling, snelle respons nodig |
| animationFrameScheduler | Gesynchroniseerd met schermtekening | Animatie, UI-updates, game development |

## Praktische gebruiksvoorbeelden

### Verwerking van grote datasets

```ts
import { from, queueScheduler } from 'rxjs';
import { mergeMap, observeOn, tap } from 'rxjs';

interface ApiRequest {
  endpoint: string;
  id: number;
}

const requests: ApiRequest[] = [
  { endpoint: '/users', id: 1 },
  { endpoint: '/posts', id: 1 },
  { endpoint: '/comments', id: 1 },
];

// Verzoeken in wachtrij plaatsen en op volgorde verwerken
from(requests)
  .pipe(
    observeOn(queueScheduler),
    tap((req) => console.log(`Toegevoegd aan wachtrij: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // Simulatie van daadwerkelijk API-verzoek
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`Resultaat van ${req.endpoint}/${req.id}`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`Voltooid: ${result}`));
```

### WebSocket berichtverwerking

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// Opmerking: Dit is pseudo-code om het concept te tonen
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // Behandel als string
});

socket$
  .pipe(
    // Berichtverwerking die snelle respons vereist
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('Bericht ontvangen:', msg);
}
```

### Controle van error retry

Door schedulers te gebruiken met de `retry` operator kun je de retry-timing nauwkeurig controleren.

#### Basis retry-controle

De `delay` optie van de `retry` operator gebruikt intern `asyncScheduler` om de retry-interval te controleren.

```ts
import { throwError, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

// Simulatie van API-aanroep
function fetchData(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.7) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Network error'));
    })
  );
}

fetchData(1)
  .pipe(
    retry({
      count: 3,
      delay: 1000  // Wacht 1 seconde met asyncScheduler voor retry
    })
  )
  .subscribe({
    next: result => console.log('✅ Succes:', result),
    error: error => console.log('❌ Definitieve error:', error.message)
  });
```

#### Scheduler-gebruik met exponentiële backoff

Voor geavanceerdere controle kun je `retryWhen` combineren met `asyncScheduler` om exponentiële backoff te implementeren.

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, mergeMap, tap } from 'rxjs';

function fetchDataWithBackoff(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.9) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Temporary error'));
    })
  );
}

fetchDataWithBackoff(1)
  .pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;

          // Controleer maximum aantal retries
          if (retryCount > 3) {
            console.log('❌ Maximum aantal retries bereikt');
            throw error;
          }

          // Exponentiële backoff: 1 sec, 2 sec, 4 sec...
          const delayTime = Math.pow(2, index) * 1000;
          console.log(`🔄 Retry ${retryCount} (na ${delayTime}ms)`);

          // timer gebruikt intern asyncScheduler
          return timer(delayTime);
        })
      )
    )
  )
  .subscribe({
    next: result => console.log('✅ Succes:', result),
    error: error => console.log('❌ Definitieve error:', error.message)
  });

// Voorbeeldoutput:
// 🔄 Retry 1 (na 1000ms)
// 🔄 Retry 2 (na 2000ms)
// 🔄 Retry 3 (na 4000ms)
// ❌ Maximum aantal retries bereikt
// ❌ Definitieve error: Temporary error
```

#### Wanneer asyncScheduler expliciet te specificeren

Door een specifieke scheduler expliciet te specificeren, wordt flexibelere controle mogelijk, zoals het vervangen door `TestScheduler` bij testen.

```ts
import { throwError, asyncScheduler, of } from 'rxjs';
import { retryWhen, mergeMap, delay } from 'rxjs';

function fetchDataWithScheduler(id: number, scheduler = asyncScheduler) {
  return of(id).pipe(
    mergeMap(() => throwError(() => new Error('Error'))),
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          if (index >= 2) throw error;

          // Specificeer scheduler expliciet
          return of(null).pipe(
            delay(1000, scheduler)
          );
        })
      )
    )
  );
}

// Productieomgeving: gebruik asyncScheduler
fetchDataWithScheduler(1).subscribe({
  error: err => console.log('Error:', err.message)
});

// Testomgeving: vervangbaar door TestScheduler
```

> [!TIP]
> Voor gedetailleerde implementatiepatronen en debugmethoden van retry-verwerking, zie de pagina [retry en catchError](/nl/guide/error-handling/retry-catch).
> - Gedetailleerd gebruik van retry operator
> - Combinatiepatronen met catchError
> - Debug-technieken voor retry (tracking van pogingen, loggen, etc.)

## Impact op prestaties

### Overhead van schedulers

```ts
import { range, asyncScheduler, pipe } from 'rxjs';
import { bufferCount, map, observeOn, tap } from 'rxjs';

// ❌ Overmatig scheduler-gebruik
range(1, 1000)
  .pipe(
    observeOn(asyncScheduler),  // 1000 setTimeout aanroepen
    map(x => x * 2),
    // tap(console.log)
  )
  .subscribe();

// ✅ Geoptimaliseerd met batch-verwerking
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeout aanroepen
    map(batch => batch.map(x => x * 2)),
    // tap(console.log)
  )
  .subscribe();
```

## Samenvatting

De keuze van scheduler heeft een grote impact op de prestaties en responsiviteit van je applicatie. Door de kenmerken van elke scheduler te begrijpen en ze correct toe te passen voor de juiste situatie, kun je efficiënte en vloeiende werking realiseren. Als algemene richtlijn wordt aanbevolen:

- `asyncScheduler` voor algemene asynchrone verwerking
- `queueScheduler` voor recursieve verwerking of synchrone queuing
- `asapScheduler` wanneer snelle respons nodig is
- `animationFrameScheduler` voor animaties

te gebruiken.
