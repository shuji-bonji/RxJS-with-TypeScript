---
description: "Leer meer over de kenmerken, implementatie en het gebruik van de belangrijkste schedulers in RxJS, zoals asyncScheduler en queueScheduler. Begrijp de verschillen tussen macrotaken, microtaken en synchrone verwerking, en leer de uitvoeringstiming en kenmerken van elke scheduler. Je kunt de prestaties en het gedrag van je applicatie optimaliseren door ze op de juiste manier te gebruiken."
---

# Soorten planners en hoe ze te gebruiken

RxJS biedt verschillende schedulers voor verschillende toepassingen. Elke planner heeft zijn eigen specifieke uitvoeringstijd en kenmerken en kan op de juiste manier worden gebruikt om de prestaties en het gedrag van de toepassing te optimaliseren.

## Classificatie van planners

RxJS schedulers vallen uiteen in drie hoofdcategorieën.

1. **Macro-taken**: uitgevoerd in de volgende taakwachtrij in de gebeurtenissenlus
2. **Microtaak**: onmiddellijk uitgevoerd nadat de huidige taak is voltooid en voordat de volgende taak begint
3. **Synchrone verwerking**: onmiddellijke uitvoering

Voor meer informatie, zie [Task and Scheduler Basics](./task-and-scheduler-basics.md).

## Belangrijkste schedulers.

### Scheduler

#### Kenmerken.
- **Interne implementatie**: gebruikt setTimeout.
- **Uitvoeringstijdstip**: macrotaken
- **Gebruik**: algemene asynchrone verwerking, time-lapse verwerking

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Asynchrone verwerking')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Einde');

// Uitvoer:
// 1: Start
// 2: Einde
// 3: Asynchrone verwerking
```

#### Gebruikscasus

```ts
import { asyncScheduler, map, observeOn, of } from "rxjs";

function heavyComputation(value: number): number {
  // Zware berekening simuleren
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
    console.log(`Berekeningsresultaten: ${result}`);
  });
```

### Scheduler

#### Kenmerken.
- **Interne implementatie**: microtaakwachtrij
- **Uitvoeringstijdstip**: binnen de huidige taak (lijkt synchroon)
- **Toepassingen**: taakwachtrijen, optimalisatie van recursie

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Verwerking in wachtrij')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: Einde');

// Uitvoer:
// 1: Start
// 2: Verwerking in wachtrij
// 3: Einde
```

#### Gebruikscasus

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs';

// Recursieve verwerkingsoptimalisatie
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

### Scheduler

#### Kenmerken.
- **Interne implementatie**: Promise.resolve().then() of setImmediate
- **Uitvoeringstijdstip**: microtaken
- **Gebruik**: Voor asynchrone uitvoering zo snel mogelijk

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('ASAPVerwerking')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Einde');

// Uitvoer:
// 1: Start
// 2: Einde
// 3: ASAPVerwerking
```

#### Gebruikscasus

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Optimalisatie van muisbewegingen
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // UIUpdateverwerking van
    updateCursor(position);
  });
```

### Scheduler voor animatie

#### Kenmerken.
- **Interne implementatie**: requestAnimationFrame
- **Uitvoeringstijdstip**: vóór de volgende schermrendering
- **Gebruik**: Animatie, tekenproces voor 60fps.

#### Voorbeeld van een eenvoudige rotatie-animatie

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// HTMLAanmaken van elementen
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Animaties instellen
let rotation = 0;

// 60fpsIn de2Seconden animatie
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2Seconden = 120Frame
    map(() => {
      rotation += 3;  // 1Elk frame3Graad rotatie
      return rotation;
    })
  )
  .subscribe(angle => {
    // DOMWerkelijke rotatie van het element
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Waarom hebben we animationFrameScheduler nodig?

De `animationFrameScheduler` voert zijn bewerkingen synchroon uit met de tekencyclus van de browser, wat de volgende voordelen heeft.

1.**Soepele animaties**: omdat de verwerking gesynchroniseerd is met de rendertijd van de browser (typisch 60 fps), kunnen soepele animaties zonder haperingen bereikt worden.
2. Efficiënt gebruik van bronnen**: wanneer de browser het tabblad deactiveert, wordt de uitvoering van het requestAnimationFrame automatisch gepauzeerd, waardoor onnodig CPU-gebruik wordt voorkomen.
3. **Het voorkomen van schermflikkering**: schermflikkering en onvolledige frames worden voorkomen omdat de berekening betrouwbaar wordt voltooid voordat het scherm wordt getekend.

Hier is een vergelijking tussen `setInterval` en `animationFrameScheduler`.

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ setIntervalInefficiënte animatie met
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // ongeveer60fps

// Problemen:
// - Niet gesynchroniseerd met rendertijden van browser
// - Blijft zelfs in achtergrondtabbladen draaien
// - Nauwkeurig60fpskan niet worden gegarandeerd

// ✅ animationFrameSchedulerEfficiënt animatie gebruiken
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

// Voordelen
// - Gesynchroniseerd met tekentijden browser
// - Automatisch gepauzeerd in achtergrondtabbladen
// - Stabiel60fpsMaakt  mogelijk
```

#### Voorbeeld van muisvolgende animatie

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Creatie van een te volgen cirkel
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Transparant voor muisgebeurtenissen
document.body.appendChild(circle);

// Huidige en doelpositie
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Gebeurtenissen van muisbewegingen volgen
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove')
  .pipe(
    map(event => ({
      x: event.clientX,
      y: event.clientY
    }))
  );

// Animatie lus
interval(0, animationFrameScheduler)
  .pipe(
    withLatestFrom(mouseMove$),
    map(([_, mousePos]) => mousePos)
  )
  .subscribe(({ x, y }) => {
    // Muispositie instellen als doel
    targetX = x;
    targetY = y;
    
    // Geleidelijke beweging van huidige positie naar doelpositie々Geleidelijke beweging (easing) van huidige positie naar doelpositie
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;
    
    // DOMElement bijwerken
    circle.style.left = `${currentX - 15}px`;  // Aanpassing aan middenpositie
    circle.style.top = `${currentY - 15}px`;
  });
```

## Gids voor het gebruik van verschillende planners

### Vergelijking op uitvoeringstijdstip

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Asynchrone verwerking')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Einde');

// Uitvoer:
// 1: Start
// 2: Einde
// 3: Asynchrone verwerking
```

### Selectiecriteria per toepassing


## Praktische gebruikssituaties

### Verwerking van grote hoeveelheden gegevens


### WebSocket berichtverwerking

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// 注: これは概念を示す疑似コードです
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // 文字列として扱う
});

socket$
  .pipe(
    // 高速な応答が必要なメッセージ処理
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('メッセージ受信:', msg);
}
```

### Foutherhaling controle

Door gebruik te maken van de scheduler met de `retry` operator kan de timing van retry's nauwkeurig worden geregeld.

#### Basisherhalingcontrole

De `delay` optie van de `retry` operator gebruikt de `async Scheduler` intern om het retry interval te regelen.


```

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Asynchrone verwerking')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Einde');

// Uitvoer:
// 1: Start
// 2: Einde
// 3: Asynchrone verwerking
```

#### Schedulergebruik met exponentiële back-off

Als meer geavanceerde controle kan exponentiële backoff geïmplementeerd worden door `retryWhen` en `asyncScheduler` te combineren.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

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
    // RxJS 7.3+ 推奨: retry({ count, delay }) 形式
    retry({
      count: 3, // Max.3回まで再試行
      delay: (error, retryCount) => {
        // 指数バックオフ: 1秒, 2秒, 4秒...
        const delayTime = Math.pow(2, retryCount - 1) * 1000;
        console.log(`🔄 リトライ ${retryCount}回目 (${delayTime}ms後)`);

        // timer は内部的に asyncScheduler を使用
        return timer(delayTime);
      }
    })
  )
  .subscribe({
    next: result => console.log('✅ 成功:', result),
    error: error => {
      console.log('❌ Max.リトライ数に到達');
      console.log('❌ 最終Fout:', error.message);
    }
  });

// UitvoerBv.:
// 🔄 リトライ 1回目 (1000ms後)
// 🔄 リトライ 2回目 (2000ms後)
// 🔄 リトライ 3回目 (4000ms後)
// ❌ Max.リトライ数に到達
// ❌ 最終Fout: Temporary error
```

#### Wanneer expliciet een Scheduler wordt opgegeven

Het expliciet specificeren van een specifieke scheduler maakt flexibelere controle mogelijk, zoals het vervangen door `TestScheduler` tijdens het testen.


```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Asynchrone verwerking')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Einde');

// Uitvoer:
// 1: Start
// 2: Einde
// 3: Asynchrone verwerking
```

UITROEPEN_18___

> - Gedetailleerde implementatiepatronen en debugmethoden voor het afhandelen van retry's worden beschreven op de pagina [retry and catchError](/nl/guide/error-handling/retry-catch).
> Gedetailleerd gebruik van de retry operator.
> Combinatiepatronen met catchError.
> Debuggingtechnieken voor retries (bijv. bijhouden van het aantal pogingen, loggen, enz.)

## Prestatie-impact.

### Scheduler overhead


```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

of('Asynchrone verwerking')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Einde');

// Uitvoer:
// 1: Start
// 2: Einde
// 3: Asynchrone verwerking
```

## Samenvatting.

De keuze van een scheduler heeft een significante invloed op de prestaties en reactiesnelheid van een applicatie. Door de karakteristieken van elke scheduler te begrijpen en ze in de juiste situaties te gebruiken, wordt een efficiënte en soepele werking gegarandeerd. Als algemene richtlijn,

- Scheduler` voor algemene asynchrone verwerking.
- `queueScheduler` voor recursieve verwerking en synchrone wachtrijen
- `asapScheduler` voor snelle reactietijden
- `animationFrameScheduler` voor animatie

voor animatie.