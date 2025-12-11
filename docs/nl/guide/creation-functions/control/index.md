---
description: Deze sectie beschrijft scheduled en using, de RxJS control Creation Functions. scheduled regelt de uitvoeringstiming van Observable door een scheduler te specificeren, en using beheert automatisch resources zoals WebSocket en file handles volgens de Observable lifecycle. Het kan ook worden gebruikt voor testen en prestatieoptimalisatie.
---

# Control Creation Functions

RxJS biedt Creation Functions om de uitvoeringstiming en resourcebeheer van Observable in detail te regelen. Deze sectie beschrijft twee functies, `scheduled()` en `using()`, in detail.

## Wat zijn Control Creation Functions?

Control Creation Functions zijn een set functies voor fijnere controle van het gedrag van Observable. Ze ondersteunen geavanceerde use cases zoals controle van uitvoeringstiming (scheduler) en beheer van resource lifecycle.

### Belangrijkste kenmerken

- **Controle van uitvoeringstiming**: Gebruik scheduler om te schakelen tussen synchrone en asynchrone uitvoering
- **Resourcebeheer**: Automatische release van resources volgens Observable lifecycle
- **Testgemak**: Schakelen tussen schedulers voor testgemak
- **Prestatieoptimalisatie**: Controle van uitvoeringstiming om UI-blokkering te voorkomen

## Lijst van Control Creation Functions

| Functie | Beschrijving | Belangrijkste toepassingen |
|------|------|---------|
| [scheduled()](/nl/guide/creation-functions/control/scheduled) | Genereer Observable met gespecificeerde scheduler | Controle van uitvoeringstiming, testen |
| [using()](/nl/guide/creation-functions/control/using) | Observable met resourcecontrole | Resourcebeheer voor WebSocket, file handles, etc. |

## scheduled() basis

`scheduled()` is een functie waarmee je expliciet een scheduler kunt specificeren bij het genereren van een Observable uit een bestaande gegevensbron (array, Promise, Iterable, etc.).

### Basisgebruik

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Zend array asynchroon uit
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Start subscription');
observable$.subscribe({
  next: val => console.log('Waarde:', val),
  complete: () => console.log('Voltooid')
});
console.log('Einde subscription');

// Output:
// Start subscription
// Einde subscription
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Voltooid
```

> [!NOTE]
> Met `asyncScheduler` wordt het uitzenden van waarden asynchroon. Hierdoor kan het subscription-proces worden uitgevoerd zonder de hoofdthread te blokkeren.

## using() basis

`using()` is een functie die automatisch resources aanmaakt en vrijgeeft volgens de lifecycle van Observable. Het maakt een resource aan bij het begin van een subscription en geeft deze automatisch vrij wanneer de subscription eindigt (`complete` of `unsubscribe`).

### Basisgebruik

```typescript
import { using, interval, Subscription, take } from 'rxjs';

const resource$ = using(
  // Resource factory: uitgevoerd bij start van subscription
  () => {
    console.log('Resource aangemaakt');
    return new Subscription(() => console.log('Resource vrijgegeven'));
  },
  // Observable factory: maak Observable met behulp van resource
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Waarde:', value),
  complete: () => console.log('Voltooid')
});

// Output:
// Resource aangemaakt
// Waarde: 0
// Waarde: 1
// Waarde: 2
// Voltooid
// Resource vrijgegeven
```

> [!IMPORTANT]
> `using()` geeft resources automatisch vrij aan het einde van een subscription, waardoor geheugenlekken worden voorkomen.

## Vergelijking: scheduled() vs using()

| Kenmerk | scheduled() | using() |
|------|-------------|---------|
| Hoofddoel | Controle van uitvoeringstiming | Beheer van resource lifecycle |
| Scheduler | ✅ Kan expliciet worden gespecificeerd | ❌ Kan niet worden gespecificeerd |
| Resourcebeheer | ❌ Handmatig beheer vereist | ✅ Automatisch beheer |
| Use cases | Testen, UI-optimalisatie | WebSocket, file handles |
| Complexiteit | Eenvoudig | Enigszins complex |

## Gebruiksrichtlijnen

### Wanneer scheduled() kiezen

1. **Wil je de uitvoeringstiming regelen**
   - Wil je synchrone verwerking naar asynchroon veranderen
   - Wil je UI-blokkering voorkomen

2. **Heb je tijdcontrole nodig voor testen**
   - Combineer met TestScheduler om tijd te regelen
   - Wil je asynchrone verwerking synchroon testen

3. **Converteer bestaande gegevensbronnen naar Observable**
   - Converteer Array, Promise, Iterable naar Observable
   - Wil je expliciet een scheduler specificeren

### Wanneer using() kiezen

1. **Automatische release van resources is vereist**
   - Beheer van WebSocket-verbindingen
   - Beheer van file handles
   - Automatische opruiming van timers

2. **Wil je geheugenlekken voorkomen**
   - Voorkom vergeten om resources vrij te geven
   - Betrouwbare opruiming aan het einde van subscription

3. **Complex resourcebeheer**
   - Beheer meerdere resources tegelijk
   - Beheer resource-afhankelijkheden

## Praktische gebruiksvoorbeelden

### scheduled() gebruiksvoorbeeld

```typescript
import { scheduled, asyncScheduler, queueScheduler } from 'rxjs';

// Verwerk grote hoeveelheden gegevens asynchroon (blokkeert UI niet)
const largeArray = Array.from({ length: 10000 }, (_, i) => i);
const async$ = scheduled(largeArray, asyncScheduler);

async$.subscribe(value => {
  // Voer hier zware verwerking uit
  // UI wordt niet geblokkeerd
});

// Voer synchroon uit in tests
const sync$ = scheduled(largeArray, queueScheduler);
```

### using() gebruiksvoorbeeld

```typescript
import { using, timer } from 'rxjs';

// Beheer WebSocket-verbinding automatisch
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    console.log('WebSocket-verbinding gestart');
    return {
      unsubscribe: () => {
        ws.close();
        console.log('WebSocket-verbinding beëindigd');
      }
    };
  },
  () => timer(0, 1000) // Ontvang berichten elke seconde
);
```

## Scheduler types (voor scheduled())

| Scheduler | Beschrijving | Use cases |
|---------------|------|---------|
| `queueScheduler` | Synchrone uitvoering (queue-methode) | Standaard, synchrone verwerking |
| `asyncScheduler` | Asynchrone uitvoering (setTimeout) | UI-optimalisatie, langlopende verwerking |
| `asapScheduler` | Snelste asynchrone uitvoering (Promise) | Hoge prioriteit asynchrone verwerking |
| `animationFrameScheduler` | Animatieframe | Animatie, UI-rendering |

> [!TIP]
> Voor meer informatie over schedulers, zie [Scheduler Types](/nl/guide/schedulers/types).

## Veelgestelde vragen

### V1: Wat is het verschil tussen scheduled() en from()?

**A:** `from()` gebruikt intern de standaard scheduler (synchroon). `scheduled()` maakt het mogelijk om de scheduler expliciet te specificeren, waardoor fijne controle van de uitvoeringstiming mogelijk is.

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - voer synchroon uit
const sync$ = from([1, 2, 3]);

// scheduled() - voer asynchroon uit
const async$ = scheduled([1, 2, 3], asyncScheduler);
```

### V2: Wanneer moet ik using() gebruiken?

**A:** Gebruik het wanneer je wilt voorkomen dat je vergeet om resources vrij te geven. Het is vooral nuttig in de volgende gevallen:
- Netwerkverbindingen zoals WebSocket, EventSource, etc.
- File handles, databaseverbindingen
- Processen die handmatige `clearInterval()` of `clearTimeout()` vereisen

### V3: Waarom is scheduled() gemakkelijker te testen?

**A:** TestScheduler maakt het mogelijk om virtueel de tijd te regelen. Asynchrone processen kunnen synchroon worden getest, waardoor de testtijd aanzienlijk wordt verkort.

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled, asyncScheduler } from 'rxjs';

const testScheduler = new TestScheduler((actual, expected) => {
  expect(actual).toEqual(expected);
});

testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

## Best practices

### 1. Voorkom UI-blokkering met scheduled()

```typescript
// ❌ Slecht voorbeeld: Verwerk grote hoeveelheden gegevens synchroon
from(largeArray).subscribe(processHeavyTask);

// ✅ Goed voorbeeld: Asynchrone verwerking met asyncScheduler
scheduled(largeArray, asyncScheduler).subscribe(processHeavyTask);
```

### 2. Zorg voor release van resources met using()

```typescript
// ❌ Slecht voorbeeld: Handmatig resourcebeheer
const ws = new WebSocket('wss://example.com');
const source$ = interval(1000);
source$.subscribe(() => ws.send('ping'));
// Resourcelek als unsubscribe wordt vergeten

// ✅ Goed voorbeeld: Automatisch beheer met using()
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return { unsubscribe: () => ws.close() };
  },
  () => interval(1000).pipe(tap(() => ws.send('ping')))
);
```

### 3. Gebruik de juiste scheduler voor testen

```typescript
// ✅ Goed voorbeeld: TestScheduler voor testen
const testScheduler = new TestScheduler(...);
const source$ = scheduled([1, 2, 3], testScheduler);

// ✅ Goed voorbeeld: asyncScheduler voor productie
const source$ = scheduled([1, 2, 3], asyncScheduler);
```

## Samenvatting

Control Creation Functions zijn geavanceerde functies voor het finetunen van het gedrag van Observable.

**scheduled():**
- Regelt expliciet de uitvoeringstiming (synchroon/asynchroon)
- Nuttig voor tijdcontrole bij testen
- Effectief voor het voorkomen van UI-blokkering

**using():**
- Automatisch beheer van resource lifecycle
- Voorkomt geheugenlekken
- Ideaal voor het beheren van verbindingen zoals WebSocket

Bij juist gebruik kun je robuustere en performantere RxJS-applicaties bouwen.

## Volgende stappen

Voor gedetailleerd gebruik van elke functie, raadpleeg de volgende pagina's:

- [scheduled() in detail](/nl/guide/creation-functions/control/scheduled) - Genereer Observable met scheduler
- [using() in detail](/nl/guide/creation-functions/control/using) - Observable met resourcecontrole

## Referentiebronnen

- [RxJS Officiële Documentatie - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [RxJS Officiële Documentatie - using](https://rxjs.dev/api/index/function/using)
- [RxJS Officiële Documentatie - Scheduler](https://rxjs.dev/guide/scheduler)
- [Scheduler Types](/nl/guide/schedulers/types)
