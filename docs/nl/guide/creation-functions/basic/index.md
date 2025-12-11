---
description: Basis Observable-creatie met Creation Functions. Leer hoe je of, from, fromEvent, interval en timer gebruikt om Observables te maken van enkele waarden, arrays, Promises, events en timers. TypeScript type-veiligheid en RxJS fundamentele concepten.
---

# Basis Creation Functions

De meest basale en veelgebruikte Creation Functions. Hiermee kun je eenvoudig Observables maken van data, arrays, events en op tijd gebaseerde bronnen.

## Wat zijn Basis Creation Functions

Basis Creation Functions zijn functies die een enkele Observable maken van verschillende databronnen. Dit zijn de meest fundamentele functies in RxJS en worden in bijna alle RxJS-code gebruikt.

Bekijk de onderstaande tabel om de kenmerken en het gebruik van elke Creation Function te begrijpen.

## Belangrijkste Basis Creation Functions

| Function | Beschrijving | Gebruikssituaties |
|----------|--------------|-------------------|
| **[of](/nl/guide/creation-functions/basic/of)** | Emit opgegeven waarden in volgorde | Vaste waarden testen, mocks maken |
| **[from](/nl/guide/creation-functions/basic/from)** | Converteer arrays, Promises, etc. | Bestaande data streamen |
| **[fromEvent](/nl/guide/creation-functions/basic/fromEvent)** | Converteer events naar Observable | DOM events, Node.js EventEmitter |
| **[interval](/nl/guide/creation-functions/basic/interval)** | Emit op opgegeven interval | Polling, periodieke uitvoering |
| **[timer](/nl/guide/creation-functions/basic/timer)** | Start emitting na vertraging | Vertraagde uitvoering, timeout |

## Selectiecriteria

De keuze voor een basis Creation Function wordt bepaald door het type databron.

### 1. Type Data

- **Statische waarden**: `of()` - Maak Observable direct van waarden
- **Arrays of iterables**: `from()` - Converteer bestaande collecties naar streams
- **Promises**: `from()` - Converteer async operaties naar Observables
- **Events**: `fromEvent()` - Converteer event listeners naar Observables
- **Tijdgebaseerd**: `interval()`, `timer()` - Emit waarden op basis van tijd

### 2. Timing van Emissie

- **Direct emitting**: `of()`, `from()` - Start emitting bij subscription
- **Bij event**: `fromEvent()` - Emit wanneer event plaatsvindt
- **Periodiek emitting**: `interval()` - Emit op vast interval
- **Na vertraging**: `timer()` - Start emitting na opgegeven tijd

### 3. Timing van Completion

- **Direct compleet**: `of()`, `from()` - Complete na alle waarden
- **Geen completion**: `fromEvent()`, `interval()` - Blijft actief tot unsubscribe
- **Eenmalig en compleet**: `timer(delay)` - Emit één waarde en complete

## Praktische Voorbeelden

### of() - Vaste waarden testen

```typescript
import { of } from 'rxjs';

// Testdata maken
const mockUser$ = of({ id: 1, name: 'Test User' });

mockUser$.subscribe(user => console.log(user));
// Output: { id: 1, name: 'Test User' }
```

### from() - Array streamen

```typescript
import { from } from 'rxjs';
import { map } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

numbers$.pipe(
  map(n => n * 2)
).subscribe(console.log);
// Output: 2, 4, 6, 8, 10
```

### fromEvent() - Click events

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Button clicked!'));
```

### interval() - Polling

```typescript
import { interval } from 'rxjs';
import { switchMap } from 'rxjs';

// Poll API elke 5 seconden
interval(5000).pipe(
  switchMap(() => fetchData())
).subscribe(data => console.log('Updated:', data));
```

### timer() - Vertraagde uitvoering

```typescript
import { timer } from 'rxjs';

// Uitvoeren na 3 seconden
timer(3000).subscribe(() => console.log('3 seconds passed'));
```

## Let op Memory Leaks

Bij het gebruik van basis Creation Functions is correcte unsubscription belangrijk.

> [!WARNING]
> `fromEvent()`, `interval()`, en periodieke `timer(delay, period)` completen niet automatisch. Je moet altijd `unsubscribe()` aanroepen bij component destruction, of automatisch unsubscriben met `takeUntil()`.
>
> Let op: `timer(delay)` zonder tweede argument completet na één emissie.

```typescript
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => console.log('Window resized'));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Van Cold naar Hot Conversion

Zoals aangegeven in de bovenstaande tabel, **genereren alle basis Creation Functions Cold Observables**. Bij elke subscription start een onafhankelijke uitvoering.

Echter, met de volgende multicasting operators kun je **Cold Observables omzetten naar Hot Observables**.

### Voorwaarden en Operators voor Hot Conversie

| Operator | Gedrag | Gebruikssituaties |
|----------|--------|-------------------|
| **share()** | Multicast + automatische connect/disconnect | HTTP requests delen tussen subscribers |
| **shareReplay(n)** | Cache laatste n waarden voor nieuwe subscribers | API response caching |
| **publish() + connect()** | Handmatig multicast starten | Start na verzamelen subscribers |
| **multicast(subject)** | Multicast met custom Subject | Geavanceerde controle nodig |

### Praktisch Voorbeeld

```typescript
import { interval } from 'rxjs';
import { take, share } from 'rxjs';

// ❄️ Cold - Onafhankelijke timer per subscription
const cold$ = interval(1000).pipe(take(3));

cold$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  cold$.subscribe(val => console.log('B:', val));
}, 1500);

// Output:
// A: 0 (0s later)
// A: 1 (1s later)
// B: 0 (1.5s later) ← B start onafhankelijk vanaf 0
// A: 2 (2s later)
// B: 1 (2.5s later)

// 🔥 Hot - Gedeelde timer tussen subscribers
const hot$ = interval(1000).pipe(take(3), share());

hot$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  hot$.subscribe(val => console.log('B:', val));
}, 1500);

// Output:
// A: 0 (0s later)
// A: 1 (1s later)
// A: 2, B: 2 (2s later) ← B joint halverwege, ontvangt dezelfde waarde
```

> [!TIP]
> **Wanneer Hot conversie nodig is**:
> - HTTP requests delen tussen meerdere subscribers
> - Eén WebSocket of server connectie onderhouden
> - Dure berekeningen delen tussen meerdere locaties
>
> Zie **Subject en Multicasting** hoofdstuk (Chapter 5) voor meer details.

## Relatie met Pipeable Operators

Basis Creation Functions hebben geen directe corresponderende Pipeable Operators. Ze worden altijd als Creation Functions gebruikt.

Wel worden ze vaak gecombineerd met Pipeable Operators in patronen zoals:

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, switchMap } from 'rxjs';

// Gebruikersinvoer → 300ms wachten → API aanroep
fromEvent(input, 'input').pipe(
  debounceTime(300),
  switchMap(event => fetchSuggestions(event.target.value))
).subscribe(suggestions => console.log(suggestions));
```

## Volgende Stappen

Klik op de links in de bovenstaande tabel om de gedetailleerde werking en praktische voorbeelden van elke Creation Function te leren.

Bekijk ook [Combinatie Creation Functions](/nl/guide/creation-functions/combination/), [Selectie/Partitie Creation Functions](/nl/guide/creation-functions/selection/), en [Conditionele Creation Functions](/nl/guide/creation-functions/conditional/) om een volledig overzicht van Creation Functions te krijgen.
