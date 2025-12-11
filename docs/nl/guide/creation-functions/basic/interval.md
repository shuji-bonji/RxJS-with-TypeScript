---
description: "interval() - Creation Function die op vast interval continu waarden emit (sequentiële nummers vanaf 0). Ideaal voor polling, periodieke uitvoering en animatie controle. Verschil met timer(), beperking met take(), TypeScript type-veilige implementatie, en memory leak preventie unsubscription patronen uitgelegd."
---

# interval() - Emit op Vast Interval

`interval()` is een Creation Function die waarden emit op een vast tijdsinterval.

## Overzicht

`interval()` emit continue sequentiële nummers vanaf 0 op het opgegeven milliseconden-interval. Wordt vaak gebruikt voor polling en periodieke taken.

**Signature**:
```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

**Officiële Documentatie**: [📘 RxJS Official: interval()](https://rxjs.dev/api/index/function/interval)

## Basisgebruik

`interval()` emit incrementerende nummers op het opgegeven interval.

```typescript
import { interval } from 'rxjs';

// Emit waarde elke seconde
const interval$ = interval(1000);

interval$.subscribe(value => {
  console.log('Waarde:', value);
});

// Output (elke seconde):
// Waarde: 0
// Waarde: 1
// Waarde: 2
// Waarde: 3
// ... (oneindig)
```

## Belangrijke Kenmerken

### 1. Sequentiële Nummers Vanaf 0

`interval()` emit altijd integers vanaf 0, incrementerend met 1.

```typescript
import { interval } from 'rxjs';
import { take } from 'rxjs';

interval(500).pipe(
  take(5) // Neem alleen eerste 5 waarden
).subscribe(value => console.log(value));

// Output (elke 500ms):
// 0
// 1
// 2
// 3
// 4
```

### 2. Completet Niet (Oneindige Stream)

`interval()` completet niet automatisch, **unsubscription is verplicht**.

```typescript
import { interval } from 'rxjs';

const subscription = interval(1000).subscribe(value => {
  console.log('Waarde:', value);
});

// Unsubscribe na 5 seconden
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Gestopt');
}, 5000);
```

> [!WARNING]
> **Vergeten Unsubscriben Veroorzaakt Memory Leaks**
>
> `interval()` emit oneindig waarden. Vergeten unsubscriben veroorzaakt memory leaks en prestatieproblemen. Gebruik altijd `unsubscribe()`, of auto-complete met `take()`, `takeUntil()`, `takeWhile()`.

### 4. Cold Observable

`interval()` is een Cold Observable. Elke subscription creëert een onafhankelijke timer.

```typescript
import { interval } from 'rxjs';

const interval$ = interval(1000);

// Subscription 1
interval$.subscribe(value => console.log('Observer 1:', value));

// Voeg subscription 2 toe na 2 seconden
setTimeout(() => {
  interval$.subscribe(value => console.log('Observer 2:', value));
}, 2000);

// Output:
// Observer 1: 0
// Observer 1: 1
// Observer 2: 0  ← Onafhankelijke timer start vanaf 0
// Observer 1: 2
// Observer 2: 1
```

> [!NOTE]
> **Cold Observable Kenmerken**
> - Bij elke subscription start een onafhankelijke uitvoering
> - Elke subscriber ontvangt zijn eigen datastream
> - Bij elke subscription start een onafhankelijke timer. Gebruik `share()` voor data-sharing.
>
> Zie [Cold en Hot Observables](/nl/guide/observables/cold-and-hot-observables) voor meer details.

## Verschil tussen interval() en timer()

`interval()` en `timer()` lijken op elkaar maar hebben belangrijke verschillen.

```typescript
import { interval, timer } from 'rxjs';
import { take } from 'rxjs';

// interval() - Start direct, continue emissie
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - Start na vertraging
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// Output:
// interval: 0  (na 1s)
// interval: 1  (na 2s)
// timer: 0     (na 2s)
// interval: 2  (na 3s)
// timer: 1     (na 3s)
// timer: 2     (na 4s)
```

| Creation Function | Start Timing | Gebruik |
|-------------------|--------------|---------|
| `interval(1000)` | Direct (eerste waarde na 1s) | Periodieke uitvoering |
| `timer(2000, 1000)` | Na opgegeven tijd | Periodieke uitvoering met vertraging |
| `timer(2000)` | Eenmalig na opgegeven tijd | Vertraagde uitvoering |

## Praktische Gebruikssituaties

### 1. API Polling

Roep API op vast interval aan om data te updaten.

```typescript
import { from, interval } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

function fetchStatus(): Promise<Status> {
  return fetch('https://jsonplaceholder.typicode.com/users/1')
    .then(res => res.json());
}

// Poll API elke 5 seconden
const polling$ = interval(5000).pipe(
  switchMap(() => from(fetchStatus())),
  catchError(error => {
    console.error('API Fout:', error);
    return of({ status: 'error', timestamp: Date.now() });
  })
);

const subscription = polling$.subscribe(data => {
  console.log('Status update:', data);
});

// Stop indien nodig
// subscription.unsubscribe();
```

### 2. Countdown Timer

Implementeer countdown met tijdslimiet.

```typescript
import { interval } from 'rxjs';
import { map, takeWhile } from 'rxjs';

const countdown$ = interval(1000).pipe(
  map(count => 10 - count), // Countdown vanaf 10 seconden
  takeWhile(time => time >= 0) // Auto-complete bij 0
);

countdown$.subscribe({
  next: time => console.log(`Resterende tijd: ${time} seconden`),
  complete: () => console.log('Tijd verstreken!')
});

// Output (elke seconde):
// Resterende tijd: 10 seconden
// Resterende tijd: 9 seconden
// ...
// Resterende tijd: 0 seconden
// Tijd verstreken!
```

### 3. Auto-save Functie

Sla formulier content periodiek automatisch op.

```typescript
import { fromEvent, from } from 'rxjs';
import { switchMap, debounceTime } from 'rxjs';

// Maak formulier
const form = document.createElement('form');
form.id = 'myForm';
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Voer tekst in';
form.appendChild(input);
document.body.appendChild(form);

const input$ = fromEvent(form, 'input');

// Auto-save 3 seconden na invoer stop (verkort voor demo)
input$.pipe(
  debounceTime(3000), // Na 3 seconden geen invoer
  switchMap(() => {
    const formData = new FormData(form);
    // Demo: Simuleer met Promise i.p.v. echte API
    return from(
      Promise.resolve({ success: true, data: formData.get('text') })
    );
  })
).subscribe(result => {
  console.log('Auto-saved:', result);
});
```

### 4. Realtime Klok Display

Update huidige tijd realtime.

```typescript
import { interval } from 'rxjs';
import { map } from 'rxjs';

// Maak klok display element
const clockElement = document.createElement('div');
clockElement.id = 'clock';
clockElement.style.fontSize = '24px';
clockElement.style.fontFamily = 'monospace';
clockElement.style.padding = '20px';
document.body.appendChild(clockElement);

const clock$ = interval(1000).pipe(
  map(() => new Date().toLocaleTimeString())
);

clock$.subscribe(time => {
  clockElement.textContent = time;
});

// Output: Huidige tijd update elke seconde
```

## Gebruik in Pipelines

`interval()` wordt gebruikt als startpunt van pipelines of als time control trigger.

```typescript
import { interval } from 'rxjs';
import { map, filter, scan } from 'rxjs';

// Tel alleen even seconden
interval(1000).pipe(
  filter(count => count % 2 === 0),
  scan((sum, count) => sum + count, 0),
  map(sum => `Som van even getallen: ${sum}`)
).subscribe(console.log);

// Output (elke seconde):
// Som van even getallen: 0
// Som van even getallen: 2  (0 + 2)
// Som van even getallen: 6  (0 + 2 + 4)
// Som van even getallen: 12 (0 + 2 + 4 + 6)
```

## Veelgemaakte Fouten

### 1. Vergeten Unsubscriben

```typescript
// ❌ Fout - Geen unsubscribe, loopt oneindig
import { interval } from 'rxjs';

function startPolling() {
  interval(1000).subscribe(value => {
    console.log('Waarde:', value); // Loopt voor altijd
  });
}

startPolling();

// ✅ Correct - Bewaar subscription, unsubscribe indien nodig
import { interval, Subscription } from 'rxjs';

let subscription: Subscription | null = null;

function startPolling() {
  subscription = interval(1000).subscribe(value => {
    console.log('Waarde:', value);
  });
}

function stopPolling() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startPolling();
// Roep stopPolling() aan indien nodig
```

### 2. Meerdere Subscriptions Creëren Onafhankelijke Timers

```typescript
// ❌ Onbedoeld - Twee onafhankelijke timers worden gemaakt
import { interval } from 'rxjs';

const interval$ = interval(1000);

interval$.subscribe(value => console.log('Observer 1:', value));
interval$.subscribe(value => console.log('Observer 2:', value));
// Twee timers lopen parallel

// ✅ Correct - Deel één timer
import { interval } from 'rxjs';
import { share } from 'rxjs';

const interval$ = interval(1000).pipe(share());

interval$.subscribe(value => console.log('Observer 1:', value));
interval$.subscribe(value => console.log('Observer 2:', value));
// Eén timer wordt gedeeld
```

## Prestatieoverwegingen

`interval()` is lichtgewicht, maar bij korte intervallen is prestatie-aandacht vereist.

> [!TIP]
> **Optimalisatie Tips**:
> - Vermijd onnodige verwerking (`filter()` om te filteren)
> - Wees voorzichtig met korte intervallen (100ms of minder)
> - Zorg voor betrouwbare unsubscription
> - Deel met `share()` bij meerdere observers

```typescript
import { interval } from 'rxjs';
import { filter, share } from 'rxjs';

// ❌ Prestatieproblemen - Zware verwerking elke 100ms
interval(100).subscribe(() => {
  // Zware berekening
  heavyCalculation();
});

// ✅ Optimalisatie - Verwerk alleen indien nodig
interval(100).pipe(
  filter(count => count % 10 === 0), // Eens per seconde (1x per 10)
  share() // Deel tussen meerdere observers
).subscribe(() => {
  heavyCalculation();
});
```

## Gerelateerde Creation Functions

| Function | Verschil | Gebruik |
|----------|----------|---------|
| **[timer()](/nl/guide/creation-functions/basic/timer)** | Start na vertraging, of eenmalige emissie | Vertraagde of eenmalige uitvoering |
| **[fromEvent()](/nl/guide/creation-functions/basic/fromEvent)** | Event-driven | Verwerking bij gebruikersacties |
| **range()** | Emit nummerbereik direct | Geen time control nodig |

## Samenvatting

- `interval()` emit continu waarden op vast interval
- Emit sequentiële integers vanaf 0
- Completet niet automatisch, unsubscription verplicht
- Werkt als Cold Observable (onafhankelijke timer per subscription)
- Ideaal voor polling, periodieke uitvoering, countdowns

## Volgende Stappen

- [timer() - Start emitting na vertraging](/nl/guide/creation-functions/basic/timer)
- [fromEvent() - Converteer events naar Observable](/nl/guide/creation-functions/basic/fromEvent)
- [Terug naar basis overzicht](/nl/guide/creation-functions/basic/)
