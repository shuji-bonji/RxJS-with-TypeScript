---
description: "timer() - Creation Function die waarden emit na opgegeven tijd. Gebruik timer(delay) voor eenmalige vertraagde uitvoering, timer(delay, period) voor periodieke uitvoering met vertraging. Verschil met interval(), TypeScript type-inferentie, en gebruik als setTimeout alternatief uitgelegd."
---

# timer() - Start Emitting Na Vertraging

`timer()` is een Creation Function die waarden emit na een opgegeven vertragingstijd. Ondersteunt zowel eenmalige als periodieke emissie.

## Overzicht

`timer()` is een flexibele Creation Function die het initiële emissie-moment controleert. Gedrag varieert met aantal argumenten: zowel eenmalige emissie als `interval()`-achtige periodieke emissie zijn mogelijk.

**Signature**:
```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

**Officiële Documentatie**: [📘 RxJS Official: timer()](https://rxjs.dev/api/index/function/timer)

## Basisgebruik

`timer()` gedrag verandert met aantal argumenten.

### Eenmalige Emissie

Met alleen eerste argument emit het 0 na opgegeven tijd en completet.

```typescript
import { timer } from 'rxjs';

// Emit 0 na 3 seconden en complete
const timer$ = timer(3000);

timer$.subscribe({
  next: value => console.log('Waarde:', value),
  complete: () => console.log('Compleet')
});

// Output na 3 seconden:
// Waarde: 0
// Compleet
```

### Periodieke Emissie

Met tweede argument voor interval, emit het periodiek na initiële vertraging.

```typescript
import { timer } from 'rxjs';

// Start na 3 seconden, emit dan elke seconde
const timer$ = timer(3000, 1000);

timer$.subscribe(value => console.log('Waarde:', value));

// Output:
// Waarde: 0  (na 3s)
// Waarde: 1  (na 4s)
// Waarde: 2  (na 5s)
// ... (oneindig)
```

## Belangrijke Kenmerken

### 1. Flexibele Vertraging Specificatie

Vertragingstijd kan als milliseconden nummer of `Date` object worden opgegeven.

```typescript
import { timer } from 'rxjs';

// Specificeer in milliseconden
timer(5000).subscribe(() => console.log('Na 5 seconden'));

// Specificeer met Date object (uitvoeren op specifieke tijd)
const targetTime = new Date(Date.now() + 10000); // Na 10 seconden
timer(targetTime).subscribe(() => console.log('Uitgevoerd op specifieke tijd'));
```

### 2. Gedrag Verandert Met/Zonder Tweede Argument

Tweede argument bepaalt of het completet.

```typescript
import { timer } from 'rxjs';

// Zonder tweede argument - Emit eenmalig en complete
timer(1000).subscribe({
  next: value => console.log('Eenmalig:', value),
  complete: () => console.log('Compleet')
});

// Met tweede argument - Emit oneindig
timer(1000, 1000).subscribe({
  next: value => console.log('Herhalend:', value),
  complete: () => console.log('Compleet (wordt niet getoond)')
});
```

> [!IMPORTANT]
> **Met Tweede Argument Completet Het Niet**
>
> `timer(1000, 1000)` met tweede argument emit oneindig zoals `interval()`. Unsubscription is verplicht.

### 5. Cold Observable

`timer()` is een Cold Observable. Elke subscription creëert een onafhankelijke timer.

```typescript
import { timer } from 'rxjs';

const timer$ = timer(1000);

console.log('Start');

// Subscription 1
timer$.subscribe(() => console.log('Observer 1'));

// Voeg subscription 2 toe na 500ms
setTimeout(() => {
  timer$.subscribe(() => console.log('Observer 2'));
}, 500);

// Output:
// Start
// Observer 1  (na 1s)
// Observer 2  (na 1.5s - onafhankelijke timer)
```

> [!NOTE]
> **Cold Observable Kenmerken**
> - Bij elke subscription start een onafhankelijke uitvoering
> - Elke subscriber ontvangt zijn eigen datastream
> - Bij elke subscription start een onafhankelijke timer. Zoals interval(), gebruik `share()` voor sharing indien nodig.
>
> Zie [Cold en Hot Observables](/nl/guide/observables/cold-and-hot-observables) voor meer details.

## Verschil tussen timer() en interval()

Het belangrijkste verschil is het initiële emissie-moment.

```typescript
import { timer, interval } from 'rxjs';
import { take } from 'rxjs';

console.log('Start');

// interval() - Start direct (eerste waarde na 1s)
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - Geen vertraging (eerste waarde direct)
timer(0, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// timer() - Start na 2s vertraging
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer(vertraging):', value);
});
```

| Creation Function | Eerste Emissie Timing | Gebruik |
|-------------------|----------------------|---------|
| `interval(1000)` | Na 1s | Direct periodieke uitvoering |
| `timer(0, 1000)` | Direct | Eerste uitvoering direct |
| `timer(2000, 1000)` | Na 2s | Periodieke uitvoering met vertraging |
| `timer(2000)` | Na 2s (eenmalig) | Vertraagde uitvoering (eenmalig) |

## Praktische Gebruikssituaties

### 1. Vertraagde Uitvoering

Voer verwerking eenmalig uit na bepaalde tijd.

```typescript
import { from, timer } from 'rxjs';
import { switchMap } from 'rxjs';

function delayedApiCall() {
  return timer(2000).pipe(
    switchMap(() => from(
      fetch('https://jsonplaceholder.typicode.com/posts/1')
        .then(res => res.json())
    ))
  );
}

delayedApiCall().subscribe(data => {
  console.log('Data opgehaald na 2 seconden:', data);
});
```

### 2. Polling Met Vertraging

Start polling niet direct maar na bepaalde tijd.

```typescript
import { from, timer } from 'rxjs';
import { switchMap, retry } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

// Start polling na 5s, daarna elke 10s
const polling$ = timer(5000, 10000).pipe(
  switchMap(() => from(
    fetch('https://jsonplaceholder.typicode.com/users/1')
      .then(res => res.json() as Promise<Status>)
  )),
  retry(3) // Retry tot 3x bij fouten
);

const subscription = polling$.subscribe(data => {
  console.log('Status update:', data);
});

// Stop indien nodig
// subscription.unsubscribe();
```

### 3. Timeout Verwerking

Timeout als verwerking niet binnen tijd compleet.

```typescript
import { timer, race, from } from 'rxjs';
import { map } from 'rxjs';

function fetchWithTimeout(url: string, timeoutMs: number) {
  const request$ = from(fetch(url).then(res => res.json()));
  const timeout$ = timer(timeoutMs).pipe(
    map(() => {
      throw new Error('Timeout');
    })
  );

  // Neem welke eerste compleet
  return race(request$, timeout$);
}

fetchWithTimeout('https://jsonplaceholder.typicode.com/posts/1', 3000).subscribe({
  next: data => console.log('Data opgehaald:', data),
  error: err => console.error('Fout:', err.message)
});
```

### 4. Automatisch Verbergen Notificaties

Verberg notificaties automatisch na bepaalde tijd.

```typescript
import { timer, Subject, map } from 'rxjs';
import { switchMap, takeUntil } from 'rxjs';

interface Notification {
  id: number;
  message: string;
}

const notifications$ = new Subject<Notification>();
const dismiss$ = new Subject<number>();

notifications$.pipe(
  switchMap(notification => {
    console.log('Toon notificatie:', notification.message);

    // Auto-verberg na 5 seconden
    return timer(5000).pipe(
      takeUntil(dismiss$), // Stop bij handmatige verberg
      map(() => notification.id)
    );
  })
).subscribe(id => {
  console.log('Verberg notificatie:', id);
});

// Toon notificatie
notifications$.next({ id: 1, message: 'Nieuw bericht ontvangen' });

// Handmatig verbergen
// dismiss$.next(1);
```

## Gebruik in Pipelines

`timer()` wordt gebruikt als startpunt voor vertraagde verwerking of periodieke uitvoering.

```typescript
import { timer } from 'rxjs';
import { map, take, scan } from 'rxjs';

// Countdown timer (10 tot 0 seconden)
timer(0, 1000).pipe(
  map(count => 10 - count),
  take(11), // 0 tot 10 (11 waarden)
  scan((acc, curr) => curr, 0)
).subscribe({
  next: time => console.log(`Resterend: ${time} seconden`),
  complete: () => console.log('Timer afgelopen')
});

// Output:
// Resterend: 10 seconden
// Resterend: 9 seconden
// ...
// Resterend: 0 seconden
// Timer afgelopen
```

## Veelgemaakte Fouten

### 1. Vergeten Unsubscriben Met Tweede Argument

```typescript
// ❌ Fout - Met tweede argument loopt oneindig
import { timer } from 'rxjs';

function startTimer() {
  timer(1000, 1000).subscribe(value => {
    console.log('Waarde:', value); // Loopt voor altijd
  });
}

startTimer();

// ✅ Correct - Bewaar subscription, unsubscribe indien nodig
import { timer, Subscription } from 'rxjs';
import { take } from 'rxjs';

let subscription: Subscription | null = null;

function startTimer() {
  subscription = timer(1000, 1000).pipe(
    take(10) // Auto-complete na 10x
  ).subscribe(value => {
    console.log('Waarde:', value);
  });
}

function stopTimer() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startTimer();
```

### 2. Verschil Met interval() Niet Begrijpen

```typescript
// ❌ Verwarring - interval() start direct (eerste waarde na 1s)
import { interval } from 'rxjs';

interval(1000).subscribe(value => {
  console.log('interval:', value); // Output na 1s
});

// ✅ timer() - Eerste waarde direct zonder vertraging
import { timer } from 'rxjs';

timer(0, 1000).subscribe(value => {
  console.log('timer:', value); // Output direct
});
```

## Prestatieoverwegingen

`timer()` is lichtgewicht, maar gebruik kan prestatie beïnvloeden.

> [!TIP]
> **Optimalisatie Tips**:
> - Gebruik geen tweede argument voor eenmalige uitvoering
> - Unsubscribe indien niet meer nodig
> - Deel met `share()` bij meerdere observers
> - Wees voorzichtig met korte intervallen (100ms of minder)

```typescript
import { timer } from 'rxjs';
import { share } from 'rxjs';

// ❌ Prestatieproblemen - Meerdere onafhankelijke timers
const timer$ = timer(0, 1000);

timer$.subscribe(value => console.log('Observer 1:', value));
timer$.subscribe(value => console.log('Observer 2:', value));
// Twee timers lopen parallel

// ✅ Optimalisatie - Deel één timer
const sharedTimer$ = timer(0, 1000).pipe(share());

sharedTimer$.subscribe(value => console.log('Observer 1:', value));
sharedTimer$.subscribe(value => console.log('Observer 2:', value));
// Eén timer wordt gedeeld
```

## Gerelateerde Creation Functions

| Function | Verschil | Gebruik |
|----------|----------|---------|
| **[interval()](/nl/guide/creation-functions/basic/interval)** | Start direct (geen vertraging) | Periodieke uitvoering zonder vertraging |
| **[of()](/nl/guide/creation-functions/basic/of)** | Emit synchroon direct | Geen async nodig |
| **defer()** | Lazy verwerking bij subscription | Dynamische waardegeneratie |

## Samenvatting

- `timer()` is Creation Function die start emitting na vertraging
- Zonder tweede argument: Eenmalige emissie (compleet)
- Met tweede argument: Periodieke emissie (compleet niet)
- Vertragingstijd specificeerbaar als milliseconden of `Date` object
- Ideaal voor vertraagde uitvoering, polling met vertraging, timeout verwerking

## Volgende Stappen

- [interval() - Emit op vast interval](/nl/guide/creation-functions/basic/interval)
- [defer() - Lazy creatie bij subscription](/nl/guide/creation-functions/conditional/defer)
- [Terug naar basis overzicht](/nl/guide/creation-functions/basic/)
