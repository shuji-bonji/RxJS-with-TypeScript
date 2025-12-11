---
description: "Introductie van 6 veelvoorkomende RxJS-debugscenario's. Praktische codevoorbeelden voor het identificeren van oorzaken en oplossingen voor problemen zoals waarden die niet stromen, onverwachte waarden, subscriptions die niet voltooien, geheugenlekken, gemiste fouten, en retry-tracking in de ontwikkelomgeving."
---

# Veelvoorkomende debugscenario's

Uitleg over typische problemen die je tegenkomt bij RxJS-ontwikkeling en hun oplossingen, met concrete codevoorbeelden.

## Scenario 1: Waarden stromen niet

- **Symptoom**: Ondanks het abonneren met `subscribe`, worden er geen waarden geproduceerd

### Oorzaak 1: Vergeten te subscriben op Cold Observable

```ts
import { interval } from 'rxjs';
import { map } from 'rxjs';

// ❌ Wordt niet uitgevoerd zonder subscription
const numbers$ = interval(1000).pipe(
  map(x => {
    console.log('Deze regel wordt niet uitgevoerd');
    return x * 2;
  })
);

// ✅ Wordt uitgevoerd door te subscriben
numbers$.subscribe(value => console.log('Waarde:', value));
```

### Oorzaak 2: Voltooide Subject

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.complete(); // Voltooid

// ❌ Na voltooiing worden geen waarden meer ontvangen
subject.subscribe(value => console.log('Deze regel wordt niet uitgevoerd'));

// ✅ Subscribe voor voltooiing
const subject2 = new Subject<number>();
subject2.subscribe(value => console.log('Waarde:', value));
subject2.next(1); // Waarde: 1
subject2.complete();
```

### Oorzaak 3: Filteren met verkeerde voorwaarden

```ts
import { of } from 'rxjs';
import { filter, tap } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('Voor filter:', value)),
    filter(x => x > 10), // Alle waarden worden uitgefilterd
    tap(value => console.log('Na filter:', value)) // Deze regel wordt niet uitgevoerd
  )
  .subscribe({
    next: value => console.log('Eindwaarde:', value),
    complete: () => console.log('Voltooid (geen waarden)')
  });

// Output:
// Voor filter: 1
// Voor filter: 2
// Voor filter: 3
// Voor filter: 4
// Voor filter: 5
// Voltooid (geen waarden)
```

### Debugtechniek
```ts
import { of, EMPTY } from 'rxjs';
import { filter, tap, defaultIfEmpty } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('🔵 Input:', value)),
    filter(x => x > 10),
    tap(value => console.log('🟢 Gepasseerd filter:', value)),
    defaultIfEmpty('Geen waarden') // Standaardwaarde als er geen waarden zijn
  )
  .subscribe(value => console.log('✅ Output:', value));

// Output:
// 🔵 Input: 1
// 🔵 Input: 2
// 🔵 Input: 3
// 🔵 Input: 4
// 🔵 Input: 5
// ✅ Output: Geen waarden
```

## Scenario 2: Onverwachte waarden worden geproduceerd

- **Symptoom**: Er worden andere waarden geproduceerd dan verwacht

### Oorzaak 1: Verkeerde operatorvolgorde

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// ❌ Onverwacht resultaat
of(1, 2, 3, 4, 5)
  .pipe(
    map(x => x * 2),     // 2, 4, 6, 8, 10
    filter(x => x < 5)   // Alleen 2, 4 passeren
  )
  .subscribe(value => console.log('Resultaat:', value));
// Output: 2, 4

// ✅ Correcte volgorde
of(1, 2, 3, 4, 5)
  .pipe(
    filter(x => x < 5),  // Alleen 1, 2, 3, 4 passeren
    map(x => x * 2)      // 2, 4, 6, 8
  )
  .subscribe(value => console.log('Resultaat:', value));
// Output: 2, 4, 6, 8
```

### Oorzaak 2: Onbedoelde wijzigingen door gedeelde referenties

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const user: User = { id: 1, name: 'Alice' };

of(user)
  .pipe(
    // ❌ Direct wijzigen van het originele object
    map(u => {
      u.name = 'Bob'; // Origineel object wordt gewijzigd
      return u;
    })
  )
  .subscribe(value => console.log('Na wijziging:', value));

console.log('Origineel object:', user); // { id: 1, name: 'Bob' }

// ✅ Nieuw object creëren
of(user)
  .pipe(
    map(u => ({ ...u, name: 'Charlie' })) // Nieuw object met spread-syntax
  )
  .subscribe(value => console.log('Na wijziging:', value));

console.log('Origineel object:', user); // { id: 1, name: 'Alice' } (niet gewijzigd)
```

### Oorzaak 3: Timing van asynchrone verwerking

```ts
import { of, delay } from 'rxjs';
import { mergeMap, tap } from 'rxjs';

// ❌ Wacht niet op voltooiing van asynchrone verwerking
of(1, 2, 3)
  .pipe(
    tap(value => console.log('Start:', value)),
    mergeMap(value =>
      of(value * 2).pipe(
        delay(100 - value * 10) // Grotere waarden voltooien sneller
      )
    )
  )
  .subscribe(value => console.log('Voltooid:', value));

// Output:
// Start: 1
// Start: 2
// Start: 3
// Voltooid: 3  ← Kortste vertraging
// Voltooid: 2
// Voltooid: 1  ← Langste vertraging

// ✅ Volgorde garanderen
import { concatMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(value => console.log('Start:', value)),
    concatMap(value =>  // mergeMap → concatMap
      of(value * 2).pipe(delay(100 - value * 10))
    )
  )
  .subscribe(value => console.log('Voltooid:', value));

// Output:
// Start: 1
// Voltooid: 1
// Start: 2
// Voltooid: 2
// Start: 3
// Voltooid: 3
```

## Scenario 3: Subscription wordt niet voltooid (oneindige stream)

- **Symptoom**: `complete` wordt niet aangeroepen en de stream eindigt niet

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

// ❌ interval blijft oneindig waarden produceren
interval(1000)
  .pipe(
    tap(value => console.log('Waarde:', value))
  )
  .subscribe({
    complete: () => console.log('Deze regel wordt niet uitgevoerd')
  });

// ✅ Expliciet voltooien met take
import { take } from 'rxjs';

interval(1000)
  .pipe(
    take(5), // Voltooid na 5 waarden
    tap(value => console.log('Waarde:', value))
  )
  .subscribe({
    complete: () => console.log('Voltooid')
  });
```

### Debugtechniek
```ts
import { interval, timer } from 'rxjs';
import { tap, takeUntil } from 'rxjs';

// Timeout instellen voor debugging
const stop$ = timer(5000); // Voltooid na 5 seconden

interval(1000)
  .pipe(
    takeUntil(stop$),
    tap({
      next: value => console.log('Waarde:', value),
      complete: () => console.log('Gestopt door timeout')
    })
  )
  .subscribe();
```

## Scenario 4: Geheugenlek (vergeten unsubscribe)

- **Symptoom**: Applicatie wordt geleidelijk langzamer

### Oorzaak: Subscriptions niet opzeggen wanneer niet meer nodig

```ts
import { interval } from 'rxjs';

class UserComponent {
  private subscription: any;

  ngOnInit() {
    // ❌ Vergeten te unsubscribe
    interval(1000).subscribe(value => {
      console.log('Waarde:', value); // Blijft uitvoeren na vernietiging component
    });
  }

  ngOnDestroy() {
    // Geen unsubscribe
  }
}

// ✅ Subscriptions juist beheren
class UserComponentFixed {
  private subscription: any;

  ngOnInit() {
    this.subscription = interval(1000).subscribe(value => {
      console.log('Waarde:', value);
    });
  }

  ngOnDestroy() {
    // Unsubscribe bij vernietiging component
    if (this.subscription) {
      this.subscription.unsubscribe();
    }
  }
}
```

**Aanbevolen patroon: gebruik `takeUntil`**

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class UserComponentBest {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    // ✅ Automatische unsubscribe met takeUntil
    interval(1000)
      .pipe(
        takeUntil(this.destroy$)
      )
      .subscribe(value => console.log('Waarde:', value));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### Detecteren van geheugenlekken

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

let subscriptionCount = 0;

const trackSubscriptions = <T>() =>
  tap<T>({
    subscribe: () => {
      subscriptionCount++;
      console.log('📈 Aantal subscriptions:', subscriptionCount);
    },
    unsubscribe: () => {
      subscriptionCount--;
      console.log('📉 Aantal subscriptions:', subscriptionCount);
    }
  });

// Gebruiksvoorbeeld
const stream$ = interval(1000).pipe(
  trackSubscriptions()
);

const sub1 = stream$.subscribe();
// Output: 📈 Aantal subscriptions: 1

const sub2 = stream$.subscribe();
// Output: 📈 Aantal subscriptions: 2

setTimeout(() => {
  sub1.unsubscribe();
  // Output: 📉 Aantal subscriptions: 1
}, 3000);
```

## Scenario 5: Fouten worden niet opgemerkt

- **Symptoom**: Fouten treden op maar worden genegeerd en niet weergegeven

```ts
import { of, throwError } from 'rxjs';
import { mergeMap, catchError } from 'rxjs';

// ❌ Fouten worden onderdrukt zonder foutafhandeling
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Fout'));
      }
      return of(value);
    })
  )
  .subscribe(); // Geen error handler

// ✅ Passende foutafhandeling
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Fout'));
      }
      return of(value);
    }),
    catchError(error => {
      console.error('🔴 Fout gevangen:', error.message);
      return of(-1); // Fallback-waarde
    })
  )
  .subscribe({
    next: value => console.log('Waarde:', value),
    error: error => console.error('🔴 Fout in subscription:', error)
  });

// Output:
// Waarde: 1
// 🔴 Fout gevangen: Fout
// Waarde: -1
```

### Globale fouthandler instellen

```ts
import { Observable } from 'rxjs';

// Vang alle onafgehandelde fouten op
const originalCreate = Observable.create;

Observable.create = function(subscribe: any) {
  return originalCreate.call(this, (observer: any) => {
    try {
      return subscribe(observer);
    } catch (error) {
      console.error('🔴 Onafgehandelde fout:', error);
      observer.error(error);
    }
  });
};
```

## Scenario 6: Retry-pogingen willen bijhouden

- **Symptoom**: De `retry`-operator wordt gebruikt, maar je weet niet hoe vaak er wordt geretried

Bij automatische retry's na fouten wordt debuggen en logging gemakkelijker door bij te houden hoeveel keer er daadwerkelijk wordt geretried.

### Basis retry-debugging

```ts
import { throwError, of, timer } from 'rxjs';
import { retryWhen, mergeMap, tap } from 'rxjs';

throwError(() => new Error('Tijdelijke fout'))
  .pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          console.log(`🔄 Retry-poging ${retryCount}`);

          if (retryCount > 2) {
            console.log('❌ Maximum aantal retry\'s bereikt');
            throw error;
          }

          return timer(1000);
        })
      )
    )
  )
  .subscribe({
    next: value => console.log('✅ Geslaagd:', value),
    error: error => console.log('🔴 Definitieve fout:', error.message)
  });

// Output:
// 🔄 Retry-poging 1
// 🔄 Retry-poging 2
// 🔄 Retry-poging 3
// ❌ Maximum aantal retry's bereikt
// 🔴 Definitieve fout: Tijdelijke fout
```

> [!TIP]
> Voor meer gedetailleerde implementatiepatronen van retry-debugging, zie het gedeelte "Retry debugging" in [retry en catchError](/nl/guide/error-handling/retry-catch#retry-debugging).
> - Basis tracking met tap error callback
> - Gedetailleerde logging met retryWhen
> - Exponentiële backoff met logging
> - RxJS 7.4+ retry-configuratieobject

## Samenvatting

Oplossingen voor veelvoorkomende debugscenario's

- ✅ **Waarden stromen niet** → Controleer vergeten subscription, filtervoorwaarden
- ✅ **Onverwachte waarden** → Let op operatorvolgorde, delen van referenties
- ✅ **Subscription wordt niet voltooid** → Gebruik `take` of `takeUntil` voor oneindige streams
- ✅ **Geheugenlek** → Automatische unsubscribe met `takeUntil`-patroon
- ✅ **Gemiste fouten** → Implementeer passende foutafhandeling
- ✅ **Retry-tracking** → Log met `retryWhen` of configuratieobject

## Gerelateerde pagina's

- [Basis debugstrategieën](/nl/guide/debugging/) - Gebruik van tap-operator en ontwikkelaarstools
- [Aangepaste debugtools](/nl/guide/debugging/custom-tools) - Named streams, debug-operators
- [Prestatiedebuggen](/nl/guide/debugging/performance) - Subscription-monitoring, geheugengebruik controleren
- [Foutafhandeling](/nl/guide/error-handling/strategies) - Strategieën voor foutverwerking
