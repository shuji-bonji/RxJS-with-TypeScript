---
description: "Leer hoe je finalize en complete gebruikt voor effectieve stream completion en resource cleanup in RxJS. Voorkomt geheugenlekken, bestandshandler vrijgave, WebSocket verbinding cleanup, UI status reset en meer praktische patronen. Inclusief uitleg over het verschil met finally."
---
# finalize en complete - Resource vrijgave en stream completion

In RxJS is het belangrijk om de beëindiging van streams en resource vrijgave correct te beheren. Deze pagina legt het mechanisme van de `finalize` operator en `complete` notificatie uit.

## finalize - Operator voor resource vrijgave

De `finalize` operator is een operator die een opgegeven cleanup code uitvoert wanneer een Observable **eindigt met completion, error of unsubscribe**.
finalize wordt **altijd precies één keer aangeroepen bij stream beëindiging** en wordt nooit meerdere keren aangeroepen.

[🌐 RxJS Officiële Documentatie - finalize](https://rxjs.dev/api/index/function/finalize)

### Basisgebruik van finalize

```ts
import { of } from 'rxjs';
import { finalize, tap } from 'rxjs';

// Variabele voor het beheren van loading status
let isLoading = true;

// Succesvolle stream
of('Data')
  .pipe(
    tap((data) => console.log('Data verwerken:', data)),
    // Wordt uitgevoerd in alle gevallen: succes, fout of annulering
    finalize(() => {
      isLoading = false;
      console.log('Loading status reset:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Waarde:', value),
    complete: () => console.log('Voltooid'),
  });

// Output:
// Data verwerken: Data
// Waarde: Data
// Voltooid
// Loading status reset: false
```

### finalize bij error

```ts
import { throwError } from 'rxjs';
import { finalize, catchError } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Data ophalen mislukt'))
  .pipe(
    catchError((err) => {
      console.error('Error afhandeling:', err.message);
      throw err; // Error opnieuw gooien
    }),
    finalize(() => {
      isLoading = false;
      console.log('Resource vrijgave na error:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Waarde:', value),
    error: (err) => console.error('Error bij subscriber:', err.message),
    complete: () => console.log('Voltooid'), // Wordt niet aangeroepen bij error
  });

// Output:
// Error afhandeling: Data ophalen mislukt
// Error bij subscriber: Data ophalen mislukt
// Resource vrijgave na error: false
```

### finalize bij unsubscribe

```ts
import { interval } from 'rxjs';
import { finalize } from 'rxjs';

let resource = 'Actief';

// Tellen elke seconde
const subscription = interval(1000)
  .pipe(
    finalize(() => {
      resource = 'Vrijgegeven';
      console.log('Resource status:', resource);
    })
  )
  .subscribe((count) => {
    console.log('Telling:', count);

    // Handmatig unsubscribe na 3 tellingen
    if (count >= 2) {
      subscription.unsubscribe();
    }
  });

// Output:
// Telling: 0
// Telling: 1
// Telling: 2
// Resource status: Vrijgegeven
```

finalize is effectief wanneer je cleanup wilt uitvoeren niet alleen bij een error, maar ook bij normale completion of handmatige unsubscribe.

## complete - Normale beëindigings notificatie van stream

Wanneer een Observable normaal eindigt, wordt de `complete` callback van de Observer aangeroepen. Dit is de laatste stap in de levenscyclus van de Observable.

### Automatische complete

Sommige Observables worden automatisch voltooid wanneer aan bepaalde voorwaarden is voldaan.

```ts
import { of } from 'rxjs';
import { take } from 'rxjs';

// Eindige sequenties worden automatisch voltooid
of(1, 2, 3).subscribe({
  next: (value) => console.log('Waarde:', value),
  complete: () => console.log('Eindige stream voltooid'),
});

// interval + take beperkte stream
interval(1000)
  .pipe(
    take(3) // Voltooid na 3 waarden
  )
  .subscribe({
    next: (value) => console.log('Telling:', value),
    complete: () => console.log('Beperkte stream voltooid'),
  });

// Output:
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Eindige stream voltooid
// Telling: 0
// Telling: 1
// Telling: 2
// Beperkte stream voltooid

```

### Handmatige complete

Bij Subject of custom Observables kun je complete handmatig aanroepen.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.subscribe({
  next: (value) => console.log('Waarde:', value),
  complete: () => console.log('Subject voltooid'),
});

subject.next(1);
subject.next(2);
subject.complete(); // Handmatig voltooien
subject.next(3); // Wordt genegeerd na completion

// Output:
// Waarde: 1
// Waarde: 2
// Subject voltooid
```

## Verschil tussen finalize en complete

Laten we de belangrijke verschillen begrijpen.

1. **Uitvoeringstiming**
   - `complete`: Wordt **alleen aangeroepen bij normale completion** van Observable
   - `finalize`: Wordt aangeroepen wanneer Observable eindigt met **completion, error of unsubscribe**

2. **Gebruiksdoel**
   - `complete`: Ontvangen van notificatie bij normale beëindiging (verwerking bij succes)
   - `finalize`: Resource vrijgave of cleanup zeker uitvoeren (verwerking die altijd uitgevoerd moet worden, ongeacht succes of fout)

## Praktische use cases

### API aanroep en loading status beheer

```ts
import { ajax } from 'rxjs/ajax';
import { finalize, catchError } from 'rxjs';
import { of } from 'rxjs';

// Loading status
let isLoading = false;

function fetchData(id: string) {
  // Start loading
  isLoading = true;
  const loading = document.createElement('p');
  loading.style.display = 'block';
  document.body.appendChild(loading);
  // document.getElementById('loading')!.style.display = 'block';

  // API verzoek
  return ajax.getJSON(`https://jsonplaceholder.typicode.com/posts/${id}`).pipe(
    catchError((error) => {
      console.error('API error:', error);
      return of({ error: true, message: 'Data ophalen mislukt' });
    }),
    // Beëindig loading ongeacht succes of fout
    finalize(() => {
      isLoading = false;
      loading!.style.display = 'none';
      console.log('Loading status reset voltooid');
    })
  );
}

// Gebruiksvoorbeeld
fetchData('123').subscribe({
  next: (data) => console.log('Data:', data),
  complete: () => console.log('Data ophalen voltooid'),
});

// Output:
//  API error: AjaxErrorImpl {message: 'ajax error', name: 'AjaxError', xhr: XMLHttpRequest, request: {…}, status: 0, …}
//  Data: {error: true, message: 'Data ophalen mislukt'}
//  Data ophalen voltooid
//  Loading status reset voltooid
//   GET https://jsonplaceholder.typicode.com/posts/123 net::ERR_NAME_NOT_RESOLVED
```

### Resource cleanup

```ts
import { interval } from 'rxjs';
import { finalize, takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

class ResourceManager {
  private destroy$ = new Subject<void>();
  private timerId: number | null = null;

  constructor() {
    // Initialisatie van resource
    this.timerId = window.setTimeout(() => console.log('Timer uitgevoerd'), 10000);

    // Periodieke verwerking
    interval(1000)
      .pipe(
        // Stop bij component vernietiging
        takeUntil(this.destroy$),
        // Zeker resource vrijgave
        finalize(() => {
          console.log('Interval gestopt');
        })
      )
      .subscribe((count) => {
        console.log('Uitvoeren...', count);
      });
  }

  dispose() {
    // Vernietigingsproces
    if (this.timerId) {
      window.clearTimeout(this.timerId);
      this.timerId = null;
    }

    // Stream stop signaal
    this.destroy$.next();
    this.destroy$.complete();

    console.log('Resource manager vernietigd');
  }
}

// Gebruiksvoorbeeld
const manager = new ResourceManager();

// Vernietigen na 5 seconden
setTimeout(() => {
  manager.dispose();
}, 5000);

// Output:
// Uitvoeren... 0
// Uitvoeren... 1
// Uitvoeren... 2
// Uitvoeren... 3
// Uitvoeren... 4
// Interval gestopt
// Resource manager vernietigd
```

[📘 RxJS Officieel: takeUntil()](https://rxjs.dev/api/index/function/takeUntil)

## Best practices

1. **Geef resources altijd vrij**: Gebruik `finalize` om cleanup te garanderen bij stream beëindiging
2. **Loading status beheer**: Gebruik `finalize` om altijd de loading status te resetten
3. **Component lifecycle beheer**: Combineer `takeUntil` en `finalize` om resources te cleanen bij component vernietiging (dit patroon wordt vooral aanbevolen in Angular)
4. **Combinatie met error handling**: Combineer `catchError` en `finalize` om fallback verwerking na error en zekere cleanup te realiseren
5. **Completion status begrijpen**: Gebruik de `complete` callback om te bepalen of de stream normaal is voltooid

## Samenvatting

`finalize` en `complete` zijn belangrijke tools voor resource beheer en completion verwerking in RxJS. `finalize` is optimaal voor resource vrijgave omdat het zeker wordt uitgevoerd ongeacht hoe de stream eindigt. `complete` wordt daarentegen gebruikt voor verwerking bij normale beëindiging. Door deze correct te combineren kun je geheugenlekken voorkomen en betrouwbare applicaties bouwen.
