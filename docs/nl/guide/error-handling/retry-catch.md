---
description: "Leer robuuste error handling strategieën door retry en catchError operators te combineren. Behandelt tijdelijke storingen opnieuw proberen, exponential backoff patronen, conditionele retry, geschikte fallback verwerking en meer, met TypeScript type-safe implementaties via praktische codevoorbeelden."
---
# retry en catchError - Effectieve error handling combinatie

We leggen in detail de twee kern operators voor error handling in RxJS uit: `retry` en `catchError`. Door deze te combineren kun je robuuste error handling strategieën realiseren.

## retry - Opnieuw proberen bij fout (basispatroon)

De `retry` operator is een operator die **stream uitvoering herstart voor een opgegeven aantal keren** wanneer een error optreedt in de stream. Het is vooral effectief voor operaties die tijdelijk kunnen mislukken, zoals netwerkverzoeken.

[🌐 RxJS Officiële Documentatie - retry](https://rxjs.dev/api/index/function/retry)

### Basispatroon

```ts
import { Observable, of } from 'rxjs';
import { retry, map } from 'rxjs';

// Functie die willekeurig error genereert
function getDataWithRandomError(): Observable<string> {
  return of('Data').pipe(
    map(() => {
      if (Math.random() < 0.7) {
        throw new Error('Willekeurige error opgetreden');
      }
      return 'Data ophalen succesvol!';
    })
  );
}

// Maximaal 3 keer opnieuw proberen
getDataWithRandomError()
  .pipe(retry(3))
  .subscribe({
    next: (data) => console.log('Succes:', data),
    error: (err) => console.error('Error (na 3 retry pogingen):', err.message),
  });

// Output:
// Succes: Data ophalen succesvol!
// Error (na 3 retry pogingen): Willekeurige error opgetreden ⇦ Weergegeven bij 3 mislukkingen
```

### Real-time monitoring van retry status

```ts
import { Observable, of } from 'rxjs';
import { retry, tap, catchError, map } from 'rxjs';

let attempts = 0;

function simulateFlakyRequest(): Observable<string> {
  return of('Verzoek').pipe(
    tap(() => {
      attempts++;
      console.log(`Poging #${attempts}`);
    }),
    map(() => {
      if (attempts < 3) {
        throw new Error(`Error #${attempts}`);
      }
      return 'Succes!';
    })
  );
}

simulateFlakyRequest()
  .pipe(
    retry(3),
    catchError((error) => {
      console.log('Alle retry pogingen mislukt:', error.message);
      return of('Fallback waarde');
    })
  )
  .subscribe({
    next: (result) => console.log('Eindresultaat:', result),
    complete: () => console.log('Voltooid'),
  });

// Output:
// Poging #1
// Poging #2
// Poging #3
// Eindresultaat: Succes!
// Voltooid
```

> [!NOTE] Retry timing en schedulers
> Wanneer je delay tijd specificeert bij `retry` operator (`retry({ delay: 1000 })` etc.), wordt intern **asyncScheduler** gebruikt. Door schedulers te gebruiken kun je retry timing fijn controleren of virtuele tijd gebruiken bij testen.
>
> Zie voor details [Scheduler types en gebruik - Error retry control](/nl/guide/schedulers/types#error-retry-control).

## catchError - Error opvangen en alternatieve verwerking (basispatroon)

De `catchError` operator vangt errors op die optreden in de stream en verwerkt ze door **een alternatieve Observable te retourneren**. Hierdoor kan de stream doorgaan zonder onderbreking zelfs bij een error.

[🌐 RxJS Officiële Documentatie - catchError](https://rxjs.dev/api/index/function/catchError)

### Basispatroon

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('API aanroep error')) // RxJS 7 en later, functie vorm aanbevolen
  .pipe(
    catchError((error) => {
      console.error('Error opgetreden:', error.message);
      return of('Standaardwaarde bij error');
    })
  )
  .subscribe({
    next: (value) => console.log('Waarde:', value),
    complete: () => console.log('Voltooid'),
  });

// Output:
// Error opgetreden: API aanroep error
// Waarde: Standaardwaarde bij error
// Voltooid
```

### Error opnieuw gooien

Wanneer je error wilt loggen en daarna opnieuw gooien

```ts
import { throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('Oorspronkelijke error')) // RxJS 7 en later, functie vorm aanbevolen
  .pipe(
    catchError((error) => {
      console.error('Error loggen:', error.message);
      // Error opnieuw gooien
      return throwError(() => new Error('Getransformeerde error'));
    })
  )
  .subscribe({
    next: (value) => console.log('Waarde:', value),
    error: (err) => console.error('Finale error:', err.message),
    complete: () => console.log('Voltooid'),
  });

// Output:
// Error loggen: Oorspronkelijke error
// Finale error: Getransformeerde error
```

## Combinatie van retry en catchError

In praktische applicaties is het gebruikelijk om `retry` en `catchError` te combineren. Deze combinatie maakt het mogelijk om tijdelijke errors op te lossen met retry, terwijl bij definitieve mislukking een fallback waarde wordt geleverd.

```ts
import { of, throwError } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

function fetchData() {
  // Observable die error genereert
  return throwError(() => new Error('Netwerkfout')) // RxJS 7 en later, functie vorm aanbevolen
    .pipe(
    // Voor debugging
    tap(() => console.log('Data ophalen proberen')),
    // Maximaal 3 keer opnieuw proberen
    retry(3),
    // Als alle retry pogingen mislukken
    catchError((error) => {
      console.error('Alle retry pogingen mislukt:', error.message);
      // Standaardwaarde retourneren
      return of({
        error: true,
        data: null,
        message: 'Data ophalen mislukt',
      });
    })
  );
}

fetchData().subscribe({
  next: (result) => console.log('Resultaat:', result),
  complete: () => console.log('Verwerking voltooid'),
});

// Output:
// Alle retry pogingen mislukt: Netwerkfout
// Resultaat: {error: true, data: null, message: 'Data ophalen mislukt'}
// Verwerking voltooid
```

## Geavanceerde retry strategie: retryWhen

Voor flexibelere retry strategieën kun je de `retryWhen` operator gebruiken. Hiermee kun je retry timing en logica aanpassen.


[🌐 RxJS Officiële Documentatie - retryWhen](https://rxjs.dev/api/index/function/retryWhen)

### Retry met exponential backoff

Voor netwerkverzoek retry is exponential backoff patroon (retry interval geleidelijk verlengen) gebruikelijk. Dit vermindert server belasting en wacht tot tijdelijke problemen zijn opgelost.

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, tap, concatMap, catchError } from 'rxjs';

function fetchWithRetry() {
  let retryCount = 0;

  return throwError(() => new Error('Netwerkfout')).pipe(
    retryWhen((errors) =>
      errors.pipe(
        // Error aantal tellen
        tap((error) => console.log('Error opgetreden:', error.message)),
        // Vertraging met exponential backoff
        concatMap(() => {
          retryCount++;
          const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
          console.log(`${retryCount}e retry na ${delayMs}ms uitvoeren`);
          // timer gebruikt intern asyncScheduler
          return timer(delayMs);
        }),
        // Maximaal 5 keer opnieuw proberen
        tap(() => {
          if (retryCount >= 5) {
            throw new Error('Maximum retry aantal overschreden');
          }
        })
      )
    ),
    // Definitieve fallback
    catchError((error) => {
      console.error('Alle retry pogingen mislukt:', error.message);
      return of({
        error: true,
        message: 'Verbinding mislukt. Probeer later opnieuw.',
      });
    })
  );
}

fetchWithRetry().subscribe({
  next: (result) => console.log('Resultaat:', result),
  error: (err) => console.error('Niet-afgehandelde error:', err),
});

// Output:
// Error opgetreden: Netwerkfout
// 1e retry na 2000ms uitvoeren
// Error opgetreden: Netwerkfout
// 2e retry na 4000ms uitvoeren
// Error opgetreden: Netwerkfout
// 3e retry na 8000ms uitvoeren
```

> [!TIP] Gedetailleerde retry control met schedulers
> Het bovenstaande voorbeeld gebruikt `timer()`, maar voor geavanceerde controle kun je schedulers expliciet specificeren om retry timing fijn af te stellen of virtuele tijd te gebruiken bij testen.
>
> Zie voor details [Scheduler types en gebruik - Error retry control](/nl/guide/schedulers/types#error-retry-control).

## Retry debugging

Bij het debuggen van retry verwerking is het belangrijk om het aantal pogingen en resultaat van elke poging te volgen. Hieronder introduceren we praktische methoden om retry status real-time te monitoren.

### Methode 1: tap error callback (basis)

Met de `error` callback van `tap` operator kun je het aantal pogingen tellen bij error optreden.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Tijdelijke error'))
  .pipe(
    tap({
      error: () => {
        attemptCount++;
        console.log(`Aantal pogingen: ${attemptCount}`);
      }
    }),
    retry(2),
    catchError((error) => {
      console.log(`Finaal aantal pogingen: ${attemptCount}`);
      return of(`Finale error: ${error.message}`);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Subscribe error:', err)
  });

// Output:
// Aantal pogingen: 1
// Aantal pogingen: 2
// Aantal pogingen: 3
// Finaal aantal pogingen: 3
// Finale error: Tijdelijke error
```

> [!NOTE] Beperking met throwError
> Omdat `throwError` onmiddellijk een error geeft zonder waarde uit te zenden, wordt de `next` callback van `tap` niet uitgevoerd. Je moet de `error` callback gebruiken.

### Methode 2: Gedetailleerd volgen met retryWhen (aanbevolen)

Voor gedetailleerde informatie (aantal pogingen, vertragingstijd, error inhoud) gebruik je `retryWhen`.

```typescript
import { throwError, of, timer, retryWhen, mergeMap, catchError } from 'rxjs';
throwError(() => new Error('Tijdelijke error'))
  .pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
          console.log(`🔄 Retry ${retryCount}e keer`);
          console.log(`   Error: ${error.message}`);

          if (retryCount > 2) {
            console.log(`❌ Maximum retry aantal bereikt`);
            console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
            throw error;
          }

          const delayMs = 1000;
          console.log(`⏳ Opnieuw proberen na ${delayMs}ms...`);
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

          return timer(delayMs);
        })
      )
    ),
    catchError((error) => {
      console.log(`\nEindresultaat: Alle retry mislukt`);
      return of(`Finale error: ${error.message}`);
    })
  )
  .subscribe(result => console.log('Resultaat:', result));

// Output:
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 1e keer
//    Error: Tijdelijke error
// ⏳ Opnieuw proberen na 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (1 seconde wachten)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 2e keer
//    Error: Tijdelijke error
// ⏳ Opnieuw proberen na 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (1 seconde wachten)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 3e keer
//    Error: Tijdelijke error
// ❌ Maximum retry aantal bereikt
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
//
// Eindresultaat: Alle retry mislukt
// Resultaat: Finale error: Tijdelijke error
```

### Methode 3: Aantal pogingen volgen met custom Observable

Voor praktische API verzoeken of Observables die waarden uitzenden, kun je het aantal pogingen beheren met een custom Observable.

```typescript
import { Observable, of, retry, catchError } from 'rxjs';
let attemptCount = 0;

// Observable die aantal pogingen kan tellen
const retryableStream$ = new Observable(subscriber => {
  attemptCount++;
  console.log(`[Poging ${attemptCount}e keer]`);

  // Eerste 2 keer mislukken, 3e keer succes
  if (attemptCount < 3) {
    subscriber.error(new Error(`Mislukt (poging${attemptCount})`));
  } else {
    subscriber.next('Succes data');
    subscriber.complete();
  }
});

retryableStream$
  .pipe(
    retry(2),
    catchError((error) => {
      console.log(`[Voltooid] Totaal ${attemptCount} keer geprobeerd en mislukt`);
      return of(`Finale error: ${error.message}`);
    })
  )
  .subscribe({
    next: data => console.log('[Resultaat]', data),
    complete: () => console.log('[Voltooid]')
  });

// Output:
// [Poging 1e keer]
// [Poging 2e keer]
// [Poging 3e keer]
// [Resultaat] Succes data
// [Voltooid]
```

### Methode 4: Exponential backoff en logging

Praktisch logging patroon voor API verzoeken.

```typescript
import { timer, throwError, of, retryWhen, mergeMap, catchError, finalize } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchWithRetryLogging(url: string, maxRetries = 3) {
  let startTime = Date.now();

  return ajax.getJSON(url).pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          const elapsed = Date.now() - startTime;

          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
          console.log(`🔄 Retry informatie`);
          console.log(`   Aantal: ${retryCount}/${maxRetries}`);
          console.log(`   Error: ${error.message || error.status}`);
          console.log(`   Verstreken tijd: ${elapsed}ms`);

          if (retryCount >= maxRetries) {
            console.log(`❌ Maximum retry aantal bereikt`);
            console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
            throw error;
          }

          // Exponential backoff
          const delayMs = Math.min(1000 * Math.pow(2, index), 10000);
          console.log(`⏳ Opnieuw proberen na ${delayMs}ms...`);
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

          return timer(delayMs);
        })
      )
    ),
    catchError((error) => {
      const totalTime = Date.now() - startTime;
      console.log(`\n❌ Definitief mislukt (totale tijd: ${totalTime}ms)`);
      return of({ error: true, message: 'Data ophalen mislukt' });
    }),
    finalize(() => {
      const totalTime = Date.now() - startTime;
      console.log(`\n✅ Verwerking voltooid (totale tijd: ${totalTime}ms)`);
    })
  );
}

// Gebruiksvoorbeeld
fetchWithRetryLogging('https://jsonplaceholder.typicode.com/users/1').subscribe({
  next: data => console.log('Data:', data),
  error: err => console.error('Error:', err)
});
```

### Methode 5: RxJS 7.4+ retry configuratie object

Vanaf RxJS 7.4 kun je een configuratie object aan `retry` doorgeven.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Tijdelijke error'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Poging ${attemptCount}e keer`);
      },
      error: (err) => console.log(`Error opgetreden:`, err.message)
    }),
    retry({
      count: 2,
      delay: 1000, // 1 seconde wachten voor retry (gebruikt intern asyncScheduler)
      resetOnSuccess: true
    }),
    catchError((error) => {
      console.log(`Definitief mislukt (totaal ${attemptCount} keer geprobeerd)`);
      return of(`Finale error: ${error.message}`);
    })
  )
  .subscribe(result => console.log('Resultaat:', result));

// Output:
// Poging 1e keer
// Error opgetreden: Tijdelijke error
// Poging 2e keer
// Error opgetreden: Tijdelijke error
// Poging 3e keer
// Error opgetreden: Tijdelijke error
// Definitief mislukt (totaal 3 keer geprobeerd)
// Resultaat: Finale error: Tijdelijke error
```

> [!TIP] Aanbevolen aanpak voor retry debugging
> - **Tijdens ontwikkeling**: Methode 2 (retryWhen) of methode 4 (gedetailleerde logs) is optimaal
> - **Productie omgeving**: Methode 4 als basis, met log verzending naar error monitoring service
> - **Eenvoudige gevallen**: Methode 1 (tap error) of methode 5 (retry configuratie) is voldoende
>
> **Gerelateerde informatie**:
> - Voor retry timing control zie [Scheduler types en gebruik - Error retry control](/nl/guide/schedulers/types#error-retry-control)
> - Voor algemeen debug overzicht zie [RxJS Debug Technieken - Retry pogingen volgen](/nl/guide/debugging/#scenario-6-retry-pogingen-volgen)

## Praktisch gebruik in applicaties: API verzoeken

Voorbeeld van gebruik van deze operators bij praktische API verzoeken.

```ts
import { Observable, of } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { retry, catchError, finalize, tap } from 'rxjs';

// Loading status
let isLoading = false;

function fetchUserData(userId: string): Observable<any> {
  isLoading = true;

  return ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`).pipe(
    // Verzoek debugging
    tap((response) => console.log('API response:', response)),
    // Netwerkfouten maximaal 2 keer opnieuw proberen
    retry(2),
    // Error handling
    catchError((error) => {
      if (error.status === 404) {
        return of({ error: true, message: 'Gebruiker niet gevonden' });
      } else if (error.status >= 500) {
        return of({ error: true, message: 'Serverfout opgetreden' });
      }
      return of({ error: true, message: 'Onbekende fout opgetreden' });
    }),
    // Altijd uitvoeren ongeacht succes of fout
    finalize(() => {
      isLoading = false;
      console.log('Loading voltooid');
    })
  );
}

// Gebruiksvoorbeeld
fetchUserData('123').subscribe({
  next: (data) => {
    if (data.error) {
      // Error informatie weergeven
      console.error('Error:', data.message);
    } else {
      // Data weergeven
      console.log('Gebruikersdata:', data);
    }
  },
});


// Output:
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// Onbekende fout opgetreden
// Loading voltooid
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
```

## Best practices

### Wanneer retry te gebruiken

- Bij **tijdelijke errors** die verwacht worden (zoals netwerkverbindingsproblemen)
- Bij **tijdelijke serverproblemen** (hoge belasting of timeouts)
- Wanneer errors **mogelijk opgelost kunnen worden** door opnieuw proberen

### Wanneer retry niet te gebruiken

- **Authenticatie errors** (401, 403) - opnieuw proberen lost het niet op
- **Resource bestaat niet** (404) - zal niet gevonden worden door opnieuw proberen
- **Validatie errors** (400) - probleem ligt bij het verzoek zelf
- **Client-side programmafouten** - opnieuw proberen is zinloos

### Effectief gebruik van catchError

- Voer **verschillende verwerking uit afhankelijk van error type**
- Geef gebruikers **begrijpelijke berichten**
- Retourneer **fallback data** wanneer gepast
- **Transformeer errors** wanneer nodig

## Samenvatting

Door `retry` en `catchError` te combineren wordt robuuste error handling mogelijk. Tijdelijke errors worden geprobeerd te herstellen met retry, en permanente errors krijgen geschikte fallback verwerking, waardoor de gebruikerservaring verbetert. In praktische applicaties is het belangrijk om geschikte strategieën te kiezen afhankelijk van de aard van errors en fallback mechanismen te bieden.

In de volgende sectie leggen we de `finalize` operator voor resource vrijgave en stream completion verwerking uit.
