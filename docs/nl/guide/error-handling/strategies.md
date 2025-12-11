---
description: "Uitgebreide RxJS error handling strategieën uitgelegd. Combinatie van catchError, retry, retryWhen, finalize operators, exponential backoff retry, error classificatie en geschikte verwerking, globale error handlers en meer, leer hoe je robuuste error handling implementeert in TypeScript."
---
# RxJS Error handling strategieën

Error handling in RxJS is een belangrijk aspect van reactief programmeren. Door geschikte error handling te implementeren verbeter je de robuustheid en betrouwbaarheid van applicaties. Dit document legt verschillende error handling strategieën uit die beschikbaar zijn in RxJS.

## Basispatronen

In RxJS verwerk je errors als deel van de Observable levenscyclus. Basis error handling methoden zijn als volgt.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

// Observable die error genereert
const error$ = throwError(() => new Error('Error opgetreden')); // RxJS 7 en later, functie vorm aanbevolen

// Basis error handling
error$
  .pipe(
    catchError((error) => {
      console.error('Error opgevangen:', error.message);
      return of('Fallback waarde na error');
    })
  )
  .subscribe({
    next: (value) => console.log('Waarde:', value),
    error: (err) => console.error('Niet-afgehandelde error:', err),
    complete: () => console.log('Voltooid'),
  });

// Output:
// Error opgevangen: Error opgetreden
// Waarde: Fallback waarde na error
// Voltooid
```

## Verschillende error handling strategieën

### 1. Error opvangen en alternatieve waarde bieden

Gebruik de `catchError` operator om errors op te vangen en een alternatieve waarde of alternatieve stream te bieden.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

const source$ = throwError(() => new Error('Data ophalen error'));

source$.pipe(
  catchError(error => {
    console.error('Error opgetreden:', error.message);
    // Alternatieve data retourneren
    return of({ isError: true, data: [], message: 'Standaarddata wordt weergegeven' });
  })
).subscribe(data => console.log('Resultaat:', data));

// Output:
// Error opgetreden: Data ophalen error
// Resultaat: {isError: true, data: Array(0), message: 'Standaarddata wordt weergegeven'}
```

### 2. Opnieuw proberen bij error

Gebruik `retry` of `retryWhen` operators om de stream opnieuw te proberen wanneer een error optreedt.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, tap } from 'rxjs';

let attemptCount = 0;

interval(1000).pipe(
  mergeMap(val => {
    if (++attemptCount <= 2) {
      return throwError(() => new Error(`Error #${attemptCount}`));
    }
    return of('Succes!');
  }),
  tap(() => console.log('Uitvoeren:', attemptCount)),
  retry(2), // Maximaal 2 keer opnieuw proberen
).subscribe({
  next: value => console.log('Waarde:', value),
  error: err => console.error('Finale error:', err.message),
});

// Output:
// Uitvoeren: 3
// Waarde: Succes!
// Uitvoeren: 4
// Waarde: Succes!
// Uitvoeren: 5
// ...
```

### 3. Retry met exponential backoff

Voor netwerkverzoeken is "exponential backoff" (retry interval geleidelijk verlengen) effectief.

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
// Error opgetreden: Netwerkfout
// 4e retry na 10000ms uitvoeren
// Error opgetreden: Netwerkfout
// 5e retry na 10000ms uitvoeren
// Alle retry pogingen mislukt: Maximum retry aantal overschreden
// Resultaat: {error: true, message: 'Verbinding mislukt. Probeer later opnieuw.'}
```

### 4. Resource vrijgave bij error

Gebruik de `finalize` operator om resources vrij te geven wanneer de stream eindigt met **completion of error**.
finalize is effectief wanneer je cleanup zeker wilt uitvoeren niet alleen bij error maar ook bij normale completion.

```ts
import { throwError } from 'rxjs';
import { catchError, finalize } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Verwerkingsfout'))
  .pipe(
    catchError((error) => {
      console.error('Error verwerking:', error.message);
      return throwError(() => error); // Error opnieuw gooien
    }),
    finalize(() => {
      isLoading = false;
      console.log('Loading status resetten:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Waarde:', value),
    error: (err) => console.error('Finale error:', err.message),
    complete: () => console.log('Voltooid'),
  });

// Output:
// Error verwerking: Verwerkingsfout
// Finale error: Verwerkingsfout
// Loading status resetten: false
```

## Error handling patronen

### UI element weergave control inclusief error handling

```ts
import { of, throwError } from 'rxjs';
import { catchError, finalize, tap } from 'rxjs';

function fetchData(shouldFail = false) {
  // Loading weergeven
  showLoadingIndicator();

  // Data ophalen (succes of error)
  return (
    shouldFail
      ? throwError(() => new Error('API error'))
      : of({ name: 'Data', value: 42 })
  ).pipe(
    tap((data) => {
      // Verwerking bij succes
      updateUI(data);
    }),
    catchError((error) => {
      // UI update bij error
      showErrorMessage(error.message);
      // Lege data of standaardwaarde retourneren
      return of({ name: 'Standaard', value: 0 });
    }),
    finalize(() => {
      // Loading verbergen ongeacht succes of error
      hideLoadingIndicator();
    })
  );
}

// UI operatie helper functies
function showLoadingIndicator() {
  console.log('Loading weergeven');
}
function hideLoadingIndicator() {
  console.log('Loading verbergen');
}
function updateUI(data: { name: string; value: number }) {
  console.log('UI update:', data);
}
function showErrorMessage(message: any) {
  console.log('Error weergeven:', message);
}

// Gebruiksvoorbeeld
fetchData(true).subscribe();

// Output:
// Loading weergeven
// Error weergeven: API error
// Loading verbergen
```

### Verwerking van meerdere error bronnen

```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, map } from 'rxjs';

// Simuleer meerdere API verzoeken
function getUser() {
  return of({ id: 1, name: 'Yamada Taro' });
}

function getPosts() {
  return throwError(() => new Error('Posts ophalen error'));
}

function getComments() {
  return throwError(() => new Error('Comments ophalen error'));
}

// Alle data ophalen en gedeeltelijke errors toestaan
forkJoin({
  user: getUser().pipe(
    catchError((error) => {
      console.error('Gebruiker ophalen error:', error.message);
      return of(null); // Retourneer null bij error
    })
  ),
  posts: getPosts().pipe(
    catchError((error) => {
      console.error('Posts ophalen error:', error.message);
      return of([]); // Retourneer lege array bij error
    })
  ),
  comments: getComments().pipe(
    catchError((error) => {
      console.error('Comments ophalen error:', error.message);
      return of([]); // Retourneer lege array bij error
    })
  ),
})
  .pipe(
    map((result) => ({
      ...result,
      // Voeg vlag toe die aangeeft of er gedeeltelijke errors waren
      hasErrors:
        !result.user ||
        result.posts.length === 0 ||
        result.comments.length === 0,
    }))
  )
  .subscribe((data) => {
    console.log('Eindresultaat:', data);

    if (data.hasErrors) {
      console.log(
        'Sommige data ophalen mislukt, maar beschikbare data wordt weergegeven'
      );
    }
  });

// Output:
// Posts ophalen error: Posts ophalen error
// Comments ophalen error: Comments ophalen error
// Eindresultaat: {user: {…}, posts: Array(0), comments: Array(0), hasErrors: true}
// Sommige data ophalen mislukt, maar beschikbare data wordt weergegeven
```

## Error handling best practices

1. **Vang errors altijd op**: Voeg altijd error handling toe in Observable chains. Vooral belangrijk voor langlopende streams.

2. **Geef zinvolle error berichten**: Neem informatie op in error objecten die helpt bij het identificeren van de locatie en oorzaak.

3. **Geef resources correct vrij**: Gebruik `finalize` om te zorgen dat resources worden vrijgegeven ongeacht succes of fout.

4. **Overweeg retry strategieën**: Voor netwerk operaties verbetert het implementeren van geschikte retry strategieën de betrouwbaarheid.

5. **Gebruiksvriendelijke error handling**: Geef in UI geen technische error berichten direct weer, maar informatie die gebruikers kunnen begrijpen.

```ts
// Voorbeeld: Conversie naar gebruiksvriendelijke error berichten
function getErrorMessage(error: any): string {
  if (error.status === 401) {
    return 'Sessie is verlopen. Log opnieuw in.';
  } else if (error.status === 404) {
    return 'Gevraagde resource niet gevonden.';
  } else if (error.status >= 500) {
    return 'Serverfout opgetreden. Probeer later opnieuw.';
  }
  return 'Onverwachte fout opgetreden.';
}
```

## Samenvatting

Error handling in RxJS is een belangrijk onderdeel voor het waarborgen van applicatie robuustheid. Door operators zoals `catchError`, `retry`, `finalize` correct te combineren kun je verschillende error scenario's aanpakken. Ontwerp niet alleen om errors op te vangen, maar ook een uitgebreide error handling strategie om de gebruikerservaring te verbeteren.

## 🔗 Gerelateerde secties

- **[Veelvoorkomende fouten en oplossingen](/nl/guide/anti-patterns/common-mistakes#9-error-onderdrukking)** - Bevestig anti-patronen gerelateerd aan error handling
- **[retry en catchError](/nl/guide/error-handling/retry-catch)** - Meer gedetailleerde gebruiksmethoden uitgelegd
