---
description: "Beschrijft een uitgebreide foutafhandelingsstrategie voor RxJS, inclusief het combineren van catchError, retry, retryWhen en finalize operatoren, retry met exponentiële backoff, foutclassificatie en gepaste afhandeling, globale foutafhandelingsstrategieën en andere robuuste foutafhandelingsstrategieën in TypeScript. Hoe robuuste foutafhandeling implementeren in TypeScript."
---

# RxJS strategie voor foutafhandeling

Foutafhandeling in RxJS is een belangrijk aspect van reactief programmeren. Het implementeren van een goede foutafhandeling verbetert de robuustheid en betrouwbaarheid van je applicatie. Dit document beschrijft de verschillende foutafhandelingsstrategieën die beschikbaar zijn in RxJS.

## Basispatronen

RxJS handelt fouten af als onderdeel van de Observable levenscyclus. Basis foutafhandeling omvat de volgende methoden.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

// Fout.Observable
const error$ = throwError(() => new Error('Er is een fout opgetreden.')); // RxJS 7Functieformaat vanaf nu aanbevolen

// Basis foutafhandeling
error$
  .pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Fout opvangen:', message);
      return of('Terugvalwaarde na fout');
    })
  )
  .subscribe({
    next: (value) => console.log('Waarde:', value),
    error: (err) => console.error('Fouten niet afgehandeld:', err),
    complete: () => console.log('Voltooid'),
  });

// Uitvoer:
// Fout opvangen: Er is een fout opgetreden.
// Waarde: Terugvalwaarde na fout
// Voltooid
```

## Diverse foutafhandelingsstrategieën

### 1. fouten opvangen en alternatieve waarden geven

Gebruik de `catchError` operator om fouten op te vangen en alternatieve waarden of alternatieve stromen te geven.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

const source$ = throwError(() => new Error('Fout bij gegevensverwerving'));

source$.pipe(
  catchError((error: unknown) => {
    const message = error instanceof Error ? error.message : String(error);
    console.error('Fout opgetreden:', message);
    // Alternatieve gegevens retourneren
    return of({ isError: true, data: [], message: 'Standaardgegevens weergeven' });
  })
).subscribe(data => console.log('Resultaat:', data));

// Uitvoer:
// Fout opgetreden: Fout bij gegevensverwerving
// Resultaat: {isError: true, data: Array(0), message: 'Standaardgegevens weergeven'}
```

### 2. Opnieuw proberen als er een fout optreedt

Gebruik de `retry` operator om de stream opnieuw te proberen als er een fout optreedt (vanaf v7.3 wordt het `retry({ count, delay })` formaat aanbevolen en vervangt het oude `retryWhen`).

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, tap } from 'rxjs';

let attemptCount = 0;

interval(1000).pipe(
  mergeMap(val => {
    if (++attemptCount <= 2) {
      return throwError(() => new Error(`Fout #${attemptCount}`));
    }
    return of('Succes！');
  }),
  tap(() => console.log('Uitvoering:', attemptCount)),
  retry(2), // Max.2Herhalingen
).subscribe({
  next: value => console.log('Waarde:', value),
  error: err => console.error('Laatste fout:', err.message),
});

// Uitvoer:
// Uitvoering: 3
// Waarde: Succes！
// Uitvoering: 4
// Waarde: Succes！
// Uitvoering: 5
// ...
```

### 3. opnieuw proberen met exponentiële backoff

Exponentiële backoff, waarbij het retry-interval geleidelijk wordt verhoogd, is bijvoorbeeld effectief voor netwerkverzoeken.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, tap, catchError } from 'rxjs';

function fetchWithRetry() {
  return throwError(() => new Error('Netwerkfout')).pipe(
    // RxJS 7.3+ Aanbevolen: retry({ count, delay }) Formaat
    retry({
      count: 5, // Max.5Maximaal drie pogingen
      delay: (error: unknown, retryCount) => {
        const message = error instanceof Error ? error.message : String(error);
        console.log('Fout opgetreden:', message);
        // Exponentiële back-off (max.10seconden)
        const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
        console.log(`${retryCount}Een tweede keer opnieuw proberen${delayMs}msUitgevoerd na`);
        return timer(delayMs);
      }
    }),
    // Laatste terugval
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Alle pogingen mislukt:', message);
      return of({
        error: true,
        message: 'Verbinding mislukt. Probeer later opnieuw.',
      });
    })
  );
}

fetchWithRetry().subscribe({
  next: (result) => console.log('Resultaat:', result),
  error: (err) => console.error('Fouten niet afgehandeld:', err),
});

// Uitvoer:
// Fout opgetreden: Netwerkfout
// 1Een tweede keer opnieuw proberen2000msUitgevoerd na
// Fout opgetreden: Netwerkfout
// 2Een tweede keer opnieuw proberen4000msUitgevoerd na
// Fout opgetreden: Netwerkfout
// 3Een tweede keer opnieuw proberen8000msUitgevoerd na
// Fout opgetreden: Netwerkfout
// 4Een tweede keer opnieuw proberen10000msUitgevoerd na
// Fout opgetreden: Netwerkfout
// 5Een tweede keer opnieuw proberen10000msUitgevoerd na
// Alle pogingen mislukt: Maximum aantal pogingen overschreden
// Resultaat: {error: true, message: 'Verbinding mislukt. Probeer later opnieuw.'}
```

### 4. Vrijgeven van middelen bij fout

Gebruik de ` finalize` operator om bronnen vrij te geven als een stream **voltooid of fout** eindigt.
Finalize is nuttig als je ervoor wilt zorgen dat het opschoonproces niet alleen bij fouten wordt uitgevoerd, maar ook bij normale voltooiing.

```ts
import { throwError } from 'rxjs';
import { catchError, finalize } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Verwerkingsfout'))
  .pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Fout Verwerking:', message);
      return throwError(() => error); // Fout opnieuw gooien
    }),
    finalize(() => {
      isLoading = false;
      console.log('Laadstatus resetten:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Waarde:', value),
    error: (err) => console.error('Laatste fout:', err.message),
    complete: () => console.log('Voltooid'),
  });

// Uitvoer:
// Fout Verwerking: Verwerkingsfout
// Laatste fout: Verwerkingsfout
// Laadstatus resetten: false
```

## Foutbehandelingspatroon

### Foutafhandeling inclusief weergavecontrole van UI-elementen

```ts
import { of, throwError } from 'rxjs';
import { catchError, finalize, tap } from 'rxjs';

function fetchData(shouldFail = false) {
  // Weergave laadstatus
  showLoadingIndicator();

  // Gegevensverwerving (succes of fout)
  return (
    shouldFail
      ? throwError(() => new Error('APIFout'))
      : of({ name: 'Gegevens', value: 42 })
  ).pipe(
    tap((data) => {
      // Bij succes
      updateUI(data);
    }),
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      // Bij foutUIbijwerken
      showErrorMessage(message);
      // Lege gegevens of standaardwaarde retourneren
      return of({ name: 'Standaard', value: 0 });
    }),
    finalize(() => {
      // Schakel het laaddisplay uit, ongeacht succes of fout
      hideLoadingIndicator();
    })
  );
}

// UIHelperfuncties voor bewerkingen
function showLoadingIndicator() {
  console.log('Weergave laadstatus');
}
function hideLoadingIndicator() {
  console.log('Verberg laden');
}
function updateUI(data: { name: string; value: number }) {
  console.log('UIbijwerken:', data);
}
function showErrorMessage(message: any) {
  console.log('Foutweergave:', message);
}

// Gebruiksvoorbeeld
fetchData(true).subscribe();

// Uitvoer:
// Weergave laadstatus
// Foutweergave: APIFout
// Verberg laden
```

### Afhandeling van meerdere foutbronnen

```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, map } from 'rxjs';

// Meerdere simulerenAPIVerzoeken simuleren
function getUser() {
  return of({ id: 1, name: 'Taro Yamada' });
}

function getPosts() {
  return throwError(() => new Error('Fout na overname'));
}

function getComments() {
  return throwError(() => new Error('Fout bij overname becommentariëren'));
}

// Alle gegevens ophalen en gedeeltelijke fouten toestaan
forkJoin({
  user: getUser().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Gebruiker overnamefout:', message);
      return of(null); // Bij een foutnullGeeft een lege matrix terug
    })
  ),
  posts: getPosts().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Fout na overname:', message);
      return of([]); // Lege matrix wordt teruggegeven bij fout
    })
  ),
  comments: getComments().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Fout bij overname becommentariëren:', message);
      return of([]); // Lege matrix wordt teruggegeven bij fout
    })
  ),
})
  .pipe(
    map((result) => ({
      ...result,
      // Vlag toevoegen om aan te geven of er een gedeeltelijke fout was
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
        'Het is niet gelukt om sommige gegevens op te halen, maar toont wel beschikbare gegevens'
      );
    }
  });

// Uitvoer:
// Fout na overname: Fout na overname
// Fout bij overname becommentariëren: Fout bij overname becommentariëren
// Eindresultaat: {user: {…}, posts: Array(0), comments: Array(0), hasErrors: true}
// Het is niet gelukt om sommige gegevens op te halen, maar toont wel beschikbare gegevens
```

## Beste praktijk voor foutafhandeling

1. **Vang altijd fouten op**: voeg altijd foutafhandeling toe in de Observable keten. Dit is vooral belangrijk voor langlopende streams.

2.**Geef betekenisvolle foutmeldingen**: neem informatie op in het foutobject om te helpen bepalen waar de fout optrad en wat de oorzaak was.

3, **Bronnen op de juiste manier vrijgeven**: gebruik ` finalize` om ervoor te zorgen dat bronnen worden vrijgegeven, ongeacht succes of mislukking.

4, **overweeg retry-strategieën**: vooral voor netwerkoperaties verbetert het implementeren van een goede retry-strategie de betrouwbaarheid.

5. **Gebruikersvriendelijke foutafhandeling**: geef in de UI informatie die gebruikers kunnen begrijpen, in plaats van technische foutmeldingen weer te geven zoals ze zijn.

```ts
// Voorbeeld.：Conversie naar gebruiksvriendelijke foutmeldingen
function getErrorMessage(error: any): string {
  if (error.status === 401) {
    return 'Sessie is verlopen. Log opnieuw in.';
  } else if (error.status === 404) {
    return 'De gevraagde bron kon niet worden gevonden.';
  } else if (error.status >= 500) {
    return 'Er is een serverfout opgetreden. Probeer het later opnieuw.';
  }
  return 'Er is een onverwachte fout opgetreden.';
}
```

## Samenvatting.

Foutafhandeling in RxJS is een belangrijk onderdeel van het waarborgen van de robuustheid van de applicatie. Door de juiste combinatie van operatoren zoals `catchError`, `retry` en ` finalize` te gebruiken, kunnen verschillende foutscenario's worden afgehandeld. Ontwerp een uitgebreide strategie voor foutafhandeling om de gebruikerservaring te verbeteren, in plaats van alleen maar fouten op te vangen.

## Gerelateerde secties.

- **[Veelvoorkomende fouten en hoe ermee om te gaan](/nl/guide/anti-patterns/common-mistakes#9-error-grasping)** - Bekijk antipatterns voor foutafhandeling.
- **[Retry en catchError](/nl/guide/error-handling/retry-catch)** - Legt meer gedetailleerd gebruik uit.