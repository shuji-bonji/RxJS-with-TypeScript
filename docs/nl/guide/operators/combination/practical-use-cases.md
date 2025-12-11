---
description: Praktische use cases voor RxJS combinatie-operators (combineLatest, forkJoin, merge, concat, withLatestFrom, etc.) worden uitgelegd. Praktische patronen van het combineren van meerdere Observables zoals formulierinvoervalidatie en API-integratie, parallelle uitvoering van meerdere verzoeken, realtime datasynchronisatie en sequentiële verwerking van streams worden gepresenteerd met TypeScript codevoorbeelden.
---

# Praktische Use Cases

In dit hoofdstuk introduceren we **praktische use cases** die profiteren van RxJS's combinatie-operators.
Verdiep uw begrip door scenario's die nuttig zijn voor daadwerkelijke applicatieontwikkeling, zoals UI-operaties en API-communicatie.

## Formulierinvoervalidatie en API-verzoeken

Een voorbeeld van het valideren van meerdere formulierinvoeren met `combineLatest`.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, debounceTime, startWith } from 'rxjs';

// Maak formulier-UI
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>Gebruikersregistratieformulier:</h3>';
document.body.appendChild(formContainer);

// Naaminvoer
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Naam: ';
formContainer.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.id = 'name';
nameInput.style.marginBottom = '10px';
nameInput.style.marginLeft = '5px';
formContainer.appendChild(nameInput);
formContainer.appendChild(document.createElement('br'));

// E-mailinvoer
const emailLabel = document.createElement('label');
emailLabel.textContent = 'E-mail: ';
formContainer.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.id = 'email';
emailInput.style.marginBottom = '10px';
emailInput.style.marginLeft = '5px';
formContainer.appendChild(emailInput);
formContainer.appendChild(document.createElement('br'));

// Wachtwoordinvoer
const passwordLabel = document.createElement('label');
passwordLabel.textContent = 'Wachtwoord: ';
formContainer.appendChild(passwordLabel);

const passwordInput = document.createElement('input');
passwordInput.type = 'password';
passwordInput.id = 'password';
passwordInput.style.marginLeft = '5px';
formContainer.appendChild(passwordInput);
formContainer.appendChild(document.createElement('br'));

// Verzendknop
const submitButton = document.createElement('button');
submitButton.textContent = 'Registreren';
submitButton.disabled = true;
submitButton.style.marginTop = '15px';
submitButton.style.padding = '8px 16px';
formContainer.appendChild(submitButton);

// Validatiebericht
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Naamvalidatie
const name$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: value.length >= 2,
      error: value.length < 2 ? 'Naam moet minimaal 2 tekens zijn' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Naam moet minimaal 2 tekens zijn',
  })
);

// E-mailvalidatie
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: emailRegex.test(value),
      error: !emailRegex.test(value)
        ? 'Voer een geldig e-mailadres in'
        : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Voer een geldig e-mailadres in',
  })
);

// Wachtwoordvalidatie
const password$ = fromEvent(passwordInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value;
    return {
      value,
      valid: value.length >= 6,
      error: value.length < 6 ? 'Wachtwoord moet minimaal 6 tekens zijn' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Wachtwoord moet minimaal 6 tekens zijn',
  })
);

// Combineer validatiestatus van alle velden
combineLatest([name$, email$, password$])
  .pipe(debounceTime(300))
  .subscribe(([nameState, emailState, passwordState]) => {
    // Controleer of formulier geldig is
    const isFormValid =
      nameState.valid && emailState.valid && passwordState.valid;
    submitButton.disabled = !isFormValid;

    // Toon foutberichten
    if (!isFormValid) {
      const errors = [
        nameState.error,
        emailState.error,
        passwordState.error,
      ].filter((error) => error !== null);

      validationMessage.textContent = errors.join('\n');
    } else {
      validationMessage.textContent = '';
    }
  });

// Verzendknop klikgebeurtenis
fromEvent(submitButton, 'click').subscribe(() => {
  const formData = {
    name: nameInput.value,
    email: emailInput.value,
    password: passwordInput.value,
  };

  // Toon formulierdata (in echt gebruik, verzend naar API)
  const successMessage = document.createElement('div');
  successMessage.textContent = 'Registratie voltooid!';
  successMessage.style.color = 'green';
  successMessage.style.fontWeight = 'bold';
  successMessage.style.marginTop = '10px';
  formContainer.appendChild(successMessage);

  console.log('Verzonden data:', formData);
});

```

## Gelijktijdige verzoeken en laadstatusbeheer

Hier is een voorbeeld van het gebruik van `forkJoin` om meerdere API-verzoeken parallel te verwerken en de resultaten samen te vatten.

```ts
import {
  forkJoin,
  of,
  throwError,
  Observable,
  ObservableInputTuple,
} from 'rxjs';
import { catchError, delay, finalize } from 'rxjs';

// Interface-definities
interface User {
  id: number;
  name: string;
  email: string;
}

interface Post {
  id: number;
  title: string;
  content: string;
}

interface WeatherSuccess {
  city: string;
  temp: number;
  condition: string;
}

interface WeatherError {
  error: string;
}

type Weather = WeatherSuccess | WeatherError;

// Resultaattype-definitie
interface ApiResponse {
  user: User;
  posts: Post[];
  weather: Weather;
}

// Maak UI-elementen
const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>Meerdere API-verzoeken Voorbeeld:</h3>';
document.body.appendChild(apiContainer);

const loadButton = document.createElement('button');
loadButton.textContent = 'Data Laden';
loadButton.style.padding = '8px 16px';
apiContainer.appendChild(loadButton);

const loadingIndicator = document.createElement('div');
loadingIndicator.style.margin = '10px 0';
loadingIndicator.style.display = 'none';
apiContainer.appendChild(loadingIndicator);

const resultContainer = document.createElement('div');
apiContainer.appendChild(resultContainer);

// Simuleer API-verzoeken
function fetchUser(id: number): Observable<User> {
  // Succesvol verzoek
  return of({
    id,
    name: `Gebruiker${id}`,
    email: `gebruiker${id}@example.com`,
  }).pipe(
    delay(2000) // 2 seconden vertraging
  );
}

function fetchPosts(userId: number): Observable<Post[]> {
  // Succesvol verzoek
  return of([
    { id: 1, title: `Bericht 1 van ${userId}`, content: 'Inhoud...' },
    { id: 2, title: `Bericht 2 van ${userId}`, content: 'Inhoud...' },
  ]).pipe(
    delay(1500) // 1,5 seconden vertraging
  );
}

function fetchWeather(city: string): Observable<WeatherSuccess> {
  // Faalt soms
  const shouldFail = Math.random() > 0.7;

  if (shouldFail) {
    return throwError(() => new Error('Ophalen weergegevens mislukt')).pipe(
      delay(1000)
    );
  }

  return of({
    city,
    temp: Math.round(15 + Math.random() * 10),
    condition: ['Zonnig', 'Bewolkt', 'Regenachtig'][Math.floor(Math.random() * 3)],
  }).pipe(
    delay(1000) // 1 seconde vertraging
  );
}

// Voer meerdere verzoeken uit bij knopklik
loadButton.addEventListener('click', () => {
  // Reset UI
  resultContainer.innerHTML = '';
  loadingIndicator.style.display = 'block';
  loadingIndicator.textContent = 'Data laden...';
  loadButton.disabled = true;

  // Voer meerdere API-verzoeken gelijktijdig uit
  forkJoin({
    user: fetchUser(1),
    posts: fetchPosts(1),
    weather: fetchWeather('Tokyo').pipe(
      // Foutafhandeling
      catchError((error: Error) => {
        console.error('Weer API-fout:', error);
        return of<WeatherError>({ error: error.message });
      })
    ),
  } as ObservableInputTuple<ApiResponse>)
    .pipe(
      // Opruimen bij voltooiing
      finalize(() => {
        loadingIndicator.style.display = 'none';
        loadButton.disabled = false;
      })
    )
    .subscribe((results: ApiResponse) => {
      // Toon gebruikersinformatie
      const userInfo = document.createElement('div');
      userInfo.innerHTML = `
      <h4>Gebruikersinformatie</h4>
      <p>Naam: ${results.user.name}</p>
      <p>E-mail: ${results.user.email}</p>
    `;
      userInfo.style.margin = '10px 0';
      userInfo.style.padding = '10px';
      userInfo.style.backgroundColor = '#f0f0f0';
      userInfo.style.borderRadius = '5px';
      resultContainer.appendChild(userInfo);

      // Toon berichten
      const postsInfo = document.createElement('div');
      postsInfo.innerHTML = `
      <h4>Berichten (${results.posts.length})</h4>
      <ul>
        ${results.posts
          .map((post: { title: string }) => `<li>${post.title}</li>`)
          .join('')}
      </ul>
    `;
      postsInfo.style.margin = '10px 0';
      postsInfo.style.padding = '10px';
      postsInfo.style.backgroundColor = '#f0f0f0';
      postsInfo.style.borderRadius = '5px';
      resultContainer.appendChild(postsInfo);

      // Toon weerinformatie
      const weatherInfo = document.createElement('div');

      if ('error' in results.weather) {
        weatherInfo.innerHTML = `
        <h4>Weerinformatie</h4>
        <p style="color: red;">Fout: ${results.weather.error}</p>
      `;
      } else {
        weatherInfo.innerHTML = `
        <h4>Weerinformatie</h4>
        <p>Stad: ${results.weather.city}</p>
        <p>Temperatuur: ${results.weather.temp}°C</p>
        <p>Conditie: ${results.weather.condition}</p>
      `;
      }

      weatherInfo.style.margin = '10px 0';
      weatherInfo.style.padding = '10px';
      weatherInfo.style.backgroundColor = '#f0f0f0';
      weatherInfo.style.borderRadius = '5px';
      resultContainer.appendChild(weatherInfo);
    });
});

```

## Annuleerbare zoekfunctie

Hier is een voorbeeld van het combineren van `withLatestFrom` en `race` om een timeout of annuleerbare zoekfunctie te implementeren.

```ts
import { fromEvent, timer, race, of, EMPTY } from 'rxjs';
import {
  map,
  debounceTime,
  switchMap,
  tap,
  delay,
  catchError,
  takeUntil,
} from 'rxjs';

// Maak zoek-UI
const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Annuleerbaar Zoeken:</h3>';
document.body.appendChild(searchContainer);

// Zoekinvoerveld
const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Voer zoekterm in...';
searchInput.style.padding = '8px';
searchInput.style.width = '250px';
searchContainer.appendChild(searchInput);

// Annuleerknop
const cancelButton = document.createElement('button');
cancelButton.textContent = 'Annuleren';
cancelButton.style.marginLeft = '10px';
cancelButton.style.padding = '8px 16px';
searchContainer.appendChild(cancelButton);

// Zoekresultatengebied
const resultsContainer = document.createElement('div');
resultsContainer.style.marginTop = '10px';
resultsContainer.style.minHeight = '200px';
resultsContainer.style.padding = '10px';
resultsContainer.style.border = '1px solid #ddd';
resultsContainer.style.borderRadius = '5px';
searchContainer.appendChild(resultsContainer);

// Simuleer zoekverzoek
function searchApi(term: string) {
  console.log(`Starten met zoeken naar "${term}"...`);

  // Simuleer zoekresultaten
  return of([
    `Zoekresultaat 1 voor "${term}"`,
    `Zoekresultaat 2 voor "${term}"`,
    `Zoekresultaat 3 voor "${term}"`,
  ]).pipe(
    // Willekeurige vertraging tussen 2-5 seconden
    delay(2000 + Math.random() * 3000),
    // Foutafhandeling
    catchError((err) => {
      console.error('Zoekfout:', err);
      return EMPTY;
    })
  );
}

// Annuleergebeurtenis
const cancel$ = fromEvent(cancelButton, 'click');

// Zoekgebeurtenis
const search$ = fromEvent(searchInput, 'input')
  .pipe(
    // Haal invoerwaarde op
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // Wacht 300ms
    debounceTime(300),
    // Negeer lege zoekopdrachten
    tap((term) => {
      if (term === '') {
        resultsContainer.innerHTML = '<p>Voer een zoekterm in</p>';
      }
    }),
    // Verwerk geen lege zoekopdrachten
    switchMap((term) => {
      if (term === '') {
        return EMPTY;
      }

      // Toon zoekbericht
      resultsContainer.innerHTML = '<p>Zoeken...</p>';

      // Timeout-afhandeling (5 seconden)
      const timeout$ = timer(5000).pipe(
        tap(() => console.log('Zoeken verlopen')),
        map(() => ({ type: 'timeout', results: null }))
      );

      // API-verzoek
      const request$ = searchApi(term).pipe(
        map((results) => ({ type: 'success', results })),
        // Annuleer als annuleerknop wordt ingedrukt
        takeUntil(
          cancel$.pipe(
            tap(() => {
              console.log('Zoeken is geannuleerd');
              resultsContainer.innerHTML = '<p>Zoeken is geannuleerd</p>';
            })
          )
        )
      );

      // Race tussen timeout en voltooiing verzoek
      return race(request$, timeout$);
    })
  )
  .subscribe((response) => {
    if (response.type === 'success') {
      // Zoeken succesvol
      resultsContainer.innerHTML = '<h4>Zoekresultaten:</h4>';

      if (response.results?.length === 0) {
        resultsContainer.innerHTML += '<p>Geen resultaten gevonden</p>';
      } else {
        const list = document.createElement('ul');
        response.results?.forEach((result) => {
          const item = document.createElement('li');
          item.textContent = result;
          list.appendChild(item);
        });
        resultsContainer.appendChild(list);
      }
    } else if (response.type === 'timeout') {
      // Timeout
      resultsContainer.innerHTML =
        '<p style="color: red;">Zoeken verlopen. Probeer het opnieuw.</p>';
    }
  });

```


## Vergelijking en selectiegids voor combinatie-operators

Vergelijk de verschillen tussen meerdere combinatie-operators en help bij het kiezen van de juiste voor uw use case.

| Operator | Timing | Output | Use Case |
|------------|------------|-----|------------|
| `merge` | Gelijktijdige uitvoering | Output in volgorde van optreden | Monitor meerdere brongebeurtenissen tegelijkertijd |
| `concat` | Sequentiële uitvoering | Output in volgorde | Async taken waar volgorde belangrijk is |
| `combineLatest` | Vereist minstens één waarde van alle bronnen | Combinatie van alle laatste waarden | Formulierinvoervalidatie |
| `zip` | Vereist corresponderende indexwaarden van alle bronnen | Combinatie van waarden op index | Synchroniseren van gerelateerde data |
| `withLatestFrom` | Wanneer hoofdbron waarde emitteert | Hoofdwaarde en laatste waarde van andere bronnen | Combineren van hulpdata |
| `forkJoin` | Wanneer alle bronnen voltooien | Laatste waarde van elke bron | Meerdere API-verzoeken |
| `race` | Alleen de eerste bron die emitteert | Alleen waarden van winnende stream | Timeout, annuleringsafhandeling |

### Operatorselectie beslissingsflow

1. **Wilt u waarden van alle bronnen tegelijkertijd ontvangen?**
   - Ja → `merge`
   - Nee → Volgende

2. **Wilt u de volgorde van bronnen behouden?**
   - Ja → `concat`
   - Nee → Volgende

3. **Heeft u een combinatie van de laatste waarden voor elke bron nodig?**
   - Ja → Wanneer combineren?
     - Voor elke nieuwe waarde van enige bron → `combineLatest`
     - Voor elke specifieke hoofdstreamwaarde → `withLatestFrom`
   - Nee → Volgende

4. **Heeft u corresponderende waarden in indexvolgorde nodig?**
   - Ja → `zip`
   - Nee → Volgende

5. **Heeft u resultaten nodig nadat alle bronnen voltooid zijn?**
   - Ja → `forkJoin`
   - Nee → Volgende

6. **Heeft u alleen de snelste van meerdere alternatieve bronnen nodig?**
   - Ja → `race`
   - Nee → Heroverweeg het doel


## Schakelstrategie

Dit is een voorbeeld van het dynamisch schakelen tussen meerdere databronnen.

```ts
import { fromEvent, merge, interval, of } from 'rxjs';
import { map, switchMap, take, tap } from 'rxjs';

// Maak UI-elementen
const switchingContainer = document.createElement('div');
switchingContainer.innerHTML = '<h3>Databron Schakelen:</h3>';
document.body.appendChild(switchingContainer);

// Maak knoppen
const source1Button = document.createElement('button');
source1Button.textContent = 'Bron 1';
source1Button.style.margin = '5px';
source1Button.style.padding = '5px 10px';
switchingContainer.appendChild(source1Button);

const source2Button = document.createElement('button');
source2Button.textContent = 'Bron 2';
source2Button.style.margin = '5px';
source2Button.style.padding = '5px 10px';
switchingContainer.appendChild(source2Button);

const source3Button = document.createElement('button');
source3Button.textContent = 'Bron 3';
source3Button.style.margin = '5px';
source3Button.style.padding = '5px 10px';
switchingContainer.appendChild(source3Button);

// Resultatenweergavegebied
const resultsArea = document.createElement('div');
resultsArea.style.marginTop = '10px';
resultsArea.style.minHeight = '150px';
resultsArea.style.padding = '10px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.backgroundColor = '#f9f9f9';
switchingContainer.appendChild(resultsArea);

// Drie databronnen
function createSource1() {
  return interval(1000).pipe(
    take(5),
    map((val) => `Bron 1: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '#c8e6c9';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource2() {
  return interval(500).pipe(
    take(8),
    map((val) => `Bron 2: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '#bbdefb';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource3() {
  return of('Bron 3: A', 'Bron 3: B', 'Bron 3: C').pipe(
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '#ffccbc';
    })
  );
}

// Knopklikgebeurtenissen
const source1Click$ = fromEvent(source1Button, 'click').pipe(map(() => 1));

const source2Click$ = fromEvent(source2Button, 'click').pipe(map(() => 2));

const source3Click$ = fromEvent(source3Button, 'click').pipe(map(() => 3));

// Voeg knopklikken samen
merge(source1Click$, source2Click$, source3Click$)
  .pipe(
    // Schakel naar geselecteerde bron
    switchMap((sourceId) => {
      // Wis resultatengebied
      resultsArea.innerHTML = '';

      // Retourneer geselecteerde bron
      switch (sourceId) {
        case 1:
          return createSource1();
        case 2:
          return createSource2();
        case 3:
          return createSource3();
        default:
          return of('Geen bron geselecteerd');
      }
    })
  )
  .subscribe((value) => {
    // Toon resultaat
    const item = document.createElement('div');
    item.textContent = value;
    item.style.padding = '5px';
    item.style.margin = '2px 0';
    item.style.backgroundColor = 'white';
    item.style.borderRadius = '3px';
    resultsArea.appendChild(item);
  });

// Initieel bericht
const initialMessage = document.createElement('div');
initialMessage.textContent =
  'Klik op een knop om een databron te selecteren';
initialMessage.style.color = '#666';
resultsArea.appendChild(initialMessage);

```

## Voorwaardelijke samenvoeging

Dit is een voorbeeld van het combineren van `merge` en `filter` om databronnen te selecteren op basis van voorwaarden.

```ts
import { merge, interval, fromEvent } from 'rxjs';
import {
  map,
  filter,
  takeUntil,
  withLatestFrom,
  startWith,
} from 'rxjs';

// Maak UI-elementen
const conditionalContainer = document.createElement('div');
conditionalContainer.innerHTML = '<h3>Voorwaardelijke Samenvoeging:</h3>';
document.body.appendChild(conditionalContainer);

// Filterinstellingen
const filterDiv = document.createElement('div');
filterDiv.style.marginBottom = '10px';
conditionalContainer.appendChild(filterDiv);

// Maak checkboxen
const slowCheck = document.createElement('input');
slowCheck.type = 'checkbox';
slowCheck.id = 'slowCheck';
slowCheck.checked = true;
filterDiv.appendChild(slowCheck);

const slowLabel = document.createElement('label');
slowLabel.htmlFor = 'slowCheck';
slowLabel.textContent = 'Langzame Bron';
slowLabel.style.marginRight = '15px';
filterDiv.appendChild(slowLabel);

const fastCheck = document.createElement('input');
fastCheck.type = 'checkbox';
fastCheck.id = 'fastCheck';
fastCheck.checked = true;
filterDiv.appendChild(fastCheck);

const fastLabel = document.createElement('label');
fastLabel.htmlFor = 'fastCheck';
fastLabel.textContent = 'Snelle Bron';
fastLabel.style.marginRight = '15px';
filterDiv.appendChild(fastLabel);

const clickCheck = document.createElement('input');
clickCheck.type = 'checkbox';
clickCheck.id = 'clickCheck';
clickCheck.checked = true;
filterDiv.appendChild(clickCheck);

const clickLabel = document.createElement('label');
clickLabel.htmlFor = 'clickCheck';
clickLabel.textContent = 'Klikgebeurtenissen';
filterDiv.appendChild(clickLabel);

// Stopknop
const stopButton = document.createElement('button');
stopButton.textContent = 'Stop';
stopButton.style.marginLeft = '15px';
filterDiv.appendChild(stopButton);

// Resultatenweergavegebied
const conditionalResults = document.createElement('div');
conditionalResults.style.height = '200px';
conditionalResults.style.overflowY = 'auto';
conditionalResults.style.padding = '10px';
conditionalResults.style.border = '1px solid #ddd';
conditionalResults.style.backgroundColor = '#f9f9f9';
conditionalContainer.appendChild(conditionalResults);

// Drie databronnen
// 1. Langzame bron (elke 1 seconde)
const slow$ = interval(1000).pipe(map((val) => ({ type: 'slow', value: val })));

// 2. Snelle bron (elke 300 milliseconden)
const fast$ = interval(300).pipe(map((val) => ({ type: 'fast', value: val })));

// 3. Klikgebeurtenissen
const click$ = fromEvent(document.body, 'click').pipe(
  map((event) => ({
    type: 'click',
    value: {
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY,
    },
  }))
);

// Monitor checkboxstatus
const slowEnabled$ = fromEvent(slowCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

const fastEnabled$ = fromEvent(fastCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

const clickEnabled$ = fromEvent(clickCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// Stopgebeurtenis
const stop$ = fromEvent(stopButton, 'click');

// Voorwaardelijke samenvoeging
merge(
  // Combineer langzame bron met ingeschakelde status
  slow$.pipe(
    withLatestFrom(slowEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Combineer snelle bron met ingeschakelde status
  fast$.pipe(
    withLatestFrom(fastEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Combineer klikbron met ingeschakelde status
  click$.pipe(
    withLatestFrom(clickEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  )
)
  .pipe(takeUntil(stop$))
  .subscribe((event) => {
    // Toon resultaat
    const item = document.createElement('div');

    switch (event.type) {
      case 'slow':
        item.textContent = `Langzame Bron: ${event.value}`;
        item.style.color = '#1b5e20';
        break;
      case 'fast':
        item.textContent = `Snelle Bron: ${event.value}`;
        item.style.color = '#0d47a1';
        break;
      case 'click':
        const clickValue = event.value as { x: number; y: number };
        item.textContent = `Klik: X=${clickValue.x}, Y=${clickValue.y}`;
        item.style.color = '#bf360c';
        break;
    }

    item.style.padding = '3px';
    item.style.margin = '2px 0';
    conditionalResults.prepend(item); // Toon nieuwste bovenaan
  });

```

## Samenvatting van het kiezen van combinatie-operators

| Doel | Operator | Kenmerken |
|------|--------------|------|
| Altijd meerdere laatste waarden synchroniseren | `combineLatest` | Combineer altijd laatste waarde van elke Observable |
| Alles samen ophalen na voltooiing | `forkJoin` | Geef alleen laatste waarde uit (eenmaal) |
| Synchroon in volgorde verwerken | `zip` | Combineer één van elke Observable en geef uit |
| Andere laatste waarden raadplegen bij trigger | `withLatestFrom` | Voeg laatste waarde van secundaire stream toe wanneer hoofdstream emitteert |

## Samenvatting

Combinatie-operators zijn krachtige tools voor het combineren van meerdere databronnen tot een enkele stream. Door de juiste operator te selecteren, kunnen complexe asynchrone datastromen beknopt en declaratief worden uitgedrukt.

### Kernpunten voor het beheersen van combinatie-operators

1. **Selecteer de juiste operator voor uw use case**: Elke operator is geoptimaliseerd voor een specifieke use case. Kies de juiste operator voor uw doel.
2. **Begrijp wanneer te emitteren**: Het gedrag van combinatie-operators is sterk afhankelijk van wanneer waarden worden geëmitteerd. Het is belangrijk om de emissietiming van elke operator te begrijpen.
3. **Overwegingen voor foutafhandeling**: Overweeg het gedrag wanneer een fout optreedt in een deel van de gecombineerde stream (of het geheel faalt of gedeeltelijk doorgaat).
4. **Ken voltooiingsvoorwaarden**: Het is ook belangrijk om te begrijpen wanneer de gecombineerde stream zal voltooien, en om deze expliciet te voltooien met `takeUntil` of vergelijkbaar indien nodig.
5. **Profiteer van typeveiligheid**: TypeScript stelt u in staat om combinatie-operators op een typeveilige manier te hanteren. Het typevoordeel is vooral groot voor complexe combinaties.

Combinatie-operators kunnen worden benut in veel praktische scenario's zoals UI-gebeurtenisafhandeling, meerdere API-verzoeken, formuliervalidatie, etc. Het beheersen van deze operators zal u helpen de ware kracht van reactief programmeren in RxJS te ontsluiten.

---
Laten we vervolgens naar [Foutafhandeling](/nl/guide/error-handling/strategies) gaan om te leren hoe u robuustere RxJS-code schrijft!
