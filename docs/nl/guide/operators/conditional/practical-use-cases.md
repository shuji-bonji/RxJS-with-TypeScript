---
description: Praktische use cases van RxJS voorwaardelijke operators (iif, defer) worden uitgelegd. Inclusief API fallback-verwerking, cache-strategieën, dynamische gegevensbron selectie en voorwaardelijke lazy evaluatie. Specifieke toepassingspatronen voor scenario's die dynamische verwerkingsvertakkingen vereisen, worden geïntroduceerd met TypeScript code voorbeelden. Implementatiepatronen die direct kunnen worden toegepast in daadwerkelijke applicatieontwikkeling.
---

# Praktische Use Cases

Door voorwaardelijke operators van RxJS te gebruiken, wordt het mogelijk om stream-vertakkingen en schakelingen uit te voeren op basis van dynamische status.
In dit hoofdstuk kunt u de toepassingspatronen van elke operator ervaren via daadwerkelijk werkende code met UI.

## Selectie van Verschillende Gegevensbronnen op Basis van Voorwaarden

```ts
import { iif, of, EMPTY } from 'rxjs';
import { switchMap, tap, catchError, retry } from 'rxjs';

// UI aanmaken
const appContainer = document.createElement('div');
appContainer.innerHTML = '<h3>Gegevensbron selectie app:</h3>';
document.body.appendChild(appContainer);

// Optie selectie
const optionsDiv = document.createElement('div');
optionsDiv.style.marginBottom = '15px';
appContainer.appendChild(optionsDiv);

// Checkbox (offline modus)
const offlineCheck = document.createElement('input');
offlineCheck.type = 'checkbox';
offlineCheck.id = 'offlineMode';
optionsDiv.appendChild(offlineCheck);

const offlineLabel = document.createElement('label');
offlineLabel.htmlFor = 'offlineMode';
offlineLabel.textContent = 'Offline modus';
offlineLabel.style.marginLeft = '5px';
optionsDiv.appendChild(offlineLabel);

// Zoek-ID invoer
const idInput = document.createElement('input');
idInput.type = 'number';
idInput.placeholder = 'ID (1-10)';
idInput.min = '1';
idInput.max = '10';
idInput.value = '1';
idInput.style.marginLeft = '15px';
idInput.style.width = '80px';
optionsDiv.appendChild(idInput);

// Zoekknop
const searchButton = document.createElement('button');
searchButton.textContent = 'Zoeken';
searchButton.style.marginLeft = '10px';
optionsDiv.appendChild(searchButton);

// Resultatengebied
const resultsArea = document.createElement('div');
resultsArea.style.padding = '15px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.borderRadius = '5px';
resultsArea.style.backgroundColor = '#f9f9f9';
resultsArea.style.minHeight = '150px';
appContainer.appendChild(resultsArea);

type User = {
  lastUpdated?: Date;
  fromCache?: boolean;
  id: number;
  name: string;
  email: string;
};
type ErrorResult = {
  error: boolean;
  message: string;
};

// Offline gegevens (cache)
const cachedData: Record<number, User> = {
  1: { id: 1, name: 'Jan Jansen', email: 'jan@example.com' },
  2: { id: 2, name: 'Pieter Pietersen', email: 'pieter@example.com' },
  3: { id: 3, name: 'Klaas Klaassen', email: 'klaas@example.com' },
};

// Daadwerkelijk gegevens ophalen van online API (JSONPlaceholder)
function fetchUserFromApi(id: number) {
  console.log(`Gebruiker ID ${id} ophalen van API...`);

  // Daadwerkelijk API endpoint
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`HTTP fout: ${response.status}`);
        }
        return response.json();
      })
    ),
    tap(() => console.log('API aanroep geslaagd')),
    catchError((err) => {
      console.error('API aanroep mislukt:', err);
      throw new Error('API verzoek mislukt');
    })
  );
}

// Gebruiker ophalen uit cache
function getUserFromCache(id: number) {
  console.log(`Gebruiker ID ${id} ophalen uit cache...`);

  return iif(
    () => id in cachedData,
    of({ ...cachedData[id], fromCache: true }),
    EMPTY.pipe(
      tap(() => {
        throw new Error('Gebruiker niet gevonden in cache');
      })
    )
  );
}

// Zoekknop klik
searchButton.addEventListener('click', () => {
  const id = parseInt(idInput.value, 10);
  const isOffline = offlineCheck.checked;

  // Invoer validatie
  if (isNaN(id) || id < 1 || id > 10) {
    resultsArea.innerHTML =
      '<p style="color: red;">Voer een geldige ID (1-10) in</p>';
    return;
  }

  // Laadweergave
  resultsArea.innerHTML = '<p>Gegevens ophalen...</p>';

  // Gegevensbron selecteren op basis van offline modus
  iif(
    () => isOffline,
    getUserFromCache(id).pipe(
      catchError((err) => {
        console.error('Cache fout:', err);
        return of({ error: err.message });
      })
    ),
    fetchUserFromApi(id).pipe(
      retry(2), // Maximaal 2 keer opnieuw proberen
      catchError((err) => {
        console.error('API fout:', err);

        // Als API mislukt, gebruik cache als fallback
        return getUserFromCache(id).pipe(
          catchError(() =>
            of({ error: 'Zowel online API als cache mislukt' })
          )
        );
      })
    )
  ).subscribe({
    next: (result: any) => {
      if ('error' in result) {
        resultsArea.innerHTML = `<p style="color: red;">Fout: ${result.message}</p>`;
      } else {
        const source = result.fromCache
          ? '<span style="color: orange;">(van cache)</span>'
          : '<span style="color: green;">(van API)</span>';

        resultsArea.innerHTML = `
          <h4>Gebruikersinformatie ${source}</h4>
          <p><strong>ID:</strong> ${result.id}</p>
          <p><strong>Naam:</strong> ${result.name}</p>
          <p><strong>E-mail:</strong> ${result.email}</p>
          ${
            result.lastUpdated
              ? `<p><small>Laatst bijgewerkt: ${new Date(
                  result.lastUpdated
                ).toLocaleString()}</small></p>`
              : ''
          }
        `;
      }
    },
    error: (err) => {
      resultsArea.innerHTML = `<p style="color: red;">Fout: ${err.message}</p>`;
    },
  });
});

// Initieel bericht
resultsArea.innerHTML = '<p>Klik op de knop om gegevens op te halen</p>';


```



## Runtime Vertakking en Fallback Strategieën

In dit voorbeeld met `iif` schakelt u dynamisch tussen gegevensbronnen "offline cache" en "online API" op basis van gebruikersacties of status.
Door `catchError` en `retry` te combineren, kunt u ook retry's bij mislukkingen en fallback-bestemmingen definiëren.

Het is vooral geschikt voor de volgende use cases.

- Offline ondersteuning in omgevingen met onstabiel netwerk
- Schakelen tussen cache-gebruik en online verzoeken
- Automatische retry's bij API-fouten en overschakelen naar alternatieve routes

## Performance Optimalisatie Patronen

In complexere scenario's kunt u geoptimaliseerde gegevensophalings patronen implementeren door voorwaardelijke operators te combineren.

```ts
import { fromEvent, Observable, of, throwError, timer } from 'rxjs';
import {
  switchMap,
  catchError,
  map,
  tap,
  debounceTime,
  distinctUntilChanged,
  withLatestFrom,
  delay,
  startWith,
} from 'rxjs';

// UI-elementen aanmaken
const optimizationContainer = document.createElement('div');
optimizationContainer.innerHTML = '<h3>Geavanceerde voorwaardelijke gegevensophaling:</h3>';
document.body.appendChild(optimizationContainer);

// Zoek UI
const searchInputGroup = document.createElement('div');
searchInputGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(searchInputGroup);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Voer gebruiker ID in (1-10)';
searchInput.value = '1';
searchInput.style.padding = '8px';
searchInput.style.width = '180px';
searchInputGroup.appendChild(searchInput);

const searchButton = document.createElement('button');
searchButton.textContent = 'Zoeken';
searchButton.style.marginLeft = '10px';
searchButton.style.padding = '8px 16px';
searchInputGroup.appendChild(searchButton);

// Opties instellingen
const optionsGroup = document.createElement('div');
optionsGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(optionsGroup);

const cacheCheckbox = document.createElement('input');
cacheCheckbox.type = 'checkbox';
cacheCheckbox.id = 'useCache';
cacheCheckbox.checked = true;
optionsGroup.appendChild(cacheCheckbox);

const cacheLabel = document.createElement('label');
cacheLabel.htmlFor = 'useCache';
cacheLabel.textContent = 'Cache gebruiken';
cacheLabel.style.marginRight = '15px';
optionsGroup.appendChild(cacheLabel);

const forceCheckbox = document.createElement('input');
forceCheckbox.type = 'checkbox';
forceCheckbox.id = 'forceRefresh';
optionsGroup.appendChild(forceCheckbox);

const forceLabel = document.createElement('label');
forceLabel.htmlFor = 'forceRefresh';
forceLabel.textContent = 'Geforceerd opnieuw ophalen';
optionsGroup.appendChild(forceLabel);

// Resultaten weergavegebied
const optimizedResults = document.createElement('div');
optimizedResults.style.padding = '15px';
optimizedResults.style.border = '1px solid #ddd';
optimizedResults.style.borderRadius = '5px';
optimizedResults.style.minHeight = '150px';
optimizedResults.style.backgroundColor = '#f9f9f9';
optimizationContainer.appendChild(optimizedResults);

// Cache beheer
const cache = new Map<string, { data: any; timestamp: number }>();
const CACHE_EXPIRY = 30000; // 30 seconden

// Gebruikersgegevens ophalen van daadwerkelijke API (JSONPlaceholder)
function fetchUserData(id: string, forceRefresh: boolean): Observable<any> {
  // Ongeldig ID
  if (!id || isNaN(Number(id)) || Number(id) < 1 || Number(id) > 10) {
    return throwError(
      () => new Error('Ongeldig gebruikers-ID: voer een getal van 1~10 in')
    );
  }

  const cacheKey = `user-${id}`;
  const cachedItem = cache.get(cacheKey);
  const now = Date.now();

  // Cache controle (binnen termijn en niet geforceerd opnieuw ophalen)
  if (
    !forceRefresh &&
    cachedItem &&
    now - cachedItem.timestamp < CACHE_EXPIRY
  ) {
    console.log(`Ophalen uit cache: ${id}`);
    return of({
      ...cachedItem.data,
      fromCache: true,
    }).pipe(delay(100)); // Simuleer snelle respons
  }

  // Daadwerkelijk API verzoek (JSONPlaceholder)
  console.log(`Gegevens ophalen van API: ${id}`);
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`HTTP fout: ${response.status}`);
        }
        return response.json();
      })
    ),
    map((userData) => {
      const processedData = {
        id: userData.id,
        name: userData.name,
        email: userData.email,
        lastUpdated: now,
        fromCache: false,
      };

      // Opslaan in cache
      cache.set(cacheKey, {
        data: processedData,
        timestamp: now,
      });

      return processedData;
    }),
    catchError((err) => {
      console.error('API fout:', err);
      throw new Error('API verzoek mislukt');
    })
  );
}

// Zoekvoorwaarde wijzigingen monitoren
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map((event) => (event.target as HTMLInputElement).value.trim()),
  debounceTime(300),
  distinctUntilChanged()
);

// Cache instellingen wijzigingen monitoren
const useCache$ = fromEvent(cacheCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// Geforceerd opnieuw ophalen wijzigingen monitoren
const forceRefresh$ = fromEvent(forceCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(false)
);

// Zoekknop klik event
const searchClick$ = fromEvent(searchButton, 'click');

// Zoeken uitvoeren
searchClick$
  .pipe(
    // Huidige invoerwaarde, cache instelling, geforceerd opnieuw ophalen instelling ophalen
    withLatestFrom(
      searchTerm$,
      useCache$,
      forceRefresh$,
      (_, term, useCache, forceRefresh) => ({
        term,
        useCache,
        forceRefresh,
      })
    ),
    tap(() => {
      // Zoeken starten weergave
      optimizedResults.innerHTML = '<p>Zoeken...</p>';
    }),
    // Voorwaardelijke stream met iif()
    switchMap(({ term, useCache, forceRefresh }) => {
      // Als zoekterm leeg is
      if (!term) {
        return of({ error: 'Voer een zoekterm in' });
      }

      // Als cache is uitgeschakeld
      if (!useCache) {
        return fetchUserData(term, true);
      }

      // Normale zoekopdracht (cache gebruiken & geforceerd opnieuw ophalen indien nodig)
      return fetchUserData(term, forceRefresh);
    }),
    // Foutafhandeling
    catchError((err) => {
      return of({ error: err.message });
    })
  )
  .subscribe({
    next: (result) => {
      if ('error' in result) {
        // Fout weergeven
        optimizedResults.innerHTML = `
        <p style="color: red;">Fout: ${result.error}</p>
      `;
      } else {
        // Gegevens weergeven
        const source = result.fromCache
          ? '<span style="color: orange;">(van cache)</span>'
          : '<span style="color: green;">(van API)</span>';

        optimizedResults.innerHTML = `
        <h4>Gebruikersinformatie ${source}</h4>
        <p><strong>ID:</strong> ${result.id}</p>
        <p><strong>Naam:</strong> ${result.name}</p>
        <p><strong>E-mail:</strong> ${result.email}</p>
        ${
          result.lastUpdated
            ? `<p><small>Laatst bijgewerkt: ${new Date(
                result.lastUpdated
              ).toLocaleString()}</small></p>`
            : ''
        }
      `;
      }
    },
  });

// Initieel bericht
optimizedResults.innerHTML =
  '<p>Voer gebruiker ID in en klik op de zoekknop</p>';

```


---

## Operator Selectie Gids

Voorwaardelijke operators lijken vaak op elkaar en kunnen verwarrend zijn, maar elk heeft een duidelijk toepassingsdoel.
Hieronder volgt een typische beslissingsstroom en vergelijking van kenmerken.

## Hoe Voorwaardelijke Operators te Kiezen

| Operator | Use Case | Kenmerken |
|----------|----------|----------|
| `iif` | Selecteer één stream tijdens runtime | Kies één uit twee opties op basis van voorwaarde |
| `partition` | Verdeel stream in twee op basis van voorwaarde | Verdeel bronstream in True/False op voorwaarde |
| `throwIfEmpty` | Detecteer lege stream | Gooi fout als geen enkele waarde wordt uitgezonden |
| `defaultIfEmpty` | Gebruik standaardwaarde als leeg | Bied fallback-waarde wanneer stream leeg is |

### Selectie Beslissingsstroom

1. **Zijn er twee opties?**
   - Ja → Gebruik `iif`
   - Nee → Ga verder

2. **Wilt u een stream splitsen?**
   - Ja → Gebruik `partition`
   - Nee → Ga verder

3. **Wilt u omgaan met lege stream?**
   - Ja → Wilt u lege stream als fout behandelen?
     - Ja → `throwIfEmpty`
     - Nee → `defaultIfEmpty`
   - Nee → Ga verder

4. **Wilt u waarden filteren op basis van voorwaarde?**
   - Ja → Gebruik `filter` operator (basis filter operator)
   - Nee → Heroverweeg doel

## Samenvatting

Voorwaardelijke operators zijn krachtige tools voor het besturen van stream flows en het vertakken van verwerking op basis van specifieke voorwaarden. De belangrijkste punten zijn als volgt:

1. **Reactieve flows op basis van beslissingen**: Door voorwaardelijke operators te gebruiken, kunt u verwerking dynamisch wijzigen op basis van event- of gegevensstatus.
2. **Verbeterde foutafhandeling**: Voorwaardelijke operators fungeren als een belangrijk onderdeel van foutafhandelingsstrategieën en maken graceful reacties op uitzonderingscases mogelijk.
3. **Optimalisatie kansen**: Door voorwaardelijke uitvoering kunt u onnodige verwerking vermijden en vooral kostbare operaties zoals netwerk verzoeken en hardwaretoegang optimaliseren.
4. **Complexe applicatie flows**: Door meerdere voorwaardelijke operators te combineren, kunt u complexe bedrijfslogica en statusbeheer declaratief uitdrukken.

Voorwaardelijke operators zijn bijzonder waardevol bij het implementeren van foutafhandeling, cache-strategieën, fallback-mechanismen en voorwaardelijke uitvoeringspatronen met RxJS. Door ze te combineren met andere operators, kunt u complexe applicatie flows bouwen op een declaratieve en type-veilige manier.
