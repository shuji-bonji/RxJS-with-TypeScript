---
description: Erläutert praktische Anwendungsfälle für RxJS-Conditional-Operatoren (iif, defer). API-Fallback-Verarbeitung, Cache-Strategien, dynamische Datenquellenauswahl, bedingte verzögerte Auswertung und andere konkrete Anwendungsmuster für Szenarien, die dynamische Verarbeitungsverzweigung erfordern, mit TypeScript-Codebeispielen. Lernen Sie Implementierungsmuster, die sofort in der tatsächlichen Anwendungsentwicklung angewendet werden können.
---

# Praktische Anwendungsfälle

Durch die Verwendung von RxJS-Conditional-Operatoren können Verzweigungen und Umschaltungen von Streams je nach dynamischem Zustand durchgeführt werden.
In diesem Kapitel können Sie durch tatsächlich funktionierenden Code mit UI die Anwendungsmuster jedes Operators erleben.

## Auswahl verschiedener Datenquellen basierend auf Bedingungen

```ts
import { iif, of, EMPTY } from 'rxjs';
import { switchMap, tap, catchError, retry } from 'rxjs';

// UI erstellen
const appContainer = document.createElement('div');
appContainer.innerHTML = '<h3>Datenquellenauswahl-App:</h3>';
document.body.appendChild(appContainer);

// Optionsauswahl
const optionsDiv = document.createElement('div');
optionsDiv.style.marginBottom = '15px';
appContainer.appendChild(optionsDiv);

// Checkbox (Offline-Modus)
const offlineCheck = document.createElement('input');
offlineCheck.type = 'checkbox';
offlineCheck.id = 'offlineMode';
optionsDiv.appendChild(offlineCheck);

const offlineLabel = document.createElement('label');
offlineLabel.htmlFor = 'offlineMode';
offlineLabel.textContent = 'Offline-Modus';
offlineLabel.style.marginLeft = '5px';
optionsDiv.appendChild(offlineLabel);

// Such-ID-Eingabe
const idInput = document.createElement('input');
idInput.type = 'number';
idInput.placeholder = 'ID (1-10)';
idInput.min = '1';
idInput.max = '10';
idInput.value = '1';
idInput.style.marginLeft = '15px';
idInput.style.width = '80px';
optionsDiv.appendChild(idInput);

// Suchschaltfläche
const searchButton = document.createElement('button');
searchButton.textContent = 'Suchen';
searchButton.style.marginLeft = '10px';
optionsDiv.appendChild(searchButton);

// Ergebnisbereich
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

// Offline-Daten (Cache)
const cachedData: Record<number, User> = {
  1: { id: 1, name: 'Taro Yamada', email: 'yamada@example.com' },
  2: { id: 2, name: 'Hanako Sato', email: 'sato@example.com' },
  3: { id: 3, name: 'Ichiro Suzuki', email: 'suzuki@example.com' },
};

// Tatsächliches Abrufen von Daten aus Online-API (JSONPlaceholder)
function fetchUserFromApi(id: number) {
  console.log(`Abrufen von Benutzer-ID ${id} von API...`);

  // Tatsächlicher API-Endpunkt
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`HTTP-Fehler: ${response.status}`);
        }
        return response.json();
      })
    ),
    tap(() => console.log('API-Aufruf erfolgreich')),
    catchError((err) => {
      console.error('API-Aufruf fehlgeschlagen:', err);
      throw new Error('API-Anfrage fehlgeschlagen');
    })
  );
}

// Benutzer aus Cache abrufen
function getUserFromCache(id: number) {
  console.log(`Abrufen von Benutzer-ID ${id} aus Cache...`);

  return iif(
    () => id in cachedData,
    of({ ...cachedData[id], fromCache: true }),
    EMPTY.pipe(
      tap(() => {
        throw new Error('Benutzer nicht im Cache gefunden');
      })
    )
  );
}

// Suchschaltflächen-Klick
searchButton.addEventListener('click', () => {
  const id = parseInt(idInput.value, 10);
  const isOffline = offlineCheck.checked;

  // Eingabevalidierung
  if (isNaN(id) || id < 1 || id > 10) {
    resultsArea.innerHTML =
      '<p style="color: red;">Bitte geben Sie eine gültige ID (1-10) ein</p>';
    return;
  }

  // Ladeanzeige
  resultsArea.innerHTML = '<p>Daten werden abgerufen...</p>';

  // Datenquelle basierend auf Offline-Modus auswählen
  iif(
    () => isOffline,
    getUserFromCache(id).pipe(
      catchError((err) => {
        console.error('Cache-Fehler:', err);
        return of({ error: err.message });
      })
    ),
    fetchUserFromApi(id).pipe(
      retry(2), // Maximal 2 Wiederholungsversuche
      catchError((err) => {
        console.error('API-Fehler:', err);

        // Bei API-Fehler Cache als Fallback verwenden
        return getUserFromCache(id).pipe(
          catchError(() =>
            of({ error: 'Sowohl Online-API als auch Cache sind fehlgeschlagen' })
          )
        );
      })
    )
  ).subscribe({
    next: (result: any) => {
      if ('error' in result) {
        resultsArea.innerHTML = `<p style="color: red;">Fehler: ${result.message}</p>`;
      } else {
        const source = result.fromCache
          ? '<span style="color: orange;">(aus Cache)</span>'
          : '<span style="color: green;">(von API)</span>';

        resultsArea.innerHTML = `
          <h4>Benutzerinformationen ${source}</h4>
          <p><strong>ID:</strong> ${result.id}</p>
          <p><strong>Name:</strong> ${result.name}</p>
          <p><strong>E-Mail:</strong> ${result.email}</p>
          ${
            result.lastUpdated
              ? `<p><small>Zuletzt aktualisiert: ${new Date(
                  result.lastUpdated
                ).toLocaleString()}</small></p>`
              : ''
          }
        `;
      }
    },
    error: (err) => {
      resultsArea.innerHTML = `<p style="color: red;">Fehler: ${err.message}</p>`;
    },
  });
});

// Anfangsmeldung
resultsArea.innerHTML = '<p>Klicken Sie auf die Schaltfläche, um Daten abzurufen</p>';


```



## Laufzeitverzweigung und Fallback-Strategien

In diesem Beispiel mit `iif` wird die Datenquelle dynamisch zwischen "Offline-Cache" und "Online-API" basierend auf Benutzeraktionen und Zustand umgeschaltet.
Außerdem können durch die Kombination von `catchError` und `retry` auch Wiederholungen bei Fehlern und Definitionen von Fallback-Zielen durchgeführt werden.

Es eignet sich besonders für folgende Anwendungsfälle:

- Offline-Unterstützung in Umgebungen mit instabilem Netzwerk
- Umschaltung zwischen Cache-Nutzung und Online-Anfragen
- Automatische Wiederholung bei API-Fehlern und Umschaltung auf alternative Routen

## Leistungsoptimierungsmuster

In komplexeren Szenarien können optimierte Datenabrufmuster durch Kombination von Conditional-Operatoren implementiert werden.

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

// UI-Elemente erstellen
const optimizationContainer = document.createElement('div');
optimizationContainer.innerHTML = '<h3>Erweitertes bedingtes Datenabrufen:</h3>';
document.body.appendChild(optimizationContainer);

// Such-UI
const searchInputGroup = document.createElement('div');
searchInputGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(searchInputGroup);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Benutzer-ID eingeben (1-10)';
searchInput.value = '1';
searchInput.style.padding = '8px';
searchInput.style.width = '180px';
searchInputGroup.appendChild(searchInput);

const searchButton = document.createElement('button');
searchButton.textContent = 'Suchen';
searchButton.style.marginLeft = '10px';
searchButton.style.padding = '8px 16px';
searchInputGroup.appendChild(searchButton);

// Optionseinstellungen
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
cacheLabel.textContent = 'Cache verwenden';
cacheLabel.style.marginRight = '15px';
optionsGroup.appendChild(cacheLabel);

const forceCheckbox = document.createElement('input');
forceCheckbox.type = 'checkbox';
forceCheckbox.id = 'forceRefresh';
optionsGroup.appendChild(forceCheckbox);

const forceLabel = document.createElement('label');
forceLabel.htmlFor = 'forceRefresh';
forceLabel.textContent = 'Erneutes Abrufen erzwingen';
optionsGroup.appendChild(forceLabel);

// Ergebnisanzeigebereich
const optimizedResults = document.createElement('div');
optimizedResults.style.padding = '15px';
optimizedResults.style.border = '1px solid #ddd';
optimizedResults.style.borderRadius = '5px';
optimizedResults.style.minHeight = '150px';
optimizedResults.style.backgroundColor = '#f9f9f9';
optimizationContainer.appendChild(optimizedResults);

// Cache-Verwaltung
const cache = new Map<string, { data: any; timestamp: number }>();
const CACHE_EXPIRY = 30000; // 30 Sekunden

// Benutzerdaten tatsächlich von API abrufen (JSONPlaceholder)
function fetchUserData(id: string, forceRefresh: boolean): Observable<any> {
  // Ungültige ID
  if (!id || isNaN(Number(id)) || Number(id) < 1 || Number(id) > 10) {
    return throwError(
      () => new Error('Ungültige Benutzer-ID: Bitte geben Sie einen Wert zwischen 1 und 10 ein')
    );
  }

  const cacheKey = `user-${id}`;
  const cachedItem = cache.get(cacheKey);
  const now = Date.now();

  // Cache-Prüfung (innerhalb der Gültigkeitsdauer und nicht erzwungenes erneutes Abrufen)
  if (
    !forceRefresh &&
    cachedItem &&
    now - cachedItem.timestamp < CACHE_EXPIRY
  ) {
    console.log(`Aus Cache abgerufen: ${id}`);
    return of({
      ...cachedItem.data,
      fromCache: true,
    }).pipe(delay(100)); // Schnelle Antwort simulieren
  }

  // Tatsächliche API-Anfrage (JSONPlaceholder)
  console.log(`Datenabruf von API: ${id}`);
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`HTTP-Fehler: ${response.status}`);
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

      // Im Cache speichern
      cache.set(cacheKey, {
        data: processedData,
        timestamp: now,
      });

      return processedData;
    }),
    catchError((err) => {
      console.error('API-Fehler:', err);
      throw new Error('API-Anfrage fehlgeschlagen');
    })
  );
}

// Änderungen der Suchbedingungen überwachen
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map((event) => (event.target as HTMLInputElement).value.trim()),
  debounceTime(300),
  distinctUntilChanged()
);

// Änderungen der Cache-Einstellung überwachen
const useCache$ = fromEvent(cacheCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// Änderungen des erzwungenen erneuten Abrufens überwachen
const forceRefresh$ = fromEvent(forceCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(false)
);

// Klickereignis der Suchschaltfläche
const searchClick$ = fromEvent(searchButton, 'click');

// Suche ausführen
searchClick$
  .pipe(
    // Aktuellen Eingabewert, Cache-Einstellung und Einstellung für erzwungenes erneutes Abrufen abrufen
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
      // Suchstartanzeige
      optimizedResults.innerHTML = '<p>Suche läuft...</p>';
    }),
    // Bedingter Stream mit iif()
    switchMap(({ term, useCache, forceRefresh }) => {
      // Wenn Suchbegriff leer ist
      if (!term) {
        return of({ error: 'Bitte geben Sie einen Suchbegriff ein' });
      }

      // Wenn Cache deaktiviert ist
      if (!useCache) {
        return fetchUserData(term, true);
      }

      // Normale Suche (Cache verwenden & erzwungenes erneutes Abrufen bei Bedarf)
      return fetchUserData(term, forceRefresh);
    }),
    // Fehlerbehandlung
    catchError((err) => {
      return of({ error: err.message });
    })
  )
  .subscribe({
    next: (result) => {
      if ('error' in result) {
        // Fehleranzeige
        optimizedResults.innerHTML = `
        <p style="color: red;">Fehler: ${result.error}</p>
      `;
      } else {
        // Datenanzeige
        const source = result.fromCache
          ? '<span style="color: orange;">(aus Cache)</span>'
          : '<span style="color: green;">(von API)</span>';

        optimizedResults.innerHTML = `
        <h4>Benutzerinformationen ${source}</h4>
        <p><strong>ID:</strong> ${result.id}</p>
        <p><strong>Name:</strong> ${result.name}</p>
        <p><strong>E-Mail:</strong> ${result.email}</p>
        ${
          result.lastUpdated
            ? `<p><small>Zuletzt aktualisiert: ${new Date(
                result.lastUpdated
              ).toLocaleString()}</small></p>`
            : ''
        }
      `;
      }
    },
  });

// Anfangsmeldung
optimizedResults.innerHTML =
  '<p>Geben Sie eine Benutzer-ID ein und klicken Sie auf die Suchschaltfläche</p>';

```


---

## Leitfaden zur Operatorauswahl

Conditional-Operatoren sehen sich oft ähnlich und können verwirrend sein, aber jeder hat einen klaren Anwendungszweck.
Im Folgenden finden Sie einen typischen Entscheidungsablauf und einen Vergleich der Merkmale.

## Auswahl des Conditional-Operators

| Operator | Anwendungsfall | Merkmal |
|------------|------------|------|
| `iif` | Einen Stream zur Laufzeit auswählen | Wählt basierend auf Bedingungen einen von zwei Optionen |
| `partition` | Stream basierend auf Bedingung in zwei teilen | Teilt ursprünglichen Stream in True/False basierend auf Bedingung |
| `throwIfEmpty` | Leeren Stream erkennen | Wirft einen Fehler, wenn kein Wert emittiert wird |
| `defaultIfEmpty` | Standardwert verwenden, wenn leer | Bietet Fallback-Wert für leere Streams |

### Entscheidungsablauf

1. **Gibt es zwei Optionen?**
   - Ja → `iif` verwenden
   - Nein → Weiter

2. **Möchten Sie den Stream teilen?**
   - Ja → `partition` verwenden
   - Nein → Weiter

3. **Möchten Sie mit leerem Stream umgehen?**
   - Ja → Möchten Sie leeren Stream als Fehler behandeln?
     - Ja → `throwIfEmpty`
     - Nein → `defaultIfEmpty`
   - Nein → Weiter

4. **Möchten Sie einfach Werte basierend auf Bedingungen filtern?**
   - Ja → `filter`-Operator verwenden (grundlegender Filteroperator)
   - Nein → Ziel überdenken

## Zusammenfassung

Conditional-Operatoren sind leistungsstarke Werkzeuge zur Steuerung des Stream-Flusses und zur Verzweigung der Verarbeitung basierend auf bestimmten Bedingungen. Die Hauptpunkte sind wie folgt:

1. **Reaktiver Fluss basierend auf Entscheidungen**: Durch die Verwendung von Conditional-Operatoren kann die Verarbeitung dynamisch je nach Ereignis- oder Datenzustand geändert werden.
2. **Verbesserte Fehlerbehandlung**: Conditional-Operatoren funktionieren als wichtiger Teil der Fehlerbehandlungsstrategie und ermöglichen eine angemessene Reaktion auf Ausnahmefälle.
3. **Optimierungsmöglichkeiten**: Durch bedingte Ausführung kann unnötige Verarbeitung vermieden werden, insbesondere bei kostspieligen Operationen wie Netzwerkanfragen oder Hardwarezugriff.
4. **Komplexe Anwendungsabläufe**: Durch die Kombination mehrerer Conditional-Operatoren kann komplexe Geschäftslogik oder Zustandsverwaltung deklarativ ausgedrückt werden.

Conditional-Operatoren sind besonders wertvoll bei der Implementierung von Fehlerbehandlung, Cache-Strategien, Fallback-Mechanismen und bedingten Ausführungsmustern mit RxJS. Durch die Kombination mit anderen Operatoren können komplexe Anwendungsabläufe auf deklarative und typsichere Weise erstellt werden.
