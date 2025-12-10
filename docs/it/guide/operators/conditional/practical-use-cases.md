---
description: Verranno spiegati i casi d'uso pratici degli operatori condizionali RxJS (iif, defer), inclusa l'elaborazione fallback API, strategie cache, selezione dinamica della fonte dati e valutazione lazy condizionale. Pattern specifici di utilizzo in situazioni che richiedono ramificazioni di elaborazione dinamiche sono introdotti con esempi di codice TypeScript. Imparerai pattern di implementazione che possono essere applicati immediatamente allo sviluppo di applicazioni reali.
---

# Casi d'Uso Pratici

Gli operatori condizionali RxJS possono essere utilizzati per ramificare e switchare stream in base a stati dinamici.
In questo capitolo, puoi sperimentare i pattern di utilizzo di ogni operatore attraverso codice funzionante con UI.

## Selezione di Diverse Fonti Dati Basata su Condizioni

```ts
import { iif, of, EMPTY } from 'rxjs';
import { switchMap, tap, catchError, retry } from 'rxjs';

// Crea UI
const appContainer = document.createElement('div');
appContainer.innerHTML = '<h3>App Selezione Fonte Dati:</h3>';
document.body.appendChild(appContainer);

// Selezione opzioni
const optionsDiv = document.createElement('div');
optionsDiv.style.marginBottom = '15px';
appContainer.appendChild(optionsDiv);

// Checkbox (modalità offline)
const offlineCheck = document.createElement('input');
offlineCheck.type = 'checkbox';
offlineCheck.id = 'offlineMode';
optionsDiv.appendChild(offlineCheck);

const offlineLabel = document.createElement('label');
offlineLabel.htmlFor = 'offlineMode';
offlineLabel.textContent = 'Modalità Offline';
offlineLabel.style.marginLeft = '5px';
optionsDiv.appendChild(offlineLabel);

// Input ID ricerca
const idInput = document.createElement('input');
idInput.type = 'number';
idInput.placeholder = 'ID (1-10)';
idInput.min = '1';
idInput.max = '10';
idInput.value = '1';
idInput.style.marginLeft = '15px';
idInput.style.width = '80px';
optionsDiv.appendChild(idInput);

// Bottone ricerca
const searchButton = document.createElement('button');
searchButton.textContent = 'Cerca';
searchButton.style.marginLeft = '10px';
optionsDiv.appendChild(searchButton);

// Area risultati
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

// Dati offline (cache)
const cachedData: Record<number, User> = {
  1: { id: 1, name: 'Mario Rossi', email: 'rossi@example.com' },
  2: { id: 2, name: 'Anna Bianchi', email: 'bianchi@example.com' },
  3: { id: 3, name: 'Giuseppe Verdi', email: 'verdi@example.com' },
};

// Ottieni dati da API online (JSONPlaceholder)
function fetchUserFromApi(id: number) {
  console.log(`Ottenimento utente ID ${id} da API...`);

  // Endpoint API reale
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`Errore HTTP: ${response.status}`);
        }
        return response.json();
      })
    ),
    tap(() => console.log('Chiamata API riuscita')),
    catchError((err) => {
      console.error('Chiamata API fallita:', err);
      throw new Error('Richiesta API fallita');
    })
  );
}

// Ottieni utente dalla cache
function getUserFromCache(id: number) {
  console.log(`Ottenimento utente ID ${id} dalla cache...`);

  return iif(
    () => id in cachedData,
    of({ ...cachedData[id], fromCache: true }),
    EMPTY.pipe(
      tap(() => {
        throw new Error('Utente non trovato nella cache');
      })
    )
  );
}

// Click bottone ricerca
searchButton.addEventListener('click', () => {
  const id = parseInt(idInput.value, 10);
  const isOffline = offlineCheck.checked;

  // Validazione input
  if (isNaN(id) || id < 1 || id > 10) {
    resultsArea.innerHTML =
      '<p style="color: red;">Per favore inserisci un ID valido (1-10)</p>';
    return;
  }

  // Visualizzazione caricamento
  resultsArea.innerHTML = '<p>Recupero dati...</p>';

  // Seleziona fonte dati in base alla modalità offline
  iif(
    () => isOffline,
    getUserFromCache(id).pipe(
      catchError((err) => {
        console.error('Errore cache:', err);
        return of({ error: err.message });
      })
    ),
    fetchUserFromApi(id).pipe(
      retry(2), // Riprova fino a 2 volte
      catchError((err) => {
        console.error('Errore API:', err);

        // Usa cache come fallback se API fallisce
        return getUserFromCache(id).pipe(
          catchError(() =>
            of({ error: 'Sia API online che cache sono fallite' })
          )
        );
      })
    )
  ).subscribe({
    next: (result: any) => {
      if ('error' in result) {
        resultsArea.innerHTML = `<p style="color: red;">Errore: ${result.message}</p>`;
      } else {
        const source = result.fromCache
          ? '<span style="color: orange;">(dalla cache)</span>'
          : '<span style="color: green;">(da API)</span>';

        resultsArea.innerHTML = `
          <h4>Informazioni Utente ${source}</h4>
          <p><strong>ID:</strong> ${result.id}</p>
          <p><strong>Nome:</strong> ${result.name}</p>
          <p><strong>Email:</strong> ${result.email}</p>
          ${
            result.lastUpdated
              ? `<p><small>Ultimo aggiornamento: ${new Date(
                  result.lastUpdated
                ).toLocaleString()}</small></p>`
              : ''
          }
        `;
      }
    },
    error: (err) => {
      resultsArea.innerHTML = `<p style="color: red;">Errore: ${err.message}</p>`;
    },
  });
});

// Messaggio iniziale
resultsArea.innerHTML = '<p>Clicca il bottone per recuperare i dati</p>';


```



## Ramificazione a Runtime e Strategie di Fallback

In questo esempio usando `iif`, la fonte dati viene switchata dinamicamente da "cache offline" e "API online" secondo le operazioni e gli stati dell'utente.
Inoltre, combinando `catchError` e `retry`, si possono definire ritentativi e destinazioni di fallback in caso di fallimento.

È particolarmente adatto per i seguenti casi d'uso:

- Supporto offline in ambienti di rete instabili
- Utilizzo cache e switching richieste online
- Ritentativo automatico e switching a route alternative in caso di fallimento API

## Pattern di Ottimizzazione Performance

In scenari più complessi, si possono implementare pattern di acquisizione dati ottimizzati combinando operatori condizionali.

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

// Crea elementi UI
const optimizationContainer = document.createElement('div');
optimizationContainer.innerHTML = '<h3>Recupero Dati Condizionale Avanzato:</h3>';
document.body.appendChild(optimizationContainer);

// UI Ricerca
const searchInputGroup = document.createElement('div');
searchInputGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(searchInputGroup);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Inserisci ID utente (1-10)';
searchInput.value = '1';
searchInput.style.padding = '8px';
searchInput.style.width = '180px';
searchInputGroup.appendChild(searchInput);

const searchButton = document.createElement('button');
searchButton.textContent = 'Cerca';
searchButton.style.marginLeft = '10px';
searchButton.style.padding = '8px 16px';
searchInputGroup.appendChild(searchButton);

// Impostazioni opzioni
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
cacheLabel.textContent = 'Usa Cache';
cacheLabel.style.marginRight = '15px';
optionsGroup.appendChild(cacheLabel);

const forceCheckbox = document.createElement('input');
forceCheckbox.type = 'checkbox';
forceCheckbox.id = 'forceRefresh';
optionsGroup.appendChild(forceCheckbox);

const forceLabel = document.createElement('label');
forceLabel.htmlFor = 'forceRefresh';
forceLabel.textContent = 'Forza Aggiornamento';
optionsGroup.appendChild(forceLabel);

// Area visualizzazione risultati
const optimizedResults = document.createElement('div');
optimizedResults.style.padding = '15px';
optimizedResults.style.border = '1px solid #ddd';
optimizedResults.style.borderRadius = '5px';
optimizedResults.style.minHeight = '150px';
optimizedResults.style.backgroundColor = '#f9f9f9';
optimizationContainer.appendChild(optimizedResults);

// Gestione cache
const cache = new Map<string, { data: any; timestamp: number }>();
const CACHE_EXPIRY = 30000; // 30 secondi

// Ottieni dati utente da API reale (JSONPlaceholder)
function fetchUserData(id: string, forceRefresh: boolean): Observable<any> {
  // ID non valido
  if (!id || isNaN(Number(id)) || Number(id) < 1 || Number(id) > 10) {
    return throwError(
      () => new Error('ID utente non valido: per favore inserisci un numero tra 1 e 10')
    );
  }

  const cacheKey = `user-${id}`;
  const cachedItem = cache.get(cacheKey);
  const now = Date.now();

  // Controllo cache (entro scadenza e non forza aggiornamento)
  if (
    !forceRefresh &&
    cachedItem &&
    now - cachedItem.timestamp < CACHE_EXPIRY
  ) {
    console.log(`Recuperato dalla cache: ${id}`);
    return of({
      ...cachedItem.data,
      fromCache: true,
    }).pipe(delay(100)); // Simula risposta veloce
  }

  // Richiesta API reale (JSONPlaceholder)
  console.log(`Recupero dati da API: ${id}`);
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`Errore HTTP: ${response.status}`);
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

      // Salva nella cache
      cache.set(cacheKey, {
        data: processedData,
        timestamp: now,
      });

      return processedData;
    }),
    catchError((err) => {
      console.error('Errore API:', err);
      throw new Error('Richiesta API fallita');
    })
  );
}

// Monitora modifiche condizioni ricerca
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map((event) => (event.target as HTMLInputElement).value.trim()),
  debounceTime(300),
  distinctUntilChanged()
);

// Monitora modifiche impostazioni cache
const useCache$ = fromEvent(cacheCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// Monitora modifiche forza aggiornamento
const forceRefresh$ = fromEvent(forceCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(false)
);

// Evento click bottone ricerca
const searchClick$ = fromEvent(searchButton, 'click');

// Esegui ricerca
searchClick$
  .pipe(
    // Ottieni valore input corrente, impostazione cache, impostazione forza aggiornamento
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
      // Visualizza inizio ricerca
      optimizedResults.innerHTML = '<p>Ricerca in corso...</p>';
    }),
    // Stream condizionale usando iif()
    switchMap(({ term, useCache, forceRefresh }) => {
      // Se termine ricerca è vuoto
      if (!term) {
        return of({ error: 'Per favore inserisci un termine di ricerca' });
      }

      // Se cache è disabilitata
      if (!useCache) {
        return fetchUserData(term, true);
      }

      // Ricerca normale (usa cache & forza aggiornamento se necessario)
      return fetchUserData(term, forceRefresh);
    }),
    // Gestione errori
    catchError((err) => {
      return of({ error: err.message });
    })
  )
  .subscribe({
    next: (result) => {
      if ('error' in result) {
        // Visualizzazione errore
        optimizedResults.innerHTML = `
        <p style="color: red;">Errore: ${result.error}</p>
      `;
      } else {
        // Visualizzazione dati
        const source = result.fromCache
          ? '<span style="color: orange;">(dalla cache)</span>'
          : '<span style="color: green;">(da API)</span>';

        optimizedResults.innerHTML = `
        <h4>Informazioni Utente ${source}</h4>
        <p><strong>ID:</strong> ${result.id}</p>
        <p><strong>Nome:</strong> ${result.name}</p>
        <p><strong>Email:</strong> ${result.email}</p>
        ${
          result.lastUpdated
            ? `<p><small>Ultimo aggiornamento: ${new Date(
                result.lastUpdated
              ).toLocaleString()}</small></p>`
            : ''
        }
      `;
      }
    },
  });

// Messaggio iniziale
optimizedResults.innerHTML =
  '<p>Inserisci un ID utente e clicca il bottone cerca</p>';

```


---

## Guida alla Selezione degli Operatori

Molti operatori condizionali sembrano simili e possono confondere, ma ognuno ha uno scopo di applicazione chiaro.
Di seguito è riportato un confronto dei flussi di decisione tipici e delle caratteristiche.

## Come Scegliere un Operatore Condizionale

| Operatore | Caso d'Uso | Caratteristiche |
|------------|------------|------|
| `iif` | Seleziona uno stream a runtime | Seleziona una delle due scelte basandosi su una condizione |
| `partition` | Separa uno stream in due stream basandosi su una condizione | Divide lo stream originale in True/False basandosi su una condizione |
| `throwIfEmpty` | Rileva stream vuoti | Lancia un errore se nessun valore viene emesso |
| `defaultIfEmpty` | Usa valore predefinito se vuoto | Fornisce valore di fallback se lo stream è vuoto |

### Flusso di Decisione per la Selezione

1. **Ci sono due opzioni?**
   - Sì → Usa `iif`
   - No → Prossimo

2. **Vuoi dividere lo stream?**
   - Sì → Usa `partition`
   - No → Prossimo

3. **Vuoi gestire stream vuoti?**
   - Sì → Vuoi trattare stream vuoti come errori?
     - Sì → `throwIfEmpty`
     - No → `defaultIfEmpty`
   - No → Prossimo

4. **Vuoi semplicemente filtrare valori basandoti su una condizione?**
   - Sì → Usa l'operatore `filter` (operatore di filtraggio base)
   - No → Riesamina lo scopo

## Riepilogo

Gli operatori condizionali sono strumenti potenti per controllare il flusso degli stream e ramificare l'elaborazione in base a condizioni specifiche. I punti principali sono i seguenti:

1. **Flusso reattivo basato su decisioni**: Gli operatori condizionali possono essere usati per cambiare dinamicamente l'elaborazione in base a eventi o condizioni dei dati.
2. **Gestione errori migliorata**: Gli operatori condizionali possono servire come parte importante della tua strategia di gestione errori, permettendo la gestione graziosa di casi eccezionali.
3. **Opportunità di ottimizzazione**: L'esecuzione condizionale evita elaborazioni non necessarie e ottimizza operazioni costose, specialmente richieste di rete e accesso hardware.
4. **Flussi applicativi complessi**: Logiche di business complesse e gestione stato possono essere espresse dichiarativamente combinando più operatori condizionali.

Gli operatori condizionali sono particolarmente preziosi quando si implementano gestione errori, strategie di caching, meccanismi di fallback e pattern di esecuzione condizionale usando RxJS. Combinati con altri operatori, permettono di costruire flussi applicativi complessi in modo dichiarativo e type-safe.
