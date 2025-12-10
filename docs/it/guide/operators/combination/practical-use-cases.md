---
description: Verranno spiegati i casi d'uso pratici per gli operatori di combinazione RxJS (combineLatest, forkJoin, merge, concat, withLatestFrom, ecc.). Pattern pratici di combinazione di più Observable come validazione input form e integrazione API, esecuzione parallela di più richieste, sincronizzazione dati in tempo reale e elaborazione sequenziale di stream saranno presentati con esempi di codice TypeScript.
---

# Casi d'Uso Pratici

In questo capitolo, introdurremo **casi d'uso pratici** che sfruttano gli operatori di combinazione di RxJS.
Approfondisci la comprensione attraverso scenari utili per lo sviluppo di applicazioni reali, come operazioni UI e comunicazione API.

## Validazione Input Form e Richieste API

Un esempio di validazione di più input form utilizzando `combineLatest`.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, debounceTime, startWith } from 'rxjs';

// Crea UI form
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>Form di Registrazione Utente:</h3>';
document.body.appendChild(formContainer);

// Input nome
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Nome: ';
formContainer.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.id = 'name';
nameInput.style.marginBottom = '10px';
nameInput.style.marginLeft = '5px';
formContainer.appendChild(nameInput);
formContainer.appendChild(document.createElement('br'));

// Input email
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Email: ';
formContainer.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.id = 'email';
emailInput.style.marginBottom = '10px';
emailInput.style.marginLeft = '5px';
formContainer.appendChild(emailInput);
formContainer.appendChild(document.createElement('br'));

// Input password
const passwordLabel = document.createElement('label');
passwordLabel.textContent = 'Password: ';
formContainer.appendChild(passwordLabel);

const passwordInput = document.createElement('input');
passwordInput.type = 'password';
passwordInput.id = 'password';
passwordInput.style.marginLeft = '5px';
formContainer.appendChild(passwordInput);
formContainer.appendChild(document.createElement('br'));

// Pulsante invio
const submitButton = document.createElement('button');
submitButton.textContent = 'Registra';
submitButton.disabled = true;
submitButton.style.marginTop = '15px';
submitButton.style.padding = '8px 16px';
formContainer.appendChild(submitButton);

// Messaggio di validazione
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Validazione nome
const name$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: value.length >= 2,
      error: value.length < 2 ? 'Il nome deve essere di almeno 2 caratteri' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Il nome deve essere di almeno 2 caratteri',
  })
);

// Validazione email
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: emailRegex.test(value),
      error: !emailRegex.test(value)
        ? 'Inserisci un indirizzo email valido'
        : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Inserisci un indirizzo email valido',
  })
);

// Validazione password
const password$ = fromEvent(passwordInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value;
    return {
      value,
      valid: value.length >= 6,
      error: value.length < 6 ? 'La password deve essere di almeno 6 caratteri' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'La password deve essere di almeno 6 caratteri',
  })
);

// Combina lo stato di validazione di tutti i campi
combineLatest([name$, email$, password$])
  .pipe(debounceTime(300))
  .subscribe(([nameState, emailState, passwordState]) => {
    // Verifica se il form è valido
    const isFormValid =
      nameState.valid && emailState.valid && passwordState.valid;
    submitButton.disabled = !isFormValid;

    // Mostra messaggi di errore
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

// Evento click pulsante invio
fromEvent(submitButton, 'click').subscribe(() => {
  const formData = {
    name: nameInput.value,
    email: emailInput.value,
    password: passwordInput.value,
  };

  // Mostra dati form (nell'uso reale, invia all'API)
  const successMessage = document.createElement('div');
  successMessage.textContent = 'Registrazione completata!';
  successMessage.style.color = 'green';
  successMessage.style.fontWeight = 'bold';
  successMessage.style.marginTop = '10px';
  formContainer.appendChild(successMessage);

  console.log('Dati inviati:', formData);
});

```

## Richieste Concorrenti e Gestione Stato di Caricamento

Ecco un esempio di utilizzo di `forkJoin` per elaborare più richieste API in parallelo e riassumere i risultati.

```ts
import {
  forkJoin,
  of,
  throwError,
  Observable,
  ObservableInputTuple,
} from 'rxjs';
import { catchError, delay, finalize } from 'rxjs';

// Definizioni interfacce
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

// Definizione tipo risultato
interface ApiResponse {
  user: User;
  posts: Post[];
  weather: Weather;
}

// Crea elementi UI
const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>Esempio Richieste API Multiple:</h3>';
document.body.appendChild(apiContainer);

const loadButton = document.createElement('button');
loadButton.textContent = 'Carica Dati';
loadButton.style.padding = '8px 16px';
apiContainer.appendChild(loadButton);

const loadingIndicator = document.createElement('div');
loadingIndicator.style.margin = '10px 0';
loadingIndicator.style.display = 'none';
apiContainer.appendChild(loadingIndicator);

const resultContainer = document.createElement('div');
apiContainer.appendChild(resultContainer);

// Simula richieste API
function fetchUser(id: number): Observable<User> {
  // Richiesta riuscita
  return of({
    id,
    name: `Utente${id}`,
    email: `utente${id}@example.com`,
  }).pipe(
    delay(2000) // Ritardo di 2 secondi
  );
}

function fetchPosts(userId: number): Observable<Post[]> {
  // Richiesta riuscita
  return of([
    { id: 1, title: `Post 1 di ${userId}`, content: 'Contenuto...' },
    { id: 2, title: `Post 2 di ${userId}`, content: 'Contenuto...' },
  ]).pipe(
    delay(1500) // Ritardo di 1.5 secondi
  );
}

function fetchWeather(city: string): Observable<WeatherSuccess> {
  // A volte fallisce
  const shouldFail = Math.random() > 0.7;

  if (shouldFail) {
    return throwError(() => new Error('Impossibile recuperare i dati meteo')).pipe(
      delay(1000)
    );
  }

  return of({
    city,
    temp: Math.round(15 + Math.random() * 10),
    condition: ['Soleggiato', 'Nuvoloso', 'Piovoso'][Math.floor(Math.random() * 3)],
  }).pipe(
    delay(1000) // Ritardo di 1 secondo
  );
}

// Esegui più richieste al click del pulsante
loadButton.addEventListener('click', () => {
  // Reset UI
  resultContainer.innerHTML = '';
  loadingIndicator.style.display = 'block';
  loadingIndicator.textContent = 'Caricamento dati...';
  loadButton.disabled = true;

  // Esegui più richieste API concorrentemente
  forkJoin({
    user: fetchUser(1),
    posts: fetchPosts(1),
    weather: fetchWeather('Tokyo').pipe(
      // Gestione errori
      catchError((error: Error) => {
        console.error('Errore API meteo:', error);
        return of<WeatherError>({ error: error.message });
      })
    ),
  } as ObservableInputTuple<ApiResponse>)
    .pipe(
      // Pulizia al completamento
      finalize(() => {
        loadingIndicator.style.display = 'none';
        loadButton.disabled = false;
      })
    )
    .subscribe((results: ApiResponse) => {
      // Mostra info utente
      const userInfo = document.createElement('div');
      userInfo.innerHTML = `
      <h4>Informazioni Utente</h4>
      <p>Nome: ${results.user.name}</p>
      <p>Email: ${results.user.email}</p>
    `;
      userInfo.style.margin = '10px 0';
      userInfo.style.padding = '10px';
      userInfo.style.backgroundColor = '#f0f0f0';
      userInfo.style.borderRadius = '5px';
      resultContainer.appendChild(userInfo);

      // Mostra post
      const postsInfo = document.createElement('div');
      postsInfo.innerHTML = `
      <h4>Post (${results.posts.length})</h4>
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

      // Mostra info meteo
      const weatherInfo = document.createElement('div');

      if ('error' in results.weather) {
        weatherInfo.innerHTML = `
        <h4>Informazioni Meteo</h4>
        <p style="color: red;">Errore: ${results.weather.error}</p>
      `;
      } else {
        weatherInfo.innerHTML = `
        <h4>Informazioni Meteo</h4>
        <p>Città: ${results.weather.city}</p>
        <p>Temperatura: ${results.weather.temp}°C</p>
        <p>Condizioni: ${results.weather.condition}</p>
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

## Funzione di Ricerca Annullabile

Ecco un esempio di combinazione di `withLatestFrom` e `race` per implementare un timeout o una funzione di ricerca annullabile.

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

// Crea UI ricerca
const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Ricerca Annullabile:</h3>';
document.body.appendChild(searchContainer);

// Campo input ricerca
const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Inserisci termine di ricerca...';
searchInput.style.padding = '8px';
searchInput.style.width = '250px';
searchContainer.appendChild(searchInput);

// Pulsante annulla
const cancelButton = document.createElement('button');
cancelButton.textContent = 'Annulla';
cancelButton.style.marginLeft = '10px';
cancelButton.style.padding = '8px 16px';
searchContainer.appendChild(cancelButton);

// Area risultati ricerca
const resultsContainer = document.createElement('div');
resultsContainer.style.marginTop = '10px';
resultsContainer.style.minHeight = '200px';
resultsContainer.style.padding = '10px';
resultsContainer.style.border = '1px solid #ddd';
resultsContainer.style.borderRadius = '5px';
searchContainer.appendChild(resultsContainer);

// Simula richiesta di ricerca
function searchApi(term: string) {
  console.log(`Avvio ricerca per "${term}"...`);

  // Simula risultati di ricerca
  return of([
    `Risultato ricerca 1 per "${term}"`,
    `Risultato ricerca 2 per "${term}"`,
    `Risultato ricerca 3 per "${term}"`,
  ]).pipe(
    // Ritardo casuale tra 2-5 secondi
    delay(2000 + Math.random() * 3000),
    // Gestione errori
    catchError((err) => {
      console.error('Errore ricerca:', err);
      return EMPTY;
    })
  );
}

// Evento annulla
const cancel$ = fromEvent(cancelButton, 'click');

// Evento ricerca
const search$ = fromEvent(searchInput, 'input')
  .pipe(
    // Ottieni valore input
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // Attendi 300ms
    debounceTime(300),
    // Ignora ricerche vuote
    tap((term) => {
      if (term === '') {
        resultsContainer.innerHTML = '<p>Inserisci un termine di ricerca</p>';
      }
    }),
    // Non elaborare ricerche vuote
    switchMap((term) => {
      if (term === '') {
        return EMPTY;
      }

      // Mostra messaggio di ricerca in corso
      resultsContainer.innerHTML = '<p>Ricerca in corso...</p>';

      // Gestione timeout (5 secondi)
      const timeout$ = timer(5000).pipe(
        tap(() => console.log('Timeout ricerca')),
        map(() => ({ type: 'timeout', results: null }))
      );

      // Richiesta API
      const request$ = searchApi(term).pipe(
        map((results) => ({ type: 'success', results })),
        // Annulla se viene premuto il pulsante annulla
        takeUntil(
          cancel$.pipe(
            tap(() => {
              console.log('Ricerca annullata');
              resultsContainer.innerHTML = '<p>Ricerca annullata</p>';
            })
          )
        )
      );

      // Gara tra timeout e completamento richiesta
      return race(request$, timeout$);
    })
  )
  .subscribe((response) => {
    if (response.type === 'success') {
      // Ricerca riuscita
      resultsContainer.innerHTML = '<h4>Risultati Ricerca:</h4>';

      if (response.results?.length === 0) {
        resultsContainer.innerHTML += '<p>Nessun risultato trovato</p>';
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
        '<p style="color: red;">Timeout ricerca. Riprova.</p>';
    }
  });

```


## Confronto Operatori di Combinazione e Guida alla Selezione

Confronta le differenze tra più operatori di combinazione e aiutati a scegliere quello giusto per il tuo caso d'uso.

| Operatore | Timing | Output | Caso d'Uso |
|------------|------------|-----|------------|
| `merge` | Esecuzione concorrente | Output in ordine di occorrenza | Monitorare più eventi sorgente simultaneamente |
| `concat` | Esecuzione sequenziale | Output in ordine | Task asincroni dove l'ordine conta |
| `combineLatest` | Richiede almeno un valore da tutte le sorgenti | Combinazione di tutti gli ultimi valori | Validazione input form |
| `zip` | Richiede valori di indice corrispondente da tutte le sorgenti | Combinazione di valori per indice | Sincronizzazione dati correlati |
| `withLatestFrom` | Quando la sorgente principale emette valore | Valore principale e ultimo valore da altre sorgenti | Combinazione dati ausiliari |
| `forkJoin` | Quando tutte le sorgenti completano | Ultimo valore da ogni sorgente | Richieste API multiple |
| `race` | Solo la prima sorgente che emette | Solo valori dello stream vincitore | Timeout, gestione annullamento |

### Flusso Decisionale Selezione Operatore

1. **Vuoi ricevere valori da tutte le sorgenti contemporaneamente?**
   - Sì → `merge`
   - No → Prossimo

2. **Vuoi preservare l'ordine delle sorgenti?**
   - Sì → `concat`
   - No → Prossimo

3. **Hai bisogno di una combinazione degli ultimi valori per ogni sorgente?**
   - Sì → Quando combinare?
     - Per ogni nuovo valore di qualsiasi sorgente → `combineLatest`
     - Per ogni valore specifico dello stream principale → `withLatestFrom`
   - No → Prossimo

4. **Hai bisogno di valori corrispondenti in ordine di indice?**
   - Sì → `zip`
   - No → Prossimo

5. **Hai bisogno dei risultati dopo che tutte le sorgenti sono complete?**
   - Sì → `forkJoin`
   - No → Prossimo

6. **Hai bisogno solo del più veloce tra più sorgenti alternative?**
   - Sì → `race`
   - No → Riesamina lo scopo


## Strategia di Cambio

Questo è un esempio di cambio dinamico tra più sorgenti dati.

```ts
import { fromEvent, merge, interval, of } from 'rxjs';
import { map, switchMap, take, tap } from 'rxjs';

// Crea elementi UI
const switchingContainer = document.createElement('div');
switchingContainer.innerHTML = '<h3>Cambio Sorgente Dati:</h3>';
document.body.appendChild(switchingContainer);

// Crea pulsanti
const source1Button = document.createElement('button');
source1Button.textContent = 'Sorgente 1';
source1Button.style.margin = '5px';
source1Button.style.padding = '5px 10px';
switchingContainer.appendChild(source1Button);

const source2Button = document.createElement('button');
source2Button.textContent = 'Sorgente 2';
source2Button.style.margin = '5px';
source2Button.style.padding = '5px 10px';
switchingContainer.appendChild(source2Button);

const source3Button = document.createElement('button');
source3Button.textContent = 'Sorgente 3';
source3Button.style.margin = '5px';
source3Button.style.padding = '5px 10px';
switchingContainer.appendChild(source3Button);

// Area visualizzazione risultati
const resultsArea = document.createElement('div');
resultsArea.style.marginTop = '10px';
resultsArea.style.minHeight = '150px';
resultsArea.style.padding = '10px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.backgroundColor = '#f9f9f9';
switchingContainer.appendChild(resultsArea);

// Tre sorgenti dati
function createSource1() {
  return interval(1000).pipe(
    take(5),
    map((val) => `Sorgente 1: ${val}`),
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
    map((val) => `Sorgente 2: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '#bbdefb';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource3() {
  return of('Sorgente 3: A', 'Sorgente 3: B', 'Sorgente 3: C').pipe(
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '#ffccbc';
    })
  );
}

// Eventi click pulsanti
const source1Click$ = fromEvent(source1Button, 'click').pipe(map(() => 1));

const source2Click$ = fromEvent(source2Button, 'click').pipe(map(() => 2));

const source3Click$ = fromEvent(source3Button, 'click').pipe(map(() => 3));

// Unisci click pulsanti
merge(source1Click$, source2Click$, source3Click$)
  .pipe(
    // Passa alla sorgente selezionata
    switchMap((sourceId) => {
      // Pulisci area risultati
      resultsArea.innerHTML = '';

      // Ritorna sorgente selezionata
      switch (sourceId) {
        case 1:
          return createSource1();
        case 2:
          return createSource2();
        case 3:
          return createSource3();
        default:
          return of('Nessuna sorgente selezionata');
      }
    })
  )
  .subscribe((value) => {
    // Mostra risultato
    const item = document.createElement('div');
    item.textContent = value;
    item.style.padding = '5px';
    item.style.margin = '2px 0';
    item.style.backgroundColor = 'white';
    item.style.borderRadius = '3px';
    resultsArea.appendChild(item);
  });

// Messaggio iniziale
const initialMessage = document.createElement('div');
initialMessage.textContent =
  'Clicca un pulsante per selezionare una sorgente dati';
initialMessage.style.color = '#666';
resultsArea.appendChild(initialMessage);

```

## Merge Condizionale

Questo è un esempio di combinazione di `merge` e `filter` per selezionare sorgenti dati basandosi su condizioni.

```ts
import { merge, interval, fromEvent } from 'rxjs';
import {
  map,
  filter,
  takeUntil,
  withLatestFrom,
  startWith,
} from 'rxjs';

// Crea elementi UI
const conditionalContainer = document.createElement('div');
conditionalContainer.innerHTML = '<h3>Merge Condizionale:</h3>';
document.body.appendChild(conditionalContainer);

// Impostazioni filtro
const filterDiv = document.createElement('div');
filterDiv.style.marginBottom = '10px';
conditionalContainer.appendChild(filterDiv);

// Crea checkbox
const slowCheck = document.createElement('input');
slowCheck.type = 'checkbox';
slowCheck.id = 'slowCheck';
slowCheck.checked = true;
filterDiv.appendChild(slowCheck);

const slowLabel = document.createElement('label');
slowLabel.htmlFor = 'slowCheck';
slowLabel.textContent = 'Sorgente Lenta';
slowLabel.style.marginRight = '15px';
filterDiv.appendChild(slowLabel);

const fastCheck = document.createElement('input');
fastCheck.type = 'checkbox';
fastCheck.id = 'fastCheck';
fastCheck.checked = true;
filterDiv.appendChild(fastCheck);

const fastLabel = document.createElement('label');
fastLabel.htmlFor = 'fastCheck';
fastLabel.textContent = 'Sorgente Veloce';
fastLabel.style.marginRight = '15px';
filterDiv.appendChild(fastLabel);

const clickCheck = document.createElement('input');
clickCheck.type = 'checkbox';
clickCheck.id = 'clickCheck';
clickCheck.checked = true;
filterDiv.appendChild(clickCheck);

const clickLabel = document.createElement('label');
clickLabel.htmlFor = 'clickCheck';
clickLabel.textContent = 'Eventi Click';
filterDiv.appendChild(clickLabel);

// Pulsante stop
const stopButton = document.createElement('button');
stopButton.textContent = 'Stop';
stopButton.style.marginLeft = '15px';
filterDiv.appendChild(stopButton);

// Area visualizzazione risultati
const conditionalResults = document.createElement('div');
conditionalResults.style.height = '200px';
conditionalResults.style.overflowY = 'auto';
conditionalResults.style.padding = '10px';
conditionalResults.style.border = '1px solid #ddd';
conditionalResults.style.backgroundColor = '#f9f9f9';
conditionalContainer.appendChild(conditionalResults);

// Tre sorgenti dati
// 1. Sorgente lenta (ogni 1 secondo)
const slow$ = interval(1000).pipe(map((val) => ({ type: 'slow', value: val })));

// 2. Sorgente veloce (ogni 300 millisecondi)
const fast$ = interval(300).pipe(map((val) => ({ type: 'fast', value: val })));

// 3. Eventi click
const click$ = fromEvent(document.body, 'click').pipe(
  map((event) => ({
    type: 'click',
    value: {
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY,
    },
  }))
);

// Monitora stato checkbox
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

// Evento stop
const stop$ = fromEvent(stopButton, 'click');

// Merge condizionale
merge(
  // Combina sorgente lenta con stato abilitato
  slow$.pipe(
    withLatestFrom(slowEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Combina sorgente veloce con stato abilitato
  fast$.pipe(
    withLatestFrom(fastEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Combina sorgente click con stato abilitato
  click$.pipe(
    withLatestFrom(clickEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  )
)
  .pipe(takeUntil(stop$))
  .subscribe((event) => {
    // Mostra risultato
    const item = document.createElement('div');

    switch (event.type) {
      case 'slow':
        item.textContent = `Sorgente Lenta: ${event.value}`;
        item.style.color = '#1b5e20';
        break;
      case 'fast':
        item.textContent = `Sorgente Veloce: ${event.value}`;
        item.style.color = '#0d47a1';
        break;
      case 'click':
        const clickValue = event.value as { x: number; y: number };
        item.textContent = `Click: X=${clickValue.x}, Y=${clickValue.y}`;
        item.style.color = '#bf360c';
        break;
    }

    item.style.padding = '3px';
    item.style.margin = '2px 0';
    conditionalResults.prepend(item); // Mostra il più recente in alto
  });

```

## Riepilogo Scelta Operatori di Combinazione

| Scopo | Operatore | Caratteristiche |
|------|--------------|------|
| Sincronizzare sempre più ultimi valori | `combineLatest` | Combina sempre l'ultimo valore di ogni Observable |
| Ottenere tutto insieme dopo il completamento | `forkJoin` | Emette solo l'ultimo valore (una volta) |
| Elaborare in ordine sincronamente | `zip` | Combina uno da ogni Observable ed emette |
| Riferire altri ultimi valori al trigger | `withLatestFrom` | Allega l'ultimo valore dello stream secondario quando lo stream principale emette |

## Riepilogo

Gli operatori di combinazione sono strumenti potenti per combinare più sorgenti dati in un singolo stream. Selezionando l'operatore appropriato, flussi di dati asincroni complessi possono essere espressi in modo conciso e dichiarativo.

### Punti Chiave per Padroneggiare gli Operatori di Combinazione

1. **Seleziona l'operatore giusto per il tuo caso d'uso**: Ogni operatore è ottimizzato per un caso d'uso specifico. Scegli l'operatore appropriato per il tuo scopo.
2. **Comprendi quando emettere**: Il comportamento degli operatori di combinazione dipende molto da quando i valori vengono emessi. È importante capire il timing di emissione di ogni operatore.
3. **Considerazioni sulla Gestione Errori**: Considera il comportamento quando si verifica un errore in parte dello stream combinato (se fallisce tutto o continua parzialmente).
4. **Conoscere le condizioni di completamento**: È anche importante capire quando lo stream combinato completerà, e completarlo esplicitamente usando `takeUntil` o simili se necessario.
5. **Sfrutta la type safety**: TypeScript ti permette di gestire gli operatori di combinazione in modo type-safe. Il beneficio del tipo è particolarmente grande per combinazioni complesse.

Gli operatori di combinazione possono essere sfruttati in molti scenari pratici come gestione eventi UI, richieste API multiple, validazione form, ecc. Padroneggiare questi operatori ti aiuterà a sbloccare il vero potere della programmazione reattiva in RxJS.

---
Prossimo, passiamo a [Gestione Errori](/it/guide/error-handling/strategies) per imparare come scrivere codice RxJS più robusto!
