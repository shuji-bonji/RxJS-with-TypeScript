---
description: Praktische Anwendungsfälle für RxJS-Kombinationsoperatoren (combineLatest, forkJoin, merge, concat, withLatestFrom usw.) werden erklärt. Formular-Eingabevalidierung und API-Integration, parallele Ausführung mehrerer Anfragen, Echtzeit-Datensynchronisierung, sequentielle Stream-Verarbeitung und andere praktische Muster zur Kombination mehrerer Observables werden mit TypeScript-Codebeispielen vorgestellt.
---

# Praktische Anwendungsfälle

Dieses Kapitel stellt **praktische Anwendungsfälle** vor, die RxJS-Kombinationsoperatoren nutzen.
Vertiefen Sie Ihr Verständnis durch Szenarien, die in der tatsächlichen Anwendungsentwicklung nützlich sind, wie UI-Operationen und API-Kommunikation.

## Formular-Eingabevalidierung und API-Anfragen

Ein Beispiel zur Validierung mehrerer Formulareingaben mit `combineLatest`.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, debounceTime, startWith } from 'rxjs';

// Formular-UI erstellen
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>Benutzerregistrierungsformular:</h3>';
document.body.appendChild(formContainer);

// Namenseingabe
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Name: ';
formContainer.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.id = 'name';
nameInput.style.marginBottom = '10px';
nameInput.style.marginLeft = '5px';
formContainer.appendChild(nameInput);
formContainer.appendChild(document.createElement('br'));

// E-Mail-Eingabe
const emailLabel = document.createElement('label');
emailLabel.textContent = 'E-Mail: ';
formContainer.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.id = 'email';
emailInput.style.marginBottom = '10px';
emailInput.style.marginLeft = '5px';
formContainer.appendChild(emailInput);
formContainer.appendChild(document.createElement('br'));

// Passworteingabe
const passwordLabel = document.createElement('label');
passwordLabel.textContent = 'Passwort: ';
formContainer.appendChild(passwordLabel);

const passwordInput = document.createElement('input');
passwordInput.type = 'password';
passwordInput.id = 'password';
passwordInput.style.marginLeft = '5px';
formContainer.appendChild(passwordInput);
formContainer.appendChild(document.createElement('br'));

// Absenden-Button
const submitButton = document.createElement('button');
submitButton.textContent = 'Registrieren';
submitButton.disabled = true;
submitButton.style.marginTop = '15px';
submitButton.style.padding = '8px 16px';
formContainer.appendChild(submitButton);

// Validierungsmeldung
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Namensvalidierung
const name$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: value.length >= 2,
      error: value.length < 2 ? 'Bitte geben Sie mindestens 2 Zeichen für den Namen ein' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Bitte geben Sie mindestens 2 Zeichen für den Namen ein',
  })
);

// E-Mail-Validierung
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: emailRegex.test(value),
      error: !emailRegex.test(value)
        ? 'Bitte geben Sie eine gültige E-Mail-Adresse ein'
        : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Bitte geben Sie eine gültige E-Mail-Adresse ein',
  })
);

// Passwortvalidierung
const password$ = fromEvent(passwordInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value;
    return {
      value,
      valid: value.length >= 6,
      error: value.length < 6 ? 'Bitte geben Sie mindestens 6 Zeichen für das Passwort ein' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Bitte geben Sie mindestens 6 Zeichen für das Passwort ein',
  })
);

// Validierungszustände aller Felder kombinieren
combineLatest([name$, email$, password$])
  .pipe(debounceTime(300))
  .subscribe(([nameState, emailState, passwordState]) => {
    // Ist das Formular gültig?
    const isFormValid =
      nameState.valid && emailState.valid && passwordState.valid;
    submitButton.disabled = !isFormValid;

    // Fehlermeldungen anzeigen
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

// Absenden-Button-Klick-Ereignis
fromEvent(submitButton, 'click').subscribe(() => {
  const formData = {
    name: nameInput.value,
    email: emailInput.value,
    password: passwordInput.value,
  };

  // Formulardaten anzeigen (in der Praxis an API senden)
  const successMessage = document.createElement('div');
  successMessage.textContent = 'Registrierung abgeschlossen!';
  successMessage.style.color = 'green';
  successMessage.style.fontWeight = 'bold';
  successMessage.style.marginTop = '10px';
  formContainer.appendChild(successMessage);

  console.log('Gesendete Daten:', formData);
});

```

## Gleichzeitige Anfragen und Ladezustandsverwaltung

Ein Beispiel zur parallelen Verarbeitung mehrerer API-Anfragen mit `forkJoin` und Zusammenfassung der Ergebnisse.

```ts
import {
  forkJoin,
  of,
  throwError,
  Observable,
  ObservableInputTuple,
} from 'rxjs';
import { catchError, delay, finalize } from 'rxjs';

// Interface-Definitionen
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

// Ergebnistyp-Definition
interface ApiResponse {
  user: User;
  posts: Post[];
  weather: Weather;
}

// UI-Elemente erstellen
const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>Beispiel für mehrere API-Anfragen:</h3>';
document.body.appendChild(apiContainer);

const loadButton = document.createElement('button');
loadButton.textContent = 'Daten laden';
loadButton.style.padding = '8px 16px';
apiContainer.appendChild(loadButton);

const loadingIndicator = document.createElement('div');
loadingIndicator.style.margin = '10px 0';
loadingIndicator.style.display = 'none';
apiContainer.appendChild(loadingIndicator);

const resultContainer = document.createElement('div');
apiContainer.appendChild(resultContainer);

// API-Anfragen simulieren
function fetchUser(id: number): Observable<User> {
  // Erfolgreiche Anfrage
  return of({
    id,
    name: `Benutzer${id}`,
    email: `user${id}@example.com`,
  }).pipe(
    delay(2000) // 2 Sekunden Verzögerung
  );
}

function fetchPosts(userId: number): Observable<Post[]> {
  // Erfolgreiche Anfrage
  return of([
    { id: 1, title: `${userId} Beitrag 1`, content: 'Inhalt...' },
    { id: 2, title: `${userId} Beitrag 2`, content: 'Inhalt...' },
  ]).pipe(
    delay(1500) // 1,5 Sekunden Verzögerung
  );
}

function fetchWeather(city: string): Observable<WeatherSuccess> {
  // Gelegentlich fehlschlagende Anfrage
  const shouldFail = Math.random() > 0.7;

  if (shouldFail) {
    return throwError(() => new Error('Wetterdatenabruf fehlgeschlagen')).pipe(
      delay(1000)
    );
  }

  return of({
    city,
    temp: Math.round(15 + Math.random() * 10),
    condition: ['Sonnig', 'Bewölkt', 'Regnerisch'][Math.floor(Math.random() * 3)],
  }).pipe(
    delay(1000) // 1 Sekunde Verzögerung
  );
}

// Bei Button-Klick mehrere Anfragen ausführen
loadButton.addEventListener('click', () => {
  // UI zurücksetzen
  resultContainer.innerHTML = '';
  loadingIndicator.style.display = 'block';
  loadingIndicator.textContent = 'Daten werden geladen...';
  loadButton.disabled = true;

  // Mehrere API-Anfragen gleichzeitig ausführen
  forkJoin({
    user: fetchUser(1),
    posts: fetchPosts(1),
    weather: fetchWeather('Berlin').pipe(
      // Fehlerbehandlung
      catchError((error: Error) => {
        console.error('Wetter-API-Fehler:', error);
        return of<WeatherError>({ error: error.message });
      })
    ),
  } as ObservableInputTuple<ApiResponse>)
    .pipe(
      // Verarbeitung bei Abschluss
      finalize(() => {
        loadingIndicator.style.display = 'none';
        loadButton.disabled = false;
      })
    )
    .subscribe((results: ApiResponse) => {
      // Benutzerinformationen anzeigen
      const userInfo = document.createElement('div');
      userInfo.innerHTML = `
      <h4>Benutzerinformationen</h4>
      <p>Name: ${results.user.name}</p>
      <p>E-Mail: ${results.user.email}</p>
    `;
      userInfo.style.margin = '10px 0';
      userInfo.style.padding = '10px';
      userInfo.style.backgroundColor = '#f0f0f0';
      userInfo.style.borderRadius = '5px';
      resultContainer.appendChild(userInfo);

      // Beiträge anzeigen
      const postsInfo = document.createElement('div');
      postsInfo.innerHTML = `
      <h4>Beitragsliste (${results.posts.length} Stück)</h4>
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

      // Wetterinformationen anzeigen
      const weatherInfo = document.createElement('div');

      if ('error' in results.weather) {
        weatherInfo.innerHTML = `
        <h4>Wetterinformationen</h4>
        <p style="color: red;">Fehler: ${results.weather.error}</p>
      `;
      } else {
        weatherInfo.innerHTML = `
        <h4>Wetterinformationen</h4>
        <p>Stadt: ${results.weather.city}</p>
        <p>Temperatur: ${results.weather.temp}°C</p>
        <p>Zustand: ${results.weather.condition}</p>
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

## Abbrechbare Suchfunktion

Ein Beispiel zur Implementierung einer Timeout- oder abbrechbaren Suchfunktion mit der Kombination von `withLatestFrom` und `race`.

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

// Such-UI erstellen
const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Abbrechbare Suche:</h3>';
document.body.appendChild(searchContainer);

// Sucheingabefeld
const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Suchbegriff eingeben...';
searchInput.style.padding = '8px';
searchInput.style.width = '250px';
searchContainer.appendChild(searchInput);

// Abbrechen-Button
const cancelButton = document.createElement('button');
cancelButton.textContent = 'Abbrechen';
cancelButton.style.marginLeft = '10px';
cancelButton.style.padding = '8px 16px';
searchContainer.appendChild(cancelButton);

// Suchergebnisbereich
const resultsContainer = document.createElement('div');
resultsContainer.style.marginTop = '10px';
resultsContainer.style.minHeight = '200px';
resultsContainer.style.padding = '10px';
resultsContainer.style.border = '1px solid #ddd';
resultsContainer.style.borderRadius = '5px';
searchContainer.appendChild(resultsContainer);

// Suchanfrage simulieren
function searchApi(term: string) {
  console.log(`Suche nach "${term}" gestartet...`);

  // Suchergebnisse simulieren
  return of([
    `Suchergebnis für "${term}" 1`,
    `Suchergebnis für "${term}" 2`,
    `Suchergebnis für "${term}" 3`,
  ]).pipe(
    // 2-5 Sekunden Verzögerung zufällig festlegen
    delay(2000 + Math.random() * 3000),
    // Fehlerbehandlung
    catchError((err) => {
      console.error('Suchfehler:', err);
      return EMPTY;
    })
  );
}

// Abbrechen-Ereignis
const cancel$ = fromEvent(cancelButton, 'click');

// Suchereignis
const search$ = fromEvent(searchInput, 'input')
  .pipe(
    // Eingabewert abrufen
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // 300ms warten
    debounceTime(300),
    // Leere Suche ignorieren
    tap((term) => {
      if (term === '') {
        resultsContainer.innerHTML = '<p>Bitte geben Sie einen Suchbegriff ein</p>';
      }
    }),
    // Leere Suche nicht verarbeiten
    switchMap((term) => {
      if (term === '') {
        return EMPTY;
      }

      // Suche läuft Anzeige
      resultsContainer.innerHTML = '<p>Suche läuft...</p>';

      // Timeout-Verarbeitung (5 Sekunden)
      const timeout$ = timer(5000).pipe(
        tap(() => console.log('Suche ist abgelaufen')),
        map(() => ({ type: 'timeout', results: null }))
      );

      // API-Anfrage
      const request$ = searchApi(term).pipe(
        map((results) => ({ type: 'success', results })),
        // Bei Abbrechen-Button-Klick abbrechen
        takeUntil(
          cancel$.pipe(
            tap(() => {
              console.log('Suche wurde abgebrochen');
              resultsContainer.innerHTML = '<p>Suche wurde abgebrochen</p>';
            })
          )
        )
      );

      // Timeout oder Anfrage-Abschluss, je nachdem was früher
      return race(request$, timeout$);
    })
  )
  .subscribe((response) => {
    if (response.type === 'success') {
      // Suche erfolgreich
      resultsContainer.innerHTML = '<h4>Suchergebnisse:</h4>';

      if (response.results?.length === 0) {
        resultsContainer.innerHTML += '<p>Keine Ergebnisse gefunden</p>';
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
        '<p style="color: red;">Suche ist abgelaufen. Bitte versuchen Sie es erneut.</p>';
    }
  });

```

## Vergleich und Auswahlhilfe für Kombinationsoperatoren

Vergleicht die Unterschiede mehrerer Kombinationsoperatoren und unterstützt die Auswahl je nach Anwendungsfall.

| Operator | Timing | Ausgabe | Anwendungsfall |
|------------|------------|-----|------------|
| `merge` | Gleichzeitige Ausführung | Ausgabe in Auftrittsreihenfolge | Gleichzeitige Überwachung mehrerer Quellenereignisse |
| `concat` | Sequentielle Ausführung | Ausgabe in Reihenfolge | Asynchrone Aufgaben, bei denen Reihenfolge wichtig ist |
| `combineLatest` | Benötigt mindestens einen Wert von allen Quellen | Kombination aller neuesten Werte | Formular-Eingabevalidierung |
| `zip` | Benötigt Werte entsprechender Indizes von allen Quellen | Kombination von Werten pro Index | Synchronisierung verwandter Daten |
| `withLatestFrom` | Bei Wertausgabe der Hauptquelle | Hauptwert und neueste Werte anderer Quellen | Kombination von Hilfsdaten |
| `forkJoin` | Wenn alle Quellen abgeschlossen | Letzte Werte jeder Quelle | Mehrere API-Anfragen |
| `race` | Nur erste Quelle, die Wert ausgibt | Nur Werte des Gewinner-Streams | Timeout, Abbrechverarbeitung |

### Entscheidungsfluss für Operatorauswahl

1. **Sollen Werte von allen Quellen gleichzeitig empfangen werden?**
   - Ja → `merge`
   - Nein → Weiter

2. **Soll die Reihenfolge der Quellen beibehalten werden?**
   - Ja → `concat`
   - Nein → Weiter

3. **Wird Kombination der neuesten Werte jeder Quelle benötigt?**
   - Ja → Wann kombinieren?
     - Bei jedem neuen Wert einer beliebigen Quelle → `combineLatest`
     - Bei jedem Wert eines bestimmten Hauptstreams → `withLatestFrom`
   - Nein → Weiter

4. **Werden entsprechende Werte nach Indexreihenfolge benötigt?**
   - Ja → `zip`
   - Nein → Weiter

5. **Wird Ergebnis nach Abschluss aller Quellen benötigt?**
   - Ja → `forkJoin`
   - Nein → Weiter

6. **Wird nur das schnellste von mehreren alternativen Quellen benötigt?**
   - Ja → `race`
   - Nein → Zweck überdenken


## Switching-Strategie

Ein Beispiel zum dynamischen Wechseln mehrerer Datenquellen.

```ts
import { fromEvent, merge, interval, of } from 'rxjs';
import { map, switchMap, take, tap } from 'rxjs';

// UI-Elemente erstellen
const switchingContainer = document.createElement('div');
switchingContainer.innerHTML = '<h3>Datenquellenwechsel:</h3>';
document.body.appendChild(switchingContainer);

// Buttons erstellen
const source1Button = document.createElement('button');
source1Button.textContent = 'Quelle 1';
source1Button.style.margin = '5px';
source1Button.style.padding = '5px 10px';
switchingContainer.appendChild(source1Button);

const source2Button = document.createElement('button');
source2Button.textContent = 'Quelle 2';
source2Button.style.margin = '5px';
source2Button.style.padding = '5px 10px';
switchingContainer.appendChild(source2Button);

const source3Button = document.createElement('button');
source3Button.textContent = 'Quelle 3';
source3Button.style.margin = '5px';
source3Button.style.padding = '5px 10px';
switchingContainer.appendChild(source3Button);

// Ergebnisanzeigebereich
const resultsArea = document.createElement('div');
resultsArea.style.marginTop = '10px';
resultsArea.style.minHeight = '150px';
resultsArea.style.padding = '10px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.backgroundColor = '#f9f9f9';
switchingContainer.appendChild(resultsArea);

// 3 Datenquellen
function createSource1() {
  return interval(1000).pipe(
    take(5),
    map((val) => `Quelle 1: ${val}`),
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
    map((val) => `Quelle 2: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '#bbdefb';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource3() {
  return of('Quelle 3: A', 'Quelle 3: B', 'Quelle 3: C').pipe(
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '#ffccbc';
    })
  );
}

// Button-Klick-Ereignisse
const source1Click$ = fromEvent(source1Button, 'click').pipe(map(() => 1));

const source2Click$ = fromEvent(source2Button, 'click').pipe(map(() => 2));

const source3Click$ = fromEvent(source3Button, 'click').pipe(map(() => 3));

// Button-Klicks zusammenführen
merge(source1Click$, source2Click$, source3Click$)
  .pipe(
    // Zur ausgewählten Quelle wechseln
    switchMap((sourceId) => {
      // Ergebnisbereich löschen
      resultsArea.innerHTML = '';

      // Ausgewählte Quelle zurückgeben
      switch (sourceId) {
        case 1:
          return createSource1();
        case 2:
          return createSource2();
        case 3:
          return createSource3();
        default:
          return of('Keine Quelle ausgewählt');
      }
    })
  )
  .subscribe((value) => {
    // Ergebnis anzeigen
    const item = document.createElement('div');
    item.textContent = value;
    item.style.padding = '5px';
    item.style.margin = '2px 0';
    item.style.backgroundColor = 'white';
    item.style.borderRadius = '3px';
    resultsArea.appendChild(item);
  });

// Anfangsmeldung
const initialMessage = document.createElement('div');
initialMessage.textContent =
  'Bitte klicken Sie auf einen Button, um eine Datenquelle auszuwählen';
initialMessage.style.color = '#666';
resultsArea.appendChild(initialMessage);

```

## Bedingtes Merge

Ein Beispiel zur bedingten Auswahl von Datenquellen mit der Kombination von `merge` und `filter`.

```ts
import { merge, interval, fromEvent } from 'rxjs';
import {
  map,
  filter,
  takeUntil,
  withLatestFrom,
  startWith,
} from 'rxjs';

// UI-Elemente erstellen
const conditionalContainer = document.createElement('div');
conditionalContainer.innerHTML = '<h3>Bedingtes Merge:</h3>';
document.body.appendChild(conditionalContainer);

// Filtereinstellungen
const filterDiv = document.createElement('div');
filterDiv.style.marginBottom = '10px';
conditionalContainer.appendChild(filterDiv);

// Checkboxen erstellen
const slowCheck = document.createElement('input');
slowCheck.type = 'checkbox';
slowCheck.id = 'slowCheck';
slowCheck.checked = true;
filterDiv.appendChild(slowCheck);

const slowLabel = document.createElement('label');
slowLabel.htmlFor = 'slowCheck';
slowLabel.textContent = 'Langsame Quelle';
slowLabel.style.marginRight = '15px';
filterDiv.appendChild(slowLabel);

const fastCheck = document.createElement('input');
fastCheck.type = 'checkbox';
fastCheck.id = 'fastCheck';
fastCheck.checked = true;
filterDiv.appendChild(fastCheck);

const fastLabel = document.createElement('label');
fastLabel.htmlFor = 'fastCheck';
fastLabel.textContent = 'Schnelle Quelle';
fastLabel.style.marginRight = '15px';
filterDiv.appendChild(fastLabel);

const clickCheck = document.createElement('input');
clickCheck.type = 'checkbox';
clickCheck.id = 'clickCheck';
clickCheck.checked = true;
filterDiv.appendChild(clickCheck);

const clickLabel = document.createElement('label');
clickLabel.htmlFor = 'clickCheck';
clickLabel.textContent = 'Klick-Ereignis';
filterDiv.appendChild(clickLabel);

// Stopp-Button
const stopButton = document.createElement('button');
stopButton.textContent = 'Stopp';
stopButton.style.marginLeft = '15px';
filterDiv.appendChild(stopButton);

// Ergebnisanzeigebereich
const conditionalResults = document.createElement('div');
conditionalResults.style.height = '200px';
conditionalResults.style.overflowY = 'auto';
conditionalResults.style.padding = '10px';
conditionalResults.style.border = '1px solid #ddd';
conditionalResults.style.backgroundColor = '#f9f9f9';
conditionalContainer.appendChild(conditionalResults);

// 3 Datenquellen
// 1. Langsame Quelle (jede Sekunde)
const slow$ = interval(1000).pipe(map((val) => ({ type: 'slow', value: val })));

// 2. Schnelle Quelle (alle 300 Millisekunden)
const fast$ = interval(300).pipe(map((val) => ({ type: 'fast', value: val })));

// 3. Klick-Ereignis
const click$ = fromEvent(document.body, 'click').pipe(
  map((event) => ({
    type: 'click',
    value: {
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY,
    },
  }))
);

// Checkbox-Zustände überwachen
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

// Stopp-Ereignis
const stop$ = fromEvent(stopButton, 'click');

// Bedingtes Merge
merge(
  // Langsame Quelle mit Aktivierungszustand kombinieren
  slow$.pipe(
    withLatestFrom(slowEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Schnelle Quelle mit Aktivierungszustand kombinieren
  fast$.pipe(
    withLatestFrom(fastEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Klick-Quelle mit Aktivierungszustand kombinieren
  click$.pipe(
    withLatestFrom(clickEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  )
)
  .pipe(takeUntil(stop$))
  .subscribe((event) => {
    // Ergebnis anzeigen
    const item = document.createElement('div');

    switch (event.type) {
      case 'slow':
        item.textContent = `Langsame Quelle: ${event.value}`;
        item.style.color = '#1b5e20';
        break;
      case 'fast':
        item.textContent = `Schnelle Quelle: ${event.value}`;
        item.style.color = '#0d47a1';
        break;
      case 'click':
        const clickValue = event.value as { x: number; y: number };
        item.textContent = `Klick: X=${clickValue.x}, Y=${clickValue.y}`;
        item.style.color = '#bf360c';
        break;
    }

    item.style.padding = '3px';
    item.style.margin = '2px 0';
    conditionalResults.prepend(item); // Neue oben anzeigen
  });

```

## Zusammenfassung der Kombinationsoperator-Auswahl

| Zweck | Operator | Besonderheiten |
|------|--------------|------|
| Mehrere neueste Werte immer synchronisieren | `combineLatest` | Neueste Werte jedes Observables immer verbinden |
| Nach Abschluss aller gebündelt abrufen | `forkJoin` | Nur letzte Werte ausgeben (nur einmal) |
| Synchron nacheinander verarbeiten | `zip` | Je einen Wert von jedem Observable verbinden und ausgeben |
| Neueste Werte anderer bei Trigger referenzieren | `withLatestFrom` | Bei Hauptstream-Ausgabe neuesten Wert des Substreams anhängen |

## Zusammenfassung

Kombinationsoperatoren sind leistungsstarke Werkzeuge zum Zusammenführen mehrerer Datenquellen zu einem einzigen Stream. Durch Auswahl des geeigneten Operators können komplexe asynchrone Datenflüsse prägnant und deklarativ ausgedrückt werden.

### Punkte zur Beherrschung von Kombinationsoperatoren

1. **Auswahl entsprechend dem Anwendungsfall**: Jeder Operator ist für bestimmte Anwendungsfälle optimiert. Wählen Sie den geeigneten Operator entsprechend Ihrem Zweck.
2. **Verständnis des Ausgabetimings**: Das Verhalten von Kombinationsoperatoren hängt stark davon ab, wann Werte ausgegeben werden. Es ist wichtig, das Ausgabetiming jedes Operators zu verstehen.
3. **Berücksichtigung der Fehlerbehandlung**: Überlegen Sie das Verhalten bei Fehlern in einem Teil des kombinierten Streams (ob der gesamte Stream fehlschlägt oder die Verarbeitung teilweise fortgesetzt wird).
4. **Kenntnis der Abschlussbedingungen**: Verstehen Sie, wann kombinierte Streams abgeschlossen werden, und schließen Sie sie bei Bedarf explizit mit `takeUntil` usw. ab.
5. **Nutzung der Typsicherheit**: Durch Verwendung von TypeScript können Sie Kombinationsoperatoren typsicher verwenden. Besonders bei komplexen Kombinationen wird der Nutzen der Typen groß.

Kombinationsoperatoren können in vielen praktischen Szenarien wie UI-Ereignisverarbeitung, mehreren API-Anfragen und Formularvalidierung eingesetzt werden. Durch Beherrschung dieser Operatoren können Sie die wahre Kraft des reaktiven Programmierens von RxJS entfalten.

---
Weiter geht es mit [Fehlerbehandlung](/de/guide/error-handling/strategies), um zu lernen, wie man robusteren RxJS-Code schreibt!
