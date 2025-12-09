---
description: "ajax() ist eine XMLHttpRequest-basierte Creation Function. Unterstützt GET/POST/PUT/DELETE, Fortschrittsüberwachung und automatisches JSON-Parsing."
---

# ajax()

[📘 RxJS Offizielle Dokumentation - ajax](https://rxjs.dev/api/ajax/ajax)

`ajax()` ist eine Creation Function zur Behandlung von XMLHttpRequest-basierter HTTP-Kommunikation als Observable. Sie unterstützt HTTP-Methoden wie GET, POST, PUT, DELETE und bietet praktische Funktionen wie Fortschrittsüberwachung und Timeout-Verarbeitung.

## Grundlegende Verwendung

### Einfache GET-Anfrage

Das einfachste Beispiel mit `ajax()` ist die Übergabe einer URL als String.

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax('https://jsonplaceholder.typicode.com/todos/1');

api$.subscribe({
  next: response => console.log('Response:', response),
  error: error => console.error('Fehler:', error),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Response: {
//   status: 200,
//   response: { userId: 1, id: 1, title: "delectus aut autem", completed: false },
//   ...
// }
// Abgeschlossen
```

### JSON-Abruf mit getJSON()

Beim Abrufen von Daten aus JSON-APIs ist die Verwendung von `ajax.getJSON()` praktisch. Es parst die Response automatisch und gibt nur die `response`-Eigenschaft zurück.

```typescript
import { ajax } from 'rxjs/ajax';

interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todos$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

todos$.subscribe({
  next: todo => console.log('Todo:', todo),
  error: error => console.error('Fehler:', error),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Todo: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Abgeschlossen
```

> [!TIP]
> **TypeScript-Typsicherheit**
>
> Durch Angabe eines generischen Typs in `ajax.getJSON<T>()` kann die Typsicherheit der Response sichergestellt werden.

## Verwendung nach HTTP-Methode

### GET-Anfrage

```typescript
import { ajax } from 'rxjs/ajax';

// Methode 1: Einfache String-Angabe
const get1$ = ajax('https://api.example.com/users');

// Methode 2: Automatisches Parsing mit getJSON()
const get2$ = ajax.getJSON('https://api.example.com/users');

// Methode 3: Detaillierte Konfiguration
const get3$ = ajax({
  url: 'https://api.example.com/users',
  method: 'GET',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer token123'
  }
});
```

### POST-Anfrage

```typescript
import { ajax } from 'rxjs/ajax';

interface CreateUserRequest {
  name: string;
  email: string;
}

interface CreateUserResponse {
  id: number;
  name: string;
  email: string;
  createdAt: string;
}

const newUser: CreateUserRequest = {
  name: 'Taro Yamada',
  email: 'taro@example.com'
};

// Methode 1: Verwendung von ajax.post()
const post1$ = ajax.post<CreateUserResponse>(
  'https://api.example.com/users',
  newUser,
  { 'Content-Type': 'application/json' }
);

// Methode 2: Detaillierte Konfiguration
const post2$ = ajax<CreateUserResponse>({
  url: 'https://api.example.com/users',
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer token123'
  },
  body: newUser
});

post1$.subscribe({
  next: response => console.log('Erfolgreich erstellt:', response.response),
  error: error => console.error('Erstellung fehlgeschlagen:', error)
});
```

### PUT-Anfrage

```typescript
import { ajax } from 'rxjs/ajax';

interface UpdateUserRequest {
  name: string;
  email: string;
}

const updatedUser: UpdateUserRequest = {
  name: 'Jiro Tanaka',
  email: 'jiro@example.com'
};

const put$ = ajax.put(
  'https://api.example.com/users/1',
  updatedUser,
  { 'Content-Type': 'application/json' }
);

put$.subscribe({
  next: response => console.log('Erfolgreich aktualisiert:', response.response),
  error: error => console.error('Aktualisierung fehlgeschlagen:', error)
});
```

### PATCH-Anfrage

```typescript
import { ajax } from 'rxjs/ajax';

interface PatchUserRequest {
  email?: string;
}

const patch$ = ajax.patch(
  'https://api.example.com/users/1',
  { email: 'new-email@example.com' } as PatchUserRequest,
  { 'Content-Type': 'application/json' }
);

patch$.subscribe({
  next: response => console.log('Teilaktualisierung erfolgreich:', response.response),
  error: error => console.error('Teilaktualisierung fehlgeschlagen:', error)
});
```

### DELETE-Anfrage

```typescript
import { ajax } from 'rxjs/ajax';

const delete$ = ajax.delete('https://api.example.com/users/1');

delete$.subscribe({
  next: response => console.log('Erfolgreich gelöscht:', response),
  error: error => console.error('Löschen fehlgeschlagen:', error)
});
```

## Praktische Muster

### Fehlerbehandlung und Retry

Bei HTTP-Kommunikation müssen Netzwerk- und Serverfehler behandelt werden.

```typescript
import { of, retry, catchError, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout nach 5 Sekunden
  retry(2), // Bei Fehler 2 Mal wiederholen
  catchError(error => {
    console.error('Fehler beim Abrufen des Benutzers:', error);
    // Standardwert zurückgeben
    return of({
      id: 0,
      name: 'Unbekannt',
      email: 'unknown@example.com'
    } as User);
  })
);

fetchUser$.subscribe({
  next: user => console.log('Benutzer:', user),
  error: error => console.error('Kritischer Fehler:', error)
});
```

### Bedingte Verzweigung nach HTTP-Statuscode

```typescript
import { throwError, catchError } from 'rxjs';
import { ajax, AjaxError } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/data').pipe(
  catchError((error: AjaxError) => {
    if (error.status === 404) {
      console.error('Ressource nicht gefunden');
    } else if (error.status === 401) {
      console.error('Authentifizierung erforderlich');
    } else if (error.status === 500) {
      console.error('Serverfehler aufgetreten');
    } else {
      console.error('Unerwarteter Fehler:', error);
    }
    return throwError(() => error);
  })
);
```

### Parallele Ausführung mehrerer Anfragen

```typescript
import { ajax } from 'rxjs/ajax';
import { forkJoin } from 'rxjs';

interface User {
  id: number;
  name: string;
}

interface Post {
  id: number;
  title: string;
  userId: number;
}

interface Comment {
  id: number;
  body: string;
  postId: number;
}

const users$ = ajax.getJSON<User[]>('https://jsonplaceholder.typicode.com/users');
const posts$ = ajax.getJSON<Post[]>('https://jsonplaceholder.typicode.com/posts');
const comments$ = ajax.getJSON<Comment[]>('https://jsonplaceholder.typicode.com/comments');

// Warten bis alle Anfragen abgeschlossen sind
forkJoin({
  users: users$,
  posts: posts$,
  comments: comments$
}).subscribe({
  next: ({ users, posts, comments }) => {
    console.log('Users:', users);
    console.log('Posts:', posts);
    console.log('Comments:', comments);
  },
  error: error => console.error('Eine Anfrage ist fehlgeschlagen:', error)
});
```

### Suche basierend auf Benutzereingabe (switchMap)

```typescript
import { fromEvent, map, debounceTime, distinctUntilChanged, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface SearchResult {
  id: number;
  title: string;
}

const searchInput = document.querySelector('#search') as HTMLInputElement;

const search$ = fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // 300ms warten
  distinctUntilChanged(), // Gleiche Werte ignorieren
  switchMap(query => {
    if (query.length === 0) {
      return of([]);
    }
    // Bei neuer Sucheingabe vorherige Anfrage stornieren
    return ajax.getJSON<SearchResult[]>(`https://api.example.com/search?q=${query}`);
  })
);

search$.subscribe({
  next: results => console.log('Suchergebnisse:', results),
  error: error => console.error('Suchfehler:', error)
});
```

> [!IMPORTANT]
> **Wichtigkeit von switchMap()**
>
> Durch Verwendung von `switchMap()` wird bei Eingabe einer neuen Suchanfrage die vorherige HTTP-Anfrage automatisch storniert. Dies verhindert, dass alte Suchergebnisse neue überschreiben.

### Fortschrittsüberwachung (Datei-Upload)

`ajax()` kann Upload- und Download-Fortschritt mit dem `progress`-Event von `XMLHttpRequest` überwachen.

```typescript
import { tap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const fileInput = document.querySelector('#file') as HTMLInputElement;
const file = fileInput.files?.[0];

if (file) {
  const formData = new FormData();
  formData.append('file', file);

  const upload$ = ajax({
    url: 'https://api.example.com/upload',
    method: 'POST',
    body: formData,
    // Fortschrittsereignisse aktivieren
    progressSubscriber: {
      next: (progress) => {
        const percentage = (progress.loaded / progress.total) * 100;
        console.log(`Upload-Fortschritt: ${percentage.toFixed(2)}%`);
      }
    }
  });

  upload$.subscribe({
    next: response => console.log('Upload abgeschlossen:', response),
    error: error => console.error('Upload fehlgeschlagen:', error)
  });
}
```

### Benutzerdefinierte Header und Cross-Domain-Anfragen

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected-resource',
  method: 'GET',
  headers: {
    'Authorization': 'Bearer your-token-here',
    'X-Custom-Header': 'CustomValue'
  },
  crossDomain: true, // CORS-Anfrage
  withCredentials: true // Cookies einschließen
});

api$.subscribe({
  next: response => console.log('Response:', response),
  error: error => console.error('Fehler:', error)
});
```

## Häufige Anwendungsfälle

### 1. API-Aufruf mit Paginierung

```typescript
import { expand, takeWhile, reduce } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface PaginatedResponse {
  data: any[];
  page: number;
  totalPages: number;
}

const fetchAllPages$ = ajax.getJSON<PaginatedResponse>(
  'https://api.example.com/items?page=1'
).pipe(
  expand(response =>
    response.page < response.totalPages
      ? ajax.getJSON<PaginatedResponse>(`https://api.example.com/items?page=${response.page + 1}`)
      : []
  ),
  takeWhile(response => response.page <= response.totalPages, true),
  reduce((acc, response) => [...acc, ...response.data], [] as any[])
);

fetchAllPages$.subscribe({
  next: allItems => console.log('Alle Elemente:', allItems),
  error: error => console.error('Fehler:', error)
});
```

### 2. Polling (Regelmäßiger Datenabruf)

```typescript
import { interval, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Status {
  status: string;
  lastUpdate: string;
}

// API alle 5 Sekunden aufrufen
const polling$ = interval(5000).pipe(
  switchMap(() => ajax.getJSON<Status>('https://api.example.com/status'))
);

const subscription = polling$.subscribe({
  next: status => console.log('Status:', status),
  error: error => console.error('Fehler:', error)
});

// Nach 30 Sekunden stoppen
setTimeout(() => subscription.unsubscribe(), 30000);
```

### 3. Abhängige Anfragen

```typescript
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
}

interface UserDetails {
  userId: number;
  address: string;
  phone: string;
}

// Zuerst Benutzerinformationen abrufen, dann Detailinformationen
const userWithDetails$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  switchMap(user =>
    ajax.getJSON<UserDetails>(`https://api.example.com/users/${user.id}/details`).pipe(
      map(details => ({ ...user, ...details }))
    )
  )
);

userWithDetails$.subscribe({
  next: userWithDetails => console.log('Benutzerdetails:', userWithDetails),
  error: error => console.error('Fehler:', error)
});
```

## ajax() Optionen

`ajax()` bietet Optionen für detaillierte Konfiguration.

```typescript
interface AjaxConfig {
  url: string;                    // Anfrage-URL
  method?: string;                // HTTP-Methode (GET, POST, PUT, DELETE etc.)
  headers?: object;               // Anfrage-Header
  body?: any;                     // Anfrage-Body
  timeout?: number;               // Timeout-Zeit (Millisekunden)
  responseType?: string;          // Response-Typ (json, text, blob etc.)
  crossDomain?: boolean;          // Ob CORS-Anfrage
  withCredentials?: boolean;      // Ob Cookies eingeschlossen werden
  progressSubscriber?: Subscriber; // Subscriber für Fortschrittsüberwachung
}
```

## Häufige Fehler und Lösungen

### 1. CORS-Fehler

**Fehlerbeispiel:**
```
Access to XMLHttpRequest at 'https://api.example.com' from origin 'http://localhost:3000'
has been blocked by CORS policy
```

**Lösung:**
- CORS-Header serverseitig konfigurieren
- Proxy-Server verwenden
- Während Entwicklung `crossDomain: true` und `withCredentials: false` versuchen

### 2. Netzwerk-Timeout

**Lösung:**
```typescript
import { timeout, retry } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/slow-endpoint').pipe(
  timeout(10000), // Timeout nach 10 Sekunden
  retry(2) // 2 Mal wiederholen
);
```

### 3. Authentifizierungsfehler (401 Unauthorized)

**Lösung:**
```typescript
import { throwError, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected',
  headers: {
    'Authorization': `Bearer ${getAccessToken()}`
  }
}).pipe(
  catchError(error => {
    if (error.status === 401) {
      // Token aktualisieren und erneut versuchen
      return refreshToken().pipe(
        switchMap(newToken =>
          ajax({
            url: 'https://api.example.com/protected',
            headers: { 'Authorization': `Bearer ${newToken}` }
          })
        )
      );
    }
    return throwError(() => error);
  })
);
```

## Vergleich ajax() vs fromFetch()

| Funktion | ajax() | fromFetch() |
|----------|--------|-------------|
| Automatisches JSON-Parsing | ✅ `getJSON()` | ❌ Manuell `.json()` |
| Fortschrittsüberwachung | ✅ | ❌ |
| Automatische HTTP-Fehlererkennung | ✅ | ❌ |
| Bundle-Größe | Etwas größer | Klein |
| IE11-Unterstützung | ✅ | ❌ |

> [!TIP]
> **Auswahlpunkte**
>
> - **Fortschrittsüberwachung erforderlich**: `ajax()` verwenden
> - **Legacy-Browser-Unterstützung**: `ajax()` verwenden
> - **Leichte HTTP-Kommunikation**: `fromFetch()` in Betracht ziehen
> - **Einfacher JSON-Abruf**: `ajax.getJSON()` ist am einfachsten

## Best Practices

### 1. Typsicherheit gewährleisten

```typescript
// ✅ Gutes Beispiel: Generischen Typ angeben
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todos$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Schlechtes Beispiel: Keine Typangabe
const todos$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
```

### 2. Fehlerbehandlung immer implementieren

```typescript
// ✅ Gutes Beispiel: Fehlerverarbeitung mit catchError
const api$ = ajax.getJSON('/api/data').pipe(
  catchError(error => {
    console.error('Fehler:', error);
    return of(defaultValue);
  })
);

// ❌ Schlechtes Beispiel: Keine Fehlerbehandlung
const api$ = ajax.getJSON('/api/data');
```

### 3. Abmeldung nicht vergessen

```typescript
// ✅ Gutes Beispiel: Abmeldung bei Komponentenzerstörung
class MyComponent {
  private subscription: Subscription;

  ngOnInit() {
    this.subscription = ajax.getJSON('/api/data').subscribe(...);
  }

  ngOnDestroy() {
    this.subscription.unsubscribe();
  }
}

// Oder takeUntil verwenden
class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    ajax.getJSON('/api/data')
      .pipe(takeUntil(this.destroy$))
      .subscribe(...);
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Zusammenfassung

`ajax()` ist eine leistungsstarke Creation Function für HTTP-Kommunikation in RxJS.

**Hauptmerkmale:**
- XMLHttpRequest-basiert, unterstützt breite Browser-Kompatibilität
- Einfacher JSON-Abruf mit `getJSON()`
- Praktische Funktionen wie Fortschrittsüberwachung, Timeout, Retry
- Automatische HTTP-Fehlererkennung

**Einsatzszenarien:**
- Wenn Legacy-Browser-Unterstützung (IE11 etc.) erforderlich ist
- Wenn Datei-Upload/Download-Fortschritt angezeigt werden soll
- Für einfache und klare JSON-API-Aufrufe

**Beachtenswerte Punkte:**
- Fehlerbehandlung immer implementieren
- Bei Nichtbedarf immer abmelden
- TypeScript-Typen nutzen für Typsicherheit

## Verwandte Seiten

- [fromFetch()](/de/guide/creation-functions/http-communication/fromFetch) - Fetch-API-basierte HTTP-Kommunikation
- [HTTP-Kommunikations-Creation-Functions](/de/guide/creation-functions/http-communication/) - Vergleich von ajax() und fromFetch()
- [switchMap()](/de/guide/operators/transformation/switchMap) - Praktischer Operator für HTTP-Anfragestornierung
- [Fehlerbehandlungsstrategien](/de/guide/error-handling/strategies) - HTTP-Kommunikations-Fehlerbehandlungsmuster

## Referenzressourcen

- [RxJS Offizielle Dokumentation - ajax](https://rxjs.dev/api/ajax/ajax)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/de/docs/Web/API/XMLHttpRequest)
- [Learn RxJS - ajax](https://www.learnrxjs.io/learn-rxjs/operators/creation/ajax)
