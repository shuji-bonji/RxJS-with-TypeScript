---
description: "fromFetch() ist eine Fetch-API-basierte Creation Function. Leicht, moderne Web-Standards, Service Worker-kompatibel. Manuelle Fehlerprüfung erforderlich."
---

# fromFetch()

[📘 RxJS Offizielle Dokumentation - fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` ist eine Creation Function zur Behandlung von Fetch-API-basierter HTTP-Kommunikation als Observable. Im Vergleich zu `ajax()` ist sie leicht und entspricht den neuesten Web-Standards.

## Grundlegende Verwendung

### Einfache GET-Anfrage

Das einfachste Beispiel mit `fromFetch()` ist die Übergabe einer URL und das manuelle Parsen der Response.

```typescript
import { of, switchMap, catchError } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const data$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1').pipe(
  switchMap(response => {
    if (response.ok) {
      // Bei erfolgreicher Response JSON parsen
      return response.json();
    } else {
      // Bei HTTP-Fehler Fehler werfen
      return throwError(() => new Error(`HTTP Error: ${response.status}`));
    }
  }),
  catchError(error => {
    console.error('Fehler:', error);
    return of({ error: true, message: error.message });
  })
);

data$.subscribe({
  next: data => console.log('Daten:', data),
  error: error => console.error('Abonnementfehler:', error),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Daten: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Abgeschlossen
```

> [!IMPORTANT]
> **Wichtiger Unterschied zu ajax()**
>
> - `fromFetch()` ruft auch bei HTTP-Fehlern (4xx, 5xx) nicht den `error`-Callback auf
> - Die `ok`-Eigenschaft der Response muss manuell überprüft werden
> - Parsing-Verarbeitung wie `.json()` muss ebenfalls manuell durchgeführt werden

## Verwendung nach HTTP-Methode

### GET-Anfrage

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface User {
  id: number;
  name: string;
  email: string;
}

const users$ = fromFetch('https://jsonplaceholder.typicode.com/users').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json() as Promise<User[]>;
  })
);

users$.subscribe({
  next: users => console.log('Benutzerliste:', users),
  error: error => console.error('Fehler:', error)
});
```

### POST-Anfrage

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

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

const createUser$ = fromFetch('https://api.example.com/users', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer token123'
  },
  body: JSON.stringify(newUser)
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json() as Promise<CreateUserResponse>;
  })
);

createUser$.subscribe({
  next: user => console.log('Erfolgreich erstellt:', user),
  error: error => console.error('Erstellung fehlgeschlagen:', error)
});
```

### PUT-Anfrage

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface UpdateUserRequest {
  name: string;
  email: string;
}

const updatedUser: UpdateUserRequest = {
  name: 'Jiro Tanaka',
  email: 'jiro@example.com'
};

const updateUser$ = fromFetch('https://api.example.com/users/1', {
  method: 'PUT',
  headers: {
    'Content-Type': 'application/json'
  },
  body: JSON.stringify(updatedUser)
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
);

updateUser$.subscribe({
  next: user => console.log('Erfolgreich aktualisiert:', user),
  error: error => console.error('Aktualisierung fehlgeschlagen:', error)
});
```

### DELETE-Anfrage

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const deleteUser$ = fromFetch('https://api.example.com/users/1', {
  method: 'DELETE',
  headers: {
    'Authorization': 'Bearer token123'
  }
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    // DELETE gibt normalerweise leere Response oder nur Status zurück
    return response.status === 204 ? of(null) : response.json();
  })
);

deleteUser$.subscribe({
  next: result => console.log('Erfolgreich gelöscht:', result),
  error: error => console.error('Löschen fehlgeschlagen:', error)
});
```

## Praktische Muster

### Universelle HTTP-Fehlerbehandlungsfunktion

Da `fromFetch()` manuelle Fehlerprüfung erfordert, ist die Erstellung einer universellen Funktion praktisch.

```typescript
import { Observable, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

function fetchJSON<T>(url: string, options?: RequestInit): Observable<T> {
  return fromFetch(url, options).pipe(
    switchMap(response => {
      if (!response.ok) {
        throw new Error(`HTTP Error ${response.status}: ${response.statusText}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// Verwendungsbeispiel
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

todo$.subscribe({
  next: todo => console.log('Todo:', todo),
  error: error => console.error('Fehler:', error)
});
```

### Detaillierte Verarbeitung nach HTTP-Statuscode

```typescript
import { throwError, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/data').pipe(
  switchMap(response => {
    switch (response.status) {
      case 200:
        return response.json();
      case 204:
        // No Content - Leere Response
        return of(null);
      case 401:
        throw new Error('Authentifizierung erforderlich');
      case 403:
        throw new Error('Zugriff verweigert');
      case 404:
        throw new Error('Ressource nicht gefunden');
      case 500:
        throw new Error('Serverfehler aufgetreten');
      default:
        throw new Error(`Unerwarteter HTTP-Status: ${response.status}`);
    }
  })
);

api$.subscribe({
  next: data => console.log('Daten:', data),
  error: error => console.error('Fehler:', error)
});
```

### Timeout und Retry

```typescript
import { switchMap, timeout, retry } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow-endpoint').pipe(
  timeout(5000), // Timeout nach 5 Sekunden
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  }),
  retry(2) // Bei Fehler 2 Mal wiederholen
);

api$.subscribe({
  next: data => console.log('Daten:', data),
  error: error => console.error('Fehler:', error)
});
```

### Anfragestornierung (AbortController)

`fromFetch()` unterstützt Anfragestornierung mit dem `AbortController` der Fetch API.

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const controller = new AbortController();
const signal = controller.signal;

const api$ = fromFetch('https://api.example.com/data', {
  signal // AbortController-Signal übergeben
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
);

const subscription = api$.subscribe({
  next: data => console.log('Daten:', data),
  error: error => console.error('Fehler:', error)
});

// Nach 3 Sekunden Anfrage stornieren
setTimeout(() => {
  controller.abort();
  // Oder subscription.unsubscribe();
}, 3000);
```

> [!TIP]
> **Automatische Stornierung durch RxJS**
>
> Durch einfachen Aufruf von `unsubscribe()` storniert RxJS intern die Anfrage mit `AbortController`. Manuelle Konfiguration von `AbortController` ist nicht erforderlich.

### Suche basierend auf Benutzereingabe (switchMap)

```typescript
import { fromEvent, map, debounceTime, distinctUntilChanged, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface SearchResult {
  id: number;
  title: string;
}

const searchInput = document.querySelector('#search') as HTMLInputElement;

const search$ = fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),
  distinctUntilChanged(),
  switchMap(query => {
    if (query.length === 0) {
      return of([]);
    }
    return fromFetch(`https://api.example.com/search?q=${encodeURIComponent(query)}`).pipe(
      switchMap(response => {
        if (!response.ok) {
          throw new Error(`HTTP Error: ${response.status}`);
        }
        return response.json() as Promise<SearchResult[]>;
      })
    );
  })
);

search$.subscribe({
  next: results => console.log('Suchergebnisse:', results),
  error: error => console.error('Suchfehler:', error)
});
```

### Parallele Ausführung mehrerer Anfragen

```typescript
import { forkJoin, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface User {
  id: number;
  name: string;
}

interface Post {
  id: number;
  title: string;
}

const users$ = fromFetch('https://jsonplaceholder.typicode.com/users').pipe(
  switchMap(response => response.json() as Promise<User[]>)
);

const posts$ = fromFetch('https://jsonplaceholder.typicode.com/posts').pipe(
  switchMap(response => response.json() as Promise<Post[]>)
);

forkJoin({
  users: users$,
  posts: posts$
}).subscribe({
  next: ({ users, posts }) => {
    console.log('Users:', users);
    console.log('Posts:', posts);
  },
  error: error => console.error('Eine Anfrage ist fehlgeschlagen:', error)
});
```

## Häufige Anwendungsfälle

### 1. Anfrage mit Authentifizierungstoken

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

function getAuthToken(): string {
  return localStorage.getItem('authToken') || '';
}

function fetchWithAuth<T>(url: string, options: RequestInit = {}): Observable<T> {
  return fromFetch(url, {
    ...options,
    headers: {
      ...options.headers,
      'Authorization': `Bearer ${getAuthToken()}`,
      'Content-Type': 'application/json'
    }
  }).pipe(
    switchMap(response => {
      if (response.status === 401) {
        throw new Error('Authentifizierung erforderlich. Bitte erneut anmelden.');
      }
      if (!response.ok) {
        throw new Error(`HTTP Error: ${response.status}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// Verwendungsbeispiel
interface UserProfile {
  id: number;
  name: string;
  email: string;
}

const profile$ = fetchWithAuth<UserProfile>('https://api.example.com/profile');

profile$.subscribe({
  next: profile => console.log('Profil:', profile),
  error: error => console.error('Fehler:', error)
});
```

### 2. Datei-Download (Blob)

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const downloadFile$ = fromFetch('https://api.example.com/files/report.pdf').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    // Als Blob abrufen
    return response.blob();
  })
);

downloadFile$.subscribe({
  next: blob => {
    // Download-Link aus Blob erstellen
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'report.pdf';
    a.click();
    window.URL.revokeObjectURL(url);
    console.log('Download abgeschlossen');
  },
  error: error => console.error('Download-Fehler:', error)
});
```

### 3. GraphQL-Abfrage

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface GraphQLResponse<T> {
  data?: T;
  errors?: Array<{ message: string }>;
}

interface User {
  id: string;
  name: string;
  email: string;
}

function graphqlQuery<T>(query: string, variables?: any): Observable<T> {
  return fromFetch('https://api.example.com/graphql', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ query, variables })
  }).pipe(
    switchMap(response => {
      if (!response.ok) {
        throw new Error(`HTTP Error: ${response.status}`);
      }
      return response.json() as Promise<GraphQLResponse<T>>;
    }),
    map(result => {
      if (result.errors) {
        throw new Error(result.errors.map(e => e.message).join(', '));
      }
      if (!result.data) {
        throw new Error('Keine Daten zurückgegeben');
      }
      return result.data;
    })
  );
}

// Verwendungsbeispiel
const query = `
  query GetUser($id: ID!) {
    user(id: $id) {
      id
      name
      email
    }
  }
`;

const user$ = graphqlQuery<{ user: User }>(query, { id: '1' });

user$.subscribe({
  next: ({ user }) => console.log('Benutzer:', user),
  error: error => console.error('Fehler:', error)
});
```

### 4. API mit Paginierung

```typescript
import { expand, takeWhile, reduce, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface PaginatedResponse<T> {
  data: T[];
  page: number;
  totalPages: number;
}

function fetchAllPages<T>(baseUrl: string): Observable<T[]> {
  return fromFetch(`${baseUrl}?page=1`).pipe(
    switchMap(response => response.json() as Promise<PaginatedResponse<T>>),
    expand(response =>
      response.page < response.totalPages
        ? fromFetch(`${baseUrl}?page=${response.page + 1}`).pipe(
            switchMap(res => res.json() as Promise<PaginatedResponse<T>>)
          )
        : []
    ),
    takeWhile(response => response.page <= response.totalPages, true),
    reduce((acc, response) => [...acc, ...response.data], [] as T[])
  );
}

// Verwendungsbeispiel
interface Item {
  id: number;
  name: string;
}

const allItems$ = fetchAllPages<Item>('https://api.example.com/items');

allItems$.subscribe({
  next: items => console.log('Alle Elemente:', items),
  error: error => console.error('Fehler:', error)
});
```

## fromFetch() Optionen

`fromFetch()` kann die `RequestInit`-Optionen der Fetch API direkt verwenden.

```typescript
interface RequestInit {
  method?: string;              // HTTP-Methode (GET, POST, PUT, DELETE etc.)
  headers?: HeadersInit;        // Anfrage-Header
  body?: BodyInit | null;       // Anfrage-Body
  mode?: RequestMode;           // cors, no-cors, same-origin
  credentials?: RequestCredentials; // omit, same-origin, include
  cache?: RequestCache;         // Cache-Modus
  redirect?: RequestRedirect;   // Redirect-Verarbeitung
  referrer?: string;            // Referrer
  integrity?: string;           // Subresource Integrity
  signal?: AbortSignal;         // AbortController-Signal
}
```

## Vergleich ajax() vs fromFetch()

| Funktion | ajax() | fromFetch() |
|----------|--------|-------------|
| Basistechnologie | XMLHttpRequest | Fetch API |
| Automatisches JSON-Parsing | ✅ `getJSON()` | ❌ Manuell `.json()` |
| Automatische HTTP-Fehlererkennung | ✅ Automatischer Fehler bei 4xx/5xx | ❌ Manueller `response.ok`-Check |
| Fortschrittsüberwachung | ✅ | ❌ |
| Timeout | ✅ Integriert | ❌ Mit RxJS `timeout()` implementieren |
| Anfragestornierung | ✅ unsubscribe() | ✅ unsubscribe() oder AbortController |
| IE11-Unterstützung | ✅ | ❌ Polyfill erforderlich |
| Bundle-Größe | Etwas größer | Klein |
| Service Worker-Unterstützung | ❌ | ✅ |

> [!TIP]
> **Auswahlpunkte**
>
> - **Nur moderne Browser unterstützt**: `fromFetch()` empfohlen
> - **Legacy-Browser-Unterstützung erforderlich**: `ajax()` verwenden
> - **Fortschrittsüberwachung erforderlich**: `ajax()` verwenden
> - **Leichte HTTP-Kommunikation**: `fromFetch()` optimal
> - **Verwendung in Service Worker**: Nur `fromFetch()` unterstützt

## Häufige Fehler und Lösungen

### 1. HTTP-Fehler werden nicht im `error`-Callback erfasst

**Problem:**
```typescript
// ❌ Auch bei 404-Fehler wird next aufgerufen
fromFetch('https://api.example.com/not-found').subscribe({
  next: response => console.log('Erfolg:', response), // ← Wird auch bei 404 aufgerufen
  error: error => console.error('Fehler:', error)
});
```

**Lösung:**
```typescript
// ✅ response.ok manuell prüfen
fromFetch('https://api.example.com/not-found').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
).subscribe({
  next: data => console.log('Daten:', data),
  error: error => console.error('Fehler:', error) // ← Wird aufgerufen
});
```

### 2. CORS-Fehler

**Lösung:**
- CORS-Header serverseitig konfigurieren
- `mode: 'cors'` explizit angeben
- Während Entwicklung Proxy-Server verwenden

```typescript
fromFetch('https://api.example.com/data', {
  mode: 'cors',
  credentials: 'include' // Wenn Cookies eingeschlossen werden sollen
});
```

### 3. Timeout-Implementierung

Da die Fetch API keine Timeout-Funktion hat, RxJS `timeout()` verwenden.

```typescript
import { timeout, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow').pipe(
  timeout(5000), // Timeout nach 5 Sekunden
  switchMap(response => response.json())
);
```

## Best Practices

### 1. Universelle fetchJSON-Funktion erstellen

```typescript
function fetchJSON<T>(url: string, options?: RequestInit): Observable<T> {
  return fromFetch(url, options).pipe(
    switchMap(response => {
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }
      return response.json() as Promise<T>;
    })
  );
}
```

### 2. TypeScript-Typen nutzen

```typescript
// ✅ Gutes Beispiel: Typ explizit angeben
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Schlechtes Beispiel: Keine Typangabe
const todo$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1')
  .pipe(switchMap(res => res.json()));
```

### 3. Fehlerbehandlung immer implementieren

```typescript
// ✅ Gutes Beispiel: response.ok und catchError
const api$ = fromFetch('/api/data').pipe(
  switchMap(response => {
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return response.json();
  }),
  catchError(error => {
    console.error('Fehler:', error);
    return of(defaultValue);
  })
);
```

### 4. Abmeldung nicht vergessen

```typescript
// ✅ Gutes Beispiel: Automatische Abmeldung mit takeUntil
class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    fromFetch('/api/data')
      .pipe(
        switchMap(res => res.json()),
        takeUntil(this.destroy$)
      )
      .subscribe(...);
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Zusammenfassung

`fromFetch()` ist eine leichte HTTP-Kommunikations-Creation-Function basierend auf der modernen Fetch API.

**Hauptmerkmale:**
- Fetch-API-basiert, entspricht neuesten Web-Standards
- Leicht mit kleiner Bundle-Größe
- Kann auch in Service Worker verwendet werden
- Manuelle Fehlerprüfung und Response-Parsing erforderlich

**Einsatzszenarien:**
- Wenn nur moderne Browser unterstützt werden
- Wenn Bundle-Größe klein gehalten werden soll
- Wenn HTTP-Kommunikation in Service Worker durchgeführt werden soll
- Wenn Fetch-API-Funktionen (Request/Response-Objekte etc.) direkt verwendet werden sollen

**Beachtenswerte Punkte:**
- Auch bei HTTP-Fehlern wird `error`-Callback nicht aufgerufen (manueller `response.ok`-Check)
- JSON-Parsing manuell durchführen (`response.json()`)
- Fortschrittsüberwachung nicht unterstützt
- In Legacy-Browsern wie IE11 ist Polyfill erforderlich

**Empfohlene Verwendung:**
- Universelle `fetchJSON()`-Funktion erstellen und wiederverwenden
- TypeScript-Typen für Typsicherheit nutzen
- Fehlerbehandlung immer implementieren
- Bei Nichtbedarf immer abmelden

## Verwandte Seiten

- [ajax()](/de/guide/creation-functions/http-communication/ajax) - XMLHttpRequest-basierte HTTP-Kommunikation
- [HTTP-Kommunikations-Creation-Functions](/de/guide/creation-functions/http-communication/) - Vergleich von ajax() und fromFetch()
- [switchMap()](/de/guide/operators/transformation/switchMap) - Praktischer Operator für HTTP-Anfragestornierung
- [Fehlerbehandlungsstrategien](/de/guide/error-handling/strategies) - HTTP-Kommunikations-Fehlerbehandlungsmuster

## Referenzressourcen

- [RxJS Offizielle Dokumentation - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/de/docs/Web/API/Fetch_API)
- [MDN Web Docs - AbortController](https://developer.mozilla.org/de/docs/Web/API/AbortController)
