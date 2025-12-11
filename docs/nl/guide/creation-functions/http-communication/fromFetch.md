---
description: fromFetch() is een RxJS Creation Function die Fetch API-gebaseerde HTTP-communicatie als Observable afhandelt, ondersteunt HTTP-methoden zoals GET, POST, PUT, DELETE, etc. Het is lichtgewicht, modern, voldoet aan webstandaarden en is compatibel met Service Workers voor moderne HTTP-communicatie. Handmatige foutcontrole en JSON-parsing zijn vereist, maar de bundelgrootte is klein en type-veilige HTTP-communicatie is mogelijk.
---

# fromFetch()

[📘 RxJS Official Documentation - fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` is een Creation Function voor het afhandelen van HTTP-communicatie als Observable gebaseerd op de moderne Fetch API. Het is lichtgewicht vergeleken met `ajax()` en voldoet aan moderne webstandaarden.

## Basisgebruik

### Eenvoudig GET-verzoek

Het eenvoudigste voorbeeld van het gebruik van `fromFetch()` is het doorgeven van een URL en het handmatig parsen van de response.

```typescript
import { of, switchMap, catchError, throwError } from 'rxjs';
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
      // Als response succesvol is, parse JSON
      return response.json();
    } else {
      // Bij HTTP-fout, throw error
      return throwError(() => new Error(`HTTP Error: ${response.status}`));
    }
  }),
  catchError(error => {
    console.error('Error:', error);
    return of({ error: true, message: error.message });
  })
);

data$.subscribe({
  next: data => console.log('Data:', data),
  error: error => console.error('Subscription error:', error),
  complete: () => console.log('Complete')
});

// Output:
// Data: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Complete
```

> [!IMPORTANT]
> **Belangrijk Verschil met ajax()**
>
> - `fromFetch()` roept `error` callback niet aan bij HTTP-fouten (4xx, 5xx)
> - De `ok` property van de response moet handmatig gecontroleerd worden
> - Parsing-operaties zoals `.json()` worden ook handmatig gedaan

## Gebruik per HTTP-methode

### GET-verzoek

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
  next: users => console.log('Gebruikerslijst:', users),
  error: error => console.error('Error:', error)
});
```

### POST-verzoek

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
  name: 'Jan Jansen',
  email: 'jan@example.com'
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
  next: user => console.log('Creatie geslaagd:', user),
  error: error => console.error('Creatie mislukt:', error)
});
```

### PUT-verzoek

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface UpdateUserRequest {
  name: string;
  email: string;
}

const updatedUser: UpdateUserRequest = {
  name: 'Piet Pietersen',
  email: 'piet@example.com'
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
  next: user => console.log('Update geslaagd:', user),
  error: error => console.error('Update mislukt:', error)
});
```

### DELETE-verzoek

```typescript
import { switchMap, of } from 'rxjs';
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
    // DELETE retourneert meestal lege response of alleen status
    return response.status === 204 ? of(null) : response.json();
  })
);

deleteUser$.subscribe({
  next: result => console.log('Verwijdering geslaagd:', result),
  error: error => console.error('Verwijdering mislukt:', error)
});
```

## Praktische Patronen

### Generieke HTTP Error Handling Functie

Omdat `fromFetch()` handmatige foutcontrole vereist, is het nuttig om een generieke functie te maken.

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

// Gebruiksvoorbeeld
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

todo$.subscribe({
  next: todo => console.log('Todo:', todo),
  error: error => console.error('Error:', error)
});
```

### Gedetailleerde Verwerking per HTTP-statuscode

```typescript
import { throwError, switchMap, of } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/data').pipe(
  switchMap(response => {
    switch (response.status) {
      case 200:
        return response.json();
      case 204:
        // No Content - lege response
        return of(null);
      case 401:
        throw new Error('Authenticatie vereist');
      case 403:
        throw new Error('Toegang geweigerd');
      case 404:
        throw new Error('Resource niet gevonden');
      case 500:
        throw new Error('Serverfout opgetreden');
      default:
        throw new Error(`Onverwachte HTTP-status: ${response.status}`);
    }
  })
);

api$.subscribe({
  next: data => console.log('Data:', data),
  error: error => console.error('Error:', error)
});
```

### Timeout en Retry

```typescript
import { switchMap, timeout, retry } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow-endpoint').pipe(
  timeout(5000), // Timeout na 5 seconden
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  }),
  retry(2) // Retry twee keer bij falen
);

api$.subscribe({
  next: data => console.log('Data:', data),
  error: error => console.error('Error:', error)
});
```

### Verzoek Annuleren (AbortController)

`fromFetch()` ondersteunt het annuleren van verzoeken met behulp van de Fetch API's `AbortController`.

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const controller = new AbortController();
const signal = controller.signal;

const api$ = fromFetch('https://api.example.com/data', {
  signal // Geef AbortController signal door
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
);

const subscription = api$.subscribe({
  next: data => console.log('Data:', data),
  error: error => console.error('Error:', error)
});

// Annuleer verzoek na 3 seconden
setTimeout(() => {
  controller.abort();
  // Of subscription.unsubscribe();
}, 3000);
```

> [!TIP]
> **Automatische Annulering door RxJS**
>
> Roep gewoon `unsubscribe()` aan en RxJS zal het verzoek intern annuleren met `AbortController`. Er is geen behoefte om handmatig een `AbortController` in te stellen.

### Zoeken op Basis van Gebruikersinvoer (switchMap)

```typescript
import { fromEvent, map, debounceTime, distinctUntilChanged, switchMap, of } from 'rxjs';
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
  next: results => console.log('Zoekresultaten:', results),
  error: error => console.error('Zoekfout:', error)
});
```

### Meerdere Verzoeken Parallel Uitvoeren

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
    console.log('Gebruikers:', users);
    console.log('Posts:', posts);
  },
  error: error => console.error('Een verzoek is mislukt:', error)
});
```

## Veelvoorkomende Gebruikssituaties

### 1. Verzoek met Authenticatietoken

```typescript
import { switchMap, Observable } from 'rxjs';
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
        throw new Error('Authenticatie vereist. Log opnieuw in.');
      }
      if (!response.ok) {
        throw new Error(`HTTP Error: ${response.status}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// Gebruiksvoorbeeld
interface UserProfile {
  id: number;
  name: string;
  email: string;
}

const profile$ = fetchWithAuth<UserProfile>('https://api.example.com/profile');

profile$.subscribe({
  next: profile => console.log('Profiel:', profile),
  error: error => console.error('Error:', error)
});
```

### 2. Bestand Downloaden (Blob)

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const downloadFile$ = fromFetch('https://api.example.com/files/report.pdf').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    // Ophalen als Blob
    return response.blob();
  })
);

downloadFile$.subscribe({
  next: blob => {
    // Genereer downloadlink van Blob
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'report.pdf';
    a.click();
    window.URL.revokeObjectURL(url);
    console.log('Download compleet');
  },
  error: error => console.error('Downloadfout:', error)
});
```

### 3. GraphQL Query

```typescript
import { switchMap, map } from 'rxjs';
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
        throw new Error('Geen data geretourneerd');
      }
      return result.data;
    })
  );
}

// Gebruiksvoorbeeld
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
  next: ({ user }) => console.log('Gebruiker:', user),
  error: error => console.error('Error:', error)
});
```

### 4. API met Paginering

```typescript
import { expand, takeWhile, reduce, switchMap, Observable } from 'rxjs';
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

// Gebruiksvoorbeeld
interface Item {
  id: number;
  name: string;
}

const allItems$ = fetchAllPages<Item>('https://api.example.com/items');

allItems$.subscribe({
  next: items => console.log('Alle items:', items),
  error: error => console.error('Error:', error)
});
```

## fromFetch() Opties

`fromFetch()` kan de `RequestInit` optie van de Fetch API zonder wijziging gebruiken.

```typescript
interface RequestInit {
  method?: string;              // HTTP-methode (GET, POST, PUT, DELETE, etc.)
  headers?: HeadersInit;        // Request header
  body?: BodyInit | null;       // Request body
  mode?: RequestMode;           // cors, no-cors, same-origin
  credentials?: RequestCredentials; // omit, same-origin, include
  cache?: RequestCache;         // Cache-modus
  redirect?: RequestRedirect;   // Redirect-verwerking
  referrer?: string;            // Referrer
  integrity?: string;           // Subresource integrity
  signal?: AbortSignal;         // AbortController signal
}
```

## ajax() vs fromFetch() Vergelijking

| Functie | ajax() | fromFetch() |
|------|--------|-------------|
| Basistechnologie | XMLHttpRequest | Fetch API |
| Automatische JSON-parsing | ✅ `getJSON()` | ❌ Handmatig `.json()` |
| Automatische HTTP-foutdetectie | ✅ Automatische error bij 4xx/5xx | ❌ Handmatige `response.ok` controle |
| Voortgangsbewaking | ✅ | ❌ |
| Timeout | ✅ Ingebouwd | ❌ Geïmplementeerd met RxJS `timeout()` |
| Verzoek annuleren | ✅ unsubscribe() | ✅ unsubscribe() of AbortController |
| IE11-ondersteuning | ✅ | ❌ polyfill vereist |
| Bundelgrootte | Iets groter | Kleiner |
| Service Worker-ondersteuning | ❌ | ✅ |

> [!TIP]
> **Hoe te Kiezen**
>
> - **Alleen moderne browsers**: `fromFetch()` aanbevolen
> - **Behoefte aan legacy browser-ondersteuning**: Gebruik `ajax()`
> - **Voortgangsbewaking vereist**: Gebruik `ajax()`
> - **Lichtgewicht HTTP-communicatie**: `fromFetch()` is het beste
> - **Gebruik in Service Worker**: Alleen `fromFetch()` ondersteund

## Veelgemaakte Fouten en Oplossingen

### 1. HTTP-fout Niet Gevangen in `error` Callback

**Probleem:**
```typescript
// ❌ next wordt aangeroepen zelfs bij 404-fout
fromFetch('https://api.example.com/not-found').subscribe({
  next: response => console.log('Succes:', response), // ← Aangeroepen zelfs bij 404
  error: error => console.error('Error:', error)
});
```

**Oplossing:**
```typescript
// ✅ Controleer handmatig response.ok
fromFetch('https://api.example.com/not-found').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
).subscribe({
  next: data => console.log('Data:', data),
  error: error => console.error('Error:', error) // ← Dit wordt aangeroepen
});
```

### 2. CORS-fout

**Oplossingen:**
- Stel CORS-headers in aan serverzijde
- Specificeer expliciet `mode: 'cors'`
- Gebruik proxyserver tijdens ontwikkeling

```typescript
fromFetch('https://api.example.com/data', {
  mode: 'cors',
  credentials: 'include' // Als cookies worden meegenomen
});
```

### 3. Timeout Implementeren

Fetch API heeft geen timeout-functionaliteit, dus gebruik RxJS `timeout()`.

```typescript
import { timeout, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow').pipe(
  timeout(5000), // Timeout na 5 seconden
  switchMap(response => response.json())
);
```

## Best Practices

### 1. Maak Generieke fetchJSON Functie

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

### 2. Gebruik TypeScript Types

```typescript
// ✅ Goed voorbeeld: Specificeer expliciet type
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Slecht voorbeeld: Geen type gespecificeerd
const todo$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1')
  .pipe(switchMap(res => res.json()));
```

### 3. Implementeer Altijd Foutafhandeling

```typescript
// ✅ Goed voorbeeld: response.ok en catchError
const api$ = fromFetch('/api/data').pipe(
  switchMap(response => {
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return response.json();
  }),
  catchError(error => {
    console.error('Error:', error);
    return of(defaultValue);
  })
);
```

### 4. Vergeet Niet te Unsubscribe

```typescript
// ✅ Goed voorbeeld: Automatische release met takeUntil
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

## Samenvatting

`fromFetch()` is een lichtgewicht Creation Function voor HTTP-communicatie gebaseerd op de moderne Fetch API.

**Belangrijkste Kenmerken:**
- Gebaseerd op Fetch API en voldoet aan de nieuwste webstandaarden
- Lichtgewicht en kleine bundelgrootte
- Kan gebruikt worden binnen een Service Worker
- Handmatige foutcontrole en response-parsing vereist

**Gebruikssituaties:**
- Bij ondersteuning van alleen moderne browsers
- Wanneer bundelgrootte gereduceerd moet worden
- Wanneer HTTP-communicatie binnen een Service Worker wordt uitgevoerd
- Wanneer je Fetch API-functies (bijv. Request/Response objecten) direct wilt gebruiken

**Belangrijke Aandachtspunten:**
- De `error` callback wordt niet aangeroepen bij HTTP-fouten (controleer `response.ok` handmatig)
- JSON-parsing wordt handmatig gedaan (`response.json()`)
- Voortgangsbewaking wordt niet ondersteund
- polyfill is vereist voor legacy browsers zoals IE11

**Aanbevolen Gebruik:**
- Maak een generieke `fetchJSON()` functie voor hergebruik
- Zorg voor type-veiligheid door TypeScript types te gebruiken
- Implementeer altijd foutafhandeling
- Unsubscribe altijd wanneer niet langer nodig

## Gerelateerde Pagina's

- [ajax()](/nl/guide/creation-functions/http-communication/ajax) - XMLHttpRequest gebaseerde HTTP-communicatie
- [HTTP Communicatie Creation Functions](/nl/guide/creation-functions/http-communication/) - ajax() vs. fromFetch()
- [switchMap()](/nl/guide/operators/transformation/switchMap) - Nuttige operator voor het annuleren van HTTP-communicatie
- [Error Handling Strategieën](/nl/guide/error-handling/strategies) - Foutafhandelingspatronen voor HTTP-communicatie

## Referenties

- [RxJS Official Documentation - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API)
- [MDN Web Docs - AbortController](https://developer.mozilla.org/en-US/docs/Web/API/AbortController)
