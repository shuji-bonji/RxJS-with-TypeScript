---
description: fromFetch() è una Creation Function di RxJS basata su Fetch API che gestisce la comunicazione HTTP come Observable. Supporta metodi HTTP come GET, POST, PUT, DELETE ed è leggera, conforme agli standard web moderni e compatibile con Service Worker per realizzare comunicazioni HTTP all'avanguardia. Richiede controllo errori e parsing JSON manuali, ma ha dimensioni del bundle ridotte e permette comunicazioni HTTP type-safe.
---

# fromFetch()

[📘 RxJS Official Documentation - fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` è una Creation Function per gestire la comunicazione HTTP basata sulla moderna Fetch API come Observable. Rispetto a `ajax()`, è leggera e conforme agli standard web più recenti.

## Uso di base

### Semplice richiesta GET

L'esempio più semplice con `fromFetch()` è passare un URL e analizzare manualmente la risposta.

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
      // Se la risposta ha successo, analizza JSON
      return response.json();
    } else {
      // In caso di errore HTTP, genera un errore
      return throwError(() => new Error(`HTTP Error: ${response.status}`));
    }
  }),
  catchError(error => {
    console.error('Errore:', error);
    return of({ error: true, message: error.message });
  })
);

data$.subscribe({
  next: data => console.log('Dati:', data),
  error: error => console.error('Errore sottoscrizione:', error),
  complete: () => console.log('Completato')
});

// Output:
// Dati: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Completato
```

> [!IMPORTANT]
> **Differenza importante da ajax()**
>
> - `fromFetch()` non chiama il callback `error` nemmeno per errori HTTP (4xx, 5xx)
> - È necessario controllare manualmente la proprietà `ok` della risposta
> - Anche il parsing con `.json()` ecc. deve essere eseguito manualmente

## Uso per metodo HTTP

### Richiesta GET

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
  next: users => console.log('Elenco utenti:', users),
  error: error => console.error('Errore:', error)
});
```

### Richiesta POST

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
  next: user => console.log('Creazione riuscita:', user),
  error: error => console.error('Creazione fallita:', error)
});
```

### Richiesta PUT

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
  next: user => console.log('Aggiornamento riuscito:', user),
  error: error => console.error('Aggiornamento fallito:', error)
});
```

### Richiesta DELETE

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
    // DELETE normalmente restituisce una risposta vuota o solo lo stato
    return response.status === 204 ? of(null) : response.json();
  })
);

deleteUser$.subscribe({
  next: result => console.log('Eliminazione riuscita:', result),
  error: error => console.error('Eliminazione fallita:', error)
});
```

## Pattern pratici

### Funzione generica di gestione errori HTTP

Poiché `fromFetch()` richiede controlli manuali degli errori, è conveniente creare una funzione generica.

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

// Esempio di utilizzo
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

todo$.subscribe({
  next: todo => console.log('Todo:', todo),
  error: error => console.error('Errore:', error)
});
```

### Elaborazione dettagliata in base al codice di stato HTTP

```typescript
import { throwError, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/data').pipe(
  switchMap(response => {
    switch (response.status) {
      case 200:
        return response.json();
      case 204:
        // No Content - risposta vuota
        return of(null);
      case 401:
        throw new Error('Autenticazione richiesta');
      case 403:
        throw new Error('Accesso negato');
      case 404:
        throw new Error('Risorsa non trovata');
      case 500:
        throw new Error('Si è verificato un errore del server');
      default:
        throw new Error(`Stato HTTP imprevisto: ${response.status}`);
    }
  })
);

api$.subscribe({
  next: data => console.log('Dati:', data),
  error: error => console.error('Errore:', error)
});
```

### Timeout e retry

```typescript
import { switchMap, timeout, retry } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow-endpoint').pipe(
  timeout(5000), // Timeout di 5 secondi
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  }),
  retry(2) // Riprova 2 volte in caso di fallimento
);

api$.subscribe({
  next: data => console.log('Dati:', data),
  error: error => console.error('Errore:', error)
});
```

### Cancellazione richiesta (AbortController)

`fromFetch()` supporta la cancellazione delle richieste utilizzando `AbortController` della Fetch API.

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const controller = new AbortController();
const signal = controller.signal;

const api$ = fromFetch('https://api.example.com/data', {
  signal // Passa il signal di AbortController
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
);

const subscription = api$.subscribe({
  next: data => console.log('Dati:', data),
  error: error => console.error('Errore:', error)
});

// Annulla la richiesta dopo 3 secondi
setTimeout(() => {
  controller.abort();
  // oppure subscription.unsubscribe();
}, 3000);
```

> [!TIP]
> **Cancellazione automatica di RxJS**
>
> Chiamando semplicemente `unsubscribe()`, RxJS utilizza internamente `AbortController` per annullare la richiesta. Non è necessario configurare manualmente `AbortController`.

### Ricerca in base all'input utente (switchMap)

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
  next: results => console.log('Risultati ricerca:', results),
  error: error => console.error('Errore ricerca:', error)
});
```

### Esecuzione parallela di più richieste

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
  error: error => console.error('Una delle richieste è fallita:', error)
});
```

## Esempi di uso comune

### 1. Richiesta con token di autenticazione

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
        throw new Error('Autenticazione richiesta. Effettuare nuovamente il login.');
      }
      if (!response.ok) {
        throw new Error(`HTTP Error: ${response.status}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// Esempio di utilizzo
interface UserProfile {
  id: number;
  name: string;
  email: string;
}

const profile$ = fetchWithAuth<UserProfile>('https://api.example.com/profile');

profile$.subscribe({
  next: profile => console.log('Profilo:', profile),
  error: error => console.error('Errore:', error)
});
```

### 2. Download file (Blob)

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const downloadFile$ = fromFetch('https://api.example.com/files/report.pdf').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    // Recupera come Blob
    return response.blob();
  })
);

downloadFile$.subscribe({
  next: blob => {
    // Genera link di download da Blob
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'report.pdf';
    a.click();
    window.URL.revokeObjectURL(url);
    console.log('Download completato');
  },
  error: error => console.error('Errore download:', error)
});
```

### 3. Query GraphQL

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
        throw new Error('Nessun dato restituito');
      }
      return result.data;
    })
  );
}

// Esempio di utilizzo
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
  next: ({ user }) => console.log('Utente:', user),
  error: error => console.error('Errore:', error)
});
```

### 4. API con paginazione

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

// Esempio di utilizzo
interface Item {
  id: number;
  name: string;
}

const allItems$ = fetchAllPages<Item>('https://api.example.com/items');

allItems$.subscribe({
  next: items => console.log('Tutti gli elementi:', items),
  error: error => console.error('Errore:', error)
});
```

## Opzioni di fromFetch()

`fromFetch()` può utilizzare direttamente le opzioni `RequestInit` della Fetch API.

```typescript
interface RequestInit {
  method?: string;              // Metodo HTTP (GET, POST, PUT, DELETE ecc.)
  headers?: HeadersInit;        // Header della richiesta
  body?: BodyInit | null;       // Body della richiesta
  mode?: RequestMode;           // cors, no-cors, same-origin
  credentials?: RequestCredentials; // omit, same-origin, include
  cache?: RequestCache;         // Modalità cache
  redirect?: RequestRedirect;   // Gestione redirect
  referrer?: string;            // Referrer
  integrity?: string;           // Integrità sottorisorse
  signal?: AbortSignal;         // Signal di AbortController
}
```

## Confronto tra ajax() e fromFetch()

| Funzionalità | ajax() | fromFetch() |
|------|--------|-------------|
| Tecnologia di base | XMLHttpRequest | Fetch API |
| Parsing JSON automatico | ✅ `getJSON()` | ❌ Manuale con `.json()` |
| Rilevamento automatico errori HTTP | ✅ Errore automatico per 4xx/5xx | ❌ Controllo manuale di `response.ok` |
| Monitoraggio progressi | ✅ | ❌ |
| Timeout | ✅ Integrato | ❌ Implementazione con `timeout()` di RxJS |
| Cancellazione richiesta | ✅ unsubscribe() | ✅ unsubscribe() o AbortController |
| Supporto IE11 | ✅ | ❌ Necessario polyfill |
| Dimensione bundle | Leggermente grande | Piccola |
| Supporto Service Worker | ❌ | ✅ |

> [!TIP]
> **Punti per la scelta**
>
> - **Supporto solo browser moderni**: consigliato `fromFetch()`
> - **Necessario supporto browser legacy**: usa `ajax()`
> - **Necessario monitoraggio progressi**: usa `ajax()`
> - **Comunicazione HTTP leggera**: `fromFetch()` è ottimale
> - **Uso all'interno di Service Worker**: solo `fromFetch()` supportato

## Errori comuni e soluzioni

### 1. Gli errori HTTP non vengono catturati dal callback `error`

**Problema:**
```typescript
// ❌ Anche con errore 404 viene chiamato next
fromFetch('https://api.example.com/not-found').subscribe({
  next: response => console.log('Successo:', response), // ← Viene chiamato anche con 404
  error: error => console.error('Errore:', error)
});
```

**Soluzione:**
```typescript
// ✅ Controlla manualmente response.ok
fromFetch('https://api.example.com/not-found').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
).subscribe({
  next: data => console.log('Dati:', data),
  error: error => console.error('Errore:', error) // ← Viene chiamato qui
});
```

### 2. Errore CORS

**Soluzione:**
- Configurare header CORS lato server
- Specificare esplicitamente `mode: 'cors'`
- In fase di sviluppo utilizzare un proxy server

```typescript
fromFetch('https://api.example.com/data', {
  mode: 'cors',
  credentials: 'include' // Quando si includono Cookie
});
```

### 3. Implementazione timeout

Poiché Fetch API non ha funzionalità di timeout, utilizzare `timeout()` di RxJS.

```typescript
import { timeout, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow').pipe(
  timeout(5000), // Timeout di 5 secondi
  switchMap(response => response.json())
);
```

## Best practice

### 1. Creare funzione fetchJSON generica

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

### 2. Utilizzare i tipi TypeScript

```typescript
// ✅ Buon esempio: Specifica esplicitamente il tipo
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Cattivo esempio: Nessuna specifica tipo
const todo$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1')
  .pipe(switchMap(res => res.json()));
```

### 3. Implementare sempre gestione errori

```typescript
// ✅ Buon esempio: response.ok e catchError
const api$ = fromFetch('/api/data').pipe(
  switchMap(response => {
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return response.json();
  }),
  catchError(error => {
    console.error('Errore:', error);
    return of(defaultValue);
  })
);
```

### 4. Non dimenticare di annullare la sottoscrizione

```typescript
// ✅ Buon esempio: Annullamento automatico con takeUntil
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

## Riepilogo

`fromFetch()` è una Creation Function leggera per comunicazioni HTTP basata sulla moderna Fetch API.

**Caratteristiche principali:**
- Basata su Fetch API, conforme agli standard web più recenti
- Leggera con dimensione del bundle ridotta
- Utilizzabile anche all'interno di Service Worker
- Richiede controllo errori e parsing risposta manuali

**Situazioni di utilizzo:**
- Quando si supportano solo browser moderni
- Quando si desidera ridurre la dimensione del bundle
- Quando si esegue comunicazione HTTP all'interno di Service Worker
- Quando si desidera utilizzare direttamente le funzionalità di Fetch API (oggetti Request/Response ecc.)

**Punti di attenzione:**
- Il callback `error` non viene chiamato nemmeno con errori HTTP (controllo manuale di `response.ok`)
- Il parsing JSON è manuale (con `response.json()`)
- Monitoraggio progressi non supportato
- Necessario polyfill per browser legacy come IE11

**Uso consigliato:**
- Creare e riutilizzare funzione generica `fetchJSON()`
- Utilizzare i tipi TypeScript per garantire type safety
- Implementare sempre la gestione degli errori
- Annullare sempre la sottoscrizione quando non più necessaria

## Pagine correlate

- [ajax()](/it/guide/creation-functions/http-communication/ajax) - Comunicazione HTTP basata su XMLHttpRequest
- [Creation Functions per comunicazione HTTP](/it/guide/creation-functions/http-communication/) - Confronto tra ajax() e fromFetch()
- [switchMap()](/it/guide/operators/transformation/switchMap) - Operatore utile per annullare comunicazioni HTTP
- [Strategie di gestione errori](/it/guide/error-handling/strategies) - Pattern di gestione errori per comunicazioni HTTP

## Risorse di riferimento

- [RxJS Official Documentation - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/it/docs/Web/API/Fetch_API)
- [MDN Web Docs - AbortController](https://developer.mozilla.org/it/docs/Web/API/AbortController)
