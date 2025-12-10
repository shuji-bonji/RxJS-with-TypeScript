---
description: ajax() è una Creation Function di RxJS basata su XMLHttpRequest che gestisce la comunicazione HTTP come Observable. Supporta metodi HTTP come GET, POST, PUT, DELETE e offre funzionalità pratiche come monitoraggio dei progressi, gestione timeout, parsing JSON automatico e supporto per browser legacy. Con getJSON() è facile chiamare API JSON e realizzare comunicazioni HTTP type-safe.
---

# ajax()

[📘 RxJS Official Documentation - ajax](https://rxjs.dev/api/ajax/ajax)

`ajax()` è una Creation Function per gestire la comunicazione HTTP basata su XMLHttpRequest come Observable. Supporta metodi HTTP come GET, POST, PUT, DELETE e fornisce funzionalità pratiche come monitoraggio dei progressi e gestione timeout.

## Uso di base

### Semplice richiesta GET

L'esempio più semplice con `ajax()` è passare semplicemente l'URL come stringa.

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax('https://jsonplaceholder.typicode.com/todos/1');

api$.subscribe({
  next: response => console.log('Risposta:', response),
  error: error => console.error('Errore:', error),
  complete: () => console.log('Completato')
});

// Output:
// Risposta: {
//   status: 200,
//   response: { userId: 1, id: 1, title: "delectus aut autem", completed: false },
//   ...
// }
// Completato
```

### Recupero JSON con getJSON()

Per recuperare dati da API JSON, è conveniente usare `ajax.getJSON()`. Analizza automaticamente la risposta e restituisce solo la proprietà `response`.

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
  error: error => console.error('Errore:', error),
  complete: () => console.log('Completato')
});

// Output:
// Todo: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Completato
```

> [!TIP]
> **Type safety di TypeScript**
>
> Specificando un tipo generico a `ajax.getJSON<T>()`, è possibile garantire la type safety della risposta.

## Uso per metodo HTTP

### Richiesta GET

```typescript
import { ajax } from 'rxjs/ajax';

// Metodo 1: Semplice specifica stringa
const get1$ = ajax('https://api.example.com/users');

// Metodo 2: Parsing automatico con getJSON()
const get2$ = ajax.getJSON('https://api.example.com/users');

// Metodo 3: Configurazione dettagliata
const get3$ = ajax({
  url: 'https://api.example.com/users',
  method: 'GET',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer token123'
  }
});
```

### Richiesta POST

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

// Metodo 1: Utilizzando ajax.post()
const post1$ = ajax.post<CreateUserResponse>(
  'https://api.example.com/users',
  newUser,
  { 'Content-Type': 'application/json' }
);

// Metodo 2: Configurazione dettagliata
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
  next: response => console.log('Creazione riuscita:', response.response),
  error: error => console.error('Creazione fallita:', error)
});
```

### Richiesta PUT

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
  next: response => console.log('Aggiornamento riuscito:', response.response),
  error: error => console.error('Aggiornamento fallito:', error)
});
```

### Richiesta PATCH

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
  next: response => console.log('Aggiornamento parziale riuscito:', response.response),
  error: error => console.error('Aggiornamento parziale fallito:', error)
});
```

### Richiesta DELETE

```typescript
import { ajax } from 'rxjs/ajax';

const delete$ = ajax.delete('https://api.example.com/users/1');

delete$.subscribe({
  next: response => console.log('Eliminazione riuscita:', response),
  error: error => console.error('Eliminazione fallita:', error)
});
```

## Pattern pratici

### Gestione errori e retry

Nelle comunicazioni HTTP, è necessario gestire errori di rete ed errori del server.

```typescript
import { of, retry, catchError, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout di 5 secondi
  retry(2), // Riprova 2 volte in caso di fallimento
  catchError(error => {
    console.error('Errore recupero utente:', error);
    // Restituisce valore predefinito
    return of({
      id: 0,
      name: 'Unknown',
      email: 'unknown@example.com'
    } as User);
  })
);

fetchUser$.subscribe({
  next: user => console.log('Utente:', user),
  error: error => console.error('Errore critico:', error)
});
```

### Branching condizionale in base al codice di stato HTTP

```typescript
import { throwError, catchError } from 'rxjs';
import { ajax, AjaxError } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/data').pipe(
  catchError((error: AjaxError) => {
    if (error.status === 404) {
      console.error('Risorsa non trovata');
    } else if (error.status === 401) {
      console.error('Autenticazione richiesta');
    } else if (error.status === 500) {
      console.error('Si è verificato un errore del server');
    } else {
      console.error('Errore imprevisto:', error);
    }
    return throwError(() => error);
  })
);
```

### Esecuzione parallela di più richieste

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

// Attende il completamento di tutte le richieste
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
  error: error => console.error('Una delle richieste è fallita:', error)
});
```

### Ricerca in base all'input utente (switchMap)

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
  debounceTime(300), // Attende 300ms
  distinctUntilChanged(), // Ignora lo stesso valore
  switchMap(query => {
    if (query.length === 0) {
      return of([]);
    }
    // Quando viene inserita una nuova ricerca, annulla la richiesta precedente
    return ajax.getJSON<SearchResult[]>(`https://api.example.com/search?q=${query}`);
  })
);

search$.subscribe({
  next: results => console.log('Risultati ricerca:', results),
  error: error => console.error('Errore ricerca:', error)
});
```

> [!IMPORTANT]
> **Importanza di switchMap()**
>
> Utilizzando `switchMap()`, quando viene inserita una nuova query di ricerca, la richiesta HTTP precedente viene automaticamente annullata. Ciò impedisce che i vecchi risultati di ricerca sovrascrivano i nuovi risultati.

### Monitoraggio progresso (Upload file)

`ajax()` può monitorare i progressi di upload/download utilizzando l'evento `progress` di `XMLHttpRequest`.

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
    // Abilita eventi di progresso
    progressSubscriber: {
      next: (progress) => {
        const percentage = (progress.loaded / progress.total) * 100;
        console.log(`Progresso upload: ${percentage.toFixed(2)}%`);
      }
    }
  });

  upload$.subscribe({
    next: response => console.log('Upload completato:', response),
    error: error => console.error('Upload fallito:', error)
  });
}
```

### Header personalizzati e richieste cross-domain

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected-resource',
  method: 'GET',
  headers: {
    'Authorization': 'Bearer your-token-here',
    'X-Custom-Header': 'CustomValue'
  },
  crossDomain: true, // Richiesta CORS
  withCredentials: true // Include Cookie
});

api$.subscribe({
  next: response => console.log('Risposta:', response),
  error: error => console.error('Errore:', error)
});
```

## Esempi di uso comune

### 1. Chiamata API con paginazione

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
  next: allItems => console.log('Tutti gli elementi:', allItems),
  error: error => console.error('Errore:', error)
});
```

### 2. Polling (Recupero dati periodico)

```typescript
import { interval, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Status {
  status: string;
  lastUpdate: string;
}

// Chiama l'API ogni 5 secondi
const polling$ = interval(5000).pipe(
  switchMap(() => ajax.getJSON<Status>('https://api.example.com/status'))
);

const subscription = polling$.subscribe({
  next: status => console.log('Stato:', status),
  error: error => console.error('Errore:', error)
});

// Interrompe dopo 30 secondi
setTimeout(() => subscription.unsubscribe(), 30000);
```

### 3. Richieste con dipendenze

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

// Prima recupera le informazioni utente, poi i dettagli
const userWithDetails$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  switchMap(user =>
    ajax.getJSON<UserDetails>(`https://api.example.com/users/${user.id}/details`).pipe(
      map(details => ({ ...user, ...details }))
    )
  )
);

userWithDetails$.subscribe({
  next: userWithDetails => console.log('Dettagli utente:', userWithDetails),
  error: error => console.error('Errore:', error)
});
```

## Opzioni di ajax()

`ajax()` ha opzioni per configurazioni dettagliate.

```typescript
interface AjaxConfig {
  url: string;                    // URL della richiesta
  method?: string;                // Metodo HTTP (GET, POST, PUT, DELETE ecc.)
  headers?: object;               // Header della richiesta
  body?: any;                     // Body della richiesta
  timeout?: number;               // Tempo di timeout (millisecondi)
  responseType?: string;          // Tipo di risposta (json, text, blob ecc.)
  crossDomain?: boolean;          // Se è una richiesta CORS
  withCredentials?: boolean;      // Se includere Cookie
  progressSubscriber?: Subscriber; // Subscriber per monitoraggio progressi
}
```

## Errori comuni e soluzioni

### 1. Errore CORS

**Esempio di errore:**
```
Access to XMLHttpRequest at 'https://api.example.com' from origin 'http://localhost:3000'
has been blocked by CORS policy
```

**Soluzione:**
- Configurare header CORS lato server
- Utilizzare un proxy server
- In fase di sviluppo provare `crossDomain: true` e `withCredentials: false`

### 2. Timeout di rete

**Soluzione:**
```typescript
import { timeout, retry } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/slow-endpoint').pipe(
  timeout(10000), // Timeout di 10 secondi
  retry(2) // Riprova 2 volte
);
```

### 3. Errore di autenticazione (401 Unauthorized)

**Soluzione:**
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
      // Aggiorna il token e riprova
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

## Confronto tra ajax() e fromFetch()

| Funzionalità | ajax() | fromFetch() |
|------|--------|-------------|
| Parsing JSON automatico | ✅ `getJSON()` | ❌ Manuale con `.json()` |
| Monitoraggio progressi | ✅ | ❌ |
| Rilevamento automatico errori HTTP | ✅ | ❌ |
| Dimensione bundle | Leggermente grande | Piccola |
| Supporto IE11 | ✅ | ❌ |

> [!TIP]
> **Punti per la scelta**
>
> - **Necessario monitoraggio progressi**: usa `ajax()`
> - **Supporto browser legacy**: usa `ajax()`
> - **Comunicazione HTTP leggera**: considera `fromFetch()`
> - **Recupero JSON semplice**: `ajax.getJSON()` è il più facile

## Best practice

### 1. Garantire type safety

```typescript
// ✅ Buon esempio: Specifica tipo generico
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todos$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Cattivo esempio: Nessuna specifica tipo
const todos$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
```

### 2. Implementare sempre gestione errori

```typescript
// ✅ Buon esempio: Gestione errori con catchError
const api$ = ajax.getJSON('/api/data').pipe(
  catchError(error => {
    console.error('Errore:', error);
    return of(defaultValue);
  })
);

// ❌ Cattivo esempio: Nessuna gestione errori
const api$ = ajax.getJSON('/api/data');
```

### 3. Non dimenticare di annullare la sottoscrizione

```typescript
// ✅ Buon esempio: Annulla alla distruzione del componente
class MyComponent {
  private subscription: Subscription;

  ngOnInit() {
    this.subscription = ajax.getJSON('/api/data').subscribe(...);
  }

  ngOnDestroy() {
    this.subscription.unsubscribe();
  }
}

// Oppure usa takeUntil
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

## Riepilogo

`ajax()` è una potente Creation Function per eseguire comunicazioni HTTP in RxJS.

**Caratteristiche principali:**
- Basato su XMLHttpRequest, supporta un'ampia gamma di browser
- Facile recupero JSON con `getJSON()`
- Funzionalità pratiche come monitoraggio progressi, timeout, retry
- Rilevamento automatico errori HTTP

**Situazioni di utilizzo:**
- Quando è necessario supportare browser legacy (IE11 ecc.)
- Quando si desidera visualizzare i progressi di upload/download file
- Chiamate semplici e chiare ad API JSON

**Punti di attenzione:**
- Implementare sempre la gestione degli errori
- Annullare sempre la sottoscrizione quando non più necessaria
- Utilizzare i tipi TypeScript per garantire type safety

## Pagine correlate

- [fromFetch()](/it/guide/creation-functions/http-communication/fromFetch) - Comunicazione HTTP basata su Fetch API
- [Creation Functions per comunicazione HTTP](/it/guide/creation-functions/http-communication/) - Confronto tra ajax() e fromFetch()
- [switchMap()](/it/guide/operators/transformation/switchMap) - Operatore utile per annullare comunicazioni HTTP
- [Strategie di gestione errori](/it/guide/error-handling/strategies) - Pattern di gestione errori per comunicazioni HTTP

## Risorse di riferimento

- [RxJS Official Documentation - ajax](https://rxjs.dev/api/ajax/ajax)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/it/docs/Web/API/XMLHttpRequest)
- [Learn RxJS - ajax](https://www.learnrxjs.io/learn-rxjs/operators/creation/ajax)
