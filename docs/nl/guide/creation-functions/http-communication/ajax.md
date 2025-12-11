---
description: ajax() is een RxJS Creation Function die XMLHttpRequest-gebaseerde HTTP-communicatie als Observable behandelt, met ondersteuning voor HTTP-methoden zoals GET, POST, PUT en DELETE, en praktische functies zoals voortgangsmonitoring, timeoutverwerking, automatische JSON-parsing en ondersteuning voor legacy browsers.
---

# ajax()

[📘 RxJS Officiële Documentatie - ajax](https://rxjs.dev/api/ajax/ajax)

`ajax()` is een Creation Function voor het behandelen van HTTP-communicatie gebaseerd op XMLHttpRequest als Observable, met ondersteuning voor HTTP-methoden zoals GET, POST, PUT en DELETE, en praktische functies zoals voortgangsmonitoring en timeoutafhandeling.

## Basisgebruik

### Eenvoudig GET-verzoek

Het eenvoudigste voorbeeld van het gebruik van `ajax()` is simpelweg een URL als string doorgeven.

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax('https://jsonplaceholder.typicode.com/todos/1');

api$.subscribe({
  next: response => console.log('Antwoord:', response),
  error: error => console.error('Fout:', error),
  complete: () => console.log('Voltooid')
});

// Output:
// Antwoord: {
//   status: 200,
//   response: { userId: 1, id: 1, title: "delectus aut autem", completed: false },
//   ...
// }
// Voltooid
```

### JSON ophalen met getJSON()

Als u gegevens van de JSON API wilt ophalen, kunt u `ajax.getJSON()` gebruiken. Het parseert automatisch het antwoord en retourneert alleen de `response`-eigenschap.

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
  error: error => console.error('Fout:', error),
  complete: () => console.log('Voltooid')
});

// Output:
// Todo: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Voltooid
```

> [!TIP]
> **TypeScript Type-veiligheid**
>
> Type-veiligheid van het antwoord kan worden gegarandeerd door een generiek type op te geven voor `ajax.getJSON<T>()`.

## Gebruik per HTTP-methode

### GET-verzoek

```typescript
import { ajax } from 'rxjs/ajax';

// Methode 1: Eenvoudige string-specificatie
const get1$ = ajax('https://api.example.com/users');

// Methode 2: Automatische parsing met getJSON()
const get2$ = ajax.getJSON('https://api.example.com/users');

// Methode 3: Gedetailleerde configuratie
const get3$ = ajax({
  url: 'https://api.example.com/users',
  method: 'GET',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer token123'
  }
});
```

### POST-verzoek

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
  name: 'Jan de Vries',
  email: 'jan@example.com'
};

// Methode 1: ajax.post() gebruiken
const post1$ = ajax.post<CreateUserResponse>(
  'https://api.example.com/users',
  newUser,
  { 'Content-Type': 'application/json' }
);

// Methode 2: Gedetailleerde configuratie
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
  next: response => console.log('Aanmaken geslaagd:', response.response),
  error: error => console.error('Aanmaken mislukt:', error)
});
```

### PUT-verzoek

```typescript
import { ajax } from 'rxjs/ajax';

interface UpdateUserRequest {
  name: string;
  email: string;
}

const updatedUser: UpdateUserRequest = {
  name: 'Piet Jansen',
  email: 'piet@example.com'
};

const put$ = ajax.put(
  'https://api.example.com/users/1',
  updatedUser,
  { 'Content-Type': 'application/json' }
);

put$.subscribe({
  next: response => console.log('Update geslaagd:', response.response),
  error: error => console.error('Update mislukt:', error)
});
```

### PATCH-verzoek

```typescript
import { ajax } from 'rxjs/ajax';

interface PatchUserRequest {
  email?: string;
}

const patch$ = ajax.patch(
  'https://api.example.com/users/1',
  { email: 'nieuw-email@example.com' } as PatchUserRequest,
  { 'Content-Type': 'application/json' }
);

patch$.subscribe({
  next: response => console.log('Gedeeltelijke update geslaagd:', response.response),
  error: error => console.error('Gedeeltelijke update mislukt:', error)
});
```

### DELETE-verzoek

```typescript
import { ajax } from 'rxjs/ajax';

const delete$ = ajax.delete('https://api.example.com/users/1');

delete$.subscribe({
  next: response => console.log('Verwijderen geslaagd:', response),
  error: error => console.error('Verwijderen mislukt:', error)
});
```

## Praktische patronen

### Foutafhandeling en opnieuw proberen

HTTP-communicatie vereist afhandeling van netwerk- en serverfouten.

```typescript
import { of, retry, catchError, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout in 5 seconden
  retry(2), // Twee keer opnieuw proberen bij falen
  catchError(error => {
    console.error('Fout bij ophalen gebruiker:', error);
    // Standaardwaarde retourneren
    return of({
      id: 0,
      name: 'Onbekend',
      email: 'onbekend@example.com'
    } as User);
  })
);

fetchUser$.subscribe({
  next: user => console.log('Gebruiker:', user),
  error: error => console.error('Fatale fout:', error)
});
```

### Voorwaardelijke vertakking op HTTP-statuscode

```typescript
import { throwError, catchError } from 'rxjs';
import { ajax, AjaxError } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/data').pipe(
  catchError((error: AjaxError) => {
    if (error.status === 404) {
      console.error('Bron niet gevonden');
    } else if (error.status === 401) {
      console.error('Authenticatie vereist');
    } else if (error.status === 500) {
      console.error('Serverfout opgetreden');
    } else {
      console.error('Onverwachte fout:', error);
    }
    return throwError(() => error);
  })
);
```

### Meerdere verzoeken parallel uitvoeren

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

// Wacht tot alle verzoeken zijn voltooid
forkJoin({
  users: users$,
  posts: posts$,
  comments: comments$
}).subscribe({
  next: ({ users, posts, comments }) => {
    console.log('Gebruikers:', users);
    console.log('Berichten:', posts);
    console.log('Reacties:', comments);
  },
  error: error => console.error('Een verzoek is mislukt:', error)
});
```

### Zoeken op basis van gebruikersinvoer (switchMap)

```typescript
import { fromEvent, map, debounceTime, distinctUntilChanged, switchMap, of } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface SearchResult {
  id: number;
  title: string;
}

const searchInput = document.querySelector('#search') as HTMLInputElement;

const search$ = fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // 300ms wachten
  distinctUntilChanged(), // Dezelfde waarde negeren
  switchMap(query => {
    if (query.length === 0) {
      return of([]);
    }
    // Annuleer vorig verzoek als nieuwe zoekopdracht wordt ingevoerd
    return ajax.getJSON<SearchResult[]>(`https://api.example.com/search?q=${query}`);
  })
);

search$.subscribe({
  next: results => console.log('Zoekresultaten:', results),
  error: error => console.error('Zoekfout:', error)
});
```

> [!IMPORTANT]
> **Belang van switchMap()**
>
> Door `switchMap()` te gebruiken wordt een eerder HTTP-verzoek automatisch geannuleerd wanneer een nieuwe zoekopdracht wordt ingevoerd. Dit voorkomt dat oude zoekresultaten nieuwe resultaten overschrijven.

### Voortgangsmonitoring (Bestandsupload)

`ajax()` kan upload- en downloadvoortgang monitoren met behulp van de `progress`-gebeurtenis van `XMLHttpRequest`.

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
    // Voortgangsgebeurtenissen inschakelen
    progressSubscriber: {
      next: (progress) => {
        const percentage = (progress.loaded / progress.total) * 100;
        console.log(`Uploadvoortgang: ${percentage.toFixed(2)}%`);
      }
    }
  });

  upload$.subscribe({
    next: response => console.log('Upload voltooid:', response),
    error: error => console.error('Upload mislukt:', error)
  });
}
```

### Aangepaste headers en cross-domain verzoeken

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected-resource',
  method: 'GET',
  headers: {
    'Authorization': 'Bearer your-token-here',
    'X-Custom-Header': 'CustomValue'
  },
  crossDomain: true, // CORS-verzoek
  withCredentials: true // Cookies toevoegen
});

api$.subscribe({
  next: response => console.log('Antwoord:', response),
  error: error => console.error('Fout:', error)
});
```

## Veelvoorkomende use cases

### 1. API-aanroep met paginering

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
  next: allItems => console.log('Alle items:', allItems),
  error: error => console.error('Fout:', error)
});
```

### 2. Polling (Periodiek gegevens ophalen)

```typescript
import { interval, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Status {
  status: string;
  lastUpdate: string;
}

// Elke 5 seconden API aanroepen
const polling$ = interval(5000).pipe(
  switchMap(() => ajax.getJSON<Status>('https://api.example.com/status'))
);

const subscription = polling$.subscribe({
  next: status => console.log('Status:', status),
  error: error => console.error('Fout:', error)
});

// Stop na 30 seconden
setTimeout(() => subscription.unsubscribe(), 30000);
```

### 3. Afhankelijke verzoeken

```typescript
import { switchMap, map } from 'rxjs';
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

// Eerst gebruikersinfo ophalen, daarna gedetailleerde informatie ophalen
const userWithDetails$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  switchMap(user =>
    ajax.getJSON<UserDetails>(`https://api.example.com/users/${user.id}/details`).pipe(
      map(details => ({ ...user, ...details }))
    )
  )
);

userWithDetails$.subscribe({
  next: userWithDetails => console.log('Gebruikersdetails:', userWithDetails),
  error: error => console.error('Fout:', error)
});
```

## ajax() opties

`ajax()` biedt opties voor geavanceerde configuratie.

```typescript
interface AjaxConfig {
  url: string;                    // Verzoek-URL
  method?: string;                // HTTP-methode (GET, POST, PUT, DELETE, enz.)
  headers?: object;               // Verzoekheader
  body?: any;                     // Verzoekbody
  timeout?: number;               // Timeouttijd (in milliseconden)
  responseType?: string;          // Responstype (json, text, blob, enz.)
  crossDomain?: boolean;          // CORS-verzoek of niet
  withCredentials?: boolean;      // Of cookies moeten worden toegevoegd
  progressSubscriber?: Subscriber; // Subscriber voor voortgangsmonitoring
}
```

## Veelvoorkomende fouten en oplossingen

### 1. CORS-fout

**Foutvoorbeeld:**
```
Access to XMLHttpRequest at 'https://api.example.com' from origin 'http://localhost:3000'
has been blocked by CORS policy
```

**Oplossingen:**
- Stel de CORS-header in aan de serverzijde
- Gebruik een proxyserver
- Probeer `crossDomain: true` en `withCredentials: false` tijdens ontwikkeling

### 2. Netwerk-timeout

**Oplossing:**
```typescript
import { timeout, retry } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/slow-endpoint').pipe(
  timeout(10000), // Timeout in 10 seconden
  retry(2) // Twee keer opnieuw proberen
);
```

### 3. Authenticatiefout (401 Unauthorized)

**Oplossing:**
```typescript
import { throwError, catchError, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected',
  headers: {
    'Authorization': `Bearer ${getAccessToken()}`
  }
}).pipe(
  catchError(error => {
    if (error.status === 401) {
      // Token vernieuwen en opnieuw proberen
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

## ajax() vs fromFetch() vergelijking

| Functie | ajax() | fromFetch() |
|---------|--------|-------------|
| Automatische JSON-parsing | ✅ `getJSON()` | ❌ Handmatig `.json()` |
| Voortgangsmonitoring | ✅ | ❌ |
| Automatische HTTP-foutdetectie | ✅ | ❌ |
| Bundelgrootte | Iets groter | Kleiner |
| IE11-ondersteuning | ✅ | ❌ |

> [!TIP]
> **Hoe te kiezen**
>
> - **Voortgangsmonitoring nodig**: Gebruik `ajax()`
> - **Ondersteuning voor legacy browsers**: Gebruik `ajax()`
> - **Lichtgewicht HTTP-communicatie**: Overweeg `fromFetch()`
> - **Eenvoudige JSON-ophaling**: `ajax.getJSON()` is het gemakkelijkst

## Best practices

### 1. Zorg voor type-veiligheid

```typescript
// ✅ Goed voorbeeld: Specificeer een generiek type
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todos$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Slecht voorbeeld: Geen type gespecificeerd
const todos$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
```

### 2. Implementeer altijd foutafhandeling

```typescript
// ✅ Goed voorbeeld: Foutafhandeling met catchError
const api$ = ajax.getJSON('/api/data').pipe(
  catchError(error => {
    console.error('Fout:', error);
    return of(defaultValue);
  })
);

// ❌ Slecht voorbeeld: Geen foutafhandeling
const api$ = ajax.getJSON('/api/data');
```

### 3. Vergeet niet te unsubscribe-en

```typescript
// ✅ Goed voorbeeld: Unsubscribe bij vernietiging van component
class MyComponent {
  private subscription: Subscription;

  ngOnInit() {
    this.subscription = ajax.getJSON('/api/data').subscribe(...);
  }

  ngOnDestroy() {
    this.subscription.unsubscribe();
  }
}

// Of gebruik takeUntil
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

## Samenvatting

`ajax()` is een krachtige Creation Function voor HTTP-communicatie in RxJS.

**Belangrijkste kenmerken:**
- Gebaseerd op XMLHttpRequest, ondersteunt een breed scala aan browsers
- Eenvoudige JSON-ophaling met `getJSON()`
- Praktische functies zoals voortgangsmonitoring, timeout, retry, enz.
- Automatische HTTP-foutdetectie

**Gebruiksscenario's:**
- Ondersteuning voor legacy browsers (bijv. IE11) vereist
- Moet bestandsupload/-downloadvoortgang weergeven
- Eenvoudige en duidelijke JSON API-aanroepen

**Belangrijke opmerkingen:**
- Implementeer altijd foutafhandeling
- Altijd unsubscribe-en wanneer niet langer nodig
- Gebruik TypeScript-types om type-veiligheid te garanderen

## Gerelateerde pagina's

- [fromFetch()](/nl/guide/creation-functions/http-communication/fromFetch) - Fetch API-gebaseerde HTTP-communicatie
- [HTTP-communicatie Creation Functions](/nl/guide/creation-functions/http-communication/) - ajax() vs. fromFetch()
- [switchMap()](/nl/guide/operators/transformation/switchMap) - Handige operator voor het annuleren van HTTP-communicatie
- [Foutafhandelingsstrategieën](/nl/guide/error-handling/strategies) - Foutafhandelingspatronen voor HTTP-communicatie

## Referenties

- [RxJS Officiële Documentatie - ajax](https://rxjs.dev/api/ajax/ajax)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/en-US/docs/Web/API/XMLHttpRequest)
- [Learn RxJS - ajax](https://www.learnrxjs.io/learn-rxjs/operators/creation/ajax)
