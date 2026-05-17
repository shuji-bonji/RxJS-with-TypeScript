---
description: "fromFetch() est une fonction de création RxJS qui gère les communications HTTP basées sur l'API Fetch en tant qu'Observable, supportant les méthodes HTTP telles que GET, POST, PUT, DELETE, etc. Elle est légère, moderne, conforme aux standards du web et compatible avec le Service Worker pour une communication HTTP moderne."
---

# fromFetch()

[📘 Documentation officielle RxJS - fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` est une fonction de création pour gérer la communication HTTP en tant qu'Observable basée sur l'API Fetch moderne. Elle est plus légère que `ajax()` et se conforme aux standards modernes du web.

## Utilisation de base

### Requête GET simple

L'exemple le plus simple d'utilisation de `fromFetch()` consiste à passer une URL et à analyser la réponse manuellement.

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
      // Si la réponse est réussie, analyser le JSON
      return response.json();
    } else {
      // Si erreur HTTP, lancer une erreur
      return throwError(() => new Error(`Erreur HTTP: ${response.status}`));
    }
  }),
  catchError((error: unknown) => {
    console.error('Erreur:', error);
    return of({ error: true, message: (error instanceof Error ? error.message : String(error)) });
  })
);

data$.subscribe({
  next: data => console.log('Données:', data),
  error: error => console.error('Erreur de souscription:', error),
  complete: () => console.log('Terminé')
});

// Sortie:
// Données: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Terminé
```

> [!IMPORTANT]
> **Différence importante par rapport à ajax()**
>
> - `fromFetch()` n'appelle pas le callback `error` sur les erreurs HTTP (4xx, 5xx)
> - La propriété `ok` de la réponse doit être vérifiée manuellement
> - Les opérations d'analyse telles que `.json()` sont également effectuées manuellement

## Utilisation par méthode HTTP

### Requête GET

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
      throw new Error(`Erreur HTTP: ${response.status}`);
    }
    return response.json() as Promise<User[]>;
  })
);

users$.subscribe({
  next: users => console.log('Liste des utilisateurs:', users),
  error: error => console.error('Erreur:', error)
});
```

### Requête POST

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
      throw new Error(`Erreur HTTP: ${response.status}`);
    }
    return response.json() as Promise<CreateUserResponse>;
  })
);

createUser$.subscribe({
  next: user => console.log('Création réussie:', user),
  error: error => console.error('Échec de la création:', error)
});
```

### Requête PUT

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
      throw new Error(`Erreur HTTP: ${response.status}`);
    }
    return response.json();
  })
);

updateUser$.subscribe({
  next: user => console.log('Mise à jour réussie:', user),
  error: error => console.error('Échec de la mise à jour:', error)
});
```

### Requête DELETE

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
      throw new Error(`Erreur HTTP: ${response.status}`);
    }
    // DELETE retourne généralement une réponse vide ou seulement le statut
    return response.status === 204 ? of(null) : response.json();
  })
);

deleteUser$.subscribe({
  next: result => console.log('Suppression réussie:', result),
  error: error => console.error('Échec de la suppression:', error)
});
```

## Modèles pratiques

### Fonction générique de gestion des erreurs HTTP

Puisque `fromFetch()` nécessite une vérification manuelle des erreurs, il est utile de créer une fonction générique.

```typescript
import { Observable, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

function fetchJSON<T>(url: string, options?: RequestInit): Observable<T> {
  return fromFetch(url, options).pipe(
    switchMap(response => {
      if (!response.ok) {
        throw new Error(`Erreur HTTP ${response.status}: ${response.statusText}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// Exemple d'utilisation
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

todo$.subscribe({
  next: todo => console.log('Todo:', todo),
  error: error => console.error('Erreur:', error)
});
```

### Traitement détaillé par code d'état HTTP

```typescript
import { throwError, switchMap, of } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/data').pipe(
  switchMap(response => {
    switch (response.status) {
      case 200:
        return response.json();
      case 204:
        // No Content - réponse vide
        return of(null);
      case 401:
        throw new Error('Authentification requise');
      case 403:
        throw new Error('Accès refusé');
      case 404:
        throw new Error('Ressource non trouvée');
      case 500:
        throw new Error('Erreur serveur survenue');
      default:
        throw new Error(`Statut HTTP inattendu: ${response.status}`);
    }
  })
);

api$.subscribe({
  next: data => console.log('Données:', data),
  error: error => console.error('Erreur:', error)
});
```

### Délai d'attente et réessai

```typescript
import { switchMap, timeout, retry } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow-endpoint').pipe(
  timeout(5000), // Timeout après 5 secondes
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`Erreur HTTP: ${response.status}`);
    }
    return response.json();
  }),
  retry(2) // Réessayer deux fois en cas d'échec
);

api$.subscribe({
  next: data => console.log('Données:', data),
  error: error => console.error('Erreur:', error)
});
```

### Annulation de requête (AbortController)

`fromFetch()` supporte l'annulation des requêtes en utilisant le `AbortController` de l'API Fetch.

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const controller = new AbortController();
const signal = controller.signal;

const api$ = fromFetch('https://api.example.com/data', {
  signal // Passer le signal AbortController
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`Erreur HTTP: ${response.status}`);
    }
    return response.json();
  })
);

const subscription = api$.subscribe({
  next: data => console.log('Données:', data),
  error: error => console.error('Erreur:', error)
});

// Annuler la requête après 3 secondes
setTimeout(() => {
  controller.abort();
  // Ou subscription.unsubscribe();
}, 3000);
```

> [!TIP]
> **Annulation automatique par RxJS**
>
> Il suffit d'appeler `unsubscribe()` et RxJS annulera la requête en interne en utilisant `AbortController`. Il n'est pas nécessaire de configurer manuellement un `AbortController`.

### Recherche basée sur les données de l'utilisateur (switchMap)

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
          throw new Error(`Erreur HTTP: ${response.status}`);
        }
        return response.json() as Promise<SearchResult[]>;
      })
    );
  })
);

search$.subscribe({
  next: results => console.log('Résultats de recherche:', results),
  error: error => console.error('Erreur de recherche:', error)
});
```

### Exécuter plusieurs requêtes en parallèle

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
    console.log('Utilisateurs:', users);
    console.log('Posts:', posts);
  },
  error: error => console.error('Une requête a échoué:', error)
});
```

## Cas d'utilisation courants

### 1. Requête avec jeton d'authentification

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
        throw new Error('Authentification requise. Veuillez vous reconnecter.');
      }
      if (!response.ok) {
        throw new Error(`Erreur HTTP: ${response.status}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// Exemple d'utilisation
interface UserProfile {
  id: number;
  name: string;
  email: string;
}

const profile$ = fetchWithAuth<UserProfile>('https://api.example.com/profile');

profile$.subscribe({
  next: profile => console.log('Profil:', profile),
  error: error => console.error('Erreur:', error)
});
```

### 2. Téléchargement de fichier (Blob)

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const downloadFile$ = fromFetch('https://api.example.com/files/report.pdf').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`Erreur HTTP: ${response.status}`);
    }
    // Récupérer comme Blob
    return response.blob();
  })
);

downloadFile$.subscribe({
  next: blob => {
    // Générer un lien de téléchargement à partir du Blob
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'report.pdf';
    a.click();
    window.URL.revokeObjectURL(url);
    console.log('Téléchargement terminé');
  },
  error: error => console.error('Erreur de téléchargement:', error)
});
```

### 3. Requête GraphQL

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
        throw new Error(`Erreur HTTP: ${response.status}`);
      }
      return response.json() as Promise<GraphQLResponse<T>>;
    }),
    map(result => {
      if (result.errors) {
        throw new Error(result.errors.map(e => e.message).join(', '));
      }
      if (!result.data) {
        throw new Error('Aucune donnée retournée');
      }
      return result.data;
    })
  );
}

// Exemple d'utilisation
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
  next: ({ user }) => console.log('Utilisateur:', user),
  error: error => console.error('Erreur:', error)
});
```

### 4. API avec pagination

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

// Exemple d'utilisation
interface Item {
  id: number;
  name: string;
}

const allItems$ = fetchAllPages<Item>('https://api.example.com/items');

allItems$.subscribe({
  next: items => console.log('Tous les éléments:', items),
  error: error => console.error('Erreur:', error)
});
```

## Options fromFetch()

`fromFetch()` peut utiliser l'option `RequestInit` de l'API Fetch sans modification.

```typescript
interface RequestInit {
  method?: string;              // Méthode HTTP (GET, POST, PUT, DELETE, etc.)
  headers?: HeadersInit;        // En-têtes de requête
  body?: BodyInit | null;       // Corps de la requête
  mode?: RequestMode;           // cors, no-cors, same-origin
  credentials?: RequestCredentials; // omit, same-origin, include
  cache?: RequestCache;         // Mode de cache
  redirect?: RequestRedirect;   // Traitement des redirections
  referrer?: string;            // Référent
  integrity?: string;           // Intégrité des sous-ressources
  signal?: AbortSignal;         // Signal AbortController
}
```

## Comparaison ajax() vs fromFetch()

| Fonctionnalité | ajax() | fromFetch() |
|----------------|--------|-------------|
| Technologie de base | XMLHttpRequest | Fetch API |
| Analyse JSON automatique | ✅ `getJSON()` | ❌ Manuelle `.json()` |
| Détection automatique des erreurs HTTP | ✅ Erreur automatique sur 4xx/5xx | ❌ Vérification manuelle de `response.ok` |
| Suivi de progression | ✅ | ❌ |
| Timeout | ✅ Intégré | ❌ Implémenté avec RxJS `timeout()` |
| Annulation de requête | ✅ unsubscribe() | ✅ unsubscribe() ou AbortController |
| Support IE11 | ✅ | ❌ polyfill requis |
| Taille du bundle | Légèrement plus grande | Plus petite |
| Support Service Worker | ❌ | ✅ |

> [!TIP]
> **Comment choisir**
>
> - **Navigateurs modernes uniquement** : `fromFetch()` recommandé
> - **Support des navigateurs anciens nécessaire** : Utilisez `ajax()`
> - **Suivi de progression requis** : Utilisez `ajax()`
> - **Communication HTTP légère** : `fromFetch()` est le meilleur choix
> - **Utilisation dans Service Worker** : Seul `fromFetch()` est supporté

## Erreurs courantes et solutions

### 1. L'erreur HTTP n'est pas capturée dans le callback `error`

**Problème:**
```typescript
// ❌ next est appelé même sur une erreur 404
fromFetch('https://api.example.com/not-found').subscribe({
  next: response => console.log('Succès:', response), // ← Appelé même sur 404
  error: error => console.error('Erreur:', error)
});
```

**Solution:**
```typescript
// ✅ Vérifier manuellement response.ok
fromFetch('https://api.example.com/not-found').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`Erreur HTTP: ${response.status}`);
    }
    return response.json();
  })
).subscribe({
  next: data => console.log('Données:', data),
  error: error => console.error('Erreur:', error) // ← Ceci est appelé
});
```

### 2. Erreur CORS

**Solutions:**
- Définir les en-têtes CORS côté serveur
- Spécifier explicitement `mode: 'cors'`
- Utiliser un serveur proxy pendant le développement

```typescript
fromFetch('https://api.example.com/data', {
  mode: 'cors',
  credentials: 'include' // Si inclusion des cookies
});
```

### 3. Implémentation du timeout

L'API Fetch n'a pas de fonctionnalité de timeout, utilisez donc RxJS `timeout()`.

```typescript
import { timeout, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow').pipe(
  timeout(5000), // Timeout après 5 secondes
  switchMap(response => response.json())
);
```

## Meilleures pratiques

### 1. Créer une fonction générique fetchJSON

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

### 2. Utiliser les types TypeScript

```typescript
// ✅ Bon exemple: Spécifier explicitement le type
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Mauvais exemple: Pas de type spécifié
const todo$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1')
  .pipe(switchMap(res => res.json()));
```

### 3. Toujours implémenter la gestion des erreurs

```typescript
// ✅ Bon exemple: response.ok et catchError
const api$ = fromFetch('/api/data').pipe(
  switchMap(response => {
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return response.json();
  }),
  catchError((error: unknown) => {
    console.error('Erreur:', error);
    return of(defaultValue);
  })
);
```

### 4. Ne pas oublier de se désabonner

```typescript
// ✅ Bon exemple: Libération automatique avec takeUntil
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

## Résumé

`fromFetch()` est une fonction de création légère pour la communication HTTP basée sur l'API moderne Fetch.

**Caractéristiques principales:**
- Basée sur l'API Fetch et conforme aux derniers standards du web
- Légère et de petite taille de bundle
- Peut être utilisée dans un Service Worker
- Vérification manuelle des erreurs et analyse des réponses nécessaires

**Scénarios d'utilisation:**
- Lorsque seuls les navigateurs modernes sont pris en charge
- Lorsque la taille du bundle doit être réduite
- Lorsque la communication HTTP est effectuée dans un Service Worker
- Lorsque vous souhaitez utiliser directement les fonctions de l'API Fetch (ex. objets Request/Response)

**Notes importantes:**
- Le callback `error` n'est pas appelé sur les erreurs HTTP (vérifier `response.ok` manuellement)
- L'analyse JSON est faite manuellement (`response.json()`)
- Le suivi de progression n'est pas supporté
- Un polyfill est nécessaire pour les navigateurs anciens comme IE11

**Utilisation recommandée:**
- Créer une fonction générique `fetchJSON()` pour la réutiliser
- Assurer la sécurité des types en utilisant les types TypeScript
- Toujours implémenter la gestion des erreurs
- Toujours se désabonner lorsque ce n'est plus nécessaire

## Pages connexes

- [ajax()](/fr/guide/creation-functions/http-communication/ajax) - Communication HTTP basée sur XMLHttpRequest
- [Fonctions de création de communication HTTP](/fr/guide/creation-functions/http-communication/) - ajax() vs. fromFetch()
- [switchMap()](/fr/guide/operators/transformation/switchMap) - Opérateur utile pour annuler une communication HTTP
- [Stratégies de gestion des erreurs](/fr/guide/error-handling/strategies) - Modèles de gestion des erreurs pour la communication HTTP

## Références

- [Documentation officielle RxJS - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API)
- [MDN Web Docs - AbortController](https://developer.mozilla.org/en-US/docs/Web/API/AbortController)
