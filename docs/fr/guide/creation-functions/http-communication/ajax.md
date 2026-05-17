---
description: "ajax() est une fonction de création RxJS qui gère la communication HTTP basée sur XMLHttpRequest en tant qu'Observable, supportant les méthodes HTTP telles que GET, POST, PUT, et DELETE, et fournissant des fonctions pratiques telles que le suivi de la progression, le traitement des délais, l'analyse JSON automatique, et le support des navigateurs anciens."
---

# ajax()

[📘 Documentation officielle RxJS - ajax](https://rxjs.dev/api/ajax/ajax)

`ajax()` est une fonction de création pour gérer la communication HTTP basée sur XMLHttpRequest en tant qu'Observable, supportant les méthodes HTTP telles que GET, POST, PUT, et DELETE, et fournissant des fonctions pratiques telles que la surveillance de la progression et la gestion des délais.

## Utilisation de base

### Requête GET simple

L'exemple le plus simple d'utilisation de `ajax()` consiste à passer une URL sous forme de chaîne de caractères.

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax('https://jsonplaceholder.typicode.com/todos/1');

api$.subscribe({
  next: response => console.log('Réponse:', response),
  error: error => console.error('Erreur:', error),
  complete: () => console.log('Terminé')
});

// Sortie:
// Réponse: {
//   status: 200,
//   response: { userId: 1, id: 1, title: "delectus aut autem", completed: false },
//   ...
// }
// Terminé
```

### Récupérer du JSON avec getJSON()

Si vous voulez obtenir des données de l'API JSON, vous pouvez utiliser `ajax.getJSON()`. Il analyse automatiquement la réponse et ne renvoie que la propriété `response`.

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
  error: error => console.error('Erreur:', error),
  complete: () => console.log('Terminé')
});

// Sortie:
// Todo: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Terminé
```

> [!TIP]
> **Sécurité de type TypeScript**
>
> La sécurité de type de la réponse peut être assurée en spécifiant un type générique pour `ajax.getJSON<T>()`.

## Utilisation par méthode HTTP

### Requête GET

```typescript
import { ajax } from 'rxjs/ajax';

// Méthode 1: Spécification simple de chaîne
const get1$ = ajax('https://api.example.com/users');

// Méthode 2: Analyse automatique avec getJSON()
const get2$ = ajax.getJSON('https://api.example.com/users');

// Méthode 3: Configuration détaillée
const get3$ = ajax({
  url: 'https://api.example.com/users',
  method: 'GET',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer token123'
  }
});
```

### Requête POST

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

// Méthode 1: Utilisation de ajax.post()
const post1$ = ajax.post<CreateUserResponse>(
  'https://api.example.com/users',
  newUser,
  { 'Content-Type': 'application/json' }
);

// Méthode 2: Configuration détaillée
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
  next: response => console.log('Création réussie:', response.response),
  error: error => console.error('Échec de la création:', error)
});
```

### Requête PUT

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
  next: response => console.log('Mise à jour réussie:', response.response),
  error: error => console.error('Échec de la mise à jour:', error)
});
```

### Requête PATCH

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
  next: response => console.log('Mise à jour partielle réussie:', response.response),
  error: error => console.error('Échec de la mise à jour partielle:', error)
});
```

### Requête DELETE

```typescript
import { ajax } from 'rxjs/ajax';

const delete$ = ajax.delete('https://api.example.com/users/1');

delete$.subscribe({
  next: response => console.log('Suppression réussie:', response),
  error: error => console.error('Échec de la suppression:', error)
});
```

## Modèles pratiques

### Gestion des erreurs et tentatives

La communication HTTP nécessite de gérer les erreurs du réseau et du serveur.

```typescript
import { of, retry, catchError, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout après 5 secondes
  retry(2), // Réessayer deux fois en cas d'échec
  catchError((error: unknown) => {
    console.error('Erreur de récupération utilisateur:', error);
    // Retourner une valeur par défaut
    return of({
      id: 0,
      name: 'Inconnu',
      email: 'unknown@example.com'
    } as User);
  })
);

fetchUser$.subscribe({
  next: user => console.log('Utilisateur:', user),
  error: error => console.error('Erreur fatale:', error)
});
```

### Branchement conditionnel par code d'état HTTP

```typescript
import { throwError, catchError } from 'rxjs';
import { ajax, AjaxError } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/data').pipe(
  catchError((error: AjaxError) => {
    if (error.status === 404) {
      console.error('Ressource non trouvée');
    } else if (error.status === 401) {
      console.error('Authentification requise');
    } else if (error.status === 500) {
      console.error('Erreur serveur survenue');
    } else {
      console.error('Erreur inattendue:', error);
    }
    return throwError(() => error);
  })
);
```

### Exécuter plusieurs requêtes en parallèle

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

// Attendre que toutes les requêtes soient terminées
forkJoin({
  users: users$,
  posts: posts$,
  comments: comments$
}).subscribe({
  next: ({ users, posts, comments }) => {
    console.log('Utilisateurs:', users);
    console.log('Posts:', posts);
    console.log('Commentaires:', comments);
  },
  error: error => console.error('Une requête a échoué:', error)
});
```

### Recherche basée sur les données de l'utilisateur (switchMap)

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
  debounceTime(300), // Attendre 300ms
  distinctUntilChanged(), // Ignorer les valeurs identiques
  switchMap(query => {
    if (query.length === 0) {
      return of([]);
    }
    // Annuler la requête précédente si une nouvelle requête est saisie
    return ajax.getJSON<SearchResult[]>(`https://api.example.com/search?q=${query}`);
  })
);

search$.subscribe({
  next: results => console.log('Résultats de recherche:', results),
  error: error => console.error('Erreur de recherche:', error)
});
```

> [!IMPORTANT]
> **Importance de switchMap()**
>
> En utilisant `switchMap()`, une requête HTTP précédente est automatiquement annulée lorsqu'une nouvelle requête est saisie. Cela permet d'éviter que les anciens résultats de recherche n'écrasent les nouveaux.

### Suivi de la progression (téléchargement de fichiers)

`ajax()` peut surveiller la progression du chargement et du téléchargement en utilisant l'événement `progress` de `XMLHttpRequest`.

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
    // Activer les événements de progression
    progressSubscriber: {
      next: (progress) => {
        const percentage = (progress.loaded / progress.total) * 100;
        console.log(`Progression du téléchargement: ${percentage.toFixed(2)}%`);
      }
    }
  });

  upload$.subscribe({
    next: response => console.log('Téléchargement terminé:', response),
    error: error => console.error('Échec du téléchargement:', error)
  });
}
```

### En-têtes personnalisés et requêtes inter-domaines

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected-resource',
  method: 'GET',
  headers: {
    'Authorization': 'Bearer your-token-here',
    'X-Custom-Header': 'CustomValue'
  },
  crossDomain: true, // Requête CORS
  withCredentials: true // Inclure les cookies
});

api$.subscribe({
  next: response => console.log('Réponse:', response),
  error: error => console.error('Erreur:', error)
});
```

## Cas d'utilisation courants

### 1. Appel API avec pagination

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
  next: allItems => console.log('Tous les éléments:', allItems),
  error: error => console.error('Erreur:', error)
});
```

### 2. Interrogation (récupération périodique des données)

```typescript
import { interval, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Status {
  status: string;
  lastUpdate: string;
}

// Appeler l'API toutes les 5 secondes
const polling$ = interval(5000).pipe(
  switchMap(() => ajax.getJSON<Status>('https://api.example.com/status'))
);

const subscription = polling$.subscribe({
  next: status => console.log('Statut:', status),
  error: error => console.error('Erreur:', error)
});

// Arrêter après 30 secondes
setTimeout(() => subscription.unsubscribe(), 30000);
```

### 3. Requêtes dépendantes

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

// D'abord récupérer les infos utilisateur, puis récupérer les informations détaillées
const userWithDetails$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  switchMap(user =>
    ajax.getJSON<UserDetails>(`https://api.example.com/users/${user.id}/details`).pipe(
      map(details => ({ ...user, ...details }))
    )
  )
);

userWithDetails$.subscribe({
  next: userWithDetails => console.log('Détails utilisateur:', userWithDetails),
  error: error => console.error('Erreur:', error)
});
```

## Options ajax()

`ajax()` fournit des options pour une configuration avancée.

```typescript
interface AjaxConfig {
  url: string;                    // URL de la requête
  method?: string;                // Méthode HTTP (GET, POST, PUT, DELETE, etc.)
  headers?: object;               // En-têtes de requête
  body?: any;                     // Corps de la requête
  timeout?: number;               // Délai d'attente (en millisecondes)
  responseType?: string;          // Type de réponse (json, text, blob, etc.)
  crossDomain?: boolean;          // Requête CORS ou non
  withCredentials?: boolean;      // Inclure les cookies ou non
  progressSubscriber?: Subscriber; // Subscriber pour le suivi de progression
}
```

## Erreurs courantes et solutions

### 1. Erreur CORS

**Exemple d'erreur:**
```
Access to XMLHttpRequest at 'https://api.example.com' from origin 'http://localhost:3000'
has been blocked by CORS policy
```

**Solutions:**
- Définir l'en-tête CORS côté serveur
- Utiliser un serveur proxy
- Essayer `crossDomain: true` et `withCredentials: false` pendant le développement

### 2. Délai d'attente du réseau

**Solution:**
```typescript
import { timeout, retry } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/slow-endpoint').pipe(
  timeout(10000), // Timeout après 10 secondes
  retry(2) // Réessayer deux fois
);
```

### 3. Erreur d'authentification (401 non autorisé)

**Solution:**
```typescript
import { throwError, catchError, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected',
  headers: {
    'Authorization': `Bearer ${getAccessToken()}`
  }
}).pipe(
  catchError((error: unknown) => {
    if (error.status === 401) {
      // Rafraîchir le token et réessayer
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

## Comparaison ajax() vs fromFetch()

| Fonctionnalité | ajax() | fromFetch() |
|----------------|--------|-------------|
| Analyse JSON automatique | ✅ `getJSON()` | ❌ Manuelle `.json()` |
| Suivi de progression | ✅ | ❌ |
| Détection automatique des erreurs HTTP | ✅ | ❌ |
| Taille du bundle | Légèrement plus grande | Plus petite |
| Support IE11 | ✅ | ❌ |

> [!TIP]
> **Comment choisir**
>
> - **Besoin de suivi de progression** : Utilisez `ajax()`
> - **Support des navigateurs anciens** : Utilisez `ajax()`
> - **Communication HTTP légère** : Considérez `fromFetch()`
> - **Récupération simple de JSON** : `ajax.getJSON()` est le plus simple

## Meilleures pratiques

### 1. Assurer la sécurité des types

```typescript
// ✅ Bon exemple: Spécifier un type générique
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todos$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Mauvais exemple: Pas de type spécifié
const todos$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
```

### 2. Toujours implémenter la gestion des erreurs

```typescript
// ✅ Bon exemple: Gestion des erreurs avec catchError
const api$ = ajax.getJSON('/api/data').pipe(
  catchError((error: unknown) => {
    console.error('Erreur:', error);
    return of(defaultValue);
  })
);

// ❌ Mauvais exemple: Pas de gestion des erreurs
const api$ = ajax.getJSON('/api/data');
```

### 3. Ne pas oublier de se désabonner

```typescript
// ✅ Bon exemple: Se désabonner lors de la destruction du composant
class MyComponent {
  private subscription: Subscription;

  ngOnInit() {
    this.subscription = ajax.getJSON('/api/data').subscribe(...);
  }

  ngOnDestroy() {
    this.subscription.unsubscribe();
  }
}

// Ou utiliser takeUntil
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

## Résumé

`ajax()` est une fonction de création puissante pour la communication HTTP dans RxJS.

**Caractéristiques principales:**
- Basée sur XMLHttpRequest, supporte une large gamme de navigateurs
- Récupération facile de JSON avec `getJSON()`
- Fonctions pratiques telles que le suivi de progression, timeout, retry, etc.
- Détection automatique des erreurs HTTP

**Scénarios d'utilisation:**
- Support des navigateurs anciens (ex. IE11) nécessaire
- Nécessité d'afficher la progression du téléchargement de fichiers
- Appels API JSON simples et directs

**Notes importantes:**
- Toujours mettre en place une gestion des erreurs
- Toujours se désabonner lorsque ce n'est plus nécessaire
- Utiliser les types TypeScript pour assurer la sécurité des types

## Pages connexes

- [fromFetch()](/fr/guide/creation-functions/http-communication/fromFetch) - Communication HTTP basée sur l'API Fetch
- [Fonctions de création de communication HTTP](/fr/guide/creation-functions/http-communication/) - ajax() vs. fromFetch()
- [switchMap()](/fr/guide/operators/transformation/switchMap) - Opérateur utile pour annuler une communication HTTP
- [Stratégies de gestion des erreurs](/fr/guide/error-handling/strategies) - Modèles de gestion des erreurs pour la communication HTTP

## Références

- [Documentation officielle RxJS - ajax](https://rxjs.dev/api/ajax/ajax)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/en-US/docs/Web/API/XMLHttpRequest)
- [Learn RxJS - ajax](https://www.learnrxjs.io/learn-rxjs/operators/creation/ajax)
