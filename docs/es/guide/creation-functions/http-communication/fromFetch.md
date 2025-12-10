---
description: fromFetch() es una Función de Creación de RxJS que maneja comunicación HTTP basada en Fetch API como Observable, soportando métodos HTTP como GET, POST, PUT, DELETE, etc. Es ligera, moderna, cumple con estándares web y compatible con Service Worker para comunicación HTTP moderna. Se requiere verificación manual de errores y parseo JSON, pero el tamaño del bundle es pequeño y es posible comunicación HTTP con seguridad de tipos.
---

# fromFetch()

[📘 Documentación Oficial RxJS - fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` es una Función de Creación para manejar comunicación HTTP como Observable basada en la moderna Fetch API. Es ligera comparada con `ajax()` y cumple con estándares web modernos.

## Uso Básico

### Solicitud GET Simple

El ejemplo más simple de usar `fromFetch()` es pasar una URL y parsear la respuesta manualmente.

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
      // Si la respuesta es exitosa, parsear JSON
      return response.json();
    } else {
      // Si es error HTTP, lanzar error
      return throwError(() => new Error(`Error HTTP: ${response.status}`));
    }
  }),
  catchError(error => {
    console.error('Error:', error);
    return of({ error: true, message: error.message });
  })
);

data$.subscribe({
  next: data => console.log('Datos:', data),
  error: error => console.error('Error de suscripción:', error),
  complete: () => console.log('Completado')
});

// Salida:
// Datos: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Completado
```

> [!IMPORTANT]
> **Diferencia Importante con ajax()**
>
> - `fromFetch()` no llama al callback `error` en errores HTTP (4xx, 5xx)
> - La propiedad `ok` de la respuesta debe verificarse manualmente
> - Operaciones de parseo como `.json()` también se hacen manualmente

## Uso por Método HTTP

### Solicitud GET

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
      throw new Error(`Error HTTP: ${response.status}`);
    }
    return response.json() as Promise<User[]>;
  })
);

users$.subscribe({
  next: users => console.log('Lista de usuarios:', users),
  error: error => console.error('Error:', error)
});
```

### Solicitud POST

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
  name: 'Juan García',
  email: 'juan@example.com'
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
      throw new Error(`Error HTTP: ${response.status}`);
    }
    return response.json() as Promise<CreateUserResponse>;
  })
);

createUser$.subscribe({
  next: user => console.log('Creación exitosa:', user),
  error: error => console.error('Creación fallida:', error)
});
```

### Solicitud PUT

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

interface UpdateUserRequest {
  name: string;
  email: string;
}

const updatedUser: UpdateUserRequest = {
  name: 'Pedro López',
  email: 'pedro@example.com'
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
      throw new Error(`Error HTTP: ${response.status}`);
    }
    return response.json();
  })
);

updateUser$.subscribe({
  next: user => console.log('Actualización exitosa:', user),
  error: error => console.error('Actualización fallida:', error)
});
```

### Solicitud DELETE

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
      throw new Error(`Error HTTP: ${response.status}`);
    }
    // DELETE usualmente retorna respuesta vacía o solo estado
    return response.status === 204 ? of(null) : response.json();
  })
);

deleteUser$.subscribe({
  next: result => console.log('Eliminación exitosa:', result),
  error: error => console.error('Eliminación fallida:', error)
});
```

## Patrones Prácticos

### Función Genérica de Manejo de Errores HTTP

Dado que `fromFetch()` requiere verificación manual de errores, es útil crear una función genérica.

```typescript
import { Observable, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

function fetchJSON<T>(url: string, options?: RequestInit): Observable<T> {
  return fromFetch(url, options).pipe(
    switchMap(response => {
      if (!response.ok) {
        throw new Error(`Error HTTP ${response.status}: ${response.statusText}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// Ejemplo de uso
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

### Procesamiento Detallado por Código de Estado HTTP

```typescript
import { throwError, switchMap, of } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/data').pipe(
  switchMap(response => {
    switch (response.status) {
      case 200:
        return response.json();
      case 204:
        // Sin Contenido - respuesta vacía
        return of(null);
      case 401:
        throw new Error('Autenticación requerida');
      case 403:
        throw new Error('Acceso denegado');
      case 404:
        throw new Error('Recurso no encontrado');
      case 500:
        throw new Error('Error del servidor ocurrido');
      default:
        throw new Error(`Estado HTTP inesperado: ${response.status}`);
    }
  })
);

api$.subscribe({
  next: data => console.log('Datos:', data),
  error: error => console.error('Error:', error)
});
```

### Timeout y Reintento

```typescript
import { switchMap, timeout, retry } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow-endpoint').pipe(
  timeout(5000), // Timeout en 5 segundos
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`Error HTTP: ${response.status}`);
    }
    return response.json();
  }),
  retry(2) // Reintentar dos veces en caso de fallo
);

api$.subscribe({
  next: data => console.log('Datos:', data),
  error: error => console.error('Error:', error)
});
```

### Cancelación de Solicitud (AbortController)

`fromFetch()` soporta cancelar solicitudes usando el `AbortController` de Fetch API.

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const controller = new AbortController();
const signal = controller.signal;

const api$ = fromFetch('https://api.example.com/data', {
  signal // Pasar señal de AbortController
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`Error HTTP: ${response.status}`);
    }
    return response.json();
  })
);

const subscription = api$.subscribe({
  next: data => console.log('Datos:', data),
  error: error => console.error('Error:', error)
});

// Cancelar solicitud después de 3 segundos
setTimeout(() => {
  controller.abort();
  // O subscription.unsubscribe();
}, 3000);
```

> [!TIP]
> **Cancelación Automática por RxJS**
>
> Solo llamar `unsubscribe()` y RxJS cancelará la solicitud internamente usando `AbortController`. No hay necesidad de configurar manualmente un `AbortController`.

### Búsqueda Basada en Entrada de Usuario (switchMap)

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
          throw new Error(`Error HTTP: ${response.status}`);
        }
        return response.json() as Promise<SearchResult[]>;
      })
    );
  })
);

search$.subscribe({
  next: results => console.log('Resultados de búsqueda:', results),
  error: error => console.error('Error de búsqueda:', error)
});
```

### Ejecutar Múltiples Solicitudes en Paralelo

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
    console.log('Usuarios:', users);
    console.log('Posts:', posts);
  },
  error: error => console.error('Alguna solicitud falló:', error)
});
```

## Casos de Uso Comunes

### 1. Solicitud con Token de Autenticación

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
        throw new Error('Autenticación requerida. Por favor inicia sesión de nuevo.');
      }
      if (!response.ok) {
        throw new Error(`Error HTTP: ${response.status}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// Ejemplo de uso
interface UserProfile {
  id: number;
  name: string;
  email: string;
}

const profile$ = fetchWithAuth<UserProfile>('https://api.example.com/profile');

profile$.subscribe({
  next: profile => console.log('Perfil:', profile),
  error: error => console.error('Error:', error)
});
```

### 2. Descarga de Archivo (Blob)

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const downloadFile$ = fromFetch('https://api.example.com/files/report.pdf').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`Error HTTP: ${response.status}`);
    }
    // Obtener como Blob
    return response.blob();
  })
);

downloadFile$.subscribe({
  next: blob => {
    // Generar enlace de descarga desde Blob
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'report.pdf';
    a.click();
    window.URL.revokeObjectURL(url);
    console.log('Descarga completada');
  },
  error: error => console.error('Error de descarga:', error)
});
```

### 3. Consulta GraphQL

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
        throw new Error(`Error HTTP: ${response.status}`);
      }
      return response.json() as Promise<GraphQLResponse<T>>;
    }),
    map(result => {
      if (result.errors) {
        throw new Error(result.errors.map(e => e.message).join(', '));
      }
      if (!result.data) {
        throw new Error('No se retornaron datos');
      }
      return result.data;
    })
  );
}

// Ejemplo de uso
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
  next: ({ user }) => console.log('Usuario:', user),
  error: error => console.error('Error:', error)
});
```

### 4. API con Paginación

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

// Ejemplo de uso
interface Item {
  id: number;
  name: string;
}

const allItems$ = fetchAllPages<Item>('https://api.example.com/items');

allItems$.subscribe({
  next: items => console.log('Todos los items:', items),
  error: error => console.error('Error:', error)
});
```

## Opciones de fromFetch()

`fromFetch()` puede usar la opción `RequestInit` de Fetch API sin modificación.

```typescript
interface RequestInit {
  method?: string;              // Método HTTP (GET, POST, PUT, DELETE, etc.)
  headers?: HeadersInit;        // Header de solicitud
  body?: BodyInit | null;       // Cuerpo de solicitud
  mode?: RequestMode;           // cors, no-cors, same-origin
  credentials?: RequestCredentials; // omit, same-origin, include
  cache?: RequestCache;         // Modo de caché
  redirect?: RequestRedirect;   // Procesamiento de redirección
  referrer?: string;            // Referrer
  integrity?: string;           // Integridad de subrecurso
  signal?: AbortSignal;         // Señal de AbortController
}
```

## Comparación ajax() vs fromFetch()

| Característica | ajax() | fromFetch() |
|------|--------|-------------|
| Tecnología Base | XMLHttpRequest | Fetch API |
| Parseo JSON automático | ✅ `getJSON()` | ❌ Manualmente `.json()` |
| Detección automática de errores HTTP | ✅ Error automático en 4xx/5xx | ❌ Verificar `response.ok` manualmente |
| Monitoreo de progreso | ✅ | ❌ |
| Timeout | ✅ Integrado | ❌ Implementado con RxJS `timeout()` |
| Cancelación de solicitud | ✅ unsubscribe() | ✅ unsubscribe() o AbortController |
| Soporte IE11 | ✅ | ❌ requiere polyfill |
| Tamaño del bundle | Ligeramente más grande | Más pequeño |
| Soporte Service Worker | ❌ | ✅ |

> [!TIP]
> **Cómo Elegir**
>
> - **Solo navegadores modernos**: `fromFetch()` recomendado
> - **Necesita soporte de navegadores legacy**: Usar `ajax()`
> - **Monitoreo de progreso requerido**: Usar `ajax()`
> - **Comunicación HTTP ligera**: `fromFetch()` es lo mejor
> - **Uso en Service Worker**: Solo `fromFetch()` soportado

## Errores Comunes y Soluciones

### 1. Error HTTP No Capturado en Callback `error`

**Problema:**
```typescript
// ❌ next se llama incluso en error 404
fromFetch('https://api.example.com/not-found').subscribe({
  next: response => console.log('Éxito:', response), // ← Llamado incluso en 404
  error: error => console.error('Error:', error)
});
```

**Solución:**
```typescript
// ✅ Verificar response.ok manualmente
fromFetch('https://api.example.com/not-found').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`Error HTTP: ${response.status}`);
    }
    return response.json();
  })
).subscribe({
  next: data => console.log('Datos:', data),
  error: error => console.error('Error:', error) // ← Esto se llama
});
```

### 2. Error CORS

**Soluciones:**
- Establecer headers CORS en el lado del servidor
- Especificar explícitamente `mode: 'cors'`
- Usar servidor proxy durante desarrollo

```typescript
fromFetch('https://api.example.com/data', {
  mode: 'cors',
  credentials: 'include' // Si se incluyen cookies
});
```

### 3. Implementar Timeout

Fetch API no tiene funcionalidad de timeout, así que usar RxJS `timeout()`.

```typescript
import { timeout, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow').pipe(
  timeout(5000), // Timeout en 5 segundos
  switchMap(response => response.json())
);
```

## Mejores Prácticas

### 1. Crear Función fetchJSON Genérica

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

### 2. Utilizar Tipos TypeScript

```typescript
// ✅ Buen ejemplo: Especificar tipo explícitamente
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Mal ejemplo: Sin tipo especificado
const todo$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1')
  .pipe(switchMap(res => res.json()));
```

### 3. Siempre Implementar Manejo de Errores

```typescript
// ✅ Buen ejemplo: response.ok y catchError
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

### 4. Recordar Desuscribirse

```typescript
// ✅ Buen ejemplo: Liberación automática con takeUntil
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

## Resumen

`fromFetch()` es una Función de Creación ligera para comunicación HTTP basada en la moderna Fetch API.

**Características Clave:**
- Basado en Fetch API y cumple con los últimos estándares web
- Ligero y tamaño de bundle pequeño
- Puede usarse dentro de un Service Worker
- Requiere verificación manual de errores y parseo de respuesta

**Escenarios de Uso:**
- Cuando solo se soportan navegadores modernos
- Cuando el tamaño del bundle necesita reducirse
- Cuando se realiza comunicación HTTP dentro de un Service Worker
- Cuando quieres usar funciones de Fetch API (ej., objetos Request/Response) directamente

**Notas Importantes:**
- El callback `error` no se llama en errores HTTP (verificar `response.ok` manualmente)
- El parseo JSON se hace manualmente (`response.json()`)
- El monitoreo de progreso no está soportado
- Se requiere polyfill para navegadores legacy como IE11

**Uso Recomendado:**
- Crear función `fetchJSON()` genérica para reutilizar
- Asegurar seguridad de tipos utilizando tipos TypeScript
- Siempre implementar manejo de errores
- Siempre desuscribirse cuando ya no se necesita

## Páginas Relacionadas

- [ajax()](/es/guide/creation-functions/http-communication/ajax) - Comunicación HTTP basada en XMLHttpRequest
- [Funciones de Creación de Comunicación HTTP](/es/guide/creation-functions/http-communication/) - ajax() vs. fromFetch()
- [switchMap()](/es/guide/operators/transformation/switchMap) - Operador útil para cancelar comunicación HTTP
- [Estrategias de Manejo de Errores](/es/guide/error-handling/strategies) - Patrones de manejo de errores para comunicación HTTP

## Referencias

- [Documentación Oficial RxJS - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/es/docs/Web/API/Fetch_API)
- [MDN Web Docs - AbortController](https://developer.mozilla.org/es/docs/Web/API/AbortController)
