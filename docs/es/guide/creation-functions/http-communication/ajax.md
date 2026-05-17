---
description: ajax() es una Función de Creación de RxJS que maneja comunicación HTTP basada en XMLHttpRequest como Observable, soportando métodos HTTP como GET, POST, PUT y DELETE, y proporcionando funciones prácticas como monitoreo de progreso, procesamiento de timeout, parseo JSON automático y soporte para navegadores legacy. getJSON() puede llamar fácilmente a la API JSON para lograr comunicación HTTP con seguridad de tipos.
---

# ajax()

[📘 Documentación Oficial RxJS - ajax](https://rxjs.dev/api/ajax/ajax)

`ajax()` es una Función de Creación para manejar comunicación HTTP basada en XMLHttpRequest como Observable, soportando métodos HTTP como GET, POST, PUT y DELETE, y proporcionando funciones prácticas como monitoreo de progreso y manejo de timeout.

## Uso Básico

### Solicitud GET Simple

El ejemplo más simple de usar `ajax()` es simplemente pasar una URL como string.

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax('https://jsonplaceholder.typicode.com/todos/1');

api$.subscribe({
  next: response => console.log('Respuesta:', response),
  error: error => console.error('Error:', error),
  complete: () => console.log('Completado')
});

// Salida:
// Respuesta: {
//   status: 200,
//   response: { userId: 1, id: 1, title: "delectus aut autem", completed: false },
//   ...
// }
// Completado
```

### Obtener JSON usando getJSON()

Si quieres obtener datos de la API JSON, puedes usar `ajax.getJSON()`. Automáticamente parsea la respuesta y retorna solo la propiedad `response`.

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
  error: error => console.error('Error:', error),
  complete: () => console.log('Completado')
});

// Salida:
// Todo: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// Completado
```

> [!TIP]
> **Seguridad de Tipos TypeScript**
>
> La seguridad de tipos de la respuesta puede asegurarse especificando un tipo genérico para `ajax.getJSON<T>()`.

## Uso por Método HTTP

### Solicitud GET

```typescript
import { ajax } from 'rxjs/ajax';

// Método 1: Especificación de string simple
const get1$ = ajax('https://api.example.com/users');

// Método 2: Parseo automático con getJSON()
const get2$ = ajax.getJSON('https://api.example.com/users');

// Método 3: Configuración detallada
const get3$ = ajax({
  url: 'https://api.example.com/users',
  method: 'GET',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer token123'
  }
});
```

### Solicitud POST

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
  name: 'Juan García',
  email: 'juan@example.com'
};

// Método 1: Usando ajax.post()
const post1$ = ajax.post<CreateUserResponse>(
  'https://api.example.com/users',
  newUser,
  { 'Content-Type': 'application/json' }
);

// Método 2: Configuración detallada
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
  next: response => console.log('Creación exitosa:', response.response),
  error: error => console.error('Creación fallida:', error)
});
```

### Solicitud PUT

```typescript
import { ajax } from 'rxjs/ajax';

interface UpdateUserRequest {
  name: string;
  email: string;
}

const updatedUser: UpdateUserRequest = {
  name: 'Pedro López',
  email: 'pedro@example.com'
};

const put$ = ajax.put(
  'https://api.example.com/users/1',
  updatedUser,
  { 'Content-Type': 'application/json' }
);

put$.subscribe({
  next: response => console.log('Actualización exitosa:', response.response),
  error: error => console.error('Actualización fallida:', error)
});
```

### Solicitud PATCH

```typescript
import { ajax } from 'rxjs/ajax';

interface PatchUserRequest {
  email?: string;
}

const patch$ = ajax.patch(
  'https://api.example.com/users/1',
  { email: 'nuevo-email@example.com' } as PatchUserRequest,
  { 'Content-Type': 'application/json' }
);

patch$.subscribe({
  next: response => console.log('Actualización parcial exitosa:', response.response),
  error: error => console.error('Actualización parcial fallida:', error)
});
```

### Solicitud DELETE

```typescript
import { ajax } from 'rxjs/ajax';

const delete$ = ajax.delete('https://api.example.com/users/1');

delete$.subscribe({
  next: response => console.log('Eliminación exitosa:', response),
  error: error => console.error('Eliminación fallida:', error)
});
```

## Patrones Prácticos

### Manejo de Errores y Reintentos

La comunicación HTTP requiere manejar errores de red y servidor.

```typescript
import { of, retry, catchError, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout en 5 segundos
  retry(2), // Reintentar dos veces en caso de fallo
  catchError((error: unknown) => {
    console.error('Error al obtener usuario:', error);
    // Retornar valor predeterminado
    return of({
      id: 0,
      name: 'Desconocido',
      email: 'desconocido@example.com'
    } as User);
  })
);

fetchUser$.subscribe({
  next: user => console.log('Usuario:', user),
  error: error => console.error('Error fatal:', error)
});
```

### Ramificación Condicional por Código de Estado HTTP

```typescript
import { throwError, catchError } from 'rxjs';
import { ajax, AjaxError } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/data').pipe(
  catchError((error: AjaxError) => {
    if (error.status === 404) {
      console.error('Recurso no encontrado');
    } else if (error.status === 401) {
      console.error('Autenticación requerida');
    } else if (error.status === 500) {
      console.error('Error del servidor ocurrido');
    } else {
      console.error('Error inesperado:', error);
    }
    return throwError(() => error);
  })
);
```

### Ejecutar Múltiples Solicitudes en Paralelo

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

// Esperar a que todas las solicitudes se completen
forkJoin({
  users: users$,
  posts: posts$,
  comments: comments$
}).subscribe({
  next: ({ users, posts, comments }) => {
    console.log('Usuarios:', users);
    console.log('Posts:', posts);
    console.log('Comentarios:', comments);
  },
  error: error => console.error('Alguna solicitud falló:', error)
});
```

### Búsqueda Basada en Entrada de Usuario (switchMap)

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
  debounceTime(300), // Esperar 300ms
  distinctUntilChanged(), // Ignorar mismo valor
  switchMap(query => {
    if (query.length === 0) {
      return of([]);
    }
    // Cancelar solicitud anterior si se ingresa nueva consulta
    return ajax.getJSON<SearchResult[]>(`https://api.example.com/search?q=${query}`);
  })
);

search$.subscribe({
  next: results => console.log('Resultados de búsqueda:', results),
  error: error => console.error('Error de búsqueda:', error)
});
```

> [!IMPORTANT]
> **Importancia de switchMap()**
>
> Usando `switchMap()`, una solicitud HTTP anterior se cancela automáticamente cuando se ingresa una nueva consulta de búsqueda. Esto previene que resultados de búsqueda antiguos sobrescriban nuevos resultados.

### Monitoreo de Progreso (Carga de Archivo)

`ajax()` puede monitorear el progreso de carga y descarga usando el evento `progress` de `XMLHttpRequest`.

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
    // Habilitar eventos de progreso
    progressSubscriber: {
      next: (progress) => {
        const percentage = (progress.loaded / progress.total) * 100;
        console.log(`Progreso de carga: ${percentage.toFixed(2)}%`);
      }
    }
  });

  upload$.subscribe({
    next: response => console.log('Carga completada:', response),
    error: error => console.error('Carga fallida:', error)
  });
}
```

### Headers Personalizados y Solicitudes Cross-Domain

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected-resource',
  method: 'GET',
  headers: {
    'Authorization': 'Bearer your-token-here',
    'X-Custom-Header': 'CustomValue'
  },
  crossDomain: true, // Solicitud CORS
  withCredentials: true // Incluir cookies
});

api$.subscribe({
  next: response => console.log('Respuesta:', response),
  error: error => console.error('Error:', error)
});
```

## Casos de Uso Comunes

### 1. Llamada API con Paginación

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
  next: allItems => console.log('Todos los items:', allItems),
  error: error => console.error('Error:', error)
});
```

### 2. Polling (Obtención Periódica de Datos)

```typescript
import { interval, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Status {
  status: string;
  lastUpdate: string;
}

// Llamar API cada 5 segundos
const polling$ = interval(5000).pipe(
  switchMap(() => ajax.getJSON<Status>('https://api.example.com/status'))
);

const subscription = polling$.subscribe({
  next: status => console.log('Estado:', status),
  error: error => console.error('Error:', error)
});

// Detener después de 30 segundos
setTimeout(() => subscription.unsubscribe(), 30000);
```

### 3. Solicitudes Dependientes

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

// Primero obtener info de usuario, luego obtener información detallada
const userWithDetails$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  switchMap(user =>
    ajax.getJSON<UserDetails>(`https://api.example.com/users/${user.id}/details`).pipe(
      map(details => ({ ...user, ...details }))
    )
  )
);

userWithDetails$.subscribe({
  next: userWithDetails => console.log('Detalles de usuario:', userWithDetails),
  error: error => console.error('Error:', error)
});
```

## Opciones de ajax()

`ajax()` proporciona opciones para configuración avanzada.

```typescript
interface AjaxConfig {
  url: string;                    // URL de solicitud
  method?: string;                // Método HTTP (GET, POST, PUT, DELETE, etc.)
  headers?: object;               // Header de solicitud
  body?: any;                     // Cuerpo de solicitud
  timeout?: number;               // Tiempo de timeout (en milisegundos)
  responseType?: string;          // Tipo de respuesta (json, text, blob, etc.)
  crossDomain?: boolean;          // Si es solicitud CORS
  withCredentials?: boolean;      // Si incluir cookies
  progressSubscriber?: Subscriber; // Suscriptor para monitoreo de progreso
}
```

## Errores Comunes y Soluciones

### 1. Error CORS

**Ejemplo de Error:**
```
Access to XMLHttpRequest at 'https://api.example.com' from origin 'http://localhost:3000'
has been blocked by CORS policy
```

**Soluciones:**
- Establecer el header CORS en el lado del servidor
- Usar un servidor proxy
- Intentar `crossDomain: true` y `withCredentials: false` durante desarrollo

### 2. Timeout de Red

**Solución:**
```typescript
import { timeout, retry } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/slow-endpoint').pipe(
  timeout(10000), // Timeout en 10 segundos
  retry(2) // Reintentar dos veces
);
```

### 3. Error de Autenticación (401 Unauthorized)

**Solución:**
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
      // Refrescar el token y reintentar
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

## Comparación ajax() vs fromFetch()

| Característica | ajax() | fromFetch() |
|------|--------|-------------|
| Parseo JSON automático | ✅ `getJSON()` | ❌ Manualmente `.json()` |
| Monitoreo de progreso | ✅ | ❌ |
| Detección automática de errores HTTP | ✅ | ❌ |
| Tamaño del bundle | Ligeramente más grande | Más pequeño |
| Soporte IE11 | ✅ | ❌ |

> [!TIP]
> **Cómo Elegir**
>
> - **Necesita monitoreo de progreso**: Usar `ajax()`
> - **Soporte de navegadores legacy**: Usar `ajax()`
> - **Comunicación HTTP ligera**: Considerar `fromFetch()`
> - **Obtención simple de JSON**: `ajax.getJSON()` es lo más fácil

## Mejores Prácticas

### 1. Asegurar Seguridad de Tipos

```typescript
// ✅ Buen ejemplo: Especificar tipo genérico
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todos$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ Mal ejemplo: Sin tipo especificado
const todos$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
```

### 2. Siempre Implementar Manejo de Errores

```typescript
// ✅ Buen ejemplo: Manejo de errores con catchError
const api$ = ajax.getJSON('/api/data').pipe(
  catchError((error: unknown) => {
    console.error('Error:', error);
    return of(defaultValue);
  })
);

// ❌ Mal ejemplo: Sin manejo de errores
const api$ = ajax.getJSON('/api/data');
```

### 3. Recordar Desuscribirse

```typescript
// ✅ Buen ejemplo: Desuscribirse al destruir componente
class MyComponent {
  private subscription: Subscription;

  ngOnInit() {
    this.subscription = ajax.getJSON('/api/data').subscribe(...);
  }

  ngOnDestroy() {
    this.subscription.unsubscribe();
  }
}

// O usar takeUntil
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

## Resumen

`ajax()` es una Función de Creación poderosa para comunicación HTTP en RxJS.

**Características Clave:**
- Basado en XMLHttpRequest, soporta amplia gama de navegadores
- Obtención fácil de JSON con `getJSON()`
- Funciones prácticas como monitoreo de progreso, timeout, reintento, etc.
- Detección automática de errores HTTP

**Escenarios de Uso:**
- Soporte de navegadores legacy (ej. IE11) requerido
- Necesita mostrar progreso de carga/descarga de archivos
- Llamadas API JSON simples y directas

**Notas Importantes:**
- Siempre implementar manejo de errores
- Siempre desuscribirse cuando ya no se necesita
- Utilizar tipos TypeScript para asegurar seguridad de tipos

## Páginas Relacionadas

- [fromFetch()](/es/guide/creation-functions/http-communication/fromFetch) - Comunicación HTTP basada en Fetch API
- [Funciones de Creación de Comunicación HTTP](/es/guide/creation-functions/http-communication/) - ajax() vs. fromFetch()
- [switchMap()](/es/guide/operators/transformation/switchMap) - Operador útil para cancelar comunicación HTTP
- [Estrategias de Manejo de Errores](/es/guide/error-handling/strategies) - Patrones de manejo de errores para comunicación HTTP

## Referencias

- [Documentación Oficial RxJS - ajax](https://rxjs.dev/api/ajax/ajax)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/es/docs/Web/API/XMLHttpRequest)
- [Learn RxJS - ajax](https://www.learnrxjs.io/learn-rxjs/operators/creation/ajax)
