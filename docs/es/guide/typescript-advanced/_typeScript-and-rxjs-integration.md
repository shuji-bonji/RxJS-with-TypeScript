---
description: Diseño reactivo robusto y con seguridad de tipos a través de definiciones de tipos de operadores personalizados, utilización de tipos condicionales y patrones de gestión de estado en la integración de TypeScript y RxJS.
---

# Integración de TypeScript y RxJS

Este documento presenta diversas técnicas y mejores prácticas para utilizar RxJS con seguridad de tipos en TypeScript.

## Qué aprenderás en este documento

- Cómo trabajar con tipos Observable en TypeScript
- Cómo definir tipos de operadores personalizados
- Uso de tipos condicionales y de utilidad
- Gestión de estado con seguridad de tipos usando RxJS
- Configuraciones recomendadas para tsconfig.json y por qué

La combinación de TypeScript y RxJS permite la programación asíncrona mientras se mantiene la seguridad de tipos.
Este documento te mostrará cómo utilizar efectivamente RxJS con TypeScript.

## Utilizando definiciones de tipos

### Especificando el tipo Observable

Una de las mayores ventajas de usar RxJS con TypeScript es la capacidad de definir explícitamente el tipo de valores que fluyen en un Observable.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

// Definición de tipo explícita
const numbers$: Observable<number> = of(1, 2, 3);

// Transformación usando tipos genéricos
interface User {
  id: number;
  name: string;
}

const users$: Observable<User> = of(
  { id: 1, name: '山田' },
  { id: 2, name: '佐藤' }
);

// Operación donde los tipos se transforman
const userNames$: Observable<string> = users$.pipe(
  map(user => user.name)
);
```

### Definición de tipos de operadores personalizados

Los tipos también pueden manejarse correctamente al crear operadores personalizados.

```ts
import { Observable, of, OperatorFunction } from 'rxjs';
import { map } from 'rxjs';

// Definir operadores personalizados con seguridad de tipos usando OperatorFunction
function doubleMap<T, R, S>(
  first: (value: T, index: number) => R,
  second: (value: R, index: number) => S
): OperatorFunction<T, S> {
  return (source: Observable<T>) => source.pipe(map(first), map(second));
}

// Ejemplo de uso
of(1, 2, 3)
  .pipe(
    doubleMap(
      (x) => x * 2,
      (x) => `Resultado: ${x}`
    )
  )
  .subscribe(console.log);
// Resultado: 2
// Resultado: 4
// Resultado: 6
```

## Interfaces y type aliases

Para diseños complejos basados en eventos, tener la estructura del evento definida por tipo hace que la integración con RxJS sea mucho más eficiente.

Cuando se trabaja con estructuras de datos complejas, es útil definir interfaces y type aliases.

```ts
// Definición de tipo de evento
interface AppEvent {
  type: string;
  payload: unknown;
}

// Tipos de eventos específicos
interface UserLoginEvent extends AppEvent {
  type: 'USER_LOGIN';
  payload: {
    userId: string;
    timestamp: number;
  };
}

interface DataUpdateEvent extends AppEvent {
  type: 'DATA_UPDATE';
  payload: {
    items: Array<{ id: string; value: number }>;
  };
}

// Composición de tipos de eventos
type ApplicationEvent = UserLoginEvent | DataUpdateEvent;

// Implementación del event bus
const eventBus$ = new Subject<ApplicationEvent>();

// Publicación de eventos con seguridad de tipos
eventBus$.next({
  type: 'USER_LOGIN',
  payload: {
    userId: 'user123',
    timestamp: Date.now(),
  },
});

// Filtrado de eventos con seguridad de tipos
const userLoginEvents$ = eventBus$.pipe(
  filter((event): event is UserLoginEvent => event.type === 'USER_LOGIN')
);

userLoginEvents$.subscribe((event) => {
  // Aquí event.payload.userId puede accederse con seguridad de tipos
  console.log(`Usuario inició sesión: ${event.payload.userId}`);
});
```

## Utilizando tipos avanzados

### Utility types

Puedes mejorar aún más tu integración con RxJS aprovechando los utility types de TypeScript.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
  role: 'admin' | 'user';
}

// Seleccionar un subconjunto de propiedades usando Pick
type UserBasicInfo = Pick<User, 'id' | 'name'>;

const users$: Observable<User> = fetchUsers();
const usersBasicInfo$: Observable<UserBasicInfo> = users$.pipe(
  map(user => ({ id: user.id, name: user.name }))
);

// Usar Omit para excluir propiedades específicas
type UserPublicInfo = Omit<User, 'email'>;

// Usar Partial para hacer todas las propiedades opcionales
type UserUpdate = Partial<User>;

function updateUser(id: number, update: UserUpdate): Observable<User> {
  return patchUser(id, update);
}
```

### Tipos Condicionales y de Mapeo

Para casos más complejos, se pueden utilizar tipos condicionales y de mapeo.

```ts
import { filter, map, Observable} from 'rxjs';

// Tipos de respuesta API
type ApiResponse<T> =
  | { status: 'success'; data: T; }
  | { status: 'error'; error: string; };

// Tipo que extrae solo datos de la respuesta
type ExtractData<T> = T extends ApiResponse<infer U> ? U : never;

function handleApiResponse<T>(response$: Observable<ApiResponse<T>>): Observable<T> {
  return response$.pipe(
    filter((response): response is ApiResponse<T> & { status: 'success' } =>
      response.status === 'success'
    ),
    map(response => response.data)
  );
}

// Ejemplo de uso
const userResponse$: Observable<ApiResponse<User>> = fetchUserApi(1);
const user$: Observable<User> = handleApiResponse(userResponse$);
```

## Optimizar tsconfig.json

La configuración adecuada de tsconfig.json es importante para el uso efectivo de RxJS y TypeScript.

```json
{
  "compilerOptions": {
    "target": "es2020",
    "module": "esnext",
    "moduleResolution": "node",
    "strict": true,
    "noImplicitAny": true,
    "strictNullChecks": true,
    "noUnusedLocals": true,
    "noUnusedParameters": true,
    "esModuleInterop": true,
    "sourceMap": true,
    "declaration": true,
    "lib": ["es2020", "dom"]
  }
}
```

Las siguientes configuraciones son de particular importancia:

💡 En particular, `"strict": true` es esencial para aprovechar al máximo RxJS.

- `strict`: verificación de tipos estricta para aprovechar completamente la seguridad de tipos de RxJS
- `noImplicitAny`: no permitir tipos any implícitos
- `strictNullChecks`: null/undefined deben manejarse explícitamente

## Optimización de las importaciones de RxJS

Al usar RxJS en un proyecto TypeScript, el método de importación también es importante.

```ts
// Método recomendado
import { Observable, of, from } from 'rxjs';
import { map, filter, catchError } from 'rxjs';
```

## Patrón RxJS para gestión de estado (configuración sin Redux)

Esta sección muestra cómo construir gestión de estado usando solo RxJS sin usar Redux o NgRx.

```ts
// Interfaz del estado de la aplicación
interface AppState {
  user: User | null;
  isLoading: boolean;
  data: Record<string, unknown>;
  error: Error | null;
}

// Estado inicial
const initialState: AppState = {
  user: null,
  isLoading: false,
  data: {},
  error: null
};

// BehaviorSubject para gestión de estado
const state$ = new BehaviorSubject<AppState>(initialState);

// Tipos de acción
type Action =
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_USER'; payload: User | null }
  | { type: 'SET_DATA'; payload: Record<string, unknown> }
  | { type: 'SET_ERROR'; payload: Error | null };

// Función de actualización de estado
function reducer(state: AppState, action: Action): AppState {
  switch (action.type) {
    case 'SET_LOADING':
      return { ...state, isLoading: action.payload };
    case 'SET_USER':
      return { ...state, user: action.payload };
    case 'SET_DATA':
      return { ...state, data: action.payload };
    case 'SET_ERROR':
      return { ...state, error: action.payload };
    default:
      return state;
  }
}

// Despacho de acción
const actions$ = new Subject<Action>();

// Actualizaciones de estado
actions$.pipe(
  scan(reducer, initialState)
).subscribe(state$);

// Ejemplo de uso
actions$.next({ type: 'SET_LOADING', payload: true });

// Monitorear parte del estado
const isLoading$ = state$.pipe(
  map(state => state.isLoading),
  distinctUntilChanged()
);

isLoading$.subscribe(isLoading => {
  console.log(`Estado de carga: ${isLoading}`);
});
```

## Ejemplos de uso de genéricos

Tipos genéricos más avanzados son útiles en flujos de datos complejos.

```ts
// Servicio que envuelve peticiones HTTP
class ApiService {
  // Método genérico
  get<T>(url: string): Observable<T> {
    return fromFetch(url).pipe(
      switchMap(response => {
        if (response.ok) {
          return response.json() as Promise<T>;
        } else {
          return throwError(() => new Error(`Error ${response.status}`));
        }
      }),
      retry(3),
      catchError((err: unknown) => this.handleError<T>(err))
    );
  }

  private handleError<T>(error: Error): Observable<T> {
    console.error('Error de API:', error);
    return EMPTY;
  }
}

// Ejemplo de uso
interface Product {
  id: string;
  name: string;
  price: number;
}

const apiService = new ApiService();
const products$ = apiService.get<Product[]>('/api/products');

products$.subscribe(products => {
  // products se tratan como tipo Product[]
  products.forEach(p => console.log(`${p.name}: ${p.price}円`));
});

// Al usar tipos genéricos de esta manera, los tipos de respuesta API pueden definirse en una forma reutilizable, haciendo las clases de servicio más genéricas.
```

## Resumen

La combinación de TypeScript y RxJS ofrece las siguientes ventajas:

- Programación asíncrona con seguridad de tipos
- Mayor eficiencia de desarrollo a través del soporte del IDE
- Detección temprana de errores mediante verificación de tipos en tiempo de compilación
- Código auto-documentado
- Seguridad en la refactorización

El desarrollo con RxJS puede mejorarse aún más utilizando definiciones de tipos adecuadas y las características avanzadas de TypeScript.
