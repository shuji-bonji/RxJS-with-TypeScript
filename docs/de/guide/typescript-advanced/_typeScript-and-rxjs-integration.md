---
description: "Typdefinitionen für benutzerdefinierte Operatoren, Conditional Types und Zustandsverwaltungsmuster in der TypeScript-RxJS-Integration."
---

# TypeScript und RxJS Integration

Dieses Dokument stellt verschiedene Techniken und Best Practices vor, um RxJS mit TypeScript typsicher zu verwenden.

## Was Sie in diesem Dokument lernen können

- Wie man Observable-Typen in TypeScript handhabt
- Methoden zur Typdefinition benutzerdefinierter Operatoren
- Nutzung von Conditional Types und Utility Types
- Typsichere Zustandsverwaltung mit RxJS
- Empfohlene tsconfig.json-Einstellungen und deren Gründe

Durch die Kombination von TypeScript und RxJS können Sie asynchrone Programmierung unter Beibehaltung der Typsicherheit durchführen.
Dieses Dokument stellt Methoden vor, um RxJS effektiv mit TypeScript zu nutzen.

## Nutzung von Typdefinitionen

### Angabe von Observable-Typen

Der größte Vorteil der Verwendung von RxJS mit TypeScript besteht darin, dass Sie den Typ der durch ein Observable fließenden Werte explizit definieren können.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

// Explizite Typdefinition
const numbers$: Observable<number> = of(1, 2, 3);

// Transformation mit generischen Typen
interface User {
  id: number;
  name: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Schmidt' },
  { id: 2, name: 'Müller' }
);

// Operation, bei der der Typ transformiert wird
const userNames$: Observable<string> = users$.pipe(
  map(user => user.name)
);
```

### Typdefinition für benutzerdefinierte Operatoren

Auch beim Erstellen benutzerdefinierter Operatoren können Sie Typen angemessen handhaben.

```ts
import { Observable, of, OperatorFunction } from 'rxjs';
import { map } from 'rxjs';

// Definition eines typsicheren benutzerdefinierten Operators mit OperatorFunction
function doubleMap<T, R, S>(
  first: (value: T, index: number) => R,
  second: (value: R, index: number) => S
): OperatorFunction<T, S> {
  return (source: Observable<T>) => source.pipe(map(first), map(second));
}

// Verwendungsbeispiel
of(1, 2, 3)
  .pipe(
    doubleMap(
      (x) => x * 2,
      (x) => `Result: ${x}`
    )
  )
  .subscribe(console.log);
// Result: 2
// Result: 4
// Result: 6
```

## Interfaces und Type-Aliase

Bei komplexen ereignisgesteuerten Designs wird die Integration mit RxJS sehr effizient, wenn Sie die Ereignisstruktur als Typdefinition vordefinieren.

Beim Umgang mit komplexen Datenstrukturen ist es praktisch, Interfaces oder Type-Aliase zu definieren.

```ts
// Typdefinition für Events
interface AppEvent {
  type: string;
  payload: unknown;
}

// Spezifischer Event-Typ
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

// Event-Typ-Komposition
type ApplicationEvent = UserLoginEvent | DataUpdateEvent;

// Event-Bus-Implementierung
const eventBus$ = new Subject<ApplicationEvent>();

// Typsicheres Event-Publishing
eventBus$.next({
  type: 'USER_LOGIN',
  payload: {
    userId: 'user123',
    timestamp: Date.now(),
  },
});

// Typsichere Event-Filterung
const userLoginEvents$ = eventBus$.pipe(
  filter((event): event is UserLoginEvent => event.type === 'USER_LOGIN')
);

userLoginEvents$.subscribe((event) => {
  // Hier ist event.payload.userId typsicher zugänglich
  console.log(`User logged in: ${event.payload.userId}`);
});
```

## Nutzung fortgeschrittener Typen

### Utility Types

Durch die Nutzung von TypeScripts Utility Types können Sie die Integration mit RxJS weiter verstärken.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
  role: 'admin' | 'user';
}

// Verwendung von Pick zur Auswahl einer Teilmenge von Properties
type UserBasicInfo = Pick<User, 'id' | 'name'>;

const users$: Observable<User> = fetchUsers();
const usersBasicInfo$: Observable<UserBasicInfo> = users$.pipe(
  map(user => ({ id: user.id, name: user.name }))
);

// Verwendung von Omit zum Ausschluss bestimmter Properties
type UserPublicInfo = Omit<User, 'email'>;

// Verwendung von Partial, um alle Properties optional zu machen
type UserUpdate = Partial<User>;

function updateUser(id: number, update: UserUpdate): Observable<User> {
  return patchUser(id, update);
}
```

### Conditional Types und Mapped Types

In komplexeren Fällen können Sie Conditional Types und Mapped Types verwenden.

```ts
import { filter, map, Observable} from 'rxjs';

// API-Response-Typ
type ApiResponse<T> =
  | { status: 'success'; data: T; }
  | { status: 'error'; error: string; };

// Typ, der nur die Daten aus der Response extrahiert
type ExtractData<T> = T extends ApiResponse<infer U> ? U : never;

function handleApiResponse<T>(response$: Observable<ApiResponse<T>>): Observable<T> {
  return response$.pipe(
    filter((response): response is ApiResponse<T> & { status: 'success' } =>
      response.status === 'success'
    ),
    map(response => response.data)
  );
}

// Verwendungsbeispiel
const userResponse$: Observable<ApiResponse<User>> = fetchUserApi(1);
const user$: Observable<User> = handleApiResponse(userResponse$);
```

## Optimierung der tsconfig.json

Für die effektive Nutzung von RxJS und TypeScript ist die richtige tsconfig.json-Konfiguration wichtig.

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

Folgende Einstellungen sind besonders wichtig.

💡 Insbesondere ist `"strict": true` eine unverzichtbare Einstellung, um die Vorteile von RxJS maximal auszuschöpfen.

- `strict`: Macht die Typprüfung streng und nutzt die Typsicherheit von RxJS maximal aus
- `noImplicitAny`: Verbietet implizite any-Typen
- `strictNullChecks`: Erfordert explizite Handhabung von null/undefined

## Optimierung der RxJS-Imports

Bei der Verwendung von RxJS in TypeScript-Projekten ist auch die Art des Imports wichtig.

```ts
// Empfohlene Methode
import { Observable, of, from } from 'rxjs';
import { map, filter, catchError } from 'rxjs';
```

## RxJS-Muster für Zustandsverwaltung (Redux-lose Konfiguration)

Dieser Abschnitt stellt vor, wie Sie Zustandsverwaltung nur mit RxJS aufbauen, ohne Redux oder NgRx zu verwenden.

```ts
// Application State Interface
interface AppState {
  user: User | null;
  isLoading: boolean;
  data: Record<string, unknown>;
  error: Error | null;
}

// Initialzustand
const initialState: AppState = {
  user: null,
  isLoading: false,
  data: {},
  error: null
};

// BehaviorSubject für Zustandsverwaltung
const state$ = new BehaviorSubject<AppState>(initialState);

// Action-Typ
type Action =
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_USER'; payload: User | null }
  | { type: 'SET_DATA'; payload: Record<string, unknown> }
  | { type: 'SET_ERROR'; payload: Error | null };

// Zustandsaktualisierungsfunktion
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

// Action-Dispatch
const actions$ = new Subject<Action>();

// Zustandsaktualisierung
actions$.pipe(
  scan(reducer, initialState)
).subscribe(state$);

// Verwendungsbeispiel
actions$.next({ type: 'SET_LOADING', payload: true });

// Überwachung eines Teils des Zustands
const isLoading$ = state$.pipe(
  map(state => state.isLoading),
  distinctUntilChanged()
);

isLoading$.subscribe(isLoading => {
  console.log(`Loading-Zustand: ${isLoading}`);
});
```

## Beispiele für die Nutzung von Generics

Bei komplexen Datenflüssen sind fortgeschrittenere generische Typen hilfreich.

```ts
// Service, der HTTP-Requests umhüllt
class ApiService {
  // Generische Methode
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
      catchError(err => this.handleError<T>(err))
    );
  }

  private handleError<T>(error: Error): Observable<T> {
    console.error('API error:', error);
    return EMPTY;
  }
}

// Verwendungsbeispiel
interface Product {
  id: string;
  name: string;
  price: number;
}

const apiService = new ApiService();
const products$ = apiService.get<Product[]>('/api/products');

products$.subscribe(products => {
  // products wird als Product[] Typ behandelt
  products.forEach(p => console.log(`${p.name}: €${p.price}`));
});

// Durch die Verwendung generischer Typen auf diese Weise können Sie API-Response-Typen in wiederverwendbarer Form definieren und die Service-Klasse generalisieren.
```

## Zusammenfassung

Die Kombination von TypeScript und RxJS bietet folgende Vorteile:

- Typsichere asynchrone Programmierung
- Verbesserung der Entwicklungseffizienz durch IDE-Unterstützung
- Früherkennung von Fehlern durch Typprüfung zur Compile-Zeit
- Selbstdokumentierender Code
- Sicherheit beim Refactoring

Durch die Nutzung geeigneter Typdefinitionen und fortgeschrittener TypeScript-Funktionen können Sie die Entwicklung mit RxJS weiter verstärken.
