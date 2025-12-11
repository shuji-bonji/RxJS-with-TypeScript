---
description: Robuust, type-veilig reactief ontwerp door aangepaste operator typedefinities, gebruik van conditionele types en state management patronen in TypeScript en RxJS integratie.
---

# TypeScript en RxJS Integratie

Dit document presenteert verschillende technieken en best practices voor het type-veilig gebruiken van RxJS in TypeScript.

## Wat u in dit document leert

- Hoe te werken met Observable types in TypeScript
- Hoe aangepaste operator types te definiëren
- Gebruik van Conditionele en Utility types
- Type-veilig state management met RxJS
- Aanbevolen instellingen voor tsconfig.json en waarom

De combinatie van TypeScript en RxJS maakt asynchrone programmering mogelijk terwijl type-veiligheid behouden blijft.
Dit document laat zien hoe u RxJS effectief kunt gebruiken met TypeScript.

## Gebruik van typedefinities

### Het Observable type specificeren

Een van de grootste voordelen van het gebruik van RxJS met TypeScript is de mogelijkheid om expliciet het type van waarden te definiëren die in een Observable stromen.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

// Expliciete typedefinitie
const numbers$: Observable<number> = of(1, 2, 3);

// Transformatie met generieke types
interface User {
  id: number;
  name: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Jansen' },
  { id: 2, name: 'de Vries' }
);

// Bewerking waarbij types worden getransformeerd
const userNames$: Observable<string> = users$.pipe(
  map(user => user.name)
);
```

### Typedefinitie voor aangepaste operators

Types kunnen ook correct worden afgehandeld bij het maken van aangepaste operators.

```ts
import { Observable, of, OperatorFunction } from 'rxjs';
import { map } from 'rxjs';

// Definieer type-veilige aangepaste operators met OperatorFunction
function doubleMap<T, R, S>(
  first: (value: T, index: number) => R,
  second: (value: R, index: number) => S
): OperatorFunction<T, S> {
  return (source: Observable<T>) => source.pipe(map(first), map(second));
}

// Gebruiksvoorbeeld
of(1, 2, 3)
  .pipe(
    doubleMap(
      (x) => x * 2,
      (x) => `Resultaat: ${x}`
    )
  )
  .subscribe(console.log);
// Resultaat: 2
// Resultaat: 4
// Resultaat: 6
```

## Interfaces en type-aliassen

Voor complexe event-driven ontwerpen maakt het hebben van de gebeurtenisstructuur met typedefinities de integratie met RxJS veel efficiënter.

Bij het werken met complexe datastructuren is het nuttig om interfaces en type-aliassen te definiëren.

```ts
// Gebeurtenistype definitie
interface AppEvent {
  type: string;
  payload: unknown;
}

// Specifieke gebeurtenistypes
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

// Samenstelling van gebeurtenistypes
type ApplicationEvent = UserLoginEvent | DataUpdateEvent;

// Event bus implementatie
const eventBus$ = new Subject<ApplicationEvent>();

// Type-veilig gebeurtenissen publiceren
eventBus$.next({
  type: 'USER_LOGIN',
  payload: {
    userId: 'user123',
    timestamp: Date.now(),
  },
});

// Type-veilig gebeurtenissen filteren
const userLoginEvents$ = eventBus$.pipe(
  filter((event): event is UserLoginEvent => event.type === 'USER_LOGIN')
);

userLoginEvents$.subscribe((event) => {
  // Hier kan event.payload.userId type-veilig worden benaderd
  console.log(`Gebruiker ingelogd: ${event.payload.userId}`);
});
```

## Gebruik van geavanceerde types

### Utility types

U kunt uw integratie met RxJS verder verbeteren door gebruik te maken van TypeScript's utility types.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
  role: 'admin' | 'user';
}

// Selecteer een subset van eigenschappen met Pick
type UserBasicInfo = Pick<User, 'id' | 'name'>;

const users$: Observable<User> = fetchUsers();
const usersBasicInfo$: Observable<UserBasicInfo> = users$.pipe(
  map(user => ({ id: user.id, name: user.name }))
);

// Gebruik Omit om specifieke eigenschappen uit te sluiten
type UserPublicInfo = Omit<User, 'email'>;

// Gebruik Partial om alle eigenschappen optioneel te maken
type UserUpdate = Partial<User>;

function updateUser(id: number, update: UserUpdate): Observable<User> {
  return patchUser(id, update);
}
```

### Conditionele en Mapping Types

Voor complexere gevallen kunnen conditionele en mapping types worden gebruikt.

```ts
import { filter, map, Observable} from 'rxjs';

// API response types
type ApiResponse<T> =
  | { status: 'success'; data: T; }
  | { status: 'error'; error: string; };

// Type dat alleen data uit response haalt
type ExtractData<T> = T extends ApiResponse<infer U> ? U : never;

function handleApiResponse<T>(response$: Observable<ApiResponse<T>>): Observable<T> {
  return response$.pipe(
    filter((response): response is ApiResponse<T> & { status: 'success' } =>
      response.status === 'success'
    ),
    map(response => response.data)
  );
}

// Gebruiksvoorbeeld
const userResponse$: Observable<ApiResponse<User>> = fetchUserApi(1);
const user$: Observable<User> = handleApiResponse(userResponse$);
```

## Optimaliseer tsconfig.json

Correcte tsconfig.json configuratie is belangrijk voor effectief gebruik van RxJS en TypeScript.

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

De volgende instellingen zijn bijzonder belangrijk:

💡 Met name `"strict": true` is essentieel om het meeste uit RxJS te halen.

- `strict`: strikte typecontrole om volledig te profiteren van RxJS type-veiligheid
- `noImplicitAny`: impliciete any types niet toestaan
- `strictNullChecks`: null/undefined moet expliciet worden afgehandeld

## Optimalisatie van RxJS imports

Bij het gebruik van RxJS in een TypeScript project is de importmethode ook belangrijk.

```ts
// Aanbevolen methode
import { Observable, of, from } from 'rxjs';
import { map, filter, catchError } from 'rxjs';
```

## RxJS patroon voor state management (Redux-loze configuratie)

Dit gedeelte laat zien hoe u state management kunt bouwen met alleen RxJS zonder Redux of NgRx te gebruiken.

```ts
// Applicatiestatus interface
interface AppState {
  user: User | null;
  isLoading: boolean;
  data: Record<string, unknown>;
  error: Error | null;
}

// Initiële status
const initialState: AppState = {
  user: null,
  isLoading: false,
  data: {},
  error: null
};

// BehaviorSubject voor state management
const state$ = new BehaviorSubject<AppState>(initialState);

// Actietypes
type Action =
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_USER'; payload: User | null }
  | { type: 'SET_DATA'; payload: Record<string, unknown> }
  | { type: 'SET_ERROR'; payload: Error | null };

// Status update functie
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

// Actie verzending
const actions$ = new Subject<Action>();

// Status updates
actions$.pipe(
  scan(reducer, initialState)
).subscribe(state$);

// Gebruiksvoorbeeld
actions$.next({ type: 'SET_LOADING', payload: true });

// Monitor een deel van de status
const isLoading$ = state$.pipe(
  map(state => state.isLoading),
  distinctUntilChanged()
);

isLoading$.subscribe(isLoading => {
  console.log(`Laadstatus: ${isLoading}`);
});
```

## Voorbeelden van generiek gebruik

Meer geavanceerde generieke types zijn nuttig in complexe datastromen.

```ts
// Service die HTTP-verzoeken omhult
class ApiService {
  // Generieke methode
  get<T>(url: string): Observable<T> {
    return fromFetch(url).pipe(
      switchMap(response => {
        if (response.ok) {
          return response.json() as Promise<T>;
        } else {
          return throwError(() => new Error(`Fout ${response.status}`));
        }
      }),
      retry(3),
      catchError(err => this.handleError<T>(err))
    );
  }

  private handleError<T>(error: Error): Observable<T> {
    console.error('API fout:', error);
    return EMPTY;
  }
}

// Gebruiksvoorbeeld
interface Product {
  id: string;
  name: string;
  price: number;
}

const apiService = new ApiService();
const products$ = apiService.get<Product[]>('/api/products');

products$.subscribe(products => {
  // products worden behandeld als type Product[]
  products.forEach(p => console.log(`${p.name}: €${p.price}`));
});

// Door op deze manier generieke types te gebruiken, kunnen API response types worden gedefinieerd in een herbruikbare vorm, waardoor serviceklassen meer generiek worden.
```

## Samenvatting

De combinatie van TypeScript en RxJS biedt de volgende voordelen:

- Type-veilige asynchrone programmering
- Verhoogde ontwikkelingsefficiëntie door IDE-ondersteuning
- Vroege detectie van fouten door compile-time typecontrole
- Zelfdocumenterende code
- Veiligheid bij refactoring

Ontwikkeling met RxJS kan verder worden verbeterd door gebruik te maken van correcte typedefinities en de geavanceerde functies van TypeScript.
