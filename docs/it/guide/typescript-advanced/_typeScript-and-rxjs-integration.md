---
description: "Definizione di tipi per operatori personalizzati, utilizzo di tipi condizionali, pattern di gestione dello stato nell'integrazione TypeScript e RxJS. Tecniche avanzate per realizzare un design reattivo robusto e type-safe utilizzando generics, mapped types e conditional types."
---

# Integrazione di TypeScript e RxJS

Questo documento presenta varie tecniche e best practice per utilizzare RxJS in modo type-safe con TypeScript.

## Cosa imparerai in questo documento

- Come gestire i tipi Observable in TypeScript
- Come definire i tipi per operatori personalizzati
- Utilizzo di tipi condizionali e utility types
- Gestione dello stato type-safe utilizzando RxJS
- Configurazione raccomandata di tsconfig.json e motivazioni

Combinando TypeScript e RxJS, è possibile eseguire la programmazione asincrona mantenendo la type safety.
Questo documento introduce metodi per utilizzare efficacemente RxJS con TypeScript.

## Utilizzo delle definizioni di tipo

### Specifica del tipo Observable

Il vantaggio principale dell'utilizzo di RxJS con TypeScript è la possibilità di definire esplicitamente il tipo dei valori che fluiscono attraverso gli Observable.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

// Definizione esplicita del tipo
const numbers$: Observable<number> = of(1, 2, 3);

// Trasformazione utilizzando tipi generici
interface User {
  id: number;
  name: string;
}

const users$: Observable<User> = of(
  { id: 1, name: '山田' },
  { id: 2, name: '佐藤' }
);

// Operazione che trasforma il tipo
const userNames$: Observable<string> = users$.pipe(
  map(user => user.name)
);
```

### Definizione di tipi per operatori personalizzati

È possibile gestire correttamente i tipi anche durante la creazione di operatori personalizzati.

```ts
import { Observable, of, OperatorFunction } from 'rxjs';
import { map } from 'rxjs';

// Definizione di un operatore personalizzato type-safe utilizzando OperatorFunction
function doubleMap<T, R, S>(
  first: (value: T, index: number) => R,
  second: (value: R, index: number) => S
): OperatorFunction<T, S> {
  return (source: Observable<T>) => source.pipe(map(first), map(second));
}

// Esempio di utilizzo
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

## Interface e type alias

Quando si gestiscono strutture dati complesse, è utile definire interface e type alias.

Nei progetti event-driven complessi, definendo la struttura degli eventi tramite tipi, l'integrazione con RxJS diventa molto efficiente.

```ts
// Definizione del tipo evento
interface AppEvent {
  type: string;
  payload: unknown;
}

// Tipo di evento specifico
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

// Composizione dei tipi di evento
type ApplicationEvent = UserLoginEvent | DataUpdateEvent;

// Implementazione dell'event bus
const eventBus$ = new Subject<ApplicationEvent>();

// Emissione di eventi type-safe
eventBus$.next({
  type: 'USER_LOGIN',
  payload: {
    userId: 'user123',
    timestamp: Date.now(),
  },
});

// Filtraggio di eventi type-safe
const userLoginEvents$ = eventBus$.pipe(
  filter((event): event is UserLoginEvent => event.type === 'USER_LOGIN')
);

userLoginEvents$.subscribe((event) => {
  // Qui event.payload.userId è accessibile in modo type-safe
  console.log(`User logged in: ${event.payload.userId}`);
});
```

## Utilizzo avanzato dei tipi

### Utility types

Utilizzando gli utility types di TypeScript, è possibile rafforzare ulteriormente l'integrazione con RxJS.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
  role: 'admin' | 'user';
}

// Utilizza Pick per selezionare un sottoinsieme di proprietà
type UserBasicInfo = Pick<User, 'id' | 'name'>;

const users$: Observable<User> = fetchUsers();
const usersBasicInfo$: Observable<UserBasicInfo> = users$.pipe(
  map(user => ({ id: user.id, name: user.name }))
);

// Utilizza Omit per escludere proprietà specifiche
type UserPublicInfo = Omit<User, 'email'>;

// Utilizza Partial per rendere tutte le proprietà opzionali
type UserUpdate = Partial<User>;

function updateUser(id: number, update: UserUpdate): Observable<User> {
  return patchUser(id, update);
}
```

### Tipi condizionali e mapped types

Nei casi più complessi, è possibile utilizzare tipi condizionali e mapped types.

```ts
import { filter, map, Observable} from 'rxjs';

// Tipo di risposta API
type ApiResponse<T> =
  | { status: 'success'; data: T; }
  | { status: 'error'; error: string; };

// Tipo che estrae solo i dati dalla risposta
type ExtractData<T> = T extends ApiResponse<infer U> ? U : never;

function handleApiResponse<T>(response$: Observable<ApiResponse<T>>): Observable<T> {
  return response$.pipe(
    filter((response): response is ApiResponse<T> & { status: 'success' } =>
      response.status === 'success'
    ),
    map(response => response.data)
  );
}

// Esempio di utilizzo
const userResponse$: Observable<ApiResponse<User>> = fetchUserApi(1);
const user$: Observable<User> = handleApiResponse(userResponse$);
```

## Ottimizzazione di tsconfig.json

Per utilizzare efficacemente RxJS e TypeScript, è importante una corretta configurazione di tsconfig.json.

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

Le seguenti impostazioni sono particolarmente importanti.

💡 In particolare, `"strict": true` è un'impostazione essenziale per sfruttare al massimo i vantaggi di RxJS.

- `strict`: Rende il controllo dei tipi rigoroso e sfrutta al massimo la type safety di RxJS
- `noImplicitAny`: Vieta i tipi any impliciti
- `strictNullChecks`: Richiede la gestione esplicita di null/undefined

## Ottimizzazione degli import di RxJS

Quando si utilizza RxJS in progetti TypeScript, è importante anche il metodo di import.

```ts
// Metodo raccomandato
import { Observable, of, from } from 'rxjs';
import { map, filter, catchError } from 'rxjs';
```

## Pattern RxJS per la gestione dello stato (configurazione senza Redux)

Questa sezione introduce come costruire la gestione dello stato utilizzando solo RxJS, senza usare Redux o NgRx.

```ts
// Interface dello stato dell'applicazione
interface AppState {
  user: User | null;
  isLoading: boolean;
  data: Record<string, unknown>;
  error: Error | null;
}

// Stato iniziale
const initialState: AppState = {
  user: null,
  isLoading: false,
  data: {},
  error: null
};

// BehaviorSubject per la gestione dello stato
const state$ = new BehaviorSubject<AppState>(initialState);

// Tipi di azione
type Action =
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_USER'; payload: User | null }
  | { type: 'SET_DATA'; payload: Record<string, unknown> }
  | { type: 'SET_ERROR'; payload: Error | null };

// Funzione di aggiornamento dello stato
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

// Dispatch delle azioni
const actions$ = new Subject<Action>();

// Aggiornamento dello stato
actions$.pipe(
  scan(reducer, initialState)
).subscribe(state$);

// Esempio di utilizzo
actions$.next({ type: 'SET_LOADING', payload: true });

// Monitoraggio di una parte dello stato
const isLoading$ = state$.pipe(
  map(state => state.isLoading),
  distinctUntilChanged()
);

isLoading$.subscribe(isLoading => {
  console.log(`Stato di caricamento: ${isLoading}`);
});
```

## Esempi di utilizzo dei generics

Per flussi di dati complessi, sono utili tipi generici più avanzati.

```ts
// Servizio che wrappa le richieste HTTP
class ApiService {
  // Metodo generico
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

// Esempio di utilizzo
interface Product {
  id: string;
  name: string;
  price: number;
}

const apiService = new ApiService();
const products$ = apiService.get<Product[]>('/api/products');

products$.subscribe(products => {
  // products viene trattato come tipo Product[]
  products.forEach(p => console.log(`${p.name}: ${p.price}円`));
});

// Utilizzando tipi generici in questo modo, è possibile definire i tipi di risposta API in modo riutilizzabile e generalizzare le classi di servizio.
```

## Riepilogo

Combinando TypeScript e RxJS, si ottengono i seguenti vantaggi:

- Programmazione asincrona type-safe
- Miglioramento dell'efficienza di sviluppo grazie al supporto dell'IDE
- Rilevamento precoce di errori tramite controllo dei tipi in fase di compilazione
- Codice auto-documentante
- Sicurezza nel refactoring

Utilizzando definizioni di tipi appropriate e le funzionalità avanzate di TypeScript, è possibile rafforzare ulteriormente lo sviluppo con RxJS.
