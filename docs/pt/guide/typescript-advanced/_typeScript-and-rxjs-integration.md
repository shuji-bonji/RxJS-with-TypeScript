---
description: Design reativo robusto e com segurança de tipos através de definições de tipos de operadores personalizados, utilização de tipos condicionais e padrões de gerenciamento de estado na integração de TypeScript e RxJS.
---

# Integração de TypeScript e RxJS

Este documento apresenta várias técnicas e melhores práticas para usar RxJS com segurança de tipos em TypeScript.

## O que você aprenderá neste documento

- Como trabalhar com tipos Observable em TypeScript
- Como definir tipos de operadores personalizados
- Usando tipos condicionais e utilitários
- Gerenciamento de estado com segurança de tipos usando RxJS
- Configurações recomendadas para tsconfig.json e por quê

A combinação de TypeScript e RxJS permite programação assíncrona mantendo a segurança de tipos.
Este documento mostrará como utilizar efetivamente RxJS com TypeScript.

## Utilizando definições de tipos

### Especificando o tipo Observable

Uma das maiores vantagens de usar RxJS com TypeScript é a capacidade de definir explicitamente o tipo de valores que fluem em um Observable.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

// Definição explícita de tipo
const numbers$: Observable<number> = of(1, 2, 3);

// Transformação usando tipos genéricos
interface User {
  id: number;
  name: string;
}

const users$: Observable<User> = of(
  { id: 1, name: '山田' },
  { id: 2, name: '佐藤' }
);

// Operação onde os tipos são transformados
const userNames$: Observable<string> = users$.pipe(
  map(user => user.name)
);
```

### Definição de tipo de operador personalizado

Os tipos também podem ser tratados adequadamente ao criar operadores personalizados.

```ts
import { Observable, of, OperatorFunction } from 'rxjs';
import { map } from 'rxjs';

// Definir operadores personalizados com segurança de tipos usando OperatorFunction
function doubleMap<T, R, S>(
  first: (value: T, index: number) => R,
  second: (value: R, index: number) => S
): OperatorFunction<T, S> {
  return (source: Observable<T>) => source.pipe(map(first), map(second));
}

// Exemplo de uso
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

## Interfaces e aliases de tipos

Para designs complexos orientados a eventos, ter a estrutura do evento definida por tipo torna a integração com RxJS muito mais eficiente.

Ao lidar com estruturas de dados complexas, é útil definir interfaces e aliases de tipos.

```ts
// Definição de tipo de evento
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

// Composição de tipos de eventos
type ApplicationEvent = UserLoginEvent | DataUpdateEvent;

// Implementação do barramento de eventos
const eventBus$ = new Subject<ApplicationEvent>();

// Publicação de eventos com segurança de tipos
eventBus$.next({
  type: 'USER_LOGIN',
  payload: {
    userId: 'user123',
    timestamp: Date.now(),
  },
});

// Filtragem de eventos com segurança de tipos
const userLoginEvents$ = eventBus$.pipe(
  filter((event): event is UserLoginEvent => event.type === 'USER_LOGIN')
);

userLoginEvents$.subscribe((event) => {
  // Aqui event.payload.userId pode ser acessado com segurança de tipos
  console.log(`Usuário fez login: ${event.payload.userId}`);
});
```

## Utilizando tipos avançados

### Tipos utilitários

Você pode aprimorar ainda mais sua integração com RxJS aproveitando os tipos utilitários do TypeScript.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
  role: 'admin' | 'user';
}

// Selecionar um subconjunto de propriedades usando Pick
type UserBasicInfo = Pick<User, 'id' | 'name'>;

const users$: Observable<User> = fetchUsers();
const usersBasicInfo$: Observable<UserBasicInfo> = users$.pipe(
  map(user => ({ id: user.id, name: user.name }))
);

// Usar Omit para excluir propriedades específicas
type UserPublicInfo = Omit<User, 'email'>;

// Usar Partial para tornar todas as propriedades opcionais
type UserUpdate = Partial<User>;

function updateUser(id: number, update: UserUpdate): Observable<User> {
  return patchUser(id, update);
}
```

### Tipos Condicionais e de Mapeamento

Para casos mais complexos, tipos condicionais e de mapeamento podem ser usados.

```ts
import { filter, map, Observable} from 'rxjs';

// Tipos de resposta da API
type ApiResponse<T> =
  | { status: 'success'; data: T; }
  | { status: 'error'; error: string; };

// Tipo que extrai apenas dados da resposta
type ExtractData<T> = T extends ApiResponse<infer U> ? U : never;

function handleApiResponse<T>(response$: Observable<ApiResponse<T>>): Observable<T> {
  return response$.pipe(
    filter((response): response is ApiResponse<T> & { status: 'success' } =>
      response.status === 'success'
    ),
    map(response => response.data)
  );
}

// Exemplo de uso
const userResponse$: Observable<ApiResponse<User>> = fetchUserApi(1);
const user$: Observable<User> = handleApiResponse(userResponse$);
```

## Otimizar tsconfig.json

A configuração adequada do tsconfig.json é importante para o uso eficaz de RxJS e TypeScript.

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

As seguintes configurações são de particular importância

💡 Em particular, `"strict": true` é essencial para aproveitar ao máximo o RxJS.

- `strict`: verificação rigorosa de tipos para aproveitar totalmente a segurança de tipos do RxJS
- `noImplicitAny`: não permitir tipos any implícitos.
- `strictNullChecks`: null/undefined devem ser tratados explicitamente.

## Otimização de importações RxJS

Ao usar RxJS em um projeto TypeScript, o método de importação também é importante.

```ts
// Método recomendado
import { Observable, of, from } from 'rxjs';
import { map, filter, catchError } from 'rxjs';
```

## Padrão RxJS para gerenciamento de estado (configuração sem Redux)

Esta seção mostra como construir gerenciamento de estado usando apenas RxJS sem usar Redux ou NgRx.

```ts
// Interface de estado da aplicação
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

// BehaviorSubject para gerenciamento de estado
const state$ = new BehaviorSubject<AppState>(initialState);

// Tipos de ação
type Action =
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_USER'; payload: User | null }
  | { type: 'SET_DATA'; payload: Record<string, unknown> }
  | { type: 'SET_ERROR'; payload: Error | null };

// Função de atualização de estado
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

// Despacho de ação
const actions$ = new Subject<Action>();

// Atualizações de estado
actions$.pipe(
  scan(reducer, initialState)
).subscribe(state$);

// Exemplo de uso
actions$.next({ type: 'SET_LOADING', payload: true });

// Monitorar parte do estado
const isLoading$ = state$.pipe(
  map(state => state.isLoading),
  distinctUntilChanged()
);

isLoading$.subscribe(isLoading => {
  console.log(`Estado de Loading: ${isLoading}`);
});
```

## Exemplos de uso de genéricos

Tipos genéricos mais avançados são úteis em fluxos de dados complexos.

```ts
// Serviço que encapsula requisições HTTP
class ApiService {
  // Método genérico
  get<T>(url: string): Observable<T> {
    return fromFetch(url).pipe(
      switchMap(response => {
        if (response.ok) {
          return response.json() as Promise<T>;
        } else {
          return throwError(() => new Error(`Erro ${response.status}`));
        }
      }),
      retry(3),
      catchError(err => this.handleError<T>(err))
    );
  }

  private handleError<T>(error: Error): Observable<T> {
    console.error('Erro da API:', error);
    return EMPTY;
  }
}

// Exemplo de uso
interface Product {
  id: string;
  name: string;
  price: number;
}

const apiService = new ApiService();
const products$ = apiService.get<Product[]>('/api/products');

products$.subscribe(products => {
  // products são tratados como tipo Product[]
  products.forEach(p => console.log(`${p.name}: ${p.price} reais`));
});

// Ao usar tipos genéricos dessa forma, os tipos de resposta da API podem ser definidos de forma reutilizável, tornando as classes de serviço mais genéricas.
```

## Resumo

A combinação de TypeScript e RxJS oferece as seguintes vantagens

- Programação assíncrona com segurança de tipos
- Aumento da eficiência de desenvolvimento através do suporte de IDE
- Detecção precoce de erros através da verificação de tipos em tempo de compilação
- Código auto-documentado
- Segurança de refatoração

O desenvolvimento com RxJS pode ser ainda mais aprimorado utilizando definições de tipos adequadas e os recursos avançados do TypeScript.
