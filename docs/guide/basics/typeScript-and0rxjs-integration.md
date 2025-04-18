# TypeScriptとRxJSの統合

TypeScriptとRxJSを組み合わせることで、型安全性を保ちながら非同期プログラミングを行うことができます。  
この文書では、TypeScriptでRxJSを効果的に活用するための方法を紹介します。

## 型定義の活用

### Observable型の指定

TypeScriptでRxJSを使用する最大の利点は、Observableに流れる値の型を明示的に定義できることです。

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs/operators';

// 明示的な型定義
const numbers$: Observable<number> = of(1, 2, 3);

// ジェネリック型を使用した変換
interface User {
  id: number;
  name: string;
}

const users$: Observable<User> = of(
  { id: 1, name: '山田' },
  { id: 2, name: '佐藤' }
);

// 型が変換されるオペレーション
const userNames$: Observable<string> = users$.pipe(
  map(user => user.name)
);
```

### カスタムオペレーターの型定義

カスタムオペレーターを作成する際も、型を適切に扱うことができます。

```ts
import { Observable, OperatorFunction } from 'rxjs';
import { map } from 'rxjs/operators';

// OperatorFunctionを使用して型安全なカスタムオペレーターを定義
function doubleMap<T, R>(project: (value: T, index: number) => R): OperatorFunction<T, R> {
  return (source: Observable<T>) => 
    source.pipe(
      map((value, index) => project(value, index)),
      map((value, index) => project(value, index))
    );
}

// 使用例
of(1, 2, 3).pipe(
  doubleMap(x => x * 2)
).subscribe(console.log); // 出力: 4, 8, 12
```

## インターフェースと型エイリアス

複雑なデータ構造を扱う場合、インターフェースや型エイリアスを定義すると便利です。

```ts
// イベントの型定義
interface AppEvent {
  type: string;
  payload: unknown;
}

// 特定のイベント型
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

// イベントの型合成
type ApplicationEvent = UserLoginEvent | DataUpdateEvent;

// イベントバスの実装
const eventBus$ = new Subject<ApplicationEvent>();

// 型安全なイベント発行
eventBus$.next({
  type: 'USER_LOGIN',
  payload: {
    userId: 'user123',
    timestamp: Date.now()
  }
});

// 型安全なイベントフィルタリング
const userLoginEvents$ = eventBus$.pipe(
  filter((event): event is UserLoginEvent => event.type === 'USER_LOGIN')
);

userLoginEvents$.subscribe(event => {
  // ここでは event.payload.userId が型安全にアクセス可能
  console.log(`User logged in: ${event.payload.userId}`);
});
```

## 高度な型の活用

### ユーティリティ型

TypeScriptのユーティリティ型を活用することで、RxJSとの統合をさらに強化できます。

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs/operators';

interface User {
  id: number;
  name: string;
  email: string;
  role: 'admin' | 'user';
}

// Pick を使ってプロパティのサブセットを選択
type UserBasicInfo = Pick<User, 'id' | 'name'>;

const users$: Observable<User> = fetchUsers();
const usersBasicInfo$: Observable<UserBasicInfo> = users$.pipe(
  map(user => ({ id: user.id, name: user.name }))
);

// Omit を使って特定のプロパティを除外
type UserPublicInfo = Omit<User, 'email'>;

// Partial を使って全てのプロパティをオプショナルにする
type UserUpdate = Partial<User>;

function updateUser(id: number, update: UserUpdate): Observable<User> {
  return patchUser(id, update);
}
```

### 条件型とマッピング型

より複雑なケースでは、条件型やマッピング型を使用できます。

```ts
// APIレスポンスの型
type ApiResponse<T> = 
  | { status: 'success'; data: T; }
  | { status: 'error'; error: string; };

// レスポンスからデータだけを抽出する型
type ExtractData<T> = T extends ApiResponse<infer U> ? U : never;

function handleApiResponse<T>(response$: Observable<ApiResponse<T>>): Observable<T> {
  return response$.pipe(
    filter((response): response is ApiResponse<T> & { status: 'success' } => 
      response.status === 'success'
    ),
    map(response => response.data)
  );
}

// 使用例
const userResponse$: Observable<ApiResponse<User>> = fetchUserApi(1);
const user$: Observable<User> = handleApiResponse(userResponse$);
```

## tsconfig.jsonの最適化

RxJSとTypeScriptを効果的に使用するには、適切なtsconfig.jsonの設定が重要です。

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

以下の設定が特に重要です。

- `strict`: 型チェックを厳格にし、RxJSの型安全性を最大限に活用します
- `noImplicitAny`: 暗黙のany型を禁止します
- `strictNullChecks`: null/undefinedを明示的に扱う必要があります

## RxJSのimportの最適化

RxJSをTypeScriptプロジェクトで使用する際、importの方法も重要です。

```ts
// 推奨される方法
import { Observable, of, from } from 'rxjs';
import { map, filter, catchError } from 'rxjs/operators';
```

## ジェネリックス活用例

複雑なデータフローでは、より高度なジェネリック型が役立ちます。

```ts
// HTTPリクエストをラップするサービス
class ApiService {
  // ジェネリックメソッド
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

// 使用例
interface Product {
  id: string;
  name: string;
  price: number;
}

const apiService = new ApiService();
const products$ = apiService.get<Product[]>('/api/products');

products$.subscribe(products => {
  // products は Product[] 型として扱われる
  products.forEach(p => console.log(`${p.name}: ${p.price}円`));
});
```

## 状態管理とTypeScript

RxJSを使った状態管理はTypeScriptと相性が良いです。

```ts
// アプリケーションの状態インターフェース
interface AppState {
  user: User | null;
  isLoading: boolean;
  data: Record<string, unknown>;
  error: Error | null;
}

// 初期状態
const initialState: AppState = {
  user: null,
  isLoading: false,
  data: {},
  error: null
};

// 状態管理用のBehaviorSubject
const state$ = new BehaviorSubject<AppState>(initialState);

// アクション型
type Action = 
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_USER'; payload: User | null }
  | { type: 'SET_DATA'; payload: Record<string, unknown> }
  | { type: 'SET_ERROR'; payload: Error | null };

// 状態更新関数
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

// アクションディスパッチ
const actions$ = new Subject<Action>();

// 状態の更新
actions$.pipe(
  scan(reducer, initialState)
).subscribe(state$);

// 使用例
actions$.next({ type: 'SET_LOADING', payload: true });

// 状態の一部を監視
const isLoading$ = state$.pipe(
  map(state => state.isLoading),
  distinctUntilChanged()
);

isLoading$.subscribe(isLoading => {
  console.log(`Loading状態: ${isLoading}`);
});
```

## まとめ

TypeScriptとRxJSを組み合わせることで、次のような利点があります：

- 型安全な非同期プログラミング
- IDEの支援による開発効率の向上
- コンパイル時の型チェックによるエラーの早期発見
- 自己文書化されたコード
- リファクタリングの安全性

適切な型定義とTypeScriptの高度な機能を活用することで、RxJSを使った開発をさらに強化できます。