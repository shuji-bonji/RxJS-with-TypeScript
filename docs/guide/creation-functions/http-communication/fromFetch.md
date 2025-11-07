---
description: RxJSのfromFetch()関数を使って、Fetch APIベースのHTTP通信をObservableとして扱う方法を、実践的なコード例とともに詳しく解説します。
---

# fromFetch()

[📘 RxJS公式ドキュメント - fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` は、モダンな Fetch API をベースにした HTTP通信を Observable として扱うための Creation Function です。`ajax()` と比較して軽量で、最新のWeb標準に準拠しています。

## 基本的な使い方

### シンプルなGETリクエスト

`fromFetch()` を使った最もシンプルな例は、URLを渡してレスポンスを手動でパースする方法です。

```typescript
import { of, switchMap, catchError } from 'rxjs';
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
      // レスポンスが成功の場合、JSONをパース
      return response.json();
    } else {
      // HTTPエラーの場合、エラーをスロー
      return throwError(() => new Error(`HTTP Error: ${response.status}`));
    }
  }),
  catchError(error => {
    console.error('エラー:', error);
    return of({ error: true, message: error.message });
  })
);

data$.subscribe({
  next: data => console.log('データ:', data),
  error: error => console.error('購読エラー:', error),
  complete: () => console.log('完了')
});

// 出力:
// データ: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// 完了
```

> [!IMPORTANT]
> **ajax() との重要な違い**
>
> - `fromFetch()` は、HTTPエラー（4xx, 5xx）でも `error` コールバックを呼び出しません
> - レスポンスの `ok` プロパティを手動でチェックする必要があります
> - `.json()` などのパース処理も手動で行います

## HTTPメソッド別の使い方

### GET リクエスト

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
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json() as Promise<User[]>;
  })
);

users$.subscribe({
  next: users => console.log('ユーザー一覧:', users),
  error: error => console.error('エラー:', error)
});
```

### POST リクエスト

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
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json() as Promise<CreateUserResponse>;
  })
);

createUser$.subscribe({
  next: user => console.log('作成成功:', user),
  error: error => console.error('作成失敗:', error)
});
```

### PUT リクエスト

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
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
);

updateUser$.subscribe({
  next: user => console.log('更新成功:', user),
  error: error => console.error('更新失敗:', error)
});
```

### DELETE リクエスト

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const deleteUser$ = fromFetch('https://api.example.com/users/1', {
  method: 'DELETE',
  headers: {
    'Authorization': 'Bearer token123'
  }
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    // DELETEは通常、空のレスポンスまたはステータスのみを返す
    return response.status === 204 ? of(null) : response.json();
  })
);

deleteUser$.subscribe({
  next: result => console.log('削除成功:', result),
  error: error => console.error('削除失敗:', error)
});
```

## 実践的なパターン

### 汎用的なHTTPエラーハンドリング関数

`fromFetch()` では手動でエラーチェックが必要なため、汎用関数を作成すると便利です。

```typescript
import { Observable, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

function fetchJSON<T>(url: string, options?: RequestInit): Observable<T> {
  return fromFetch(url, options).pipe(
    switchMap(response => {
      if (!response.ok) {
        throw new Error(`HTTP Error ${response.status}: ${response.statusText}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// 使用例
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

todo$.subscribe({
  next: todo => console.log('Todo:', todo),
  error: error => console.error('エラー:', error)
});
```

### HTTPステータスコードによる詳細な処理

```typescript
import { throwError, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/data').pipe(
  switchMap(response => {
    switch (response.status) {
      case 200:
        return response.json();
      case 204:
        // No Content - 空のレスポンス
        return of(null);
      case 401:
        throw new Error('認証が必要です');
      case 403:
        throw new Error('アクセスが拒否されました');
      case 404:
        throw new Error('リソースが見つかりません');
      case 500:
        throw new Error('サーバーエラーが発生しました');
      default:
        throw new Error(`予期しないHTTPステータス: ${response.status}`);
    }
  })
);

api$.subscribe({
  next: data => console.log('データ:', data),
  error: error => console.error('エラー:', error)
});
```

### タイムアウトとリトライ

```typescript
import { switchMap, timeout, retry } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow-endpoint').pipe(
  timeout(5000), // 5秒でタイムアウト
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  }),
  retry(2) // 失敗時に2回リトライ
);

api$.subscribe({
  next: data => console.log('データ:', data),
  error: error => console.error('エラー:', error)
});
```

### リクエストのキャンセル（AbortController）

`fromFetch()` は、Fetch APIの `AbortController` を使ったリクエストのキャンセルに対応しています。

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const controller = new AbortController();
const signal = controller.signal;

const api$ = fromFetch('https://api.example.com/data', {
  signal // AbortController の signal を渡す
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
);

const subscription = api$.subscribe({
  next: data => console.log('データ:', data),
  error: error => console.error('エラー:', error)
});

// 3秒後にリクエストをキャンセル
setTimeout(() => {
  controller.abort();
  // または subscription.unsubscribe();
}, 3000);
```

> [!TIP]
> **RxJSによる自動キャンセル**
>
> `unsubscribe()` を呼ぶだけで、RxJSが内部的に `AbortController` を使ってリクエストをキャンセルします。手動で `AbortController` を設定する必要はありません。

### ユーザー入力に応じた検索（switchMap）

```typescript
import { fromEvent, map, debounceTime, distinctUntilChanged, switchMap } from 'rxjs';
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
          throw new Error(`HTTP Error: ${response.status}`);
        }
        return response.json() as Promise<SearchResult[]>;
      })
    );
  })
);

search$.subscribe({
  next: results => console.log('検索結果:', results),
  error: error => console.error('検索エラー:', error)
});
```

### 複数のリクエストを並列実行

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
    console.log('Users:', users);
    console.log('Posts:', posts);
  },
  error: error => console.error('いずれかのリクエストが失敗:', error)
});
```

## よくある使用例

### 1. 認証トークン付きリクエスト

```typescript
import { switchMap } from 'rxjs';
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
        throw new Error('認証が必要です。再ログインしてください。');
      }
      if (!response.ok) {
        throw new Error(`HTTP Error: ${response.status}`);
      }
      return response.json() as Promise<T>;
    })
  );
}

// 使用例
interface UserProfile {
  id: number;
  name: string;
  email: string;
}

const profile$ = fetchWithAuth<UserProfile>('https://api.example.com/profile');

profile$.subscribe({
  next: profile => console.log('プロフィール:', profile),
  error: error => console.error('エラー:', error)
});
```

### 2. ファイルのダウンロード（Blob）

```typescript
import { switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const downloadFile$ = fromFetch('https://api.example.com/files/report.pdf').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    // Blobとして取得
    return response.blob();
  })
);

downloadFile$.subscribe({
  next: blob => {
    // Blobからダウンロードリンクを生成
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'report.pdf';
    a.click();
    window.URL.revokeObjectURL(url);
    console.log('ダウンロード完了');
  },
  error: error => console.error('ダウンロードエラー:', error)
});
```

### 3. GraphQL クエリ

```typescript
import { switchMap } from 'rxjs';
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
        throw new Error(`HTTP Error: ${response.status}`);
      }
      return response.json() as Promise<GraphQLResponse<T>>;
    }),
    map(result => {
      if (result.errors) {
        throw new Error(result.errors.map(e => e.message).join(', '));
      }
      if (!result.data) {
        throw new Error('データが返されませんでした');
      }
      return result.data;
    })
  );
}

// 使用例
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
  next: ({ user }) => console.log('ユーザー:', user),
  error: error => console.error('エラー:', error)
});
```

### 4. ページネーション付きAPI

```typescript
import { expand, takeWhile, reduce, switchMap } from 'rxjs';
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

// 使用例
interface Item {
  id: number;
  name: string;
}

const allItems$ = fetchAllPages<Item>('https://api.example.com/items');

allItems$.subscribe({
  next: items => console.log('全アイテム:', items),
  error: error => console.error('エラー:', error)
});
```

## fromFetch() のオプション

`fromFetch()` は、Fetch API の `RequestInit` オプションをそのまま使用できます。

```typescript
interface RequestInit {
  method?: string;              // HTTPメソッド (GET, POST, PUT, DELETE等)
  headers?: HeadersInit;        // リクエストヘッダー
  body?: BodyInit | null;       // リクエストボディ
  mode?: RequestMode;           // cors, no-cors, same-origin
  credentials?: RequestCredentials; // omit, same-origin, include
  cache?: RequestCache;         // キャッシュモード
  redirect?: RequestRedirect;   // リダイレクト処理
  referrer?: string;            // リファラー
  integrity?: string;           // サブリソース整合性
  signal?: AbortSignal;         // AbortController のシグナル
}
```

## ajax() vs fromFetch() の比較

| 機能 | ajax() | fromFetch() |
|------|--------|-------------|
| ベース技術 | XMLHttpRequest | Fetch API |
| 自動JSONパース | ✅ `getJSON()` | ❌ 手動で `.json()` |
| 自動HTTPエラー検出 | ✅ 4xx/5xxで自動エラー | ❌ 手動で `response.ok` チェック |
| 進捗監視 | ✅ | ❌ |
| タイムアウト | ✅ ビルトイン | ❌ RxJSの `timeout()` で実装 |
| リクエストキャンセル | ✅ unsubscribe() | ✅ unsubscribe() または AbortController |
| IE11対応 | ✅ | ❌ polyfill必要 |
| バンドルサイズ | やや大きい | 小さい |
| Service Worker対応 | ❌ | ✅ |

> [!TIP]
> **使い分けのポイント**
>
> - **モダンブラウザのみ対応**: `fromFetch()` を推奨
> - **レガシーブラウザ対応が必要**: `ajax()` を使用
> - **進捗監視が必要**: `ajax()` を使用
> - **軽量なHTTP通信**: `fromFetch()` が最適
> - **Service Worker内での使用**: `fromFetch()` のみ対応

## よくあるエラーと対処法

### 1. HTTPエラーが `error` コールバックで捕捉されない

**問題:**
```typescript
// ❌ 404エラーでも next が呼ばれる
fromFetch('https://api.example.com/not-found').subscribe({
  next: response => console.log('成功:', response), // ← 404でもこちらが呼ばれる
  error: error => console.error('エラー:', error)
});
```

**対処法:**
```typescript
// ✅ response.ok を手動でチェック
fromFetch('https://api.example.com/not-found').pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP Error: ${response.status}`);
    }
    return response.json();
  })
).subscribe({
  next: data => console.log('データ:', data),
  error: error => console.error('エラー:', error) // ← こちらが呼ばれる
});
```

### 2. CORS エラー

**対処法:**
- サーバー側でCORSヘッダーを設定
- `mode: 'cors'` を明示的に指定
- 開発時はプロキシサーバーを使用

```typescript
fromFetch('https://api.example.com/data', {
  mode: 'cors',
  credentials: 'include' // Cookieを含める場合
});
```

### 3. タイムアウトの実装

Fetch APIにはタイムアウト機能がないため、RxJSの `timeout()` を使用します。

```typescript
import { timeout, switchMap } from 'rxjs';
import { fromFetch } from 'rxjs/fetch';

const api$ = fromFetch('https://api.example.com/slow').pipe(
  timeout(5000), // 5秒でタイムアウト
  switchMap(response => response.json())
);
```

## ベストプラクティス

### 1. 汎用的なfetchJSON関数を作成

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

### 2. TypeScriptの型を活用

```typescript
// ✅ 良い例: 型を明示的に指定
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todo$ = fetchJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ 悪い例: 型指定なし
const todo$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1')
  .pipe(switchMap(res => res.json()));
```

### 3. エラーハンドリングを必ず実装

```typescript
// ✅ 良い例: response.ok とcatchError
const api$ = fromFetch('/api/data').pipe(
  switchMap(response => {
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return response.json();
  }),
  catchError(error => {
    console.error('エラー:', error);
    return of(defaultValue);
  })
);
```

### 4. 購読解除を忘れない

```typescript
// ✅ 良い例: takeUntil で自動解除
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

## まとめ

`fromFetch()` は、モダンな Fetch API をベースにした軽量なHTTP通信の Creation Function です。

**主な特徴:**
- Fetch APIベースで、最新のWeb標準に準拠
- 軽量でバンドルサイズが小さい
- Service Worker内でも使用可能
- 手動でのエラーチェックとレスポンスパースが必要

**使用場面:**
- モダンブラウザのみをサポートする場合
- バンドルサイズを小さくしたい場合
- Service Worker内でHTTP通信を行う場合
- Fetch APIの機能（Request/Responseオブジェクトなど）を直接使いたい場合

**注意点:**
- HTTPエラーでも `error` コールバックは呼ばれない（手動で `response.ok` をチェック）
- JSONパースは手動で行う（`response.json()`）
- 進捗監視は非対応
- IE11などレガシーブラウザでは polyfill が必要

**推奨される使い方:**
- 汎用的な `fetchJSON()` 関数を作成して再利用
- TypeScriptの型を活用して型安全性を確保
- 必ずエラーハンドリングを実装
- 不要になったら必ず購読解除

## 関連ページ

- [ajax()](/guide/creation-functions/http-communication/ajax) - XMLHttpRequestベースのHTTP通信
- [HTTP通信系 Creation Functions](/guide/creation-functions/http-communication/) - ajax() と fromFetch() の比較
- [switchMap()](/guide/operators/transformation/switchMap) - HTTP通信のキャンセルに便利なオペレーター
- [エラーハンドリング戦略](/guide/error-handling/strategies) - HTTP通信のエラー処理パターン

## 参考リソース

- [RxJS公式ドキュメント - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/ja/docs/Web/API/Fetch_API)
- [MDN Web Docs - AbortController](https://developer.mozilla.org/ja/docs/Web/API/AbortController)
