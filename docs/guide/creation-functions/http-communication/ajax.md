---
description: ajax()はXMLHttpRequestベースのHTTP通信をObservableとして扱うRxJS Creation Functionです。GET、POST、PUT、DELETEなどのHTTPメソッドをサポートし、進捗監視、タイムアウト処理、自動JSONパース、レガシーブラウザ対応など実践的な機能を提供します。getJSON()で簡単にJSON APIを呼び出せ、型安全なHTTP通信を実現します。
---

# ajax()

[📘 RxJS公式ドキュメント - ajax](https://rxjs.dev/api/ajax/ajax)

`ajax()` は XMLHttpRequest をベースにした HTTP通信を Observable として扱うための Creation Function です。GET、POST、PUT、DELETE などの HTTP メソッドをサポートし、進捗監視やタイムアウト処理など、実践的な機能を提供します。

## 基本的な使い方

### シンプルなGETリクエスト

`ajax()` を使った最もシンプルな例は、URLを文字列で渡すだけです。

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax('https://jsonplaceholder.typicode.com/todos/1');

api$.subscribe({
  next: response => console.log('レスポンス:', response),
  error: error => console.error('エラー:', error),
  complete: () => console.log('完了')
});

// 出力:
// レスポンス: {
//   status: 200,
//   response: { userId: 1, id: 1, title: "delectus aut autem", completed: false },
//   ...
// }
// 完了
```

### getJSON() を使ったJSONの取得

JSON APIからデータを取得する場合は、`ajax.getJSON()` を使うと便利です。自動的にレスポンスをパースして、`response` プロパティだけを返します。

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
  error: error => console.error('エラー:', error),
  complete: () => console.log('完了')
});

// 出力:
// Todo: { userId: 1, id: 1, title: "delectus aut autem", completed: false }
// 完了
```

> [!TIP]
> **TypeScriptの型安全性**
>
> `ajax.getJSON<T>()` にジェネリック型を指定することで、レスポンスの型安全性を確保できます。

## HTTPメソッド別の使い方

### GET リクエスト

```typescript
import { ajax } from 'rxjs/ajax';

// 方法1: シンプルな文字列指定
const get1$ = ajax('https://api.example.com/users');

// 方法2: getJSON()で自動パース
const get2$ = ajax.getJSON('https://api.example.com/users');

// 方法3: 詳細な設定
const get3$ = ajax({
  url: 'https://api.example.com/users',
  method: 'GET',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer token123'
  }
});
```

### POST リクエスト

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

// 方法1: ajax.post() を使用
const post1$ = ajax.post<CreateUserResponse>(
  'https://api.example.com/users',
  newUser,
  { 'Content-Type': 'application/json' }
);

// 方法2: 詳細な設定
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
  next: response => console.log('作成成功:', response.response),
  error: error => console.error('作成失敗:', error)
});
```

### PUT リクエスト

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
  next: response => console.log('更新成功:', response.response),
  error: error => console.error('更新失敗:', error)
});
```

### PATCH リクエスト

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
  next: response => console.log('部分更新成功:', response.response),
  error: error => console.error('部分更新失敗:', error)
});
```

### DELETE リクエスト

```typescript
import { ajax } from 'rxjs/ajax';

const delete$ = ajax.delete('https://api.example.com/users/1');

delete$.subscribe({
  next: response => console.log('削除成功:', response),
  error: error => console.error('削除失敗:', error)
});
```

## 実践的なパターン

### エラーハンドリングとリトライ

HTTP通信では、ネットワークエラーやサーバーエラーに対処する必要があります。

```typescript
import { of, retry, catchError, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // 5秒でタイムアウト
  retry(2), // 失敗時に2回リトライ
  catchError(error => {
    console.error('ユーザー取得エラー:', error);
    // デフォルト値を返す
    return of({
      id: 0,
      name: 'Unknown',
      email: 'unknown@example.com'
    } as User);
  })
);

fetchUser$.subscribe({
  next: user => console.log('ユーザー:', user),
  error: error => console.error('致命的エラー:', error)
});
```

### HTTPステータスコードによる条件分岐

```typescript
import { throwError, catchError } from 'rxjs';
import { ajax, AjaxError } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/data').pipe(
  catchError((error: AjaxError) => {
    if (error.status === 404) {
      console.error('リソースが見つかりません');
    } else if (error.status === 401) {
      console.error('認証が必要です');
    } else if (error.status === 500) {
      console.error('サーバーエラーが発生しました');
    } else {
      console.error('予期しないエラー:', error);
    }
    return throwError(() => error);
  })
);
```

### 複数のリクエストを並列実行

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

// すべてのリクエストが完了するまで待つ
forkJoin({
  users: users$,
  posts: posts$,
  comments: comments$
}).subscribe({
  next: ({ users, posts, comments }) => {
    console.log('Users:', users);
    console.log('Posts:', posts);
    console.log('Comments:', comments);
  },
  error: error => console.error('いずれかのリクエストが失敗:', error)
});
```

### ユーザー入力に応じた検索（switchMap）

```typescript
import { fromEvent, map, debounceTime, distinctUntilChanged, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface SearchResult {
  id: number;
  title: string;
}

const searchInput = document.querySelector('#search') as HTMLInputElement;

const search$ = fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // 300ms待つ
  distinctUntilChanged(), // 同じ値は無視
  switchMap(query => {
    if (query.length === 0) {
      return of([]);
    }
    // 新しい検索が入力されたら、前のリクエストをキャンセル
    return ajax.getJSON<SearchResult[]>(`https://api.example.com/search?q=${query}`);
  })
);

search$.subscribe({
  next: results => console.log('検索結果:', results),
  error: error => console.error('検索エラー:', error)
});
```

> [!IMPORTANT]
> **switchMap() の重要性**
>
> `switchMap()` を使うことで、新しい検索クエリが入力された際に、前のHTTPリクエストが自動的にキャンセルされます。これにより、古い検索結果が新しい結果を上書きすることを防げます。

### 進捗監視（ファイルアップロード）

`ajax()` は、`XMLHttpRequest` の `progress` イベントを利用して、アップロード・ダウンロードの進捗を監視できます。

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
    // 進捗イベントを有効化
    progressSubscriber: {
      next: (progress) => {
        const percentage = (progress.loaded / progress.total) * 100;
        console.log(`アップロード進捗: ${percentage.toFixed(2)}%`);
      }
    }
  });

  upload$.subscribe({
    next: response => console.log('アップロード完了:', response),
    error: error => console.error('アップロード失敗:', error)
  });
}
```

### カスタムヘッダーとクロスドメインリクエスト

```typescript
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected-resource',
  method: 'GET',
  headers: {
    'Authorization': 'Bearer your-token-here',
    'X-Custom-Header': 'CustomValue'
  },
  crossDomain: true, // CORSリクエスト
  withCredentials: true // Cookieを含める
});

api$.subscribe({
  next: response => console.log('レスポンス:', response),
  error: error => console.error('エラー:', error)
});
```

## よくある使用例

### 1. ページネーション付きAPI呼び出し

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
  next: allItems => console.log('全アイテム:', allItems),
  error: error => console.error('エラー:', error)
});
```

### 2. ポーリング（定期的なデータ取得）

```typescript
import { interval, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Status {
  status: string;
  lastUpdate: string;
}

// 5秒ごとにAPIを呼び出す
const polling$ = interval(5000).pipe(
  switchMap(() => ajax.getJSON<Status>('https://api.example.com/status'))
);

const subscription = polling$.subscribe({
  next: status => console.log('ステータス:', status),
  error: error => console.error('エラー:', error)
});

// 30秒後に停止
setTimeout(() => subscription.unsubscribe(), 30000);
```

### 3. 依存関係のあるリクエスト

```typescript
import { switchMap } from 'rxjs';
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

// まずユーザー情報を取得し、その後詳細情報を取得
const userWithDetails$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  switchMap(user =>
    ajax.getJSON<UserDetails>(`https://api.example.com/users/${user.id}/details`).pipe(
      map(details => ({ ...user, ...details }))
    )
  )
);

userWithDetails$.subscribe({
  next: userWithDetails => console.log('ユーザー詳細:', userWithDetails),
  error: error => console.error('エラー:', error)
});
```

## ajax() のオプション

`ajax()` には、詳細な設定を行うためのオプションが用意されています。

```typescript
interface AjaxConfig {
  url: string;                    // リクエストURL
  method?: string;                // HTTPメソッド（GET, POST, PUT, DELETE等）
  headers?: object;               // リクエストヘッダー
  body?: any;                     // リクエストボディ
  timeout?: number;               // タイムアウト時間（ミリ秒）
  responseType?: string;          // レスポンスタイプ（json, text, blob等）
  crossDomain?: boolean;          // CORSリクエストかどうか
  withCredentials?: boolean;      // Cookieを含めるかどうか
  progressSubscriber?: Subscriber; // 進捗監視用Subscriber
}
```

## よくあるエラーと対処法

### 1. CORS エラー

**エラー例:**
```
Access to XMLHttpRequest at 'https://api.example.com' from origin 'http://localhost:3000'
has been blocked by CORS policy
```

**対処法:**
- サーバー側でCORSヘッダーを設定
- プロキシサーバーを使用
- 開発時は `crossDomain: true` と `withCredentials: false` を試す

### 2. ネットワークタイムアウト

**対処法:**
```typescript
import { timeout, retry } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://api.example.com/slow-endpoint').pipe(
  timeout(10000), // 10秒でタイムアウト
  retry(2) // 2回リトライ
);
```

### 3. 認証エラー（401 Unauthorized）

**対処法:**
```typescript
import { throwError, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const api$ = ajax({
  url: 'https://api.example.com/protected',
  headers: {
    'Authorization': `Bearer ${getAccessToken()}`
  }
}).pipe(
  catchError(error => {
    if (error.status === 401) {
      // トークンをリフレッシュして再試行
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

## ajax() vs fromFetch() の比較

| 機能 | ajax() | fromFetch() |
|------|--------|-------------|
| 自動JSONパース | ✅ `getJSON()` | ❌ 手動で `.json()` |
| 進捗監視 | ✅ | ❌ |
| HTTPエラー自動検出 | ✅ | ❌ |
| バンドルサイズ | やや大きい | 小さい |
| IE11対応 | ✅ | ❌ |

> [!TIP]
> **使い分けのポイント**
>
> - **進捗監視が必要**: `ajax()` を使用
> - **レガシーブラウザ対応**: `ajax()` を使用
> - **軽量なHTTP通信**: `fromFetch()` を検討
> - **シンプルなJSON取得**: `ajax.getJSON()` が最も簡単

## ベストプラクティス

### 1. 型安全性を確保する

```typescript
// ✅ 良い例: ジェネリック型を指定
interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}

const todos$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');

// ❌ 悪い例: 型指定なし
const todos$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
```

### 2. エラーハンドリングを必ず実装する

```typescript
// ✅ 良い例: catchError でエラー処理
const api$ = ajax.getJSON('/api/data').pipe(
  catchError(error => {
    console.error('エラー:', error);
    return of(defaultValue);
  })
);

// ❌ 悪い例: エラーハンドリングなし
const api$ = ajax.getJSON('/api/data');
```

### 3. 購読解除を忘れない

```typescript
// ✅ 良い例: コンポーネント破棄時に解除
class MyComponent {
  private subscription: Subscription;

  ngOnInit() {
    this.subscription = ajax.getJSON('/api/data').subscribe(...);
  }

  ngOnDestroy() {
    this.subscription.unsubscribe();
  }
}

// または takeUntil を使用
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

## まとめ

`ajax()` は、RxJSでHTTP通信を行うための強力な Creation Function です。

**主な特徴:**
- XMLHttpRequestベースで、幅広いブラウザに対応
- `getJSON()` による簡単なJSON取得
- 進捗監視、タイムアウト、リトライなどの実践的な機能
- 自動的なHTTPエラー検出

**使用場面:**
- レガシーブラウザ（IE11など）のサポートが必要な場合
- ファイルアップロード/ダウンロードの進捗を表示したい場合
- シンプルで分かりやすいJSON APIの呼び出し

**注意点:**
- 必ずエラーハンドリングを実装する
- 不要になったら必ず購読解除する
- TypeScriptの型を活用して型安全性を確保する

## 関連ページ

- [fromFetch()](/guide/creation-functions/http-communication/fromFetch) - Fetch APIベースのHTTP通信
- [HTTP通信系 Creation Functions](/guide/creation-functions/http-communication/) - ajax() と fromFetch() の比較
- [switchMap()](/guide/operators/transformation/switchMap) - HTTP通信のキャンセルに便利なオペレーター
- [エラーハンドリング戦略](/guide/error-handling/strategies) - HTTP通信のエラー処理パターン

## 参考リソース

- [RxJS公式ドキュメント - ajax](https://rxjs.dev/api/ajax/ajax)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/ja/docs/Web/API/XMLHttpRequest)
- [Learn RxJS - ajax](https://www.learnrxjs.io/learn-rxjs/operators/creation/ajax)
