---
description: ajax() - XMLHttpRequestベースのHTTP通信Creation Function。進捗イベント、タイムアウト制御、自動JSON解析などの機能を提供します。
---

# ajax() - Ajax/HTTPリクエスト

`ajax()`は、XMLHttpRequestをベースとした強力なHTTP通信機能をObservableで提供するCreation Functionです。

## 概要

`ajax()`はXMLHttpRequestをラップし、HTTP通信をリアクティブに扱えます。GET、POST、PUT、DELETEなどの各種HTTPメソッドに対応し、進捗イベントの取得、タイムアウト制御、レスポンスの自動解析などの機能を持ちます。

**シグネチャ**:
```typescript
function ajax<T>(config: AjaxConfig): Observable<AjaxResponse<T>>
```

**公式ドキュメント**: [📘 RxJS公式: ajax()](https://rxjs.dev/api/ajax/ajax)

## 基本的な使い方

### パターン1: シンプルなGETリクエスト

最も簡単な使用方法です。

```typescript
import { ajax } from 'rxjs/ajax';

ajax('https://jsonplaceholder.typicode.com/users').subscribe({
  next: response => console.log('レスポンス:', response),
  error: err => console.error('エラー:', err)
});

// response の構造:
// {
//   status: 200,
//   response: [...], // ユーザー配列
//   responseType: 'json',
//   xhr: XMLHttpRequest
// }
```

### パターン2: 設定オブジェクトを使用

詳細な設定が可能です。

```typescript
import { ajax } from 'rxjs/ajax';

ajax({
  url: 'https://jsonplaceholder.typicode.com/users',
  method: 'GET',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer your-token'
  },
  timeout: 5000 // 5秒でタイムアウト
}).subscribe({
  next: response => console.log('データ:', response.response),
  error: err => console.error('エラー:', err)
});
```

### パターン3: ヘルパーメソッド

よく使うHTTPメソッドにはヘルパーが用意されています。

```typescript
import { ajax } from 'rxjs/ajax';

// GET (JSON自動解析)
ajax.getJSON<User[]>('https://jsonplaceholder.typicode.com/users').subscribe(
  users => console.log('ユーザー一覧:', users)
);

// POST（新規作成）
ajax.post(
  'https://jsonplaceholder.typicode.com/posts',
  { title: 'New Post', body: 'This is a new post', userId: 1 },
  { 'Content-Type': 'application/json' }
).subscribe(
  response => console.log('作成成功:', response)
);

// PUT（全体更新）
ajax.put(
  'https://jsonplaceholder.typicode.com/posts/1',
  { id: 1, title: 'Updated Post', body: 'Updated content', userId: 1 }
).subscribe(
  response => console.log('更新成功:', response)
);

// PATCH（部分更新）
ajax.patch(
  'https://jsonplaceholder.typicode.com/posts/1',
  { title: 'Patched Title' }
).subscribe(
  response => console.log('部分更新成功:', response)
);

// DELETE（削除）
ajax.delete('https://jsonplaceholder.typicode.com/posts/1').subscribe(
  response => console.log('削除成功:', response)
);
```

## 重要な特徴

### 1. 自動JSON解析

`ajax.getJSON()`を使うと、レスポンスを自動的にJSONとして解析します。

```typescript
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

ajax.getJSON<User[]>('https://jsonplaceholder.typicode.com/users').subscribe(
  users => {
    // 型安全にアクセス可能
    users.forEach(user => {
      console.log(`${user.name} (${user.email})`);
    });
  }
);
```

### 2. 進捗イベントの取得

アップロード/ダウンロードの進捗を監視できます。

```typescript
import { ajax } from 'rxjs/ajax';

const formData = new FormData();
formData.append('file', fileInput.files[0]);

ajax({
  url: 'https://jsonplaceholder.typicode.com/upload',
  method: 'POST',
  body: formData,
  progressSubscriber: {
    next: event => {
      if (event.lengthComputable) {
        const percentDone = (event.loaded / event.total) * 100;
        console.log(`アップロード進捗: ${percentDone.toFixed(2)}%`);
      }
    }
  }
}).subscribe({
  next: response => console.log('アップロード完了:', response),
  error: err => console.error('アップロード失敗:', err)
});
```

### 3. タイムアウト制御

リクエストのタイムアウトを設定できます。

```typescript
import { ajax } from 'rxjs/ajax';

ajax({
  url: 'https://jsonplaceholder.typicode.com/slow-endpoint',
  timeout: 3000 // 3秒でタイムアウト
}).subscribe({
  next: response => console.log('成功:', response),
  error: err => {
    if (err.name === 'AjaxTimeoutError') {
      console.error('タイムアウトしました');
    } else {
      console.error('エラー:', err);
    }
  }
});
```

## 実践的なユースケース

### 1. RESTful API操作

CRUD操作の実装例です。

```typescript
import { ajax } from 'rxjs/ajax';
import { Observable } from 'rxjs';
import { map } from 'rxjs/operators';

interface User {
  id: number;
  name: string;
  email: string;
}

class UserAPI {
  private baseUrl = 'https://jsonplaceholder.typicode.com/users';

  // CREATE
  create(user: Omit<User, 'id'>): Observable<User> {
    return ajax.post(
      this.baseUrl,
      user,
      { 'Content-Type': 'application/json' }
    ).pipe(
      map(response => response.response as User)
    );
  }

  // READ (一覧)
  getAll(): Observable<User[]> {
    return ajax.getJSON<User[]>(this.baseUrl);
  }

  // READ (単体)
  getById(id: number): Observable<User> {
    return ajax.getJSON<User>(`${this.baseUrl}/${id}`);
  }

  // UPDATE
  update(id: number, user: Partial<User>): Observable<User> {
    return ajax.put(
      `${this.baseUrl}/${id}`,
      user
    ).pipe(
      map(response => response.response as User)
    );
  }

  // DELETE
  delete(id: number): Observable<void> {
    return ajax.delete(`${this.baseUrl}/${id}`).pipe(
      map(() => undefined)
    );
  }
}

// 使用例
const userAPI = new UserAPI();

// ユーザー作成
userAPI.create({ name: 'Alice', email: 'alice@example.com' }).subscribe(
  user => console.log('作成されたユーザー:', user)
);

// ユーザー一覧取得
userAPI.getAll().subscribe(
  users => console.log('ユーザー一覧:', users)
);

// ユーザー更新（ID 1 は存在するユーザー）
userAPI.update(1, { name: 'Alice Updated' }).subscribe(
  user => console.log('更新されたユーザー:', user)
);

// ユーザー削除（ID 1 は存在するユーザー）
userAPI.delete(1).subscribe(
  () => console.log('ユーザーを削除しました')
);
```

### 2. 認証トークン付きリクエスト

認証が必要なAPIへのリクエスト例です。

```typescript
import { ajax } from 'rxjs/ajax';
import { map, catchError } from 'rxjs/operators';
import { Observable, throwError } from 'rxjs';

class AuthenticatedAPI {
  private getHeaders() {
    const token = localStorage.getItem('auth_token');
    return {
      'Content-Type': 'application/json',
      'Authorization': token ? `Bearer ${token}` : ''
    };
  }

  get<T>(url: string): Observable<T> {
    return ajax({
      url,
      method: 'GET',
      headers: this.getHeaders()
    }).pipe(
      map(response => response.response as T),
      catchError(error => {
        if (error.status === 401) {
          console.error('認証が必要です');
          // ログイン画面にリダイレクトなど
        }
        return throwError(() => error);
      })
    );
  }

  post<T>(url: string, body: any): Observable<T> {
    return ajax({
      url,
      method: 'POST',
      headers: this.getHeaders(),
      body
    }).pipe(
      map(response => response.response as T)
    );
  }
}

// 使用例
interface User {
  id: number;
  name: string;
  username?: string;
  email?: string;
  [key: string]: any;
}

const api = new AuthenticatedAPI();

api.get<User>('https://jsonplaceholder.typicode.com/users/1').subscribe(
  user => console.log('ユーザー情報:', user),
  error => console.error('エラー:', error)
);

api.post<any>('https://jsonplaceholder.typicode.com/posts', {
  title: 'New Post',
  body: 'Post content',
  userId: 1
}).subscribe(
  post => console.log('投稿作成:', post),
  error => console.error('エラー:', error)
);
```

### 3. ファイルアップロード with 進捗表示

ファイルアップロードと進捗表示の実装例です。

```typescript
import { ajax } from 'rxjs/ajax';

// DOM要素を動的に作成
const fileInput = document.createElement('input');
fileInput.type = 'file';
document.body.appendChild(fileInput);

const progressDiv = document.createElement('div');
progressDiv.style.marginTop = '10px';
document.body.appendChild(progressDiv);

// ファイル選択時にアップロード
fileInput.addEventListener('change', () => {
  if (!fileInput.files || fileInput.files.length === 0) {
    console.error('ファイルが選択されていません');
    return;
  }

  const formData = new FormData();
  formData.append('file', fileInput.files[0]);

  ajax({
    url: 'https://jsonplaceholder.typicode.com/posts',
    method: 'POST',
    body: formData,
    progressSubscriber: {
      next: event => {
        if (event.lengthComputable) {
          const percentDone = (event.loaded / event.total) * 100;
          progressDiv.textContent = `アップロード進捗: ${percentDone.toFixed(2)}%`;
          console.log(`アップロード進捗: ${percentDone.toFixed(2)}%`);
        }
      }
    }
  }).subscribe({
    next: response => {
      console.log('アップロード完了:', response);
      progressDiv.textContent = 'アップロード完了！';
    },
    error: err => {
      console.error('アップロード失敗:', err);
      progressDiv.textContent = 'アップロード失敗';
    }
  });
});
```

### 4. リトライとエラーハンドリング

ネットワークエラーに対する堅牢な処理です。

```typescript
import { ajax } from 'rxjs/ajax';
import { retry, catchError, delay } from 'rxjs/operators';
import { Observable, of } from 'rxjs';
// 使用例
interface User {
  id: number;
  name: string;
  username?: string;
  email?: string;
  [key: string]: any;
}

function fetchWithRetry<T>(url: string, maxRetries: number = 3): Observable<T> {
  return ajax.getJSON<T>(url).pipe(
    retry({
      count: maxRetries,
      delay: (error, retryCount) => {
        // 指数バックオフ: 1秒、2秒、4秒
        const delayMs = Math.pow(2, retryCount - 1) * 1000;
        console.log(`リトライ ${retryCount}/${maxRetries} (${delayMs}ms後)`);
        return of(null).pipe(delay(delayMs));
      }
    }),
    catchError(error => {
      console.error(`${maxRetries}回試行後も失敗:`, error);
      // デフォルト値を返す
      return of({} as T);
    })
  );
}

// 使用例
fetchWithRetry<User[]>('https://jsonplaceholder.typicode.com/users').subscribe(
  users => console.log('ユーザー:', users)
);
```

### 5. 並列リクエストと順次リクエスト

複数のAPIリクエストを効率的に処理します。

```typescript
import { ajax } from 'rxjs/ajax';
import { forkJoin } from 'rxjs';
import { concatMap } from 'rxjs/operators';
// 使用例
interface User {
  id: number;
  name: string;
  username?: string;
  email?: string;
  [key: string]: any;
}

interface Post {
  id: number;
  userId?: number;
  title?: string;
  body?: string;
  [key: string]: any;
}
// 並列リクエスト（すべての完了を待つ）
function fetchMultipleResources() {
  return forkJoin({
    users: ajax.getJSON('/api/users'),
    posts: ajax.getJSON('/api/posts'),
    comments: ajax.getJSON('/api/comments')
  }).subscribe(({ users, posts, comments }) => {
    console.log('すべてのデータ取得完了');
    console.log('ユーザー:', users);
    console.log('投稿:', posts);
    console.log('コメント:', comments);
  });
}

// 順次リクエスト（前のリクエストに依存）
function fetchUserAndPosts(userId: number) {
  ajax.getJSON<User>(`/api/users/${userId}`).pipe(
    concatMap(user => {
      console.log('ユーザー取得:', user);
      // ユーザー情報を取得後、そのユーザーの投稿を取得
      return ajax.getJSON<Post[]>(`/api/users/${user.id}/posts`);
    })
  ).subscribe(posts => {
    console.log('投稿:', posts);
  });
}
```

## エラーハンドリング

`ajax()`で発生する主なエラーと対処方法です。

### HTTPステータスエラー

```typescript
import { ajax, AjaxError } from 'rxjs/ajax';
import { catchError } from 'rxjs/operators';
import { throwError } from 'rxjs';

ajax.getJSON('/api/users').pipe(
  catchError((error: AjaxError) => {
    switch (error.status) {
      case 400:
        console.error('リクエストが不正です');
        break;
      case 401:
        console.error('認証が必要です');
        // ログイン画面へリダイレクト
        break;
      case 404:
        console.error('リソースが見つかりません');
        break;
      case 500:
        console.error('サーバーエラーが発生しました');
        break;
      default:
        console.error('予期しないエラー:', error.message);
    }
    return throwError(() => error);
  })
).subscribe({
  next: data => console.log('データ:', data),
  error: err => console.error('最終エラー:', err)
});
```

### ネットワークエラー

```typescript
import { ajax } from 'rxjs/ajax';
import { catchError, timeout } from 'rxjs/operators';
import { of } from 'rxjs';

ajax.getJSON('/api/data').pipe(
  timeout(5000), // 5秒でタイムアウト
  catchError(error => {
    if (error.name === 'TimeoutError') {
      console.error('リクエストがタイムアウトしました');
    } else if (error.status === 0) {
      console.error('ネットワークエラーまたはCORS問題');
    }
    // デフォルト値を返す
    return of({ error: true, message: error.message });
  })
).subscribe(data => console.log(data));
```

## TypeScriptでの型安全性

`ajax()`はTypeScriptと完全に統合されています。

```typescript
import { ajax } from 'rxjs/ajax';
import { map } from 'rxjs/operators';

interface User {
  id: number;
  name: string;
  email: string;
}

interface ApiResponse<T> {
  data: T;
  message: string;
}

// レスポンスの型を指定
ajax<ApiResponse<User[]>>({
  url: '/api/users',
  method: 'GET'
}).pipe(
  map(response => response.response.data)
).subscribe(users => {
  // users は User[] 型
  users.forEach(user => {
    console.log(user.name); // 型安全
  });
});
```

## パフォーマンスとセキュリティ

### CORS設定

クロスオリジンリクエストの処理です。

```typescript
import { ajax } from 'rxjs/ajax';

ajax({
  url: 'https://jsonplaceholder.typicode.com/data',
  method: 'GET',
  crossDomain: true, // クロスドメインリクエスト
  withCredentials: true, // クッキーを含める
  headers: {
    'Content-Type': 'application/json'
  }
}).subscribe(response => console.log(response));
```

> [!WARNING]
> **セキュリティの注意点**:
> - `withCredentials: true`を使用する場合、サーバー側で適切なCORS設定が必要
> - APIトークンをクライアント側コードに直接埋め込まない
> - HTTPS通信を使用する
> - 機密情報をURLパラメーターに含めない

### リクエストのキャンセル

不要になったリクエストをキャンセルします。

```typescript
import { ajax } from 'rxjs/ajax';

const subscription = ajax.getJSON('/api/data').subscribe(
  data => console.log(data)
);

// リクエストをキャンセル
setTimeout(() => {
  subscription.unsubscribe();
  console.log('リクエストをキャンセルしました');
}, 1000);
```

## まとめ

`ajax()`は、XMLHttpRequestをベースとした強力で柔軟なHTTP通信機能を提供します。

> [!IMPORTANT]
> **ajax()の特徴**:
> - ✅ 自動JSON解析（`ajax.getJSON()`）
> - ✅ 進捗イベントの取得
> - ✅ タイムアウト制御
> - ✅ 広範なブラウザサポート（IE11+）
> - ✅ TypeScript完全対応
> - ⚠️ XHRベース（モダンなFetch APIではない）
> - ⚠️ Service Workerとの統合は制限あり

## 関連項目

- [fromFetch()](/guide/creation-functions/conversion/fromFetch) - Fetch APIのラッパー
- [switchMap()](/guide/operators/transformation/switchMap) - リクエストの切り替え
- [catchError()](/guide/error-handling/retry-catch) - エラーハンドリング
- [retry()](/guide/error-handling/retry-catch) - リトライ処理

## 参考リソース

- [RxJS公式: ajax()](https://rxjs.dev/api/ajax/ajax)
- [Learn RxJS: ajax](https://www.learnrxjs.io/learn-rxjs/operators/creation/ajax)
- [MDN: XMLHttpRequest](https://developer.mozilla.org/ja/docs/Web/API/XMLHttpRequest)
