---
description: fromFetch() - Fetch APIのラッパーCreation Function。モダンなHTTP通信、ストリーミング対応、Service Worker統合に最適です。
---

# fromFetch() - Fetch APIのラッパー

`fromFetch()`は、モダンなFetch APIをObservableでラップするCreation Functionです。

## 概要

`fromFetch()`はブラウザネイティブのFetch APIをRxJSのObservableに変換します。`ajax()`に比べてよりモダンなAPI設計で、ストリーミングレスポンス、Service Worker統合、AbortControllerによるキャンセル制御などをサポートします。

**シグネチャ**:
```typescript
function fromFetch(
  input: string | Request,
  init?: RequestInit & { selector?: (response: Response) => ObservableInput<any> }
): Observable<Response>
```

**公式ドキュメント**: [📘 RxJS公式: fromFetch()](https://rxjs.dev/api/fetch/fromFetch)

## 基本的な使い方

### パターン1: シンプルなGETリクエスト

最も基本的な使用方法です。

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap, catchError } from 'rxjs/operators';
import { of } from 'rxjs';

fromFetch('https://jsonplaceholder.typicode.com/users').pipe(
  switchMap(response => {
    if (response.ok) {
      // レスポンスが成功の場合、JSONを解析
      return response.json();
    } else {
      // エラーレスポンスの処理
      return of({ error: true, message: `HTTP ${response.status}` });
    }
  }),
  catchError(err => {
    // ネットワークエラーの処理
    console.error('リクエスト失敗:', err);
    return of({ error: true, message: err.message });
  })
).subscribe(data => console.log('データ:', data));
```

### パターン2: selectorを使用した簡潔な記述

`selector`オプションでレスポンス処理を簡潔に書けます。

```typescript
import { fromFetch } from 'rxjs/fetch';
import { catchError } from 'rxjs/operators';
import { of } from 'rxjs';

fromFetch('https://jsonplaceholder.typicode.com/users', {
  selector: response => response.json()
}).pipe(
  catchError(err => {
    console.error('エラー:', err);
    return of({ error: true });
  })
).subscribe(data => console.log('データ:', data));
```

### パターン3: POSTリクエスト

データを送信する場合の例です。

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs/operators';

const post = {
  title: 'New Post',
  body: 'This is a new post content',
  userId: 1
};

fromFetch('https://jsonplaceholder.typicode.com/posts', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json'
  },
  body: JSON.stringify(post)
}).pipe(
  switchMap(response => {
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    return response.json();
  })
).subscribe({
  next: data => console.log('作成成功:', data),
  error: err => console.error('作成失敗:', err)
});
```

## 重要な特徴

### 1. AbortControllerによるキャンセル

`fromFetch()`は購読解除時に自動的にリクエストをキャンセルします。

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs/operators';

const subscription = fromFetch('https://jsonplaceholder.typicode.com/slow-data').pipe(
  switchMap(response => response.json())
).subscribe(data => console.log(data));

// 1秒後にリクエストをキャンセル
setTimeout(() => {
  subscription.unsubscribe();
  console.log('リクエストをキャンセルしました');
}, 1000);
```

### 2. ストリーミングレスポンス

レスポンスボディをストリーミングとして処理できます。

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs/operators';
import { from } from 'rxjs';

fromFetch('https://jsonplaceholder.typicode.com/todos').pipe(
  switchMap(response => {
    if (!response.body) {
      throw new Error('レスポンスボディがありません');
    }

    const reader = response.body.getReader();
    const decoder = new TextDecoder();

    // ReadableStreamをObservableに変換
    return from(new ReadableStream({
      start(controller) {
        function push() {
          reader.read().then(({ done, value }) => {
            if (done) {
              controller.close();
              return;
            }
            const text = decoder.decode(value);
            console.log('チャンク受信:', text);
            controller.enqueue(text);
            push();
          });
        }
        push();
      }
    }));
  })
).subscribe({
  complete: () => console.log('ストリーム完了')
});
```

### 3. Service Workerとの統合

Service Workerを使ったオフライン対応が可能です。

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap, catchError } from 'rxjs/operators';
import { of } from 'rxjs';

function fetchWithCache(url: string) {
  return fromFetch(url).pipe(
    switchMap(response => {
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      // Service Workerがキャッシュを処理
      return response.json();
    }),
    catchError(err => {
      console.log('キャッシュから取得を試みます');
      // フォールバック処理
      return of({ cached: true, data: [] });
    })
  );
}
```

## 実践的なユースケース

### 1. TypeScript型安全なAPIクライアント

型安全なAPIクライアントの実装例です。

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap, map, catchError } from 'rxjs/operators';
import { Observable, throwError } from 'rxjs';

interface Post {
  id: number;
  title: string;
  body: string;
  userId: number;
}

interface ApiError {
  error: true;
  status: number;
  message: string;
}

class TypeSafeAPI {
  private baseUrl = 'https://jsonplaceholder.typicode.com';

  private request<T>(
    endpoint: string,
    options?: RequestInit
  ): Observable<T> {
    return fromFetch(`${this.baseUrl}${endpoint}`, options).pipe(
      switchMap(response => {
        if (!response.ok) {
          return throwError(() => ({
            error: true,
            status: response.status,
            message: response.statusText
          } as ApiError));
        }
        return response.json() as Promise<T>;
      }),
      catchError(err => {
        console.error('リクエストエラー:', err);
        return throwError(() => err);
      })
    );
  }

  getPosts(): Observable<Post[]> {
    return this.request<Post[]>('/posts');
  }

  getPost(id: number): Observable<Post> {
    return this.request<Post>(`/posts/${id}`);
  }

  createPost(post: Omit<Post, 'id'>): Observable<Post> {
    return this.request<Post>('/posts', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(post)
    });
  }

  updatePost(id: number, post: Partial<Post>): Observable<Post> {
    return this.request<Post>(`/posts/${id}`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(post)
    });
  }

  deletePost(id: number): Observable<void> {
    return this.request<void>(`/posts/${id}`, {
      method: 'DELETE'
    });
  }
}

// 使用例
const api = new TypeSafeAPI();

api.getPosts().subscribe({
  next: posts => console.log('投稿一覧:', posts),
  error: (err: ApiError) => console.error(`エラー ${err.status}:`, err.message)
});

api.createPost({
  title: 'New Post',
  body: 'Post content here',
  userId: 1
}).subscribe({
  next: post => console.log('作成された投稿:', post),
  error: err => console.error('作成失敗:', err)
});
```

### 2. インターセプター実装

リクエスト/レスポンスのインターセプター実装です。

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap, tap, catchError } from 'rxjs/operators';
import { Observable, throwError } from 'rxjs';

class APIClient {
  private token: string | null = null;

  setToken(token: string) {
    this.token = token;
  }

  private addAuthHeader(init: RequestInit = {}): RequestInit {
    const headers = new Headers(init.headers);

    if (this.token) {
      headers.set('Authorization', `Bearer ${this.token}`);
    }

    return {
      ...init,
      headers
    };
  }

  request<T>(url: string, init?: RequestInit): Observable<T> {
    // リクエストインターセプター
    const requestInit = this.addAuthHeader(init);

    console.log('[Request]', init?.method || 'GET', url);

    return fromFetch(url, requestInit).pipe(
      // レスポンスインターセプター
      tap(response => {
        console.log('[Response]', response.status, url);
      }),
      switchMap(response => {
        if (response.status === 401) {
          // 認証エラー処理
          console.error('認証エラー: トークンが無効です');
          this.token = null;
          return throwError(() => new Error('Unauthorized'));
        }

        if (!response.ok) {
          return throwError(() => new Error(`HTTP ${response.status}`));
        }

        return response.json() as Promise<T>;
      }),
      catchError(err => {
        console.error('[Error]', err);
        return throwError(() => err);
      })
    );
  }
}

// 使用例
const client = new APIClient();
client.setToken('your-auth-token');

client.request<{ data: string }>('/api/protected').subscribe({
  next: data => console.log('データ:', data),
  error: err => console.error('エラー:', err)
});
```

### 3. 検索機能with デバウンス

ユーザー入力に応じた検索機能です。

> [!TIP]
> このコードは**ブラウザ環境専用**です。ブラウザのコンソールやHTMLファイルにそのまま貼り付けて試せます。

```typescript
import { fromEvent, of } from 'rxjs';
import { debounceTime, distinctUntilChanged, switchMap, map } from 'rxjs/operators';
import { fromFetch } from 'rxjs/fetch';

interface SearchResult {
  id: number;
  title: string;
  body: string;
}

// DOM要素を動的に作成
const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = '検索...';
const label = document.createElement('label');
label.innerText = '検索: ';
label.appendChild(searchInput);
document.body.appendChild(label);

const resultsDiv = document.createElement('div');
resultsDiv.style.marginTop = '20px';
document.body.appendChild(resultsDiv);

// 検索機能を実装
const inputEvent$ = fromEvent(searchInput, 'input');
const result = inputEvent$.pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // 300ms待機
  distinctUntilChanged(), // 値が変化した時のみ
  switchMap(query => {
    if (!query) {
      return of([]);
    }

    // 新しい検索で古い検索をキャンセル
    return fromFetch(`https://jsonplaceholder.typicode.com/posts?q=${encodeURIComponent(query)}`, {
      selector: response => {
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        return response.json();
      }
    });
  })
);

result.subscribe({
  next: (results: SearchResult[]) => {
    resultsDiv.innerHTML = results.length === 0
      ? '<p>結果がありません</p>'
      : results.map(r => `
          <div class="result" style="border: 1px solid #ccc; padding: 10px; margin: 10px 0;">
            <h3>${r.title}</h3>
            <p>${r.body}</p>
          </div>
        `).join('');
  },
  error: err => {
    console.error('検索エラー:', err);
    resultsDiv.innerHTML = '<p>検索中にエラーが発生しました</p>';
  }
});
```

### 4. ページネーション

無限スクロールの実装例です。

> [!TIP]
> このコードは**ブラウザ環境専用**です。ブラウザのコンソールやHTMLファイルにそのまま貼り付けて試せます。

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, scan } from 'rxjs/operators';
import { fromFetch } from 'rxjs/fetch';

interface Post {
  id: number;
  title: string;
  body: string;
}

// DOM要素を動的に作成
const postsContainer = document.createElement('div');
postsContainer.style.marginBottom = '20px';
document.body.appendChild(postsContainer);

const loadMoreButton = document.createElement('button');
loadMoreButton.innerText = 'もっと読み込む';
document.body.appendChild(loadMoreButton);

// ページネーション機能を実装
const loadMoreClick$ = fromEvent(loadMoreButton, 'click');

const result = loadMoreClick$.pipe(
  scan((page, _) => page + 1, 0), // ページ番号を累積
  switchMap(page => {
    console.log(`ページ ${page + 1} を読み込み中...`);

    return fromFetch(`https://jsonplaceholder.typicode.com/posts?_page=${page}&_limit=10`, {
      selector: response => response.json()
    });
  }),
  scan((allPosts: Post[], newPosts: Post[]) => {
    // すべての投稿を蓄積
    return [...allPosts, ...newPosts];
  }, [])
);

result.subscribe({
  next: posts => {
    // 投稿を表示
    postsContainer.innerHTML = posts.map(post => `
      <article style="border: 1px solid #ccc; padding: 10px; margin: 10px 0;">
        <h2>${post.title}</h2>
        <p>${post.body}</p>
      </article>
    `).join('');

    console.log(`合計 ${posts.length} 件の投稿を表示中`);
  },
  error: err => console.error('読み込みエラー:', err)
});

// 初回読み込み
loadMoreButton.click();
```

### 5. ファイルアップロード

FormDataを使ったファイルアップロードです。

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs/operators';

function uploadFile(file: File): Observable<any> {
  const formData = new FormData();
  formData.append('file', file);
  formData.append('description', 'Uploaded via fromFetch');

  return fromFetch('https://jsonplaceholder.typicode.com/posts', {
    method: 'POST',
    body: formData
    // Content-Typeヘッダーは自動で設定される
  }).pipe(
    switchMap(response => {
      if (!response.ok) {
        throw new Error(`アップロード失敗: HTTP ${response.status}`);
      }
      return response.json();
    })
  );
}

// 使用例
const fileInput = document.getElementById('file') as HTMLInputElement;
fileInput.addEventListener('change', (event) => {
  const file = (event.target as HTMLInputElement).files?.[0];
  if (file) {
    uploadFile(file).subscribe({
      next: result => console.log('アップロード成功:', result),
      error: err => console.error('アップロード失敗:', err)
    });
  }
});
```

## ajax() との比較

`fromFetch()`と`ajax()`の違いを理解して、適切に選択しましょう。

### fromFetch() の利点

```typescript
import { fromFetch } from 'rxjs/fetch';

// モダンなFetch API
fromFetch('/api/data', {
  credentials: 'include', // クッキーを含める
  mode: 'cors', // CORSモード
  cache: 'no-cache' // キャッシュ制御
}).pipe(
  switchMap(response => response.json())
).subscribe(data => console.log(data));
```

**利点**:
- よりモダンなAPI設計
- AbortControllerによる標準的なキャンセル
- ストリーミングレスポンス対応
- Service Workerとの統合
- Promiseベースで自然な非同期処理

### ajax() の利点

```typescript
import { ajax } from 'rxjs/ajax';

// 自動JSON解析と進捗イベント
ajax({
  url: 'https://jsonplaceholder.typicode.com/posts',
  method: 'POST',
  body: formData,
  progressSubscriber: {
    next: event => {
      const progress = (event.loaded / event.total) * 100;
      console.log(`進捗: ${progress}%`);
    }
  }
}).subscribe(response => console.log(response));
```

**利点**:
- 進捗イベントの取得が簡単
- レスポンスの自動JSON解析
- タイムアウト設定が組み込み
- 古いブラウザ対応（IE11+）

> [!TIP]
> **選択のガイドライン**:
> - **進捗表示が必要** → `ajax()`
> - **古いブラウザサポート** → `ajax()`
> - **モダンなAPI、ストリーミング** → `fromFetch()`
> - **Service Worker統合** → `fromFetch()`
> - **AbortControllerを使いたい** → `fromFetch()`

## エラーハンドリング

`fromFetch()`では、HTTPエラーとネットワークエラーを区別して処理します。

### HTTPステータスエラー

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap, catchError } from 'rxjs/operators';
import { of } from 'rxjs';

fromFetch('/api/users').pipe(
  switchMap(response => {
    // HTTPステータスコードをチェック
    if (response.ok) {
      return response.json();
    }

    // HTTPエラーを明示的に処理
    switch (response.status) {
      case 400:
        throw new Error('リクエストが不正です');
      case 401:
        throw new Error('認証が必要です');
      case 404:
        throw new Error('リソースが見つかりません');
      case 500:
        throw new Error('サーバーエラー');
      default:
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
    }
  }),
  catchError(err => {
    console.error('エラー:', err.message);
    return of({ error: true, message: err.message });
  })
).subscribe(data => console.log(data));
```

### ネットワークエラー

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap, catchError, timeout } from 'rxjs/operators';
import { of } from 'rxjs';

fromFetch('/api/data').pipe(
  timeout(5000), // 5秒でタイムアウト
  switchMap(response => response.json()),
  catchError(err => {
    if (err.name === 'TimeoutError') {
      console.error('タイムアウトしました');
    } else if (err instanceof TypeError) {
      console.error('ネットワークエラーまたはCORS問題');
    } else {
      console.error('予期しないエラー:', err);
    }
    return of({ error: true });
  })
).subscribe(data => console.log(data));
```

## まとめ

`fromFetch()`は、モダンなFetch APIをRxJSで活用するための強力なツールです。

> [!IMPORTANT]
> **fromFetch()の特徴**:
> - ✅ モダンなFetch APIベース
> - ✅ AbortControllerによる標準的なキャンセル
> - ✅ ストリーミングレスポンス対応
> - ✅ Service Workerとの統合
> - ✅ TypeScript完全対応
> - ⚠️ 進捗イベントは手動実装が必要
> - ⚠️ IE11未対応（モダンブラウザのみ）
> - ⚠️ レスポンス処理が若干冗長（`switchMap`が必要）

## 関連項目

- [ajax()](/guide/creation-functions/conversion/ajax) - XMLHttpRequestベースのHTTP通信
- [switchMap()](/guide/operators/transformation/switchMap) - リクエストの切り替え
- [catchError()](/guide/error-handling/retry-catch) - エラーハンドリング

## 参考リソース

- [RxJS公式: fromFetch()](https://rxjs.dev/api/fetch/fromFetch)
- [MDN: Fetch API](https://developer.mozilla.org/ja/docs/Web/API/Fetch_API)
- [MDN: AbortController](https://developer.mozilla.org/ja/docs/Web/API/AbortController)
