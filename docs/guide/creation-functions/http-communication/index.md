---
description: RxJSでHTTP通信を行うための Creation Functions である ajax と fromFetch の概要、違い、使い分けのガイドラインについて解説します。
---

# HTTP通信系 Creation Functions

RxJSでは、HTTP通信を Observable として扱うための Creation Functions が提供されています。このセクションでは、`ajax()` と `fromFetch()` の2つの関数について詳しく解説します。

## HTTP通信系 Creation Functions とは

HTTP通信系 Creation Functions は、外部APIやサーバーとの通信を Observable ストリームとして扱えるようにする関数群です。これらを使用することで、非同期HTTP通信をRxJSのオペレーターチェーンに統合し、エラーハンドリングやリトライ処理などを宣言的に記述できます。

### 主な特徴

- **宣言的なHTTP通信**: HTTP通信を Observable として扱うことで、オペレーターを使った宣言的な処理が可能
- **統一的なエラーハンドリング**: `catchError()` や `retry()` などのオペレーターでエラー処理を統一
- **キャンセル可能**: `unsubscribe()` でリクエストをキャンセル可能
- **他のストリームとの統合**: `switchMap()` などで他のObservableと組み合わせ可能

## HTTP通信系 Creation Functions の一覧

| 関数 | 説明 | ベース技術 | 主な用途 |
|------|------|-----------|---------|
| [ajax()](/guide/creation-functions/http-communication/ajax) | XMLHttpRequestベースのHTTP通信 | XMLHttpRequest | レガシーブラウザ対応、進捗監視 |
| [fromFetch()](/guide/creation-functions/http-communication/fromFetch) | Fetch APIベースのHTTP通信 | Fetch API | モダンブラウザ、軽量なHTTP通信 |

## ajax() vs fromFetch() の比較

### 基本的な違い

```typescript
import { ajax } from 'rxjs/ajax';
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs/operators';

// ajax() - レスポンスを自動的にパース
const ajax$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');
ajax$.subscribe(data => console.log(data));

// fromFetch() - 手動でレスポンスをパース
const fetch$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1').pipe(
  switchMap(response => response.json())
);
fetch$.subscribe(data => console.log(data));

interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}
```

### 機能比較表

| 機能 | ajax() | fromFetch() |
|------|--------|-------------|
| ベース技術 | XMLHttpRequest | Fetch API |
| 自動JSONパース | ✅ `getJSON()` で対応 | ❌ 手動で `.json()` 呼び出し |
| 進捗イベント | ✅ 対応 | ❌ 非対応 |
| タイムアウト | ✅ ビルトイン対応 | ❌ 手動実装が必要 |
| HTTPエラー自動検出 | ✅ 4xx/5xxで自動的にエラー | ❌ 手動でステータスチェックが必要 |
| リクエストキャンセル | ✅ unsubscribe()で可能 | ✅ unsubscribe()で可能 |
| IE11対応 | ✅ 対応 | ❌ polyfill必要 |
| バンドルサイズ | やや大きい | 小さい |

## 使い分けのガイドライン

### ajax() を選ぶべき場合

1. **レガシーブラウザ対応が必要**
   - IE11など古いブラウザをサポートする必要がある場合

2. **進捗監視が必要**
   - ファイルアップロード・ダウンロードの進捗を表示したい場合

3. **シンプルなJSON取得**
   - `getJSON()` で簡単にJSONを取得したい場合

4. **自動的なエラー検出が必要**
   - HTTPステータスコードでの自動エラー検出を利用したい場合

### fromFetch() を選ぶべき場合

1. **モダンブラウザのみ対応**
   - Fetch APIが使える環境のみをサポートする場合

2. **バンドルサイズを小さくしたい**
   - 軽量なHTTP通信機能で十分な場合

3. **Fetch APIの機能を使いたい**
   - Request/Responseオブジェクトを直接操作したい場合
   - Service Worker内で使用したい場合

4. **細かい制御が必要**
   - レスポンスの処理を細かくカスタマイズしたい場合

## 実践的な使用例

### API呼び出しパターン

```typescript
import { ajax } from 'rxjs/ajax';
import { catchError, retry, timeout } from 'rxjs/operators';
import { of } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
}

// ajax() を使った実践的なパターン
const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // 5秒でタイムアウト
  retry(2), // 失敗時に2回リトライ
  catchError(error => {
    console.error('ユーザー取得エラー:', error);
    return of(null); // エラー時はnullを返す
  })
);

fetchUser$.subscribe({
  next: user => {
    if (user) {
      console.log('ユーザー:', user);
    } else {
      console.log('ユーザーの取得に失敗しました');
    }
  }
});
```

### フォーム送信パターン

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, map } from 'rxjs/operators';
import { ajax } from 'rxjs/ajax';

// フォームのsubmitイベントをObservable化
const form = document.querySelector('form') as HTMLFormElement;
const submit$ = fromEvent(form, 'submit').pipe(
  map(event => {
    event.preventDefault();
    const formData = new FormData(form);
    return Object.fromEntries(formData.entries());
  }),
  switchMap(data =>
    ajax.post('https://api.example.com/submit', data, {
      'Content-Type': 'application/json'
    })
  )
);

submit$.subscribe({
  next: response => console.log('送信成功:', response),
  error: error => console.error('送信エラー:', error)
});
```

## よくある質問

### Q1: ajax() と fromFetch() のどちらを使うべきですか？

**A:** モダンブラウザのみ対応であれば `fromFetch()` を推奨します。理由は以下の通りです。
- Fetch APIは最新のWeb標準
- バンドルサイズが小さい
- 将来的な互換性が高い

ただし、以下の場合は `ajax()` を選択してください。
- IE11対応が必要
- 進捗監視が必要
- シンプルなJSON取得で十分

### Q2: HTTPエラー（4xx, 5xx）はどのように処理されますか？

**A:**
- **ajax()**: HTTPステータスコードが400以上の場合、自動的にエラーとして扱われ、`error` コールバックが呼ばれます
- **fromFetch()**: HTTPエラーでも `next` コールバックが呼ばれます。手動で `response.ok` をチェックする必要があります

### Q3: リクエストをキャンセルするには？

**A:** どちらも `unsubscribe()` でキャンセル可能です。

```typescript
const subscription = ajax.getJSON('/api/data').subscribe(...);

// 3秒後にキャンセル
setTimeout(() => subscription.unsubscribe(), 3000);
```

## 次のステップ

各関数の詳細な使い方については、以下のページを参照してください。

- [ajax() の詳細](/guide/creation-functions/http-communication/ajax) - XMLHttpRequestベースのHTTP通信
- [fromFetch() の詳細](/guide/creation-functions/http-communication/fromFetch) - Fetch APIベースのHTTP通信

## 参考リソース

- [RxJS公式ドキュメント - ajax](https://rxjs.dev/api/ajax/ajax)
- [RxJS公式ドキュメント - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/ja/docs/Web/API/Fetch_API)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/ja/docs/Web/API/XMLHttpRequest)
