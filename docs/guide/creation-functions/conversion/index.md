---
description: 既存のAPIや非同期処理をObservableに変換するCreation Functionsについて解説します。ajax、fromFetch、bindCallback、bindNodeCallbackの使い方を学びます。
---

# 変換系 Creation Functions

既存のAPIやコールバックベースの非同期処理をObservableに変換するためのCreation Functionsです。

## 変換系 Creation Functions とは

変換系のCreation Functionsは、既存の非RxJS APIをObservableストリームに変換します。HTTPリクエスト、Fetch API、コールバック関数など、従来の非同期処理パターンをリアクティブなストリームに統合できます。

以下の表で、各Creation Functionの特徴と使い分けを確認してください。

## 主要な変換系 Creation Functions

| Function | 説明 | ユースケース |
|----------|------|-------------|
| **[ajax](/guide/creation-functions/conversion/ajax)** | Ajax/HTTPリクエスト | API呼び出し、RESTful通信 |
| **[fromFetch](/guide/creation-functions/conversion/fromFetch)** | Fetch APIのラッパー | モダンなHTTPリクエスト、ストリーミング対応 |
| **[bindCallback](/guide/creation-functions/conversion/bindCallback)** | コールバック関数をObservableに変換 | 既存のコールバックAPIの統合 |
| **[bindNodeCallback](/guide/creation-functions/conversion/bindNodeCallback)** | Node.js形式のコールバックをObservableに変換 | Node.js APIのRxJS化 |

## 使い分けの基準

変換系Creation Functionsの選択は、以下の観点で判断します。

### 1. HTTPリクエストの方法

- **XMLHttpRequest (XHR)**: `ajax()` - 古いブラウザ対応、進捗イベント、タイムアウト制御
- **Fetch API**: `fromFetch()` - モダンブラウザ、ストリーミング対応、より柔軟なAPI

### 2. コールバックの形式

- **標準コールバック**: `bindCallback()` - jQuery、一般的なライブラリ
- **Node.js形式 (error-first)**: `bindNodeCallback()` - Node.js標準ライブラリ、fs、http等

### 3. ブラウザ互換性

- **広範なブラウザサポート**: `ajax()` - IE11+対応
- **モダンブラウザ**: `fromFetch()` - ES2015+、Fetch API対応ブラウザ

## 実践的な使用例

### ajax() - HTTP GET リクエスト

`ajax()`はXMLHttpRequestをベースとした強力なHTTP通信機能を提供します。

```typescript
import { ajax } from 'rxjs/ajax';

ajax({
  url: 'https://jsonplaceholder.typicode.com/users',
  method: 'GET',
  headers: {
    'Content-Type': 'application/json'
  }
}).subscribe({
  next: response => console.log('データ:', response.response),
  error: err => console.error('エラー:', err)
});
```

### fromFetch() - Fetch APIの活用

モダンなFetch APIをObservableでラップします。

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap, catchError } from 'rxjs/operators';
import { of } from 'rxjs';

fromFetch('https://jsonplaceholder.typicode.com/users').pipe(
  switchMap(response => {
    if (response.ok) {
      return response.json();
    } else {
      return of({ error: true, message: `エラー: ${response.status}` });
    }
  }),
  catchError(err => {
    console.error('リクエスト失敗:', err);
    return of({ error: true, message: err.message });
  })
).subscribe(data => console.log('データ:', data));
```

### bindCallback() - コールバック関数の変換

既存のコールバックベースAPIをObservableに変換します。

```typescript
import { bindCallback } from 'rxjs';

// 既存のコールバックベースAPI
function legacyAPI(value: string, callback: (result: string) => void) {
  setTimeout(() => callback(`処理結果: ${value}`), 1000);
}

// Observableに変換
const apiAsObservable = bindCallback(legacyAPI);

apiAsObservable('テスト').subscribe(result => {
  console.log(result); // 出力: 処理結果: テスト
});
```

### bindNodeCallback() - Node.js APIの変換

Node.js形式のerror-firstコールバックを変換します。

```typescript
import { bindNodeCallback } from 'rxjs';
import * as fs from 'fs';

// Node.jsのfs.readFileをObservableに変換
const readFileAsObservable = bindNodeCallback(fs.readFile);

// Observable化されたreadFileを使用
const result = readFileAsObservable('./data.txt', 'utf8');
result.subscribe(
  data => console.log('ファイル内容:', data),
  err => console.error('読み込みエラー:', err)
);
```

## ajax() vs fromFetch()

HTTPリクエストには`ajax()`と`fromFetch()`の2つの選択肢があります。

### ajax()の利点

```typescript
import { ajax } from 'rxjs/ajax';

ajax({
  url: '/api/upload',
  method: 'POST',
  body: formData,
  // 進捗イベントを取得
  progressSubscriber: {
    next: event => {
      const percentDone = (event.loaded / event.total) * 100;
      console.log(`アップロード進捗: ${percentDone}%`);
    }
  }
}).subscribe(response => console.log('完了:', response));
```

**利点**:
- アップロード/ダウンロード進捗の取得
- タイムアウト設定が簡単
- レスポンスの自動JSON解析
- 古いブラウザ対応

### fromFetch()の利点

```typescript
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs/operators';

fromFetch('/api/data', {
  headers: { 'Authorization': 'Bearer token' }
}).pipe(
  switchMap(response => response.json())
).subscribe(data => console.log(data));
```

**利点**:
- よりモダンなAPI
- ストリーミングレスポンス対応
- Service Workerとの統合
- CORSのより良い処理

> [!TIP]
> **選択のガイドライン**:
> - **進捗表示が必要** → `ajax()`
> - **古いブラウザサポート** → `ajax()`
> - **モダンなAPI設計** → `fromFetch()`
> - **ストリーミングデータ** → `fromFetch()`

## bindCallback() vs bindNodeCallback()

コールバック関数の形式によって使い分けます。

### 標準コールバック (bindCallback)

```typescript
import { bindCallback } from 'rxjs';

// 標準的なコールバック: 結果のみを受け取る
function standardCallback(value: number, cb: (result: number) => void) {
  cb(value * 2);
}

const wrapped = bindCallback(standardCallback);
wrapped(5).subscribe(result => console.log(result)); // 10
```

### Node.js形式コールバック (bindNodeCallback)

```typescript
import { bindNodeCallback } from 'rxjs';

// Node.js形式: error-first コールバック
function nodejsStyleCallback(
  value: number,
  cb: (error: Error | null, result?: number) => void
) {
  if (value < 0) {
    cb(new Error('負の数は処理できません'));
  } else {
    cb(null, value * 2);
  }
}

const wrapped = bindNodeCallback(nodejsStyleCallback);
wrapped(5).subscribe({
  next: result => console.log(result), // 10
  error: err => console.error(err)
});
```

> [!IMPORTANT]
> **コールバック形式の違い**:
> - **bindCallback**: `callback(result)` 形式
> - **bindNodeCallback**: `callback(error, result)` 形式（Node.js標準）
>
> 間違った関数を使用すると、正しく動作しません。

## Cold から Hot への変換

上記の表に示した通り、**全ての変換系Creation Functionsは Cold Observable を生成します**。購読するたびに独立した実行が開始されます。

### 実践例：APIリクエストの共有

```typescript
import { ajax } from 'rxjs/ajax';
import { share } from 'rxjs/operators';

// ❄️ Cold - 購読ごとに独立したHTTPリクエスト
const coldApi$ = ajax.getJSON('/api/user/123');

coldApi$.subscribe(user => console.log('購読者1:', user));
coldApi$.subscribe(user => console.log('購読者2:', user));
// → 2回のHTTPリクエストが発行される

// 🔥 Hot - 購読者間でリクエストを共有
const hotApi$ = ajax.getJSON('/api/user/123').pipe(share());

hotApi$.subscribe(user => console.log('購読者1:', user));
hotApi$.subscribe(user => console.log('購読者2:', user));
// → 1回のHTTPリクエストのみ（結果を共有）
```

> [!TIP]
> **Hot化が必要なケース**:
> - 同じAPIリクエストを複数のコンポーネントで使用
> - リソース集約的な処理（ファイル読み込みなど）
> - 複数の購読者で同じデータソースを共有
>
> 詳しくは [基本作成系 - Cold から Hot への変換](/guide/creation-functions/basic/#cold-から-hot-への変換) を参照してください。

## エラーハンドリングのパターン

変換系Creation Functionsでは、適切なエラーハンドリングが重要です。

### HTTPリクエストのエラー処理

```typescript
import { ajax } from 'rxjs/ajax';
import { catchError, retry } from 'rxjs/operators';
import { of } from 'rxjs';

ajax.getJSON('/api/data').pipe(
  retry(3), // 3回まで自動リトライ
  catchError(error => {
    console.error('API呼び出し失敗:', error);
    // デフォルト値を返す
    return of({ data: [], error: true });
  })
).subscribe(data => console.log(data));
```

### コールバックのエラー処理

```typescript
import { bindNodeCallback } from 'rxjs';
import { catchError } from 'rxjs/operators';
import { of } from 'rxjs';
import * as fs from 'fs';

const readFile = bindNodeCallback(fs.readFile);

readFile('./data.txt', 'utf8').pipe(
  catchError(error => {
    console.error('ファイル読み込みエラー:', error);
    return of('デフォルトコンテンツ');
  })
).subscribe(content => console.log(content));
```

## Pipeable Operator との関係

変換系Creation Functionsには、直接対応するPipeable Operatorはありません。これらは常にCreation Functionとして使用されます。

ただし、以下のようなオペレーターと組み合わせることで、より高度な処理が可能になります。

| 組み合わせるオペレーター | 用途 |
|-------------------|------|
| `switchMap()` | 新しいリクエストで古いリクエストをキャンセル |
| `concatMap()` | リクエストを順次実行 |
| `mergeMap()` | 複数のリクエストを並列実行 |
| `catchError()` | エラーハンドリング |
| `retry()`, `retryWhen()` | リトライ処理 |
| `timeout()` | タイムアウト制御 |

### 実践例：検索フォームの実装

> [!TIP]
> このコードは**ブラウザ環境専用**です。ブラウザのコンソールやHTMLファイルにそのまま貼り付けて試せます。

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, distinctUntilChanged, switchMap, map } from 'rxjs/operators';
import { ajax } from 'rxjs/ajax';

// DOM要素を動的に作成
const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = '検索...';
const label = document.createElement('label');
label.innerText = '検索: ';
label.appendChild(searchInput);
document.body.appendChild(label);

const inputEvent$ = fromEvent(searchInput, 'input');
const result = inputEvent$.pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // 300ms待機
  distinctUntilChanged(), // 値が変化した時のみ
  switchMap(query => {
    // 新しい検索で古い検索をキャンセル
    return ajax.getJSON(`https://jsonplaceholder.typicode.com/posts?q=${query}`);
  })
);

result.subscribe(
  results => console.log('検索結果:', results),
  err => console.error('エラー:', err)
);
```

## パフォーマンスとセキュリティ

### CORSとセキュリティ

HTTPリクエストを行う際は、CORSポリシーとセキュリティに注意が必要です。

```typescript
import { ajax } from 'rxjs/ajax';

ajax({
  url: 'https://jsonplaceholder.typicode.com/posts',
  method: 'GET',
  headers: {
    'Content-Type': 'application/json',
    // 認証トークンなど
    'Authorization': 'Bearer your-token-here'
  },
  // クレデンシャルを含める場合
  crossDomain: true,
  withCredentials: true
}).subscribe(response => console.log(response));
```

> [!WARNING]
> **セキュリティの注意点**:
> - APIトークンをクライアント側のコードに直接埋め込まない
> - HTTPS通信を使用する
> - CORSポリシーを適切に設定する
> - 機密情報をURLパラメーターに含めない

### パフォーマンス最適化

```typescript
import { ajax } from 'rxjs/ajax';
import { shareReplay, timeout } from 'rxjs/operators';

// キャッシュ付きAPIリクエスト
const cachedData$ = ajax.getJSON('/api/config').pipe(
  timeout(5000), // 5秒でタイムアウト
  shareReplay(1) // 結果をキャッシュ（1つの値を保持）
);

// 複数箇所で使用してもリクエストは1回のみ
cachedData$.subscribe(config => console.log('設定1:', config));
cachedData$.subscribe(config => console.log('設定2:', config));
```

## 次のステップ

各Creation Functionの詳細な動作と実践例を学ぶには、上記の表からリンクをクリックしてください。

また、[基本作成系 Creation Functions](/guide/creation-functions/basic/)、[ループ生成系 Creation Functions](/guide/creation-functions/loop/)、[結合系 Creation Functions](/guide/creation-functions/combination/)、[選択・分割系 Creation Functions](/guide/creation-functions/selection/)、[条件分岐系 Creation Functions](/guide/creation-functions/conditional/)も併せて学習することで、Creation Functionsの全体像を理解できます。
