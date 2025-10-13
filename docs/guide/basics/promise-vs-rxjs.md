---
description: PromiseとRxJSの違いを理解し、適切な使い分けを学びます。単一の非同期処理にはPromise、複雑なストリーム処理にはRxJSが適しています。
---

# PromiseとRxJSの違い

## 概要

JavaScript/TypeScriptにおける非同期処理を扱う主要なツールとして、 **Promise**と**RxJS（Observable）** があります。両者は似た目的で使用されることがありますが、設計思想とユースケースが大きく異なります。

このページでは、PromiseとRxJSの違いを理解し、どちらを使うべきかを判断するための情報を提供します。

## 基本的な違い

| 項目 | Promise | RxJS (Observable) |
|------|---------|-------------------|
| **標準化** | JavaScript標準（ES6/ES2015） | サードパーティライブラリ |
| **発行する値** | 単一の値 | 0個以上の複数の値 |
| **評価** | Eager（作成時に即実行） | Lazy（購読時に実行） |
| **キャンセル** | 不可[^1] | 可（`unsubscribe()`） |
| **再利用** | 不可（結果は1度だけ） | 可（何度でも購読可能） |
| **学習コスト** | 低い | 高い（オペレーターの理解が必要） |
| **ユースケース** | 単一の非同期処理 | 複雑なストリーム処理 |

[^1]: AbortControllerを使えばPromiseベースの処理（fetchなど）のキャンセルは可能ですが、Promise自体の仕様にキャンセル機能はありません。

## コード比較

### 単一の非同期処理

#### Promise
```ts
// Promiseは作成時に即実行される（Eager）
const promise = fetch('https://api.example.com/data')
  .then(response => response.json())
  .then(data => console.log(data))
  .catch(error => console.error(error));
```

#### RxJS
```ts
import { from } from 'rxjs';
import { map, catchError } from 'rxjs';
import { of } from 'rxjs';

// Observableは購読するまで実行されない（Lazy）
const observable$ = from(fetch('https://api.example.com/data')).pipe(
  map(response => response.json()),
  catchError(error => {
    console.error(error);
    return of(null);
  })
);

// 購読して初めて実行される
observable$.subscribe(data => console.log(data));
```

### 複数の値を扱う場合

#### Promiseでは不可能
```ts
// Promiseは単一の値しか返せない
const promise = new Promise(resolve => {
  resolve(1);
  resolve(2); // この値は無視される
  resolve(3); // この値も無視される
});

promise.then(value => console.log(value));
// 出力: 1（最初の値のみ）
```

#### RxJSでは可能
```ts
import { Observable } from 'rxjs';

// Observableは複数の値を発行できる
const observable$ = new Observable(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  subscriber.complete();
});

observable$.subscribe(value => console.log(value));
// 出力: 1, 2, 3
```

## キャンセルの比較

### Promise（キャンセル不可）

```ts
const promise = new Promise(resolve => {
  setTimeout(() => resolve('完了'), 3000);
});

promise.then(result => console.log(result));
// この処理をキャンセルする標準的な方法はない
```

### RxJS（キャンセル可能）

```ts
import { timer } from 'rxjs';

const subscription = timer(3000).subscribe(
  () => console.log('完了')
);

// 1秒後にキャンセル
setTimeout(() => {
  subscription.unsubscribe(); // キャンセル
  console.log('キャンセルしました');
}, 1000);
// 出力: キャンセルしました（「完了」は出力されない）
```

## どちらを選ぶべきか

### Promiseを選ぶべき場合

以下の条件に当てはまる場合は、Promiseが適しています。

| 条件 | 理由 |
|------|------|
| 単一の非同期処理 | APIリクエスト1回、ファイル読み込み1回など |
| シンプルなワークフロー | `Promise.all`, `Promise.race`で十分 |
| 小規模プロジェクト | 依存関係を最小限にしたい |
| 標準APIのみ使用 | 外部ライブラリを避けたい |
| 初心者向けコード | 学習コストを抑えたい |

#### 具体例
```ts
// 単一のAPIリクエスト
async function getUserData(userId: string): Promise<User> {
  const response = await fetch(`/api/users/${userId}`);
  if (!response.ok) {
    throw new Error('ユーザーデータの取得に失敗しました');
  }
  return response.json();
}

// 複数の非同期処理を並列実行
async function loadAllData(): Promise<[User[], Post[]]> {
  const [users, posts] = await Promise.all([
    fetch('/api/users').then(r => r.json()),
    fetch('/api/posts').then(r => r.json())
  ]);
  return [users, posts];
}
```

### RxJSを選ぶべき場合

以下の条件に当てはまる場合は、RxJSが適しています。

| 条件 | 理由 |
|------|------|
| 連続的なイベント処理 | マウス移動、キーボード入力、WebSocketなど |
| 複雑なストリーム処理 | 複数のイベントソースの結合や変換 |
| キャンセルが必要 | リソース管理を細かく制御したい |
| リトライ・タイムアウト | エラー処理を柔軟に行いたい |
| Angularプロジェクト | RxJSがフレームワークに統合されている |
| リアルタイムデータ | データが継続的に更新される |

#### 具体例
```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap } from 'rxjs';

// リアルタイム検索（オートコンプリート）
const searchInput = document.querySelector<HTMLInputElement>('#search') as HTMLInputElement;

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),              // 300ms待ってから処理
  distinctUntilChanged(),         // 値が変わった時だけ処理
  switchMap(query =>              // 最新のリクエストのみ実行
    fetch(`/api/search?q=${query}`).then(r => r.json())
  )
).subscribe(results => {
  console.log('検索結果:', results);
});
```

この処理をPromiseだけで実装するのは非常に困難です。

## PromiseとRxJSの相互運用

PromiseとRxJSは相互に変換できます。

### PromiseをObservableに変換

```ts
import { from } from 'rxjs';

// Promiseを作成
const promise = fetch('https://api.example.com/data')
  .then(response => response.json());

// from()でObservableに変換
const observable$ = from(promise);

observable$.subscribe({
  next: data => console.log('データ:', data),
  error: error => console.error('エラー:', error),
  complete: () => console.log('完了')
});
```

### ObservableをPromiseに変換

```ts
import { of, firstValueFrom, lastValueFrom } from 'rxjs';
import { delay } from 'rxjs';

const observable$ = of(1, 2, 3).pipe(delay(1000));

// 最初の値をPromiseとして取得
const firstValue = await firstValueFrom(observable$);
console.log(firstValue); // 1

// 最後の値をPromiseとして取得
const lastValue = await lastValueFrom(observable$);
console.log(lastValue); // 3
```

> [!WARNING]
> `toPromise()`は非推奨です。代わりに`firstValueFrom()`または`lastValueFrom()`を使用してください。

## 実践例：両者を組み合わせる

実際のアプリケーションでは、PromiseとRxJSを組み合わせて使用することが一般的です。

```ts
import { fromEvent, from } from 'rxjs';
import { debounceTime, switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface SearchResult {
  items: string[];
}

// Promise ベースのAPI関数
async function searchAPI(query: string): Promise<SearchResult> {
  const response = await fetch(`/api/search?q=${query}`);
  if (!response.ok) {
    throw new Error('検索に失敗しました');
  }
  return response.json();
}

// RxJSでイベントストリームを管理
const searchInput = document.querySelector<HTMLInputElement>('#search')!;

fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  switchMap(event => {
    const query = (event.target as HTMLInputElement).value;
    // Promise関数をObservableに変換
    return from(searchAPI(query));
  }),
  catchError(error => {
    console.error(error);
    return of({ items: [] }); // エラー時は空の結果を返す
  })
).subscribe(result => {
  console.log('検索結果:', result.items);
});
```
#### この例では
- RxJSがユーザー入力イベントの制御を担当（debounce、switchMapなど）
- Promise（async/await）がHTTPリクエストを担当
- `from()`で両者を橋渡し

## メリットとデメリット
### Promise
<div class="comparison-cards">

::: tip メリット
- JavaScript標準のため依存関係不要
- `async/await`により直感的で読みやすいコード
- 学習コストが低い
- 単一タスクの処理がシンプル
:::

::: danger デメリット
- 複数の値を扱えない
- キャンセル機能がない
- 連続的なストリーム処理には不向き
- 複雑なイベント処理が困難
:::

</div>

### RxJS
<div class="comparison-cards">

::: tip メリット
- 複数の値を時系列で扱える
- 豊富なオペレーターで複雑な処理が可能
- キャンセル（`unsubscribe`）が簡単
- エラー処理やリトライを柔軟に実装可能
- 宣言的でテストしやすい
:::

::: danger デメリット
- 学習コストが高い
- ライブラリへの依存が必要
- オーバーヘッドがある（小規模プロジェクトでは過剰）
- デバッグが難しい場合がある
:::

</div>

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>

## RxJSが特に活躍する分野

RxJSは以下のような分野で特に強力です。Promiseだけでは実現が困難な複雑な要件を、エレガントに解決できます。

| 分野 | 具体例 | Promiseとの比較 |
|------|--------|----------------|
| **リアルタイム通信** | WebSocket、SSE、チャット、株価更新 | Promiseは単発の通信のみ。連続的なメッセージ処理には不向き |
| **ユーザー入力制御** | 検索オートコンプリート、フォームバリデーション | debounce、distinctUntilChangedなどが標準装備 |
| **複数ソースの結合** | 検索条件×ソート順×フィルタの組み合わせ | combineLatest、withLatestFromで簡潔に記述可能 |
| **オフライン対応** | PWA、ネットワーク状態監視、自動再同期 | retry、retryWhenで柔軟なリトライ制御 |
| **ストリーミングAPI** | OpenAI、AI応答のトークン逐次出力 | 連続データをリアルタイムで処理可能 |
| **キャンセル制御** | 長時間処理の中断、古いリクエストの破棄 | unsubscribe()で即座にキャンセル可能 |

> [!NOTE]
> RxJSの活用分野の詳細は、[RxJSとは何か - ユースケース](./what-is-rxjs.md#ユースケース)も参照してください。

## まとめ

| 目的 | 推奨 |
|------|------|
| 単一のHTTPリクエスト | Promise（`async/await`） |
| ユーザー入力イベントの処理 | RxJS |
| リアルタイムデータ（WebSocket） | RxJS |
| 複数の非同期処理の並列実行 | Promise（`Promise.all`） |
| 連続的なイベントストリーム | RxJS |
| キャンセル可能な処理 | RxJS |
| シンプルなアプリケーション | Promise |
| Angularアプリケーション | RxJS |

### 基本方針
- **シンプルに済むならPromise**を使う
- **複雑なストリーム処理が必要ならRxJS**を使う
- **両方を組み合わせる**のも有効（`from()`で橋渡し）

RxJSは強力ですが、すべての非同期処理にRxJSを使う必要はありません。適切なツールを適切な場面で使い分けることが重要です。
