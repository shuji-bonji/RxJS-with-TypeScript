---
description: PromiseとRxJSの違いを理解し、適切な使い分けを学びます。Promiseは単一の非同期処理に特化し即座に実行されますが、RxJSは複数の値を扱えるLazy評価でキャンセルや再利用が可能です。コード比較と具体的なユースケースを通じて、それぞれの特性と選択基準を詳しく解説します。
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

## コード比較： 単一の非同期処理

### Promise

```ts
// Promiseは作成時に即実行される（Eager）
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json())
  .then(data => console.log(data))
  .catch(error => console.error(error));
```

Promiseは**定義した瞬間に実行が始まります**（Eager評価）。

### RxJS

```ts
import { from } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

// Observableは購読するまで実行されない（Lazy）
const observable$ = from(fetch('https://jsonplaceholder.typicode.com/posts/1')).pipe(
  switchMap(response => response.json()), // response.json()はPromiseを返すのでswitchMapを使用
  catchError(error => {
    console.error(error);
    return of(null);
  })
);

// 購読して初めて実行される
observable$.subscribe(data => console.log(data));
```

RxJSは **`subscribe()` が呼ばれるまで実行されません** （Lazy評価）。同じObservableを複数回購読すると独立した実行が行われ、`unsubscribe()` で処理を中断できます。

> [!TIP]
> **実務での使い分け**
> - 即座に実行したい単発の処理 → Promise
> - 必要なタイミングで実行したい、または複数回実行したい処理 → RxJS

## コード比較： 複数の値を扱う場合

PromiseとRxJSの最も大きな違いの一つが、発行できる値の数です。Promiseは単一の値しか返せませんが、RxJSは複数の値を時系列で発行できます。

### Promiseでは不可能

Promiseは**一度しか解決できません**。

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

最初の `resolve()` で値が確定すると、それ以降の `resolve()` は無視されます。

### RxJSでは可能

Observableは**何度でも値を発行できます**。
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

`next()` を呼ぶたびに、購読者に値が届きます。すべての値を発行した後は `complete()` で完了を通知します。この特性により、リアルタイム通信、ストリーミングデータ、連続的なイベント処理など、時系列で変化するデータを自然に扱えます。

> [!NOTE]
> **実務での応用例**
> - WebSocketのメッセージ受信
> - キーボード入力の逐次処理
> - サーバーからのイベントストリーム（SSE）
> - センサーデータの継続的な監視

## キャンセルの比較

長時間かかる処理や、不要になった非同期処理をキャンセルできるかどうかは、リソース管理とユーザー体験の観点で重要です。PromiseとRxJSでは、キャンセル機能に大きな違いがあります。

### Promise（キャンセル不可）
Promiseには**標準的なキャンセル機能がありません**。

```ts
const promise = new Promise(resolve => {
  setTimeout(() => resolve('完了'), 3000);
});

promise.then(result => console.log(result));
// この処理をキャンセルする標準的な方法はない
```

一度実行が始まると完了するまで止められず、メモリリークやパフォーマンス低下の原因になります。

> [!WARNING]
> **AbortController について**
> `fetch()` などのWeb APIは `AbortController` を使ってキャンセルできますが、これはPromise自体の機能ではなく、個別のAPIが提供する仕組みです。すべての非同期処理で使えるわけではありません。

### RxJS（キャンセル可能）

RxJSは **`unsubscribe()` でいつでもキャンセルできます**。
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

購読を解除すると進行中の処理が即座に停止し、メモリリークを防げます。

> [!TIP]
> **実務でのキャンセル活用例**
> - ユーザーが画面を離れたときにHTTPリクエストをキャンセル
> - 古い検索クエリの結果を破棄して、最新のクエリだけ処理（`switchMap`）
> - コンポーネント破棄時に、すべてのObservableを自動的にキャンセル（`takeUntil`パターン）

## どちらを選ぶべきか

PromiseとRxJSのどちらを使うべきかは、処理の性質とプロジェクトの要件によって変わります。以下の基準を参考に、適切なツールを選択しましょう。

### Promiseを選ぶべき場合

以下の条件に当てはまる場合は、Promiseが適しています。

| 条件 | 理由 |
|------|------|
| 単一の非同期処理 | APIリクエスト1回、ファイル読み込み1回など |
| シンプルなワークフロー | `Promise.all`, `Promise.race`で十分 |
| 小規模プロジェクト | 依存関係を最小限にしたい |
| 標準APIのみ使用 | 外部ライブラリを避けたい |
| 初心者向けコード | 学習コストを抑えたい |

#### 単一のAPIリクエスト:


```ts
interface User {
  id: number;
  name: string;
  email: string;
  username: string;
}

async function getUserData(userId: string): Promise<User> {
  const response = await fetch(`https://jsonplaceholder.typicode.com/users/${userId}`);
  if (!response.ok) {
    throw new Error('ユーザーデータの取得に失敗しました');
  }
  return response.json();
}

// 使用例
getUserData('1').then(user => {
  console.log('ユーザー名:', user.name);
  console.log('メール:', user.email);
});
```

このコードは、単一のユーザー情報を取得する典型的なパターンです。`async/await` を使うことで、同期的なコードのように読みやすく書けます。エラーハンドリングも `try/catch` で統一でき、シンプルで直感的です。

#### 複数の非同期処理を並列実行:

```ts
interface Post {
  id: number;
  userId: number;
  title: string;
  body: string;
}

async function loadAllData(): Promise<[User[], Post[]]> {
  const [users, posts] = await Promise.all([
    fetch('https://jsonplaceholder.typicode.com/users').then(r => r.json()),
    fetch('https://jsonplaceholder.typicode.com/posts').then(r => r.json())
  ]);
  return [users, posts];
}

// 使用例
loadAllData().then(([users, posts]) => {
  console.log('ユーザー数:', users.length);
  console.log('投稿数:', posts.length);
});
```

`Promise.all()` を使うことで、複数のAPIリクエストを並列に実行し、すべてが完了するのを待つことができます。これは初期データ読み込みなどで非常に便利です。一つでも失敗すると全体がエラーになる点に注意が必要ですが、そのシンプルさゆえに理解しやすく、メンテナンスも容易です。

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

const label = document.createElement('label');
label.innerText = 'search: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);

// リアルタイム検索（オートコンプリート）
if (!searchInput) throw new Error('検索入力欄が見つかりません');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),              // 300ms待ってから処理
  distinctUntilChanged(),         // 値が変わった時だけ処理
  switchMap(query =>              // 最新のリクエストのみ実行
    fetch(`https://api.github.com/search/users?q=${query}`).then(r => r.json())
  )
).subscribe(results => {
  console.log('検索結果:', results.items); // GitHub APIはitemsプロパティに結果を格納
});
```

この例は、RxJSの真価が発揮される典型的なケースです。ユーザーの入力を監視し、300msの待機時間を設けて無駄なリクエストを減らし、値が変わったときだけ処理を行い、さらに最新のリクエストだけを有効にする（`switchMap`）ことで、古いリクエストの結果を自動的に破棄します。

> [!IMPORTANT]
> **Promiseだけでは困難な理由**
> - debounce（連続入力の制御）を手動実装する必要がある
> - 古いリクエストのキャンセルを自分で管理しなければならない
> - イベントリスナーのクリーンアップを忘れるとメモリリークが発生する
> - 複数の状態（タイマー、フラグ、リクエスト管理）を同時に追跡する必要がある
>
> RxJSでは、これらがすべて宣言的に、数行で実現できます。

## PromiseとRxJSの相互運用

PromiseとRxJSは排他的なものではなく、相互に変換して組み合わせることができます。既存のPromiseベースのコードをRxJSのパイプラインに統合したり、逆にObservableを既存のPromiseベースのコードで使いたい場合に便利です。

### PromiseをObservableに変換

既存のPromiseベースのAPIや関数を、RxJSのパイプラインで使いたい場合は、`from()` を使って変換します。

```ts
import { from } from 'rxjs';

// Promiseを作成
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json());

// from()でObservableに変換
const observable$ = from(promise);

observable$.subscribe({
  next: data => console.log('データ:', data),
  error: error => console.error('エラー:', error),
  complete: () => console.log('完了')
});
```

`from()` は、Promiseが解決すると1つの値を発行し、即座に `complete` します。エラーが発生すると `error` 通知が送られます。この変換により、Promise由来のデータに対しても、RxJSのオペレーター（`map`, `filter`, `retry` など）を自由に適用できるようになります。

### ObservableをPromiseに変換

逆に、ObservableをPromiseに変換したい場合もあります。例えば、async/awaitで書かれた既存コードにRxJSの処理を統合する場合や、単一の値だけが必要な場合に便利です。

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

> [!TIP]
> **使い分けのポイント**
> - **`firstValueFrom()`**: 最初の値だけが必要な場合（例：ログイン認証の結果）
> - **`lastValueFrom()`**: すべてのデータを処理した後の最終結果が必要な場合（例：集計結果）

## 実践例：両者を組み合わせる

実際のアプリケーションでは、PromiseとRxJSを組み合わせて使用することが一般的です。

> [!WARNING] 実務での注意事項
> PromiseとObservableの混在は、**設計の境界を明確にしないとアンチパターンに陥りやすい**です。
>
> **よくある問題：**
> - キャンセル不能になる
> - エラーハンドリングの分離
> - `subscribe`内での`await`（特に危険）
> - 同じデータを Promise と Observable で並行取得
>
> 詳しくは **[Chapter 10: PromiseとObservableの混在アンチパターン](/guide/anti-patterns/promise-observable-mixing)** を参照してください。

```ts
import { fromEvent, from } from 'rxjs';
import { debounceTime, switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface SearchResult {
  items: Array<{
    login: string;
    id: number;
    avatar_url: string;
  }>;
  total_count: number;
}

// Promise ベースのAPI関数
async function searchAPI(query: string): Promise<SearchResult> {
  const response = await fetch(`https://api.github.com/search/users?q=${query}`);
  if (!response.ok) {
    throw new Error('検索に失敗しました');
  }
  return response.json();
}

// RxJSでイベントストリームを管理
const label = document.createElement('label');
label.innerText = 'search: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);
if (!searchInput) throw new Error('検索入力欄が見つかりません');

fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  switchMap(event => {
    const query = (event.target as HTMLInputElement).value;
    // Promise関数をObservableに変換
    return from(searchAPI(query));
  }),
  catchError(error => {
    console.error(error);
    return of({ items: [], total_count: 0 }); // エラー時は空の結果を返す
  })
).subscribe(result => {
  console.log('検索結果:', result.items);
  console.log('合計:', result.total_count);
});
```

> [!TIP]
> **責務の分離による設計**
>
> - **RxJS**: イベント制御を担当（debounce、switchMapなど）
> - **Promise**: HTTPリクエストを担当（async/await）
> - **`from()`**: 両者を橋渡し
>
> 各技術を適材適所で使い分けることで、コードの可読性と保守性が向上します。

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

| 目的 | 推奨 | 理由 |
|------|------|------|
| 単一のHTTPリクエスト | Promise（`async/await`） | シンプルで読みやすく、標準API |
| ユーザー入力イベントの処理 | RxJS | debounce、distinctなどの制御が必要 |
| リアルタイムデータ（WebSocket） | RxJS | 連続的なメッセージを自然に扱える |
| 複数の非同期処理の並列実行 | Promise（`Promise.all`） | 単純な並列実行ならPromiseで十分 |
| 連続的なイベントストリーム | RxJS | 複数の値を時系列で扱える |
| キャンセル可能な処理 | RxJS | unsubscribe()で確実にキャンセル |
| シンプルなアプリケーション | Promise | 学習コストが低く、依存関係が少ない |
| Angularアプリケーション | RxJS | フレームワークに標準統合されている |

### 基本方針
- **シンプルに済むならPromise**を使う
- **複雑なストリーム処理が必要ならRxJS**を使う
- **両方を組み合わせる**のも有効（`from()`で橋渡し）

RxJSは強力ですが、すべての非同期処理にRxJSを使う必要はありません。適切なツールを適切な場面で使い分けることが重要です。
