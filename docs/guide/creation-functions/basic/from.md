---
description: from() - 配列・Promise・イテラブルなどからObservableを生成するCreation Function。既存のデータ構造を簡単にストリーム化できます。
---

# from() - 配列・Promise等から変換

`from()`は、配列・Promise・イテラブル・Observable-likeなオブジェクトから、Observableを生成するCreation Functionです。

## 概要

`from()`は、既存のデータ構造（配列、Promise、イテラブル等）をObservableストリームに変換します。特に、非同期処理（Promise）をRxJSの世界に統合する際に頻繁に使用されます。

**シグネチャ**:
```typescript
function from<T>(input: ObservableInput<T>, scheduler?: SchedulerLike): Observable<T>
```

**公式ドキュメント**: [📘 RxJS公式: from()](https://rxjs.dev/api/index/function/from)

## 基本的な使い方

`from()`は、様々な入力タイプを受け付けます。

```typescript
import { from } from 'rxjs';

// 配列から作成
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('配列値:', value),
  complete: () => console.log('配列完了')
});

// Promiseから作成
const promise$ = from(Promise.resolve('Promiseの結果'));
promise$.subscribe({
  next: value => console.log('Promise結果:', value),
  complete: () => console.log('Promise完了')
});

// イテラブルから作成
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('イテラブル値:', value),
  complete: () => console.log('イテラブル完了')
});

// 出力:
// 配列値: 1
// 配列値: 2
// 配列値: 3
// 配列完了
// イテラブル値: 1
// イテラブル値: 2
// イテラブル値: 3
// イテラブル完了
// Promise結果: Promiseの結果
// Promise完了
```

## 重要な特徴

### 1. 配列の各要素を個別に発行

`from()`は配列を受け取ると、その各要素を順番に個別に発行します。

```typescript
import { from } from 'rxjs';

from([10, 20, 30]).subscribe(value => console.log(value));

// 出力:
// 10
// 20
// 30
```

> [!IMPORTANT]
> **`of()`との違い**:
> - `of([1, 2, 3])` → 配列そのものを1つの値として発行
> - `from([1, 2, 3])` → 各要素 `1`, `2`, `3` を個別に発行

### 2. Promiseを自動的に処理

Promiseを渡すと、解決された値を発行し、即座に完了します。

```typescript
import { from } from 'rxjs';

const fetchData = (): Promise<string> => {
  return new Promise(resolve => {
    setTimeout(() => resolve('データ取得完了'), 1000);
  });
};

from(fetchData()).subscribe({
  next: value => console.log(value),
  complete: () => console.log('完了')
});

// 1秒後に出力:
// データ取得完了
// 完了
```

> [!WARNING]
> Promiseがrejectされた場合、Observableはエラーを発行します。
> ```typescript
> import { from } from "rxjs";
> from(Promise.reject('エラー')).subscribe({
>   error: err => console.error('エラー発生:', err)
> });
> ```

### 3. イテラブルをサポート

配列以外にも、`Set`、`Map`、`Generator`など、イテラブルなオブジェクトをサポートします。

```typescript
import { from } from 'rxjs';

// Set
from(new Set(['A', 'B', 'C'])).subscribe(console.log);
// 出力: A, B, C

// Map（キーと値のペア）
from(new Map([['key1', 'value1'], ['key2', 'value2']])).subscribe(console.log);
// 出力: ['key1', 'value1'], ['key2', 'value2']

// Generator
function* numberGenerator() {
  yield 1;
  yield 2;
  yield 3;
}
from(numberGenerator()).subscribe(console.log);
// 出力: 1, 2, 3
```

### 4. Cold Observable

`from()`は**Cold Observable**です。購読するたびに独立した実行が開始されます。

```typescript
import { from } from 'rxjs';

const numbers$ = from([1, 2, 3]);

numbers$.subscribe(val => console.log('購読者A:', val));
numbers$.subscribe(val => console.log('購読者B:', val));

// 各購読者が独立して配列を処理
// 出力:
// 購読者A: 1
// 購読者A: 2
// 購読者A: 3
// 購読者B: 1
// 購読者B: 2
// 購読者B: 3
```

> [!NOTE]
> **Cold Observableの特徴**
> - 購読するたびに独立した実行が開始されます
> - 各購読者は独自のデータストリームを受け取ります
> - Promiseの場合も購読ごとに評価されます
>
> 詳しくは [コールドObservableとホットObservable](/guide/observables/cold-and-hot-observables) を参照してください。

## from() vs of() の違い

両者の最も重要な違いは、配列の扱い方です。

```typescript
import { from, of } from 'rxjs';

const array = [1, 2, 3];

// of() - 配列を1つの値として発行
of(array).subscribe(value => {
  console.log('of():', value); // [1, 2, 3]
});

// from() - 配列の各要素を個別に発行
from(array).subscribe(value => {
  console.log('from():', value); // 1, 2, 3
});
```

| Creation Function | 配列の扱い | 用途 |
|-------------------|-----------|------|
| `of([1, 2, 3])` | 配列そのものを発行 | 配列をデータとして扱いたい |
| `from([1, 2, 3])` | 各要素を個別に発行 | 配列の要素を1つずつ処理したい |

## 実践的なユースケース

### 1. API呼び出しをストリーム化

Fetch APIやaxiosなどのPromiseベースのHTTPクライアントをストリーム化します。

```typescript
import { from, Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
}

function fetchUser(id: number): Observable<User> {
  return from(
    fetch(`https://jsonplaceholder.typicode.com/users/${id}`)
      .then(response => response.json())
  ).pipe(
    catchError(error => {
      console.error('API Error:', error);
      return of({ id: 0, name: 'Unknown', email: '' });
    })
  );
}

fetchUser(1).subscribe(user => console.log('User:', user));
```

### 2. 配列の要素を順次処理

配列の各要素に対して、順次非同期処理を実行します。

```typescript
import { from } from 'rxjs';
import { concatMap, delay } from 'rxjs';

const urls = [
  'https://jsonplaceholder.typicode.com/posts/1',
  'https://jsonplaceholder.typicode.com/posts/2',
  'https://jsonplaceholder.typicode.com/posts/3'
];

from(urls).pipe(
  concatMap(url =>
    from(fetch(url).then(res => res.json())).pipe(
      delay(500) // レート制限対策
    )
  )
).subscribe(data => console.log('取得:', data));
```

### 3. 非同期イテレータの処理

非同期イテレータ（async generator）もサポートしています。

```typescript
import { from } from 'rxjs';

async function* asyncGenerator() {
  yield await Promise.resolve(1);
  yield await Promise.resolve(2);
  yield await Promise.resolve(3);
}

from(asyncGenerator()).subscribe(value => console.log(value));
// 出力: 1, 2, 3
```

### 4. イベントエミッターの統合

Node.jsのEventEmitterやカスタムイベントシステムをストリーム化します。

```typescript
import { from } from 'rxjs';

// イテラブルなカスタムオブジェクト
class DataSource {
  *[Symbol.iterator]() {
    yield 'データA';
    yield 'データB';
    yield 'データC';
  }
}

from(new DataSource()).subscribe(console.log);
// 出力: データA, データB, データC
```

## パイプラインでの使用

`from()`は、既存のデータをパイプライン処理の起点として使用する際に便利です。

```typescript
import { from } from 'rxjs';
import { map, filter, reduce } from 'rxjs';

interface Product {
  id: number;
  name: string;
  price: number;
}

const products: Product[] = [
  { id: 1, name: '商品A', price: 1000 },
  { id: 2, name: '商品B', price: 2000 },
  { id: 3, name: '商品C', price: 500 }
];

from(products).pipe(
  filter(product => product.price >= 1000),
  map(product => product.price),
  reduce((sum, price) => sum + price, 0)
).subscribe(total => console.log('合計金額:', total));
// 出力: 合計金額: 3000
```

## よくある間違い

### 1. Promiseの実行タイミングを誤解

```typescript
// ❌ 間違い - Promiseは作成時点で実行される
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1'); // すでに実行開始
from(promise).subscribe(console.log); // 購読時点ではない

// ✅ 正しい - 購読時に実行したい場合はdefer()を使う
import { defer, from } from 'rxjs';

const deferred$ = defer(() =>
  from(fetch('https://jsonplaceholder.typicode.com/posts/1'))
);
deferred$.subscribe(console.log); // 購読時に実行
```

> [!WARNING]
> **Promiseは遅延評価されない**
>
> Promiseは作成された時点で実行開始します。`from(promise)`は既に実行中のPromiseをラップするだけです。購読時に実行したい場合は、`defer(() => from(promise))`を使用してください。

### 2. 配列とof()を混同

```typescript
import { from, map, of } from "rxjs";

// ❌ 意図と異なる - 配列全体が発行される
of([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// 出力: [1, 2, 3]（配列そのもの）

// ✅ 正しい - 各要素を個別に処理
from([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// 出力: 2, 4, 6
```

## パフォーマンスの考慮事項

`from()`は入力タイプによってパフォーマンスが異なります。

> [!TIP]
> **最適化のヒント**:
> - 大量のデータ（数千〜数万要素）を処理する場合、`concatMap`や`mergeMap`と組み合わせる際は並行数を制限しましょう。
> - Promiseの配列を処理する場合は、`forkJoin`や`combineLatest`の使用も検討してください。

```typescript
import { from } from 'rxjs';
import { mergeMap } from 'rxjs';

const urls = [...Array(100)].map((_, i) => `https://jsonplaceholder.typicode.com/posts/${i + 1}`);

from(urls).pipe(
  mergeMap(
    url => from(fetch(url).then(res => res.json())),
    5 // 並行実行数を5に制限
  )
).subscribe(data => console.log(data));
```

## 関連するCreation Functions

| Function | 違い | 使い分け |
|----------|------|----------|
| **[of()](/guide/creation-functions/basic/of)** | 引数を順番に発行 | 値をそのまま発行したい |
| **[fromEvent()](/guide/creation-functions/basic/fromEvent)** | イベントをストリーム化 | DOMイベントやEventEmitterを扱う |
| **[defer()](/guide/creation-functions/conditional/defer)** | 購読時に生成を遅延 | Promiseの遅延実行が必要 |
| **ajax()** | HTTP通信専用 | RxJS内でHTTPリクエストを完結させたい |

## まとめ

- `from()`は配列・Promise・イテラブルからObservableを生成
- 配列の各要素を個別に発行（`of()`と異なる）
- Promiseを自動的に処理し、結果を発行
- 非同期処理をRxJSの世界に統合する際に最適
- Promiseは作成時点で実行されることに注意（遅延実行には`defer()`を使用）

## 次のステップ

- [fromEvent() - イベントをObservableに変換](/guide/creation-functions/basic/fromEvent)
- [defer() - 購読時に生成を遅延](/guide/creation-functions/conditional/defer)
- [基本作成系の概要に戻る](/guide/creation-functions/basic/)
