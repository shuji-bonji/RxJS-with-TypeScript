---
description: generate() - 汎用的なループ生成Creation Function。while文のような柔軟な条件制御とカスタム状態管理が可能です。
---

# generate() - 汎用的なループ生成

`generate()`は、初期状態、継続条件、状態更新、結果選択を指定して、柔軟なループ処理をObservableとして実現するCreation Functionです。

## 概要

`generate()`はwhile文やfor文のような柔軟なループ処理を宣言的に記述できます。`range()`よりも複雑な条件や状態管理が必要な場合に使用します。

**シグネチャ**:
```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

**パラメーター**:
- `initialState`: ループの初期状態
- `condition`: 継続条件を判定する関数（`false`でループ終了）
- `iterate`: 状態を次に進める関数（状態更新）
- `resultSelector`: 状態から発行する値を選択する関数（省略時は状態そのものを発行）
- `scheduler`: 値を発行するスケジューラー（省略時は同期的に発行）

**公式ドキュメント**: [📘 RxJS公式: generate()](https://rxjs.dev/api/index/function/generate)

## 基本的な使い方

### パターン1: シンプルなカウンター

最も基本的な使い方です。

```typescript
import { generate } from 'rxjs';

// 1から5までカウント
generate(
  1,              // 初期状態
  x => x <= 5,    // 継続条件
  x => x + 1      // 状態更新
).subscribe({
  next: value => console.log('値:', value),
  complete: () => console.log('完了')
});

// 出力:
// 値: 1
// 値: 2
// 値: 3
// 値: 4
// 値: 5
// 完了
```

このコードは以下のwhile文と等価です。

```typescript
let x = 1;
while (x <= 5) {
  console.log('値:', x);
  x = x + 1;
}
console.log('完了');
```

### パターン2: resultSelectorで値を変換

状態と発行する値を分離できます。

```typescript
import { generate } from 'rxjs';

// 内部状態はカウンターだが、発行するのは2乗した値
generate(
  1,              // 初期状態: 1
  x => x <= 5,    // 継続条件: x <= 5
  x => x + 1,     // 状態更新: x + 1
  x => x * x      // 結果選択: x^2を発行
).subscribe(console.log);

// 出力: 1, 4, 9, 16, 25
```

### パターン3: 複雑な状態オブジェクト

状態として複雑なオブジェクトを使用できます。

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

// 累積和を計算
generate<number, State>(
  { count: 1, sum: 0 },           // 初期状態
  state => state.count <= 5,      // 継続条件
  state => ({                     // 状態更新
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => state.sum              // 結果選択
).subscribe(console.log);

// 出力: 0, 1, 3, 6, 10
// (0, 0+1, 0+1+2, 0+1+2+3, 0+1+2+3+4)
```

## 重要な特徴

### 1. while文的な動作

`generate()`はwhile文のような柔軟な制御が可能です。

```typescript
import { generate } from "rxjs";

// while文
let i = 1;
while (i <= 10) {
  console.log(i);
  i = i * 2;
}

// generate()で同じことを実現
generate(
  1,              // let i = 1;
  i => i <= 10,   // while (i <= 10)
  i => i * 2      // i = i * 2;
).subscribe(console.log);

// 出力: 1, 2, 4, 8
```

### 2. 同期的に発行

デフォルトでは購読と同時に、すべての値を**同期的に**発行します。

```typescript
import { generate } from 'rxjs';

console.log('購読前');

generate(1, x => x <= 3, x => x + 1).subscribe(val => console.log('値:', val));

console.log('購読後');

// 出力:
// 購読前
// 値: 1
// 値: 2
// 値: 3
// 購読後
```

### 3. 無限ループに注意

条件が常に`true`の場合、無限ループになります。

```typescript
import { generate } from 'rxjs';
import { take } from 'rxjs/operators';

// ❌ 危険: 無限ループ（ブラウザがフリーズ）
// generate(0, x => true, x => x + 1).subscribe(console.log);

// ✅ 安全: take()で個数を制限
generate(
  0,
  x => true,  // 常にtrue
  x => x + 1
).pipe(
  take(10)    // 最初の10個だけ取得
).subscribe(console.log);

// 出力: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

> [!WARNING]
> **無限ループに注意**:
> - 条件が常に`true`になる場合、無限ループが発生
> - `take()`, `takeWhile()`, `takeUntil()`で発行数を制限
> - または条件関数で適切な終了条件を設定

## 実践的なユースケース

### 1. フィボナッチ数列

複雑な状態遷移の例です。

```typescript
import { generate } from 'rxjs';
import { take } from 'rxjs/operators';

interface FibState {
  current: number;
  next: number;
}

// フィボナッチ数列の最初の10項
generate<number, FibState>(
  { current: 0, next: 1 },           // 初期状態: F(0)=0, F(1)=1
  state => true,                     // 無限に生成
  state => ({                        // 状態更新
    current: state.next,
    next: state.current + state.next
  }),
  state => state.current             // 現在の値を発行
).pipe(
  take(10)                           // 最初の10項
).subscribe(console.log);

// 出力: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### 2. 指数バックオフ

リトライ処理で使用する指数的な待機時間の生成です。

```typescript
import { generate } from 'rxjs';

interface RetryState {
  attempt: number;
  delay: number;
}

// 指数バックオフの遅延時間を生成（1秒、2秒、4秒、8秒、16秒）
generate<number, RetryState>(
  { attempt: 0, delay: 1000 },       // 初期状態: 1秒
  state => state.attempt < 5,        // 最大5回
  state => ({                        // 状態更新
    attempt: state.attempt + 1,
    delay: state.delay * 2           // 遅延時間を2倍に
  }),
  state => state.delay               // 遅延時間を発行
).subscribe(delay => {
  console.log(`リトライ ${delay / 1000}秒後`);
});

// 出力:
// リトライ 1秒後
// リトライ 2秒後
// リトライ 4秒後
// リトライ 8秒後
// リトライ 16秒後
```

### 3. ページネーションの制御

次のページが存在する限り取得を続けます。

```typescript
import { generate, of, Observable } from 'rxjs';
import { concatMap, delay } from 'rxjs/operators';

interface PageState {
  page: number;
  hasNext: boolean;
}

interface PageData {
  page: number;
  items: string[];
  hasNext: boolean;
}

// ページデータ取得をシミュレートする関数
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`アイテム${page}-1`, `アイテム${page}-2`, `アイテム${page}-3`],
    hasNext: page < 10 // 10ページまで
  }).pipe(
    delay(500) // API呼び出しをシミュレート
  );
}

// ページが存在する限り取得（実際にはAPIレスポンスからhasNextを取得）
generate<number, PageState>(
  { page: 1, hasNext: true },        // 初期状態
  state => state.hasNext,            // 次ページがある限り継続
  state => ({                        // 状態更新
    page: state.page + 1,
    hasNext: state.page < 10         // 仮に10ページまでとする
  }),
  state => state.page                // ページ番号を発行
).pipe(
  concatMap(page => fetchPage(page)) // 各ページを順次取得
).subscribe(
  data => console.log(`ページ ${data.page} 取得:`, data.items),
  err => console.error('エラー:', err),
  () => console.log('全ページ取得完了')
);

// 出力:
// ページ 1 取得: ['アイテム1-1', 'アイテム1-2', 'アイテム1-3']
// ページ 2 取得: ['アイテム2-1', 'アイテム2-2', 'アイテム2-3']
// ...
// ページ 10 取得: ['アイテム10-1', 'アイテム10-2', 'アイテム10-3']
// 全ページ取得完了
```

### 4. カスタムタイマー

不規則な間隔でイベントを発行します。

```typescript
import { generate, of } from 'rxjs';
import { concatMap, delay } from 'rxjs/operators';

interface TimerState {
  count: number;
  delay: number;
}

// 遅延時間が徐々に増加するタイマー
generate<string, TimerState>(
  { count: 0, delay: 1000 },         // 初期状態: 1秒
  state => state.count < 5,          // 5回まで
  state => ({                        // 状態更新
    count: state.count + 1,
    delay: state.delay + 500         // 遅延を500ms増加
  }),
  state => `イベント${state.count + 1}`
).pipe(
  concatMap((message, index) => {
    const delayTime = 1000 + index * 500;
    console.log(`${delayTime}ms待機後に発行`);
    return of(message).pipe(delay(delayTime));
  })
).subscribe(console.log);

// 出力:
// 1000ms待機後に発行
// イベント1 (1秒後)
// 1500ms待機後に発行
// イベント2 (2.5秒後)
// 2000ms待機後に発行
// イベント3 (4.5秒後)
// ...
```

### 5. 階乗の計算

数学的な計算をストリームとして表現します。

```typescript
import { generate } from 'rxjs';

interface FactorialState {
  n: number;
  result: number;
}

// 5の階乗を計算 (5! = 5 × 4 × 3 × 2 × 1 = 120)
generate<number, FactorialState>(
  { n: 5, result: 1 },               // 初期状態
  state => state.n > 0,              // n > 0の間継続
  state => ({                        // 状態更新
    n: state.n - 1,
    result: state.result * state.n
  }),
  state => state.result              // 中間結果を発行
).subscribe(console.log);

// 出力: 5, 20, 60, 120, 120
// (1*5, 5*4, 20*3, 60*2, 120*1)
```

## 他の Creation Functions との比較

### generate() vs range()

```typescript
import { generate, range } from 'rxjs';

// range() - シンプルな連番
range(1, 5).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5

// generate() - 同じことをより明示的に
generate(
  1,
  x => x <= 5,
  x => x + 1
).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5

// generate()の真価: 複雑なステップ
generate(
  1,
  x => x <= 100,
  x => x * 2  // 2倍ずつ増加
).subscribe(console.log);
// 出力: 1, 2, 4, 8, 16, 32, 64
// (range()では不可能)
```

### generate() vs defer()

```typescript
import { generate, defer, of } from 'rxjs';

// generate() - ループ処理
generate(1, x => x <= 3, x => x + 1).subscribe(console.log);
// 出力: 1, 2, 3

// defer() - 購読時に生成（ループではない）
defer(() => of(1, 2, 3)).subscribe(console.log);
// 出力: 1, 2, 3

// 違い: generate()は状態を持ち、deferは遅延評価のみ
```

> [!TIP]
> **選択基準**:
> - **シンプルな連番** → `range()`
> - **複雑な条件やステップ** → `generate()`
> - **購読時に動的に決定** → `defer()`
> - **フィボナッチ、階乗など** → `generate()`

## スケジューラーによる非同期化

大量のデータを処理する場合、スケジューラーを指定して非同期実行できます。

```typescript
import { generate, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs/operators';

console.log('開始');

// 100万回のループを非同期で実行
generate(
  1,
  x => x <= 1000000,
  x => x + 1
).pipe(
  observeOn(asyncScheduler)
).subscribe({
  next: val => {
    if (val % 100000 === 0) {
      console.log(`進捗: ${val}`);
    }
  },
  complete: () => console.log('完了')
});

console.log('購読後（非同期なので即座に実行される）');

// 出力:
// 開始
// 購読後（非同期なので即座に実行される）
// 進捗: 100000
// 進捗: 200000
// ...
// 完了
```

## パフォーマンスに関する注意

`generate()`は同期的に値を発行するため、大量の値を生成する場合や複雑な計算を行う場合は、パフォーマンスに注意が必要です。

> [!WARNING]
> **パフォーマンス最適化**:
> ```typescript
> // ❌ 悪い例: 複雑な計算を同期的に実行（UIがブロックされる）
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).subscribe(console.log);
>
> // ✅ 良い例1: スケジューラーで非同期化
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ 良い例2: take()で個数制限
> generate(
>   1,
>   x => true,  // 無限ループ
>   x => x + 1
> ).pipe(
>   take(100)   // 最初の100個だけ
> ).subscribe(console.log);
> ```

## エラーハンドリング

`generate()`自体はエラーを発行しませんが、パイプラインや状態更新関数でエラーが発生する可能性があります。

```typescript
import { generate } from 'rxjs';
import { map, catchError } from 'rxjs/operators';
import { of } from 'rxjs';

generate(
  1,
  x => x <= 10,
  x => x + 1
).pipe(
  map(n => {
    if (n === 5) {
      throw new Error('5でエラー');
    }
    return n * 2;
  }),
  catchError(error => {
    console.error('エラー発生:', error.message);
    return of(-1); // デフォルト値を返す
  })
).subscribe(console.log);

// 出力: 2, 4, 6, 8, -1
```

### 状態更新関数内でのエラー

状態更新関数内でエラーが発生すると、Observableがエラー状態になります。

```typescript
import { generate } from 'rxjs';
import { catchError } from 'rxjs/operators';
import { EMPTY } from 'rxjs';

generate(
  1,
  x => x <= 10,
  x => {
    if (x === 5) {
      throw new Error('状態更新でエラー');
    }
    return x + 1;
  }
).pipe(
  catchError(error => {
    console.error('エラー:', error.message);
    return EMPTY; // 空のObservableを返す
  })
).subscribe({
  next: console.log,
  complete: () => console.log('完了')
});

// 出力: 1, 2, 3, 4, エラー: 状態更新でエラー, 完了
```

## TypeScriptでの型安全性

`generate()`は状態の型と発行値の型を分離できます。

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

interface Result {
  index: number;
  average: number;
}

// 状態: State、発行値: Result
const stats$ = generate<Result, State>(
  { count: 1, sum: 0 },
  state => state.count <= 5,
  state => ({
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => ({
    index: state.count,
    average: state.sum / state.count
  })
);

stats$.subscribe(result => {
  console.log(`[${result.index}] 平均: ${result.average}`);
});

// 出力:
// [1] 平均: 0
// [2] 平均: 0.5
// [3] 平均: 1
// [4] 平均: 1.5
// [5] 平均: 2
```

## まとめ

`generate()`は、複雑なループ処理を宣言的に記述できる強力なCreation Functionです。

> [!IMPORTANT]
> **generate()の特徴**:
> - ✅ while文/for文的な柔軟なループ制御
> - ✅ 複雑な状態管理が可能
> - ✅ フィボナッチ、階乗など数学的な計算に最適
> - ✅ 状態と発行値を分離できる
> - ⚠️ 無限ループに注意（`take()`で制限）
> - ⚠️ 大量データは非同期化を検討
> - ⚠️ シンプルな連番は`range()`を使用

## 関連項目

- [range()](/guide/creation-functions/loop/range) - シンプルな連番生成
- [defer()](/guide/creation-functions/conditional/defer) - 購読時に動的生成
- [expand()](/guide/operators/transformation/expand) - 再帰的な展開（高階オペレーター）
- [scan()](/guide/operators/transformation/scan) - 累積計算

## 参考リソース

- [RxJS公式: generate()](https://rxjs.dev/api/index/function/generate)
- [Learn RxJS: generate](https://www.learnrxjs.io/learn-rxjs/operators/creation/generate)
