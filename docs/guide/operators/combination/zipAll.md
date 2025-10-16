---
description: zipAllは、Higher-order Observable（Observable of Observables）を受け取り、各内部Observableの対応する順番の値をペア化して配列として出力するオペレーターです。
---

# zipAll - 各内部Observableの対応する値をペア化

`zipAll` オペレーターは、**Higher-order Observable**（Observable of Observables）を受け取り、
**各内部Observableの対応する順番の値をペア化**して配列として出力します。

## 🔰 基本構文と使い方

```ts
import { interval, of } from 'rxjs';
import { zipAll, take } from 'rxjs';

// 3つの内部Observableを持つHigher-order Observable
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// 各内部Observableの対応する順番の値をペア化
higherOrder$
  .pipe(zipAll())
  .subscribe(values => console.log(values));

// 出力:
// [0, 0, 0] ← 全ての1番目の値
// [1, 1, 1] ← 全ての2番目の値
// （ここで完了：3つ目のObservableが2個しか値を出さないため）
```

- Higher-order Observableが**完了**した時点で内部Observableを収集
- 各内部Observableの**同じインデックスの値をペア化**
- **最も短い内部Observableが完了**したら、全体が完了

[🌐 RxJS公式ドキュメント - `zipAll`](https://rxjs.dev/api/index/function/zipAll)

## 💡 典型的な活用パターン

- **複数のAPIレスポンスを順番に対応付ける**
- **複数のストリームの同じタイミングの値を比較**
- **パラレル処理の結果を順番に組み合わせる**

## 🧠 実践コード例

複数のカウンターの対応する値をペア化する例

```ts
import { interval, of } from 'rxjs';
import { zipAll, take, map } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// 3つの異なる速度のカウンターを作成
const counters$ = of(
  interval(1000).pipe(take(4), map(n => `遅い: ${n}`)),
  interval(500).pipe(take(5), map(n => `普通: ${n}`)),
  interval(300).pipe(take(6), map(n => `速い: ${n}`))
);

// 各カウンターの対応する順番の値をペア化
counters$
  .pipe(zipAll())
  .subscribe(values => {
    const item = document.createElement('div');
    item.textContent = `[${values.join(', ')}]`;
    output.appendChild(item);
  });

// 出力:
// [遅い: 0, 普通: 0, 速い: 0]
// [遅い: 1, 普通: 1, 速い: 1]
// [遅い: 2, 普通: 2, 速い: 2]
// [遅い: 3, 普通: 3, 速い: 3]
// （ここで完了：「遅い」カウンターが4個しか値を出さないため）
```

## 🔄 関連Creation Function

`zipAll` は主にHigher-order Observableの平坦化に使用されますが、
通常の複数Observableのペア化には **Creation Function** の `zip` を使用します。

```ts
import { zip, interval } from 'rxjs';
import { take } from 'rxjs';

// Creation Function版（より一般的な使い方）
const zipped$ = zip(
  interval(1000).pipe(take(3)),
  interval(500).pipe(take(4)),
  interval(2000).pipe(take(2))
);

zipped$.subscribe(console.log);
```

[3章 Creation Functions - zip](/guide/creation-functions/zip) を参照。

## 🔄 関連オペレーター

| オペレーター | 説明 |
|---|---|
| [combineLatestAll](./combineLatestAll) | 全ての内部Observableの最新値を組み合わせる |
| [mergeAll](./mergeAll) | 全ての内部Observableを並行に購読 |
| [concatAll](./concatAll) | 内部Observableを順番に購読 |
| [switchAll](./switchAll) | 新しい内部Observableに切り替え |

## 🔄 zipAll vs combineLatestAll

| オペレーター | 組み合わせ方 | 完了タイミング |
|---|---|---|
| `zipAll` | **同じインデックス**の値をペア化 | **最も短い**内部Observableが完了したら |
| `combineLatestAll` | **最新値**を組み合わせ | **全ての**内部Observableが完了したら |

```ts
// zipAll: [0番目, 0番目, 0番目], [1番目, 1番目, 1番目], ...
// combineLatestAll: [最新, 最新, 最新], [最新, 最新, 最新], ...
```

## ⚠️ 注意点

### Higher-order Observableの完了が必要

`zipAll` は、Higher-order Observable（外側のObservable）が**完了するまで**内部Observableの収集を待ちます。

#### ❌ Higher-order Observableが完了しないため、何も出力されない
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log); // 何も出力されない
```

#### ✅ take で完了させる
```ts
interval(1000).pipe(
  take(3), // 3回で完了
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log);
```

### 最も短い内部Observableで完了

**最も短い内部Observableが完了**したら、全体が完了します。

```ts
import { of, zipAll } from "rxjs";

of(
  of(1, 2, 3, 4, 5), // 5個
  of(1, 2)           // 2個 ← 最も短い
).pipe(
  zipAll()
).subscribe(console.log);

// 出力: [1, 1], [2, 2]
// （2個で完了。3, 4, 5 は使用されない）
```

### バックプレッシャー（メモリ使用量）

内部Observableの発行速度が異なる場合、**速い内部Observableの値がメモリに溜まります**。

```ts
import { interval, of, take, zipAll } from "rxjs";

// 速いカウンター（100ms）の値が、遅いカウンター（10000ms）を待つ間メモリに溜まる
of(
  interval(10000).pipe(take(3)), // 遅い
  interval(100).pipe(take(100))  // 速い
).pipe(
  zipAll()
).subscribe(console.log);
```

速度差が大きい場合は、メモリ使用量に注意してください。
