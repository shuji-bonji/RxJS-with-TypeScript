---
description: combineLatestAllは、Higher-order Observable（Observable of Observables）を受け取り、全ての内部Observableが最低1回発火したら、それぞれの最新値を組み合わせて出力するオペレーターです。
---

# combineLatestAll - 全ての内部Observableの最新値を組み合わせる

`combineLatestAll` オペレーターは、**Higher-order Observable**（Observable of Observables）を受け取り、
**全ての内部Observableが最低1回発火したら**、それぞれの**最新値を組み合わせて**配列として出力します。

## 🔰 基本構文と使い方

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// 3つの内部Observableを持つHigher-order Observable
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// 全ての内部Observableが最低1回発火したら、最新値を組み合わせる
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// 出力:
// [1, 3, 0] ← 全てが最低1回発火した時点（2秒後）
// [2, 3, 0] ← 1つ目のObservableが2を発火（3秒後）
// [2, 3, 1] ← 3つ目のObservableが1を発火（4秒後）
```

- Higher-order Observableが**完了**した時点で内部Observableを収集
- **全ての内部Observableが最低1回発火**したら、組み合わせを開始
- いずれかの内部Observableが値を発行するたびに、**全ての最新値を組み合わせて**出力

[🌐 RxJS公式ドキュメント - `combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 典型的な活用パターン

- **複数のAPI呼び出しの最新結果を組み合わせる**
- **複数のフォーム入力の最新値を同期**
- **複数のリアルタイムデータソースを統合**

## 🧠 実践コード例

複数のAPI呼び出しの最新結果を組み合わせて表示する例：

```ts
import { of, timer, Observable } from 'rxjs';
import { map, combineLatestAll, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// 3つの模擬API呼び出しを作成（Higher-order Observable）
const apiCalls$: Observable<Observable<string>> = of(
  // API 1: ユーザー情報（1秒後に完了、3回更新）
  timer(0, 1000).pipe(
    take(3),
    map(n => `ユーザー: User${n}`)
  ),
  // API 2: 通知数（0.5秒後に完了、4回更新）
  timer(0, 500).pipe(
    take(4),
    map(n => `通知: ${n}件`)
  ),
  // API 3: ステータス（2秒後に完了、2回更新）
  timer(0, 2000).pipe(
    take(2),
    map(n => n === 0 ? 'ステータス: オフライン' : 'ステータス: オンライン')
  )
);

// 全てのAPI呼び出しの最新値を組み合わせて表示
apiCalls$
  .pipe(combineLatestAll())
  .subscribe(values => {
    output.innerHTML = '<strong>最新の状態:</strong><br>';
    values.forEach((value, index) => {
      const item = document.createElement('div');
      item.textContent = `${index + 1}. ${value}`;
      output.appendChild(item);
    });
  });
```

- 3つのAPI呼び出しが並行実行されます
- **全てが最低1回発火したら**、組み合わせた結果が表示されます
- いずれかのAPIが更新されるたびに、**最新の組み合わせ**が表示されます

## 🔄 関連Creation Function

`combineLatestAll` は主にHigher-order Observableの平坦化に使用されますが、
通常の複数Observableの組み合わせには **Creation Function** の `combineLatest` を使用します。

```ts
import { combineLatest, interval } from 'rxjs';

// Creation Function版（より一般的な使い方）
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

[3章 Creation Functions - combineLatest](/guide/creation-functions/combineLatest) を参照。

## 🔄 関連オペレーター

| オペレーター | 説明 |
|---|---|
| [mergeAll](./mergeAll) | 全ての内部Observableを並行に購読 |
| [concatAll](./concatAll) | 内部Observableを順番に購読 |
| [switchAll](./switchAll) | 新しい内部Observableに切り替え |
| [zipAll](./zipAll) | 各内部Observableの対応する順番の値をペア化 |

## ⚠️ 注意点

### Higher-order Observableの完了が必要

`combineLatestAll` は、Higher-order Observable（外側のObservable）が**完了するまで**内部Observableの収集を待ちます。

#### ❌ Higher-order Observableが完了しないため、何も出力されない
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // 何も出力されない
```

#### ✅ take で完了させる
```ts
interval(1000).pipe(
  take(3), // 3回で完了
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### 全ての内部Observableが最低1回発火が必要

全ての内部Observableが**最低1回発火するまで**、値は出力されません。

```ts
// 1つでも発火しない内部Observableがあると、何も出力されない
of(
  of(1, 2, 3),
  NEVER // 永遠に発火しない
).pipe(
  combineLatestAll()
).subscribe(console.log); // 何も出力されない
```

### メモリ使用量

全ての内部Observableの**最新値をメモリに保持**するため、内部Observableが多い場合はメモリ使用量に注意してください。
