---
description: reduceオペレーターは、ストリームのすべての値を累積し、完了時に最終結果のみを出力するRxJSの演算子で、集計処理や最終的な計算結果の取得に使用されます。
---

# reduce - 最終的な累積結果のみを出力する

`reduce`オペレーターは、ストリームの各値に累積関数を適用し、**ストリーム完了時に最終的な累積結果のみ**を出力します。
配列の`Array.prototype.reduce`と同じ動作で、中間結果は出力されません。

## 🔰 基本構文と使い方

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// 出力: 15（最終結果のみ）
```

- `acc`は累積値、`curr`は現在の値です。
- 初期値（この場合は`0`）から開始して順次累積します。
- ストリームが完了するまで値を出力せず、**完了時に最終結果のみ**を出力します。

[🌐 RxJS公式ドキュメント - `reduce`](https://rxjs.dev/api/operators/reduce)

## 💡 典型的な活用パターン

- 数値の合計、平均、最大値、最小値の計算
- オブジェクトの集約や変換
- 配列の構築や結合
- 最終的な集計結果のみが必要な場合

## 🔍 scan との違い

| オペレーター | 出力タイミング | 出力内容 | 用途 |
|:---|:---|:---|:---|
| `reduce` | **完了時に1回のみ** | 最終的な累積結果 | 最終結果のみ必要な集計 |
| `scan` | **各値ごとに毎回** | 中間結果を含むすべて | リアルタイム集計・状態管理 |

```ts
import { of } from 'rxjs';
import { reduce, scan } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== reduce ===');
source$.pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// 出力: 15

console.log('=== scan ===');
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// 出力: 1, 3, 6, 10, 15
```

## 🧠 実践コード例（UI付き）

複数の入力フィールドの値を合計し、ボタンクリック時に最終結果を表示する例です。

```ts
import { fromEvent, from } from 'rxjs';
import { map, reduce, switchMap } from 'rxjs';

// 入力フィールドを作成
const inputs: HTMLInputElement[] = [];
for (let i = 1; i <= 3; i++) {
  const label = document.createElement('label');
  label.textContent = `値${i}: `;
  const input = document.createElement('input');
  input.type = 'number';
  input.value = '0';
  label.appendChild(input);
  document.body.appendChild(label);
  document.body.appendChild(document.createElement('br'));
  inputs.push(input);
}

// 計算ボタン
const button = document.createElement('button');
button.textContent = '合計を計算';
document.body.appendChild(button);

// 結果表示エリア
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// ボタンクリック時に合計を計算
fromEvent(button, 'click').pipe(
  switchMap(() => {
    // すべての入力値を取得
    const values = inputs.map(input => Number(input.value) || 0);
    return from(values).pipe(
      reduce((acc, curr) => acc + curr, 0)
    );
  })
).subscribe(total => {
  output.textContent = `合計: ${total}`;
  console.log('合計:', total);
});
```

- ボタンをクリックすると、すべての入力値が集計され、最終的な合計のみが表示されます。
- 中間結果は出力されません。

## 🎯 オブジェクトの集約例

複数の値をオブジェクトにまとめる実践的な例です。

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface Product {
  category: string;
  price: number;
}

const products: Product[] = [
  { category: '食品', price: 500 },
  { category: '飲料', price: 200 },
  { category: '食品', price: 800 },
  { category: '飲料', price: 150 },
  { category: '食品', price: 300 },
];

// カテゴリ別の合計金額を集計
from(products).pipe(
  reduce((acc, product) => {
    acc[product.category] = (acc[product.category] || 0) + product.price;
    return acc;
  }, {} as Record<string, number>)
).subscribe(result => {
  console.log('カテゴリ別合計:', result);
});

// 出力:
// カテゴリ別合計: { 食品: 1600, 飲料: 350 }
```

## 🎯 配列の構築例

ストリームの値を配列にまとめる例です。

```ts
import { interval } from 'rxjs';
import { take, reduce } from 'rxjs';

interval(100).pipe(
  take(5),
  reduce((acc, value) => {
    acc.push(value);
    return acc;
  }, [] as number[])
).subscribe(array => {
  console.log('収集された配列:', array);
});

// 出力:
// 収集された配列: [0, 1, 2, 3, 4]
```

::: tip
配列を構築する場合は、より簡潔な [`toArray`](../utility/toArray) オペレーターの使用を検討してください。
```ts
interval(100).pipe(
  take(5),
  toArray()
).subscribe(console.log);
// 出力: [0, 1, 2, 3, 4]
```
:::

## 💡 型安全な reduce の活用

TypeScriptの型推論を活用した例です。

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface UserAction {
  type: 'click' | 'scroll' | 'input';
  timestamp: number;
}

const actions: UserAction[] = [
  { type: 'click', timestamp: 100 },
  { type: 'scroll', timestamp: 200 },
  { type: 'click', timestamp: 300 },
  { type: 'input', timestamp: 400 },
];

const actions$ = from(actions);

// アクション種類ごとの回数を集計
actions$.pipe(
  reduce((acc, action) => {
    acc[action.type] = (acc[action.type] || 0) + 1;
    return acc;
  }, {} as Record<UserAction['type'], number>)
).subscribe(result => {
  console.log('アクション集計:', result);
});

// 出力:
// アクション集計: { click: 2, scroll: 1, input: 1 }
```

## ⚠️ 注意点

### ❌ 無限ストリームでは完了しない（重要）

> [!WARNING]
> **`reduce`は`complete()`が呼ばれるまで1件も値を出力しません。** 無限ストリーム（`interval`, `fromEvent`など）では永久に値が得られないため、実務での事故の原因となります。

```ts
import { interval } from 'rxjs';
import { reduce } from 'rxjs';

// ❌ 悪い例: 無限ストリームなので値が出力されない
interval(1000).pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// 出力なし（ストリームが完了しないため）
```

**対策1: ローリング集計が必要な場合は`scan`を使用**

```ts
import { interval, scan, take } from 'rxjs';

// ✅ 良い例: リアルタイムで中間結果を取得
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// 出力: 0, 1, 3, 6, 10（累積値を毎回出力）
```

**対策2: 最終値のみ必要な場合は`scan` + `takeLast(1)`**

```ts
import { interval, scan, take, takeLast } from 'rxjs';

// ✅ 良い例: scanで累積し、最終値のみ取得
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0),
  takeLast(1)
).subscribe(console.log);
// 出力: 10（最終結果のみ）
```

**対策3: `take`で終了条件を明示**

```ts
import { interval, take, reduce } from 'rxjs';

// ✅ 良い例: takeで終了条件を設定
interval(1000).pipe(
  take(5),
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// 出力: 10
```

> [!TIP]
> **選択の基準**:
> - 中間結果が必要 → `scan`
> - 最終結果のみ必要 & ストリーム完了が保証される → `reduce`
> - 最終結果のみ必要 & 無限ストリーム → `scan` + `takeLast(1)` または `take` + `reduce`

### メモリ使用量

累積値が大きなオブジェクトや配列になる場合、メモリ使用量に注意が必要です。

```ts
// メモリに注意が必要な例
from(largeDataArray).pipe(
  reduce((acc, item) => {
    acc.push(item); // 大量のデータを蓄積
    return acc;
  }, [])
).subscribe();
```

## 📚 関連オペレーター

- [`scan`](./scan) - 各値ごとに中間結果を出力
- [`toArray`](../utility/toArray) - すべての値を配列にまとめる
- [`count`](https://rxjs.dev/api/operators/count) - 値の個数を数える
- [`min`](https://rxjs.dev/api/operators/min) / [`max`](https://rxjs.dev/api/operators/max) - 最小値・最大値を取得

## まとめ

`reduce`オペレーターは、ストリームのすべての値を累積し、**完了時に最終結果のみ**を出力します。中間結果が不要で、最終的な集計結果だけが必要な場合に適しています。ただし、ストリームが完了しない場合は結果が得られないため、無限ストリームでは `scan` を使用するか、`take` などで終了条件を設定する必要があります。
