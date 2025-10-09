---
description: pairwiseオペレーターは、連続する2つの値をペアの配列として出力するRxJSの演算子で、前回値と現在値の比較や差分計算に活用されます。
---

# pairwise - 連続する2つの値をペアで処理する

`pairwise`オペレーターは、ストリームから発行される**連続する2つの値を配列 `[前の値, 現在の値]` としてまとめて出力**します。
前回の値と現在の値を比較したり、変化量を計算したりする場合に便利です。

## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { pairwise, take } from 'rxjs/operators';

interval(1000).pipe(
  take(6),
  pairwise()
).subscribe(console.log);

// 出力:
// [0, 1]
// [1, 2]
// [2, 3]
// [3, 4]
// [4, 5]
```

- 最初の値（0）は単独では出力されず、2つ目の値（1）が来た時点で `[0, 1]` として出力されます。
- 常に**直前の値と現在の値**のペアが出力されます。

[🌐 RxJS公式ドキュメント - `pairwise`](https://rxjs.dev/api/operators/pairwise)

## 💡 典型的な活用パターン

- マウスやタッチの移動量の計算
- 価格や数値の変化量（差分）の計算
- 状態の変化検知（前の状態と現在の状態の比較）
- スクロール方向の判定

## 🧠 実践コード例（UI付き）

マウスの移動方向と移動量を表示する例です。

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise } from 'rxjs/operators';

// 出力エリア作成
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// マウス移動イベント
fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY })),
  pairwise()
).subscribe(([prev, curr]) => {
  const deltaX = curr.x - prev.x;
  const deltaY = curr.y - prev.y;
  const direction = deltaX > 0 ? '右' : deltaX < 0 ? '左' : '停止';

  output.innerHTML = `
    前回: (${prev.x}, ${prev.y})<br>
    現在: (${curr.x}, ${curr.y})<br>
    移動量: Δx=${deltaX}, Δy=${deltaY}<br>
    方向: ${direction}
  `;
});
```

- マウスを動かすと、前回と現在の座標、移動量が表示されます。
- `pairwise`により、前回の座標と現在の座標が自動的にペアで取得できます。

## 🎯 数値の変化量を計算する例

数値ストリームの変化量（差分）を計算する実践例です。

```ts
import { interval } from 'rxjs';
import { map, pairwise, take } from 'rxjs/operators';

// 0, 1, 4, 9, 16, 25 (平方数)
interval(500).pipe(
  take(6),
  map(n => n * n),
  pairwise(),
  map(([prev, curr]) => ({
    prev,
    curr,
    diff: curr - prev
  }))
).subscribe(result => {
  console.log(`${result.prev} → ${result.curr} (差分: +${result.diff})`);
});

// 出力:
// 0 → 1 (差分: +1)
// 1 → 4 (差分: +3)
// 4 → 9 (差分: +5)
// 9 → 16 (差分: +7)
// 16 → 25 (差分: +9)
```

## 🎯 スクロール方向の判定

スクロールの方向（上下）を判定する例です。

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise, throttleTime } from 'rxjs/operators';

// 固定表示される出力エリア作成
const output = document.createElement('div');
output.style.position = 'fixed';
output.style.top = '10px';
output.style.right = '10px';
output.style.padding = '15px';
output.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
output.style.color = 'white';
output.style.fontFamily = 'monospace';
output.style.fontSize = '14px';
output.style.borderRadius = '5px';
output.style.zIndex = '9999';
document.body.appendChild(output);

// スクロール可能にするためのダミーコンテンツ
const content = document.createElement('div');
content.style.height = '200vh'; // ページの高さを2倍にする
content.innerHTML = '<h1>下にスクロールしてください</h1>';
document.body.appendChild(content);

// スクロール位置を取得
fromEvent(window, 'scroll').pipe(
  throttleTime(100), // 100msごとに間引き
  map(() => window.scrollY),
  pairwise()
).subscribe(([prevY, currY]) => {
  const diff = currY - prevY;
  const direction = diff > 0 ? '↓ 下' : '↑ 上';
  const arrow = diff > 0 ? '⬇️' : '⬆️';

  output.innerHTML = `
    ${arrow} スクロール方向: ${direction}<br>
    前回位置: ${prevY.toFixed(0)}px<br>
    現在位置: ${currY.toFixed(0)}px<br>
    移動量: ${Math.abs(diff).toFixed(0)}px
  `;
});
```

- ページをスクロールすると、右上に固定表示されたエリアに方向と位置情報が表示されます。
- `pairwise`により、前回と現在のスクロール位置が自動的にペアで取得できます。

## 🎯 型安全な pairwise の活用

TypeScriptの型推論を活用した例です。

```ts
import { from } from 'rxjs';
import { pairwise } from 'rxjs/operators';

interface Stock {
  symbol: string;
  price: number;
  timestamp: number;
}

const stockPrices: Stock[] = [
  { symbol: 'AAPL', price: 150, timestamp: 1000 },
  { symbol: 'AAPL', price: 152, timestamp: 2000 },
  { symbol: 'AAPL', price: 148, timestamp: 3000 },
  { symbol: 'AAPL', price: 155, timestamp: 4000 },
];

from(stockPrices).pipe(
  pairwise()
).subscribe(([prev, curr]) => {
  const change = curr.price - prev.price;
  const changePercent = ((change / prev.price) * 100).toFixed(2);
  const trend = change > 0 ? '📈' : change < 0 ? '📉' : '➡️';

  console.log(
    `${curr.symbol}: $${prev.price} → $${curr.price} ` +
    `(${changePercent}%) ${trend}`
  );
});

// 出力:
// AAPL: $150 → $152 (1.33%) 📈
// AAPL: $152 → $148 (-2.63%) 📉
// AAPL: $148 → $155 (4.73%) 📈
```

## 🔍 bufferCount(2, 1) との比較

`pairwise()`は`bufferCount(2, 1)`と同等の動作をします。

```ts
import { of } from 'rxjs';
import { pairwise, bufferCount } from 'rxjs/operators';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== pairwise ===');
source$.pipe(pairwise()).subscribe(console.log);
// 出力: [1,2], [2,3], [3,4], [4,5]

console.log('=== bufferCount(2, 1) ===');
source$.pipe(bufferCount(2, 1)).subscribe(console.log);
// 出力: [1,2], [2,3], [3,4], [4,5]
```

**使い分け**:
- `pairwise()`: 連続する2つの値のペアを扱うことが明示的で、コードの意図が明確
- `bufferCount(2, 1)`: より柔軟（3つ以上のウィンドウサイズにも対応可能）

## ⚠️ 注意点

### 最初の値は出力されない

`pairwise`は2つの値が揃うまで何も出力しないため、最初の値は単独では取得できません。

```ts
import { of } from 'rxjs';
import { pairwise } from 'rxjs/operators';

of(1).pipe(pairwise()).subscribe(
  console.log,
  null,
  () => console.log('完了')
);

// 出力:
// 完了
// （値は1つも出力されない）
```

**対策**: 最初の値も処理したい場合は、`startWith`で初期値を追加します。

```ts
import { of } from 'rxjs';
import { startWith, pairwise } from 'rxjs/operators';

of(10, 20, 30).pipe(
  startWith(0),
  pairwise()
).subscribe(console.log);

// 出力:
// [0, 10]
// [10, 20]
// [20, 30]
```

### メモリ使用量

`pairwise`は常に直前の1つの値のみを保持するため、メモリ効率は良好です。

## 📚 関連オペレーター

- [`scan`](./scan) - より複雑な累積処理
- [`bufferCount`](./bufferCount) - 指定個数ごとに値をまとめる
- [`distinctUntilChanged`](../filtering/distinctUntilChanged) - 連続する重複値を除去
- [`startWith`](../utility/startWith) - 初期値を追加

## まとめ

`pairwise`オペレーターは、連続する2つの値を `[前の値, 現在の値]` のペアとして出力します。マウス移動の追跡、価格変動の計算、状態遷移の検出など、**前回値と現在値の比較が必要な場面**で非常に便利です。最初の値は2つ目の値が来るまで出力されない点に注意が必要ですが、`startWith`で初期値を追加することで対応できます。
