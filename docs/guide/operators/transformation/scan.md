---
description: scanオペレーターは、各値を逐次的に蓄積しながら中間結果を出力するRxJSの演算子で、リアルタイム集計や状態管理に活用されます。
---

# scan - 累積的に値を生成する

`scan`オペレーターは、ストリームの各値に累積関数を適用し、**逐次的な中間結果**を出力します。  
配列の`Array.prototype.reduce`に似ていますが、すべての値が到達する前に中間結果を逐次出力する点が異なります。

## 🔰 基本構文と使い方

```ts
import { of } from 'rxjs';
import { scan } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(scan((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// 出力: 1, 3, 6, 10, 15

```

- `acc`は累積値、`curr`は現在の値です。
- 初期値（この場合は`0`）から開始して順次累積していきます。

[🌐 RxJS公式ドキュメント - `scan`](https://rxjs.dev/api/operators/scan)

## 💡 典型的な活用パターン

- カウントアップやスコア集計
- フォームのリアルタイム検証状態の管理
- バッファリングされたイベントの累積処理
- リアルタイム集計グラフのデータ構築 

## 🧠 実践コード例（UI付き）

ボタンをクリックするたびに、累計クリック回数を表示します。

```ts
import { fromEvent } from 'rxjs';
import { scan, tap } from 'rxjs';

// ボタン作成
const button = document.createElement('button');
button.textContent = 'クリック';
document.body.appendChild(button);

// 出力エリア作成
const counter = document.createElement('div');
counter.style.marginTop = '10px';
document.body.appendChild(counter);

// クリックイベントを累計
fromEvent(button, 'click')
  .pipe(
    tap((v) => console.log(v)),
    scan((count) => count + 1, 0)
  )
  .subscribe((count) => {
    counter.textContent = `クリック回数: ${count}`;
  });
```

- ボタンをクリックするたびに、カウンターが1ずつ増加します。
- `scan`を使うことで、**状態管理なしに簡単なカウントロジック**が書けます。