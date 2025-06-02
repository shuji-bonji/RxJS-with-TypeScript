---
description: takeオペレーターは、Observableから指定数のデータだけを取得し、それ以降は無視してストリームを完了させます。
---

# take - 最初の指定数の値だけを取得する

`take` オペレーターは、ストリームから**最初の指定された数**だけ値を取得し、それ以降の値は無視します。  
完了後はストリームも自動的に`complete`します。

## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs/operators';

const source$ = interval(1000);

source$.pipe(
  take(3)
).subscribe(console.log);
// 出力: 0, 1, 2
```

- 最初の3件の値だけを取得して購読します。
- 3件取得後、Observableは自動的に`complete`します。

[🌐 RxJS公式ドキュメント - `take`](https://rxjs.dev/api/operators/take)

## 💡 典型的な活用パターン

- 最初の数件だけをUI表示やログに記録
- 最初のレスポンスだけを取り出す一時的なサブスク
- テストデータやデモデータの限定的な取得

## 🧠 実践コード例（UI付き）

1秒ごとに発行される数値から、最初の5件だけを取得して表示します。

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs/operators';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>take の実践例:</h3>';
document.body.appendChild(output);

// 1秒ごとに値を発行
const source$ = interval(1000);

// 最初の5つの値だけを取得
source$.pipe(take(5)).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `値: ${value}`;
    output.appendChild(item);
  },
  complete: () => {
    const complete = document.createElement('div');
    complete.textContent = '完了しました';
    complete.style.fontWeight = 'bold';
    output.appendChild(complete);
  },
});

```

- 最初の5件 (`0`, `1`, `2`, `3`, `4`) が順番に表示され、
- その後に「完了しました」というメッセージが表示されます。
