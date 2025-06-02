---
description: zipオペレーターは複数のObservableから対応する順番の値を揃えてペアにし、すべてのソースが1つずつ値を発行したタイミングで出力します。
---

# zip - 対応する値をペアにする

`zip` オペレーターは、複数のObservableから発行される**対応する順番の値**をまとめ、配列やタプルにして出力します。  
すべてのソースObservableから1つずつ値が到着するまで待機し、揃ったタイミングでペアを作成します。


## 🔰 基本構文と使い方

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs/operators';

const source1$ = of('A', 'B', 'C');
const source2$ = interval(1000).pipe(
  map((val) => val * 10),
  take(3)
);

zip(source1$, source2$).subscribe(([letter, number]) => {
  console.log(letter, number);
});

// 出力:
// A 0
// B 10
// C 20
```

- 各Observableが1つずつ値を発行したタイミングで、ペアが作られて出力されます。
- 片方が遅れても、両方揃うまで待機します。

[🌐 RxJS公式ドキュメント - `zip`](https://rxjs.dev/api/index/function/zip)


## 💡 典型的な活用パターン

- **リクエストとレスポンスを対応づける**
- **IDと対応するデータを同期的にペア化する**
- **並列処理した複数ストリームを1セットにまとめる**


## 🧠 実践コード例（UI付き）

異なるデータソース（フルーツと価格）を**組み合わせて表示**する例です。

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs/operators';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>zip の実践例:</h3>';
document.body.appendChild(output);

// フルーツ名ストリーム
const fruits$ = of('🍎 Apple', '🍌 Banana', '🍇 Grape');

// 価格ストリーム（2秒おきに発行）
const prices$ = interval(2000).pipe(
  map((i) => [100, 200, 300][i]),
  take(3)
);

// zipして表示
zip(fruits$, prices$).subscribe(([fruit, price]) => {
  const item = document.createElement('div');
  item.textContent = `${fruit} - ￥${price}`;
  output.appendChild(item);
});
```

- フルーツと価格のリストが**1対1対応で揃った時点**でペアになって表示されます。
- どちらかが不足している場合は、その時点では出力されません。
