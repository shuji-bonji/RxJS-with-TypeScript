---
description: bufferCountは指定した個数ごとに値をまとめて配列で出力するRxJSの変換オペレーターです。バッチ処理や一定件数ごとのデータ集計、パケット分割送信など個数ベースのストリーム制御に最適で、TypeScriptの型推論により型安全な配列操作を実現します。
---

# bufferCount - 指定した個数ごとに値をまとめる

`bufferCount`オペレーターは、発行された値を指定した個数ごとに**まとめて配列として出力**します。  
値の数で区切るバッチ処理を行いたいときに便利です。

## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { bufferCount } from 'rxjs';

// 100msごとに値を発行
const source$ = interval(100);

source$.pipe(
  bufferCount(5)
).subscribe(buffer => {
  console.log('5つごとの値:', buffer);
});

// 出力:
// 5つごとの値: [0, 1, 2, 3, 4]
// 5つごとの値: [5, 6, 7, 8, 9]
// ...
```

- 5個の値をまとめて配列にして出力します。
- 時間ではなく、**個数ベース**でまとめる点が特徴です。

[🌐 RxJS公式ドキュメント - `bufferCount`](https://rxjs.dev/api/operators/bufferCount)

## 💡 典型的な活用パターン

- データパケットの分割送信
- 一定件数ごとにバッチ保存やバッチ処理
- 入力イベントの一定回数ごとの集計

## 🧠 実践コード例（UI付き）

キーボードのキー入力を5回ごとにまとめて表示する例です。

```ts
import { fromEvent } from 'rxjs';
import { map, bufferCount } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// キー入力イベントストリーム
fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  bufferCount(5)
).subscribe(keys => {
  const message = `5回の入力: ${keys.join(', ')}`;
  console.log(message);
  output.textContent = message;
});
```

- キーを5回押すたびに、その5回分のキーがまとめて表示されます。
- 個数に応じた集約処理が体験できます。
