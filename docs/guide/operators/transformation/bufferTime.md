---
description: bufferTimeは一定時間ごとに発行された値をまとめて出力するRxJSオペレーターで、時間単位でのバッチ処理や集約に適しています。
---

# bufferTime - 一定時間ごとに値をまとめて出力する

`bufferTime`オペレーターは、指定した時間間隔で、**値をまとめて配列として出力**します。  
ストリームを一定時間で区切り、バッチ処理のように扱いたい場合に便利です。

## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { bufferTime } from 'rxjs';

// 100msごとに値を発行
const source$ = interval(100);

source$.pipe(
  bufferTime(1000)
).subscribe(buffer => {
  console.log('1秒間に収集された値:', buffer);
});

// 出力例:
// 1秒間に収集された値: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// 1秒間に収集された値: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
// ...
```

- 1秒間に発行された値が配列にまとめられ、順番に出力されます。

[🌐 RxJS公式ドキュメント - `bufferTime`](https://rxjs.dev/api/operators/bufferTime)

## 💡 典型的な活用パターン

- 一定時間ごとにバッチ送信する
- ユーザー操作をまとめて処理する（例：ドラッグ操作）
- センサーやIoTデバイスからのデータ収集
- ログやトレース情報の間引きと圧縮

## 🧠 実践コード例（UI付き）

クリックイベントを1秒間バッファリングして、1秒ごとにまとめて出力します。

```ts
import { fromEvent } from 'rxjs';
import { bufferTime } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// クリックイベントストリーム
const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  bufferTime(1000)
).subscribe(clickArray => {
  const message = `1秒間のクリック数: ${clickArray.length}`;
  console.log(message);
  output.textContent = message;
});
```

- 1秒間に何回クリックされたかをまとめて表示します。
- バッファリング処理により、イベントの連続発生をまとめて管理できます。
