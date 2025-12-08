---
description: "bufferオペレーターは別のObservableが値を発行するタイミングで、蓄積された値を配列にまとめて出力します。ボタンクリックでの一括送信、ウィンドウ閉じる際のデータ保存など、イベント駆動型のバッチ処理に最適です。TypeScriptでの型安全な実装を解説します。"
---

# buffer - 別のObservableのタイミングで値をまとめる

`buffer`オペレーターは、**別のObservableが値を発行するまで**ソースObservableの値を蓄積し、そのタイミングで蓄積した値を**配列として一括出力**します。
時間や個数ではなく、外部のイベントやシグナルに応じてバッファリングを制御したい場合に便利です。

## 🔰 基本構文と使い方

```ts
import { interval, fromEvent } from 'rxjs';
import { buffer } from 'rxjs';

// 100msごとに値を発行
const source$ = interval(100);

// クリックイベントをトリガーとして使用
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  buffer(clicks$)
).subscribe(bufferedValues => {
  console.log('クリックまでに蓄積された値:', bufferedValues);
});

// 出力例（クリックするたびに出力）:
// クリックまでに蓄積された値: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
// クリックまでに蓄積された値: [11, 12, 13, 14, 15, 16, 17]
// ...
```

- `clicks$`が値を発行するたびに、それまでに蓄積された値が配列として出力されます。
- バッファの区切りを外部のObservableで制御できるのが特徴です。

[🌐 RxJS公式ドキュメント - `buffer`](https://rxjs.dev/api/operators/buffer)

## 💡 典型的な活用パターン

- ユーザーアクションをトリガーにしたバッチ処理
- 外部シグナルに基づくデータ収集と送信
- 動的な区切りでのイベントグループ化
- WebSocketやAPIの接続確立時にまとめて送信

## 🔍 bufferTime / bufferCount との違い

| オペレーター | 区切りのタイミング | 用途 |
|:---|:---|:---|
| `buffer` | **別のObservableの発行** | イベント駆動型の制御 |
| `bufferTime` | **一定時間** | 時間ベースのバッチ処理 |
| `bufferCount` | **一定個数** | 個数ベースのバッチ処理 |

```ts
import { interval, timer } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);
// 1秒ごとにトリガー
const trigger$ = timer(1000, 1000);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('1秒ごとの値:', values);
});

// 出力:
// 1秒ごとの値: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// 1秒ごとの値: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
```

## 🧠 実践コード例（UI付き）

ボタンクリックをトリガーに、それまでのマウス移動イベントをまとめて記録する例です。

```ts
import { fromEvent } from 'rxjs';
import { map, buffer } from 'rxjs';

// ボタンと出力エリアを作成
const button = document.createElement('button');
button.textContent = 'マウス移動を記録';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// マウス移動イベント
const mouseMoves$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
);

// ボタンクリックをトリガーに
const clicks$ = fromEvent(button, 'click');

mouseMoves$.pipe(
  buffer(clicks$)
).subscribe(positions => {
  const message = `検出されたイベント数: ${positions.length}件`;
  console.log(message);
  console.log('座標データ:', positions.slice(0, 5)); // 最初の5件のみ表示
  output.textContent = message;
});
```

- ボタンをクリックするまでのマウス移動がすべてバッファに蓄積されます。
- クリック時にまとめて処理されるため、任意のタイミングでのバッチ処理が可能です。

## 🎯 複数のトリガーを使った高度な例

複数のトリガーObservableを組み合わせることで、より柔軟な制御が可能です。

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { buffer, mapTo } from 'rxjs';

const source$ = interval(100);

// 複数のトリガー: クリックまたは5秒経過
const clicks$ = fromEvent(document, 'click').pipe(mapTo('click'));
const fiveSeconds$ = timer(5000, 5000).pipe(mapTo('timer'));
const trigger$ = merge(clicks$, fiveSeconds$);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log(`バッファ出力 (${values.length}個):`, values);
});
```

## ⚠️ 注意点

### メモリリークに注意

`buffer`は次のトリガーまで値を蓄積し続けるため、トリガーが長時間発生しない場合、メモリを圧迫する可能性があります。

```ts
// 悪い例: トリガーが発生しない可能性がある
const neverTrigger$ = fromEvent(document.querySelector('.non-existent'), 'click');

source$.pipe(
  buffer(neverTrigger$) // トリガーが発生せず、無限にバッファが蓄積される
).subscribe();
```

**対策**:
- `bufferTime`や`bufferCount`と組み合わせて最大バッファサイズを制限
- タイムアウト処理を追加

```ts
import { interval, fromEvent, timer, race } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);

// 複数のトリガー: クリックまたは5秒経過
const clicks$ = fromEvent(document, 'click');
const timeout$ = timer(10000); // 最大10秒でタイムアウト

source$.pipe(
  buffer(race(clicks$, timeout$)) // どちらか早い方で発行
).subscribe(values => {
  console.log('バッファ:', values);
});
```

## 📚 関連オペレーター

- [`bufferTime`](./bufferTime) - 時間ベースでバッファリング
- [`bufferCount`](./bufferCount) - 個数ベースでバッファリング
- [`bufferToggle`](https://rxjs.dev/api/operators/bufferToggle) - 開始・終了のObservableでバッファリング制御
- [`bufferWhen`](https://rxjs.dev/api/operators/bufferWhen) - 動的なクロージング条件でバッファリング
- [`window`](./windowTime) - バッファの代わりにObservableを返す

## まとめ

`buffer`オペレーターは、外部のObservableをトリガーとして値をまとめて処理するための強力なツールです。時間や個数ではなく、**イベント駆動型**のバッチ処理を実現できます。ただし、トリガーが発生しない場合のメモリリークには注意が必要です。
