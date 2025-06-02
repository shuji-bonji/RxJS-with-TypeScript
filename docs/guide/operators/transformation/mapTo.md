---
description: mapToオペレーターは、ストリーム内のすべての値を特定の固定値に変換し、ユーザー操作の検知や単純なトリガー処理に利用されます。
---

# mapTo - 常に固定の値を出力する

`mapTo`オペレーターは、入力ストリームに流れる値に関係なく、**常に同じ固定値を出力**します。  
クリックやイベントストリームなど、発生を検知するだけで十分な場面でよく使われます。


## 🔰 基本構文と使い方

```ts
import { fromEvent } from 'rxjs';
import { mapTo, tap } from 'rxjs/operators';

const clicks$ = fromEvent(document, 'click');

clicks$
  .pipe(
    // tap(console.log), // イベントが発火されていること確認する場合コメントアウトを解除してください。
    mapTo('クリックされました！')
  )
  .subscribe(console.log);

// 出力 (クリックごとに):
// クリックされました！
// クリックされました！
// ...

```

入力されたイベントの中身は無視され、常に「クリックされました！」という文字列だけが出力されます。

[🌐 RxJS公式ドキュメント - `mapTo`](https://rxjs.dev/api/operators/mapTo)

## 💡 典型的な活用パターン

- ユーザー操作を検知して、単一のアクションを発生させる
- イベント検知時に固定メッセージや通知を送信する
- インジケーターのON/OFFトリガーを簡素化する


## 🧠 実践コード例（UI付き）

ボタンをクリックすると、常に「イベント検知！」というメッセージを表示します。

```ts
import { fromEvent } from 'rxjs';
import { mapTo } from 'rxjs/operators';

// ボタン作成
const button = document.createElement('button');
button.textContent = 'クリック';
document.body.appendChild(button);

// 出力領域
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// クリックイベントを検知
fromEvent(button, 'click')
  .pipe(mapTo('イベント検知！'))
  .subscribe((message) => {
    output.textContent = message;
    output.style.color = 'red'; // 色を一瞬赤にする
    setTimeout(() => {
      output.style.color = 'black'; // 500ms後に元に戻す
    }, 300);
  });


```

- クリックごとに出力領域に「イベント検知！」と表示されます。
- 入力の内容は一切気にせず、単に**イベントが発生したことだけをトリガー**にしています。

