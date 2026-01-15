---
description: firstオペレーターは、ストリームから最初の値、または指定した条件を満たす最初の値だけを取得し、その後ストリームを完了させます。最初に到達したイベントだけを処理したい場合や初回データの取得に便利です。
---

# first - 最初の値を取得

`first` オペレーターは、ストリームから**最初の値**、または**条件を満たす最初の値**だけを取得し、ストリームを完了させます。


## 🔰 基本構文と使い方

```ts
import { from } from 'rxjs';
import { first } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// 最初の値だけを取得
numbers$.pipe(
  first()
).subscribe(console.log);

// 条件を満たす最初の値だけを取得
numbers$.pipe(
  first(n => n > 3)
).subscribe(console.log);

// 出力:
// 1
// 4
```

- `first()` は、最初に流れた値を取得して完了します。
- 条件を渡すと、**条件を満たす最初の値**が取得されます。
- 条件に合う値が存在しない場合、エラーを発生させます。

[🌐 RxJS公式ドキュメント - `first`](https://rxjs.dev/api/operators/first)


## 💡 典型的な活用パターン

- 最初に到達したイベントだけを処理する
- 条件を満たす最初のデータ（例: 5点以上のスコア）を検出する
- タイムアウトやキャンセル前に来た最初のデータだけを採用する


## 🧠 実践コード例（UI付き）

ボタンを複数回クリックしても、**最初のクリックだけを処理**します。

```ts
import { fromEvent } from 'rxjs';
import { first } from 'rxjs';

const title = document.createElement('div');
title.innerHTML = '<h3>first の実践例:</h3>';
document.body.appendChild(title);

// ボタン作成
const button = document.createElement('button');
button.textContent = 'クリックしてください（最初の一回のみ反応）';
document.body.appendChild(button);

// 出力エリア作成
let count = 0;
const output = document.createElement('div');
document.body.appendChild(output);
// ボタンクリックストリーム
fromEvent(button, 'click')
  .pipe(first())
  .subscribe(() => {
    const message = document.createElement('div');
    count++;
    message.textContent = `最初のクリックを検出しました！${count}`;
    output.appendChild(message);
  });
```

- 最初のクリックイベントだけを受け取り、それ以降は無視されます。
- ストリームは最初のクリック後に自動で`complete`します。
