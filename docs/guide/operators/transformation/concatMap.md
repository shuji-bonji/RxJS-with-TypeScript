---
description: concatMapは各Observableを順番に処理し、前の処理が完了するまで次を待つRxJSオペレーターで、順序が重要な処理に適しています。
---

# concatMap - 各Observableを順番に実行する

`concatMap`オペレーターは、入力ストリームの各値をObservableに変換し、**それらを順番に実行して結合**します。  
**前のObservableが完了するまで次のObservableを開始しない**のが特徴です。

## 🔰 基本構文と使い方

```ts
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs/operators';

of('A', 'B', 'C').pipe(
  concatMap(value =>
    of(`${value} 完了`).pipe(delay(1000))
  )
).subscribe(console.log);

// 出力（順番通りに）:
// A 完了
// B 完了
// C 完了
```
- 各値をObservableに変換します。
- 前のObservableが完了してから次のObservableが実行されます。

[🌐 RxJS公式ドキュメント - concatMap](https://rxjs.dev/api/index/function/concatMap)

## 💡 典型的な活用パターン
- 順番が重要なAPIリクエストの実行
- キューに基づくタスク処理
- アニメーションやステップバイステップUIの制御
- 送信順序が重要なメッセージ送信処理


## 🧠 実践コード例（UI付き）

ボタンをクリックするたびにリクエストを発生させ、リクエストは必ず順番に処理される例です。

```ts
import { fromEvent, of } from 'rxjs';
import { concatMap, delay } from 'rxjs/operators';

// ボタン作成
const button = document.createElement('button');
button.textContent = 'リクエスト送信';
document.body.appendChild(button);

// 出力領域
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// クリックイベント
fromEvent(button, 'click')
  .pipe(
    concatMap((_, index) => {
      const requestId = index + 1;
      console.log(`リクエスト${requestId}開始`);
      return of(`レスポンス${requestId}`).pipe(delay(2000));
    })
  )
  .subscribe((response) => {
    const div = document.createElement('div');
    div.textContent = `✅ ${response}`;
    output.appendChild(div);
  });

```

- 各リクエストは必ず順番に送信・完了します。
- 次のリクエストは前のリクエストが完了してから発行されます。
