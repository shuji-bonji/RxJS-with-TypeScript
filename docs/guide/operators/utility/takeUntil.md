---
description: takeUntilオペレーターは、通知用Observableが値を発行するまで元のObservableを購読し、通知された時点で購読を解除するために使われます。
---

# takeUntil

`takeUntil` は、**指定した Observable（通知トリガー）が最初の値を発行するまで、元の Observable を購読し続ける** オペレーターです。通知トリガーが発行されたタイミングで、元の Observable の購読は解除されます。

## 🔁 基本構文

```ts
source$.pipe(
  takeUntil(notifier$)
)
```

- `source$`: 元の Observable（購読対象）
- `notifier$`: 停止を知らせる Observable（この Observable が最初の値を出すと購読が止まる）

[🌐 RxJS公式ドキュメント - takeUntil](https://rxjs.dev/api/index/function/takeUntil)

## 🧪 使用例：ボタンクリックで購読停止

```ts
import { interval, fromEvent } from 'rxjs';
import { takeUntil } from 'rxjs/operators';

const stopButton = document.createElement('button');
stopButton.textContent = 'stop';
document.body.appendChild(stopButton)

const stop$ = fromEvent(stopButton, 'click');
const source$ = interval(1000); // 1秒ごとに数値を発行

source$
  .pipe(takeUntil(stop$))
  .subscribe((val) => console.log(`値: ${val}`));
```

📌 `stopButton` がクリックされると、その瞬間に `source$` の購読が停止されます。

## ✅ よくあるユースケース

- キャンセルボタンで HTTP リクエストやポーリング処理を停止したいとき
- コンポーネントのライフサイクルに応じて購読を解除したいとき
- ページ遷移やアンマウントで非同期処理を打ち切りたいとき

## 🔗 関連オペレーター

- `take`: 一定回数まで値を受け取る
- `first`: 最初の1件だけを取得して終了
- `skipUntil`: 特定の Observable が値を出すまでは無視する
