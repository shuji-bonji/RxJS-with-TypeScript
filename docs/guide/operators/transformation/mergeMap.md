---
description: mergeMapは各値をObservableに変換して並列実行するRxJSの変換オペレーターで、非同期処理の同時実行や複数リクエストの管理に便利です。
---

# mergeMap - 各値をObservableに変換し、同時並行でマージする

`mergeMap`（別名 `flatMap`）オペレーターは、各値を新しいObservableに変換し、**それらを同時並行でフラットに結合**します。  
リクエストを順番待ちせずにすぐ実行したいときや、ネストされた非同期処理に非常に便利です。

## 🔰 基本構文と使い方

```ts
import { of } from 'rxjs';
import { mergeMap, delay } from 'rxjs/operators';

of('A', 'B', 'C').pipe(
  mergeMap(value =>
    of(`${value} 完了`).pipe(delay(1000))
  )
).subscribe(console.log);

// 出力例 (順不同):
// A 完了
// B 完了
// C 完了
```

- 各値ごとに新しいObservableを生成します。
- それらのObservableは**並列に実行**され、結果は順不同で出力されます。

[🌐 RxJS公式ドキュメント - `mergeMap`](https://rxjs.dev/api/operators/mergeMap)

## 💡 典型的な活用パターン

- ボタンクリックごとにAPIリクエストを投げる
- ファイルドロップイベントごとにファイルアップロードを開始する
- ユーザー操作をトリガーに非同期タスクを同時進行させる

## 🧠 実践コード例（UI付き）

ボタンをクリックするたびに、非同期リクエスト（2秒後にレスポンス）を発生させる例です。

```ts
import { fromEvent, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs/operators';

// ボタン作成
const button = document.createElement('button');
button.textContent = 'リクエスト送信';
document.body.appendChild(button);

// 出力領域
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// クリックイベント
fromEvent(button, 'click').pipe(
  mergeMap((_, index) => {
    const requestId = index + 1;
    console.log(`リクエスト${requestId}開始`);
    return of(`レスポンス${requestId}`).pipe(delay(2000));
  })
).subscribe((response) => {
  const div = document.createElement('div');
  div.textContent = `✅ ${response}`;
  output.appendChild(div);
});
```

- 各クリックで非同期リクエストが即座に発行されます。
- **リクエストごとに個別に2秒待つ**ため、結果は到着順に並びません。
- 並列処理を理解するのに最適なサンプルです。