---
description: finalizeはObservableが完了・エラー・アンサブスクライブされた際に必ず実行される処理を定義するRxJSユーティリティオペレーターです。リソース解放、ローディング表示の終了、クリーンアップ処理など、ストリーム終了時の後始末が必要な場面に最適です。try-finallyのような確実な処理実行を保証し、メモリリーク防止に役立ちます。
---

# finalize - 完了時の処理

`finalize`オペレーターは、**Observableが完了・エラー・アンサブスクライブされたとき**に必ず呼ばれる処理を定義します。  
後始末やUIのローディング解除など、「必ず実行したい処理」に最適です。

## 🔰 基本構文・動作

```ts
import { of } from 'rxjs';
import { finalize } from 'rxjs';

of('完了')
  .pipe(finalize(() => console.log('ストリームが終了しました')))
  .subscribe(console.log);
// 出力:
// 完了
// ストリームが終了しました
```

この例では、`of()` で値を1つ発行した後に `finalize` の中の処理が実行されます。  
**`complete` でも `error` でも確実に呼ばれる**のが特徴です。

[🌐 RxJS公式ドキュメント - finalize](https://rxjs.dev/api/index/function/finalize)

## 💡 典型的な活用例

以下は、ストリームの前後でローディング表示の切り替えを行う例です。

```ts
import { of } from 'rxjs';
import { tap, delay, finalize } from 'rxjs';

let isLoading = false;

of('データ')
  .pipe(
    tap(() => {
      isLoading = true;
      console.log('ローディング開始');
    }),
    delay(1000),
    finalize(() => {
      isLoading = false;
      console.log('ローディング終了');
    })
  )
  .subscribe((value) => console.log('取得:', value));
// 出力:
// ローディング開始
// 取得: データ
// ローディング終了
```

## 🧪 実践コード例（UI付き）

```ts
import { interval } from 'rxjs';
import { take, finalize, tap } from 'rxjs';

// 出力表示エリア
const finalizeOutput = document.createElement('div');
finalizeOutput.innerHTML = '<h3>finalize の例:</h3>';
document.body.appendChild(finalizeOutput);

// ローディングインジケータ
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'データ読み込み中...';
loadingIndicator.style.backgroundColor = '#e0f7fa';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.borderRadius = '5px';
finalizeOutput.appendChild(loadingIndicator);

// 進捗状況表示
const progressContainer = document.createElement('div');
progressContainer.style.marginTop = '10px';
finalizeOutput.appendChild(progressContainer);

// 完了メッセージ用の要素
const completionMessage = document.createElement('div');
completionMessage.style.marginTop = '10px';
completionMessage.style.fontWeight = 'bold';
finalizeOutput.appendChild(completionMessage);

// データ取得のシミュレーション
interval(500)
  .pipe(
    take(5), // 5つの値を取得
    tap((val) => {
      const progressItem = document.createElement('div');
      progressItem.textContent = `アイテム ${val + 1} を処理中...`;
      progressContainer.appendChild(progressItem);
    }),
    finalize(() => {
      loadingIndicator.style.display = 'none';
      completionMessage.textContent = '処理が完了しました！';
      completionMessage.style.color = 'green';
    })
  )
  .subscribe({
    complete: () => {
      const successMsg = document.createElement('div');
      successMsg.textContent = 'すべてのデータが正常に読み込まれました。';
      completionMessage.appendChild(successMsg);
    },
  });
```

## ✅ まとめ

- `finalize`は、**完了・エラー・手動終了を問わず必ず実行**される
- クリーンアップ処理やローディング終了処理に最適
- 他の演算子（`tap`, `delay` など）と組み合わせて、**安全に非同期処理の後始末**を実現できる
