---
description: retryオペレーターは、Observableでエラーが発生した際にソースを指定回数だけ再購読して再試行します。ネットワーク障害などの一時的な通信失敗から回復させたい場合や、失敗しても再試行すれば成功する可能性がある処理に有効です。
---

# retry - エラー時の再試行

`retry`オペレーターは、**エラーが発生した際にソースObservableを指定回数だけ再購読**するオペレーターです。  
一時的なネットワーク障害など、**失敗しても再試行すれば成功する可能性がある処理**に向いています。

## 🔰 基本構文・動作

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError } from 'rxjs';

throwError(() => new Error('一時的なエラー'))
  .pipe(
    retry(2), // 最大2回まで再試行
    catchError((error) => of(`最終エラー: ${error.message}`))
  )
  .subscribe(console.log);
// 出力:
// 最終エラー: 一時的なエラー
```

この例では、最初の失敗後に2回まで再試行され、すべて失敗した場合にフォールバックでメッセージが出力されます。

[🌐 RxJS公式ドキュメント - retry](https://rxjs.dev/api/index/function/retry)

## 💡 典型的な活用例

以下の例は、**ランダムに成功/失敗する非同期処理**を3回まで再試行する構成です。

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

let attempt = 0;

interval(1000)
  .pipe(
    mergeMap(() => {
      attempt++;
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        return throwError(() => new Error(`失敗 #${attempt}`));
      } else {
        return of(`成功 #${attempt}`);
      }
    }),
    retry(3),
    catchError((err) => of(`最終失敗: ${err.message}`))
  )
  .subscribe(console.log);
// 出力:
// 成功 #1
// 成功 #5
// 成功 #6
// 最終失敗: 失敗 #7
```

## 🧪 実践コード例（UI付き）

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

// 出力表示エリア
const retryOutput = document.createElement('div');
retryOutput.innerHTML = '<h3>retry の例 (APIリクエストシミュレーション):</h3>';
document.body.appendChild(retryOutput);

// リクエスト状態表示
const requestStatus = document.createElement('div');
requestStatus.style.marginTop = '10px';
requestStatus.style.padding = '10px';
requestStatus.style.border = '1px solid #ddd';
requestStatus.style.maxHeight = '200px';
requestStatus.style.overflowY = 'auto';
retryOutput.appendChild(requestStatus);

// ランダムに成功または失敗するAPIリクエスト
let attemptCount = 0;

function simulateRequest() {
  attemptCount++;

  const logEntry = document.createElement('div');
  logEntry.textContent = `試行 #${attemptCount} リクエスト送信中...`;
  requestStatus.appendChild(logEntry);

  return interval(1000).pipe(
    mergeMap(() => {
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        const errorMsg = document.createElement('div');
        errorMsg.textContent = `試行 #${attemptCount} 失敗: ネットワークエラー`;
        errorMsg.style.color = 'red';
        requestStatus.appendChild(errorMsg);

        return throwError(() => new Error('ネットワークエラー'));
      } else {
        const successMsg = document.createElement('div');
        successMsg.textContent = `試行 #${attemptCount} 成功！`;
        successMsg.style.color = 'green';
        requestStatus.appendChild(successMsg);

        return of({ id: 1, name: 'データが正常に取得されました' });
      }
    }),
    retry(3),
    catchError((err) => {
      const finalError = document.createElement('div');
      finalError.textContent = `すべての再試行に失敗しました: ${err.message}`;
      finalError.style.color = 'red';
      finalError.style.fontWeight = 'bold';
      requestStatus.appendChild(finalError);

      return of({ error: true, message: '再試行が失敗しました' });
    })
  );
}

// リクエスト開始ボタン
const startButton = document.createElement('button');
startButton.textContent = 'リクエスト開始';
startButton.style.padding = '8px 16px';
startButton.style.marginTop = '10px';
retryOutput.insertBefore(startButton, requestStatus);

startButton.addEventListener('click', () => {
  attemptCount = 0;
  requestStatus.innerHTML = '';
  startButton.disabled = true;

  simulateRequest().subscribe((result) => {
    const resultElement = document.createElement('div');
    if ('error' in result) {
      resultElement.textContent = `最終結果: ${result.message}`;
      resultElement.style.backgroundColor = '#ffebee';
    } else {
      resultElement.textContent = `最終結果: ${result.name}`;
      resultElement.style.backgroundColor = '#e8f5e9';
    }

    resultElement.style.padding = '10px';
    resultElement.style.marginTop = '10px';
    resultElement.style.borderRadius = '5px';
    requestStatus.appendChild(resultElement);

    startButton.disabled = false;
  });
});
```

## ✅ まとめ

- `retry(n)` は、Observableがエラーを出した場合に最大`n`回まで再試行する
- `retry`は**正常に完了するまで再実行される**（失敗が続くとエラーが出る）
- 一時的な障害が起こる**非同期APIやネットワークリクエスト**に有効
- `catchError`と組み合わせて**フォールバック処理**を指定するのが一般的