---
description: timeoutは指定時間内にObservableから値が発行されない場合にエラーを投げるRxJSユーティリティオペレーターです。APIリクエストのタイムアウト制御、ユーザー操作の応答待ち、ストリームの遅延検出など、時間制約のあるリアクティブ処理に最適です。catchErrorと組み合わせてフォールバック動作を実装でき、TypeScriptの型推論により型安全なタイムアウト処理が可能です。
---

# timeout - タイムアウト設定

`timeout`オペレーターは、**指定した時間内にObservableから値が発行されない場合にエラーを投げる**オペレーターです。  
APIリクエストやユーザー操作の応答待ちなど、リアクティブな処理でよく用いられます。


## 🔰 基本構文・動作

タイムアウトにかからなければ通常通り動作し、一定時間を超えるとエラーが発生します。

```ts
import { of } from 'rxjs';
import { delay, timeout, catchError } from 'rxjs';

of('response')
  .pipe(
    delay(500), // 👈 1500に指定すると、`タイムアウトエラー: fallback`と出力される
    timeout(1000),
    catchError((err: unknown) => of('タイムアウトエラー: fallback', err))
  )
  .subscribe(console.log);
// 出力:
// response
```

この例では `delay(500)` により500ms後に値が発行され、`timeout(1000)` の条件を満たしているため、正常に `'response'` が表示されます。

`delay(1200)`と指定すると、以下のように`タイムアウトエラー`が出力される。
```sh
タイムアウトエラー: fallback
TimeoutErrorImpl {stack: 'Error\n    at _super (http://localhost:5174/node_mo…s/.vite/deps/chunk-RF6VPQMH.js?v=f6400bce:583:26)', message: 'Timeout has occurred', name: 'TimeoutError', info: {…}}
```

[🌐 RxJS公式ドキュメント - timeout](https://rxjs.dev/api/index/function/timeout)

## 💡 典型的な活用例

次の例では、**ストリームが遅延して値を発行しない場合にタイムアウトを発生させる**パターンと、**正常に発行するパターン**の両方を示しています。

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

const slow$ = interval(1500).pipe(take(3));
const fast$ = interval(500).pipe(take(3));

fast$
  .pipe(
    timeout(1000),
    catchError((err: unknown) => of('fallback: timeout occurred'))
  )
  .subscribe(console.log);

slow$
  .pipe(
    timeout(1000),
    catchError((err: unknown) => of('fallback: timeout triggered'))
  )
  .subscribe(console.log);
// 出力:
// 0
// 1
// fallback: timeout triggered
// 2
```


## 🧪 実践コード例（UI付き）

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

// 出力表示エリア
const timeoutOutput = document.createElement('div');
timeoutOutput.innerHTML = '<h3>timeout の例:</h3>';
document.body.appendChild(timeoutOutput);

// タイムアウトの成功例
const normalStream$ = interval(500).pipe(take(5));

const timeoutSuccess = document.createElement('div');
timeoutSuccess.innerHTML = '<h4>正常なストリーム (タイムアウトなし):</h4>';
timeoutOutput.appendChild(timeoutSuccess);

normalStream$
  .pipe(
    timeout(1000),
    catchError((err: unknown) => {
      const message = err instanceof Error ? err.message : String(err);
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `エラー: ${message}`;
      errorMsg.style.color = 'red';
      timeoutSuccess.appendChild(errorMsg);
      return of('エラー後のフォールバック値');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `値: ${val}`;
    timeoutSuccess.appendChild(item);
  });

// タイムアウトのエラー例
const slowStream$ = interval(1500).pipe(take(5));

const timeoutError = document.createElement('div');
timeoutError.innerHTML = '<h4>遅いストリーム (タイムアウト発生):</h4>';
timeoutOutput.appendChild(timeoutError);

slowStream$
  .pipe(
    timeout(1000),
    catchError((err: unknown) => {
      const message = err instanceof Error ? err.message : String(err);
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `エラー: ${message}`;
      errorMsg.style.color = 'red';
      timeoutError.appendChild(errorMsg);
      return of('タイムアウト後のフォールバック値');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `値: ${val}`;
    timeoutError.appendChild(item);
  });
```


## ✅ まとめ

- `timeout`は**一定時間内に発行がなければエラーを出す**制御演算子
- ネットワークAPIやUI操作待ちのタイムアウト処理に有効
- `catchError`と組み合わせることで**フォールバック動作**を指定できる
