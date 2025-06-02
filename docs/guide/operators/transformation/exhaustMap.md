---
description: exhaustMapは現在のObservableが完了するまで新たな入力を無視するRxJSオペレーターで、重複リクエストの防止などに効果的です。
---

# exhaustMap - 実行中は新しい入力を無視する

`exhaustMap`オペレーターは、現在処理中のObservableが完了するまで、**新しい入力を無視**します。  
重複クリック防止や、リクエストの多重送信防止に最適です。

## 🔰 基本構文と使い方

```ts
import { fromEvent, of } from 'rxjs';
import { exhaustMap, delay } from 'rxjs/operators';

const clicks$ = fromEvent(document, 'click');

clicks$
  .pipe(exhaustMap(() => of('リクエスト完了').pipe(delay(1000))))
  .subscribe(console.log);

// 出力例:
// （最初のクリックのみ1秒後に「リクエスト完了」が出力される）

```

- 実行中のリクエストが完了するまで、後続の入力は無視されます。

[🌐 RxJS公式ドキュメント - `exhaustMap`](https://rxjs.dev/api/operators/exhaustMap)

## 💡 典型的な活用パターン

- フォーム送信ボタンの多重クリック防止
- 二重リクエスト防止（特にログイン・決済処理など）
- モーダルやダイアログの単一表示制御

## 🧠 実践コード例（UI付き）

送信ボタンをクリックすると、送信処理を開始します。  
**送信中に何度クリックしても無視**され、最初の処理が完了するまで次の送信は受け付けません。

```ts
import { fromEvent } from 'rxjs';
import { exhaustMap, delay } from 'rxjs/operators';
import { ajax } from 'rxjs/ajax';

// ボタン作成
const submitButton = document.createElement('button');
submitButton.textContent = '送信';
document.body.appendChild(submitButton);

// 出力エリア作成
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// 送信処理
fromEvent(submitButton, 'click')
  .pipe(
    exhaustMap(() => {
      output.textContent = '送信中...';
      return ajax
        .post('https://jsonplaceholder.typicode.com/posts', {
          title: 'foo',
          body: 'bar',
          userId: 1,
        })
        .pipe(delay(2000)); // 2秒間の送信遅延をシミュレート
    })
  )
  .subscribe({
    next: (response) => {
      output.textContent = '送信成功！';
      console.log('送信成功:', response);
    },
    error: (error) => {
      output.textContent = '送信エラー';
      console.error('送信エラー:', error);
    },
  });

```

- ボタンクリック中に他のクリックがあっても無視されます。
- 2秒後に「送信成功！」または「送信エラー」と表示されます。
