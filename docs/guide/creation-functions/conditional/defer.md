---
description: deferオペレーターは、Observableのファクトリ関数を購読時点まで遅延実行させます。購読のたびに異なる値や処理を評価したい場合、現在時刻やランダム値、動的なAPIリクエストなど実行タイミングで結果が変わる処理に有効です。
---

# defer - 遅延評価によるObservable作成

`defer`オペレーターは、Observableのファクトリ関数を**購読時点**で実行し、その結果のObservableを返します。これにより、実際に購読されるまでObservableの作成を遅延させることができます。

## 基本構文・動作

```ts
import { defer, of } from 'rxjs';

const random$ = defer(() => of(Math.random()));

random$.subscribe(console.log);
random$.subscribe(console.log);

// 出力:
// 0.8727962287400634
// 0.8499299688934545
```

この例では、購読のたびに `Math.random()` が評価されるため、毎回異なる値が発行されます。

[🌐 RxJS公式ドキュメント - defer](https://rxjs.dev/api/index/function/defer)

## 典型的な活用例

APIや外部リソース、現在時刻や乱数など、**実行タイミングによって結果が変わる処理**を都度行いたいときに有効です。

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchUser(userId: number) {
  return defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );
}

fetchUser(1).subscribe(console.log);

// 出力:
// {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
```

## 実践コード例（UI付き）

`defer`は副作用のある処理や毎回異なる結果を生成する処理に特に有用です。

以下のコードでは、`defer` を使って「購読されるたびに毎回異なるObservableを生成」することの意味を体験できます。  
特に、キャッシュではなく**毎回取得処理を行いたいようなケース**で便利です。

### ✅ 1. 都度ランダムな数値を生成する
```ts
import { defer, of } from 'rxjs';

// ランダムな数値を生成するObservable
const randomNumber$ = defer(() => {
  const random = Math.floor(Math.random() * 100);
  return of(random);
});

// UI要素を作成
const randomContainer = document.createElement('div');
randomContainer.innerHTML = '<h3>defer によるランダム値生成:</h3>';
document.body.appendChild(randomContainer);

// 生成ボタン
const generateButton = document.createElement('button');
generateButton.textContent = 'ランダム値を生成';
randomContainer.appendChild(generateButton);

// 履歴表示エリア
const randomHistory = document.createElement('div');
randomHistory.style.marginTop = '10px';
randomHistory.style.padding = '10px';
randomHistory.style.border = '1px solid #ddd';
randomHistory.style.maxHeight = '200px';
randomHistory.style.overflowY = 'auto';
randomContainer.appendChild(randomHistory);

// ボタンイベント
generateButton.addEventListener('click', () => {
  randomNumber$.subscribe(value => {
    const entry = document.createElement('div');
    entry.textContent = `生成された値: ${value}`;
    entry.style.padding = '5px';
    entry.style.margin = '2px 0';
    entry.style.backgroundColor = '#f5f5f5';
    entry.style.borderRadius = '3px';
    randomHistory.insertBefore(entry, randomHistory.firstChild);
  });
});

// 説明テキスト
const randomExplanation = document.createElement('p');
randomExplanation.textContent = '「ランダム値を生成」ボタンをクリックするたびに、新しいランダム値が生成されます。通常のofを使用した場合、値は最初の一度だけ生成されますが、deferを使用することで毎回新しい値を生成できます。';
randomContainer.appendChild(randomExplanation);
```

### ✅ 2. APIリクエストの都度実行

`defer` は、購読されるたびに新たな Observable を生成するため、**ユーザー入力などに応じて異なる API リクエストを実行したい場面で特に有効です**。  
例えば、以下のようなシナリオで使います。

- ✅ 動的なクエリやパラメータに応じて異なるURLで取得
- ✅ キャッシュを使わず**毎回最新のデータを取得**したい
- ✅ イベント発生時に処理を遅延評価したい

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const container = document.createElement('div');
container.innerHTML = '<h3>defer によるAPIリクエスト:</h3>';
document.body.appendChild(container);

// 入力フィールド
const input = document.createElement('input');
input.placeholder = 'ユーザーIDを入力';
container.appendChild(input);

// 実行ボタン
const button = document.createElement('button');
button.textContent = 'ユーザー情報取得';
container.appendChild(button);

// 結果表示
const resultBox = document.createElement('pre');
resultBox.style.border = '1px solid #ccc';
resultBox.style.padding = '10px';
resultBox.style.marginTop = '10px';
container.appendChild(resultBox);

// ボタンイベント
button.addEventListener('click', () => {
  const userId = input.value.trim();
  if (!userId) {
    resultBox.textContent = 'ユーザーIDを入力してください';
    return;
  }

  const user$ = defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );

  resultBox.textContent = '読み込み中...';
  user$.subscribe({
    next: (data) => (resultBox.textContent = JSON.stringify(data, null, 2)),
    error: (err) => (resultBox.textContent = `エラー: ${err.message}`),
  });
});
```

この例では、`defer` によりユーザーがボタンを押したタイミングで `ajax.getJSON()` を呼び出すようにしており、  
**`of(ajax.getJSON(...))` のように最初から評価してしまう場合と異なり、実行タイミングを完全に制御**できます。