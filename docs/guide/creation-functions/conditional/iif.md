---
description: iifオペレーターは、条件式に応じて2つのObservableのうちどちらかを選択するRxJSの条件分岐演算子で、三項演算子のような使い方が可能です。
---

# iif - 条件に基づくObservableの選択

`iif`オペレーターは、条件式の評価結果に基づいて、2つのObservableのうちどちらかを選択します。  
JavaScriptの三項演算子（`condition ? trueValue : falseValue`）に似た機能を持ちます。


## 基本構文・動作

```ts
import { iif, of } from 'rxjs';

function getData(condition: boolean) {
  return iif(() => condition, of('YES'), of('NO'));
}

getData(true).subscribe(console.log);

// 出力:
// YES
```

条件が `true` の場合 `'YES'` が、`false` の場合 `'NO'` が返されます。

[🌐 RxJS公式ドキュメント - iif](https://rxjs.dev/api/index/function/iif)

## 典型的な活用例

`iif` はよく `EMPTY` と組み合わせて、条件を満たさない場合に「何も発行しないストリーム」を返す使い方がされます。

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(
    () => value > 0,
    of(`正の値: ${value}`),
    EMPTY
  );
}

conditionalData(0).subscribe(console.log);
conditionalData(1).subscribe(console.log);

// 出力:
// 正の値: 1
```


## 実践コード例（UI付き）

以下のUI付きコード例では、ユーザーの操作や数値入力に応じて  
Observableの発行内容や発行の有無を `iif` によって動的に切り替えています。

このようなパターンは、次のような実務ユースケースに適しています。

- ✅ 入力値に応じて API リクエストを抑制（例: 数値が0以下なら送信しない）
- ✅ 設定フラグにより画面表示や処理モードを切り替える
- ✅ 条件に基づく確認表示やモーダル制御

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(() => value > 0, of(`正の値: ${value}`), EMPTY);
}

// 条件に基づいて異なるObservableを返す
function getDataBasedOnCondition(condition: boolean) {
  return iif(() => condition, of('条件はtrueです'), of('条件はfalseです'));
}

// UI要素を作成
const iifContainer = document.createElement('div');
iifContainer.innerHTML = '<h3>iif オペレーターの例:</h3>';
document.body.appendChild(iifContainer);

const trueButton = document.createElement('button');
trueButton.textContent = 'True条件で実行';
trueButton.style.marginRight = '10px';
iifContainer.appendChild(trueButton);

const falseButton = document.createElement('button');
falseButton.textContent = 'False条件で実行';
iifContainer.appendChild(falseButton);

const iifResult = document.createElement('div');
iifResult.style.marginTop = '10px';
iifResult.style.padding = '10px';
iifResult.style.border = '1px solid #ddd';
iifContainer.appendChild(iifResult);

trueButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(true).subscribe((result) => {
    iifResult.textContent = result;
    iifResult.style.color = 'green';
  });
});

falseButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(false).subscribe((result) => {
    iifResult.textContent = result;
    iifResult.style.color = 'red';
  });
});

// EMPTYとの組み合わせ例（数値による条件分岐）
const emptyContainer = document.createElement('div');
emptyContainer.innerHTML = '<h3>iif と EMPTY の組み合わせ:</h3>';
document.body.appendChild(emptyContainer);

const valueInput = document.createElement('input');
valueInput.type = 'number';
valueInput.placeholder = '数値を入力';
valueInput.style.marginRight = '10px';
emptyContainer.appendChild(valueInput);

const checkButton = document.createElement('button');
checkButton.textContent = '実行';
emptyContainer.appendChild(checkButton);

const emptyResult = document.createElement('div');
emptyResult.style.marginTop = '10px';
emptyResult.style.padding = '10px';
emptyResult.style.border = '1px solid #ddd';
emptyContainer.appendChild(emptyResult);

checkButton.addEventListener('click', () => {
  const value = Number(valueInput.value);
  emptyResult.textContent = '';

  conditionalData(value).subscribe({
    next: (result) => {
      emptyResult.textContent = result;
      emptyResult.style.color = 'green';
    },
    complete: () => {
      if (!emptyResult.textContent) {
        emptyResult.textContent =
          '0以下の値が入力されたため、何も発行されませんでした';
        emptyResult.style.color = 'gray';
      }
    },
  });
});

```