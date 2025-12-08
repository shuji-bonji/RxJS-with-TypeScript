---
description: "defaultIfEmptyオペレーターは、Observableが値を発行しなかった場合にデフォルト値を返します。空のAPIレスポンス処理、初期値補完、検索結果がない場合のフォールバックなど、実践的なユースケースとTypeScriptでの型安全な実装を解説します。"
---

# defaultIfEmpty - ストリームが空の場合のデフォルト値

`defaultIfEmpty` オペレーターは、**Observable が何も値を発行せずに complete された場合に、指定したデフォルト値を発行する**演算子です。  
空の配列や空の API 結果に対応するために使用されます。

## 🔰 基本構文・動作

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

from([]).pipe(
  defaultIfEmpty('値がありません')
).subscribe(console.log);

// 出力:
// 値がありません
```

この例では、空の配列を `from` で Observable 化したものに対し、`defaultIfEmpty` により `'値がありません'` が出力されます。

[🌐 RxJS公式ドキュメント - defaultIfEmpty](https://rxjs.dev/api/index/function/defaultIfEmpty)

## 💡 典型的な活用例

- ユーザーが何も入力しなかった場合
- APIが空の結果を返した場合
- 条件を満たす値が一つもなかった場合

などにおいて、**"なにも返ってこなかった" という状況を補完**するために使用されます。

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of(['A', 'B', 'C']).pipe(delay(500))
    : EMPTY.pipe(delay(500));
}

mockApiCall(false)
  .pipe(defaultIfEmpty('データがありません'))
  .subscribe(console.log);

// 出力:
// データがありません
```

## 🧪 実践コード例（UI付き）

### ✅ 1. 配列の空判定に使う

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

// UI構築
const container = document.createElement('div');
container.innerHTML = '<h3>defaultIfEmpty オペレーターの例:</h3>';
document.body.appendChild(container);

const emptyBtn = document.createElement('button');
emptyBtn.textContent = '空の配列を処理';
container.appendChild(emptyBtn);

const nonEmptyBtn = document.createElement('button');
nonEmptyBtn.textContent = '非空の配列を処理';
container.appendChild(nonEmptyBtn);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

emptyBtn.addEventListener('click', () => {
  result.textContent = '処理中...';
  from([]).pipe(
    defaultIfEmpty('データがありません')
  ).subscribe(value => {
    result.textContent = `結果: ${value}`;
  });
});

nonEmptyBtn.addEventListener('click', () => {
  result.textContent = '処理中...';
  from([1, 2, 3]).pipe(
    defaultIfEmpty('データがありません')
  ).subscribe(value => {
    result.textContent = `結果: ${value}`;
  });
});
```

### ✅ 2. APIの空結果にデフォルトを補完する

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of([
        { id: 1, name: 'Item 1' },
        { id: 2, name: 'Item 2' },
      ]).pipe(delay(1000))
    : EMPTY.pipe(delay(1000));
}

const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>defaultIfEmpty を使った API 結果処理:</h3>';
document.body.appendChild(apiContainer);

const dataBtn = document.createElement('button');
dataBtn.textContent = 'データあり';
dataBtn.style.marginRight = '10px';
apiContainer.appendChild(dataBtn);

const emptyBtn2 = document.createElement('button');
emptyBtn2.textContent = 'データなし';
apiContainer.appendChild(emptyBtn2);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
apiContainer.appendChild(output);

dataBtn.addEventListener('click', () => {
  output.textContent = '取得中...';
  mockApiCall(true)
    .pipe(defaultIfEmpty('データが見つかりませんでした'))
    .subscribe({
      next: (val) => {
        if (Array.isArray(val)) {
          const ul = document.createElement('ul');
          val.forEach((item) => {
            const li = document.createElement('li');
            li.textContent = `${item.id}: ${item.name}`;
            ul.appendChild(li);
          });
          output.innerHTML = '<h4>取得結果:</h4>';
          output.appendChild(ul);
        } else {
          output.textContent = val;
        }
      },
    });
});

emptyBtn2.addEventListener('click', () => {
  output.textContent = '取得中...';
  mockApiCall(false)
    .pipe(defaultIfEmpty('データが見つかりませんでした'))
    .subscribe({
      next: (val) => {
        output.textContent = val.toString();
      },
    });
});

```