---
description: "isEmptyオペレーターは、Observableが値を発行せずに完了したかどうかを判定します。空データの検出、条件分岐、データ存在チェックに活用されます。filter()で結果が空になった場合の検出、TypeScriptでの型安全な実装を実践的なコード例で解説します。"
---

# isEmpty - ストリームが空かどうかを判定

`isEmpty` オペレーターは、Observableが**値を1つも発行しないまま完了した場合に `true` を発行**します。  
値を1つでも発行すれば `false` を発行して完了します。

## 🔰 基本構文・動作

```ts
import { of, EMPTY } from 'rxjs';
import { isEmpty } from 'rxjs';

EMPTY.pipe(isEmpty()).subscribe(console.log); // 出力: true
of(1).pipe(isEmpty()).subscribe(console.log); // 出力: false
```

[🌐 RxJS公式ドキュメント - isEmpty](https://rxjs.dev/api/index/function/isEmpty)

## 💡 典型的な活用例

- フィルタリングの結果や検索結果が空であるかどうかを判定したいとき
- 空である場合にエラーを出したり、別の処理へ切り替えたいとき

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

from([1, 3, 5])
  .pipe(
    filter((x) => x % 2 === 0),
    isEmpty()
  )
  .subscribe((result) => {
    console.log('空かどうか:', result);
  });

// 出力:
// 空かどうか: true
```

## 🧪 実践コード例（UI付き）

### ✅ 1. 空の結果かどうかを判定する

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>isEmpty オペレーターの例:</h3>';
document.body.appendChild(container);

const checkButton = document.createElement('button');
checkButton.textContent = '偶数が含まれているか確認';
container.appendChild(checkButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
container.appendChild(output);

checkButton.addEventListener('click', () => {
  from([1, 3, 5])
    .pipe(
      filter((x) => x % 2 === 0),
      isEmpty()
    )
    .subscribe((isEmptyResult) => {
      output.textContent = isEmptyResult
        ? '偶数は含まれていませんでした。'
        : '偶数が含まれています。';
      output.style.color = isEmptyResult ? 'red' : 'green';
    });
});
```

### ✅ 2. ユーザーの検索結果が空かをチェックする

```ts
import { fromEvent, of, from } from 'rxjs';
import { debounceTime, switchMap, map, filter, isEmpty, delay } from 'rxjs';

const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>isEmpty を使った検索結果チェック:</h3>';
document.body.appendChild(searchContainer);

const input = document.createElement('input');
input.placeholder = '検索ワードを入力';
input.style.marginBottom = '10px';
searchContainer.appendChild(input);

const resultBox = document.createElement('div');
resultBox.style.padding = '10px';
resultBox.style.border = '1px solid #ccc';
searchContainer.appendChild(resultBox);

const mockData = ['apple', 'banana', 'orange', 'grape'];

fromEvent(input, 'input')
  .pipe(
    debounceTime(300),
    map((e) => (e.target as HTMLInputElement).value.trim().toLowerCase()),
    filter((text) => text.length > 0),
    switchMap((query) =>
      of(mockData).pipe(
        delay(300),
        map((list) => list.filter((item) => item.includes(query))),
        switchMap((filtered) => from(filtered).pipe(isEmpty()))
      )
    )
  )
  .subscribe((noResults) => {
    resultBox.textContent = noResults
      ? '一致する項目はありませんでした'
      : '一致する項目が見つかりました';
    resultBox.style.color = noResults ? 'red' : 'green';
  });
```

> [!WARNING] 本番コードでの注意
> 上記サンプルは説明の簡略化のため `fromEvent` の購読解除を省略しています。実コードでは `takeUntil(destroy$)`、`take(N)`、もしくは `Subscription.unsubscribe()` で明示的にライフサイクル管理してください。詳細: [困難点克服: ライフサイクル管理](/guide/overcoming-difficulties/lifecycle-management.md)
