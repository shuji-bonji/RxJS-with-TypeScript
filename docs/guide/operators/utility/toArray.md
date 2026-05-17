---
description: toArrayはObservableが完了するまでに発行したすべての値を1つの配列にまとめて発行するRxJSユーティリティオペレーターです。バッチ処理、一括取得後のUI表示、集計処理など、ストリーム全体を配列として扱いたい場面に最適です。完了するまで値を蓄積するため、無限ストリームでは使用できません。
---

# toArray - 値の配列への変換

`toArray`オペレーターは、**Observableが完了するまでに発行したすべての値を1つの配列にまとめて発行**する演算子です。  
バッチ処理や、一括取得後のUI表示、集計などに役立ちます。


## 🔰 基本構文・動作

```ts
import { of } from 'rxjs';
import { toArray } from 'rxjs';

of(1, 2, 3).pipe(
  toArray()
).subscribe(console.log);

// 出力:
// [1, 2, 3]
```

すべての値が1つの配列としてまとめられ、Observableの完了時に発行されます。

[🌐 RxJS公式ドキュメント - toArray](https://rxjs.dev/api/index/function/toArray)

## 💡 典型的な活用例

複数の非同期結果をまとめて処理したい場合や、バッチでUIに出力したい場面で活用できます。

```ts
import { interval, of } from 'rxjs';
import { take, toArray, delayWhen, delay } from 'rxjs';

interval(500)
  .pipe(
    take(5),
    delayWhen((val) => of(val).pipe(delay(val * 200))),
    toArray()
  )
  .subscribe((result) => {
    console.log('完了時にまとめて受け取る:', result);
  });

// 出力:
// 完了時にまとめて受け取る: [0, 1, 2, 3, 4]
```


## 🧪 実践コード例（UI付き）

```ts
import { interval } from 'rxjs';
import { take, toArray } from 'rxjs';

// 出力表示エリア
const toArrayOutput = document.createElement('div');
toArrayOutput.innerHTML = '<h3>toArray の例:</h3>';
document.body.appendChild(toArrayOutput);

// 個別の値表示エリア
const individualValues = document.createElement('div');
individualValues.innerHTML = '<h4>個別の値:</h4>';
toArrayOutput.appendChild(individualValues);

// 配列結果表示エリア
const arrayResult = document.createElement('div');
arrayResult.innerHTML = '<h4>配列結果:</h4>';
arrayResult.style.marginTop = '20px';
toArrayOutput.appendChild(arrayResult);

// 個別の値を購読
// 注: 以下の 2 つの subscribe は別々のストリーム（並列の購読）であり、
// ネスト subscribe ではありません。toArray あり/なしの違いを比較するために
// 意図的に 2 本のストリームを並列で購読しています。
interval(500)
  .pipe(take(5))
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `値: ${val}`;
    individualValues.appendChild(valueItem);
  });

// 同じストリームを配列として購読（上の subscribe とは独立した別ストリーム）
interval(500)
  .pipe(take(5), toArray())
  .subscribe((array) => {
    const resultItem = document.createElement('div');
    resultItem.textContent = `結果配列: [${array.join(', ')}]`;
    resultItem.style.fontWeight = 'bold';
    resultItem.style.padding = '10px';
    resultItem.style.backgroundColor = '#f5f5f5';
    resultItem.style.borderRadius = '5px';
    arrayResult.appendChild(resultItem);

    // 配列の要素を個別に表示
    const arrayItems = document.createElement('div');
    arrayItems.style.marginTop = '10px';

    array.forEach((item, index) => {
      const arrayItem = document.createElement('div');
      arrayItem.textContent = `array[${index}] = ${item}`;
      arrayItems.appendChild(arrayItem);
    });

    arrayResult.appendChild(arrayItems);
  });
```


## ✅ まとめ

- `toArray`は**完了時にすべての値を配列でまとめて発行**
- ストリーム全体を集計して扱いたい場面に最適
- `concatMap`や`delay`などと組み合わせることで、**非同期シーケンスのバッチ処理**にも対応できる
