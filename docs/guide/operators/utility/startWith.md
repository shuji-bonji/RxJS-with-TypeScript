# startWith - 初期値の提供

`startWith` オペレーターは、**ソースObservableが値を発行する前に指定した初期値を先に発行する**ための演算子です。  
状態管理、初期表示、プレースホルダー値などに活用されます。


## 🔰 基本構文・動作

```ts
import { of } from 'rxjs';
import { startWith } from 'rxjs/operators';

of('B', 'C').pipe(
  startWith('A')
).subscribe(console.log);
// 出力:
// A
// B
// C
```

このように、`startWith`は最初に `'A'` を追加し、その後ソースObservableの値が続きます。


## 💡 典型的な活用例

状態やカウンターの初期値を設定したい場合に便利です。以下は、初期値 `100` から始まるカウンターの例です。

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs/operators';

interval(1000)
  .pipe(
    startWith(-1), // 最初に -1 を挿入
    scan((acc, curr) => acc + 1, 100), // 初期値100からインクリメント
    take(10) // 合計10回出力
  )
  .subscribe(console.log);
// 出力:
// 101
// 102
// 103
// 104
// 105
// 106
// 107
// 108
// 109
// 110
```


## 🧪 実践コード例（UI付き）

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs/operators';

// 出力表示エリア
const startWithOutput = document.createElement('div');
startWithOutput.innerHTML = '<h3>startWith の例:</h3>';
document.body.appendChild(startWithOutput);

// カウンター表示エリア
const counterDisplay = document.createElement('div');
counterDisplay.style.fontSize = '24px';
counterDisplay.style.fontWeight = 'bold';
counterDisplay.style.textAlign = 'center';
counterDisplay.style.padding = '20px';
counterDisplay.style.border = '1px solid #ddd';
counterDisplay.style.borderRadius = '5px';
counterDisplay.style.margin = '10px 0';
startWithOutput.appendChild(counterDisplay);

// 値のリスト表示エリア
const valuesList = document.createElement('div');
valuesList.style.marginTop = '10px';
startWithOutput.appendChild(valuesList);

// カウンターストリーム (1秒ごと)
interval(1000)
  .pipe(
    // 最初に100から開始
    startWith(-1),
    // 各値を前の値に1を加える
    scan((acc, curr) => acc + 1, 100),
    // 10回で終了
    take(10)
  )
  .subscribe((count) => {
    // カウンター表示を更新
    counterDisplay.textContent = count.toString();

    // 値をリストに追加
    const valueItem = document.createElement('div');

    if (count === 100) {
      valueItem.textContent = `初期値: ${count} (startWithで追加)`;
      valueItem.style.color = 'blue';
    } else {
      valueItem.textContent = `次の値: ${count}`;
    }

    valuesList.appendChild(valueItem);
  });
```


## ✅ まとめ

- `startWith`は**最初に固定値を挿入**したい場面に便利
- 状態初期化、UIプレースホルダー、フォームの初期表示などによく使われる
- `scan`や`combineLatest`などと組み合わせて、**状態管理の基盤構築**にも活用される