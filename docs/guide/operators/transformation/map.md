---
description: mapオペレーターは、Observable内の各値に関数を適用して新しい値を生成する基本的な変換手段で、フォーム整形やAPIレスポンス処理に多用されます。
---

# map - 各値に変換関数を適用する

`map`オペレーターは、ストリーム内の**各値**に指定した関数を適用し、変換後の新しい値を生成します。  
配列の`Array.prototype.map`メソッドに似ていますが、これは**非同期ストリーム上**で動作します。
 

## 🔰 基本構文と使い方

```ts
import { of } from 'rxjs';
import { map } from 'rxjs/operators';

of(1, 2, 3).pipe(
  map(value => value * 10)
).subscribe(console.log);
// 出力: 10, 20, 30
```

各値に関数 value => value * 10 を適用し、新しい値を生成します。

[🌐 RxJS公式ドキュメント - map](https://rxjs.dev/api/index/function/map)
 

## 💡 典型的な活用パターン
- APIレスポンスの変換（必要なプロパティだけ抽出）
- フォーム入力データの整形
- ストリーム内の数値や文字列を加工
- UIイベントから必要なデータだけ取り出す
 

## 🧠 実践コード例（UI付き）

入力された数値をリアルタイムで2倍して表示する例です。

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs/operators';

// 入力フィールド作成
const input = document.createElement('input');
input.type = 'number';
input.placeholder = '数値を入力';
document.body.appendChild(input);

// 出力フィールド作成
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// 入力イベントストリーム
fromEvent(input, 'input').pipe(
  map(event => Number((event.target as HTMLInputElement).value)),
  map(value => value * 2)
).subscribe(result => {
  output.textContent = `2倍の値: ${result}`;
});
```

- 入力値がリアルタイムで2倍されて出力されます。
- mapを連続適用することで、シンプルなデータ変換チェーンを実現しています。
