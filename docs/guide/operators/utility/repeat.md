---
description: repeatオペレーターは、Observableが正常完了した後に全体を指定回数繰り返す演算子で、ポーリングや繰り返しアニメーションに活用されます。
---

# repeat - ストリームの繰り返し

`repeat`オペレーターは、ソースObservableが**正常に完了したあと**に、**指定した回数だけストリーム全体を再実行**します。  
ポーリング処理や繰り返しアニメーション、リトライとは異なる制御に役立ちます。

## 🔰 基本構文・動作

最もシンプルな使い方は、値の列を一定回数繰り返す構成です。

```ts
import { of } from 'rxjs';
import { repeat } from 'rxjs/operators';

of('A', 'B')
  .pipe(
    repeat(2) // 全体を2回繰り返す（合計2回出力される）
  )
  .subscribe(console.log);
// 出力:
// A
// B
// A
// B
```

[🌐 RxJS公式ドキュメント - repeat](https://rxjs.dev/api/index/function/repeat)

## 💡 典型的な活用例

たとえば、簡単なポーリング処理や、表示アニメーションの繰り返しなどに使われます。

```ts
import { of } from 'rxjs';
import { tap, delay, repeat } from 'rxjs/operators';

of('✅ データ取得成功')
  .pipe(
    tap(() => console.log('リクエスト開始')),
    delay(1000),
    repeat(3) // 3回繰り返す
  )
  .subscribe(console.log);
// 出力:
// リクエスト開始
// ✅ データ取得成功
// main.ts:6 リクエスト開始
// ✅ データ取得成功
// main.ts:6 リクエスト開始
// ✅ データ取得成功
```

この例では、1秒おきに「リクエスト → データ取得」が3回繰り返されます。

## 🧪 実践コード例（UI付き）

```ts
import { of } from 'rxjs';
import { repeat, tap } from 'rxjs/operators';

// 出力表示エリア
const repeatOutput = document.createElement('div');
repeatOutput.innerHTML = '<h3>repeat の例:</h3>';
document.body.appendChild(repeatOutput);

// 繰り返し回数の表示
let repeatCount = 0;
const repeatCountDisplay = document.createElement('div');
repeatCountDisplay.textContent = `繰り返し回数: ${repeatCount}`;
repeatCountDisplay.style.fontWeight = 'bold';
repeatOutput.appendChild(repeatCountDisplay);

// 値の出力エリア
const valuesOutput = document.createElement('div');
valuesOutput.style.marginTop = '10px';
valuesOutput.style.padding = '10px';
valuesOutput.style.border = '1px solid #ddd';
valuesOutput.style.maxHeight = '200px';
valuesOutput.style.overflowY = 'auto';
repeatOutput.appendChild(valuesOutput);

// シーケンスの繰り返し
of('A', 'B', 'C')
  .pipe(
    tap(() => {
      repeatCount++;
      repeatCountDisplay.textContent = `繰り返し回数: ${repeatCount}`;
    }),
    repeat(3)
  )
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `値: ${val} (繰り返し ${repeatCount})`;
    valuesOutput.appendChild(valueItem);
  });

```

## ✅ まとめ

- `repeat`は、**Observableが正常に完了したあとに全体を再実行**する
- `retry`とは異なり、**エラー時には再実行されない**
- ポーリング処理や**プレースホルダーの点滅**など、繰り返しアニメーションにも使える
