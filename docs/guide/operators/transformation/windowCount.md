---
description: windowCountは指定した個数ごとにObservableを分割するRxJS変換オペレーターです。個数ベースのストリーム処理や一定件数ごとの集計、ページネーション処理に最適で、bufferCountと異なり各ウィンドウに独立した処理を適用できます。TypeScriptの型推論により型安全なウィンドウ分割とストリーム操作が可能です。
---

# windowCount - 個数で分割

`windowCount`オペレーターは、発行された値を指定した個数ごとに**新しいObservableとして分割**します。
`bufferCount`が配列を返すのに対し、`windowCount`は**Observable\<T>を返す**ため、各ウィンドウに対してさらにオペレーターを適用できます。

## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { windowCount, mergeAll } from 'rxjs';

// 100msごとに値を発行
const source$ = interval(100);

source$.pipe(
  windowCount(5),
  mergeAll() // 各ウィンドウをフラット化
).subscribe(value => {
  console.log('ウィンドウ内の値:', value);
});

// 出力:
// ウィンドウ内の値: 0
// ウィンドウ内の値: 1
// ウィンドウ内の値: 2
// ウィンドウ内の値: 3
// ウィンドウ内の値: 4
// (新しいウィンドウ開始)
// ウィンドウ内の値: 5
// ...
```

- 5個の値ごとに新しいウィンドウ（Observable）が作成されます。
- 個数ベースで分割する点が特徴です。

[🌐 RxJS公式ドキュメント - `windowCount`](https://rxjs.dev/api/operators/windowCount)

## 💡 典型的な活用パターン

- 一定件数ごとの集計処理
- データのバッチ送信（ウィンドウごとに異なる処理）
- ページネーション処理
- ウィンドウごとに統計情報を計算

## 🔍 bufferCount との違い

| オペレーター | 出力 | ユースケース |
|:---|:---|:---|
| `bufferCount` | **配列 (T[])** | グループ化された値をまとめて処理 |
| `windowCount` | **Observable\<T>** | グループごとに異なるストリーム処理 |

```ts
import { interval } from 'rxjs';
import { bufferCount, windowCount, mergeAll } from 'rxjs';

const source$ = interval(100);

// bufferCount - 配列として出力
source$.pipe(
  bufferCount(5)
).subscribe(values => {
  console.log('バッファ（配列）:', values);
  // 出力: バッファ（配列）: [0, 1, 2, 3, 4]
});

// windowCount - Observable として出力
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  console.log('ウィンドウ（Observable）:', window$);
  window$.subscribe(value => {
    console.log('  ウィンドウ内の値:', value);
  });
});
```

## 🧠 実践コード例1: ウィンドウごとの合計値

5個ごとの値の合計を計算する例です。

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, reduce } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>5個ごとの合計値</h3>';
document.body.appendChild(output);

const source$ = interval(200);

let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;
    console.log(`ウィンドウ ${current} 開始`);

    // 各ウィンドウの合計を計算
    return window$.pipe(
      reduce((sum, value) => sum + value, 0),
      map(sum => ({ windowNum: current, sum }))  // ウィンドウ番号を含める
    );
  }),
  mergeAll()
).subscribe(result => {
  const div = document.createElement('div');
  div.textContent = `ウィンドウ ${result.windowNum} の合計: ${result.sum}`;
  output.appendChild(div);
});

// 出力:
// ウィンドウ 1 の合計: 10  (0+1+2+3+4)
// ウィンドウ 2 の合計: 35  (5+6+7+8+9)
// ウィンドウ 3 の合計: 60  (10+11+12+13+14)
```

## 🎯 実践コード例2: スタートインデックスの指定

第2引数で開始インデックスを指定できます。オーバーラップウィンドウを作成できます。

```ts
import { range } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// 0から9までの値を発行
range(0, 10).pipe(
  windowCount(3, 2), // 3個ずつ、2個ずつずらして開始
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('ウィンドウ:', values);
});

// 出力:
// ウィンドウ: [0, 1, 2]
// ウィンドウ: [2, 3, 4]    ← 2個ずらして開始（2から）
// ウィンドウ: [4, 5, 6]    ← 2個ずらして開始（4から）
// ウィンドウ: [6, 7, 8]
// ウィンドウ: [8, 9]       ← 最後は2個
```

### スタートインデックスの動作パターン

```ts
// windowCount(bufferSize, startBufferEvery)

windowCount(3, 3) // 連続（デフォルト）: [0,1,2], [3,4,5], [6,7,8]
windowCount(3, 2) // オーバーラップ: [0,1,2], [2,3,4], [4,5,6]
windowCount(3, 4) // ギャップあり: [0,1,2], [4,5,6], [8,9,10]
```

## 🎯 実用例: ウィンドウごとに異なる処理

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, take } from 'rxjs';

const source$ = interval(100);
let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // 偶数ウィンドウ: 最初の2個のみ取得
      console.log(`ウィンドウ ${current}: 最初の2個を取得`);
      return window$.pipe(take(2));
    } else {
      // 奇数ウィンドウ: すべて取得
      console.log(`ウィンドウ ${current}: すべて取得`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`値: ${value} (ウィンドウ ${windowNumber})`);
});
```

## 🧠 実践コード例3: ページネーション風の処理

```ts
import { from } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// 1-20までのデータ
const data$ = from(Array.from({ length: 20 }, (_, i) => i + 1));

// 5件ずつページ分割
data$.pipe(
  windowCount(5),
  mergeMap((window$, index) => {
    const pageNumber = index + 1;
    return window$.pipe(
      toArray(),
      map(items => ({ page: pageNumber, items }))
    );
  })
).subscribe(page => {
  console.log(`ページ ${page.page}:`, page.items);
});

// 出力:
// ページ 1: [1, 2, 3, 4, 5]
// ページ 2: [6, 7, 8, 9, 10]
// ページ 3: [11, 12, 13, 14, 15]
// ページ 4: [16, 17, 18, 19, 20]
```

## ⚠️ 注意点

### 1. ウィンドウのサブスクリプション管理

各ウィンドウは独立したObservableなので、明示的に購読する必要があります。

```ts
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  // ウィンドウ自体を購読しないと値は流れない
  window$.subscribe(value => {
    console.log('値:', value);
  });
});
```

または、`mergeAll()`, `concatAll()`, `switchAll()`などを使用してフラット化します。

### 2. 最後のウィンドウ

ソースObservableの完了時、最後のウィンドウは指定個数未満でも出力されます。

```ts
import { of } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

of(1, 2, 3, 4, 5, 6, 7).pipe(
  windowCount(3),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('ウィンドウ:', values);
});

// 出力:
// ウィンドウ: [1, 2, 3]
// ウィンドウ: [4, 5, 6]
// ウィンドウ: [7]  ← 1個のみ
```

### 3. スタートインデックスによるメモリ使用

`startBufferEvery`が`bufferSize`より小さい場合（オーバーラップ）、複数のウィンドウが同時にアクティブになり、メモリ使用量が増加します。

```ts
// オーバーラップ: 最大2つのウィンドウが同時にアクティブ
windowCount(5, 3)

// 対策: 必要に応じてtake()で制限
source$.pipe(
  take(100), // 最大100個
  windowCount(5, 3)
)
```

## 🆚 window系オペレーターの比較

| オペレーター | 区切りのタイミング | ユースケース |
|:---|:---|:---|
| `window` | 別のObservableの発行 | イベント駆動型の分割 |
| `windowTime` | 一定時間 | 時間ベースの分割 |
| `windowCount` | **一定個数** | **個数ベースの分割** |
| `windowToggle` | 開始・終了のObservable | 動的な開始/終了制御 |
| `windowWhen` | 動的なクロージング条件 | ウィンドウごとに異なる終了条件 |

## 📚 関連オペレーター

- [`bufferCount`](./bufferCount) - 配列として値をまとめる（windowCountの配列版）
- [`window`](./window) - 別のObservableのタイミングでウィンドウ分割
- [`windowTime`](./windowTime) - 時間ベースでウィンドウ分割
- [`windowToggle`](./windowToggle) - 開始・終了のObservableでウィンドウ制御
- [`windowWhen`](./windowWhen) - 動的なクロージング条件でウィンドウ分割

## まとめ

`windowCount`オペレーターは、個数ベースでストリームを分割し、各グループを独立したObservableとして処理できる便利なツールです。

- ✅ 一定個数ごとの集計・処理に最適
- ✅ 各ウィンドウに対して異なる処理を適用可能
- ✅ スタートインデックスでオーバーラップ可能
- ⚠️ サブスクリプション管理が必要
- ⚠️ オーバーラップ時のメモリ使用に注意
