---
description: windowは別のObservableが値を発行するタイミングで、ソースObservableをネストされたObservableに分割するRxJSオペレーターで、イベント駆動型の高度なストリーム処理に最適です。
---

# window - 別のObservableのタイミングでObservableを分割する

`window`オペレーターは、**別のObservableが値を発行するまで**ソースObservableの値をグループ化し、そのグループを**新しいObservableとして出力**します。
`buffer`が配列を返すのに対し、`window`は**Observable\<T>を返す**ため、各ウィンドウに対してさらにオペレーターを適用できます。

## 🔰 基本構文と使い方

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

// 100msごとに値を発行
const source$ = interval(100);

// クリックイベントをトリガーとして使用
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  window(clicks$),
  mergeAll() // 各ウィンドウをフラット化
).subscribe(value => {
  console.log('ウィンドウ内の値:', value);
});

// クリックするたびに新しいウィンドウが開始される
```

- `clicks$`が値を発行するたびに、新しいウィンドウ（Observable）が作成されます。
- 各ウィンドウは独立したObservableとして処理できます。

[🌐 RxJS公式ドキュメント - `window`](https://rxjs.dev/api/operators/window)

## 💡 典型的な活用パターン

- イベント駆動型のストリーム分割
- ウィンドウごとに異なる処理を適用
- 動的な区切りでのデータグループ化
- 各ウィンドウに対する集計処理

## 🔍 buffer との違い

| オペレーター | 出力 | ユースケース |
|:---|:---|:---|
| `buffer` | **配列 (T[])** | グループ化された値をまとめて処理 |
| `window` | **Observable\<T>** | グループごとに異なるストリーム処理 |

```ts
import { interval, timer } from 'rxjs';
import { buffer, window, mergeAll } from 'rxjs';

const source$ = interval(100);
const trigger$ = timer(1000, 1000);

// buffer - 配列として出力
source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('バッファ（配列）:', values);
  // 出力: バッファ（配列）: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// window - Observable として出力
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  console.log('ウィンドウ（Observable）:', window$);
  window$.subscribe(value => {
    console.log('  ウィンドウ内の値:', value);
  });
});
```

## 🧠 実践コード例1: ウィンドウごとのカウント

ボタンクリックをトリガーに、それまでのイベント数をカウントする例です。

```ts
import { interval, fromEvent } from 'rxjs';
import { window, map, mergeAll, scan } from 'rxjs';

// ボタン作成
const button = document.createElement('button');
button.textContent = 'ウィンドウを区切る';
document.body.appendChild(button);

// 出力エリア
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// 100msごとに値を発行
const source$ = interval(100);

// ボタンクリックをトリガーに
const clicks$ = fromEvent(button, 'click');

let windowCount = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const currentWindow = ++windowCount;
    console.log(`ウィンドウ ${currentWindow} 開始`);

    // 各ウィンドウの値をカウント
    return window$.pipe(
      scan((count) => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  output.textContent = `現在のウィンドウ: ${windowCount}, カウント: ${count}`;
});
```

- ボタンをクリックするたびに新しいウィンドウが作成されます。
- 各ウィンドウ内の値の個数がリアルタイムでカウントされます。

## 🎯 実践コード例2: ウィンドウごとに異なる処理

各ウィンドウに対して異なる処理を適用する高度な例です。

```ts
import { interval, fromEvent } from 'rxjs';
import { window, take, mergeAll, map } from 'rxjs';

const source$ = interval(200);
const clicks$ = fromEvent(document, 'click');

let windowNumber = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // 偶数ウィンドウ: 最初の3個のみ取得
      console.log(`ウィンドウ ${current}: 最初の3個を取得`);
      return window$.pipe(take(3));
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

- ウィンドウごとに条件分岐して異なる処理を適用できます。
- 各ウィンドウは独立したObservableなので、自由にオペレーターを組み合わせられます。

## 🎯 実用例: 複数のトリガーを使った制御

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { window, mergeAll, scan, map } from 'rxjs';

const source$ = interval(100);

// 複数のトリガー: クリックまたは3秒経過
const clicks$ = fromEvent(document, 'click');
const threeSeconds$ = timer(3000, 3000);
const trigger$ = merge(clicks$, threeSeconds$);

source$.pipe(
  window(trigger$),
  map((window$, index) => {
    console.log(`ウィンドウ ${index + 1} 開始`);

    // 各ウィンドウの合計値を計算
    return window$.pipe(
      scan((sum, value) => sum + value, 0)
    );
  }),
  mergeAll()
).subscribe(sum => {
  console.log('現在の合計:', sum);
});
```

## ⚠️ 注意点

### 1. ウィンドウのサブスクリプション管理

各ウィンドウは独立したObservableなので、明示的に購読する必要があります。

```ts
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  // ウィンドウ自体を購読しないと値は流れない
  window$.subscribe(value => {
    console.log('値:', value);
  });
});
```

または、`mergeAll()`, `concatAll()`, `switchAll()`などを使用してフラット化します。

```ts
source$.pipe(
  window(trigger$),
  mergeAll() // すべてのウィンドウをマージ
).subscribe(value => {
  console.log('値:', value);
});
```

### 2. メモリリークに注意

**問題**: トリガーObservableが値を発行しない場合、最初のウィンドウが永遠に開いたままになり、値が無限に蓄積されます。

#### ❌ 悪い例: トリガーが発生しない

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100); // 100msごとに値を発行し続ける

// ボタンが存在しない、またはユーザーがクリックしない
const button = document.querySelector('#start-button'); // null の可能性
const clicks$ = fromEvent(button, 'click'); // エラーまたは永遠に発火しない

source$.pipe(
  window(clicks$), // clicks$が発火しないと最初のウィンドウが閉じない
  mergeAll()
).subscribe();

// 問題点:
// - clicks$が値を発行しないと、最初のウィンドウがずっと開いたまま
// - source$の値（0, 1, 2, 3...）がメモリに蓄積され続ける
// - メモリリークの原因となる
```

#### ✅ 良い例1: タイムアウトを設定

最初のウィンドウが開きすぎないように、タイムアウトを設定します。

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = button ? fromEvent(button, 'click') : interval(0); // fallback to a dummy observable if button is null

// クリックまたは5秒経過のどちらか早い方でウィンドウを閉じる
const autoClose$ = timer(5000); // 5秒後に自動的に値を発行
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // 必ず5秒以内にウィンドウが閉じる
  mergeAll()
).subscribe();
```

#### ✅ 良い例2: 定期的にウィンドウを閉じる

クリックがなくても、定期的にウィンドウを閉じて新しいウィンドウを開始します。

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = fromEvent(button, 'click');

// クリックまたは3秒ごとにウィンドウを閉じる
const autoClose$ = timer(3000, 3000); // 最初の3秒後、以降3秒ごと
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // クリックがなくても3秒ごとにウィンドウが閉じる
  mergeAll()
).subscribe();

// 結果:
// - ユーザーがクリックしなくても、3秒ごとに自動的にウィンドウが閉じる
// - メモリに値が無限に蓄積されることを防ぐ
```

### 3. ウィンドウの重複

デフォルトでは、ウィンドウは重複しません（前のウィンドウが閉じてから次が開始）。
重複が必要な場合は、`windowToggle`や`windowWhen`を使用します。

## 🆚 window系オペレーターの比較

| オペレーター | 区切りのタイミング | ユースケース |
|:---|:---|:---|
| `window` | 別のObservableの発行 | イベント駆動型の分割 |
| `windowTime` | 一定時間 | 時間ベースの分割 |
| `windowCount` | 一定個数 | 個数ベースの分割 |
| `windowToggle` | 開始・終了のObservable | 動的な開始/終了制御 |
| `windowWhen` | 動的なクロージング条件 | ウィンドウごとに異なる終了条件 |

## 📚 関連オペレーター

- [`buffer`](./buffer) - 配列として値をまとめる（windowの配列版）
- [`windowTime`](./windowTime) - 時間ベースでウィンドウ分割
- [`windowCount`](./windowCount) - 個数ベースでウィンドウ分割
- [`windowToggle`](./windowToggle) - 開始・終了のObservableでウィンドウ制御
- [`windowWhen`](./windowWhen) - 動的なクロージング条件でウィンドウ分割
- [`groupBy`](./groupBy) - キーごとにObservableをグループ化

## まとめ

`window`オペレーターは、外部のObservableをトリガーとしてストリームを分割し、各グループを独立したObservableとして処理できる強力なツールです。

- ✅ 各ウィンドウに対して異なる処理を適用可能
- ✅ イベント駆動型の柔軟な制御
- ✅ 高度なストリーム操作に対応
- ⚠️ サブスクリプション管理が必要
- ⚠️ メモリリークに注意
