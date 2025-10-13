---
description: windowToggleオペレーターは、開始と終了のトリガーを別々のObservableで制御し、複数のウィンドウ期間を独立して管理できる高度なウィンドウ演算子です。
---

# windowToggle - 開始と終了を独立制御するウィンドウ

`windowToggle` オペレーターは、**開始トリガー**と**終了トリガー**を別々のObservableで制御し、各期間を新しいObservableとして発行します。複数のウィンドウ期間を同時に管理できる高度なウィンドウ演算子です。

## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500); // 0.5秒ごとに値を発行

// 開始トリガー: 2秒ごと
const opening$ = interval(2000);

// 終了トリガー: 開始から1秒後
const closing = () => interval(1000);

source$.pipe(
  windowToggle(opening$, closing),
  mergeAll()
).subscribe(value => {
  console.log('ウィンドウ内の値:', value);
});

// 2秒目に開始、3秒目に終了 → 値: 4, 5
// 4秒目に開始、5秒目に終了 → 値: 8, 9
// 6秒目に開始、7秒目に終了 → 値: 12, 13
```

**動作の流れ**:
1. `opening$` が値を発行 → ウィンドウ開始
2. `closing()` が返すObservableが値を発行 → ウィンドウ終了
3. 複数のウィンドウ期間が重複することも可能

[🌐 RxJS公式ドキュメント - `windowToggle`](https://rxjs.dev/api/operators/windowToggle)

## 💡 典型的な活用パターン

- 営業時間中のデータ収集
- ボタン押下中のイベント記録
- アクティブセッション中のアクション追跡
- 動的な期間管理が必要なストリーム処理

## 🔍 bufferToggle との違い

| オペレーター | 出力 | ユースケース |
|:---|:---|:---|
| `bufferToggle` | **配列 (T[])** | グループ化された値をまとめて処理 |
| `windowToggle` | **Observable\<T>** | グループごとに異なるストリーム処理 |

```ts
import { interval } from 'rxjs';
import { bufferToggle, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500);
const opening$ = interval(2000);
const closing = () => interval(1000);

// bufferToggle - 配列として出力
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(values => {
  console.log('バッファ（配列）:', values);
  // 出力: バッファ（配列）: [4, 5]
});

// windowToggle - Observable として出力
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  console.log('ウィンドウ（Observable）:', window$);
  window$.subscribe(value => {
    console.log('  ウィンドウ内の値:', value);
  });
});
```

## 🧠 実践コード例1: ボタン押下中のイベント記録

マウスダウンからマウスアップまでの間のデータを記録する例です。

```ts
import { fromEvent, interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

// ボタン作成
const button = document.createElement('button');
button.textContent = 'ホールド';
document.body.appendChild(button);

// 出力エリア
const display = document.createElement('div');
display.style.marginTop = '10px';
document.body.appendChild(display);

// データストリーム（100msごと）
const data$ = interval(100);

// 開始: マウスダウン
const mouseDown$ = fromEvent(button, 'mousedown');

// 終了: マウスアップ
const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

data$.pipe(
  windowToggle(mouseDown$, mouseUp),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(events => {
  display.textContent = `ホールド中に記録されたイベント: ${events.length}件`;
  console.log('記録されたデータ:', events);
});
```

## 🎯 実践コード例2: 営業時間中のデータ収集

営業開始から営業終了までのセンサーデータを収集する例です。

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, mergeMap, scan, map } from 'rxjs';

// センサーデータ（常時取得）
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10, // 20-30度
    humidity: 40 + Math.random() * 20     // 40-60%
  }))
);

// 営業開始: 2秒後、その後10秒ごと
const businessOpen$ = timer(2000, 10000);

// 営業終了: 開始から5秒後
const businessClose = () => timer(5000);

let sessionNumber = 0;

sensorData$.pipe(
  windowToggle(businessOpen$, businessClose),
  mergeMap(window$ => {
    const current = ++sessionNumber;
    console.log(`営業セッション ${current} 開始`);

    // 各ウィンドウの統計情報を計算
    return window$.pipe(
      scan((stats, data) => ({
        count: stats.count + 1,
        totalTemp: stats.totalTemp + data.temperature,
        totalHumidity: stats.totalHumidity + data.humidity
      }), { count: 0, totalTemp: 0, totalHumidity: 0 }),
      map(stats => ({
        session: current,
        count: stats.count,
        avgTemp: stats.totalTemp / stats.count,
        avgHumidity: stats.totalHumidity / stats.count
      }))
    );
  })
).subscribe(stats => {
  console.log(`セッション ${stats.session}: サンプル数 ${stats.count}件`);
  console.log(`  平均温度: ${stats.avgTemp.toFixed(1)}°C`);
  console.log(`  平均湿度: ${stats.avgHumidity.toFixed(1)}%`);
});
```

## 🎯 実用例: ダウンロード期間の管理

開始ボタンと停止ボタンで、データのダウンロード期間を管理する例です。

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { windowToggle, mergeMap, toArray, map } from 'rxjs';

// UI要素の作成
const startButton = document.createElement('button');
startButton.textContent = '開始';
document.body.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = '停止';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
document.body.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = '待機中...';
document.body.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
document.body.appendChild(result);

// データストリーム（1秒ごとにダウンロードデータを生成）
const downloadData$ = interval(1000).pipe(
  map(i => ({
    id: i,
    size: Math.floor(Math.random() * 1000) + 100, // 100-1100KB
    timestamp: new Date()
  }))
);

// 開始と終了のトリガー
const start$ = fromEvent(startButton, 'click');
const stop$ = new Subject<void>();

fromEvent(stopButton, 'click').subscribe(() => {
  stop$.next();
  status.textContent = '停止しました';
  startButton.disabled = false;
  stopButton.disabled = true;
});

start$.subscribe(() => {
  status.textContent = 'ダウンロード中...';
  startButton.disabled = true;
  stopButton.disabled = false;
});

// ウィンドウ管理
downloadData$.pipe(
  windowToggle(start$, () => stop$),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>ダウンロード完了</strong><br>
    件数: ${downloads.length}件<br>
    合計サイズ: ${(totalSize / 1024).toFixed(2)} MB<br>
    平均サイズ: ${avgSize.toFixed(0)} KB
  `;
});
```

## 🎯 重複するウィンドウ期間

`windowToggle` の特徴として、複数のウィンドウ期間を同時に管理できます。

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// 開始: 1秒ごと
const opening$ = interval(1000);

// 終了: 開始から1.5秒後
const closing = () => interval(1500);

source$.pipe(
  windowToggle(opening$, closing),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('ウィンドウ:', values);
});

// 出力:
// ウィンドウ: [4, 5, 6, 7]       （1秒目開始 → 2.5秒目終了）
// ウィンドウ: [9, 10, 11, 12]    （2秒目開始 → 3.5秒目終了）
// ウィンドウ: [14, 15, 16, 17]   （3秒目開始 → 4.5秒目終了）
```

**タイムライン**:
```
ソース:    0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
開始:      ----1秒----2秒----3秒----4秒
期間1:     [------1.5秒-----]
            └→ ウィンドウ1: [4,5,6,7]
期間2:            [------1.5秒-----]
                   └→ ウィンドウ2: [9,10,11,12]
期間3:                   [------1.5秒-----]
                          └→ ウィンドウ3: [14,15,16,17]
```

## ⚠️ 注意点

### 1. ウィンドウのサブスクリプション管理

各ウィンドウは独立したObservableなので、明示的に購読するか、`mergeAll()`などでフラット化する必要があります。

```ts
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  // ウィンドウ自体を購読しないと値は流れない
  window$.subscribe(value => {
    console.log('値:', value);
  });
});
```

### 2. メモリリークに注意

開始トリガーが頻繁すぎると、多くのウィンドウが同時に存在し、メモリを消費します。

```ts
// ❌ 悪い例: 開始が100msごと、終了が5秒後
const opening$ = interval(100); // 頻繁すぎる
const closing = () => interval(5000);

source$.pipe(
  windowToggle(opening$, closing)
).subscribe();
// 同時に50個のウィンドウが存在する可能性 → メモリリスク

// ✅ 良い例: 適切な間隔を設定
const opening$ = interval(2000); // 2秒ごと
const closing = () => interval(1000); // 1秒間
```

### 3. ウィンドウ期間の重複

ウィンドウ期間が重複すると、同じ値が複数のウィンドウに含まれます。これが意図した動作かを確認してください。

```ts
// 重複あり
opening$ = interval(1000);    // 1秒ごとに開始
closing = () => interval(1500); // 1.5秒間

// 重複なし
opening$ = interval(2000);    // 2秒ごとに開始
closing = () => interval(1000); // 1秒間
```

## 🆚 window系オペレーターの比較

| オペレーター | 制御 | ウィンドウ期間 | ユースケース |
|:---|:---|:---|:---|
| `window` | 別のObservableの発行 | 連続的 | イベント駆動型の分割 |
| `windowTime` | 一定時間 | 連続的 | 時間ベースの分割 |
| `windowCount` | 一定個数 | 連続的 | 個数ベースの分割 |
| `windowToggle` | **開始と終了を別制御** | **重複可能** | **複雑な開始/終了条件** |
| `windowWhen` | 終了のみ制御 | 連続的 | シンプルな周期的制御 |

## 🔄 windowWhen との違い

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, windowWhen, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowToggle: 開始と終了を別々に制御
source$.pipe(
  windowToggle(
    interval(1000),          // 開始トリガー
    () => timer(500)         // 終了トリガー（開始から500ms後）
  ),
  mergeAll()
).subscribe();

// windowWhen: 終了のタイミングのみを制御（終了後すぐ次が開始）
source$.pipe(
  windowWhen(() => timer(1000)), // 1秒ごとにウィンドウ
  mergeAll()
).subscribe();
```

| オペレーター | 制御 | ウィンドウ期間 | ユースケース |
|:---|:---|:---|:---|
| `windowToggle(open$, close)` | 開始と終了を別制御 | 重複可能 | 複雑な開始/終了条件 |
| `windowWhen(closing)` | 終了のみ制御 | 連続的 | シンプルな周期的ウィンドウ |

## 📚 関連オペレーター

- [`bufferToggle`](./bufferToggle) - 配列として値をまとめる（windowToggleの配列版）
- [`window`](./window) - 別のObservableのタイミングでウィンドウ分割
- [`windowTime`](./windowTime) - 時間ベースでウィンドウ分割
- [`windowCount`](./windowCount) - 個数ベースでウィンドウ分割
- [`windowWhen`](./windowWhen) - 動的なクロージング条件でウィンドウ分割

## まとめ

`windowToggle`オペレーターは、開始と終了を独立して制御し、各期間を独立したObservableとして処理できる高度なツールです。

- ✅ 開始と終了を別々に制御可能
- ✅ 複数のウィンドウを同時に管理可能
- ✅ 各ウィンドウに対して異なる処理を適用可能
- ⚠️ サブスクリプション管理が必要
- ⚠️ 頻繁な開始トリガーはメモリを消費
- ⚠️ ウィンドウ期間の重複に注意
