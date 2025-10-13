---
description: bufferToggleオペレーターは、開始と終了のトリガーを別々のObservableで制御し、複数のバッファリング期間を独立して管理できる高度なバッファリング演算子です。
---

# bufferToggle - 開始と終了を独立制御するバッファ

`bufferToggle` オペレーターは、**開始トリガー**と**終了トリガー**を別々のObservableで制御し、値を配列にまとめて発行します。複数のバッファリング期間を同時に管理できる高度なバッファリング演算子です。


## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // 0.5秒ごとに値を発行

// 開始トリガー: 2秒ごと
const opening$ = interval(2000);

// 終了トリガー: 開始から1秒後
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// 出力:
// [3, 4, 5]     （2秒目に開始、3秒目に終了）
// [7, 8, 9]     （4秒目に開始、5秒目に終了）
// [11, 12, 13]  （6秒目に開始、7秒目に終了）
```

**動作の流れ**:
1. `opening$` が値を発行 → バッファリング開始
2. `closing()` が返すObservableが値を発行 → バッファリング終了、配列を出力
3. 複数のバッファリング期間が重複することも可能

[🌐 RxJS公式ドキュメント - `bufferToggle`](https://rxjs.dev/api/operators/bufferToggle)


## 🆚 他のバッファ系オペレーターとの対比

`bufferToggle` は他のバッファ系オペレーターと比べて、**開始と終了を独立制御**できる点が特徴です。

### 各オペレーターの比較

| オペレーター | トリガー | 特徴 | ユースケース |
|---|---|---|---|
| `buffer(trigger$)` | 単一のObservable | シンプル | イベントドリブンなバッファリング |
| `bufferTime(ms)` | 時間 | 定期的 | 一定間隔でのデータ集約 |
| `bufferCount(n)` | 個数 | 定量的 | N個単位での処理 |
| `bufferToggle(open$, close)` | 開始と終了を別制御 | 柔軟 | 複雑な期間管理 |

### コード例での比較

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0-9を300msごとに発行

// bufferToggle: 開始と終了を独立制御
const opening$ = interval(1000); // 1秒ごとに開始
const closing = () => interval(500); // 開始から500ms後に終了

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// 出力: [3, 4], [6, 7], [9]
//
// タイムライン:
// 0ms  300ms 600ms 900ms 1200ms 1500ms 1800ms 2100ms 2400ms 2700ms
// 0    1     2     3     4      5      6      7      8      9
//                  [開始        終了]  [開始        終了]  [開始  終了]
//                  └→ [3,4]           └→ [6,7]           └→ [9]
```

**他のオペレーターとの使い分け**:
- **`buffer`** → トリガーObservableが値を発行するたびにバッファを出力
- **`bufferTime`** → 一定時間ごとに自動でバッファを出力
- **`bufferCount`** → 指定個数たまったらバッファを出力
- **`bufferToggle`** → 開始と終了を別々に制御、重複期間も可能

> [!TIP]
> 各オペレーターの詳細は、[buffer](./buffer)、[bufferTime](./bufferTime)、[bufferCount](./bufferCount) を参照してください。


## 💡 典型的な活用パターン

1. **営業時間中のデータ収集**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // センサーデータ（常時取得）
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       value: Math.random() * 100
     }))
   );

   // 営業開始: 9:00（シミュレーション: 2秒後）
   const businessOpen$ = timer(2000, 10000); // 2秒後、その後10秒ごと

   // 営業終了: 開始から5秒後
   const businessClose = () => timer(5000);

   sensorData$.pipe(
     bufferToggle(businessOpen$, businessClose)
   ).subscribe(data => {
     console.log(`営業時間中のデータ: ${data.length}件`);
     console.log(`平均値: ${(data.reduce((sum, d) => sum + d.value, 0) / data.length).toFixed(2)}`);
   });
   ```

2. **ボタン押下中のイベント記録**
   ```ts
   import { fromEvent, interval } from 'rxjs';
   import { bufferToggle, map, take } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'ホールド';
   document.body.appendChild(button);

   const display = document.createElement('div');
   display.style.marginTop = '10px';
   document.body.appendChild(display);

   // データストリーム
   const data$ = interval(100).pipe(
     map(i => ({ id: i, timestamp: Date.now() }))
   );

   // 開始: マウスダウン
   const mouseDown$ = fromEvent(button, 'mousedown');

   // 終了: マウスアップ（mousedownから発生するmouseupまで）
   const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

   data$.pipe(
     bufferToggle(mouseDown$, mouseUp)
   ).subscribe(events => {
     display.textContent = `ホールド中に記録されたイベント: ${events.length}件`;
     console.log('記録されたイベント:', events);
   });
   ```

3. **アクティブユーザーのアクション記録**
   ```ts
   import { fromEvent, merge, timer } from 'rxjs';
    mport { bufferToggle, map } from 'rxjs';

   // ユーザーアクション
   const clicks$ = fromEvent(document, 'click').pipe(
     map(() => ({ type: 'click' as const, timestamp: Date.now() }))
   );

   const scrolls$ = fromEvent(window, 'scroll').pipe(
     map(() => ({ type: 'scroll' as const, timestamp: Date.now() }))
   );

   const keypresses$ = fromEvent(document, 'keypress').pipe(
     map(() => ({ type: 'keypress' as const, timestamp: Date.now() }))
   );

   const actions$ = merge(clicks$, scrolls$, keypresses$);

   // アクティブ状態の開始: 最初のアクション
   const activeStart$ = actions$;

   // アクティブ状態の終了: 5秒間アクションがない
   const activeEnd = () => timer(5000);

   actions$.pipe(
     bufferToggle(activeStart$, activeEnd)
   ).subscribe(bufferedActions => {
     console.log(`アクティブセッション: ${bufferedActions.length}件のアクション`);
     const summary = bufferedActions.reduce((acc, action) => {
       acc[action.type] = (acc[action.type] || 0) + 1;
       return acc;
     }, {} as Record<string, number>);
     console.log('内訳:', summary);
   });
   ```


## 🧠 実践コード例（ダウンロード期間の管理）

開始ボタンと停止ボタンで、データのダウンロード期間を管理する例です。

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { bufferToggle, map, take } from 'rxjs';

// UI要素の作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'データダウンロード管理';
container.appendChild(title);

const startButton = document.createElement('button');
startButton.textContent = '開始';
container.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = '停止';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
container.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = '待機中...';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

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

// バッファリング
downloadData$.pipe(
  bufferToggle(start$, () => stop$)
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>ダウンロード完了</strong><br>
    件数: ${downloads.length}件<br>
    合計サイズ: ${(totalSize / 1024).toFixed(2)} MB<br>
    平均サイズ: ${avgSize.toFixed(0)} KB
  `;

  console.log('ダウンロードデータ:', downloads);
});
```


## 🎯 重複するバッファ期間

`bufferToggle` の特徴として、複数のバッファリング期間を同時に管理できます。

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// 開始: 1秒ごと
const opening$ = interval(1000);

// 終了: 開始から1.5秒後
const closing = () => interval(1500);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// 出力:
// [4, 5, 6]        （1秒目開始 → 2.5秒目終了）
// [9, 10, 11, 12]  （2秒目開始 → 3.5秒目終了）※一部重複
// [14, 15, 16, 17] （3秒目開始 → 4.5秒目終了）
```

**タイムライン**:
```
ソース:    0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
開始:      ----1秒----2秒----3秒----4秒
期間1:     [------1.5秒-----]
            └→ 出力: [4,5,6]
期間2:            [------1.5秒-----]
                   └→ 出力: [9,10,11,12]
期間3:                   [------1.5秒-----]
                          └→ 出力: [14,15,16,17]
```


## 📋 型安全な使い方

TypeScript でジェネリクスを活用した型安全な実装例です。

```ts
import { Observable, Subject, interval } from 'rxjs';
import { bufferToggle, map } from 'rxjs';

interface MetricData {
  timestamp: Date;
  cpu: number;
  memory: number;
}

interface SessionControl {
  start$: Observable<void>;
  stop$: Observable<void>;
}

class MetricsCollector {
  private startSubject = new Subject<void>();
  private stopSubject = new Subject<void>();

  start(): void {
    this.startSubject.next();
  }

  stop(): void {
    this.stopSubject.next();
  }

  collectMetrics(source$: Observable<MetricData>): Observable<MetricData[]> {
    return source$.pipe(
      bufferToggle(
        this.startSubject,
        () => this.stopSubject
      )
    );
  }
}

// 使用例
const metricsStream$ = interval(500).pipe(
  map(() => ({
    timestamp: new Date(),
    cpu: Math.random() * 100,
    memory: Math.random() * 100
  } as MetricData))
);

const collector = new MetricsCollector();

collector.collectMetrics(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avgCpu = metrics.reduce((sum, m) => sum + m.cpu, 0) / metrics.length;
    const avgMemory = metrics.reduce((sum, m) => sum + m.memory, 0) / metrics.length;
    console.log(`収集期間: ${metrics.length}件`);
    console.log(`平均CPU: ${avgCpu.toFixed(1)}%`);
    console.log(`平均メモリ: ${avgMemory.toFixed(1)}%`);
  }
});

// 3秒後に開始
setTimeout(() => {
  console.log('収集開始');
  collector.start();
}, 3000);

// 6秒後に停止
setTimeout(() => {
  console.log('収集停止');
  collector.stop();
}, 6000);
```


## 🔄 bufferWhen との違い

`bufferToggle` と `bufferWhen` は似ていますが、制御方法が異なります。

```ts
import { interval, timer } from 'rxjs';
import { bufferToggle, bufferWhen } from 'rxjs';

const source$ = interval(200);

// bufferToggle: 開始と終了を別々に制御
source$.pipe(
  bufferToggle(
    interval(1000),          // 開始トリガー
    () => timer(500)         // 終了トリガー（開始から500ms後）
  )
).subscribe(console.log);

// bufferWhen: 終了のタイミングのみを制御（終了後すぐ次が開始）
source$.pipe(
  bufferWhen(() => timer(1000)) // 1秒ごとにバッファ
).subscribe(console.log);
```

| オペレーター | 制御 | バッファ期間 | ユースケース |
|---|---|---|---|
| `bufferToggle(open$, close)` | 開始と終了を別制御 | 重複可能 | 複雑な開始/終了条件 |
| `bufferWhen(closing)` | 終了のみ制御 | 連続的 | シンプルな周期的バッファ |


## ⚠️ よくある間違い

> [!WARNING]
> `bufferToggle` は複数のバッファ期間を同時に管理できますが、開始トリガーが頻繁に発火すると多くのバッファが同時に存在し、メモリを消費します。

### 誤: 開始トリガーが頻繁すぎる

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ❌ 悪い例: 開始が100msごと、終了が5秒後
const opening$ = interval(100); // 頻繁すぎる
const closing = () => interval(5000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// 同時に50個のバッファが存在する可能性あり → メモリリスク
```

### 正: 適切な間隔を設定

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ✅ 良い例: 開始を適切な間隔に
const opening$ = interval(2000); // 2秒ごと
const closing = () => interval(1000); // 1秒間バッファ

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// 最大でも同時に1-2個のバッファのみ
```


## 🎓 まとめ

### bufferToggle を使うべき場合
- ✅ 開始と終了を独立して制御したい場合
- ✅ ボタン押下中などの期間限定でデータを収集したい場合
- ✅ 複数のバッファリング期間を同時に管理したい場合
- ✅ 営業時間中のみなど、複雑な条件でのデータ収集

### buffer/bufferTime/bufferCount を使うべき場合
- ✅ シンプルな定期的なバッファリングで十分な場合
- ✅ 単一のトリガーで制御できる場合

### bufferWhen を使うべき場合
- ✅ 終了条件のみを動的に制御したい場合
- ✅ 連続的なバッファリング期間が必要な場合

### 注意点
- ⚠️ 開始トリガーが頻繁だと、多くのバッファが同時に存在しメモリを消費
- ⚠️ バッファリング期間が重複する可能性がある
- ⚠️ 複雑な制御が必要なため、デバッグが難しい場合がある


## 🚀 次のステップ

- **[buffer](./buffer)** - 基本的なバッファリングを学ぶ
- **[bufferTime](./bufferTime)** - 時間ベースのバッファリングを学ぶ
- **[bufferCount](./bufferCount)** - 個数ベースのバッファリングを学ぶ
- **[bufferWhen](https://rxjs.dev/api/operators/bufferWhen)** - 動的な終了制御を学ぶ（公式ドキュメント）
- **[変換オペレーター実践例](./practical-use-cases)** - 実際のユースケースを学ぶ
