---
description: bufferWhenオペレーターは、終了条件を動的に制御して値を配列にまとめて発行するRxJSの演算子です。バッファ終了後すぐに次のバッファが開始される連続的なバッファリングを実現します。
---

# bufferWhen - 動的な終了制御バッファ

`bufferWhen` オペレーターは、**終了条件を動的に制御**して値を配列にまとめて発行します。バッファが終了すると即座に次のバッファが開始される、連続的なバッファリングパターンを実現します。


## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(500); // 0.5秒ごとに値を発行

// 終了条件: 1秒後
const closingSelector = () => interval(1000);

source$.pipe(
  bufferWhen(closingSelector),
  take(4)
).subscribe(console.log);
// 出力:
// [0]           （0秒開始 → 1秒終了、値0のみ）
// [1, 2, 3]     （1秒開始 → 2秒終了、値1,2,3）
// [4, 5]        （2秒開始 → 3秒終了、値4,5）
// [6, 7]        （3秒開始 → 4秒終了、値6,7）
```

**動作の流れ**:
1. 最初のバッファが自動的に開始
2. `closingSelector()` が返すObservableが値を発行 → バッファ終了、配列を出力
3. **即座に次のバッファが開始**（source$の発行と同時になることが多い）
4. 2-3を繰り返す

> [!NOTE]
> 最初のバッファは `interval(1000)` が最初の値を発行するまでの1秒間なので `[0]` のみです。2番目以降はバッファ開始と `source$` の発行が同時になるため、より多くの値が含まれます。

[🌐 RxJS公式ドキュメント - `bufferWhen`](https://rxjs.dev/api/operators/bufferWhen)


## 🆚 bufferToggle との違い

`bufferWhen` と `bufferToggle` は似ていますが、**制御方法と動作パターンが大きく異なります**。

### bufferWhen の動作

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // 0-11を300msごとに発行

// bufferWhen: 終了のみ制御（終了後すぐ次が開始）
source$.pipe(
  bufferWhen(() => interval(1000))
).subscribe(console.log);
// 出力: [0, 1, 2], [3, 4, 5], [6, 7, 8, 9], [10, 11]
//
// タイムライン:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms 3600ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//  [----------1秒----------][----------1秒----------][----------1秒----------][-----1秒-----]
//   バッファ1(0-2)           バッファ2(3-5)           バッファ3(6-9)          バッファ4(10-11)
//   連続的・重複なし・即座に次が開始
```

### bufferToggle の動作

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // 0-11を300msごとに発行

// bufferToggle: 開始と終了を別制御（重複可能）
const opening$ = interval(1000); // 1秒ごとに開始
const closing = () => interval(800); // 開始から800ms後に終了

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// 出力: [3, 4, 5], [6, 7, 8], [9, 10, 11]
//
// タイムライン:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//        ----開始1(1000ms)----[---800ms後終了(1800ms)---]
//                        3      4      5
//                        └→ [3,4,5]
//                    ----開始2(2000ms)----[---800ms後終了(2800ms)---]
//                                            6      7      8
//                                            └→ [6,7,8]
//                              ----開始3(3000ms)----[---800ms後終了(3800ms)---]
//                                                      9      10     11
//                                                      └→ [9,10,11]
//  開始トリガーを待ち、期間は独立（最初の0-2はバッファ開始前なので含まれない）
```

### 主な違い

| オペレーター | 開始制御 | 終了制御 | バッファ期間 | 特徴 |
|---|---|---|---|---|
| `bufferWhen(closing)` | 自動（終了後すぐ） | 動的 | 連続的 | バッファ間の隙間なし |
| `bufferToggle(open$, close)` | 独立したObservable | 動的 | 独立・重複可能 | バッファ間に隙間あり |

**使い分けのポイント**:
- **`bufferWhen`**: すべてのデータを漏れなく連続的にバッファリングしたい（ログ収集、データ集約など）
- **`bufferToggle`**: 特定の期間だけデータを収集したい（営業時間中、ボタン押下中など）

> [!TIP]
> - **連続的なバッファリング**（データを漏らさない） → `bufferWhen`
> - **期間限定のバッファリング**（開始/終了を明示的に制御） → `bufferToggle`


## 💡 典型的な活用パターン

1. **動的な時間間隔でのデータ収集**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // センサーデータ
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       temperature: 20 + Math.random() * 10
     }))
   );

   // 終了条件: 前回の温度に応じて動的に変更
   let previousAvgTemp = 25;

   sensorData$.pipe(
     bufferWhen(() => {
       // 温度が高いほど短い間隔でバッファ
       const duration = previousAvgTemp > 27 ? 500 : 1000;
       return timer(duration);
     })
   ).subscribe(data => {
     const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
     previousAvgTemp = avgTemp;
     console.log(`平均温度: ${avgTemp.toFixed(1)}°C, サンプル数: ${data.length}`);
   });
   ```

2. **負荷に応じた適応的なバッチ処理**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   interface Task {
     id: number;
     timestamp: number;
   }

   // タスクストリーム
   let taskCounter = 0;
   const tasks$ = fromEvent(document, 'click').pipe(
     map(() => ({
       id: taskCounter++,
       timestamp: Date.now()
     } as Task))
   );

   // バッファサイズに応じて次のバッファ期間を調整
   tasks$.pipe(
     bufferWhen(() => timer(2000))
   ).subscribe(bufferedTasks => {
     if (bufferedTasks.length > 0) {
       console.log(`バッチ処理: ${bufferedTasks.length}件のタスク`);
       console.log('タスクID:', bufferedTasks.map(t => t.id));

       // 次のバッファ期間を動的に決定
       // （実際はこのロジックをbufferWhenの関数内に移動）
     }
   });
   ```

3. **ランダムな間隔でのサンプリング**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // データストリーム
   const data$ = interval(100).pipe(
     map(i => ({
       value: Math.sin(i / 10) * 100,
       timestamp: Date.now()
     }))
   );

   // ランダムな間隔（500ms〜2000ms）でバッファ
   data$.pipe(
     bufferWhen(() => {
       const randomDelay = 500 + Math.random() * 1500;
       return timer(randomDelay);
     })
   ).subscribe(samples => {
     const avg = samples.reduce((sum, s) => sum + s.value, 0) / samples.length;
     console.log(`サンプル数: ${samples.length}, 平均値: ${avg.toFixed(2)}`);
   });
   ```


## 🧠 実践コード例（負荷に応じたログ収集）

システムの負荷に応じて、ログ収集の頻度を動的に変更する例です。

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { bufferWhen, map, share } from 'rxjs';

// UI要素の作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = '適応的ログ収集システム';
container.appendChild(title);

const loadButton = document.createElement('button');
loadButton.textContent = '負荷を生成';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#f0f0f0';
status.textContent = '低負荷: 5秒間隔で収集';
container.appendChild(status);

const logDisplay = document.createElement('pre');
logDisplay.style.marginTop = '10px';
logDisplay.style.padding = '10px';
logDisplay.style.backgroundColor = '#f9f9f9';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// ログストリーム（常時生成）
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    message: `Log message ${logCounter}`,
    timestamp: new Date()
  })),
  share()
);

// 負荷カウンター（ボタンクリックでインクリメント）
let loadLevel = 0;
fromEvent(loadButton, 'click').subscribe(() => {
  loadLevel = Math.min(loadLevel + 1, 5);
  updateStatus();
});

// 30秒ごとに負荷を減少
interval(30000).subscribe(() => {
  loadLevel = Math.max(loadLevel - 1, 0);
  updateStatus();
});

function updateStatus() {
  const interval = getBufferInterval(loadLevel);
  const loadText = loadLevel === 0 ? '低負荷' :
                   loadLevel <= 2 ? '中負荷' : '高負荷';
  status.textContent = `${loadText} (Level ${loadLevel}): ${interval / 1000}秒間隔で収集`;
  status.style.backgroundColor =
    loadLevel === 0 ? '#d4edda' :
    loadLevel <= 2 ? '#fff3cd' : '#f8d7da';
}

function getBufferInterval(load: number): number {
  // 負荷が高いほど短い間隔でバッファ
  switch (load) {
    case 0: return 5000;  // 5秒
    case 1: return 3000;  // 3秒
    case 2: return 2000;  // 2秒
    case 3: return 1000;  // 1秒
    case 4: return 500;   // 0.5秒
    default: return 300;  // 0.3秒
  }
}

// 適応的バッファリング
logs$.pipe(
  bufferWhen(() => timer(getBufferInterval(loadLevel)))
).subscribe(bufferedLogs => {
  if (bufferedLogs.length > 0) {
    const errors = bufferedLogs.filter(log => log.level === 'ERROR').length;
    const timestamp = new Date().toLocaleTimeString();

    const summary = `[${timestamp}] 収集: ${bufferedLogs.length}件 (エラー: ${errors}件)\n`;
    logDisplay.textContent = summary + logDisplay.textContent;

    console.log('収集されたログ:', bufferedLogs);
  }
});
```


## 📋 型安全な使い方

TypeScript でジェネリクスを活用した型安全な実装例です。

```ts
import { Observable, interval, timer } from 'rxjs';
import { bufferWhen, map } from 'rxjs';

interface MetricData {
  value: number;
  timestamp: Date;
  source: string;
}

interface BufferConfig {
  minDuration: number;
  maxDuration: number;
  adaptive: boolean;
}

class AdaptiveBuffer<T> {
  constructor(private config: BufferConfig) {}

  private getNextBufferDuration(previousCount: number): number {
    if (!this.config.adaptive) {
      return this.config.minDuration;
    }

    // データ量に応じて次のバッファ期間を調整
    const ratio = Math.min(previousCount / 10, 1);
    const duration =
      this.config.minDuration +
      (this.config.maxDuration - this.config.minDuration) * (1 - ratio);

    return Math.floor(duration);
  }

  apply(source$: Observable<T>): Observable<T[]> {
    let previousCount = 0;

    return source$.pipe(
      bufferWhen(() => {
        const duration = this.getNextBufferDuration(previousCount);
        return timer(duration);
      }),
      map(buffer => {
        previousCount = buffer.length;
        return buffer;
      })
    );
  }
}

// 使用例
const metricsStream$ = interval(300).pipe(
  map(i => ({
    value: Math.random() * 100,
    timestamp: new Date(),
    source: `sensor-${i % 3}`
  } as MetricData))
);

const buffer = new AdaptiveBuffer<MetricData>({
  minDuration: 1000,  // 最小1秒
  maxDuration: 5000,  // 最大5秒
  adaptive: true      // 適応的
});

buffer.apply(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avg = metrics.reduce((sum, m) => sum + m.value, 0) / metrics.length;
    console.log(`バッファサイズ: ${metrics.length}, 平均値: ${avg.toFixed(2)}`);
  }
});
```


## 🎯 他のバッファ系オペレーターとの比較

```ts
import { interval, timer, Subject } from 'rxjs';
import { buffer, bufferTime, bufferCount, bufferWhen, bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0-9

// 1. buffer: 固定トリガー
const trigger$ = new Subject<void>();
source$.pipe(buffer(trigger$)).subscribe(console.log);
setInterval(() => trigger$.next(), 1000);
// 出力: [0, 1, 2], [3, 4, 5], ... (トリガーのタイミングで)

// 2. bufferTime: 固定時間間隔
source$.pipe(bufferTime(1000)).subscribe(console.log);
// 出力: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 3. bufferCount: 固定個数
source$.pipe(bufferCount(3)).subscribe(console.log);
// 出力: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 4. bufferWhen: 動的な終了制御（連続的）
source$.pipe(
  bufferWhen(() => timer(1000))
).subscribe(console.log);
// 出力: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 5. bufferToggle: 開始と終了を独立制御（重複可能）
const opening$ = interval(1000);
const closing = () => timer(800);
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// 出力: [3, 4, 5], [6, 7, 8]
```

| オペレーター | トリガー | 動的制御 | 重複 | ユースケース |
|---|---|---|---|---|
| `buffer` | 外部Observable | ❌ | ❌ | イベントドリブン |
| `bufferTime` | 固定時間 | ❌ | ❌ | 定期的な集約 |
| `bufferCount` | 固定個数 | ❌ | ❌ | 定量的な処理 |
| `bufferWhen` | 動的（終了のみ） | ✅ | ❌ | 適応的なバッチ処理 |
| `bufferToggle` | 動的（開始と終了） | ✅ | ✅ | 複雑な期間管理 |


## ⚠️ よくある間違い

> [!WARNING]
> `bufferWhen` の終了条件関数は**毎回新しいObservableを返す必要**があります。同じObservableインスタンスを返すと、正しく動作しません。

### 誤: 同じObservableインスタンスを返す

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ❌ 悪い例: 同じObservableインスタンスを使い回し
const closingObservable = timer(1000);

source$.pipe(
  bufferWhen(() => closingObservable) // 2回目以降動作しない！
).subscribe(console.log);
// 最初のバッファのみ出力され、以降は出力されない
```

### 正: 毎回新しいObservableを返す

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ✅ 良い例: 毎回新しいObservableを生成
source$.pipe(
  bufferWhen(() => timer(1000)) // 毎回新しいtimerを生成
).subscribe(console.log);
// 出力: [0, 1], [2, 3], [4, 5], ...
```

> [!IMPORTANT]
> `closingSelector` 関数は、前のバッファが終了するたびに**必ず呼び出され**、新しいObservableを返すことが期待されています。


## 🎓 まとめ

### bufferWhen を使うべき場合
- ✅ 終了条件を動的に制御したい場合
- ✅ 連続的なバッファリング期間が必要な場合
- ✅ 前回のバッファ結果に基づいて次の期間を調整したい場合
- ✅ 適応的なバッチ処理を実装したい場合

### bufferToggle を使うべき場合
- ✅ 開始と終了を独立して制御したい場合
- ✅ バッファ期間が重複する可能性がある場合
- ✅ ボタン押下中など、明確な開始/終了イベントがある場合

### bufferTime を使うべき場合
- ✅ 固定時間間隔でのバッファリングで十分な場合
- ✅ シンプルな実装が求められる場合

### 注意点
- ⚠️ `closingSelector` は毎回新しいObservableを返す必要がある
- ⚠️ 終了条件が複雑になりすぎると、デバッグが困難になる
- ⚠️ 適応的な制御では、予期しない動作を避けるためテストが重要


## 🚀 次のステップ

- **[buffer](./buffer)** - 基本的なバッファリングを学ぶ
- **[bufferTime](./bufferTime)** - 時間ベースのバッファリングを学ぶ
- **[bufferCount](./bufferCount)** - 個数ベースのバッファリングを学ぶ
- **[bufferToggle](./bufferToggle)** - 開始と終了を独立制御するバッファリングを学ぶ
- **[変換オペレーター実践例](./practical-use-cases)** - 実際のユースケースを学ぶ
