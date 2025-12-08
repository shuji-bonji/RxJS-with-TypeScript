---
description: "windowWhenオペレーターは、終了条件を動的に制御してObservableを分割します。ウィンドウ終了後すぐに次のウィンドウが開始されるため、連続したデータ区切りに最適です。bufferWhenとの違い、TypeScriptでの型安全な実装と実践例を解説します。"
---

# windowWhen - 動的な終了制御ウィンドウ

`windowWhen` オペレーターは、**終了条件を動的に制御**してObservableを分割します。ウィンドウが終了すると即座に次のウィンドウが開始される、連続的なストリーム処理パターンを実現します。

## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500); // 0.5秒ごとに値を発行

// 終了条件: 1秒後
const closingSelector = () => interval(1000);

source$.pipe(
  windowWhen(closingSelector),
  take(4),
  mergeAll()
).subscribe(value => {
  console.log('ウィンドウ内の値:', value);
});

// ウィンドウ1: 0       （0秒開始 → 1秒終了）
// ウィンドウ2: 1, 2    （1秒開始 → 2秒終了）
// ウィンドウ3: 3, 4    （2秒開始 → 3秒終了）
// ウィンドウ4: 5, 6    （3秒開始 → 4秒終了）
```

**動作の流れ**:
1. 最初のウィンドウが自動的に開始
2. `closingSelector()` が返すObservableが値を発行 → ウィンドウ終了
3. **即座に次のウィンドウが開始**
4. 2-3を繰り返す

[🌐 RxJS公式ドキュメント - `windowWhen`](https://rxjs.dev/api/operators/windowWhen)

## 💡 典型的な活用パターン

- 動的な時間間隔でのデータ収集
- 負荷に応じた適応的なストリーム処理
- 前回の結果に基づくウィンドウ制御
- 連続的なデータグループ化

## 🔍 bufferWhen との違い

| オペレーター | 出力 | ユースケース |
|:---|:---|:---|
| `bufferWhen` | **配列 (T[])** | グループ化された値をまとめて処理 |
| `windowWhen` | **Observable\<T>** | グループごとに異なるストリーム処理 |

```ts
import { interval } from 'rxjs';
import { bufferWhen, windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500);
const closing = () => interval(1000);

// bufferWhen - 配列として出力
source$.pipe(
  bufferWhen(closing),
  take(3)
).subscribe(values => {
  console.log('バッファ（配列）:', values);
  // 出力: バッファ（配列）: [0]
  // 出力: バッファ（配列）: [1, 2]
  // 出力: バッファ（配列）: [3, 4]
});

// windowWhen - Observable として出力
source$.pipe(
  windowWhen(closing),
  take(3),
  mergeAll()
).subscribe(value => {
  console.log('ウィンドウ内の値:', value);
  // 出力: ウィンドウ内の値: 0
  // 出力: ウィンドウ内の値: 1
  // 出力: ウィンドウ内の値: 2
  // ...
});
```

## 🧠 実践コード例1: 動的な時間間隔でのデータ収集

前回のウィンドウの結果に基づいて次のウィンドウ期間を調整する例です。

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, scan, map } from 'rxjs';

// センサーデータ（常時生成）
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10 // 20-30度
  }))
);

let windowNumber = 0;
let previousAvgTemp = 25;

sensorData$.pipe(
  windowWhen(() => {
    const current = ++windowNumber;
    // 温度が高いほど短い間隔でウィンドウ
    const duration = previousAvgTemp > 27 ? 500 : 1000;
    console.log(`ウィンドウ ${current} 開始（期間: ${duration}ms）`);
    return timer(duration);
  }),
  mergeMap(window$ => {
    const currentWindow = windowNumber;  // 現在のウィンドウ番号を保持
    return window$.pipe(
      toArray(),
      map(data => {
        const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
        previousAvgTemp = avgTemp;
        return {
          window: currentWindow,  // 保持したウィンドウ番号を使用
          count: data.length,
          avgTemp
        };
      })
    );
  })
).subscribe(stats => {
  console.log(`ウィンドウ ${stats.window}: 平均温度 ${stats.avgTemp.toFixed(1)}°C, サンプル数 ${stats.count}件`);
});
```

## 🎯 実践コード例2: 負荷に応じた適応的なストリーム処理

システムの負荷に応じて、ウィンドウの長さを動的に変更する例です。

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { windowWhen, mergeMap, scan, map } from 'rxjs';

// 出力エリア作成
const container = document.createElement('div');
document.body.appendChild(container);

const loadButton = document.createElement('button');
loadButton.textContent = '負荷を生成';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = '低負荷: 5秒間隔で収集';
container.appendChild(status);

const logDisplay = document.createElement('div');
logDisplay.style.marginTop = '10px';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// ログストリーム（常時生成）
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    timestamp: new Date()
  }))
);

// 負荷レベル
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
  const interval = getWindowDuration(loadLevel);
  const loadText = loadLevel === 0 ? '低負荷' :
                   loadLevel <= 2 ? '中負荷' : '高負荷';
  status.textContent = `${loadText} (Level ${loadLevel}): ${interval / 1000}秒間隔で収集`;
}

function getWindowDuration(load: number): number {
  // 負荷が高いほど短い間隔
  switch (load) {
    case 0: return 5000;
    case 1: return 3000;
    case 2: return 2000;
    case 3: return 1000;
    case 4: return 500;
    default: return 300;
  }
}

let windowNum = 0;

// 適応的ウィンドウ処理
logs$.pipe(
  windowWhen(() => {
    windowNum++;
    return timer(getWindowDuration(loadLevel));
  }),
  mergeMap(window$ =>
    window$.pipe(
      scan((stats, log) => ({
        count: stats.count + 1,
        errors: stats.errors + (log.level === 'ERROR' ? 1 : 0),
        window: windowNum
      }), { count: 0, errors: 0, window: windowNum })
    )
  )
).subscribe(stats => {
  const timestamp = new Date().toLocaleTimeString();
  const div = document.createElement('div');
  div.textContent = `[${timestamp}] ウィンドウ ${stats.window}: ${stats.count}件 (エラー: ${stats.errors}件)`;
  logDisplay.insertBefore(div, logDisplay.firstChild);
});
```

## 🆚 windowToggle との違い

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowWhen: 終了のみ制御（終了後すぐ次が開始）
source$.pipe(
  windowWhen(() => timer(1000)),
  mergeAll()
).subscribe();

// windowToggle: 開始と終了を別々に制御
source$.pipe(
  windowToggle(
    interval(1000),          // 開始トリガー
    () => timer(500)         // 終了トリガー（開始から500ms後）
  ),
  mergeAll()
).subscribe();
```

| オペレーター | 制御 | ウィンドウ期間 | ユースケース |
|:---|:---|:---|:---|
| `windowWhen(closing)` | 終了のみ制御 | 連続的 | シンプルな周期的ウィンドウ |
| `windowToggle(open$, close)` | 開始と終了を別制御 | 重複可能 | 複雑な開始/終了条件 |

**使い分けのポイント**:
- **`windowWhen`**: すべてのデータを漏れなく連続的に処理したい（ログ収集、データ集約など）
- **`windowToggle`**: 特定の期間だけデータを処理したい（営業時間中、ボタン押下中など）

## 🎯 実用例: 適応的なウィンドウサイズ制御

前回のウィンドウの結果に基づいて次のウィンドウ期間を自動調整する例です。

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, map } from 'rxjs';

interface WindowStats {
  count: number;
  nextDuration: number;
}

const data$ = interval(100);

let previousCount = 0;

// データ量に応じて次のウィンドウ期間を調整
function getNextDuration(count: number): number {
  if (count > 20) {
    return 500;  // データ量が多い → 短い間隔
  } else if (count > 10) {
    return 1000; // 中程度 → 中間の間隔
  } else {
    return 2000; // データ量が少ない → 長い間隔
  }
}

data$.pipe(
  windowWhen(() => timer(getNextDuration(previousCount))),
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(data => {
        previousCount = data.length;
        return {
          count: data.length,
          nextDuration: getNextDuration(data.length)
        } as WindowStats;
      })
    )
  )
).subscribe(stats => {
  console.log(`ウィンドウサイズ: ${stats.count}件, 次の期間: ${stats.nextDuration}ms`);
});
```

## ⚠️ 注意点

### 1. ウィンドウのサブスクリプション管理

各ウィンドウは独立したObservableなので、明示的に購読するか、`mergeAll()`などでフラット化する必要があります。

```ts
source$.pipe(
  windowWhen(closing)
).subscribe(window$ => {
  // ウィンドウ自体を購読しないと値は流れない
  window$.subscribe(value => {
    console.log('値:', value);
  });
});
```

### 2. 毎回新しいObservableを返す必要がある

`closingSelector` 関数は**毎回新しいObservableを返す必要**があります。同じインスタンスを返すと、正しく動作しません。

```ts
// ❌ 悪い例: 同じObservableインスタンスを使い回し
const closingObservable = timer(1000);

source$.pipe(
  windowWhen(() => closingObservable) // 2回目以降動作しない！
).subscribe();

// ✅ 良い例: 毎回新しいObservableを生成
source$.pipe(
  windowWhen(() => timer(1000)) // 毎回新しいtimerを生成
).subscribe();
```

### 3. 終了条件の複雑化に注意

終了条件が複雑になりすぎると、デバッグが困難になります。

```ts
// 複雑すぎる例
let counter = 0;
source$.pipe(
  windowWhen(() => {
    counter++;
    const duration = counter % 3 === 0 ? 500 :
                     counter % 2 === 0 ? 1000 : 1500;
    return timer(duration);
  })
).subscribe();
// デバッグが困難
```

## 🆚 window系オペレーターの比較

| オペレーター | 制御 | ウィンドウ期間 | ユースケース |
|:---|:---|:---|:---|
| `window` | 別のObservableの発行 | 連続的 | イベント駆動型の分割 |
| `windowTime` | 一定時間 | 連続的 | 時間ベースの分割 |
| `windowCount` | 一定個数 | 連続的 | 個数ベースの分割 |
| `windowToggle` | 開始と終了を別制御 | 重複可能 | 複雑な開始/終了条件 |
| `windowWhen` | **終了のみ動的制御** | **連続的** | **適応的なウィンドウ処理** |

## 📚 関連オペレーター

- [`bufferWhen`](./bufferWhen) - 配列として値をまとめる（windowWhenの配列版）
- [`window`](./window) - 別のObservableのタイミングでウィンドウ分割
- [`windowTime`](./windowTime) - 時間ベースでウィンドウ分割
- [`windowCount`](./windowCount) - 個数ベースでウィンドウ分割
- [`windowToggle`](./windowToggle) - 開始・終了のObservableでウィンドウ制御

## まとめ

`windowWhen`オペレーターは、終了条件を動的に制御し、連続的なウィンドウ処理を実現する便利なツールです。

- ✅ 終了条件を動的に制御可能
- ✅ 連続的なウィンドウ処理（データを漏らさない）
- ✅ 前回の結果に基づいて次のウィンドウを調整可能
- ⚠️ サブスクリプション管理が必要
- ⚠️ 毎回新しいObservableを返す必要がある
- ⚠️ 終了条件が複雑になりすぎないよう注意
