---
description: sampleTimeオペレーターは、指定した時間間隔で定期的にストリームの最新値をサンプリングするRxJSフィルタリングオペレーターです。定期的なスナップショット取得に最適です。
---

# sampleTime - 指定した時間間隔で最新値をサンプリングする

`sampleTime` オペレーターは、**指定した時間間隔で定期的に**ソースObservableの**最新値をサンプリング**して出力します。
定期的なスナップショットのように、その時点での最新値を取得します。

## 🔰 基本構文と使い方

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs/operators';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2秒ごとのサンプル');
});
```

**動作の流れ**:
1. 2秒ごとに定期的にタイマーが発火
2. その時点で最新のクリックイベントがあれば出力
3. サンプル期間中に値がない場合は何も出力しない

[🌐 RxJS公式ドキュメント - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

## 💡 典型的な活用パターン

- **定期的なセンサーデータの取得**：毎秒の最新温度や位置情報
- **リアルタイムダッシュボード**：定期的な状態の更新
- **パフォーマンス監視**：一定間隔でのメトリクス収集
- **ゲームのフレーム処理**：FPS制御のための定期サンプリング

## 🧠 実践コード例1: マウス位置の定期サンプリング

マウスの位置を1秒ごとにサンプリングして表示する例です。

```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs/operators';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'マウス位置サンプリング（1秒ごと）';
container.appendChild(title);

const area = document.createElement('div');
area.style.width = '100%';
area.style.height = '300px';
area.style.border = '2px solid #4CAF50';
area.style.backgroundColor = '#f5f5f5';
area.style.display = 'flex';
area.style.alignItems = 'center';
area.style.justifyContent = 'center';
area.style.fontSize = '18px';
area.textContent = 'この領域内でマウスを動かしてください';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// マウス移動イベント
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // 1秒ごとにサンプリング
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>サンプル #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    位置: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // 最大10件まで表示
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- マウスを動かし続けていても、1秒ごとにその時点の最新位置だけがサンプリングされます。
- 1秒間マウスを動かさなければ、その期間は何も出力されません。

## 🎯 実践コード例2: リアルタイムデータのダッシュボード

センサーデータを定期的にサンプリングしてダッシュボードに表示する例です。

```ts
import { interval } from 'rxjs';
import { sampleTime, map } from 'rxjs/operators';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'センサーモニタリングダッシュボード';
container.appendChild(title);

const dashboard = document.createElement('div');
dashboard.style.display = 'grid';
dashboard.style.gridTemplateColumns = '1fr 1fr';
dashboard.style.gap = '10px';
dashboard.style.marginTop = '10px';
container.appendChild(dashboard);

// ダッシュボードカード作成
function createCard(label: string, unit: string) {
  const card = document.createElement('div');
  card.style.padding = '20px';
  card.style.border = '2px solid #2196F3';
  card.style.borderRadius = '8px';
  card.style.backgroundColor = '#E3F2FD';

  const labelDiv = document.createElement('div');
  labelDiv.textContent = label;
  labelDiv.style.fontSize = '14px';
  labelDiv.style.color = '#666';
  card.appendChild(labelDiv);

  const valueDiv = document.createElement('div');
  valueDiv.style.fontSize = '32px';
  valueDiv.style.fontWeight = 'bold';
  valueDiv.style.marginTop = '10px';
  valueDiv.textContent = '--';
  card.appendChild(valueDiv);

  const unitDiv = document.createElement('div');
  unitDiv.textContent = unit;
  unitDiv.style.fontSize = '14px';
  unitDiv.style.color = '#666';
  card.appendChild(unitDiv);

  dashboard.appendChild(card);
  return valueDiv;
}

const tempValue = createCard('温度', '°C');
const humidityValue = createCard('湿度', '%');
const pressureValue = createCard('気圧', 'hPa');
const lightValue = createCard('照度', 'lux');

// センサーデータストリーム（100msごとに更新）
const sensorData$ = interval(100).pipe(
  map(() => ({
    temperature: (20 + Math.random() * 10).toFixed(1),
    humidity: (40 + Math.random() * 40).toFixed(1),
    pressure: (1000 + Math.random() * 30).toFixed(1),
    light: Math.floor(Math.random() * 1000)
  }))
);

// 2秒ごとにサンプリングしてダッシュボードを更新
sensorData$.pipe(
  sampleTime(2000)
).subscribe(data => {
  tempValue.textContent = data.temperature;
  humidityValue.textContent = data.humidity;
  pressureValue.textContent = data.pressure;
  lightValue.textContent = data.light.toString();

  // アニメーション効果
  [tempValue, humidityValue, pressureValue, lightValue].forEach(elem => {
    elem.style.color = '#2196F3';
    setTimeout(() => {
      elem.style.color = 'black';
    }, 500);
  });
});
```

- センサーデータは100msごとに更新されますが、ダッシュボードは2秒ごとにサンプリングされた値で更新されます。
- 高頻度なデータストリームを適切な間隔で表示することで、パフォーマンスを最適化できます。

## 🆚 類似オペレーターとの比較

### sampleTime vs throttleTime vs auditTime

```ts
import { interval } from 'rxjs';
import { sampleTime, throttleTime, auditTime, take } from 'rxjs/operators';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, ...

// sampleTime: 1秒ごとにその時点の最新値をサンプリング
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));
// 出力例: 2, 5, 8（1秒ごとのスナップショット）

// throttleTime: 最初の値を出力後、1秒間は無視
source$.pipe(
  throttleTime(1000)
).subscribe(val => console.log('throttleTime:', val));
// 出力例: 0, 3, 6, 9（各期間の最初の値）

// auditTime: 最初の値から1秒後にその期間の最後の値を出力
source$.pipe(
  auditTime(1000)
).subscribe(val => console.log('auditTime:', val));
// 出力例: 2, 5, 8（各期間の最後の値）
```

| オペレーター | 発火タイミング | 発行する値 | ユースケース |
|:---|:---|:---|:---|
| `sampleTime(1000)` | **1秒ごとの定期的なタイミング** | その時点の最新値 | 定期的なスナップショット |
| `throttleTime(1000)` | 値受信後の1秒間は無視 | 期間開始時の最初の値 | イベントの間引き |
| `auditTime(1000)` | 値受信から1秒後 | 期間内の最後の値 | 期間内の最新状態 |

**視覚的な違い**:

```
入力: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (定期的にサンプリング)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (最初を通して期間中は無視)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (期間終了時に最後の値)
```

## ⚠️ 注意点

### 1. サンプル期間中に値がない場合

サンプルタイミングで新しい値がない場合、何も出力されません。

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs/operators';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('サンプル取得');
});
// 2秒間に1回もクリックがなければ何も出力されない
```

### 2. 最初のサンプルタイミングまで待機

`sampleTime` は指定した時間が経過するまで何も出力しません。

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs/operators';

interval(100).pipe(
  sampleTime(1000)
).subscribe(console.log);
// 最初の値は1秒後に出力される
```

### 3. 完了タイミング

ソースが完了しても、次のサンプルタイミングまで完了は伝播しません。

```ts
import { of } from 'rxjs';
import { sampleTime, delay } from 'rxjs/operators';

of(1, 2, 3).pipe(
  delay(100),
  sampleTime(1000)
).subscribe({
  next: console.log,
  complete: () => console.log('完了')
});
// 1秒後: 3
// 1秒後: 完了
```

### 4. メモリ使用

内部で最新値を1つだけ保持するため、メモリ効率は良好です。

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs/operators';

// 高頻度のストリーム（10msごと）
interval(10).pipe(
  sampleTime(1000) // 1秒ごとにサンプリング
).subscribe(console.log);
// メモリには最新の1つの値のみ保持される
```

## 💡 sample との違い

`sample` は別のObservableをトリガーとして使用しますが、`sampleTime` は固定時間間隔を使用します。

```ts
import { interval, fromEvent } from 'rxjs';
import { sample, sampleTime } from 'rxjs/operators';

const source$ = interval(100);

// sampleTime: 固定時間間隔（1秒ごと）
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));

// sample: 別のObservableをトリガーとして使用
const clicks$ = fromEvent(document, 'click');
source$.pipe(
  sample(clicks$)
).subscribe(val => console.log('sample:', val));
// クリックするたびにその時点の最新値を出力
```

| オペレーター | トリガー | ユースケース |
|:---|:---|:---|
| `sampleTime(ms)` | 固定時間間隔 | 定期的なサンプリング |
| `sample(notifier$)` | 別のObservable | 動的なタイミングでのサンプリング |

## 📚 関連オペレーター

- **[sample](https://rxjs.dev/api/operators/sample)** - 別のObservableをトリガーとしてサンプリング（公式ドキュメント）
- **[throttleTime](./throttleTime)** - 期間開始時の最初の値を取得
- **[auditTime](./auditTime)** - 期間終了時の最後の値を取得
- **[debounceTime](./debounceTime)** - 静止後に値を発行

## まとめ

`sampleTime` オペレーターは、指定した時間間隔で定期的に最新値をサンプリングします。

- ✅ 定期的なスナップショット取得に最適
- ✅ 高頻度ストリームの間引きに有効
- ✅ メモリ効率が良い（最新値1つのみ保持）
- ✅ ダッシュボードやモニタリングに最適
- ⚠️ サンプル期間中に値がないと何も出力されない
- ⚠️ 最初のサンプルまで待機時間がある
- ⚠️ 完了は次のサンプルタイミングで伝播
