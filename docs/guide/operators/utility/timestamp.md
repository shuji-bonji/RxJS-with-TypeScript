---
description: timestampオペレーターは各値にタイムスタンプを付与し、値が発行された時刻を記録することでパフォーマンス測定やデバッグに活用できます。
---

# timestamp - タイムスタンプの付与

`timestamp` オペレーターは、ストリーム内の各値に**タイムスタンプを付与**します。値が発行された正確な時刻を記録することで、パフォーマンス測定やデバッグ、イベントの時系列分析に活用できます。

## 🔰 基本構文・動作

各値をタイムスタンプ付きのオブジェクトに変換します。

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

interval(1000)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(console.log);

// 出力:
// { value: 0, timestamp: 1640000000000 }
// { value: 1, timestamp: 1640000001000 }
// { value: 2, timestamp: 1640000002000 }
```

返されるオブジェクトは以下の構造を持ちます:
- `value`: 元の値
- `timestamp`: タイムスタンプ（ミリ秒単位のUnix時刻）

[🌐 RxJS公式ドキュメント - timestamp](https://rxjs.dev/api/index/function/timestamp)

## 💡 典型的な活用例

- **パフォーマンス測定**: 処理時間の計測
- **イベントのタイミング分析**: ユーザー操作の間隔測定
- **デバッグとロギング**: 値の発行タイミングの記録
- **時系列データの記録**: センサーデータなどのタイムスタンプ付き保存

## 🧪 実践コード例1: クリック間隔の測定

ユーザーのクリック間隔を測定する例です。

```ts
import { fromEvent } from 'rxjs';
import { timestamp, pairwise, map } from 'rxjs';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'timestamp - クリック間隔測定';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'クリックしてください';
button.style.marginBottom = '10px';
button.style.padding = '10px 20px';
button.style.fontSize = '16px';
container.appendChild(button);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '250px';
output.style.overflow = 'auto';
container.appendChild(output);

let clickCount = 0;

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.insertBefore(logItem, output.firstChild);  // 最新を上に表示
}

fromEvent(button, 'click')
  .pipe(
    timestamp(),
    pairwise(),
    map(([prev, curr]) => {
      const interval = curr.timestamp - prev.timestamp;
      return {
        clickNumber: clickCount + 1,
        interval: interval,
        timestamp: new Date(curr.timestamp).toLocaleTimeString('ja-JP')
      };
    })
  )
  .subscribe(data => {
    clickCount++;
    const color = data.interval < 500 ? '#ffcdd2' :
                  data.interval < 1000 ? '#fff9c4' : '#c8e6c9';

    const speed = data.interval < 500 ? '高速クリック!' :
                  data.interval < 1000 ? '普通' : 'ゆっくり';

    addLog(
      `${data.clickNumber}回目: ${data.interval}ms間隔 [${speed}] (${data.timestamp})`,
      color
    );
  });

addLog('ボタンをクリックしてください（2回目から間隔を測定）', '#e3f2fd');
```

- クリック間隔を正確に測定
- 速度に応じて色分け表示
- タイムスタンプで発生時刻を記録

## 🧪 実践コード例2: 処理時間の測定

各処理にかかった時間を測定する例です。

```ts
import { interval } from 'rxjs';
import { timestamp, map, take, tap } from 'rxjs';

// UI作成
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'timestamp - 処理時間測定';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.fontSize = '12px';
  logItem.style.fontFamily = 'monospace';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

addLog2('処理開始...');

interval(500)
  .pipe(
    take(5),
    timestamp(),  // 処理前のタイムスタンプ
    map(data => {
      const start = data.timestamp;

      // 重い処理をシミュレート（ランダムな処理時間）
      const iterations = Math.floor(Math.random() * 5000000) + 1000000;
      let sum = 0;
      for (let i = 0; i < iterations; i++) {
        sum += i;
      }

      const end = Date.now();
      const duration = end - start;

      return {
        value: data.value,
        startTime: new Date(start).toLocaleTimeString('ja-JP', { hour12: false }) +
                   '.' + (start % 1000).toString().padStart(3, '0'),
        duration: duration
      };
    })
  )
  .subscribe({
    next: result => {
      addLog2(
        `値${result.value}: 開始=${result.startTime}, 処理時間=${result.duration}ms`
      );
    },
    complete: () => {
      addLog2('--- すべての処理が完了 ---');
    }
  });
```

- 各値の処理開始時刻を記録
- 処理にかかった時間を測定
- パフォーマンス分析に活用

## 🧪 実践コード例3: イベントログ

すべてのイベントにタイムスタンプを付けてログ出力する例です。

```ts
import { merge, fromEvent, interval } from 'rxjs';
import { timestamp, map, take } from 'rxjs';

// UI作成
const container3 = document.createElement('div');
container3.style.marginTop = '20px';
document.body.appendChild(container3);

const title3 = document.createElement('h3');
title3.textContent = 'timestamp - イベントログ';
container3.appendChild(title3);

const clickButton = document.createElement('button');
clickButton.textContent = 'クリック';
clickButton.style.marginRight = '10px';
container3.appendChild(clickButton);

const hoverDiv = document.createElement('div');
hoverDiv.textContent = 'ここにマウスを乗せる';
hoverDiv.style.display = 'inline-block';
hoverDiv.style.padding = '10px';
hoverDiv.style.border = '2px solid #4CAF50';
hoverDiv.style.cursor = 'pointer';
container3.appendChild(hoverDiv);

const log3 = document.createElement('div');
log3.style.marginTop = '10px';
log3.style.border = '1px solid #ccc';
log3.style.padding = '10px';
log3.style.maxHeight = '200px';
log3.style.overflow = 'auto';
log3.style.fontFamily = 'monospace';
log3.style.fontSize = '12px';
container3.appendChild(log3);

function addLog3(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.backgroundColor = color;
  logItem.style.padding = '2px';
  logItem.textContent = message;
  log3.insertBefore(logItem, log3.firstChild);
}

// 複数のイベントソースを統合
const events$ = merge(
  fromEvent(clickButton, 'click').pipe(map(() => 'CLICK')),
  fromEvent(hoverDiv, 'mouseenter').pipe(map(() => 'HOVER_IN')),
  fromEvent(hoverDiv, 'mouseleave').pipe(map(() => 'HOVER_OUT')),
  interval(3000).pipe(take(5), map(i => `TIMER_${i}`))
);

events$
  .pipe(
    timestamp()
  )
  .subscribe(data => {
    const time = new Date(data.timestamp).toLocaleTimeString('ja-JP', { hour12: false }) +
                 '.' + (data.timestamp % 1000).toString().padStart(3, '0');

    const colors: Record<string, string> = {
      'CLICK': '#c8e6c9',
      'HOVER_IN': '#fff9c4',
      'HOVER_OUT': '#ffccbc',
    };

    const color = data.value.startsWith('TIMER') ? '#e1bee7' :
                  (colors[data.value] || '#e3f2fd');

    addLog3(`[${time}] イベント: ${data.value}`, color);
  });

addLog3('イベントログ記録中...', '#e3f2fd');
```

- 複数のイベントソースを統合
- すべてのイベントにタイムスタンプを付与
- 時系列でイベントを追跡

## タイムスタンプの活用方法

```ts
import { of } from 'rxjs';
import { timestamp, map } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    timestamp(),
    map(data => {
      // タイムスタンプを使った処理
      const date = new Date(data.timestamp);
      return {
        value: data.value,
        time: date.toISOString(),
        unixTime: data.timestamp
      };
    })
  )
  .subscribe(console.log);
// 出力:
// { value: 'A', time: '2024-01-01T00:00:00.000Z', unixTime: 1704067200000 }
// ...
```

## ⚠️ 注意点

### 1. タイムスタンプの精度

JavaScriptの`Date.now()`を使用するため、ミリ秒単位の精度です。

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

// 高頻度のイベント（1ms間隔）
interval(1)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(data => {
    console.log(`値: ${data.value}, タイムスタンプ: ${data.timestamp}`);
  });
// 同じタイムスタンプになる可能性がある
```

より高精度が必要な場合は`performance.now()`の使用を検討してください。

### 2. タイムスタンプは発行時点

値が発行された時点のタイムスタンプであり、生成時点ではありません。

```ts
import { of, asyncScheduler } from 'rxjs';
import { delay, timestamp } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delay(1000),      // 1秒遅延
    timestamp()       // 遅延後のタイムスタンプ
  )
  .subscribe(console.log);
```

### 3. オブジェクト構造の変化

`timestamp`を使用すると、値がオブジェクトにラップされます。

```ts
import { of } from 'rxjs';
import { timestamp, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    timestamp(),
    map(data => data.value * 2)  // .valueで元の値にアクセス
  )
  .subscribe(console.log);
// 出力: 2, 4, 6
```

## 📚 関連オペレーター

- **[tap](./tap)** - 副作用の実行（デバッグ用）
- **[delay](./delay)** - 固定時間の遅延
- **[timeout](./timeout)** - タイムアウト制御

## ✅ まとめ

`timestamp` オペレーターは、各値にタイムスタンプを付与します。

- ✅ 各値の発行時刻を正確に記録
- ✅ パフォーマンス測定に有効
- ✅ イベント間隔の分析が可能
- ✅ デバッグとロギングに活用
- ⚠️ ミリ秒単位の精度
- ⚠️ 値がオブジェクトにラップされる
- ⚠️ タイムスタンプは発行時点のもの
