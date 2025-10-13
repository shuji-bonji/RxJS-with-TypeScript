---
description: auditオペレーターは、カスタムObservableで制御される期間内の最後の値のみを発行するRxJSフィルタリングオペレーターです。動的なタイミング制御に最適です。
---

# audit - カスタムObservableで制御される期間の最後の値を発行

`audit` オペレーターは、カスタムObservableが値を発行するまで待機し、その期間内にソースから発行された**最後の値**を発行します。
`auditTime`が固定時間で制御するのに対し、`audit`は**動的なObservableで期間を制御**できます。

## 🔰 基本構文と使い方

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs/operators';

// クリックイベント
const clicks$ = fromEvent(document, 'click');

// 1秒ごとに期間を区切る
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('クリックが記録されました');
});
```

- クリックが発生すると、1秒間の期間が開始されます。
- その1秒間の最後のクリックだけが発行されます。
- 1秒後に次の期間が開始されます。

[🌐 RxJS公式ドキュメント - `audit`](https://rxjs.dev/api/operators/audit)

## 💡 典型的な活用パターン

- **動的な間隔でのサンプリング**：負荷に応じて期間を調整
- **カスタムタイミング制御**：他のObservableに基づく期間制御
- **適応的なイベント制限**：状況に応じた間引き

## 🔍 auditTime との違い

| オペレーター | 期間の制御 | ユースケース |
|:---|:---|:---|
| `auditTime` | 固定時間（ミリ秒） | シンプルな時間ベースの制御 |
| `audit` | **カスタムObservable** | **動的な期間制御** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs/operators';

const clicks$ = fromEvent(document, 'click');

// auditTime - 固定1秒
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('固定1秒'));

// audit - 動的な期間
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0〜2秒のランダム期間
    return timer(period);
  })
).subscribe(() => console.log(`動的期間: ${period}ms`));
```

## 🧠 実践コード例1: 負荷に応じた動的サンプリング

システムの負荷に応じてサンプリング間隔を調整する例です。

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs/operators';

// UI作成
const output = document.createElement('div');
output.innerHTML = '<h3>動的サンプリング</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = '負荷を変更';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// 負荷レベル（0: 低負荷、1: 中負荷、2: 高負荷）
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['低負荷', '中負荷', '高負荷'];
  statusDiv.textContent = `現在の負荷: ${levels[loadLevel]}`;
});

// マウス移動イベント
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // 負荷に応じて期間を調整
    const periods = [2000, 1000, 500]; // 低負荷→長い期間、高負荷→短い期間
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] マウス位置: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // 最大10件まで表示
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- 負荷が低い時は2秒間隔で間引き（省電力モード）
- 負荷が高い時は500ms間隔で細かくサンプリング
- 負荷に応じて動的に期間を調整できます。

## 🎯 実践コード例2: 他のストリームに基づく期間制御

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map, startWith } from 'rxjs/operators';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('間隔: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// スライダーの値を監視
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// クリックイベント
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// スライダーの値を更新
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// クリックをauditで制御
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] クリック記録（間隔: ${currentInterval}ms）`;
  output.insertBefore(log, output.firstChild);
});
```

## ⚠️ 注意点

### 1. 最初の値は即座に発行されない

`audit`は最初の値を受け取った後、期間が終了するまで待機します。

```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs/operators';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// 出力:
// 9  (1秒後、0〜9の最後の値)
// 19 (2秒後、10〜19の最後の値)
// 29 (3秒後、20〜29の最後の値)
```

### 2. duration Observableは毎回新しく生成される

`audit`に渡す関数は**毎回新しいObservableを返す必要**があります。

```ts
// ❌ 悪い例: 同じObservableインスタンスを使い回し
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2回目以降動作しない
).subscribe();

// ✅ 良い例: 毎回新しいObservableを生成
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. メモリとパフォーマンス

頻繁に値が発行されるストリームで`audit`を使用すると、メモリを消費します。

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs/operators';

// 高速ストリーム（10msごと）
interval(10).pipe(
  audit(() => timer(1000)) // 1秒ごとにサンプリング
).subscribe();
// 1秒間に100個の値がメモリに蓄積され、最後の1つだけが発行される
```

## 🆚 類似オペレーターとの比較

| オペレーター | 発行するタイミング | 発行する値 | ユースケース |
|:---|:---|:---|:---|
| `audit` | 期間**終了時** | 期間内の**最後**の値 | 期間内の最新状態を取得 |
| `throttle` | 期間**開始時** | 期間内の**最初**の値 | 連続イベントの最初を取得 |
| `debounce` | **静止後** | 静止直前の値 | 入力完了を待つ |
| `sample` | **別のObservable発火時** | その時点の最新値 | 定期的なスナップショット |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs/operators';

const clicks$ = fromEvent(document, 'click');

// audit: 1秒間の最後のクリック
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: 最後'));

// throttle: 1秒間の最初のクリック
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: 最初'));

// debounce: クリック停止から1秒後
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: 停止後'));

// sample: 1秒ごとにサンプリング
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: 定期的'));
```

## 📚 関連オペレーター

- **[auditTime](./auditTime)** - 固定時間で制御（`audit`のシンプル版）
- **[throttle](./throttleTime)** - 期間開始時に最初の値を発行
- **[debounce](./debounceTime)** - 静止後に値を発行
- **[sample](./sampleTime)** - 別のObservableのタイミングでサンプリング

## まとめ

`audit`オペレーターは、カスタムObservableで動的に制御される期間内の最後の値を発行します。

- ✅ 動的な期間制御が可能
- ✅ 負荷に応じた適応的なサンプリング
- ✅ 他のストリームに基づく制御
- ⚠️ 毎回新しいObservableを生成する必要がある
- ⚠️ 頻繁な発行ではメモリに注意
