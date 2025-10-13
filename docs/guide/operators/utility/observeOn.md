---
description: observeOnオペレーターはObservableの値の発行タイミングを指定したスケジューラーで制御し、非同期処理やアニメーションの最適化に活用します。
---

# observeOn - 実行コンテキストの制御

`observeOn` オペレーターは、Observable の**値の発行タイミングと実行コンテキストを指定したスケジューラーで制御**します。ストリーム内の後続の処理を特定のスケジューラー上で実行させることができます。

## 🔰 基本構文・動作

スケジューラーを指定して、後続の処理を非同期化します。

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('開始');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(v => console.log('値:', v));

console.log('終了');

// 出力:
// 開始
// 終了
// 値: 1
// 値: 2
// 値: 3
```

`observeOn` より前の処理は同期的に実行され、`observeOn` より後の処理は指定したスケジューラーで実行されます。

[🌐 RxJS公式ドキュメント - observeOn](https://rxjs.dev/api/index/function/observeOn)

## 💡 典型的な活用例

- **UIスレッドのブロック回避**: 重い処理を非同期化
- **アニメーションの最適化**: `animationFrameScheduler`で滑らかな描画
- **処理の優先順位付け**: 異なるスケジューラーで実行タイミングを制御
- **マイクロタスク/マクロタスクの制御**: 実行タイミングの細かい調整

## スケジューラーの種類

| スケジューラー | 特徴 | ユースケース |
|:---|:---|:---|
| `asyncScheduler` | `setTimeout`ベース | 一般的な非同期処理 |
| `asapScheduler` | マイクロタスク（Promise.then） | できるだけ早い非同期実行 |
| `queueScheduler` | 同期的キュー | 再帰処理の最適化 |
| `animationFrameScheduler` | `requestAnimationFrame` | アニメーション、60fps描画 |

> [!TIP]
> スケジューラーの詳細については、[スケジューラーの種類と使い分け](../../schedulers/types.md)を参照してください。

## 🧪 実践コード例1: UIブロック回避

大量のデータ処理をバッチに分けて非同期実行する例です。

```ts
import { range, asapScheduler } from 'rxjs';
import { observeOn, bufferCount, tap } from 'rxjs';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'observeOn - UIブロック回避';
container.appendChild(title);

const progress = document.createElement('div');
progress.style.marginBottom = '10px';
container.appendChild(progress);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

function addLog(message: string) {
  const logItem = document.createElement('div');
  logItem.style.fontSize = '12px';
  logItem.style.marginBottom = '2px';
  logItem.textContent = message;
  output.appendChild(logItem);
}

const totalItems = 10000;
const batchSize = 100;
const totalBatches = Math.ceil(totalItems / batchSize);
let processedBatches = 0;

addLog('処理開始...');
progress.textContent = '進捗: 0%';

range(1, totalItems)
  .pipe(
    bufferCount(batchSize),
    observeOn(asapScheduler),  // 各バッチを非同期で処理
    tap(batch => {
      // 重い計算をシミュレート
      const sum = batch.reduce((acc, n) => acc + n, 0);
      processedBatches++;
      const percent = Math.floor((processedBatches / totalBatches) * 100);
      progress.textContent = `進捗: ${percent}%`;

      if (processedBatches % 10 === 0 || processedBatches === totalBatches) {
        addLog(`バッチ ${processedBatches}/${totalBatches} 完了 (合計: ${sum})`);
      }
    })
  )
  .subscribe({
    complete: () => {
      addLog('--- すべての処理が完了 ---');
      progress.textContent = '進捗: 100% ✅';
    }
  });
```

- 10,000件のデータを100件ずつバッチ処理
- `asapScheduler`でUIをブロックせずに処理
- 進捗をリアルタイムで表示

## 🧪 実践コード例2: アニメーションの最適化

`animationFrameScheduler`を使った滑らかなアニメーション例です。

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { observeOn, take, map } from 'rxjs';

// UI作成
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'observeOn - アニメーション';
container2.appendChild(title2);

const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = '#4CAF50';
box.style.position = 'relative';
box.style.transition = 'none';
container2.appendChild(box);

let position = 0;

interval(0)
  .pipe(
    observeOn(animationFrameScheduler),  // 60fpsで実行
    take(180),  // 3秒間（60fps × 3秒）
    map(() => {
      position += 2;  // 1フレームごとに2px移動
      return position;
    })
  )
  .subscribe({
    next: pos => {
      box.style.left = `${pos}px`;
    },
    complete: () => {
      const message = document.createElement('div');
      message.textContent = 'アニメーション完了';
      message.style.marginTop = '10px';
      message.style.color = '#4CAF50';
      container2.appendChild(message);
    }
  });
```

- `animationFrameScheduler`でブラウザの描画サイクルに同期
- 滑らかな60fpsアニメーション
- バックグラウンドタブで自動的に一時停止

## 🆚 subscribeOn との違い

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

console.log('=== observeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('observeOn前（同期）')),
    observeOn(asyncScheduler),
    tap(() => console.log('observeOn後（非同期）'))
  )
  .subscribe();

console.log('=== subscribeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('subscribeOn後（非同期）')),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// 出力:
// === observeOn ===
// observeOn前（同期）
// observeOn前（同期）
// observeOn前（同期）
// === subscribeOn ===
// observeOn後（非同期）
// observeOn後（非同期）
// observeOn後（非同期）
// subscribeOn後（非同期）
// subscribeOn後（非同期）
// subscribeOn後（非同期）
```

| オペレーター | 影響範囲 | タイミング制御 |
|:---|:---|:---|
| `observeOn` | 後続の処理のみ | 値の発行タイミング |
| `subscribeOn` | ストリーム全体 | 購読開始のタイミング |

> [!NOTE]
> `subscribeOn`の詳細については、[subscribeOn](./subscribeOn.md)を参照してください。

## ⚠️ 注意点

### 1. 配置位置が重要

`observeOn`の配置場所によって、どの処理が非同期化されるかが変わります。

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, map, tap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(() => console.log('処理1（同期）')),
    map(x => x * 2),
    observeOn(asyncScheduler),  // ここから非同期
    tap(() => console.log('処理2（非同期）')),
    map(x => x + 10)
  )
  .subscribe();

// 処理1は同期、処理2は非同期
```

### 2. 複数のobserveOnは累積しない

```ts
import { of, asyncScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    observeOn(queueScheduler)  // 最後のスケジューラーが適用される
  )
  .subscribe();
```

最後の`observeOn`のスケジューラー（この場合`queueScheduler`）が使用されます。

### 3. パフォーマンスへの影響

頻繁な`observeOn`の使用はオーバーヘッドになります。

```ts
// ❌ 悪い例: 各値ごとに非同期化
range(1, 1000)
  .pipe(
    map(x => x * 2),
    observeOn(asyncScheduler)  // 1000回のsetTimeout
  )
  .subscribe();

// ✅ 良い例: バッチ処理
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10回のsetTimeout
    concatMap(batch => from(batch).pipe(map(x => x * 2)))
  )
  .subscribe();
```

## 実行タイミングの比較

```ts
import { of, asyncScheduler, asapScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: 開始');

// 同期処理
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(v => console.log(`5: ${v}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: 終了');

// 実行順序:
// 1: 開始
// 2: sync
// 7: 終了
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```

## 📚 関連オペレーター

- **[subscribeOn](./subscribeOn)** - 購読開始のタイミング制御
- **[delay](./delay)** - 固定時間の遅延
- **[debounceTime](../filtering/debounceTime)** - 入力停止後の遅延

## 📖 関連ドキュメント

- **[非同期処理の制御](../../schedulers/async-control.md)** - スケジューラーの基本
- **[スケジューラーの種類と使い分け](../../schedulers/types.md)** - 各スケジューラーの詳細

## ✅ まとめ

`observeOn` オペレーターは、値の発行タイミングと実行コンテキストを制御します。

- ✅ 後続の処理を指定したスケジューラーで実行
- ✅ UIブロック回避に有効
- ✅ アニメーション最適化に活用
- ✅ 処理の優先順位付けが可能
- ⚠️ 配置位置が重要
- ⚠️ パフォーマンスオーバーヘッドに注意
- ⚠️ 複数使用時は最後のスケジューラーが適用
