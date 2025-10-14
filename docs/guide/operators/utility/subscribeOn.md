---
description: subscribeOnオペレーターはObservableの購読開始タイミングを指定したスケジューラーで制御し、ストリーム全体の実行コンテキストを変更します。
---

# subscribeOn - 購読開始タイミングの制御

`subscribeOn` オペレーターは、Observable の**購読開始のタイミングと実行コンテキストを指定したスケジューラーで制御**します。ストリーム全体の実行タイミングに影響を与えます。

## 🔰 基本構文・動作

スケジューラーを指定して、購読開始を非同期化します。

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

console.log('開始');

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler)
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

購読開始自体が非同期化されるため、`subscribe()`の呼び出しが即座に返ります。

[🌐 RxJS公式ドキュメント - subscribeOn](https://rxjs.dev/api/index/function/subscribeOn)

## 💡 典型的な活用例

- **重い初期化処理の非同期化**: データ読み込みなどの開始を遅延
- **UIのフリーズ防止**: 購読開始を非同期にして応答性を維持
- **処理の優先順位付け**: 複数のストリームの開始タイミングを制御
- **テストでのタイミング制御**: TestSchedulerを使った制御

## 🧪 実践コード例1: 重い初期化処理の非同期化

データの読み込みや初期化を非同期で開始する例です。

```ts
import { Observable, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'subscribeOn - 重い初期化処理';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const timestamp = now.toLocaleTimeString('ja-JP', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${timestamp}] ${message}`;
  output.appendChild(logItem);
}

// 重い初期化処理をシミュレート
const heavyInit$ = new Observable<string>(subscriber => {
  addLog('データ読み込み開始...', '#fff9c4');

  // 重い処理をシミュレート
  let sum = 0;
  for (let i = 0; i < 10000000; i++) {
    sum += i;
  }

  addLog('データ読み込み完了', '#c8e6c9');
  subscriber.next(`結果: ${sum}`);
  subscriber.complete();
});

addLog('購読開始（UI操作可能）', '#e3f2fd');

heavyInit$
  .pipe(
    subscribeOn(asyncScheduler)  // 購読開始を非同期化
  )
  .subscribe({
    next: result => addLog(`受信: ${result}`, '#c8e6c9'),
    complete: () => addLog('完了', '#e3f2fd')
  });

addLog('購読リクエスト後（すぐに実行が続く）', '#e3f2fd');
```

- 購読開始が非同期化され、UIがすぐに応答
- 重い処理は非同期で実行される
- メインスレッドがブロックされない

## 🧪 実践コード例2: 複数ストリームの優先順位制御

複数のストリームの開始タイミングを制御する例です。

```ts
import { interval, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn, take, tap } from 'rxjs';

// UI作成
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'subscribeOn - 優先順位制御';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string, color: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('ja-JP', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.marginBottom = '2px';
  logItem.style.backgroundColor = color;
  logItem.style.fontSize = '12px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('開始', '#e3f2fd');

// 高優先度タスク（asapScheduler）
interval(500)
  .pipe(
    take(3),
    subscribeOn(asapScheduler),
    tap(v => addLog2(`高優先度: ${v}`, '#c8e6c9'))
  )
  .subscribe();

// 通常優先度タスク（asyncScheduler）
interval(500)
  .pipe(
    take(3),
    subscribeOn(asyncScheduler),
    tap(v => addLog2(`通常優先度: ${v}`, '#fff9c4'))
  )
  .subscribe();

addLog2('購読リクエスト完了', '#e3f2fd');
```

- 異なるスケジューラーで優先順位を制御
- `asapScheduler`は`asyncScheduler`より早く実行開始

## 🆚 observeOn との違い

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

// observeOn の例
console.log('=== observeOn ===');
console.log('1: 開始');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('2: tap（同期）')),
    observeOn(asyncScheduler),
    tap(() => console.log('4: tap（非同期）'))
  )
  .subscribe(() => console.log('5: subscribe'));

console.log('3: 終了');

// subscribeOn の例
console.log('\n=== subscribeOn ===');
console.log('1: 開始');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('3: tap（非同期）')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(() => console.log('4: subscribe'));

console.log('2: 終了');
```

**主な違い**:

| 項目 | observeOn | subscribeOn |
|:---|:---|:---|
| **影響範囲** | 後続の処理のみ | ストリーム全体 |
| **制御対象** | 値の発行タイミング | 購読開始のタイミング |
| **配置位置** | 重要（どこに置くかで動作が変わる） | どこに置いても同じ |
| **複数使用** | 最後のものが適用 | 最初のものが適用 |

> [!NOTE]
> `observeOn`の詳細については、[observeOn](./observeOn.md)を参照してください。

## ⚠️ 注意点

### 1. 配置位置は影響しない

`subscribeOn`はパイプライン内のどこに置いても同じ効果を持ちます。

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, map } from 'rxjs';

// パターン1: 最初
of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),
    map(x => x * 2)
  )
  .subscribe();

// パターン2: 最後
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// どちらも同じ動作
```

### 2. 複数のsubscribeOnは最初のものが適用

```ts
import { of, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),  // これが使用される
    subscribeOn(asapScheduler)    // これは無視される
  )
  .subscribe();
```

最初の`subscribeOn`のスケジューラー（`asyncScheduler`）が使用されます。

### 3. 一部のObservableには効果がない

`interval`や`timer`など、独自のスケジューラーを持つObservableには`subscribeOn`は影響しません。

```ts
import { interval, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// ❌ subscribeOnは効果なし
interval(1000)
  .pipe(
    subscribeOn(asyncScheduler)  // intervalは独自のスケジューラーを使用
  )
  .subscribe();

// ✅ intervalの引数でスケジューラーを指定
interval(1000, asyncScheduler)
  .subscribe();
```

## 実践的な組み合わせ例

```ts
import { of, asyncScheduler, animationFrameScheduler } from 'rxjs';
import { subscribeOn, observeOn, map, tap } from 'rxjs';

console.log('開始');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('タップ1（非同期）')),
    subscribeOn(asyncScheduler),        // 購読開始を非同期化
    map(x => x * 2),
    observeOn(animationFrameScheduler), // 値の発行をアニメーションフレームに同期
    tap(() => console.log('タップ2（アニメーションフレーム）'))
  )
  .subscribe(v => console.log('値:', v));

console.log('終了');

// 実行順序:
// 開始
// 終了
// タップ1（非同期）
// タップ1（非同期）
// タップ1（非同期）
// タップ2（アニメーションフレーム）
// 値: 2
// ... (以下続く)
```

## 使い分けのガイドライン

### ケース1: 購読開始を遅延させたい
```ts

// → subscribeOn を使用
of(データ)
  .pipe(subscribeOn(asyncScheduler))
  .subscribe();
```

### ケース2: 特定の処理だけ非同期化したい
```ts
// → observeOn を使用
of(データ)
  .pipe(
    map(重い処理),
    observeOn(asyncScheduler),  // 重い処理の後だけ非同期化
    map(軽い処理)
  )
  .subscribe();
```

### ケース3: 全体を非同期化 + 一部をさらに制御
```ts
// → subscribeOn + observeOn の併用
of(データ)
  .pipe(
    subscribeOn(asyncScheduler),           // 全体を非同期化
    map(処理1),
    observeOn(animationFrameScheduler),    // アニメーション用に変更
    map(処理2)
  )
  .subscribe();
```

## 📚 関連オペレーター

- **[observeOn](./observeOn)** - 値の発行タイミング制御
- **[delay](./delay)** - 固定時間の遅延

## 📖 関連ドキュメント

- **[非同期処理の制御](../../schedulers/async-control.md)** - スケジューラーの基本
- **[スケジューラーの種類と使い分け](../../schedulers/types.md)** - 各スケジューラーの詳細

## ✅ まとめ

`subscribeOn` オペレーターは、購読開始のタイミングと実行コンテキストを制御します。

- ✅ ストリーム全体の購読開始を非同期化
- ✅ 重い初期化処理の非同期化に有効
- ✅ UIのフリーズ防止に活用
- ✅ 配置位置は影響しない
- ⚠️ 複数使用時は最初のものが適用
- ⚠️ 一部のObservableには効果がない
- ⚠️ `observeOn`とは目的が異なる
