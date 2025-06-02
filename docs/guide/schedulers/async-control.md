---
description: RxJSにおけるスケジューラーの使い方を解説し、observeOnやsubscribeOnによる非同期処理の制御方法やパフォーマンス最適化手法を具体例とともに紹介します。
---
# 非同期処理の制御

RxJSにおけるスケジューラーは、非同期処理のタイミングと実行コンテキストを制御するための重要な仕組みです。本章では、スケジューラーを使って非同期処理をどのように制御するかについて解説します。

## スケジューラーの役割

スケジューラーは、以下の3つの重要な役割を担います。

|役割|説明|
|---|---|
|実行タイミングの制御|いつタスクを実行するか決定|
|実行コンテキストの管理|どのスレッドや実行環境でタスクを動作させるか|
|タスクの優先順位付け|複数のタスクの実行順序を管理|

## 同期処理と非同期処理の理解

### デフォルトの動作（同期実行）

RxJSのオペレーターは、デフォルトでは可能な限り同期的に実行されます。

```ts
import { of } from 'rxjs';
import { map } from 'rxjs/operators';

console.log('実行開始');

of(1, 2, 3)
  .pipe(
    map((x) => {
      console.log(`map: ${x}`);
      return x * 2;
    })
  )
  .subscribe((x) => console.log(`subscribe: ${x}`));

console.log('実行終了');

// 出力:
// 実行開始
// map: 1
// subscribe: 2
// map: 2
// subscribe: 4
// map: 3
// subscribe: 6
// 実行終了
```

### スケジューラーによる非同期化

スケジューラーを使用することで、処理を非同期化できます。

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs/operators';

console.log('実行開始');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(x => console.log(`subscribe: ${x}`));

console.log('実行終了');

// 出力:
// 実行開始
// 実行終了
// subscribe: 1
// subscribe: 2
// subscribe: 3
```

## スケジューラーを使用するオペレーター

### observeOn オペレーター

`observeOn`は、ストリームの実行コンテキストを変更します。指定したスケジューラーで値の発行を行います。

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { take, observeOn } from 'rxjs/operators';

// アニメーション用途での使用例
interval(16)
  .pipe(
    take(10),
    observeOn(animationFrameScheduler)
  )
  .subscribe(() => {
    // アニメーションフレームに同期して実行
    updateAnimation();
  });

function updateAnimation() {
  // アニメーション更新処理
}
```

### subscribeOn オペレーター

`subscribeOn`は、ストリームの購読開始タイミングを制御します。

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, tap } from 'rxjs/operators';

console.log('購読開始前');

of('タスク実行')
  .pipe(
    tap(() => console.log('タスク開始')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(value => console.log(value));

console.log('購読開始後');

// 出力:
// 購読開始前
// 購読開始後
// タスク開始
// タスク実行
```


## 非同期処理の実践的な例

### APIリクエストの制御

```ts
import { from, queueScheduler } from 'rxjs';
import { mergeMap, observeOn, tap } from 'rxjs/operators';

interface ApiRequest {
  endpoint: string;
  id: number;
}

const requests: ApiRequest[] = [
  { endpoint: '/users', id: 1 },
  { endpoint: '/posts', id: 1 },
  { endpoint: '/comments', id: 1 },
];

// リクエストをキューに入れて順番に処理
from(requests)
  .pipe(
    observeOn(queueScheduler),
    tap((req) => console.log(`キューに追加: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // 実際のAPIリクエストのシミュレーション
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`${req.endpoint}/${req.id} の結果`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`完了: ${result}`));

// 出力:
// キューに追加: /users
// キューに追加: /posts
// キューに追加: /comments
// 完了: /users/1 の結果
// 完了: /posts/1 の結果
// 完了: /comments/1 の結果
```

### UIスレッドのブロック回避

大量のデータ処理を行う際、UIスレッドをブロックしないようにスケジューラーを活用します。

```ts
import { from, asapScheduler } from 'rxjs';
import { observeOn, bufferCount } from 'rxjs/operators';

const largeDataSet = Array.from({ length: 10000 }, (_, i) => i);

// バッチ単位
const batchSize = 100;
// 全体のバッチ数を計算
const totalBatches = Math.ceil(largeDataSet.length / batchSize);
// バッチカウンター
let batchIndex = 0;

from(largeDataSet)
  .pipe(
    bufferCount(100), // 100件ずつまとめる
    observeOn(asapScheduler) // できるだけ早く、でもUIブロックしない
  )
  .subscribe((batch) => {
    batchIndex++;
    processBatch(batch, batchIndex, totalBatches);
  });

function processBatch(
  batch: number[],
  batchIndex: number,
  totalBatches: number
) {
  // バッチデータの処理
  const processed = batch.map((n) => n * 2);
  console.log(
    `全${totalBatches}バッチ中 ${batchIndex}バッチ目 完了: ${processed.length}件処理しました。`
  );
}

// 出力:
// 全100バッチ中 1バッチ目 完了: 100件処理しました。
// 全100バッチ中 2バッチ目 完了: 100件処理しました。
// ...
// ...
// 全100バッチ中 100バッチ目 完了: 100件処理しました。
```

## パフォーマンス最適化とデバッグ

### スケジューラーを活用したテスト

```ts
import { TestScheduler } from 'rxjs/testing';
import { delay } from 'rxjs/operators';
import { beforeEach, describe, expect, it } from 'vitest';

describe('非同期処理のテスト', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('delay オペレーターのテスト', () => {
    scheduler.run(({ cold, expectObservable }) => {
      const source = cold('a-b-c|');
      const expected =    '1000ms a-b-(c|)';

      const result = source.pipe(delay(1000, scheduler));

      expectObservable(result).toBe(expected);
    });
  });
});
```

### デバッグ用のログ出力

```ts
import { of, asyncScheduler } from 'rxjs';
import { tap, observeOn } from 'rxjs/operators';

console.log('開始');

of(1, 2, 3)
  .pipe(
    tap(value => console.log(`[スケジューラー前・同期] 値: ${value}`)),
    observeOn(asyncScheduler),  // asyncSchedulerを使用
    tap(value => console.log(`[スケジューラー後・非同期] 値: ${value}`))
  )
  .subscribe();

console.log('終了');

// 実際の出力:
// 開始
// [スケジューラー前・同期] 値: 1
// [スケジューラー前・同期] 値: 2
// [スケジューラー前・同期] 値: 3
// 終了
// [スケジューラー後・非同期] 値: 1
// [スケジューラー後・非同期] 値: 2
// [スケジューラー後・非同期] 値: 3
```

`asyncScheduler`を使用すると、期待通りの非同期動作が確認できます。`queueScheduler`はマイクロタスクキューを使用するため、同期コードの実行中に処理されてしまうのに対し、`asyncScheduler`はsetTimeoutを内部で使用するため、完全に非同期で実行されます。

## スケジューラーの動作の違いを示す例
この例では、異なるスケジューラーの実行タイミングの違いを示しています。

```ts
import { of, queueScheduler, asyncScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs/operators';

console.log('1: 開始');

// 同期処理
of('sync').subscribe(value => console.log(`2: ${value}`));

// queueScheduler（マイクロタスク）
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`3: ${value}`));

// asapScheduler（マイクロタスク）
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`4: ${value}`));

// asyncScheduler（マクロタスク）
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`5: ${value}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: 終了');

// 実際の出力順序:
// 1: 開始
// 2: sync
// 7: 終了
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```


## ベストプラクティス

1. **必要な時だけスケジューラーを使用**: デフォルトの同期動作で十分な場合は、無理にスケジューラーを使用しない

2. **適切なスケジューラーの選択**: 用途に応じて最適なスケジューラーを選択する
   - アニメーション: `animationFrameScheduler`
   - UIブロック回避: `asapScheduler`
   - キュー処理: `queueScheduler`
   - 非同期処理: `asyncScheduler`

3. **パフォーマンスモニタリング**: スケジューラーの使用によるパフォーマンスへの影響を常に監視

4. **テスト容易性の確保**: `TestScheduler`を活用して非同期処理のテストを書く

## よくある間違いと対策

### 過剰な非同期化

```ts
// ❌ 不要な非同期化
of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(x => x * 2),
    observeOn(asyncScheduler),  // 重複した非同期化
    filter(x => x > 3)
  )
  .subscribe();

// ✅ 必要な箇所だけ非同期化
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    filter(x => x > 3),
    observeOn(asyncScheduler)  // 最後にまとめて非同期化
  )
  .subscribe();
```

### スケジューラーの誤用

```ts
// ❌ 間違った使い方
interval(1000)
  .pipe(
    subscribeOn(animationFrameScheduler)  // intervalには影響しない
  )
  .subscribe();

// ✅ 正しい使い方
interval(1000, animationFrameScheduler)  // 生成時にスケジューラーを指定
  .subscribe();
```

## まとめ

スケジューラーは、RxJSにおける非同期処理を細かく制御するための強力なツールです。適切に使用することで、パフォーマンスの最適化、UIスレッドのブロック回避、テストの容易化などが実現できます。ただし、過剰な非同期化はかえってパフォーマンスを悪化させる可能性があるため、必要な場合にのみ使用することが重要です。

次のセクションでは、具体的なスケジューラーの種類とその使い分けについて詳しく解説します。