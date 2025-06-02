---
description: asyncSchedulerやqueueSchedulerなど、RxJSにおける主要なスケジューラーの特徴・実装・用途について詳しく解説します。
---

# スケジューラーの種類と使い分け

RxJSには、異なる用途に応じた複数のスケジューラーが用意されています。それぞれのスケジューラーには特有の実行タイミングと特性があり、適切に使い分けることでアプリケーションのパフォーマンスと動作を最適化できます。

## スケジューラーの分類

RxJSのスケジューラーは、大きく3つのカテゴリーに分類されます。

1. **マクロタスク**: イベントループの次のタスクキューで実行
2. **マイクロタスク**: 現在のタスク完了直後、次のタスク開始前に実行
3. **同期処理**: 即時実行

詳しくは[タスクとスケジューラーの基礎知識](./task-and-scheduler-basics.md)も参照してください。

## 主要なスケジューラー

### asyncScheduler

#### 特徴
- **内部実装**: setTimeoutを使用
- **実行タイミング**: マクロタスク
- **用途**: 一般的な非同期処理、時間の経過を伴う処理

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs/operators';

console.log('1: 開始');

of('非同期処理')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: 終了');

// 出力:
// 1: 開始
// 2: 終了
// 3: 非同期処理
```

#### ユースケース

```ts
import { asyncScheduler, map, observeOn, of } from "rxjs";

function heavyComputation(value: number): number {
  // 重い計算をシミュレート
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result = Math.sin(result);
  }
  return result;
}

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(value => heavyComputation(value))
  )
  .subscribe(result => {
    console.log(`計算結果: ${result}`);
  });
```

### queueScheduler

#### 特徴
- **内部実装**: マイクロタスクキュー
- **実行タイミング**: 現在のタスク内（同期的に見える）
- **用途**: タスクのキューイング、再帰処理の最適化

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs/operators';

console.log('1: 開始');

of('キュー処理')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: 終了');

// 出力:
// 1: 開始
// 2: キュー処理
// 3: 終了
```

#### ユースケース

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs/operators';

// 再帰的な処理の最適化
function fibonacci(n: number): Observable<number> {
  return of([0, 1]).pipe(
    observeOn(queueScheduler),
    expand(([a, b]) => of([b, a + b])),
    map(([a]) => a),
    take(n)
  );
}

fibonacci(10).subscribe(value => console.log(value));
```

### asapScheduler

#### 特徴
- **内部実装**: Promise.resolve().then() または setImmediate
- **実行タイミング**: マイクロタスク
- **用途**: できるだけ早く非同期実行したい場合

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs/operators';

console.log('1: 開始');

of('ASAP処理')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: 終了');

// 出力:
// 1: 開始
// 2: 終了
// 3: ASAP処理
```

#### ユースケース

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs/operators';

// マウス移動イベントの最適化
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // UIの更新処理
    updateCursor(position);
  });
```

### animationFrameScheduler

#### 特徴
- **内部実装**: requestAnimationFrame
- **実行タイミング**: 次の画面描画前
- **用途**: アニメーション、60fps対応の描画処理

#### 簡単な回転アニメーションの例

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs/operators';

// HTML要素を作成
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// アニメーションの設定
let rotation = 0;

// 60fpsで2秒間アニメーション
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2秒 = 120フレーム
    map(() => {
      rotation += 3;  // 1フレームごとに3度回転
      return rotation;
    })
  )
  .subscribe(angle => {
    // DOM要素を実際に回転
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### なぜ animationFrameScheduler が必要なのか

`animationFrameScheduler` は、ブラウザの描画サイクルに同期して処理を実行するため、以下のような利点があります：

1. **滑らかなアニメーション**: ブラウザの描画タイミング（通常60fps）に合わせて処理を実行するため、カクつきのない滑らかなアニメーションを実現できます。
2. **効率的なリソース利用**: ブラウザがタブを非アクティブにした際は、requestAnimationFrameの実行が自動的に一時停止されるため、無駄なCPU使用を防げます。
3. **画面のちらつき防止**: 画面の描画前に確実に計算を完了させるため、画面のちらつきや不完全なフレームの表示を防止できます。

以下は、`setInterval` と `animationFrameScheduler` の比較です。

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ setIntervalを使用した非効率的なアニメーション
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // 約60fps

// 問題点：
// - ブラウザの描画タイミングと同期していない
// - バックグラウンドタブでも実行され続ける
// - 正確な60fpsを保証できない

// ✅ animationFrameSchedulerを使用した効率的なアニメーション
interval(0, animationFrameScheduler)
  .pipe(
    map(() => {
      position += 1;
      return position;
    })
  )
  .subscribe(pos => {
    element.style.transform = `translateX(${pos}px)`;
  });

// 利点：
// - ブラウザの描画タイミングに同期
// - バックグラウンドタブで自動的に一時停止
// - 安定した60fpsを実現
```


#### マウス追従アニメーションの例

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs/operators';

// 追従する円を作成
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // マウスイベントを透過
document.body.appendChild(circle);

// 現在の位置と目標位置
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// マウス移動イベントを監視
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove')
  .pipe(
    map(event => ({
      x: event.clientX,
      y: event.clientY
    }))
  );

// アニメーションループ
interval(0, animationFrameScheduler)
  .pipe(
    withLatestFrom(mouseMove$),
    map(([_, mousePos]) => mousePos)
  )
  .subscribe(({ x, y }) => {
    // マウス位置を目標として設定
    targetX = x;
    targetY = y;
    
    // 現在位置から目標位置に向かって徐々に移動（イージング）
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;
    
    // DOM要素を更新
    circle.style.left = `${currentX - 15}px`;  // 中心位置に調整
    circle.style.top = `${currentY - 15}px`;
  });
```

## スケジューラーの使い分けガイド

### 実行タイミングによる比較

```ts
import { of, asyncScheduler, queueScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs/operators';

console.log('1: 開始');

// 同期処理
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler（マイクロタスク）
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler（マイクロタスク）
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler（マクロタスク）
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

### 用途別の選択基準

| スケジューラー | 特徴 | 適した用途 |
|--------------|------|----------|
| asyncScheduler | setTimeout使用、完全な非同期 | 時間のかかる処理、遅延実行 |
| queueScheduler | 同期的だが再帰を最適化 | 再帰処理、タスクキュー管理 |
| asapScheduler | できるだけ早い非同期実行 | イベントハンドリング、高速な応答が必要な処理 |
| animationFrameScheduler | 画面描画に同期 | アニメーション、UI更新、ゲーム開発 |

## 実践的な使用例

### 大量データの処理

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
```

### WebSocketのメッセージ処理

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs/operators';

const socket$ = webSocket('wss://api.example.com');

socket$
  .pipe(
    // 高速な応答が必要なメッセージ処理
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });
```

## パフォーマンスへの影響

### スケジューラーのオーバーヘッド

```ts
import { range, asyncScheduler, pipe } from 'rxjs';
import { bufferCount, map, observeOn, tap } from 'rxjs/operators';

// ❌ 過剰なスケジューラー使用
range(1, 1000)
  .pipe(
    observeOn(asyncScheduler),  // 1000回のsetTimeout
    map(x => x * 2),
    // tap(console.log)
  )
  .subscribe();

// ✅ バッチ処理で最適化
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10回のsetTimeout
    map(batch => batch.map(x => x * 2)),
    // tap(console.log)
  )
  .subscribe();
```

## まとめ

スケジューラーの選択は、アプリケーションのパフォーマンスと応答性に大きな影響を与えます。各スケジューラーの特性を理解し、適切な場面で使い分けることで、効率的で滑らかな動作を実現できます。一般的なガイドラインとして：

- 一般的な非同期処理には`asyncScheduler`
- 再帰処理や同期的なキューイングには`queueScheduler`
- 高速な応答が必要な場合は`asapScheduler`
- アニメーションには`animationFrameScheduler`

を使用することをお勧めします。