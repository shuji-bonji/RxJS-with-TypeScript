---
description: "interval() - 指定間隔で連続的に値（0から始まる連番）を発行するCreation Function。ポーリングや定期実行、アニメーション制御に最適です。timer()との違い、take()での制限方法、TypeScriptでの型安全な実装、メモリリーク防止のための購読解除パターンを解説します。"
---

# interval() - 指定間隔で連続発行

`interval()`は、指定した時間間隔で連続的に値を発行するCreation Functionです。

## 概要

`interval()`は、指定したミリ秒間隔で0から始まる連続した数値を発行し続けます。ポーリング処理や定期的なタスク実行に頻繁に使用されます。

**シグネチャ**:
```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

**公式ドキュメント**: [📘 RxJS公式: interval()](https://rxjs.dev/api/index/function/interval)

## 基本的な使い方

`interval()`は、指定した間隔でカウントアップする数値を発行します。

```typescript
import { interval } from 'rxjs';

// 1秒ごとに値を発行
const interval$ = interval(1000);

interval$.subscribe(value => {
  console.log('値:', value);
});

// 出力（1秒ごと）:
// 値: 0
// 値: 1
// 値: 2
// 値: 3
// ...（無限に続く）
```

## 重要な特徴

### 1. 0から始まる連続した数値

`interval()`は、常に0から始まり、1ずつ増加する整数を発行します。

```typescript
import { interval } from 'rxjs';
import { take } from 'rxjs';

interval(500).pipe(
  take(5) // 最初の5つの値のみ取得
).subscribe(value => console.log(value));

// 出力（500msごと）:
// 0
// 1
// 2
// 3
// 4
```

### 2. 完了しない（無限ストリーム）

`interval()`は自動的に完了しないため、**必ず購読解除が必要**です。

```typescript
import { interval } from 'rxjs';

const subscription = interval(1000).subscribe(value => {
  console.log('値:', value);
});

// 5秒後に購読解除
setTimeout(() => {
  subscription.unsubscribe();
  console.log('停止しました');
}, 5000);
```

> [!WARNING]
> **購読解除を忘れるとメモリリーク**
>
> `interval()`は無限に値を発行し続けるため、購読解除を忘れるとメモリリークやパフォーマンス問題の原因になります。必ず`unsubscribe()`を呼ぶか、`take()`、`takeUntil()`、`takeWhile()`などのオペレーターで自動完了させてください。

### 4. Cold Observable

`interval()`はCold Observableであり、購読ごとに独立したタイマーが作成されます。

```typescript
import { interval } from 'rxjs';

const interval$ = interval(1000);

// 購読1
interval$.subscribe(value => console.log('Observer 1:', value));

// 2秒後に購読2を追加
setTimeout(() => {
  interval$.subscribe(value => console.log('Observer 2:', value));
}, 2000);

// 出力:
// Observer 1: 0
// Observer 1: 1
// Observer 2: 0  ← 独立したタイマーで0から開始
// Observer 1: 2
// Observer 2: 1
```

> [!NOTE]
> **Cold Observableの特徴**
> - 購読するたびに独立した実行が開始されます
> - 各購読者は独自のデータストリームを受け取ります
> - 購読ごとに独立したタイマーが開始されます。データの共有が必要な場合は`share()`を使用してください。
>
> 詳しくは [コールドObservableとホットObservable](/guide/observables/cold-and-hot-observables) を参照してください。

## interval() vs timer() の違い

`interval()`と`timer()`は似ていますが、いくつか重要な違いがあります。

```typescript
import { interval, timer } from 'rxjs';
import { take } from 'rxjs';

// interval() - 即座に開始、連続発行
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - 遅延後に開始
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// 出力:
// interval: 0  （1秒後）
// interval: 1  （2秒後）
// timer: 0     （2秒後）
// interval: 2  （3秒後）
// timer: 1     （3秒後）
// timer: 2     （4秒後）
```

| Creation Function | 開始タイミング | 用途 |
|-------------------|--------------|------|
| `interval(1000)` | 即座に開始（1秒後に最初の値） | 定期実行 |
| `timer(2000, 1000)` | 指定時間後に開始 | 遅延付き定期実行 |
| `timer(2000)` | 指定時間後に1回のみ発行 | 遅延実行 |

## 実践的なユースケース

### 1. APIポーリング

一定間隔でAPIを呼び出し、データを更新します。

```typescript
import { from, interval } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

function fetchStatus(): Promise<Status> {
  return fetch('https://jsonplaceholder.typicode.com/users/1')
    .then(res => res.json());
}

// 5秒ごとにAPIをポーリング
const polling$ = interval(5000).pipe(
  switchMap(() => from(fetchStatus())),
  catchError(error => {
    console.error('API Error:', error);
    return of({ status: 'error', timestamp: Date.now() });
  })
);

const subscription = polling$.subscribe(data => {
  console.log('ステータス更新:', data);
});

// 必要に応じて停止
// subscription.unsubscribe();
```

### 2. カウントダウンタイマー

制限時間のカウントダウンを実装します。

```typescript
import { interval } from 'rxjs';
import { map, takeWhile } from 'rxjs';

const countdown$ = interval(1000).pipe(
  map(count => 10 - count), // 10秒からカウントダウン
  takeWhile(time => time >= 0) // 0で自動完了
);

countdown$.subscribe({
  next: time => console.log(`残り時間: ${time}秒`),
  complete: () => console.log('時間切れ！')
});

// 出力（1秒ごと）:
// 残り時間: 10秒
// 残り時間: 9秒
// ...
// 残り時間: 0秒
// 時間切れ！
```

### 3. 自動保存機能

フォームの内容を定期的に自動保存します。

```typescript
import { fromEvent, from } from 'rxjs';
import { switchMap, debounceTime } from 'rxjs';

// フォームを作成
const form = document.createElement('form');
form.id = 'myForm';
const input = document.createElement('input');
input.type = 'text';
input.placeholder = '入力してください';
form.appendChild(input);
document.body.appendChild(form);

const input$ = fromEvent(form, 'input');

// 入力が止まってから3秒後に自動保存（デモ用に短縮）
input$.pipe(
  debounceTime(3000), // 3秒間入力がなければ
  switchMap(() => {
    const formData = new FormData(form);
    // デモ用: 実際のAPIではなくPromiseでシミュレート
    return from(
      Promise.resolve({ success: true, data: formData.get('text') })
    );
  })
).subscribe(result => {
  console.log('自動保存しました:', result);
});
```

### 4. リアルタイム時計の表示

現在時刻をリアルタイムで更新します。

```typescript
import { interval } from 'rxjs';
import { map } from 'rxjs';

// 時計表示用の要素を作成
const clockElement = document.createElement('div');
clockElement.id = 'clock';
clockElement.style.fontSize = '24px';
clockElement.style.fontFamily = 'monospace';
clockElement.style.padding = '20px';
document.body.appendChild(clockElement);

const clock$ = interval(1000).pipe(
  map(() => new Date().toLocaleTimeString())
);

clock$.subscribe(time => {
  clockElement.textContent = time;
});

// 出力: 現在時刻が1秒ごとに更新される
```

## パイプラインでの使用

`interval()`は、パイプラインの起点として、または時間制御のトリガーとして使用されます。

```typescript
import { interval } from 'rxjs';
import { map, filter, scan } from 'rxjs';

// 偶数秒のみカウント
interval(1000).pipe(
  filter(count => count % 2 === 0),
  scan((sum, count) => sum + count, 0),
  map(sum => `偶数の合計: ${sum}`)
).subscribe(console.log);

// 出力（1秒ごと）:
// 偶数の合計: 0
// 偶数の合計: 2  （0 + 2）
// 偶数の合計: 6  （0 + 2 + 4）
// 偶数の合計: 12 （0 + 2 + 4 + 6）
```

## よくある間違い

### 1. 購読解除を忘れる

```typescript
// ❌ 間違い - 購読解除せず無限に実行される
import { interval } from 'rxjs';

function startPolling() {
  interval(1000).subscribe(value => {
    console.log('値:', value); // 永遠に実行され続ける
  });
}

startPolling();

// ✅ 正しい - 購読を保持し、必要に応じて解除
import { interval, Subscription } from 'rxjs';

let subscription: Subscription | null = null;

function startPolling() {
  subscription = interval(1000).subscribe(value => {
    console.log('値:', value);
  });
}

function stopPolling() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startPolling();
// 必要に応じてstopPolling()を呼ぶ
```

### 2. 複数購読で独立したタイマーが作成される

```typescript
// ❌ 意図しない - 2つの独立したタイマーが作成される
import { interval } from 'rxjs';

const interval$ = interval(1000);

interval$.subscribe(value => console.log('Observer 1:', value));
interval$.subscribe(value => console.log('Observer 2:', value));
// 2つのタイマーが並行して動作

// ✅ 正しい - 1つのタイマーを共有
import { interval } from 'rxjs';
import { share } from 'rxjs';

const interval$ = interval(1000).pipe(share());

interval$.subscribe(value => console.log('Observer 1:', value));
interval$.subscribe(value => console.log('Observer 2:', value));
// 1つのタイマーが共有される
```

## パフォーマンスの考慮事項

`interval()`は軽量ですが、短い間隔で実行する場合はパフォーマンスに注意が必要です。

> [!TIP]
> **最適化のヒント**:
> - 不要な処理は実行しない（`filter()`で絞り込む）
> - 短い間隔（100ms以下）での使用は慎重に
> - 購読解除を確実に行う
> - 複数のObserverが必要な場合は`share()`で共有

```typescript
import { interval } from 'rxjs';
import { filter, share } from 'rxjs';

// ❌ パフォーマンス問題 - 100msごとに重い処理
interval(100).subscribe(() => {
  // 重い処理
  heavyCalculation();
});

// ✅ 最適化 - 必要な時だけ処理
interval(100).pipe(
  filter(count => count % 10 === 0), // 1秒ごとに1回（10回に1回）
  share() // 複数のObserverで共有
).subscribe(() => {
  heavyCalculation();
});
```

## 関連するCreation Functions

| Function | 違い | 使い分け |
|----------|------|----------|
| **[timer()](/guide/creation-functions/basic/timer)** | 遅延後に開始、または1回のみ発行 | 遅延実行や1回限りの処理 |
| **[fromEvent()](/guide/creation-functions/basic/fromEvent)** | イベント駆動 | ユーザー操作に応じた処理 |
| **range()** | 指定範囲の数値を即座に発行 | 時間制御が不要な場合 |

## まとめ

- `interval()`は指定間隔で連続的に値を発行
- 0から始まる連続した整数を発行
- 自動完了しないため、必ず購読解除が必要
- Cold Observableとして動作（購読ごとに独立したタイマー）
- ポーリング、定期実行、カウントダウンなどに最適

## 次のステップ

- [timer() - 遅延後に発行開始](/guide/creation-functions/basic/timer)
- [fromEvent() - イベントをObservableに変換](/guide/creation-functions/basic/fromEvent)
- [基本作成系の概要に戻る](/guide/creation-functions/basic/)
