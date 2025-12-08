---
description: "timer() - 指定時間後に値を発行するCreation Function。1回限りの遅延実行にはtimer(delay)、遅延付き定期実行にはtimer(delay, period)を使用します。interval()との使い分け、TypeScriptでの型推論、setTimeout代替としての活用法を解説します。"
---

# timer() - 遅延後に発行開始

`timer()`は、指定した遅延時間の後に値を発行し始めるCreation Functionです。1回限りの発行、または定期的な発行の両方に対応しています。

## 概要

`timer()`は、初回の発行タイミングを制御できる柔軟なCreation Functionです。引数の数によって動作が変わり、1回限りの発行も、`interval()`のような定期発行も可能です。

**シグネチャ**:
```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

**公式ドキュメント**: [📘 RxJS公式: timer()](https://rxjs.dev/api/index/function/timer)

## 基本的な使い方

`timer()`は引数の数によって動作が変わります。

### 1回限りの発行

第1引数のみを指定すると、指定時間後に0を発行して完了します。

```typescript
import { timer } from 'rxjs';

// 3秒後に0を発行して完了
const timer$ = timer(3000);

timer$.subscribe({
  next: value => console.log('値:', value),
  complete: () => console.log('完了')
});

// 3秒後の出力:
// 値: 0
// 完了
```

### 定期的な発行

第2引数に間隔を指定すると、初回遅延後に定期的に発行し続けます。

```typescript
import { timer } from 'rxjs';

// 3秒後に開始し、その後1秒ごとに値を発行
const timer$ = timer(3000, 1000);

timer$.subscribe(value => console.log('値:', value));

// 出力:
// 値: 0  （3秒後）
// 値: 1  （4秒後）
// 値: 2  （5秒後）
// ...（無限に続く）
```

## 重要な特徴

### 1. 遅延の柔軟な指定

遅延時間はミリ秒の数値か、`Date`オブジェクトで指定できます。

```typescript
import { timer } from 'rxjs';

// ミリ秒で指定
timer(5000).subscribe(() => console.log('5秒後'));

// Date オブジェクトで指定（特定の時刻に実行）
const targetTime = new Date(Date.now() + 10000); // 10秒後
timer(targetTime).subscribe(() => console.log('指定時刻に実行'));
```

### 2. 第2引数の有無で動作が変わる

第2引数を指定するかどうかで、完了するかしないかが決まります。

```typescript
import { timer } from 'rxjs';

// 第2引数なし - 1回発行して完了
timer(1000).subscribe({
  next: value => console.log('1回目:', value),
  complete: () => console.log('完了')
});

// 第2引数あり - 無限に発行し続ける
timer(1000, 1000).subscribe({
  next: value => console.log('繰り返し:', value),
  complete: () => console.log('完了（表示されない）')
});
```

> [!IMPORTANT]
> **第2引数を指定すると完了しない**
>
> `timer(1000, 1000)`のように第2引数を指定すると、`interval()`と同様に無限に発行し続けます。必ず購読解除が必要です。

### 5. Cold Observable

`timer()`はCold Observableであり、購読ごとに独立したタイマーが作成されます。

```typescript
import { timer } from 'rxjs';

const timer$ = timer(1000);

console.log('開始');

// 購読1
timer$.subscribe(() => console.log('Observer 1'));

// 500ms後に購読2を追加
setTimeout(() => {
  timer$.subscribe(() => console.log('Observer 2'));
}, 500);

// 出力:
// 開始
// Observer 1  （1秒後）
// Observer 2  （1.5秒後 - 独立したタイマー）
```

> [!NOTE]
> **Cold Observableの特徴**
> - 購読するたびに独立した実行が開始されます
> - 各購読者は独自のデータストリームを受け取ります
> - 購読ごとに独立したタイマーが開始されます。interval()と同様、共有が必要な場合は`share()`を使用してください。
>
> 詳しくは [コールドObservableとホットObservable](/guide/observables/cold-and-hot-observables) を参照してください。

## timer() vs interval() の違い

両者の主な違いは、初回発行のタイミングです。

```typescript
import { timer, interval } from 'rxjs';
import { take } from 'rxjs';

console.log('開始');

// interval() - すぐに開始（1秒後に最初の値）
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - 遅延なし（即座に最初の値）
timer(0, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// timer() - 2秒の遅延後に開始
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer(遅延):', value);
});
```

| Creation Function | 初回発行タイミング | 用途 |
|-------------------|------------------|------|
| `interval(1000)` | 1秒後 | 即座に定期実行を開始 |
| `timer(0, 1000)` | 即座 | 最初だけすぐに実行したい |
| `timer(2000, 1000)` | 2秒後 | 遅延してから定期実行 |
| `timer(2000)` | 2秒後（1回のみ） | 遅延実行（1回限り） |

## 実践的なユースケース

### 1. 遅延実行

一定時間後に1回だけ処理を実行します。

```typescript
import { from, timer } from 'rxjs';
import { switchMap } from 'rxjs';

function delayedApiCall() {
  return timer(2000).pipe(
    switchMap(() => from(
      fetch('https://jsonplaceholder.typicode.com/posts/1')
        .then(res => res.json())
    ))
  );
}

delayedApiCall().subscribe(data => {
  console.log('2秒後にデータ取得:', data);
});
```

### 2. 遅延付きポーリング

初回は即座に実行せず、一定時間後にポーリングを開始します。

```typescript
import { from, timer } from 'rxjs';
import { switchMap, retry } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

// 5秒後にポーリング開始、その後10秒ごと
const polling$ = timer(5000, 10000).pipe(
  switchMap(() => from(
    fetch('https://jsonplaceholder.typicode.com/users/1')
      .then(res => res.json() as Promise<Status>)
  )),
  retry(3) // エラー時は3回までリトライ
);

const subscription = polling$.subscribe(data => {
  console.log('ステータス更新:', data);
});

// 必要に応じて停止
// subscription.unsubscribe();
```

### 3. タイムアウト処理

一定時間内に処理が完了しない場合にタイムアウトします。

```typescript
import { timer, race, from } from 'rxjs';
import { map } from 'rxjs';

function fetchWithTimeout(url: string, timeoutMs: number) {
  const request$ = from(fetch(url).then(res => res.json()));
  const timeout$ = timer(timeoutMs).pipe(
    map(() => {
      throw new Error('タイムアウト');
    })
  );

  // どちらか早い方を採用
  return race(request$, timeout$);
}

fetchWithTimeout('https://jsonplaceholder.typicode.com/posts/1', 3000).subscribe({
  next: data => console.log('データ取得:', data),
  error: err => console.error('エラー:', err.message)
});
```

### 4. 通知の自動非表示

通知を表示してから一定時間後に自動的に非表示にします。

```typescript
import { timer, Subject, map } from 'rxjs';
import { switchMap, takeUntil } from 'rxjs';

interface Notification {
  id: number;
  message: string;
}

const notifications$ = new Subject<Notification>();
const dismiss$ = new Subject<number>();

notifications$.pipe(
  switchMap(notification => {
    console.log('通知表示:', notification.message);

    // 5秒後に自動非表示
    return timer(5000).pipe(
      takeUntil(dismiss$), // 手動で非表示にされたら中止
      map(() => notification.id)
    );
  })
).subscribe(id => {
  console.log('通知を非表示:', id);
});

// 通知を表示
notifications$.next({ id: 1, message: '新しいメッセージが届きました' });

// 手動で非表示にする場合
// dismiss$.next(1);
```

## パイプラインでの使用

`timer()`は、遅延処理や定期実行の起点として使用されます。

```typescript
import { timer } from 'rxjs';
import { map, take, scan } from 'rxjs';

// カウントダウンタイマー（10秒から0秒まで）
timer(0, 1000).pipe(
  map(count => 10 - count),
  take(11), // 0から10まで（11個）
  scan((acc, curr) => curr, 0)
).subscribe({
  next: time => console.log(`残り: ${time}秒`),
  complete: () => console.log('タイマー終了')
});

// 出力:
// 残り: 10秒
// 残り: 9秒
// ...
// 残り: 0秒
// タイマー終了
```

## よくある間違い

### 1. 第2引数を指定して購読解除を忘れる

```typescript
// ❌ 間違い - 第2引数を指定すると無限に実行される
import { timer } from 'rxjs';

function startTimer() {
  timer(1000, 1000).subscribe(value => {
    console.log('値:', value); // 永遠に実行され続ける
  });
}

startTimer();

// ✅ 正しい - 購読を保持し、必要に応じて解除
import { timer, Subscription } from 'rxjs';
import { take } from 'rxjs';

let subscription: Subscription | null = null;

function startTimer() {
  subscription = timer(1000, 1000).pipe(
    take(10) // 10回で自動完了
  ).subscribe(value => {
    console.log('値:', value);
  });
}

function stopTimer() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startTimer();
```

### 2. interval()との違いを理解していない

```typescript
// ❌ 混同 - interval()は即座に開始（1秒後に最初の値）
import { interval } from 'rxjs';

interval(1000).subscribe(value => {
  console.log('interval:', value); // 1秒後に0が出力
});

// ✅ timer() - 遅延なしで即座に最初の値を発行したい場合
import { timer } from 'rxjs';

timer(0, 1000).subscribe(value => {
  console.log('timer:', value); // 即座に0が出力
});
```

## パフォーマンスの考慮事項

`timer()`は軽量ですが、使い方によってはパフォーマンスに影響を与えます。

> [!TIP]
> **最適化のヒント**:
> - 1回限りの実行には第2引数を指定しない
> - 不要になったら必ず購読解除
> - 複数のObserverが必要な場合は`share()`で共有
> - 短い間隔（100ms以下）での使用は慎重に

```typescript
import { timer } from 'rxjs';
import { share } from 'rxjs';

// ❌ パフォーマンス問題 - 複数の独立したタイマー
const timer$ = timer(0, 1000);

timer$.subscribe(value => console.log('Observer 1:', value));
timer$.subscribe(value => console.log('Observer 2:', value));
// 2つのタイマーが並行して動作

// ✅ 最適化 - 1つのタイマーを共有
const sharedTimer$ = timer(0, 1000).pipe(share());

sharedTimer$.subscribe(value => console.log('Observer 1:', value));
sharedTimer$.subscribe(value => console.log('Observer 2:', value));
// 1つのタイマーが共有される
```

## 関連するCreation Functions

| Function | 違い | 使い分け |
|----------|------|----------|
| **[interval()](/guide/creation-functions/basic/interval)** | 即座に開始（遅延なし） | 遅延不要な定期実行 |
| **[of()](/guide/creation-functions/basic/of)** | 同期的に即座に発行 | 非同期不要な場合 |
| **defer()** | 購読時に処理を遅延 | 動的な値の生成 |

## まとめ

- `timer()`は遅延後に発行開始するCreation Function
- 第2引数なし: 1回限りの発行（完了する）
- 第2引数あり: 定期的な発行（完了しない）
- 遅延時間はミリ秒または`Date`オブジェクトで指定
- 遅延実行、遅延付きポーリング、タイムアウト処理に最適

## 次のステップ

- [interval() - 指定間隔で連続発行](/guide/creation-functions/basic/interval)
- [defer() - 購読時に生成を遅延](/guide/creation-functions/conditional/defer)
- [基本作成系の概要に戻る](/guide/creation-functions/basic/)
