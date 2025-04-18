# RxJSの主要概念

RxJSを理解するには、その中核となる概念を把握することが重要です。この文書では、RxJSの主要な構成要素とその使い方について詳しく説明します。

## 主要概念で重要なキーワード

|用語|説明|
|---|---|
|[Observable](#observable)|時間の経過とともにデータを発行するストリーム（ストリーミングデータの源）|
|[Observer](#observer)|Observableからの通知（データ）を受け取る存在|
|[Subscription](#subscription)|Observable の購読。データを受け取り始める操作|
|[Operator](#operator)|ストリームを変換・フィルター・合成などする関数群（例：map, filter, mergeMap）|
|[Subject](#subject)|Observer でもあり Observable でもあるハイブリッドな存在。複数購読者に同じ値を流すときに使う|
|[Scheduler](#scheduler)|実行タイミングやスレッド制御を行う仕組み（高度な制御用途）|

## オペレーターの分類と用途

| 種類 | 代表オペレーター | 用途の例 |
|------|------------------|---------|
| 変換 | map, scan | 値を変換したり、累積処理を行う |
| フィルタ | filter, take, debounceTime | 条件に合う値のみを通す、入力制御など |
| 結合 | merge, concat, combineLatest | 複数のObservableを結合・合成 |
| エラー処理 | catchError, retry | エラー発生時のフォールバックや再試行 |

## Observable

### 基本概念

Observableは、RxJSの中心的な概念で、「時間の経過とともに発生する値のシーケンス」を表現します。これは非同期データストリームを抽象化したものです。

```ts
import { Observable } from 'rxjs';

// 基本的なObservableの作成
const observable = new Observable<number>(subscriber => {
  subscriber.next(1);      // 値を発行
  subscriber.next(2);
  subscriber.next(3);
  
  // 非同期で値を発行
  setTimeout(() => {
    subscriber.next(4);
    subscriber.complete(); // 完了を通知
  }, 1000);
});
```

Observableは`subscribe()`を呼び出すまで何も実行されない「遅延評価」の特性を持ちます。

### Observableの作成

RxJSでは、さまざまな方法でObservableを作成できます。

```ts
import { of, from, fromEvent, interval } from 'rxjs';

// 固定値からのObservable
const ofObservable = of(1, 2, 3);

// 配列からのObservable
const arrayObservable = from([1, 2, 3]);

// DOMイベントからのObservable
const clickObservable = fromEvent(document, 'click');

// 定期的な間隔でのObservable
const intervalObservable = interval(1000); // 1秒ごとに値を発行
```

## Observer

Observerは、Observableから発行される値を受け取るインターフェースです。三つの主要なコールバック関数があります。

```ts
import { Observable } from 'rxjs';
// Observableで値を発行する。
const observable = new Observable<number>(subscriber => {
   // 省略 Observable の基礎概念を参照
});
// Observer で値を受け取る 👈
const observer = {
  next: (value: any) => console.log('受信した値:', value),
  error: (error: any) => console.error('エラー発生:', error),
  complete: () => console.log('完了通知を受信')
};

// Observerを使用してObservableを購読
observable.subscribe(observer);
```

#### 実行結果
```sh
受信した値: 1
受信した値: 2
受信した値: 3
受信した値: 4
完了通知を受信
```

上記以外に、[Observableの作成](#observableの作成) のサンプルコード例のそれぞれの Observable の値を試しに受け取ってみるのも良い。

## Subscription

Subscription は、Observableの実行を表し、主にその実行をキャンセルするために使用されます。

```ts
import { interval } from 'rxjs';

// 1秒ごとに値を発行するObservable
const observable = interval(1000);

// 購読開始
const subscription = observable.subscribe(value => console.log(value)); // 👈

// 5秒後に購読を解除（キャンセル）
setTimeout(() => {
  subscription.unsubscribe();
  console.log('購読を解除しました');
}, 5000);
```

メモリリークを防ぐために、不要になったObservableは確実に`unsubscribe()`することが重要です。

#### 実行結果
```sh
0
1
2
3
4
購読を解除しました
```

## Operator

オペレーターは、Observableを変換するための関数です。パイプ(`pipe()`)を使用して、複数のオペレーターを連鎖させることができます。

### 変換オペレーター

```ts
import { of } from 'rxjs';
import { map, scan } from 'rxjs/operators';

const source = of(1, 2, 3, 4, 5);

source.pipe(
  map(x => x * 2) // 各値を2倍に変換 👈 map オペレーター
).subscribe(value => console.log(`map: ${value}`));
// 出力: map: 2, map: 4, map: 6, map: 8, map: 10

source.pipe(
  scan((acc, curr) => acc + curr, 0) // 累積値を計算 👈 scan オペレーター
).subscribe(value => console.log(`scan: ${value}`));
// 出力: scan: 1, scan: 3, scan: 6, scan: 10, scan: 15
```

### フィルタリングオペレーター

```ts
import { of } from 'rxjs';
import { filter, take, debounceTime } from 'rxjs/operators';

const source = of(1, 2, 3, 4, 5);

source.pipe(
  filter(x => x % 2 === 0) // 偶数のみをフィルタリング 👈 filter オペレーター
).subscribe(value => console.log(`filter: ${value}`));
// 出力: filter: 2, filter: 4

source.pipe(
  take(3) // 最初のn個の値だけを取得 👈 take オペレーター
).subscribe(value => console.log(`take: ${value}`));
// 出力: take: 1, take: 2, take: 3

// フォームの入力などに有用
fromEvent(inputElement, 'input').pipe(
  debounceTime(300) // 300ms間隔をあけてイベントを発行 👈 debounceTime オペレーター
).subscribe();
```

### 結合オペレーター

```ts
import { combineLatest, merge, concat } from 'rxjs';
import { map } from 'rxjs/operators';

const observable1 = interval(1000).pipe(map(x => `First: ${x}`));
const observable2 = interval(1500).pipe(map(x => `Second: ${x}`));

// 最新の値を組み合わせる
combineLatest([observable1, observable2]).subscribe(
  ([first, second]) => console.log(`Combine Latest: ${first}, ${second}`)
);

// 複数のObservableをマージする
merge(observable1, observable2).subscribe(
  value => console.log(`Merged: ${value}`)
);

// Observableを順番に連結する
concat(observable1.pipe(take(3)), observable2.pipe(take(2))).subscribe(
  value => console.log(`Concatenated: ${value}`)
);
```

## Subject

Subjectは、ObservableとObserverの両方の性質を持ちます。値を発行し、購読することができます。

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

// Observerとして機能
subject.subscribe(value => console.log(`Observer A: ${value}`));
subject.subscribe(value => console.log(`Observer B: ${value}`));

// Observableとして機能
subject.next(1);
subject.next(2);

// 出力:
// Observer A: 1
// Observer B: 1
// Observer A: 2
// Observer B: 2
```

### Subject変種

RxJSには、特殊な動作をする複数のSubject変種があります：

```ts
import { BehaviorSubject, ReplaySubject, AsyncSubject } from 'rxjs';

// 初期値を持ち、最新の値を新しい購読者に提供する
const behaviorSubject = new BehaviorSubject<number>(0);
behaviorSubject.subscribe(value => console.log(`Behavior Observer: ${value}`));
// 出力: Behavior Observer: 0
behaviorSubject.next(1);
// 出力: Behavior Observer: 1

// 指定された数の過去の値を新しい購読者に再生する
const replaySubject = new ReplaySubject<number>(2); // 最新の2つの値を保持
replaySubject.next(1);
replaySubject.next(2);
replaySubject.next(3);
replaySubject.subscribe(value => console.log(`Replay Observer: ${value}`));
// 出力: Replay Observer: 2, Replay Observer: 3

// completeが呼ばれた時点で最後の値のみを発行する
const asyncSubject = new AsyncSubject<number>();
asyncSubject.subscribe(value => console.log(`Async Observer: ${value}`));
asyncSubject.next(1);
asyncSubject.next(2);
asyncSubject.next(3);
asyncSubject.complete();
// 出力: Async Observer: 3
```

Subjectは、フォームやユーザー操作イベントの中継、状態管理などでも広く利用されます。

## Scheduler

スケジューラーは、Observableの実行コンテキストを制御します。これにより、コードが実行されるタイミングと場所を制御できます。

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs/operators';

console.log('開始');

of(1, 2, 3).pipe(
  observeOn(asyncScheduler)
).subscribe({
  next: value => console.log(`値: ${value}`),
  complete: () => console.log('完了')
});

console.log('購読後');

// 出力順序:
// 開始
// 購読後
// 値: 1
// 値: 2
// 値: 3
// 完了
```

## エラー処理

RxJSでは、エラー処理も非常に重要です。`catchError`オペレーターを使用してエラーを処理できます。

```ts
import { of, throwError } from 'rxjs';
import { catchError, retry } from 'rxjs/operators';

const source = throwError(() => new Error('エラーが発生しました'));

source.pipe(
  catchError(error => {
    console.log(`エラーをキャッチしました: ${error.message}`);
    return of('エラー後のフォールバック値');
  })
).subscribe({
  next: value => console.log(`次の値: ${value}`),
  error: err => console.log(`エラー: ${err.message}`),
  complete: () => console.log('完了')
});

// HTTPリクエストの再試行の例
httpGet('/api/data').pipe(
  retry(3), // 失敗した場合、3回再試行
  catchError(error => of({ error: true }))
).subscribe();
```

## まとめ

RxJSの主要概念を理解することで、複雑な非同期処理を効果的に管理できるようになります。これらの概念は、実際のアプリケーション開発において強力なツールとなります。

