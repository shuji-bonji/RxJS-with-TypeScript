# Observableの作成方法

このページでは、RxJSでのObservableの作成方法について、基本的な構文から実践的な用途までを網羅的に紹介します。

## 1. `new Observable()`を使用する方法

最も基本的な方法は、`Observable`コンストラクタを直接使用することです。この方法はカスタムなObservableロジックを定義したい場合に最も柔軟です。明示的な `next`, `error`, `complete` 呼び出しによって細かな挙動制御が可能です。

```ts
import { Observable } from 'rxjs';

const observable$ = new Observable<number>(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  setTimeout(() => {
    subscriber.next(4);
    subscriber.complete();
  }, 1000);
});

observable$.subscribe({
  next: value => console.log('値:', value),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});
```
#### 実行結果
```sh
値: 1
値: 2
値: 3
値: 4
完了
```

## 2. 作成操作子（Creation Operators）を使用する方法

より簡潔で用途に特化したObservable作成には、RxJSが提供する「作成操作子（creation operator）」が便利です。繰り返し使われるユースケースにはこれらを使うことでコードが簡素化されます。

### `of()` - 指定した値をシーケンスとして発行

```ts
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('値:', value),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});
// 出力: 値: 1, 値: 2, 値: 3, 値: 4, 値: 5, 完了
```

### `from()` - 配列やPromiseなどからObservableを作成

```ts
import { from } from 'rxjs';

// 配列から作成
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('配列値:', value),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});

// Promiseから作成
const promise$ = from(Promise.resolve('Promiseの結果'));
promise$.subscribe({
  next: value => console.log('Promise結果:', value),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});

// イテラブルから作成
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('イテラブル値:', value),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});
```

#### 実行結果
```sh
配列値: 1
配列値: 2
配列値: 3
イテラブル値: 1
イテラブル値: 2
イテラブル値: 3
Promise結果: Promiseの結果
```

### `fromEvent()` - DOMイベントからObservableを作成

```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe({
  next: event => console.log('クリックイベント:', event),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});
```

#### 実行結果
```sh
クリックイベント: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### `interval()` と `timer()` - 時間ベースのObservable

```ts
import { interval, timer } from 'rxjs';

// 1秒ごとに値を発行
const interval$ = interval(1000);
interval$.subscribe({
  next: value => console.log('インターバル:', value),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});

// 3秒後に開始し、その後1秒ごとに値を発行
const timer$ = timer(3000, 1000);
timer$.subscribe({
  next: value => console.log('タイマー:', value),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});
```

#### 実行結果
```sh
インターバル: 0
インターバル: 1
インターバル: 2
タイマー: 0
インターバル: 3
タイマー: 1
インターバル: 4
タイマー: 2
.
.
.

```
`interval()` と `timer()` は時間制御に関する処理で頻繁に使われ、特にアニメーション、ポーリング、非同期イベント遅延などに適しています。


### `ajax()` - HTTPリクエスト用Observable

```ts
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
api$.subscribe({
  next: response => console.log('APIレスポンス:', response),
  error: error => console.error('APIエラー:', error),
  complete: () => console.log('API完了')
});
```

#### 実行結果
```sh
APIレスポンス: {userId: 1, id: 1, title: 'delectus aut autem', completed: false}
 API完了
```

## 3. `defer()` - Observableファクトリの遅延実行

```ts
import { defer, of } from 'rxjs';

const deferred$ = defer(() => {
  const randomValue = Math.random();
  return randomValue > 0.5 ? 
    of('50%より大きい値:', randomValue) : 
    of('50%以下の値:', randomValue);
});

// 購読するごとに新しいObservableが作成される
deferred$.subscribe(value => console.log(value));
deferred$.subscribe(value => console.log(value));
```

#### 実行結果
```sh
50%以下の値:
0.08011364416212319
50%以下の値:
0.3141403962502316
```
`defer()` は副作用のある処理をObservable作成時ではなく購読時に遅延させたい場合に有効です。ランダム生成や現在時刻の取得などの用途に適しています。

## 4. `Subject` - ObservableとObserverの両方の特性を持つ

```ts
import { Subject } from 'rxjs';

const subject$ = new Subject<number>();

// Observerとして使用
subject$.subscribe(value => console.log('Observer 1:', value));
subject$.subscribe(value => console.log('Observer 2:', value));

// Observableとして使用
subject$.next(1);
subject$.next(2);
subject$.next(3);
subject$.complete();
```

#### 実行結果
```ts
Observer 1: 1
Observer 2: 1
Observer 1: 2
Observer 2: 2
Observer 1: 3
Observer 2: 3
```

`Subject` は「ホットObservable」として動作するため、購読タイミングによっては過去の値を受け取れない点に注意が必要です。

### 5. 特殊な Observable の生成

実行制御や例外処理、学習用として役立つ特殊なObservableもRxJSには用意されています。

```ts
import { EMPTY, throwError, NEVER } from 'rxjs';

// 即座に完了するObservable
const empty$ = EMPTY;
empty$.subscribe({
  next: () => console.log('これは表示されない'),
  complete: () => console.log('即座に完了')
});

// エラーを発行するObservable
const error$ = throwError(() => new Error('エラー発生'));
error$.subscribe({
  next: () => console.log('これは表示されない'),
  error: err => console.error('エラー:', err.message),
  complete: () => console.log('完了')
});

// 何も発行せず、完了もしないObservable
const never$ = NEVER;
never$.subscribe({
  next: () => console.log('これは表示されない'),
  complete: () => console.log('これも表示されない')
});
```
#### 実行結果
```sh
即座に完了
main.ts:18 エラー: エラー発生
```

RxJSのストリームは、従来のJavaScriptのイベント処理やAJAX通信などを統一的なインターフェイスで扱えるようにします。特に時間的に変化するデータを扱う場合や、複数のイベントソースを組み合わせる場合に威力を発揮します。

## Observable作成手法まとめ表

| 操作子 / API | 主な用途 | 特徴 |
|--------------|----------|------|
| `of()`       | 単純な値の列 | 完了も発行される |
| `from()`     | 配列/Promiseなど | 複数データソースに対応 |
| `fromEvent()`| DOMイベント | UI連携に便利 |
| `interval()` | 定期的なイベント | 時間ベース |
| `timer()`    | 遅延 + 定期イベント | `interval`の上位版 |
| `defer()`    | 実行タイミングを遅延 | 購読ごとに新しいObservable |
| `Subject`    | 双方向通信 / マルチキャスト | `next()`手動発火が可能 |
| `EMPTY`      | すぐ完了する | `next()`は呼ばれない |
| `NEVER`      | 何もしない | 学習用途に便利 |
| `throwError()` | エラーを即発行 | エラーハンドリング検証に便利 |
