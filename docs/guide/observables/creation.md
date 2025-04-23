# Observableの作成方法

Observableとは「データのストリーム」を定義するものであり、それを作成する方法は多岐にわたります。  
RxJSでは、カスタムObservableの作成や、イベント・配列・HTTPレスポンスなどから簡単にObservableを生成できるよう、さまざまな手段が提供されています.

ここでは、RxJSでのObservableの作成方法について、基本的な構文から実践的な用途までを網羅的に紹介します.

## Observable作成手法の分類

以下は主な作成手法のカテゴリ別一覧です。

| カテゴリ | 主な手法 | 説明 |
|----------|----------|------|
| カスタム作成 | [`new Observable()`](#new-observable) | 自由度が高いが記述量も多い。手動でクリーンアップが必要 |
| 作成演算子 | [`of()`](#of), [`from()`](#from), [`fromEvent()`](#fromevent), [`interval()`](#interval-timer), [`timer()`](#interval-timer), [`ajax()`](#ajax) | よく使われるデータ・イベント・時間ベースの生成関数群 |
| 特殊な作成演算子 | [`defer()`](#defer), [`range()`](#range), [`generate()`](#generate), [`iif()`](#iif) | 制御的・ループ的な生成、条件による切り替えなど |
| 特殊Observable | [`EMPTY`](#empty-never-throwerror), [`NEVER`](#empty-never-throwerror), [`throwError()`](#empty-never-throwerror) | 完了・何もしない・エラー発行用 |
| Subject系 | [`Subject`](#subject-behaviorsubject-など), [`BehaviorSubject`](#subject-behaviorsubject-など) | 観測者としても送信者としても機能する特殊なObservable |
| コールバック変換 | [`bindCallback()`](#bindcallback), [`bindNodeCallback()`](#bindnodecallback) | コールバックベースの関数をObservableに変換 |
| WebSocket | [`webSocket()`](#websocket) | WebSocket通信を双方向Observableとして扱う |



## カスタム作成

### new Observable()

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

> [!CAUTION]
> `new Observable()` を使う場合は、明示的なリソース解放（クリーンアップ処理）を自分で記述する必要があります。
> ```ts
> const obs$ = new Observable(subscriber => {
>   const id = setInterval(() => subscriber.next(Date.now()), 1000);
>   return () => {
>     clearInterval(id); // 明示的なリソース解放
>   };
> });
> ```
> 一方、`fromEvent()` や `interval()` などRxJSのビルトイン作成関数は、内部に適切なクリーンアップ処理を持ちます。
> ```ts
> const click$ = fromEvent(document, 'click');
> const timer$ = interval(1000);
> ```
> これらは内部で `addEventListener` や `setInterval` を使用していて、`unsubscribe()` 時に RxJS が自動で `removeEventListener` や `clearInterval()` を呼ぶように設計されています。
> 
> なお、RxJSの内部でクリーンアップ処理が実装されていても、`unsubscribe()`を呼ばなければその処理は実行されないため注意が必要です。
> ```ts
>  const subscription = observable$.subscribe({
>  //省略...
>  });
>
>  subscription.unsubscribe(); // 👈 
> ```
> - どのObservableの作成方法でも、不要になったら必ず`unsubscribe()`する習慣を持ちましょう。
> - 購読解除を忘れると、イベントリスナーやタイマーが動き続けるため、メモリリークや予期せぬ副作用の原因になります。


## 作成演算子

より簡潔で用途に特化したObservable作成には、RxJSが提供する「作成操作子（creation operator）」が便利です。繰り返し使われるユースケースにはこれらを使うことでコードが簡素化されます。

### of()

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

> [!IMPORTANT]
> `of()` と `from()` の違い  
> - `of([1, 2, 3])` → 1つの配列を発行します。  
> - `from([1, 2, 3])` → 個別の値 `1`, `2`, `3` を順に発行します。  
> - よく混同されるので注意が必要です。

### from()

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
完了
イテラブル値: 1
イテラブル値: 2
イテラブル値: 3
完了
Promise結果: Promiseの結果
完了
```

### fromEvent()

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

> [!CAUTION]
> DOM以外では使えないことに注意  
> - `fromEvent()` はブラウザ環境でのみ利用でき、Node.jsでは利用できません。  
> - 複数回購読すると、複数のイベントリスナーが追加される可能性があります。

> 👉 より詳細なイベントストリームの活用例については、[イベントのストリーム化](../observables/events) を参照してください。

### interval(), timer()

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

> [!CAUTION]
> Cold Observable である点に注意  
> - `interval()` や `timer()` は Cold Observable であり、購読のたびに独立して実行されます。  
> - 必要に応じて `share()` などでHot化することも検討できます。

### ajax()

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

## 特殊な作成演算子

### defer()

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

> [!IMPORTANT]
> `of()`との違い  
> - `of()` は作成時点で値が確定します。  
> - `defer()` は購読時に初めて処理されるため、購読するたびに値が変わるような処理に適しています。

### range()

```ts
import { range } from 'rxjs';

const range$ = range(5, 3); // 5から3つ → 5, 6, 7
range$.subscribe({
  next: val => console.log('range:', val),
  complete: () => console.log('完了')
});
```

#### 実行結果
```sh
range: 5
range: 6
range: 7
完了
```

### generate()

```ts
import { generate } from 'rxjs';

const generate$ = generate({
  initialState: 0,
  condition: x => x < 5,
  iterate: x => x + 1
});

generate$.subscribe({
  next: val => console.log('generate:', val),
  complete: () => console.log('完了')
});
```

#### 実行結果
```sh
generate: 0
generate: 1
generate: 2
generate: 3
generate: 4
完了
```

### iif()

```ts
import { iif, of, EMPTY } from 'rxjs';

const condition = true;
const iif$ = iif(() => condition, of('条件はtrue'), EMPTY);

iif$.subscribe({
  next: val => console.log('iif:', val),
  complete: () => console.log('完了')
});
```

#### 実行結果
```sh
iif: 条件はtrue
完了
```

> [!NOTE]
> `iif()` は条件によって返すObservableを動的に切り替えることができます。フロー制御に便利です。

## 特殊Observable

### EMPTY, NEVER, throwError()

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
エラー: エラー発生
```

> [!IMPORTANT]
> 主に制御・検証・学習用途  
> - `EMPTY`, `NEVER`, `throwError()` は、通常のデータストリームではなく、**フロー制御や例外ハンドリングの検証**、または学習用途で活用されます。


## Subject系

### Subject, BehaviorSubject など

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
```sh
Observer 1: 1
Observer 2: 1
Observer 1: 2
Observer 2: 2
Observer 1: 3
Observer 2: 3
```

> [!IMPORTANT]
> Hot Observableであることに注意  
> - `Subject` は購読者に「同時に」通知されるため、`from()` や `of()` などの Cold Observable とは異なり、**購読タイミングによって値を受け取れないことがあります**。

詳しくは、[「Subjectとは」](../subjects/what-is-subject.md)を参照してください。

## コールバック変換

RxJSには、コールバックベースの非同期関数をObservableに変換するための関数として `bindCallback()` および `bindNodeCallback()` が用意されています。

### bindCallback()

```ts
import { bindCallback } from 'rxjs';

function asyncFn(input: string, callback: (result: string) => void) {
  setTimeout(() => callback(`Hello, ${input}`), 1000);
}

const observableFn = bindCallback(asyncFn);
const result$ = observableFn('RxJS');

result$.subscribe({
  next: val => console.log(val), // Hello, RxJS
  complete: () => console.log('完了')
});
```

### bindNodeCallback()

```ts
import { bindNodeCallback } from 'rxjs';
import { readFile } from 'fs';

const readFile$ = bindNodeCallback(readFile);
readFile__('./some.txt', 'utf8').subscribe({
  next: data => console.log('内容:', data),
  error: err => console.error('エラー:', err)
});
```

> [!NOTE]
> `bindNodeCallback()` は Node.js の `(err, result)` 型の非同期関数に対応しています。

---

## WebSocket()

RxJSの `rxjs/webSocket` モジュールには、WebSocketをObservable/Observerとして扱える `webSocket()` 関数が用意されています。

```ts
import { webSocket } from 'rxjs/webSocket';

const socket$ = webSocket('wss://echo.websocket.org');

socket$.subscribe({
  next: msg => console.log('受信:', msg),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});

// メッセージ送信（Observerとしての機能）
socket$.next('Hello WebSocket!');
```

> [!IMPORTANT]
> `webSocket()` は双方向通信が可能な Observable/Observer ハイブリッドです。
> WebSocketの接続・送信・受信を簡単にObservableとして扱えるため、リアルタイム通信に便利です。


## まとめ

RxJSのストリームは、従来のJavaScriptのイベント処理やAJAX通信などを統一的なインターフェイスで扱えるようにします。特に時間的に変化するデータを扱う場合や、複数のイベントソースを組み合わせる場合に威力を発揮します。

