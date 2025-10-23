---
description: RxJSにおけるObservableの作成方法を、ofやfromなどの基本的な生成関数から、カスタムObservableの定義、HTTP通信、イベントストリーム化まで、実践的なコード例とともに網羅的に解説します。
---
# Observableの作成方法

Observableとは「データのストリーム」を定義するものであり、それを作成する方法は多岐にわたります。  
RxJSでは、カスタムObservableの作成や、イベント・配列・HTTPレスポンスなどから簡単にObservableを生成できるよう、さまざまな手段が提供されています。

ここでは、RxJSでのObservableの作成方法について、基本的な構文から実践的な用途までを網羅的に紹介します。

## Observable作成手法の分類

以下は主な作成手法のカテゴリ別一覧です。

| カテゴリ | 主な手法 | 説明 |
|----------|----------|------|
| カスタム作成 | [`new Observable()`](#new-observable) | 自由度が高いが記述量も多い。手動でクリーンアップが必要 |
| Creation Functions | [`of()`](#of), [`from()`](#from), [`fromEvent()`](#fromevent), [`interval()`](#interval-timer), [`timer()`](#interval-timer), [`ajax()`](#ajax), [`fromFetch()`](#fromfetch), [`scheduled()`](#scheduled) | よく使われるデータ・イベント・時間ベースの生成関数群 |
| 特殊な Creation Functions | [`defer()`](#defer), [`range()`](#range), [`generate()`](#generate), [`iif()`](#iif) | 制御的・ループ的な生成、条件による切り替えなど |
| 特殊Observable | [`EMPTY`](#empty-never-throwerror), [`NEVER`](#empty-never-throwerror), [`throwError()`](#empty-never-throwerror) | 完了・何もしない・エラー発行用 |
| Subject系 | [`Subject`](#subject-behaviorsubject), [`BehaviorSubject`](#subject-behaviorsubject) | 観測者としても送信者としても機能する特殊なObservable |
| コールバック変換 | [`bindCallback()`](#bindcallback), [`bindNodeCallback()`](#bindnodecallback) | コールバックベースの関数をObservableに変換 |
| リソース制御 | [`using()`](#using) | Observableの購読と同時にリソース制御を行う |
| WebSocket | [`webSocket()`](#websocket) | WebSocket通信を双方向Observableとして扱う |



## カスタム作成

### new Observable()
[📘 RxJS公式: Observable](https://rxjs.dev/api/index/class/Observable)


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
// 出力:
// 値: 1
// 値: 2
// 値: 3
// 値: 4
// 完了
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


## Creation Functions（作成関数）

より簡潔で用途に特化したObservable作成には、RxJSが提供する「Creation Functions（作成関数）」が便利です。繰り返し使われるユースケースにはこれらを使うことでコードが簡素化されます。

> [!NOTE]
> RxJS公式ではこれらは「Creation Functions（作成関数）」と分類されています。
> 従来（RxJS 5.x ~ 6）は「creation operator（作成演算子）」と呼ばれていましたが、RxJS 7以降は「Creation Functions」が正式な用語です。


### of()
[📘 RxJS公式: of()](https://rxjs.dev/api/index/function/of)

複数の値を**1つずつ順に発行**するもっともシンプルなObservable作成関数です。


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
>
> よく混同されるので注意が必要です。

> [!TIP]
> 詳細な使い方と実践例は [of() の詳細ページ](/guide/creation-functions/basic/of) を参照してください。

### from()
[📘 RxJS公式: from()](https://rxjs.dev/api/index/function/from)

配列・Promise・イテラブルなど、**既存のデータ構造からObservableを生成**します。

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

// 出力:
// 配列値: 1
// 配列値: 2
// 配列値: 3
// 完了
// イテラブル値: 1
// イテラブル値: 2
// イテラブル値: 3
// 完了
// Promise結果: Promiseの結果
// 完了
```

> [!TIP]
> 詳細な使い方と実践例は [from() の詳細ページ](/guide/creation-functions/basic/from) を参照してください。

### fromEvent()
[📘 RxJS公式: fromEvent](https://rxjs.dev/api/index/function/fromEvent)

DOMイベントなど、**イベントソースをObservableとして扱う**ための関数です。

```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe({
  next: event => console.log('クリックイベント:', event),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});

// 出力:
// クリックイベント: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!CAUTION]
> DOM以外では使えないことに注意
> - `fromEvent()` はブラウザ環境でのみ利用でき、Node.jsでは利用できません。
> - 複数回購読すると、複数のイベントリスナーが追加される可能性があります。

> 👉 より詳細なイベントストリームの活用例については、[イベントのストリーム化](../observables/events) を参照してください。

> [!TIP]
> 詳細な使い方と実践例は [fromEvent() の詳細ページ](/guide/creation-functions/basic/fromEvent) を参照してください。

### interval(), timer()
[📘 RxJS公式: interval](https://rxjs.dev/api/index/function/interval), [📘 RxJS公式: timer](https://rxjs.dev/api/index/function/timer)

一定間隔で連続的に値を発行したいときや、**時間制御が必要な場合**に使われます。

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

// 出力:
// インターバル: 0
// インターバル: 1
// インターバル: 2
// タイマー: 0
// インターバル: 3
// タイマー: 1
// インターバル: 4
// タイマー: 2
// .
// .
```
`interval()` と `timer()` は時間制御に関する処理で頻繁に使われ、特にアニメーション、ポーリング、非同期イベント遅延などに適しています。

> [!CAUTION]
> Cold Observable である点に注意
> - `interval()` や `timer()` は Cold Observable であり、購読のたびに独立して実行されます。
> - 必要に応じて `share()` などでHot化することも検討できます。
>
> 詳しくは「[コールドObservableとホットObservable](./cold-and-hot-observables.md)」のセクションでを参照してください。

> [!TIP]
> 詳細な使い方と実践例は [interval() の詳細ページ](/guide/creation-functions/basic/interval) と [timer() の詳細ページ](/guide/creation-functions/basic/timer) を参照してください。

### ajax()
[📘 RxJS公式: ajax](https://rxjs.dev/api/ajax/ajax)

HTTP通信の結果を**Observableとして非同期的に扱う**ための関数です。

```ts
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
api$.subscribe({
  next: response => console.log('APIレスポンス:', response),
  error: error => console.error('APIエラー:', error),
  complete: () => console.log('API完了')
});

// 出力:
// APIレスポンス: {userId: 1, id: 1, title: 'delectus aut autem', completed: false}
//  API完了
```

> [!NOTE]
> RxJSのajaxは、内部的にはXMLHttpRequestを使用しています。一方、RxJSにはfromFetchというオペレーターもあり、これはFetch APIを利用してHTTPリクエストを行います。

> [!TIP]
> 詳細な使い方と実践例は [ajax() の詳細ページ](/guide/creation-functions/http-communication/ajax) を参照してください。HTTP通信系の概要は [HTTP通信系 Creation Functions](/guide/creation-functions/http-communication/) を参照してください。

### fromFetch()
[📘 RxJS公式: fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` は Fetch API をラップして、HTTP リクエストを Observable として扱うことができる関数です。  
`ajax()` と似ていますが、こちらはよりモダンで軽量です。

```ts
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs';

const api$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1');

api$.pipe(
  switchMap(response => response.json())
).subscribe({
  next: data => console.log('データ:', data),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});

// 出力:
// データ: Objectcompleted: falseid: 1title: "delectus aut autem"userId: 1[[Prototype]]: Object
// 完了
```

> [!NOTE]
> `fromFetch()` は Fetch API を使用しているため、`ajax()` と異なりリクエスト設定の初期化やレスポンスの `.json()` 変換は手動で行う必要があります。
> エラーハンドリングや HTTP ステータスのチェックなども適切に行う必要があります。

> [!TIP]
> 詳細な使い方と実践例は [fromFetch() の詳細ページ](/guide/creation-functions/http-communication/fromFetch) を参照してください。HTTP通信系の概要は [HTTP通信系 Creation Functions](/guide/creation-functions/http-communication/) を参照してください。

### scheduled()
[📘 RxJS公式: scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` は `of()` や `from()` などの発行関数にスケジューラを明示的に指定できる関数です。  
同期・非同期の実行タイミングを細かく制御したい場合に使用します。

```ts
import { scheduled, asyncScheduler } from 'rxjs';

const observable$ = scheduled([1, 2, 3], asyncScheduler);
observable$.subscribe({
  next: val => console.log('値:', val),
  complete: () => console.log('完了')
});

// 実行は非同期で行われる
// 出力:
// 値: 1
// 値: 2
// 値: 3
// 完了
```

> [!NOTE]
> `scheduled()` を使用することで、既存の同期関数（例: `of()`, `from()`）を非同期で動作させることが可能になります。
> 非同期での処理制御が求められるテストやUIパフォーマンス最適化に役立ちます。

> [!TIP]
> 詳細な使い方と実践例は [scheduled() の詳細ページ](/guide/creation-functions/control/scheduled) を参照してください。制御系の概要は [制御系 Creation Functions](/guide/creation-functions/control/) を参照してください。

### defer()
[📘 RxJS公式: defer](https://rxjs.dev/api/index/function/defer)

Observableの生成を**購読時まで遅延させたいとき**に使用されます。

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

// 出力:
// 50%以下の値:
// 0.08011364416212319
// 50%以下の値:
// 0.3141403962502316
```
`defer()` は副作用のある処理をObservable作成時ではなく購読時に遅延させたい場合に有効です。ランダム生成や現在時刻の取得などの用途に適しています。

> [!IMPORTANT]
> `of()`との違い  
> - `of()` は作成時点で値が確定します。  
> - `defer()` は購読時に初めて処理されるため、購読するたびに値が変わるような処理に適しています。

### range()
[📘 RxJS公式: range](https://rxjs.dev/api/index/function/range)

指定された範囲内の一連の数値を発行するObservableを作成します。

```ts
import { range } from 'rxjs';

const range$ = range(5, 3); // 5から3つ → 5, 6, 7
range$.subscribe({
  next: val => console.log('range:', val),
  complete: () => console.log('完了')
});

// 出力:
// range: 5
// range: 6
// range: 7
// 完了
```

### generate()
[📘 RxJS公式: generate](https://rxjs.dev/api/index/function/generate)

初期値・条件・更新式を指定して、**ループのように数値や状態を生成する** ための関数です。

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

// 出力:
// generate: 0
// generate: 1
// generate: 2
// generate: 3
// generate: 4
// 完了
```

### iif()
[📘 RxJS公式: iif](https://rxjs.dev/api/index/function/iif)

条件に応じて、**実行するObservableを動的に切り替える** ための関数です。

```ts
import { iif, of, EMPTY } from 'rxjs';

const condition = true;
const iif$ = iif(() => condition, of('条件はtrue'), EMPTY);

iif$.subscribe({
  next: val => console.log('iif:', val),
  complete: () => console.log('完了')
});

// 出力:
// iif: 条件はtrue
// 完了
```

> [!NOTE]
> `iif()` は条件によって返すObservableを動的に切り替えることができます。フロー制御に便利です。

## 特殊Observable

### EMPTY, NEVER, throwError()
[📘 RxJS公式: EMPTY](https://rxjs.dev/api/index/const/EMPTY), [📘 RxJS公式: NEVER](https://rxjs.dev/api/index/const/NEVER), [📘 RxJS公式: throwError](https://rxjs.dev/api/index/function/throwError)


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

// 出力:
// 即座に完了
// エラー: エラー発生
```

> [!IMPORTANT]
> 主に制御・検証・学習用途  
> - `EMPTY`, `NEVER`, `throwError()` は、通常のデータストリームではなく、**フロー制御や例外ハンドリングの検証**、または学習用途で活用されます。


## Subject系

### Subject, BehaviorSubject など {#subject-behaviorsubject}
[📘 RxJS公式: Subject](https://rxjs.dev/api/index/class/Subject), [📘 RxJS公式: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

自ら値を発行できるObservableで、**マルチキャストや状態共有**に向いています。

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

// 出力:
// Observer 1: 1
// Observer 2: 1
// Observer 1: 2
// Observer 2: 2
// Observer 1: 3
// Observer 2: 3
```

> [!IMPORTANT]
> Hot Observableであることに注意  
> - `Subject` は購読者に「同時に」通知されるため、`from()` や `of()` などの Cold Observable とは異なり、**購読タイミングによって値を受け取れないことがあります**。

詳しくは、[「Subjectとは」](../subjects/what-is-subject.md)を参照してください。


## コールバック変換

RxJSには、コールバックベースの非同期関数をObservableに変換するための関数として `bindCallback()` および `bindNodeCallback()` が用意されています。

### bindCallback()
[📘 RxJS公式: bindCallback](https://rxjs.dev/api/index/function/bindCallback)

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

// 出力:
// Hello, RxJS
// 完了
```

### bindNodeCallback()
[📘 RxJS公式: bindNodeCallback](https://rxjs.dev/api/index/function/bindNodeCallback)

```ts
import { bindNodeCallback } from 'rxjs';
import { readFile } from 'fs';

const readFile$ = bindNodeCallback(readFile);
readFile$('./some.txt').subscribe({
  next: data => console.log('内容:', data),
  error: err => console.error('エラー:', err)
});
```

> [!NOTE]
> `bindNodeCallback()` は Node.js の `(err, result)` 型の非同期関数に対応しています。

### bindCallback() と bindNodeCallback() の違い
bindCallback() と bindNodeCallback() の違いは、対象とするコールバック関数の形式です。

|関数|対象となる関数の形式|特徴|
|---|---|---|
|bindCallback()|callback(result)|通常のコールバック（引数1つ）に対応|
|bindNodeCallback()|callback(error, result) |Node.jsスタイルのエラーファースト形式に対応|

#### 具体例: bindCallback() の対象

```ts
function doSomething(input: string, callback: (result: string) => void) {
  callback(`結果: ${input}`);
}
```
→ bindCallback() で変換可能


#### 具体例:  bindNodeCallback() の対象（Node.js流）

```ts
function readFile(path: string, cb: (err: Error | null, data: string) => void) {
  if (path === 'valid.txt') cb(null, 'file content');
  else cb(new Error('not found'), '');
}
```
→ bindNodeCallback() を使えば、エラー発生時に error がObservableとして通知される。


> [!NOTE]
> 使い分け
> - コールバックの第1引数が「エラーかどうか」なら bindNodeCallback()
> - 単純に「値だけ返す」コールバックなら bindCallback()

## リソース制御

### using()
[📘 RxJS公式: using](https://rxjs.dev/api/index/function/using)

`using()` は Observable のライフサイクルに合わせて、リソースの作成と解放を関連付けるための関数です。  
WebSocket、イベントリスナー、外部リソースなどの**手動クリーンアップが必要な処理**と組み合わせると便利です。

```ts
import { using, interval, Subscription } from 'rxjs';

const resource$ = using(
  () => new Subscription(() => console.log('リソース解放')),
  () => interval(1000)
);

const sub = resource$.subscribe(value => console.log('値:', value));

// 数秒後に購読解除
setTimeout(() => sub.unsubscribe(), 3500);

// 出力:
// 値: 0
// 値: 1
// 値: 2
// リソース解放
```

> [!IMPORTANT]
> `using()` はリソースのスコープをObservableの購読と一致させる際に便利です。
> `unsubscribe()` されたタイミングで、明示的なクリーンアップ処理が自動的に呼び出されます。

> [!TIP]
> 詳細な使い方と実践例は [using() の詳細ページ](/guide/creation-functions/control/using) を参照してください。制御系の概要は [制御系 Creation Functions](/guide/creation-functions/control/) を参照してください。

## WebSocket()
[📘 RxJS公式: webSocket](https://rxjs.dev/api/webSocket/webSocket)

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
