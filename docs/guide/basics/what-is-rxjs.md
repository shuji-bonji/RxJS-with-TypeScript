# RxJSとは何か？

## 概要
RxJS（Reactive Extensions for JavaScript）とは、「リアクティブプログラミング」を JavaScript で行うためのライブラリです。

> ### リアクティブプログラミングとは？
> リアクティブプログラミングは、データの変化に応じて自動的に更新されるプログラムの作成方法です。
> イベント駆動型プログラミングの一種であり、特に非同期なデータストリームの扱いに焦点を当てています。 データの流れ（ストリーム）を中心に考え、その流れに対して反応（リアクション）する方式でプログラムを構築します。

つまり、RxJSはイベントや非同期データの流れ（ストリーム）を、関数型スタイルで扱うためのライブラリです。Observableパターンを利用して、非同期データストリームを扱うための強力なツールを提供します。

> Observableとは、イベントや非同期データの流れ（ストリーム）を表現するRxJSの中核的な構成要素です。値が「流れてくる」源であり、購読（subscribe）によって値を受け取ることができます。Observableとは、時間の経過とともに値を発行する「データの流れ（ストリーム）」です。購読（subscribe）することで、その値を受け取ることができます。


> [!TIP]
> 「ストリームってそもそも何？」という方は、[ストリームとは？](/guide/basics/what-is-a-stream) も参照してみてください。


## 簡単な使用例

```ts
import { fromEvent } from 'rxjs';

fromEvent(document, 'click').subscribe(event => {
  console.log('クリックされました:', event);
});
```

## RxJSの基本構成要素

RxJSを使いこなすには、以下の中核的な構成要素を理解することが重要です。

| 構成要素 | 概要 |
|------|------|
| [`Observable`](../observables/what-is-observable.md) | 非同期または時間に沿って発生するデータを表すストリームの源です。 |
| [`Observer`](../observables/observable-lifecycle.md#2-observerオブザーバー)| Observableからデータを購読して受け取る側の存在です。 |
| [`Subscription`](../observables/observable-lifecycle.md#3-subscriptionサブスクリプション) | Observableの購読と解除の管理を行います。 |
| [`Operator`](../operators/index.md) | Observableを変換・合成・制御するための関数群です。 |
| [`Subject`](../subjects/what-is-subject.md)[^1] | ObservableとObserverの両方の性質を持つ中継器です。 |
| [`Scheduler`](../schedulers/)[^2]| Observableの実行タイミングを制御する仕組みです。 |

これらはそれぞれ独立した機能を持ちながらも連携して動作します。  
たとえば、Observableが値を発行し、それをObserverが購読し、Operatorで変換し、Schedulerで制御する、といった形で、全体としてストリーム処理を構成します。

※ 各構成要素の詳細な使い方や例については、それぞれの専用章で個別に解説します。

[^1]: Subjectは、値を発行するObservableであると同時に、値を受け取るObserverとしても振る舞える特殊な存在です。
[^2]: Schedulerは、非同期処理の実行タイミングやコンテキストを制御するために使われ、デバッグやパフォーマンス管理にも役立ちます。

## RxJSの利点

| 利点 | 内容 |
|---|---|
| 宣言的コード[^3] | `map`, `filter` などで「何をしたいか」を記述し、forループなどの手続き的記述を避けられる |
| 非同期処理の単純化 | `Promise` やコールバックのネストを避け、直感的な流れで書ける |
| エラー処理 | `.pipe(catchError(...))` などでストリーム中のエラーを統一的に処理可能 |
| キャンセル可能 | `Subscription.unsubscribe()` によりストリームの中断が可能 |
| 多様なオペレーター | `debounceTime`, `mergeMap`, `combineLatest` など多数の演算子で変換や合成が可能 |

[^3]: > - 宣言的コード: 「どういう結果が欲しいのか」を素直に書くコード  
      > - 手続き的コード: 「どういう計算を行っていけば欲しい結果が手に入るのか」を書くコード


## ユースケース

RxJSは以下のような状況で特に役立ちます。

|ユースケース|内容|
|---|---|
|UIイベント処理|クリック、スクロール、入力などのユーザーインタラクション|
|HTTP要求|サーバーとの通信|
|WebSocketの処理|リアルタイムデータストリーム|
|状態管理|アプリケーションの状態を扱う（NgRx、Redux-Observable、またはカスタム状態管理）|

## まとめ

RxJSは、非同期およびイベントベースのプログラミングに対する強力なアプローチを提供します。Observableを中心としたデータストリームの考え方は、複雑な非同期処理を扱う際に特に役立ちます。
