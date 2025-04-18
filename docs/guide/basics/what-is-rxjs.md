# RxJSとは何か？

## 概要
RxJS（Reactive Extensions for JavaScript）とは、「リアクティブプログラミング」を JavaScript で行うためのライブラリです。

> ### リアクティブプログラミングとは？
> リアクティブプログラミングは、データの変化に応じて自動的に更新されるプログラムの作成方法です。
> イベント駆動型プログラミングの一種であり、特に非同期なデータストリームの扱いに焦点を当てています。 データの流れ（ストリーム）を中心に考え、その流れに対して反応（リアクション）する方式でプログラムを構築します。

つまり、RxJSはイベントや非同期データの流れ（ストリーム）を、関数型スタイルで扱うためのライブラリです。Observableパターンを利用して、非同期データストリームを扱うための強力なツールを提供します。

> Observableとは、イベントや非同期データの流れ（ストリーム）を表現するRxJSの中核的な構成要素です。値が「流れてくる」源であり、購読（subscribe）によって値を受け取ることができます。Observableとは、時間の経過とともに値を発行する「データの流れ（ストリーム）」です。購読（subscribe）することで、その値を受け取ることができます。

## 簡単な使用例

```ts
import { fromEvent } from 'rxjs';

fromEvent(document, 'click').subscribe(event => {
  console.log('クリックされました:', event);
});
```

## RxJSの利点

| 利点 | 内容 |
|---|---|
| 宣言的コード | `map`, `filter` などで「何をしたいか」を記述し、forループなどの手続き的記述を避けられる |
| 非同期処理の単純化 | `Promise` やコールバックのネストを避け、直感的な流れで書ける |
| エラー処理 | `.pipe(catchError(...))` などでストリーム中のエラーを統一的に処理可能 |
| キャンセル可能 | `Subscription.unsubscribe()` によりストリームの中断が可能 |
| 多様なオペレーター | `debounceTime`, `mergeMap`, `combineLatest` など多数の演算子で変換や合成が可能 |

> 宣言的コード - 「どういう結果が欲しいのか」を素直に書くコード  
> 手続き的コード - 対比として「どういう計算を行っていけば欲しい結果が手に入るのか」を書くコード


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

## 次に学ぶ

RxJSの基本的な構成要素については、[RxJSの主要概念](./key-concepts.md) をご覧ください。