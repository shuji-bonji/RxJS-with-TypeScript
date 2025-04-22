

 # Observableとは
 
 RxJSにおけるObservableとは、「時間の経過とともに発生するデータの流れ（ストリーム）」を表現する中核的な構成要素です。Observerパターンに基づいて設計されており、非同期処理やイベント駆動の処理を統一的に扱うことができます。
 
 ## Observableの役割
 
 Observableは、複数の値を時間の経過とともに発行する「データの生産者」として機能します。これに対して、Observerが「消費者」となり、`subscribe()` によって値を購読します。
 
 ```ts
 import { Observable } from 'rxjs';
 
 const observable$ = new Observable<number>(subscriber => {
   subscriber.next(1);
   subscriber.next(2);
   subscriber.complete();
 });
 
 observable$.subscribe({
   next: value => console.log('次の値:', value),
   complete: () => console.log('完了')
 });
 ```
 
 ## 通知の種類
 
 Observableは以下の3種類の通知をObserverに送ります。
 
 - `next`: 値の通知
 - `error`: エラー発生時の通知（以降の通知は行われない）
 - `complete`: 正常終了の通知
 
 ## ObservableとPromiseの違い
 
 | 特徴 | Observable | Promise |
 |------|------------|---------|
 | 複数の値 | ◯ | ×（1つのみ） |
 | キャンセル可能 | ◯（`unsubscribe()`） | × |
 | 遅延実行 | ◯ | ◯ |
 | 同期/非同期 | 両方 | 非同期のみ |
 
 ## Observableの使いどころ
 
 - UIイベント（クリック、スクロール、キーボード操作など）
 - HTTPリクエスト
 - 時間ベースの処理（インターバルやタイマー）
 - WebSocketやリアルタイム通信
 - アプリケーション状態管理
 
 ## まとめ
 
 Observableは、非同期データの扱いを柔軟かつ統一的に行うための基盤です。次章では、実際にどのようにObservableを作成するかを見ていきましょう。
 