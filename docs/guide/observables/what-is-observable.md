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
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
});

// 出力:
// 次の値: 1
// 次の値: 2
// 完了
```
 
## 通知の種類
 
Observableは以下の3種類の通知をObserverに送ります。
 
- `next`: 値の通知
- `error`: エラー発生時の通知（以降の通知は行われない）
- `complete`: 正常終了の通知

詳しくは、[「Observableのライフサイクル」の Observer（オブザーバー）](./observable-lifecycle.html#_2-observer-オブザーバー)のセクションで解説します。

## ObservableとPromiseの違い
 
| 特徴 | Observable | Promise |
|---|---|---|
| 複数の値 | ◯ | ×（1つのみ） |
| キャンセル可能 | ◯（`unsubscribe()`） | × |
| 遅延実行 | ◯ | ◯ |
| 同期/非同期 | 両方 | 非同期のみ |

ObservableとPromiseの最大の違いは、「複数の値を扱えるかどうか」と「途中でキャンセルできるかどうか」です。  
Promiseは1回限りの非同期処理に適していますが、Observableはイベントストリームのような「継続的に発生する非同期データ」に強みがあります。

また、Observableは`unsubscribe()`によって購読を途中で解除できるため、メモリリーク防止や不要な通信の停止など、リソース管理の観点でも重要です。

一方で、Promiseは標準APIに広く採用されており、`async/await`と組み合わせた直感的な書き方が可能です。用途に応じて使い分けることが望まれます。

## コールドとホットの区別

RxJSのObservableには「コールド」と「ホット」の2種類があります。

- **コールドObservable**：各サブスクライバーが独自のデータストリームを持ち、購読したときに実行が開始されます。（例：`of()`, `from()`, `ajax()`）
- **ホットObservable**：サブスクライバーが同一のデータストリームを共有し、購読の有無に関わらずデータが流れ続けます。（例：`fromEvent()`, `Subject`）

この区別はデータ共有とリソース効率に大きく影響します。  
詳しくは[「コールドObservableとホットObservable」](./cold-and-hot-observables.md)のセクションで解説します。

## Observableとパイプライン

Observableの真価は`pipe()`メソッドを使ってオペレーターと組み合わせることで発揮されます：

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs/operators';

const numbers$ = of(1, 2, 3, 4, 5);
numbers$.pipe(
  filter(n => n % 2 === 0), // 偶数のみ通過
  map(n => n * 10)          // 10倍に変換
).subscribe(value => console.log(value));
// 出力: 20, 40
```

## Observableのライフサイクル

Observableは以下のライフサイクルを持ちます。

1. **作成** - Observableインスタンスの生成
2. **購読** - `subscribe()`によるデータ受信の開始
3. **実行** - データの発行(`next`)、エラー(`error`)、または完了(`complete`)
4. **解除** - `unsubscribe()`による購読の終了

リソースリークを防ぐためには、不要になったObservableの購読を解除することが重要です。  
詳しくは[「Observableのライフサイクル」](./observable-lifecycle.md)のセクションで解説します。

## Observableの使いどころ
 
- UIイベント（クリック、スクロール、キーボード操作など）
- HTTPリクエスト
- 時間ベースの処理（インターバルやタイマー）
- WebSocketやリアルタイム通信
- アプリケーション状態管理
 
## まとめ
 
Observableは、非同期データの扱いを柔軟かつ統一的に行うための基盤です。ReactiveX（RxJS）の中心的な概念として、複雑な非同期処理やイベントストリームを簡潔に表現できます。