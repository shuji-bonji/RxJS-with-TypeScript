# Observableのライフサイクル

このページでは、RxJSにおけるObservableのライフサイクルについて、作成から購読、データの発行、完了・エラー通知、購読解除、そしてリソース管理に至るまでの一連の流れを段階的に解説します。このページでは、RxJSでのObservableの作成方法について、基本的な構文から実践的な用途までを網羅的に紹介します。このライフサイクルを理解することは、RxJSを効果的に使用するための基礎となります。

## 1. Observableのライフサイクルの概要

この方法はカスタムなObservableロジックを定義したい場合に最も柔軟です。明示的な `next`, `error`, `complete` 呼び出しによって細かな挙動制御が可能です。

### Observableのライフサイクルの構成
Observableのライフサイクルは、以下のフェーズで構成されています。

|順番|フェーズ|内容|
|---|---|---|
|1|作成（Creation）|Observableインスタンスの作成|
|2|サブスクリプション（Subscription）|`subscribe()`メソッドによる購読開始|
|3|実行（Execution）|<li>`next()`: データの発行</li><li>`error()`: エラー通知</li><li>`complete()`: 完了通知</li>|
|4|解除（Disposal）|`unsubscribe()`メソッドによる購読解除|

Observableは「遅延実行（lazy）」であり、`subscribe()` を呼び出すまでストリームは発行されません。また、`complete()` または `error()` が呼ばれるとストリームは終了し、それ以上の `next()` 呼び出しは無視されます。

> [!CAUTION]
> `subscribe()` によって直接的に何かの「処理」が実行されるわけではありません。  
> よく『`subscribe()`を呼び出すまで実行されません。』という表現が使われますが、  
> 正確には「Observableの内部ロジック（通知処理）」は `subscribe()` によって初めてトリガーされます。  
> 
> `pipe()` などのオペレーターは評価チェーンを構築するだけで、実行はObserverのコールバックによって開始されます。

#### Observableのライフサイクルの例
```ts
import { Observable } from 'rxjs';

// 1. Observable作成
const observable$ = new Observable<number>(subscriber => {
  console.log('Observable実行開始');
  
  // 3. 実行：データ発行
  subscriber.next(1);
  subscriber.next(2);
  
  // タイマーのセットアップ
  const timerId = setTimeout(() => {
    subscriber.next(3);
    subscriber.complete(); // 3. 実行：完了通知
    console.log('Observable完了');
  }, 1000);
  
  // クリーンアップ関数を返す（unsubscribe時に呼ばれる）
  return () => {
    console.log('クリーンアップ実行');
    clearTimeout(timerId);
  };
});

// 2. サブスクリプション
const subscription = observable$.subscribe({
  next: value => console.log('次の値:', value),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了通知を受け取りました')
});

// 4. 購読解除（手動または完了時）
setTimeout(() => {
  console.log('手動で購読解除');
  subscription.unsubscribe();
}, 500); // 500msで解除（完了通知の前）

// 処理結果:
// Observable実行開始
// 次の値: 1
// 次の値: 2
// 手動で購読解除
// クリーンアップ実行
```

## 2. Observer（オブザーバー）

より簡潔で用途に特化したObservable作成には、RxJSが提供する「作成操作子（creation operator）」が便利です。繰り返し使われるユースケースにはこれらを使うことでコードが簡素化されます。

### Observerが持つ、3つのコールバック関数
Observerは、Observableから通知を受け取るためのインターフェースです。  
3つのコールバック関数を持ちます。
- `next`: データの発行
- `error`: エラー通知
- `complete`: 完了通知

#### 例
```ts
import { Observer } from 'rxjs';

// 完全なObserverオブジェクト
const observer: Observer<number> = {
  next: value => console.log('値:', value),// データの発行
  error: err => console.error('エラー:', err), // エラー通知
  complete: () => console.log('完了') // 完了通知
};

const observable$ = of(1, 2, 3); // 簡易的にObervableを作成

// 部分的なObserverも可能
observable$.subscribe({
  next: value => console.log('値のみ処理:', value)
});

// 簡略記法
observable$.subscribe(
  value => console.log('値:', value),
  err => console.error('エラー:', err),
  () => console.log('完了')
);

// 処理結果:
// 値のみ処理: 1
// 値のみ処理: 2
// 値のみ処理: 3
// 値: 1
// 値: 2
// 値: 3
// 完了
```

## 3. Subscription（サブスクリプション）

サブスクリプションはObservableの実行を表し、主に購読解除`unsubscribe()`のために使用されます。

#### 例
```ts
import { interval } from 'rxjs';
import { take } from 'rxjs/operators';

const numbers$ = interval(1000).pipe(take(5));

// サブスクリプションを保持
const subscription = numbers$.subscribe({
  next: value => console.log('値:', value), 
  complete: () => console.log('完了')
});

// 3秒後に手動で購読解除
setTimeout(() => {
  subscription.unsubscribe(); // 購読解除
  console.log('購読解除済み');
}, 3000);

// 処理結果:
// 値: 0
// 値: 1
// 値: 2
// 購読解除済み
```


## 4. エラー処理

Observableのライフサイクルでは、 `error()` を呼び出すと直ちにストリームが終了し、`complete()` は呼び出されません。このため、`catchError` の使用や `retry` の設計は重要です。

#### 例
```ts
import { Observable, throwError, of } from 'rxjs';
import { catchError, retry } from 'rxjs/operators';

// エラーを発生させるObservable
const failingObservable$ = new Observable<number>(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.error(new Error('意図的なエラー'));
  // エラー後はcompleteが呼ばれないことに注意
});

// エラー処理の例
failingObservable$.pipe(
  // エラー発生時、3回リトライ
  retry(3),
  // それでもエラーになったら、代替Observableに切り替え
  catchError(error => {
    console.error('エラーをキャッチ:', error.message);
    return of('エラー後の代替値');
  })
).subscribe({
  next: value => console.log('値:', value),
  error: err => console.error('ハンドリングされなかったエラー:', err),
  complete: () => console.log('完了')
});
```

#### 実行結果
```ts
値: 1
値: 2
値: 1
値: 2
値: 1
値: 2
値: 1
値: 2
エラーをキャッチ: 意図的なエラー
値: エラー後の代替値
完了
```

## 5. 完了のライフサイクル

Observableの完了は、明示的に`complete()`が呼ばれるか、有限のストリームが終了した場合に発生します。

#### 例
```ts
import { of, interval, Observable } from 'rxjs';
import { take } from 'rxjs/operators';

// 有限のObservable（自動的に完了）
const finite$ = of(1, 2, 3);
finite$.subscribe({
  next: value => console.log('有限値:', value),
  complete: () => console.log('有限Observable完了')
});

// 無限のObservableを有限に変換
const limited$ = interval(1000).pipe(take(3));
limited$.subscribe({
  next: value => console.log('制限付き値:', value),
  complete: () => console.log('制限付きObservable完了')
});

// 手動で完了させるObservable
const manual$ = new Observable<number>(subscriber => {
  subscriber.next(1);
  
  setTimeout(() => {
    subscriber.next(2);
    subscriber.complete(); // 明示的に完了
  }, 2000);
});

manual$.subscribe({
  next: value => console.log('手動値:', value),
  complete: () => console.log('手動Observable完了')
});
```

#### 実行結果
```ts
有限値: 1
有限値: 2
有限値: 3
有限Observable完了
手動値: 1
制限付き値: 0
制限付き値: 1
手動値: 2
手動Observable完了
制限付き値: 2
制限付きObservable完了
```

## 7. リソース管理とメモリリーク防止

適切なタイミングでのサブスクリプション解除は、メモリリークを防ぐために重要です。

#### 例
```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs/operators';

// コンポーネントのライフサイクルを模倣
class Component {
  private destroy$ = new Subject<void>();
  
  constructor() {
    // 1秒ごとのインターバル（潜在的なメモリリークの原因）
    interval(1000).pipe(
      // コンポーネント破棄時に自動的に購読解除
      takeUntil(this.destroy$)
    ).subscribe(value => {
      console.log('コンポーネント内の値:', value);
    });
  }
  
  // コンポーネントの破棄
  ngOnDestroy() {
    console.log('コンポーネント破棄');
    this.destroy$.next();
    this.destroy$.complete();
  }
}

// 使用例
const component = new Component();

// 5秒後にコンポーネントを破棄
setTimeout(() => {
  (component as any).ngOnDestroy();
}, 5000);
```

#### 実行結果
```
コンポーネント内の値: 0
コンポーネント内の値: 1
コンポーネント内の値: 2
コンポーネント内の値: 3
コンポーネント内の値: 4
コンポーネント破棄
```

## 8. まとめ

Observableのライフサイクルを理解することで、以下のことが可能になります。

- 適切なタイミングでのリソース解放
- エラー処理と回復戦略の実装
- コールドObservableとホットObservableの使い分け
- 副作用の管理

特に、AngularやReactなどのコンポーネントベースのフレームワークでは、`takeUntil`, `unsubscribe`, `finalize` などを活用して、ライフサイクルに沿った購読管理が必要です。