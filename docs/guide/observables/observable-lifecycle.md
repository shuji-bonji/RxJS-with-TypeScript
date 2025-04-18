# Observableのライフサイクル

Observableのライフサイクルは、作成からサブスクリプションの終了までの一連のプロセスです。このライフサイクルを理解することは、RxJSを効果的に使用するための基礎となります。

## 1. Observableのライフサイクルの概要

Observableのライフサイクルは、以下のフェーズで構成されています：

1. **作成（Creation）**: Observableインスタンスの作成
2. **サブスクリプション（Subscription）**: `subscribe()`メソッドによる購読開始
3. **実行（Execution）**: データの発行（`next()`）、エラー通知（`error()`）、完了通知（`complete()`）
4. **解除（Disposal）**: `unsubscribe()`メソッドによる購読解除

```typescript
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
```

## 2. Observer（オブザーバー）

Observerは、Observableから通知を受け取るためのインターフェースです。3つのコールバック関数を持ちます：

```typescript
import { Observer } from 'rxjs';

// 完全なObserverオブジェクト
const observer: Observer<number> = {
  next: value => console.log('値:', value),
  error: err => console.error('エラー:', err),
  complete: () => console.log('完了')
};

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
```

## 3. Subscription（サブスクリプション）

サブスクリプションはObservableの実行を表し、主に購読解除のために使用されます。

```typescript
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
  subscription.unsubscribe();
  console.log('購読解除済み');
}, 3000);
```

## 4. コールドObservableとホットObservable

Observableには「コールド」と「ホット」の2種類があります。

### コールドObservable

- 各サブスクライバーにデータ全体を個別に提供
- 購読時に実行を開始（遅延実行）
- 例：HTTP請求、ファイル読み込み

```typescript
import { Observable } from 'rxjs';

// コールドObservableの例
const cold$ = new Observable<number>(subscriber => {
  console.log('コールドObservable: 新しい実行開始');
  subscriber.next(Math.random());
  subscriber.next(Math.random());
  subscriber.complete();
});

// 各購読は独立した実行となる
cold$.subscribe(value => console.log('購読1:', value));
cold$.subscribe(value => console.log('購読2:', value));
```

### ホットObservable

- すべてのサブスクライバーに同じデータを共有
- 購読の有無に関わらず実行される可能性がある
- 例：マウスクリック、WebSocket

```typescript
import { Subject } from 'rxjs';

// ホットObservableの例（Subject）
const hot$ = new Subject<number>();

// 購読
hot$.subscribe(value => console.log('購読1:', value));

// データ発行
hot$.next(1);
hot$.next(2);

// 2番目の購読（後からの購読）
hot$.subscribe(value => console.log('購読2:', value));

// 更にデータ発行
hot$.next(3);
hot$.complete();
```

## 5. エラー処理

Observableのライフサイクルでは、エラー処理も重要です：

```typescript
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

## 6. 完了のライフサイクル

Observableの完了は、明示的に`complete()`が呼ばれるか、有限のストリームが終了した場合に発生します。

```typescript
import { of, interval } from 'rxjs';
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

## 7. リソース管理とメモリリーク防止

適切なタイミングでのサブスクリプション解除は、メモリリークを防ぐために重要です。

```typescript
import { interval } from 'rxjs';
import { takeUntil } from 'rxjs/operators';
import { Subject } from 'rxjs';

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

## 8. まとめ

Observableのライフサイクルを理解することで、以下のことが可能になります：

- 適切なタイミングでのリソース解放
- エラー処理と回復戦略の実装
- コールドObservableとホットObservableの使い分け
- 副作用の管理

Observableのライフサイクルを適切に管理することで、メモリリークを防ぎ、安定したアプリケーションを構築できます。