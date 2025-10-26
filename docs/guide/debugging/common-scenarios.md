---
description: RxJSのよくあるデバッグシナリオを6つ紹介します。値が流れてこない、期待と異なる値が出力される、購読が完了しない、メモリリーク、エラーの見逃し、リトライ追跡など、実践的な問題と解決方法を解説します。
---

# よくあるデバッグシナリオ

RxJS開発で遭遇する典型的な問題とその解決方法を、具体的なコード例とともに解説します。

## シナリオ1: 値が流れてこない

- **症状**: `subscribe` しているのに、値が一つも出力されない


### 原因1: Cold Observable の購読忘れ

```ts
import { interval } from 'rxjs';
import { map } from 'rxjs';

// ❌ 購読していないため、何も実行されない
const numbers$ = interval(1000).pipe(
  map(x => {
    console.log('この行は実行されない');
    return x * 2;
  })
);

// ✅ 購読することで実行される
numbers$.subscribe(value => console.log('値:', value));
```

### 原因2: 完了済みの Subject

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.complete(); // 完了

// ❌ 完了後の購読では値を受け取れない
subject.subscribe(value => console.log('この行は実行されない'));

// ✅ 完了前に購読する
const subject2 = new Subject<number>();
subject2.subscribe(value => console.log('値:', value));
subject2.next(1); // 値: 1
subject2.complete();
```

### 原因3: 誤った条件でのフィルタリング

```ts
import { of } from 'rxjs';
import { filter, tap } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('filter前:', value)),
    filter(x => x > 10), // すべて除外されてしまう
    tap(value => console.log('filter後:', value)) // この行は実行されない
  )
  .subscribe({
    next: value => console.log('最終値:', value),
    complete: () => console.log('完了（値なし）')
  });

// 出力:
// filter前: 1
// filter前: 2
// filter前: 3
// filter前: 4
// filter前: 5
// 完了（値なし）
```

### デバッグ手法
```ts
import { of, EMPTY } from 'rxjs';
import { filter, tap, defaultIfEmpty } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('🔵 入力:', value)),
    filter(x => x > 10),
    tap(value => console.log('🟢 filter通過:', value)),
    defaultIfEmpty('値なし') // 値がない場合のデフォルト
  )
  .subscribe(value => console.log('✅ 出力:', value));

// 出力:
// 🔵 入力: 1
// 🔵 入力: 2
// 🔵 入力: 3
// 🔵 入力: 4
// 🔵 入力: 5
// ✅ 出力: 値なし
```

## シナリオ2: 期待と異なる値が出力される

- **症状**: 想定していた値とは異なる値が出力される

### 原因1: オペレーターの順序が誤っている

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// ❌ 期待と異なる結果
of(1, 2, 3, 4, 5)
  .pipe(
    map(x => x * 2),     // 2, 4, 6, 8, 10
    filter(x => x < 5)   // 2, 4 のみ通過
  )
  .subscribe(value => console.log('結果:', value));
// 出力: 2, 4

// ✅ 正しい順序
of(1, 2, 3, 4, 5)
  .pipe(
    filter(x => x < 5),  // 1, 2, 3, 4 のみ通過
    map(x => x * 2)      // 2, 4, 6, 8
  )
  .subscribe(value => console.log('結果:', value));
// 出力: 2, 4, 6, 8
```

### 原因2: 参照の共有による意図しない変更

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const user: User = { id: 1, name: 'Alice' };

of(user)
  .pipe(
    // ❌ 元のオブジェクトを直接変更してしまう
    map(u => {
      u.name = 'Bob'; // 元のオブジェクトが変更される
      return u;
    })
  )
  .subscribe(value => console.log('変更後:', value));

console.log('元のオブジェクト:', user); // { id: 1, name: 'Bob' }

// ✅ 新しいオブジェクトを作成する
of(user)
  .pipe(
    map(u => ({ ...u, name: 'Charlie' })) // スプレッド構文で新しいオブジェクト
  )
  .subscribe(value => console.log('変更後:', value));

console.log('元のオブジェクト:', user); // { id: 1, name: 'Alice' }（変更されない）
```

### 原因3: 非同期処理のタイミング

```ts
import { of, delay } from 'rxjs';
import { mergeMap, tap } from 'rxjs';

// ❌ 非同期処理の完了を待たない
of(1, 2, 3)
  .pipe(
    tap(value => console.log('開始:', value)),
    mergeMap(value =>
      of(value * 2).pipe(
        delay(100 - value * 10) // 値が大きいほど早く完了
      )
    )
  )
  .subscribe(value => console.log('完了:', value));

// 出力:
// 開始: 1
// 開始: 2
// 開始: 3
// 完了: 3  ← 遅延が最も短い
// 完了: 2
// 完了: 1  ← 遅延が最も長い

// ✅ 順序を保証する
import { concatMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(value => console.log('開始:', value)),
    concatMap(value =>  // mergeMap → concatMap
      of(value * 2).pipe(delay(100 - value * 10))
    )
  )
  .subscribe(value => console.log('完了:', value));

// 出力:
// 開始: 1
// 完了: 1
// 開始: 2
// 完了: 2
// 開始: 3
// 完了: 3
```

## シナリオ3: 購読が完了しない（無限ストリーム）

- **症状**: `complete` が呼ばれず、ストリームが終了しない

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

// ❌ interval は無限に値を発行し続ける
interval(1000)
  .pipe(
    tap(value => console.log('値:', value))
  )
  .subscribe({
    complete: () => console.log('この行は実行されない')
  });

// ✅ take で明示的に完了させる
import { take } from 'rxjs';

interval(1000)
  .pipe(
    take(5), // 5個の値の後に完了
    tap(value => console.log('値:', value))
  )
  .subscribe({
    complete: () => console.log('完了')
  });
```

### デバッグ手法
```ts
import { interval, timer } from 'rxjs';
import { tap, takeUntil } from 'rxjs';

// タイムアウトを設定してデバッグ
const stop$ = timer(5000); // 5秒後に完了

interval(1000)
  .pipe(
    takeUntil(stop$),
    tap({
      next: value => console.log('値:', value),
      complete: () => console.log('タイムアウトで停止')
    })
  )
  .subscribe();
```

## シナリオ4: メモリリーク（購読解除忘れ）

- **症状**: アプリケーションの動作が徐々に重くなる

### 原因: 不要になった購読を解除していない

```ts
import { interval } from 'rxjs';

class UserComponent {
  private subscription: any;

  ngOnInit() {
    // ❌ 購読解除を忘れている
    interval(1000).subscribe(value => {
      console.log('値:', value); // コンポーネント破棄後も実行され続ける
    });
  }

  ngOnDestroy() {
    // 購読解除が行われない
  }
}

// ✅ 購読を適切に管理
class UserComponentFixed {
  private subscription: any;

  ngOnInit() {
    this.subscription = interval(1000).subscribe(value => {
      console.log('値:', value);
    });
  }

  ngOnDestroy() {
    // コンポーネント破棄時に購読解除
    if (this.subscription) {
      this.subscription.unsubscribe();
    }
  }
}
```

**推奨パターン: `takeUntil` を使用**

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class UserComponentBest {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    // ✅ takeUntil で自動的に購読解除
    interval(1000)
      .pipe(
        takeUntil(this.destroy$)
      )
      .subscribe(value => console.log('値:', value));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### メモリリークの検出

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

let subscriptionCount = 0;

const trackSubscriptions = <T>() =>
  tap<T>({
    subscribe: () => {
      subscriptionCount++;
      console.log('📈 購読数:', subscriptionCount);
    },
    unsubscribe: () => {
      subscriptionCount--;
      console.log('📉 購読数:', subscriptionCount);
    }
  });

// 使用例
const stream$ = interval(1000).pipe(
  trackSubscriptions()
);

const sub1 = stream$.subscribe();
// 出力: 📈 購読数: 1

const sub2 = stream$.subscribe();
// 出力: 📈 購読数: 2

setTimeout(() => {
  sub1.unsubscribe();
  // 出力: 📉 購読数: 1
}, 3000);
```

## シナリオ5: エラーが発生しているのに気づかない

- **症状**: エラーが発生しているが、表示されずに無視されている

```ts
import { of, throwError } from 'rxjs';
import { mergeMap, catchError } from 'rxjs';

// ❌ エラーハンドリングがないため、エラーが握りつぶされる
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('エラー'));
      }
      return of(value);
    })
  )
  .subscribe(); // エラーハンドラーなし

// ✅ 適切なエラーハンドリング
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('エラー'));
      }
      return of(value);
    }),
    catchError(error => {
      console.error('🔴 エラーをキャッチ:', error.message);
      return of(-1); // フォールバック値
    })
  )
  .subscribe({
    next: value => console.log('値:', value),
    error: error => console.error('🔴 購読でエラー:', error)
  });

// 出力:
// 値: 1
// 🔴 エラーをキャッチ: エラー
// 値: -1
```

### グローバルエラーハンドラーの設定

```ts
import { Observable } from 'rxjs';

// すべての未処理エラーをキャッチ
const originalCreate = Observable.create;

Observable.create = function(subscribe: any) {
  return originalCreate.call(this, (observer: any) => {
    try {
      return subscribe(observer);
    } catch (error) {
      console.error('🔴 未処理のエラー:', error);
      observer.error(error);
    }
  });
};
```

## シナリオ6: リトライの試行回数を追跡したい

- **症状**: `retry` オペレーターを使用しているが、何回リトライしているのか分からない

エラー発生時に自動的にリトライする場合、実際に何回リトライが実行されているのかを追跡することで、デバッグやログ記録が容易になります。

### 基本的なリトライデバッグ

```ts
import { throwError, of, timer } from 'rxjs';
import { retryWhen, mergeMap, tap } from 'rxjs';

throwError(() => new Error('一時的なエラー'))
  .pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          console.log(`🔄 リトライ ${retryCount}回目`);

          if (retryCount > 2) {
            console.log('❌ 最大リトライ数に到達');
            throw error;
          }

          return timer(1000);
        })
      )
    )
  )
  .subscribe({
    next: value => console.log('✅ 成功:', value),
    error: error => console.log('🔴 最終エラー:', error.message)
  });

// 出力:
// 🔄 リトライ 1回目
// 🔄 リトライ 2回目
// 🔄 リトライ 3回目
// ❌ 最大リトライ数に到達
// 🔴 最終エラー: 一時的なエラー
```

> [!TIP]
> リトライのデバッグ方法について、より詳細な実装パターンは[retry と catchError](/guide/error-handling/retry-catch#リトライのデバッグ)の「リトライのデバッグ」セクションで解説しています。
> - tap の error コールバックを使った基本的な追跡
> - retryWhen での詳細なログ記録
> - 指数バックオフとログ記録
> - RxJS 7.4+ の retry 設定オブジェクト

## まとめ

よくあるデバッグシナリオの解決方法

- ✅ **値が流れてこない** → 購読忘れ、フィルタリング条件の確認
- ✅ **期待と異なる値** → オペレーターの順序、参照の共有に注意
- ✅ **購読が完了しない** → 無限ストリームに `take` や `takeUntil` を使用
- ✅ **メモリリーク** → `takeUntil` パターンで自動購読解除
- ✅ **エラーの見逃し** → 適切なエラーハンドリングの実装
- ✅ **リトライ追跡** → `retryWhen` や設定オブジェクトでログ記録

## 関連ページ

- [デバッグの基本戦略](/guide/debugging/) - tap オペレーターや開発者ツールの使い方
- [カスタムデバッグツール](/guide/debugging/custom-tools) - 名前付きストリーム、デバッグオペレーター
- [パフォーマンスデバッグ](/guide/debugging/performance) - 購読数監視、メモリ使用量確認
- [エラーハンドリング](/guide/error-handling/strategies) - エラー処理の戦略
