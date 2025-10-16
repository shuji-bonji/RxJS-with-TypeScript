---
description: RxJSのデバッグ手法について、基本戦略、よくあるデバッグシナリオ、デバッグツール、パフォーマンスデバッグの観点から実践的に解説します。
---

# RxJSのデバッグ手法

RxJSのデバッグは、非同期ストリームの性質上、従来の同期的なデバッグ手法とは異なるアプローチが必要です。

このページでは、RxJSアプリケーションをデバッグするための実践的な手法とツールについて解説します。

## デバッグの基本戦略

### 1. `tap` オペレーターでのログ出力

`tap` オペレーターは、ストリームの値に副作用を与えずに観察できる、最も基本的なデバッグ手法です。

```ts
import { interval } from 'rxjs';
import { map, filter, tap } from 'rxjs';

interval(1000)
  .pipe(
    tap(value => console.log('🔵 元の値:', value)),
    map(x => x * 2),
    tap(value => console.log('🟢 map後:', value)),
    filter(x => x > 5),
    tap(value => console.log('🟡 filter後:', value))
  )
  .subscribe(value => console.log('✅ 最終値:', value));

// 出力:
// 🔵 元の値: 0
// 🟢 map後: 0
// 🔵 元の値: 1
// 🟢 map後: 2
// 🔵 元の値: 2
// 🟢 map後: 4
// 🔵 元の値: 3
// 🟢 map後: 6
// 🟡 filter後: 6
// ✅ 最終値: 6
```

#### ポイント
- パイプラインの各ステップに `tap` を挿入することで、データの流れを追跡できる
- 絵文字やラベルを使うことで、ログの視認性を向上させる
- `tap` は値を変更しないため、安全にデバッグログを挿入できる

### 2. 詳細なログ情報の出力

より詳細なデバッグ情報を取得するには、Observer オブジェクトを使用します。

```ts
import { of, throwError, concat } from 'rxjs';
import { tap } from 'rxjs';

const debug = (tag: string) =>
  tap({
    next: value => console.log(`[${tag}] next:`, value),
    error: error => console.error(`[${tag}] error:`, error),
    complete: () => console.log(`[${tag}] complete`)
  });

// 正常なストリーム
of(1, 2, 3)
  .pipe(debug('正常'))
  .subscribe();

// 出力:
// [正常] next: 1
// [正常] next: 2
// [正常] next: 3
// [正常] complete

// エラーを含むストリーム
concat(
  of(1, 2),
  throwError(() => new Error('エラー発生'))
)
  .pipe(debug('エラー'))
  .subscribe({
    error: () => {} // エラーハンドリング
  });

// 出力:
// [エラー] next: 1
// [エラー] next: 2
// [エラー] error: Error: エラー発生
```

### 3. 開発者ツールでの確認

ブラウザの開発者ツールを活用したデバッグ手法です。

```ts
import { fromEvent, timer } from 'rxjs';
import { map, tap, debounceTime } from 'rxjs';

// デバッグ用のヘルパー関数
function tapDebugger<T>(label: string) {
  return tap<T>({
    next: value => {
      console.group(`🔍 ${label}`);
      console.log('Value:', value);
      console.log('Type:', typeof value);
      console.log('Timestamp:', new Date().toISOString());
      console.trace('Stack trace');
      console.groupEnd();
    }
  });
}

// ボタンクリックイベントのデバッグ
const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tapDebugger('Click Event'),
      debounceTime(300),
      tapDebugger('After Debounce'),
      map(() => ({ timestamp: Date.now() }))
    )
    .subscribe(data => console.log('📤 送信:', data));
}
```

#### 開発者ツールの活用
- `console.group()` でログをグループ化
- `console.trace()` でスタックトレースを表示
- `console.table()` で配列やオブジェクトを見やすく表示
- ブレークポイントを `tap` 内に設置

### 4. RxJS DevTools の活用

RxJS DevTools は、ブラウザ拡張機能として提供されるデバッグツールです。

#### インストール
- Chrome: [RxJS DevTools - Chrome Web Store](https://chrome.google.com/webstore)
- Firefox: [RxJS DevTools - Firefox Add-ons](https://addons.mozilla.org/)

#### 主な機能
- Observable の購読状態の可視化
- ストリームの値のタイムライン表示
- メモリリークの検出
- パフォーマンス分析

#### 使用例

```ts
import { interval } from 'rxjs';
import { take, map } from 'rxjs';

// 開発環境でのみデバッグを有効化
// ビルドツールによって異なる環境変数の判定方法
const isDevelopment =
  // Vite: import.meta.env.DEV
  // webpack: process.env.NODE_ENV === 'development'
  // 手動設定: グローバル変数を使用
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

const stream$ = interval(1000).pipe(
  take(5),
  map(x => x * 2)
);

if (isDevelopment) {
  // DevTools で観察可能にする
  stream$.subscribe({
    next: value => console.log('DevTools:', value)
  });
}
```

## よくあるデバッグシナリオ

### シナリオ1: 値が流れてこない

- **症状**: `subscribe` しているのに、値が一つも出力されない


#### 原因1: Cold Observable の購読忘れ

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

#### 原因2: 完了済みの Subject

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

#### 原因3: 誤った条件でのフィルタリング

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

#### デバッグ手法
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

### シナリオ2: 期待と異なる値が出力される

- **症状**: 想定していた値とは異なる値が出力される

#### 原因1: オペレーターの順序が誤っている

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

#### 原因2: 参照の共有による意図しない変更

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

#### 原因3: 非同期処理のタイミング

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

### シナリオ3: 購読が完了しない（無限ストリーム）

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

#### デバッグ手法
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

### シナリオ4: メモリリーク（購読解除忘れ）

- **症状**: アプリケーションの動作が徐々に重くなる

#### 原因: 不要になった購読を解除していない

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

#### メモリリークの検出

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

### シナリオ5: エラーが発生しているのに気づかない

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

#### グローバルエラーハンドラーの設定

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

## デバッグツール

### 1. 名前付きストリームのデバッグ

Observable に名前を付けて追跡できるカスタムオペレーターを作成します。

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// ストリーム名を管理するマップ
const namedStreams = new Map<string, any[]>();

/**
 * Observable に名前を付けて追跡する
 */
function tagStream<T>(name: string) {
  if (!namedStreams.has(name)) {
    namedStreams.set(name, []);
  }

  return tap<T>({
    next: value => {
      const log = {
        name,
        type: 'next',
        value,
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.log(`[${name}] next:`, value);
    },
    error: error => {
      const log = {
        name,
        type: 'error',
        error,
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.error(`[${name}] error:`, error);
    },
    complete: () => {
      const log = {
        name,
        type: 'complete',
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.log(`[${name}] complete`);
    }
  });
}

/**
 * 特定の名前付きストリームのログを取得
 */
function getStreamLogs(name: string) {
  return namedStreams.get(name) || [];
}

/**
 * すべての名前付きストリームの一覧を取得
 */
function getAllStreamNames() {
  return Array.from(namedStreams.keys());
}

/**
 * ログをクリア
 */
function clearStreamLogs(name?: string) {
  if (name) {
    namedStreams.set(name, []);
  } else {
    namedStreams.clear();
  }
}
```

#### 使用例

```ts
import { interval } from 'rxjs';
import { map, take } from 'rxjs';

// Observable に名前を付ける
interval(1000)
  .pipe(
    tagStream('interval-stream'),
    map(x => x * 2),
    take(5)
  )
  .subscribe();

// 3秒後にログを確認
setTimeout(() => {
  console.log('すべてのストリーム:', getAllStreamNames());
  console.log('interval-stream のログ:', getStreamLogs('interval-stream'));
}, 3000);

// 出力:
// [interval-stream] next: 0
// [interval-stream] next: 1
// [interval-stream] next: 2
// すべてのストリーム: ['interval-stream']
// interval-stream のログ: [
//   { name: 'interval-stream', type: 'next', value: 0, timestamp: 1697280000000 },
//   { name: 'interval-stream', type: 'next', value: 1, timestamp: 1697280001000 },
//   { name: 'interval-stream', type: 'next', value: 2, timestamp: 1697280002000 }
// ]
```

#### 複数のストリームを追跡

```ts
import { interval, fromEvent } from 'rxjs';
import { map, take } from 'rxjs';

// 複数のストリームに名前を付ける
interval(1000)
  .pipe(
    tagStream('timer-stream'),
    map(x => x * 2),
    take(3)
  )
  .subscribe();

const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tagStream('click-stream'),
      take(5)
    )
    .subscribe();
}

// すべてのストリームを確認
console.log('追跡中のストリーム:', getAllStreamNames());
// 出力: ['timer-stream', 'click-stream']
```

> [!NOTE]
> **rxjs-spy について**
>
> `rxjs-spy` は Observable のデバッグに便利なライブラリでしたが、現在はメンテナンスされておらず、最新の RxJS との互換性に問題があります。
>
> 代わりに、上記のようなカスタムデバッグオペレーターを使用することを推奨します。より柔軟で、プロジェクトの要件に合わせてカスタマイズできます。

### 2. カスタムデバッグオペレーターの作成

独自のデバッグ用オペレーターを作成することで、より柔軟なデバッグが可能になります。

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

interface DebugOptions {
  enabled?: boolean;
  label?: string;
  logValues?: boolean;
  logErrors?: boolean;
  logComplete?: boolean;
  logTimestamp?: boolean;
}

/**
 * カスタムデバッグオペレーター
 */
function debug<T>(options: DebugOptions = {}) {
  const {
    enabled = true,
    label = 'Debug',
    logValues = true,
    logErrors = true,
    logComplete = true,
    logTimestamp = false
  } = options;

  if (!enabled) {
    return (source: Observable<T>) => source;
  }

  return tap<T>({
    next: value => {
      if (logValues) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.log(`${timestamp} [${label}] next:`, value);
      }
    },
    error: error => {
      if (logErrors) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.error(`${timestamp} [${label}] error:`, error);
      }
    },
    complete: () => {
      if (logComplete) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.log(`${timestamp} [${label}] complete`);
      }
    }
  });
}

// 使用例
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    debug({ label: 'Input', logTimestamp: true }),
    map(x => x * 2),
    debug({ label: 'After Map', logTimestamp: true })
  )
  .subscribe();

// 出力:
// [2025-10-14T12:00:00.000Z] [Input] next: 1
// [2025-10-14T12:00:00.001Z] [After Map] next: 2
// [2025-10-14T12:00:00.001Z] [Input] next: 2
// [2025-10-14T12:00:00.002Z] [After Map] next: 4
// [2025-10-14T12:00:00.002Z] [Input] next: 3
// [2025-10-14T12:00:00.003Z] [After Map] next: 6
// [2025-10-14T12:00:00.003Z] [Input] complete
// [2025-10-14T12:00:00.004Z] [After Map] complete
```

#### パフォーマンス計測用のデバッグオペレーター

```ts
import { tap } from 'rxjs';

function measure<T>(label: string) {
  let startTime: number;
  let count = 0;

  return tap<T>({
    subscribe: () => {
      startTime = performance.now();
      console.log(`⏱️ [${label}] 開始`);
    },
    next: value => {
      count++;
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] 値 #${count} (${elapsed.toFixed(2)}ms):`, value);
    },
    complete: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] 完了 (合計: ${elapsed.toFixed(2)}ms, ${count}個の値)`);
    },
    error: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] エラー (${elapsed.toFixed(2)}ms)`);
    }
  });
}

// 使用例
import { interval } from 'rxjs';
import { take, delay } from 'rxjs';

interval(100)
  .pipe(
    take(5),
    measure('Interval Stream'),
    delay(50)
  )
  .subscribe();

// 出力:
// ⏱️ [Interval Stream] 開始
// ⏱️ [Interval Stream] 値 #1 (150.23ms): 0
// ⏱️ [Interval Stream] 値 #2 (250.45ms): 1
// ⏱️ [Interval Stream] 値 #3 (350.67ms): 2
// ⏱️ [Interval Stream] 値 #4 (450.89ms): 3
// ⏱️ [Interval Stream] 値 #5 (551.12ms): 4
// ⏱️ [Interval Stream] 完了 (合計: 551.12ms, 5個の値)
```

## パフォーマンスデバッグ

### 1. 購読数の確認

複数の購読が意図せず作成されていないか確認します。

```ts
import { Observable, defer } from 'rxjs';
import { finalize } from 'rxjs';

let globalSubscriptionId = 0;
let activeSubscriptions = 0;

/**
 * 購読数を追跡するカスタムオペレーター
 */
function tracked<T>(label: string) {
  return (source: Observable<T>) =>
    defer(() => {
      const id = ++globalSubscriptionId;
      activeSubscriptions++;
      console.log(`➕ 購読開始 [${label}] #${id} (アクティブ: ${activeSubscriptions})`);

      return source.pipe(
        finalize(() => {
          activeSubscriptions--;
          console.log(`➖ 購読終了 [${label}] #${id} (アクティブ: ${activeSubscriptions})`);
        })
      );
    });
}

// 使用例
import { interval } from 'rxjs';
import { take } from 'rxjs';

const stream$ = interval(1000).pipe(
  take(3),
  tracked('Test Stream')
);

const sub1 = stream$.subscribe();
const sub2 = stream$.subscribe();

setTimeout(() => {
  sub1.unsubscribe();
  sub2.unsubscribe();
}, 5000);

// 出力:
// ➕ 購読開始 [Test Stream] #1 (アクティブ: 1)
// ➕ 購読開始 [Test Stream] #2 (アクティブ: 2)
// ➖ 購読終了 [Test Stream] #1 (アクティブ: 1)
// ➖ 購読終了 [Test Stream] #2 (アクティブ: 0)
```

この実装では、
- ✅ `defer` で購読時に毎回新しい ID を生成
- ✅ `finalize` で購読解除時の処理を確実に実行
- ✅ アクティブな購読数をリアルタイムで追跡
- ✅ 型安全で RxJS v8 でも動作

### 2. 不要な再評価の検出

同じ値が複数回計算されていないか確認します。

```ts
import { of } from 'rxjs';
import { map, tap, shareReplay } from 'rxjs';

let computeCount = 0;

function expensiveComputation(value: number): number {
  computeCount++;
  console.log(`💰 計算実行 (${computeCount}回目):`, value);
  // 重い計算をシミュレート
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result += Math.sin(i);
  }
  return result;
}

// ❌ shareReplay なし → 購読ごとに計算される
console.log('=== shareReplay なし ===');
computeCount = 0;
const withoutShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x))
);

withoutShare$.subscribe(v => console.log('購読1:', v));
withoutShare$.subscribe(v => console.log('購読2:', v));
// 出力: 計算が6回実行される (3値 × 2購読)

// ✅ shareReplay あり → 計算結果が共有される
console.log('\n=== shareReplay あり ===');
computeCount = 0;
const withShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x)),
  shareReplay(3)
);

withShare$.subscribe(v => console.log('購読1:', v));
withShare$.subscribe(v => console.log('購読2:', v));
// 出力: 計算が3回のみ実行される
```

### 3. メモリ使用量の監視

メモリリークを検出するための監視方法です。

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MemoryMonitor {
  private intervals: ReturnType<typeof setInterval>[] = [];

  start(intervalMs: number = 5000) {
    const id = setInterval(() => {
      if (typeof performance !== 'undefined' && (performance as any).memory) {
        const memory = (performance as any).memory;
        console.log('📊 メモリ使用量:', {
          使用中: `${(memory.usedJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          合計: `${(memory.totalJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          上限: `${(memory.jsHeapSizeLimit / 1024 / 1024).toFixed(2)} MB`
        });
      }
    }, intervalMs);

    this.intervals.push(id);
  }

  stop() {
    this.intervals.forEach(id => clearInterval(id));
    this.intervals = [];
  }
}

// 使用例
const monitor = new MemoryMonitor();
monitor.start(5000); // 5秒ごとにメモリ使用量を表示

// メモリリークのテスト
const leakyStreams: any[] = [];

for (let i = 0; i < 100; i++) {
  // ❌ 購読解除されないストリーム
  const sub = interval(100).subscribe();
  leakyStreams.push(sub);
}

// 10秒後に購読解除
setTimeout(() => {
  console.log('購読解除開始');
  leakyStreams.forEach(sub => sub.unsubscribe());
  console.log('購読解除完了');

  // さらに10秒後に監視を停止
  setTimeout(() => {
    monitor.stop();
  }, 10000);
}, 10000);
```

## ベストプラクティス

### 1. デバッグ環境の構築

開発環境でのみデバッグログを有効化する方法です。

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// デバッグモードの判定（ビルドツールに応じて調整）
const IS_DEVELOPMENT =
  // Vite使用時: import.meta.env.DEV
  // webpack使用時: process.env.NODE_ENV === 'development'
  // 手動設定: グローバル変数を定義
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

function devLog<T>(label: string) {
  if (!IS_DEVELOPMENT) {
    return (source: Observable<T>) => source;
  }

  return tap<T>({
    next: value => console.log(`[${label}]`, value),
    error: error => console.error(`[${label}] Error:`, error),
    complete: () => console.log(`[${label}] Complete`)
  });
}

// 使用例
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    devLog('Input'),
    map(x => x * 2),
    devLog('Output')
  )
  .subscribe();
// 本番環境ではログが出力されない
```

### 2. 型安全なデバッグ

TypeScript の型システムを活用したデバッグ方法です。

```ts
import { tap } from 'rxjs';

type LogLevel = 'debug' | 'info' | 'warn' | 'error';

interface TypedDebugOptions<T> {
  label: string;
  level?: LogLevel;
  transform?: (value: T) => any;
  filter?: (value: T) => boolean;
}

function typedDebug<T>(options: TypedDebugOptions<T>) {
  const { label, level = 'debug', transform, filter } = options;

  const logFn = console[level] || console.log;

  return tap<T>({
    next: value => {
      if (filter && !filter(value)) return;

      const displayValue = transform ? transform(value) : value;
      logFn(`[${label}]`, displayValue);
    }
  });
}

// 使用例
interface User {
  id: number;
  name: string;
  email: string;
}

import { of } from 'rxjs';

of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob', email: 'bob@example.com' },
  { id: 3, name: 'Charlie', email: 'charlie@example.com' }
)
  .pipe(
    typedDebug<User>({
      label: 'User Stream',
      level: 'info',
      transform: user => `${user.name} (${user.email})`,
      filter: user => user.id > 1
    })
  )
  .subscribe();

// 出力:
// [User Stream] Bob (bob@example.com)
// [User Stream] Charlie (charlie@example.com)
```

### 3. エラー境界の設定

エラーを適切に分離してデバッグしやすくします。

```ts
import { Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

function errorBoundary<T>(label: string) {
  return (source: Observable<T>) =>
    source.pipe(
      catchError(error => {
        console.error(`🔴 [${label}] エラーをキャッチ:`, {
          message: error.message,
          stack: error.stack,
          timestamp: new Date().toISOString()
        });

        // エラーを再スローするか、フォールバック値を返す
        throw error;
      })
    );
}

// 使用例
import { throwError } from 'rxjs';
import { mergeMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    errorBoundary('メイン処理'),
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('値2でエラー'));
      }
      return of(value);
    }),
    errorBoundary('非同期処理')
  )
  .subscribe({
    next: value => console.log('成功:', value),
    error: error => console.log('最終エラー:', error.message)
  });
```

## まとめ

RxJSのデバッグは、以下のポイントを押さえることで効率的に行えます。

### 基本戦略
- ✅ `tap` オペレーターでストリームの各段階を観察
- ✅ 開発者ツールを活用した詳細なログ出力
- ✅ RxJS DevTools でストリームを可視化

### よくあるシナリオ
- ✅ 値が流れてこない → 購読忘れ、フィルタリング条件の確認
- ✅ 期待と異なる値 → オペレーターの順序、参照の共有に注意
- ✅ 購読が完了しない → 無限ストリームに `take` や `takeUntil` を使用
- ✅ メモリリーク → `takeUntil` パターンで自動購読解除
- ✅ エラーの見逃し → 適切なエラーハンドリングの実装

### デバッグツール
- ✅ rxjs-spy で Observable に名前を付けて追跡
- ✅ カスタムデバッグオペレーターで柔軟なデバッグ
- ✅ パフォーマンス計測でボトルネックを特定

### パフォーマンス
- ✅ 購読数の監視でメモリリークを防止
- ✅ 不要な再計算を `shareReplay` で回避
- ✅ メモリ使用量を定期的に確認

これらの手法を組み合わせることで、RxJSアプリケーションのデバッグを効率的に行うことができます。

## 関連ページ

- [エラーハンドリング](/guide/error-handling/strategies) - エラー処理の戦略
- [テスト手法](/guide/testing/unit-tests) - RxJSのテスト方法
- [RxJSアンチパターン集](/guide/anti-patterns/) - よくある間違いと対処法
- [パイプライン](/guide/operators/pipeline) - オペレーターの連鎖
