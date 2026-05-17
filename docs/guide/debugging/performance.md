---
description: "RxJSアプリケーションのパフォーマンスデバッグ手法を解説。購読数の追跡、不要な再評価の検出、メモリ使用量の監視、開発環境でのデバッグ設定、型安全なパフォーマンス計測オペレーター作成など、本番環境で問題を起こさないための実践的なテクニックを紹介します。"
---

# パフォーマンスデバッグ

RxJSアプリケーションのパフォーマンスを最適化し、効率的なデバッグ環境を構築するための手法を解説します。

## 購読数の確認

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

## 不要な再評価の検出

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
  shareReplay({ bufferSize: 3, refCount: true })
);

withShare$.subscribe(v => console.log('購読1:', v));
withShare$.subscribe(v => console.log('購読2:', v));
// 出力: 計算が3回のみ実行される
```

## メモリ使用量の監視

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

### デバッグ環境の構築

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

### 型安全なデバッグ

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

### エラー境界の設定

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

パフォーマンスデバッグとベストプラクティス

### パフォーマンス監視
- ✅ **購読数の追跡** - defer と finalize を使った購読管理
- ✅ **再評価の検出** - shareReplay で不要な計算を回避
- ✅ **メモリ監視** - performance API でメモリ使用量を追跡

### 開発環境の最適化
- ✅ **環境別設定** - 開発環境でのみデバッグログを有効化
- ✅ **型安全なデバッグ** - TypeScript の型システムを活用
- ✅ **エラー境界** - エラーを適切に分離してデバッグ

これらの手法を組み合わせることで、RxJSアプリケーションのパフォーマンスを最適化し、効率的なデバッグ環境を構築できます。

## 関連ページ

- [デバッグの基本戦略](/guide/debugging/) - tap オペレーターや開発者ツールの使い方
- [よくあるデバッグシナリオ](/guide/debugging/common-scenarios) - 問題別のトラブルシューティング
- [カスタムデバッグツール](/guide/debugging/custom-tools) - 名前付きストリーム、デバッグオペレーター
- [オペレーター - shareReplay](/guide/operators/multicasting/shareReplay) - 不要な再評価を回避
