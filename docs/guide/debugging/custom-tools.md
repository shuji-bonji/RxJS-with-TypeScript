---
description: "RxJSデバッグ用のカスタムツールの作成方法を解説。名前付きストリーム追跡オペレーター、設定可能なデバッグオペレーター、パフォーマンス計測用オペレーター、エラー境界オペレーターなど、実践的なデバッグツールをTypeScriptで型安全に実装する方法を紹介します。"
---

# カスタムデバッグツール

独自のデバッグツールを作成することで、プロジェクトの要件に合わせた柔軟なデバッグが可能になります。

## 名前付きストリームのデバッグ

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

### 使用例

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

### 複数のストリームを追跡

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

## カスタムデバッグオペレーターの作成

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

## パフォーマンス計測用のデバッグオペレーター

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

## まとめ

カスタムデバッグツールの作成により

- ✅ **名前付きストリーム** - 複数のストリームを名前で識別して追跡
- ✅ **柔軟な設定** - プロジェクト要件に合わせたデバッグオペレーター
- ✅ **パフォーマンス計測** - 実行時間と値の数を自動記録
- ✅ **ログ管理** - タイムスタンプ付きログの記録と取得

これらのツールは、開発環境でのみ有効化し、本番環境では無効化することを推奨します。

## 関連ページ

- [デバッグの基本戦略](/guide/debugging/) - tap オペレーターや開発者ツールの使い方
- [よくあるデバッグシナリオ](/guide/debugging/common-scenarios) - 問題別のトラブルシューティング
- [パフォーマンスデバッグ](/guide/debugging/performance) - 購読数監視、メモリ使用量確認
