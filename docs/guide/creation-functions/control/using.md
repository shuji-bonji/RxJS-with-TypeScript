---
description: RxJSのusing()関数を使って、リソースのライフサイクルを自動管理し、メモリリークを防ぐ方法を、実践的なコード例とともに詳しく解説します。
---

# using()

[📘 RxJS公式ドキュメント - using](https://rxjs.dev/api/index/function/using)

`using()` は、Observable のライフサイクルに合わせてリソースを自動的に作成・解放する Creation Function です。WebSocket、ファイルハンドル、タイマーなど、手動でクリーンアップが必要なリソースを安全に管理し、メモリリークを防ぎます。

## 基本的な使い方

### シンプルなリソース管理

```typescript
import { using, interval, Subscription, take } from 'rxjs';
const resource$ = using(
  // リソースファクトリー: 購読開始時に実行
  () => {
    console.log('リソース作成');
    return new Subscription(() => console.log('リソース解放'));
  },
  // Observable ファクトリー: リソースを使ってObservableを作成
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('値:', value),
  complete: () => console.log('完了')
});

// 出力:
// リソース作成
// 値: 0
// 値: 1
// 値: 2
// 完了
// リソース解放
```

> [!IMPORTANT]
> **自動リソース解放**
>
> `using()` は、Observable が完了（`complete`）または購読解除（`unsubscribe`）されたタイミングで、自動的にリソースを解放します。

## using() の仕組み

`using()` は以下の2つの関数を受け取ります。

```typescript
function using<T>(
  resourceFactory: () => Unsubscribable | void,
  observableFactory: (resource: Unsubscribable | void) => ObservableInput<T>
): Observable<T>
```

### 1. resourceFactory（リソースファクトリー）

購読開始時に実行され、リソースを作成します。返す必要があるのは `unsubscribe()` メソッドを持つオブジェクトです。

```typescript
// Subscription を返す
() => new Subscription(() => {
  console.log('クリーンアップ処理');
});

// または、unsubscribe メソッドを持つオブジェクトを返す
() => ({
  unsubscribe: () => {
    console.log('クリーンアップ処理');
  }
});
```

### 2. observableFactory（Observable ファクトリー）

リソースを使って Observable を作成します。

```typescript
(resource) => interval(1000);
```

## 実践的なパターン

### WebSocket接続の管理

```typescript
import { using, interval, Subject, map, takeUntil } from 'rxjs';
function createWebSocketStream(url: string) {
  return using(
    // WebSocket接続を作成
    () => {
      const ws = new WebSocket(url);
      console.log('WebSocket接続開始:', url);

      ws.onopen = () => console.log('接続完了');
      ws.onerror = (error) => console.error('接続エラー:', error);

      return {
        unsubscribe: () => {
          console.log('WebSocket接続終了');
          ws.close();
        }
      };
    },
    // メッセージストリームを作成
    () => {
      const messages$ = new Subject<MessageEvent>();
      const ws = new WebSocket(url);

      ws.onmessage = (event) => messages$.next(event);
      ws.onerror = (error) => messages$.error(error);
      ws.onclose = () => messages$.complete();

      return messages$;
    }
  );
}

// 使用例
const websocket$ = createWebSocketStream('wss://echo.websocket.org');

const subscription = websocket$.subscribe({
  next: message => console.log('受信:', message.data),
  error: error => console.error('エラー:', error),
  complete: () => console.log('完了')
});

// 10秒後に自動的にWebSocketを閉じる
setTimeout(() => subscription.unsubscribe(), 10000);
```

### タイマーの自動クリーンアップ

```typescript
import { using, Observable, Subscription } from 'rxjs';

function createTimerStream(intervalMs: number) {
  return using(
    // タイマーリソースを作成
    () => {
      let timerId: number | null = null;
      console.log('タイマー開始');

      return new Subscription(() => {
        if (timerId !== null) {
          clearInterval(timerId);
          console.log('タイマー停止');
        }
      });
    },
    // タイマーストリームを作成
    () => new Observable(subscriber => {
      const timerId = setInterval(() => {
        subscriber.next(Date.now());
      }, intervalMs);

      return () => clearInterval(timerId);
    })
  );
}

// 使用例
const timer$ = createTimerStream(1000);

const subscription = timer$.subscribe({
  next: time => console.log('現在時刻:', new Date(time).toLocaleTimeString())
});

// 5秒後に停止
setTimeout(() => subscription.unsubscribe(), 5000);
```

### ファイル操作（Node.js）

```typescript
import { using, Observable } from 'rxjs';
import * as fs from 'fs';

function readFileStream(filePath: string) {
  return using(
    // ファイルハンドルを開く
    () => {
      const fd = fs.openSync(filePath, 'r');
      console.log('ファイルオープン:', filePath);

      return {
        unsubscribe: () => {
          fs.closeSync(fd);
          console.log('ファイルクローズ');
        }
      };
    },
    // ファイル読み込みストリームを作成
    () => new Observable<string>(subscriber => {
      const stream = fs.createReadStream(filePath, { encoding: 'utf8' });

      stream.on('data', (chunk) => subscriber.next(chunk));
      stream.on('error', (error) => subscriber.error(error));
      stream.on('end', () => subscriber.complete());

      return () => stream.destroy();
    })
  );
}

// 使用例
const file$ = readFileStream('./data.txt');

file$.subscribe({
  next: chunk => console.log('読み込み:', chunk),
  error: error => console.error('エラー:', error),
  complete: () => console.log('読み込み完了')
});
```

### イベントリスナーの管理

```typescript
import { using, Observable } from 'rxjs';

function createClickStream(element: HTMLElement) {
  return using(
    // イベントリスナーを登録
    () => {
      console.log('イベントリスナー登録');

      return {
        unsubscribe: () => {
          console.log('イベントリスナー解除');
          // 実際の解除は Observable ファクトリー内で行う
        }
      };
    },
    // クリックイベントストリームを作成
    () => new Observable<MouseEvent>(subscriber => {
      const handler = (event: MouseEvent) => subscriber.next(event);

      element.addEventListener('click', handler);

      return () => {
        element.removeEventListener('click', handler);
      };
    })
  );
}

// 使用例
const button = document.querySelector('#myButton') as HTMLElement;
const clicks$ = createClickStream(button);

const subscription = clicks$.subscribe({
  next: event => console.log('クリック位置:', event.clientX, event.clientY)
});

// 30秒後に自動解除
setTimeout(() => subscription.unsubscribe(), 30000);
```

## よくある使用例

### 1. データベース接続の管理

```typescript
import { using, from, mergeMap } from 'rxjs';
interface DbConnection {
  query: (sql: string) => Promise<any[]>;
  close: () => Promise<void>;
}

function queryWithConnection(sql: string) {
  return using(
    // データベース接続を確立
    () => {
      const connection = createDbConnection();
      console.log('DB接続確立');

      return {
        unsubscribe: async () => {
          await connection.close();
          console.log('DB接続クローズ');
        }
      };
    },
    // クエリを実行
    () => {
      const connection = createDbConnection();
      return from(connection.query(sql));
    }
  );
}

// 使用例
const users$ = queryWithConnection('SELECT * FROM users');

users$.subscribe({
  next: rows => console.log('取得:', rows),
  error: error => console.error('エラー:', error),
  complete: () => console.log('クエリ完了')
});

function createDbConnection(): DbConnection {
  // 実際の接続処理
  return {
    query: async (sql) => [],
    close: async () => {}
  };
}
```

### 2. リソースプールの管理

```typescript
import { using, Observable, defer } from 'rxjs';

class ResourcePool<T> {
  private available: T[] = [];
  private inUse = new Set<T>();

  constructor(private factory: () => T, size: number) {
    for (let i = 0; i < size; i++) {
      this.available.push(factory());
    }
  }

  acquire(): T | null {
    const resource = this.available.pop();
    if (resource) {
      this.inUse.add(resource);
      return resource;
    }
    return null;
  }

  release(resource: T): void {
    if (this.inUse.has(resource)) {
      this.inUse.delete(resource);
      this.available.push(resource);
    }
  }
}

// 使用例
const pool = new ResourcePool(() => ({ id: Math.random() }), 5);

function usePooledResource<T>(
  pool: ResourcePool<T>,
  work: (resource: T) => Observable<any>
) {
  return using(
    () => {
      const resource = pool.acquire();
      if (!resource) {
        throw new Error('リソースプールが枯渇しています');
      }
      console.log('リソース取得:', resource);

      return {
        unsubscribe: () => {
          pool.release(resource);
          console.log('リソース返却:', resource);
        }
      };
    },
    (subscription) => {
      const resource = pool.acquire();
      return resource ? work(resource) : defer(() => {
        throw new Error('リソース取得失敗');
      });
    }
  );
}

// リソースを使って処理
const work$ = usePooledResource(pool, (resource) =>
  new Observable(subscriber => {
    subscriber.next(`処理中: ${resource.id}`);
    setTimeout(() => subscriber.complete(), 1000);
  })
);

work$.subscribe({
  next: result => console.log(result),
  complete: () => console.log('処理完了')
});
```

### 3. 複数リソースの連携管理

```typescript
import { using, merge, Subject } from 'rxjs';

interface MultiResource {
  ws: WebSocket;
  timer: number;
}

function createMultiResourceStream() {
  return using(
    // 複数のリソースを作成
    () => {
      const ws = new WebSocket('wss://echo.websocket.org');
      const timer = setInterval(() => {
        console.log('定期実行');
      }, 1000);

      console.log('複数リソース作成');

      return {
        unsubscribe: () => {
          ws.close();
          clearInterval(timer);
          console.log('複数リソース解放');
        }
      };
    },
    // 複数のストリームを結合
    () => {
      const messages$ = new Subject<string>();
      const ticks$ = new Subject<number>();

      return merge(messages$, ticks$);
    }
  );
}

// 使用例
const multiStream$ = createMultiResourceStream();

const subscription = multiStream$.subscribe({
  next: value => console.log('受信:', value)
});

// 10秒後に全リソースを解放
setTimeout(() => subscription.unsubscribe(), 10000);
```

### 4. 条件付きリソース管理

```typescript
import { using, interval, EMPTY, take } from 'rxjs';
function conditionalResource(shouldCreate: boolean) {
  return using(
    () => {
      if (shouldCreate) {
        console.log('リソース作成');
        return {
          unsubscribe: () => console.log('リソース解放')
        };
      } else {
        console.log('リソース作成スキップ');
        return { unsubscribe: () => {} };
      }
    },
    () => {
      if (shouldCreate) {
        return interval(1000).pipe(take(3));
      } else {
        return EMPTY;
      }
    }
  );
}

// リソースを作成する場合
conditionalResource(true).subscribe({
  next: val => console.log('値:', val),
  complete: () => console.log('完了')
});

// リソースを作成しない場合
conditionalResource(false).subscribe({
  next: val => console.log('値:', val),
  complete: () => console.log('完了')
});
```

## エラーハンドリング

### エラー発生時のリソース解放

```typescript
import { using, throwError, of, catchError } from 'rxjs';
const errorHandling$ = using(
  () => {
    console.log('リソース作成');
    return {
      unsubscribe: () => console.log('リソース解放（エラー時も実行）')
    };
  },
  () => throwError(() => new Error('意図的なエラー'))
);

errorHandling$.pipe(
  catchError(error => {
    console.error('エラー捕捉:', error.message);
    return of('デフォルト値');
  })
).subscribe({
  next: val => console.log('値:', val),
  complete: () => console.log('完了')
});

// 出力:
// リソース作成
// リソース解放（エラー時も実行）
// エラー捕捉: 意図的なエラー
// 値: デフォルト値
// 完了
```

> [!IMPORTANT]
> **エラー時も確実にリソース解放**
>
> `using()` は、エラーが発生した場合でも、`resourceFactory` で作成したリソースを必ず解放します。

## よくあるエラーと対処法

### 1. unsubscribe メソッドの実装忘れ

**エラー例:**
```typescript
// ❌ エラー: unsubscribe メソッドがない
using(
  () => {
    console.log('リソース作成');
    return {}; // unsubscribe がない
  },
  () => interval(1000)
);
```

**対処法:**
```typescript
// ✅ 正しい: unsubscribe メソッドを実装
using(
  () => {
    console.log('リソース作成');
    return {
      unsubscribe: () => console.log('リソース解放')
    };
  },
  () => interval(1000)
);
```

### 2. 非同期リソースの作成

**問題:**
```typescript
// ❌ 問題: resourceFactory は非同期にできない
using(
  async () => { // async は使えない
    const resource = await createResourceAsync();
    return resource;
  },
  () => interval(1000)
);
```

**対処法:**
```typescript
import { defer, from, mergeMap } from 'rxjs';
// ✅ 正しい: defer と mergeMap で非同期処理

defer(() =>
  from(createResourceAsync()).pipe(
    mergeMap(resource =>
      using(
        () => resource,
        () => interval(1000)
      )
    )
  )
);
```

### 3. リソースの重複作成

**問題:**
```typescript
// ❌ 問題: resourceFactory と observableFactory で別々にリソースを作成
let sharedResource: any;

using(
  () => {
    sharedResource = createResource(); // ここで作成
    return { unsubscribe: () => sharedResource.close() };
  },
  () => {
    const resource = createResource(); // また作成してしまう
    return from(resource.getData());
  }
);
```

**対処法:**
```typescript
// ✅ 正しい: リソースを共有
using(
  () => {
    const resource = createResource();
    return {
      resource, // リソースを保持
      unsubscribe: () => resource.close()
    };
  },
  (subscription: any) => {
    return from(subscription.resource.getData());
  }
);
```

## using() のベストプラクティス

### 1. リソースの確実な解放

```typescript
// ✅ 良い例: try-finally パターン
using(
  () => {
    const resource = createResource();
    return {
      unsubscribe: () => {
        try {
          resource.close();
        } catch (error) {
          console.error('リソース解放エラー:', error);
        }
      }
    };
  },
  () => interval(1000)
);
```

### 2. リソース作成のログ記録

```typescript
// ✅ 良い例: リソースのライフサイクルをログに記録
using(
  () => {
    const resourceId = Math.random();
    console.log(`[${resourceId}] リソース作成`);

    return {
      unsubscribe: () => {
        console.log(`[${resourceId}] リソース解放`);
      }
    };
  },
  () => interval(1000)
);
```

### 3. 型安全なリソース管理

```typescript
// ✅ 良い例: TypeScript の型を活用
interface ManagedResource {
  id: string;
  close: () => void;
}

function createManagedStream(resource: ManagedResource) {
  return using(
    () => {
      console.log('リソース開始:', resource.id);
      return {
        unsubscribe: () => {
          resource.close();
          console.log('リソース終了:', resource.id);
        }
      };
    },
    () => interval(1000)
  );
}
```

## 手動管理との比較

### 手動でのリソース管理（❌ 推奨しない）

```typescript
// ❌ 悪い例: 手動管理（解放忘れのリスク）
const ws = new WebSocket('wss://example.com');
const subscription = interval(1000).subscribe(() => {
  ws.send('ping');
});

// 解放忘れる可能性がある
// subscription.unsubscribe();
// ws.close();
```

### using() によるリソース管理（✅ 推奨）

```typescript
// ✅ 良い例: using() で自動管理
const stream$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return {
      unsubscribe: () => ws.close()
    };
  },
  () => interval(1000)
);

const subscription = stream$.subscribe(() => {
  // WebSocketを使った処理
});

// unsubscribe() だけでWebSocketも自動的にクローズ
subscription.unsubscribe();
```

## まとめ

`using()` は、Observable のライフサイクルに合わせてリソースを自動管理する Creation Function です。

**主な特徴:**
- 購読開始時にリソースを作成
- 購読終了時（complete または unsubscribe）に自動解放
- メモリリークを防ぐ
- エラー時も確実にリソース解放

**使用場面:**
- WebSocket、EventSource などのネットワーク接続
- ファイルハンドル、データベース接続
- タイマー、インターバルの自動クリーンアップ
- イベントリスナーの自動解除

**注意点:**
- `resourceFactory` は同期関数である必要がある
- 必ず `unsubscribe` メソッドを実装する
- エラーハンドリングを適切に行う

**推奨される使い方:**
- リソースの解放忘れを防ぐ
- ライフサイクルをログに記録
- TypeScriptの型を活用して型安全に管理

## 関連ページ

- [scheduled()](/guide/creation-functions/control/scheduled) - スケジューラーを指定してObservableを生成
- [制御系 Creation Functions](/guide/creation-functions/control/) - scheduled() と using() の比較
- [finalize()](/guide/error-handling/finalize) - 購読終了時の処理を追加するオペレーター

## 参考リソース

- [RxJS公式ドキュメント - using](https://rxjs.dev/api/index/function/using)
- [RxJS公式ドキュメント - Subscription](https://rxjs.dev/guide/subscription)
