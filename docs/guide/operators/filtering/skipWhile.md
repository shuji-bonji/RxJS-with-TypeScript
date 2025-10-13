---
description: skipWhileオペレーターは、指定した条件を満たす間は値をスキップし、条件がfalseになった時点から以降のすべての値を発行します。動的な開始条件でストリームを制御したい場合に便利です。
---

# skipWhile - 条件を満たす間値をスキップする

`skipWhile` オペレーターは、**指定した条件を満たす間**は値をスキップし続け、条件が`false`になった時点から以降の**すべての値を発行**します。

## 🔰 基本構文と使い方

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs/operators';

const numbers$ = range(0, 10); // 0から9まで

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// 出力: 5, 6, 7, 8, 9
```

**動作の流れ**:
1. 0 が発行 → `0 < 5` は `true` → スキップ
2. 1 が発行 → `1 < 5` は `true` → スキップ
3. 2 が発行 → `2 < 5` は `true` → スキップ
4. 3 が発行 → `3 < 5` は `true` → スキップ
5. 4 が発行 → `4 < 5` は `true` → スキップ
6. 5 が発行 → `5 < 5` は `false` → 出力開始
7. 6 以降 → すべて出力（条件は再評価されない）

[🌐 RxJS公式ドキュメント - `skipWhile`](https://rxjs.dev/api/operators/skipWhile)

## 💡 典型的な活用パターン

- **初期の不要なデータのスキップ**：ウォームアップ期間のデータを除外
- **閾値に達するまでスキップ**：特定の条件を満たすまで待機
- **ヘッダー行のスキップ**：CSVなどのヘッダーを除外
- **準備期間のスキップ**：システムの準備が完了するまで待つ

## 🧠 実践コード例1: センサーのウォームアップ期間をスキップ

センサーが安定するまでの初期データをスキップする例です。

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs/operators';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = '温度センサーモニタリング';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginBottom = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#fff3e0';
status.style.border = '1px solid #FF9800';
status.textContent = '🔄 センサー準備中...（温度が20°C以上で計測開始）';
container.appendChild(status);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

let isWarmedUp = false;

// 温度センサーのシミュレーション（徐々に温まる）
interval(500).pipe(
  take(20),
  map(i => {
    // 最初は低温、徐々に上昇
    const baseTemp = 15 + i * 0.5;
    const noise = (Math.random() - 0.5) * 2;
    return baseTemp + noise;
  }),
  skipWhile(temp => temp < 20) // 20°C未満はスキップ
).subscribe({
  next: temp => {
    // 最初の値が来たときにステータスを更新
    if (!isWarmedUp) {
      isWarmedUp = true;
      status.textContent = '✅ センサー準備完了（計測開始）';
      status.style.backgroundColor = '#e8f5e9';
      status.style.borderColor = '#4CAF50';
    }

    const log = document.createElement('div');
    log.style.padding = '5px';
    log.style.marginBottom = '3px';
    log.style.backgroundColor = temp > 25 ? '#ffebee' : '#f1f8e9';
    log.textContent = `[${new Date().toLocaleTimeString()}] 温度: ${temp.toFixed(1)}°C`;
    output.insertBefore(log, output.firstChild);

    // 最大10件まで表示
    while (output.children.length > 10) {
      output.removeChild(output.lastChild!);
    }
  },
  complete: () => {
    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = '計測完了';
    container.appendChild(summary);
  }
});
```

- センサーが20°C未満の間はデータをスキップします。
- 20°C以上になった時点から、すべてのデータが記録されます。

## 🎯 実践コード例2: 準備完了後のイベント処理

システムの初期化が完了するまでイベントをスキップする例です。

```ts
import { fromEvent, merge, Subject } from 'rxjs';
import { skipWhile, map, tap } from 'rxjs/operators';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'イベント処理システム';
container.appendChild(title);

const initButton = document.createElement('button');
initButton.textContent = '初期化完了';
initButton.style.marginRight = '10px';
container.appendChild(initButton);

const eventButton = document.createElement('button');
eventButton.textContent = 'イベント発火';
container.appendChild(eventButton);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
statusDiv.style.padding = '10px';
statusDiv.style.backgroundColor = '#ffebee';
statusDiv.style.border = '1px solid #f44336';
statusDiv.innerHTML = '<strong>⏸️ システム未初期化</strong><br>イベントはスキップされます';
container.appendChild(statusDiv);

const eventLog = document.createElement('div');
eventLog.style.marginTop = '10px';
eventLog.style.border = '1px solid #ccc';
eventLog.style.padding = '10px';
eventLog.style.minHeight = '100px';
container.appendChild(eventLog);

// 初期化状態
let isInitialized = false;
const initSubject = new Subject<boolean>();

// 初期化ボタン
fromEvent(initButton, 'click').subscribe(() => {
  if (!isInitialized) {
    isInitialized = true;
    initSubject.next(true);
    statusDiv.style.backgroundColor = '#e8f5e9';
    statusDiv.style.borderColor = '#4CAF50';
    statusDiv.innerHTML = '<strong>✅ システム初期化完了</strong><br>イベントを処理します';
    initButton.disabled = true;
  }
});

// イベント処理（初期化完了までスキップ）
let eventCount = 0;
fromEvent(eventButton, 'click').pipe(
  map(() => {
    eventCount++;
    return {
      id: eventCount,
      timestamp: new Date(),
      initialized: isInitialized
    };
  }),
  tap(event => {
    if (!event.initialized) {
      const skipLog = document.createElement('div');
      skipLog.style.padding = '5px';
      skipLog.style.marginBottom = '3px';
      skipLog.style.color = '#999';
      skipLog.textContent = `⏭️ イベント #${event.id} スキップ（未初期化）`;
      eventLog.insertBefore(skipLog, eventLog.firstChild);
    }
  }),
  skipWhile(event => !event.initialized)
).subscribe(event => {
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.marginBottom = '3px';
  log.style.backgroundColor = '#e8f5e9';
  log.style.border = '1px solid #4CAF50';
  log.innerHTML = `
    <strong>✅ イベント #${event.id} 処理</strong>
    [${event.timestamp.toLocaleTimeString()}]
  `;
  eventLog.insertBefore(log, eventLog.firstChild);

  // 最大10件まで表示
  while (eventLog.children.length > 10) {
    eventLog.removeChild(eventLog.lastChild!);
  }
});
```

- システムが初期化されるまでのイベントはすべてスキップされます。
- 初期化完了後は、すべてのイベントが処理されます。

## 🆚 類似オペレーターとの比較

### skipWhile vs takeWhile vs skip vs filter

```ts
import { range } from 'rxjs';
import { skipWhile, takeWhile, skip, filter } from 'rxjs/operators';

const numbers$ = range(0, 10); // 0から9まで

// skipWhile: 条件を満たす間スキップ、以降すべて出力
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// 出力: 5, 6, 7, 8, 9

// takeWhile: 条件を満たす間のみ取得
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4

// skip: 最初のN個をスキップ
numbers$.pipe(
  skip(5)
).subscribe(console.log);
// 出力: 5, 6, 7, 8, 9

// filter: 条件を満たす値のみ通過（全体を評価）
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(console.log);
// 出力: 5, 6, 7, 8, 9
```

| オペレーター | 動作 | 条件の再評価 | 完了タイミング |
|:---|:---|:---|:---|
| `skipWhile(predicate)` | 条件を満たす間スキップ | しない（一度falseになれば終了） | 元のストリーム完了時 |
| `takeWhile(predicate)` | 条件を満たす間取得 | 毎回評価 | 条件がfalseになった時 |
| `skip(n)` | 最初のn個をスキップ | なし（個数ベース） | 元のストリーム完了時 |
| `filter(predicate)` | 条件を満たす値のみ | **毎回評価** | 元のストリーム完了時 |

**視覚的な違い**:

```
入力: 0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0

skipWhile(n => n < 5):
[0,1,2,3,4 スキップ] | 5, 4, 3, 2, 1, 0
                      ^条件がfalseになった後はすべて出力

filter(n => n >= 5):
[0,1,2,3,4 除外] 5 [4,3,2,1,0 除外]
                 ^条件を満たす値のみ出力（毎回評価）

takeWhile(n => n < 5):
0, 1, 2, 3, 4 | [5以降すべて無視して完了]
```

## ⚠️ 注意点

### 1. 条件は一度falseになったら再評価されない

これが `filter` との最大の違いです。

```ts
import { from } from 'rxjs';
import { skipWhile, filter } from 'rxjs/operators';

const numbers$ = from([1, 2, 3, 4, 5, 4, 3, 2, 1]);

// skipWhile: 一度条件がfalseになったら、以降すべて出力
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(val => console.log('skipWhile:', val));
// 出力: skipWhile: 5, 4, 3, 2, 1（5以降すべて出力）

// filter: 毎回条件を評価
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(val => console.log('filter:', val));
// 出力: filter: 5（5のみ出力）
```

### 2. 最初から条件がfalseの場合

最初から条件が `false` の場合、すべての値が出力されます。

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs/operators';

range(5, 5).pipe( // 5から9まで
  skipWhile(n => n < 3) // 最初から条件がfalse
).subscribe(console.log);
// 出力: 5, 6, 7, 8, 9（すべて出力）
```

### 3. すべての値が条件を満たす場合

すべての値が条件を満たす場合、何も出力されません。

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs/operators';

range(0, 5).pipe( // 0から4まで
  skipWhile(n => n < 10) // すべての値が条件を満たす
).subscribe({
  next: console.log,
  complete: () => console.log('完了（何も出力されない）')
});
// 出力: 完了（何も出力されない）
```

### 4. TypeScript での型

`skipWhile` は型を変更しません。

```ts
import { Observable, from } from 'rxjs';
import { skipWhile } from 'rxjs/operators';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

const users$: Observable<User> = from([
  { id: 1, name: 'Alice', isActive: false },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true },
  { id: 4, name: 'Dave', isActive: true }
]);

// 型は Observable<User> のまま
const activeUsers$: Observable<User> = users$.pipe(
  skipWhile(user => !user.isActive)
);

activeUsers$.subscribe(user => {
  console.log(`${user.name} (ID: ${user.id})`);
});
// 出力: Charlie (ID: 3), Dave (ID: 4)
```

## 💡 実用的な組み合わせパターン

### パターン1: ヘッダー行のスキップ

CSVなどのヘッダー行をスキップ

```ts
import { from } from 'rxjs';
import { skipWhile, map } from 'rxjs/operators';

const csvLines$ = from([
  'Name,Age,City',     // ヘッダー行
  'Alice,25,Tokyo',
  'Bob,30,Osaka',
  'Charlie,35,Kyoto'
]);

let isFirstLine = true;

csvLines$.pipe(
  skipWhile(() => {
    if (isFirstLine) {
      isFirstLine = false;
      return true; // 最初の行（ヘッダー）をスキップ
    }
    return false;
  }),
  map(line => {
    const [name, age, city] = line.split(',');
    return { name, age: Number(age), city };
  })
).subscribe(console.log);
// 出力:
// { name: 'Alice', age: 25, city: 'Tokyo' }
// { name: 'Bob', age: 30, city: 'Osaka' }
// { name: 'Charlie', age: 35, city: 'Kyoto' }
```

### パターン2: タイムスタンプベースのフィルタリング

特定時刻以降のデータのみ処理

```ts
import { from } from 'rxjs';
import { skipWhile } from 'rxjs/operators';

interface LogEntry {
  timestamp: Date;
  message: string;
}

const startTime = new Date('2025-01-01T12:00:00');

const logs$ = from([
  { timestamp: new Date('2025-01-01T10:00:00'), message: 'Log 1' },
  { timestamp: new Date('2025-01-01T11:00:00'), message: 'Log 2' },
  { timestamp: new Date('2025-01-01T12:00:00'), message: 'Log 3' },
  { timestamp: new Date('2025-01-01T13:00:00'), message: 'Log 4' }
] as LogEntry[]);

logs$.pipe(
  skipWhile(log => log.timestamp < startTime)
).subscribe(log => {
  console.log(`[${log.timestamp.toISOString()}] ${log.message}`);
});
// 出力:
// [2025-01-01T12:00:00.000Z] Log 3
// [2025-01-01T13:00:00.000Z] Log 4
```

### パターン3: 状態ベースのスキップ

システムの準備が整うまでスキップ

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs/operators';

interface SystemState {
  tick: number;
  isReady: boolean;
  data: number;
}

// システム状態のシミュレーション
interval(500).pipe(
  take(10),
  map(i => ({
    tick: i,
    isReady: i >= 3, // 3秒後に準備完了
    data: Math.floor(Math.random() * 100)
  } as SystemState)),
  skipWhile(state => !state.isReady)
).subscribe(state => {
  console.log(`Tick ${state.tick}: データ=${state.data}`);
});
// 出力: Tick 3以降のデータのみ
```

## 📚 関連オペレーター

- **[takeWhile](./takeWhile)** - 条件を満たす間のみ値を取得
- **[skip](./skip)** - 最初のN個の値をスキップ
- **[skipLast](./skipLast)** - 最後のN個の値をスキップ
- **[skipUntil](./skipUntil)** - 別のObservableが発火するまでスキップ
- **[filter](./filter)** - 条件を満たす値のみ通過

## まとめ

`skipWhile` オペレーターは、条件を満たす間値をスキップし、条件がfalseになった時点から以降のすべての値を発行します。

- ✅ 初期の不要なデータをスキップするのに最適
- ✅ 条件は一度falseになったら再評価されない
- ✅ ウォームアップ期間や準備期間のスキップに便利
- ✅ ヘッダー行のスキップに使える
- ⚠️ `filter` とは異なり、条件は一度のみ評価
- ⚠️ すべての値が条件を満たすと何も出力されない
- ⚠️ 元のストリームが完了するまで続く
