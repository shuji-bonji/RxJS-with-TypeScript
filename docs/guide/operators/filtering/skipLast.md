---
description: skipLastオペレーターは、Observableストリームの最後のN個の値をスキップして、それ以前の値のみを出力するRxJSフィルタリングオペレーターです。
---

# skipLast - 最後のN個の値をスキップする

`skipLast` オペレーターは、ソースObservableから発行される値のうち、**最後のN個をスキップ**して、それ以前の値のみを出力します。ストリームが完了するまで最後のN個をバッファに保持し、それ以外を出力します。

## 🔰 基本構文と使い方

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

const numbers$ = range(0, 10); // 0から9まで

numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4, 5, 6
// （7, 8, 9 はスキップされる）
```

**動作の流れ**:
1. ストリームが 0, 1, 2, ... を発行
2. 最後の3個（7, 8, 9）をバッファに保持
3. バッファサイズを超えた値（0〜6）を出力
4. ストリーム完了時、バッファの値（7, 8, 9）は出力されずに破棄

[🌐 RxJS公式ドキュメント - `skipLast`](https://rxjs.dev/api/operators/skipLast)

## 💡 典型的な活用パターン

- **最新データの除外**：確定していない最新データを除外
- **バッチ処理**：処理完了前の未確定データを除く
- **データ検証**：後続の値で検証が必要な場合
- **遅延確定データの処理**：最後のN個が確定していない場合

## 🧠 実践コード例1: データ処理パイプライン

データ処理で最後の未確定データをスキップする例です。

```ts
import { from, interval } from 'rxjs';
import { skipLast, map, take, concatMap, delay } from 'rxjs';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'データ処理パイプライン';
container.appendChild(title);

const description = document.createElement('div');
description.style.marginBottom = '10px';
description.style.color = '#666';
description.textContent = '最後の2件（未確定データ）をスキップして処理します';
container.appendChild(description);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

interface DataPoint {
  id: number;
  value: number;
  status: 'processing' | 'confirmed' | 'skipped';
}

// データストリーム（10件）
const data: DataPoint[] = Array.from({ length: 10 }, (_, i) => ({
  id: i,
  value: Math.floor(Math.random() * 100),
  status: 'processing' as const
}));

// 0.5秒ごとにデータを発行
from(data).pipe(
  concatMap(item => interval(500).pipe(
    take(1),
    map(() => item)
  )),
  skipLast(2) // 最後の2件をスキップ
).subscribe({
  next: item => {
    const div = document.createElement('div');
    div.style.padding = '5px';
    div.style.marginBottom = '5px';
    div.style.backgroundColor = '#e8f5e9';
    div.style.border = '1px solid #4CAF50';
    div.innerHTML = `
      <strong>✅ 確定</strong>
      ID: ${item.id} |
      値: ${item.value}
    `;
    output.appendChild(div);
  },
  complete: () => {
    // スキップされたアイテムを表示
    const skippedItems = data.slice(-2);
    skippedItems.forEach(item => {
      const div = document.createElement('div');
      div.style.padding = '5px';
      div.style.marginBottom = '5px';
      div.style.backgroundColor = '#ffebee';
      div.style.border = '1px solid #f44336';
      div.innerHTML = `
        <strong>⏭️ スキップ</strong>
        ID: ${item.id} |
        値: ${item.value} |
        （未確定データ）
      `;
      output.appendChild(div);
    });

    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = `処理完了: ${data.length - 2}件確定、2件スキップ`;
    output.appendChild(summary);
  }
});
```

- データは順次処理されますが、最後の2件は未確定として扱われスキップされます。
- 完了後に、スキップされたアイテムも表示されます。

## 🎯 実践コード例2: ログのフィルタリング

ログストリームから最新の未確定ログをスキップする例です。

```ts
import { interval } from 'rxjs';
import { skipLast, map, take } from 'rxjs';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'ログモニタリング';
container.appendChild(title);

const info = document.createElement('div');
info.style.marginBottom = '10px';
info.textContent = '最新3件のログは確定待ちとしてスキップされます';
info.style.color = '#666';
container.appendChild(info);

const confirmedLogs = document.createElement('div');
confirmedLogs.innerHTML = '<strong>📋 確定済みログ:</strong>';
confirmedLogs.style.marginBottom = '10px';
container.appendChild(confirmedLogs);

const confirmedList = document.createElement('div');
confirmedList.style.border = '1px solid #4CAF50';
confirmedList.style.padding = '10px';
confirmedList.style.backgroundColor = '#f1f8e9';
confirmedList.style.minHeight = '100px';
container.appendChild(confirmedList);

const pendingLogs = document.createElement('div');
pendingLogs.innerHTML = '<strong>⏳ 確定待ちログ（スキップ）:</strong>';
pendingLogs.style.marginTop = '10px';
pendingLogs.style.marginBottom = '10px';
container.appendChild(pendingLogs);

const pendingList = document.createElement('div');
pendingList.style.border = '1px solid #FF9800';
pendingList.style.padding = '10px';
pendingList.style.backgroundColor = '#fff3e0';
pendingList.style.minHeight = '60px';
container.appendChild(pendingList);

interface LogEntry {
  id: number;
  timestamp: Date;
  level: 'info' | 'warn' | 'error';
  message: string;
}

// ログを生成（合計12件、1秒ごと）
const logs$ = interval(1000).pipe(
  take(12),
  map(i => {
    const levels: ('info' | 'warn' | 'error')[] = ['info', 'warn', 'error'];
    const messages = [
      'ユーザーログイン',
      'データ取得開始',
      'キャッシュ更新',
      '接続エラー',
      'リトライ実行',
      'データ処理完了'
    ];
    return {
      id: i,
      timestamp: new Date(),
      level: levels[Math.floor(Math.random() * levels.length)],
      message: messages[Math.floor(Math.random() * messages.length)]
    } as LogEntry;
  })
);

const allLogs: LogEntry[] = [];

// すべてのログを記録（確認用）
logs$.subscribe(log => {
  allLogs.push(log);
});

// 最後の3件をスキップして確定済みログを表示
logs$.pipe(
  skipLast(3)
).subscribe({
  next: log => {
    const logDiv = document.createElement('div');
    logDiv.style.padding = '3px';
    logDiv.style.marginBottom = '3px';
    const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
    logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
    confirmedList.appendChild(logDiv);
  },
  complete: () => {
    // 最後の3件（スキップされたログ）を表示
    const skippedLogs = allLogs.slice(-3);
    skippedLogs.forEach(log => {
      const logDiv = document.createElement('div');
      logDiv.style.padding = '3px';
      logDiv.style.marginBottom = '3px';
      const icon = log.level === 'error' ? '❌' : log.level === 'warn' ? '⚠️' : 'ℹ️';
      logDiv.textContent = `${icon} [${log.id}] ${log.timestamp.toLocaleTimeString()} - ${log.message}`;
      pendingList.appendChild(logDiv);
    });
  }
});
```

- ログは順次追加されますが、最新3件は確定待ちとしてスキップされます。
- 完了後、スキップされたログも表示されます。

## 🆚 類似オペレーターとの比較

### skipLast vs takeLast vs skip

```ts
import { range } from 'rxjs';
import { skipLast, takeLast, skip } from 'rxjs';

const numbers$ = range(0, 10); // 0から9まで

// skipLast: 最後のN個をスキップ
numbers$.pipe(
  skipLast(3)
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4, 5, 6

// takeLast: 最後のN個のみ取得
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// 出力: 7, 8, 9

// skip: 最初のN個をスキップ
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// 出力: 3, 4, 5, 6, 7, 8, 9
```

| オペレーター | スキップ位置 | 出力タイミング | 完了待ち |
|:---|:---|:---|:---|
| `skipLast(n)` | 最後のn個 | バッファを超えた時点で出力 | 必要 |
| `takeLast(n)` | 最後のn個以外 | 完了後にまとめて出力 | 必要 |
| `skip(n)` | 最初のn個 | 即座に出力 | 不要 |

**視覚的な違い**:

```
入力: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

skipLast(3): 0, 1, 2, 3, 4, 5, 6 | [7, 8, 9 スキップ]
                                   ^最後の3個

takeLast(3): [0~6 スキップ] | 7, 8, 9
                             ^最後の3個のみ

skip(3): [0, 1, 2 スキップ] | 3, 4, 5, 6, 7, 8, 9
          ^最初の3個
```

## ⚠️ 注意点

### 1. 無限ストリームでの動作

`skipLast` は完了するまで最後のN個を特定できないため、無限ストリームでは意図した動作になりません。

```ts
import { interval } from 'rxjs';
import { skipLast } from 'rxjs';

// ❌ 悪い例: 無限ストリームで skipLast を使用
interval(1000).pipe(
  skipLast(3)
).subscribe(console.log);
// 出力: 0（3秒後）, 1（4秒後）, 2（5秒後）, ...
// N個の遅延を伴って無限に出力され続ける
// 最後の3個は永遠にバッファに残り、決して出力されない
```

無限ストリームの場合、最後のN個が確定しないため、すべての値がN個遅延して出力され続けます。真の「最後のN個」は存在しないため、`skipLast` の本来の目的を達成できません。

**解決策**: `take` で有限ストリームにする

```ts
import { interval } from 'rxjs';
import { take, skipLast } from 'rxjs';

// ✅ 良い例: 有限ストリームにしてから skipLast を使用
interval(1000).pipe(
  take(10),      // 最初の10個で完了
  skipLast(3)    // 最後の3個をスキップ
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4, 5, 6
// （7, 8, 9 はスキップされる）
```

### 2. バッファサイズに注意

`skipLast(n)` は常にn個の値をバッファに保持します。

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

// ⚠️ 1000個をバッファに保持
range(0, 1000000).pipe(
  skipLast(1000)
).subscribe(console.log);
```

### 3. 出力の遅延

`skipLast(n)` はバッファがn個溜まるまで何も出力しません。

```ts
import { interval } from 'rxjs';
import { take, skipLast, tap } from 'rxjs';

interval(1000).pipe(
  take(5),
  tap(val => console.log('入力:', val)),
  skipLast(2)
).subscribe(val => console.log('出力:', val));
// 入力: 0
// 入力: 1
// 入力: 2
// 出力: 0  ← バッファが2個溜まってから出力開始
// 入力: 3
// 出力: 1
// 入力: 4
// 出力: 2
// 完了（3, 4 はスキップ）
```

### 4. skipLast(0) の動作

`skipLast(0)` は何もスキップしません。

```ts
import { range } from 'rxjs';
import { skipLast } from 'rxjs';

range(0, 5).pipe(
  skipLast(0)
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4（すべて出力）
```

## 💡 実用的な組み合わせパターン

### パターン1: 中間部分のみ取得

最初と最後をスキップして、中間部分のみを取得

```ts
import { range } from 'rxjs';
import { skip, skipLast } from 'rxjs';

range(0, 10).pipe(
  skip(2),      // 最初の2個をスキップ
  skipLast(2)   // 最後の2個をスキップ
).subscribe(console.log);
// 出力: 2, 3, 4, 5, 6, 7
```

### パターン2: データ検証

後続の値で検証が必要な場合

```ts
import { from } from 'rxjs';
import { skipLast, map } from 'rxjs';

interface Transaction {
  id: number;
  amount: number;
  pending: boolean;
}

const transactions$ = from([
  { id: 1, amount: 100, pending: false },
  { id: 2, amount: 200, pending: false },
  { id: 3, amount: 150, pending: false },
  { id: 4, amount: 300, pending: true },  // 未確定
  { id: 5, amount: 250, pending: true }   // 未確定
]);

// 未確定のトランザクション（最後の2件）をスキップ
transactions$.pipe(
  skipLast(2)
).subscribe(tx => {
  console.log(`確定: ID ${tx.id}, 金額 ${tx.amount}円`);
});
// 出力:
// 確定: ID 1, 金額 100円
// 確定: ID 2, 金額 200円
// 確定: ID 3, 金額 150円
```

### パターン3: ウィンドウ処理

最新N件を除いたデータでウィンドウ処理

```ts
import { range } from 'rxjs';
import { skipLast, bufferCount } from 'rxjs';

range(0, 10).pipe(
  skipLast(2),      // 最後の2件をスキップ
  bufferCount(3, 1) // 3件ずつのウィンドウ
).subscribe(window => {
  console.log('ウィンドウ:', window);
});
// 出力:
// ウィンドウ: [0, 1, 2]
// ウィンドウ: [1, 2, 3]
// ウィンドウ: [2, 3, 4]
// ...
```

## 📚 関連オペレーター

- **[skip](./skip)** - 最初のN個の値をスキップ
- **[takeLast](./takeLast)** - 最後のN個の値のみ取得
- **[take](./take)** - 最初のN個の値のみ取得
- **[skipUntil](./skipUntil)** - 別のObservableが発火するまでスキップ
- **[skipWhile](./skipWhile)** - 条件を満たす間スキップ

## まとめ

`skipLast` オペレーターは、ストリームの最後のN個の値をスキップします。

- ✅ 最後のN個のデータが不要な場合に最適
- ✅ 未確定データの除外に便利
- ✅ バッファサイズはN個のみ（メモリ効率良好）
- ✅ ストリーム完了が必要
- ⚠️ 無限ストリームでは使用不可
- ⚠️ バッファがN個溜まるまで出力されない
- ⚠️ `take` と組み合わせて有限ストリームにする必要がある場合が多い
