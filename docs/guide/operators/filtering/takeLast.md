---
description: takeLastオペレーターは、Observableストリームが完了した時点で、最後のN個の値のみを出力します。ログの最新件数取得や、完了時の最終データ取得に便利です。
---

# takeLast - 最後のN個の値を取得する

`takeLast` オペレーターは、ストリームが**完了した時点**で、最後のN個の値のみを出力します。ストリームが完了するまでバッファに値を保持し、完了後にまとめて出力します。


## 🔰 基本構文と使い方

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs/operators';

const numbers$ = range(0, 10); // 0から9まで

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// 出力: 7, 8, 9
```

**動作の流れ**:
1. ストリームが 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 を発行
2. 内部的に最後の3個をバッファに保持
3. ストリームが完了
4. バッファの値 7, 8, 9 を順番に出力

[🌐 RxJS公式ドキュメント - `takeLast`](https://rxjs.dev/api/operators/takeLast)


## 🆚 take との対比

`take` と `takeLast` は対照的な動作をします。

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs/operators';

const numbers$ = range(0, 10); // 0から9まで

// take: 最初のN個を取得
numbers$.pipe(
  take(3)
).subscribe(console.log);
// 出力: 0, 1, 2（すぐに出力される）

// takeLast: 最後のN個を取得
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// 出力: 7, 8, 9（完了を待ってから出力される）
```

| オペレーター | 取得位置 | 出力タイミング | 完了前の動作 |
|---|---|---|---|
| `take(n)` | 最初のn個 | 即座に出力 | n個取得後に自動完了 |
| `takeLast(n)` | 最後のn個 | 完了後にまとめて出力 | バッファに保持 |


## 💡 典型的な活用パターン

1. **ログの最新N件を取得**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs/operators';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'warn' as const, message: 'Slow query detected' },
     { timestamp: 4, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 5, level: 'info' as const, message: 'Retry successful' },
   ] as LogEntry[]);

   // 最新の3件のログを取得
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // 出力:
   // [warn] Slow query detected
   // [error] Connection failed
   // [info] Retry successful
   ```

2. **リーダーボードの上位N件を取得**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs/operators';

   interface Score {
     player: string;
     score: number;
   }

   const scores$ = from([
     { player: 'Alice', score: 100 },
     { player: 'Bob', score: 150 },
     { player: 'Charlie', score: 200 },
     { player: 'Dave', score: 180 },
     { player: 'Eve', score: 220 }
   ] as Score[]).pipe(
     // スコアでソート済みと仮定
   );

   // トップ3を取得
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // 出力: Charlie: 200, Dave: 180, Eve: 220
   ```

3. **データ処理完了後の最終N件のサマリー**
   ```ts
   import { interval } from 'rxjs';
   import { take, map, takeLast } from 'rxjs/operators';

   // センサーデータのシミュレーション
   const sensorData$ = interval(100).pipe(
     take(20),
     map(i => ({
       id: i,
       temperature: 20 + Math.random() * 10
     }))
   );

   // 最後の5件の平均温度を計算
   sensorData$.pipe(
     takeLast(5)
   ).subscribe({
     next: data => {
       console.log(`データ${data.id}: ${data.temperature.toFixed(1)}°C`);
     },
     complete: () => {
       console.log('最新5件のデータ取得完了');
     }
   });
   ```


## 🧠 実践コード例（入力履歴）

ユーザーが入力した値の最新3件を表示する例です。

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeLast } from 'rxjs/operators';

// UI要素の作成
const container = document.createElement('div');
document.body.appendChild(container);

const input = document.createElement('input');
input.placeholder = '値を入力してEnter';
container.appendChild(input);

const submitButton = document.createElement('button');
submitButton.textContent = '履歴を表示（最新3件）';
container.appendChild(submitButton);

const historyDisplay = document.createElement('div');
historyDisplay.style.marginTop = '10px';
container.appendChild(historyDisplay);

// 入力値を保持するSubject
const inputs$ = new Subject<string>();

// **重要**: takeLast の購読を先に設定しておく
inputs$.pipe(
  takeLast(3)
).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `・ ${value}`;
    historyDisplay.appendChild(item);
  },
  complete: () => {
    const note = document.createElement('div');
    note.style.marginTop = '5px';
    note.style.color = 'gray';
    note.textContent = '（ページをリロードすると再度入力できます）';
    historyDisplay.appendChild(note);

    // 入力欄とボタンを無効化
    input.disabled = true;
    submitButton.disabled = true;
  }
});

// Enter キーで入力を追加
fromEvent<KeyboardEvent>(input, 'keydown').subscribe(event => {
  if (event.key === 'Enter' && input.value.trim()) {
    inputs$.next(input.value);
    console.log(`追加: ${input.value}`);
    input.value = '';
  }
});

// ボタンクリックで完了して履歴表示
fromEvent(submitButton, 'click').subscribe(() => {
  historyDisplay.innerHTML = '<strong>履歴（最新3件）:</strong><br>';
  inputs$.complete(); // ストリームを完了 → takeLast が発火
});
```

> [!IMPORTANT]
> **重要なポイント**:
> - `takeLast(3)` の購読を**先に**設定しておく必要があります
> - ボタンクリック時に `complete()` を呼ぶと、それまでに受け取った値の最後の3件が出力されます
> - `complete()` を呼んだ**後**に `subscribe` しても値は流れません


## ⚠️ 重要な注意点

> [!WARNING]
> `takeLast` はストリームが**完了するまで待つ**ため、無限ストリームでは動作しません。また、`takeLast(n)` のnが大きいとメモリを大量に消費します。

### 1. 無限ストリームでは使用不可

`takeLast` はストリームが完了するまで待つため、無限ストリームでは動作しません。

```ts
import { interval } from 'rxjs';
import { takeLast } from 'rxjs/operators';

// ❌ 悪い例: 無限ストリームで takeLast を使用
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);
// 何も出力されない（ストリームが完了しないため）
```

**解決策**: `take` と組み合わせて有限ストリームにする

```ts
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs/operators';

// ✅ 良い例: 有限ストリームにしてから takeLast を使用
interval(1000).pipe(
  take(10),      // 最初の10個で完了
  takeLast(3)    // その中から最後の3個を取得
).subscribe(console.log);
// 出力: 7, 8, 9
```

### 2. メモリ使用量に注意

`takeLast(n)` は最後のn個をバッファに保持するため、nが大きいとメモリを消費します。

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs/operators';

// ⚠️ 注意: 大量のデータをバッファに保持
range(0, 1000000).pipe(
  takeLast(100000) // 10万件をメモリに保持
).subscribe(console.log);
```


## 🎯 last との違い

```ts
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs/operators';

const numbers$ = range(0, 10);

// last: 最後の1個のみ
numbers$.pipe(
  last()
).subscribe(console.log);
// 出力: 9

// takeLast(1): 最後の1個（配列ではなく単一値として出力）
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);
// 出力: 9

// takeLast(3): 最後の3個
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// 出力: 7, 8, 9
```

| オペレーター | 取得数 | 条件指定 | ユースケース |
|---|---|---|---|
| `last()` | 1個 | 可能 | 最後の1個または条件を満たす最後の1個 |
| `takeLast(n)` | n個 | 不可 | 最後のn個を単純に取得 |


## 📋 型安全な使い方

TypeScript でジェネリクスを活用した型安全な実装例です。

```ts
import { Observable, from } from 'rxjs';
import { takeLast } from 'rxjs/operators';

interface Transaction {
  id: string;
  amount: number;
  timestamp: Date;
  status: 'pending' | 'completed' | 'failed';
}

function getRecentTransactions(
  transactions$: Observable<Transaction>,
  count: number
): Observable<Transaction> {
  return transactions$.pipe(
    takeLast(count)
  );
}

// 使用例
const transactions$ = from([
  { id: '1', amount: 100, timestamp: new Date('2025-01-01'), status: 'completed' as const },
  { id: '2', amount: 200, timestamp: new Date('2025-01-02'), status: 'completed' as const },
  { id: '3', amount: 150, timestamp: new Date('2025-01-03'), status: 'pending' as const },
  { id: '4', amount: 300, timestamp: new Date('2025-01-04'), status: 'completed' as const },
  { id: '5', amount: 250, timestamp: new Date('2025-01-05'), status: 'failed' as const },
] as Transaction[]);

// 最新3件のトランザクションを取得
getRecentTransactions(transactions$, 3).subscribe(tx => {
  console.log(`${tx.id}: ${tx.amount}円 (${tx.status})`);
});
// 出力:
// 3: 150円 (pending)
// 4: 300円 (completed)
// 5: 250円 (failed)
```


## 🔄 skip と takeLast の組み合わせ

中間部分の値を除外して、最後のN個のみを取得できます。

```ts
import { range } from 'rxjs';
import { skip, takeLast } from 'rxjs/operators';

const numbers$ = range(0, 10); // 0から9まで

// 最初の5個をスキップして、残りの最後3個を取得
numbers$.pipe(
  skip(5),      // 0, 1, 2, 3, 4 をスキップ
  takeLast(3)   // 残りの 5, 6, 7, 8, 9 から最後の3個
).subscribe(console.log);
// 出力: 7, 8, 9
```


## 🎓 まとめ

### takeLast を使うべき場合
- ✅ ストリームの最後のN個のデータが必要な場合
- ✅ ログやトランザクションの最新N件を取得したい場合
- ✅ ストリームが完了することが保証されている場合
- ✅ データのサマリーやトップN件を表示したい場合

### take を使うべき場合
- ✅ ストリームの最初のN個のデータが必要な場合
- ✅ すぐに結果を取得したい場合
- ✅ 無限ストリームから一部を取得したい場合

### 注意点
- ⚠️ 無限ストリームでは使用不可（完了しないため）
- ⚠️ `takeLast(n)` のnが大きいとメモリを消費
- ⚠️ 出力は完了後にまとめて行われる（即座には出力されない）
- ⚠️ `take(n)` と組み合わせて有限ストリームにする必要がある場合が多い


## 🚀 次のステップ

- **[take](./take)** - 最初のN個の値を取得する方法を学ぶ
- **[last](./last)** - 最後の1個の値を取得する方法を学ぶ
- **[skip](./skip)** - 最初のN個の値をスキップする方法を学ぶ
- **[filter](./filter)** - 条件に基づいてフィルタリングする方法を学ぶ
- **[フィルタリングオペレーター実践例](./practical-use-cases)** - 実際のユースケースを学ぶ
