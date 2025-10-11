---
description: takeWhileオペレーターは、指定した条件を満たす間は値を取得し続け、条件がfalseになった時点でストリームを完了させます。動的な条件でストリームを制御したい場合に便利です。
---

# takeWhile - 条件を満たす間値を取得する

`takeWhile` オペレーターは、**指定した条件を満たす間**は値を取得し続け、条件が`false`になった時点でストリームを完了させます。


## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { takeWhile } from 'rxjs/operators';

const source$ = interval(1000);

source$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('完了')
});
// 出力: 0, 1, 2, 3, 4, 完了
```

**動作の流れ**:
1. 0 が発行 → `0 < 5` は `true` → 出力
2. 1 が発行 → `1 < 5` は `true` → 出力
3. 2 が発行 → `2 < 5` は `true` → 出力
4. 3 が発行 → `3 < 5` は `true` → 出力
5. 4 が発行 → `4 < 5` は `true` → 出力
6. 5 が発行 → `5 < 5` は `false` → 完了（5は出力されない）

[🌐 RxJS公式ドキュメント - `takeWhile`](https://rxjs.dev/api/operators/takeWhile)


## 🆚 take との対比

`take` と `takeWhile` は取得条件が異なります。

```ts
import { interval } from 'rxjs';
import { take, takeWhile } from 'rxjs/operators';

const source$ = interval(1000);

// take: 個数で制御
source$.pipe(
  take(5)
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4

// takeWhile: 条件で制御
source$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4
```

| オペレーター | 制御方法 | 完了条件 | 最後の値 |
|---|---|---|---|
| `take(n)` | 個数 | n個取得後 | n番目の値を含む |
| `takeWhile(predicate)` | 条件関数 | 条件が`false`になった時 | `false`になった値は含まない* |

\* デフォルトでは`false`になった値は出力されませんが、`inclusive: true`オプションで含めることができます


## 🎯 inclusive オプション

条件が`false`になった値も含めたい場合は、`inclusive: true`を指定します。

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs/operators';

const numbers$ = range(0, 10);

// デフォルト（inclusive: false）
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4

// inclusive: true
numbers$.pipe(
  takeWhile(n => n < 5, true)
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4, 5（条件がfalseになった5も含む）
```


## 💡 典型的な活用パターン

1. **閾値までのデータ取得**
   ```ts
   import { interval } from 'rxjs';
   import { takeWhile, map } from 'rxjs/operators';

   // 温度センサーのシミュレーション
   const temperature$ = interval(100).pipe(
     map(() => 20 + Math.random() * 15)
   );

   // 30度未満の間だけ記録
   temperature$.pipe(
     takeWhile(temp => temp < 30)
   ).subscribe({
     next: temp => console.log(`温度: ${temp.toFixed(1)}°C`),
     complete: () => console.log('警告: 温度が30度を超えました！')
   });
   ```

2. **配列の条件付き処理**
   ```ts
   import { from } from 'rxjs';
   import { takeWhile } from 'rxjs/operators';

   interface Task {
     id: number;
     priority: 'high' | 'medium' | 'low';
     completed: boolean;
   }

   const tasks$ = from([
     { id: 1, priority: 'high' as const, completed: false },
     { id: 2, priority: 'high' as const, completed: false },
     { id: 3, priority: 'medium' as const, completed: false },
     { id: 4, priority: 'low' as const, completed: false },
   ] as Task[]);

   // 優先度がhighの間だけ処理
   tasks$.pipe(
     takeWhile(task => task.priority === 'high')
   ).subscribe(task => {
     console.log(`タスク${task.id}を処理中`);
   });
   // 出力: タスク1を処理中, タスク2を処理中
   ```

3. **ページング処理**
   ```ts
   import { range } from 'rxjs';
   import { takeWhile, map } from 'rxjs/operators';

   interface Page {
     pageNumber: number;
     hasMore: boolean;
   }

   const pages$ = range(1, 10).pipe(
     map(pageNum => ({
       pageNumber: pageNum,
       hasMore: pageNum < 5
     } as Page))
   );

   // hasMoreがtrueの間だけページを読み込む
   pages$.pipe(
     takeWhile(page => page.hasMore, true) // inclusive: true
   ).subscribe(page => {
     console.log(`ページ${page.pageNumber}を読み込み`);
   });
   // 出力: ページ1〜5を読み込み
   ```


## 🧠 実践コード例（カウントアップ制限）

特定の条件に達するまでカウントアップを続ける例です。

```ts
import { fromEvent, interval } from 'rxjs';
import { takeWhile, scan, switchMap } from 'rxjs/operators';

// UI要素の作成
const container = document.createElement('div');
document.body.appendChild(container);

const startButton = document.createElement('button');
startButton.textContent = 'カウント開始';
container.appendChild(startButton);

const counter = document.createElement('div');
counter.style.fontSize = '24px';
counter.style.marginTop = '10px';
counter.textContent = 'カウント: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = '10未満の間カウントし続けます';
container.appendChild(message);

// ボタンクリックでカウント開始
fromEvent(startButton, 'click').pipe(
  switchMap(() =>
    interval(500).pipe(
      scan(count => count + 1, 0),
      takeWhile(count => count < 10)
    )
  )
).subscribe({
  next: (count) => {
    counter.textContent = `カウント: ${count}`;
    startButton.disabled = true;
  },
  complete: () => {
    message.textContent = '10に達したので完了しました！';
    message.style.color = 'green';
    startButton.disabled = false;
  }
});
```

このコードは、0から9までカウントアップし、10に達する直前で自動的に完了します。


## 🎯 skipWhile との対比

`takeWhile` と `skipWhile` は対照的な動作をします。

```ts
import { range } from 'rxjs';
import { takeWhile, skipWhile } from 'rxjs/operators';

const numbers$ = range(0, 10);

// takeWhile: 条件を満たす間は取得
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4

// skipWhile: 条件を満たす間はスキップ
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// 出力: 5, 6, 7, 8, 9
```

| オペレーター | 動作 | 完了タイミング |
|---|---|---|
| `takeWhile(predicate)` | 条件を満たす間**取得** | 条件が`false`になった時 |
| `skipWhile(predicate)` | 条件を満たす間**スキップ** | 元のストリームの完了時 |


## 📋 型安全な使い方

TypeScript でジェネリクスを活用した型安全な実装例です。

```ts
import { Observable, from } from 'rxjs';
import { takeWhile } from 'rxjs/operators';

interface SensorReading {
  timestamp: Date;
  value: number;
  unit: string;
  status: 'normal' | 'warning' | 'critical';
}

function getReadingsUntilWarning(
  readings$: Observable<SensorReading>
): Observable<SensorReading> {
  return readings$.pipe(
    takeWhile(reading => reading.status === 'normal')
  );
}

// 使用例
const readings$ = from([
  { timestamp: new Date(), value: 25, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 28, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 32, unit: '°C', status: 'warning' as const },
  { timestamp: new Date(), value: 35, unit: '°C', status: 'critical' as const },
] as SensorReading[]);

getReadingsUntilWarning(readings$).subscribe(reading => {
  console.log(`${reading.value}${reading.unit} - ${reading.status}`);
});
// 出力:
// 25°C - normal
// 28°C - normal
```


## 🔄 takeWhile と filter の違い

`takeWhile` は完了する点が `filter` と異なります。

```ts
import { range } from 'rxjs';
import { takeWhile, filter } from 'rxjs/operators';

const numbers$ = range(0, 10);

// filter: 条件に合う値のみ通過（ストリームは続く）
numbers$.pipe(
  filter(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter完了')
});
// 出力: 0, 1, 2, 3, 4, filter完了

// takeWhile: 条件を満たす間のみ（条件がfalseで完了）
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('takeWhile完了')
});
// 出力: 0, 1, 2, 3, 4, takeWhile完了
```

| オペレーター | 動作 | ストリーム完了 |
|---|---|---|
| `filter(predicate)` | 条件に合う値のみ通過 | 元のストリーム完了時 |
| `takeWhile(predicate)` | 条件を満たす間取得 | 条件が`false`になった時 |


## ⚠️ よくある間違い

### 誤: 条件が最初から false

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs/operators';

// ❌ 悪い例: 最初の値で条件がfalse
range(5, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// 何も出力されない（最初の値5で条件がfalse）
```

### 正: 条件を確認する

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs/operators';

// ✅ 良い例: 条件を適切に設定
range(0, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4
```


## 🎓 まとめ

### takeWhile を使うべき場合
- ✅ 動的な条件でストリームを制御したい場合
- ✅ 閾値に達するまでデータを取得したい場合
- ✅ 特定の状態が続く間だけ処理したい場合
- ✅ 条件ベースの早期完了が必要な場合

### take を使うべき場合
- ✅ 取得する個数が決まっている場合
- ✅ シンプルな個数制限が必要な場合

### filter を使うべき場合
- ✅ ストリーム全体から条件に合う値のみを抽出したい場合
- ✅ ストリームを完了させたくない場合

### 注意点
- ⚠️ 条件が最初から`false`の場合、何も出力されずに完了する
- ⚠️ デフォルトでは条件が`false`になった値は出力されない（`inclusive: true`で含められる）
- ⚠️ 無限ストリームで条件が常に`true`の場合、永遠に続く


## 🚀 次のステップ

- **[take](./take)** - 最初のN個の値を取得する方法を学ぶ
- **[takeLast](./takeLast)** - 最後のN個の値を取得する方法を学ぶ
- **[takeUntil](../utility/takeUntil)** - 別のObservableが発火するまで値を取得する方法を学ぶ
- **[filter](./filter)** - 条件に基づいてフィルタリングする方法を学ぶ
- **[フィルタリングオペレーター実践例](./practical-use-cases)** - 実際のユースケースを学ぶ
