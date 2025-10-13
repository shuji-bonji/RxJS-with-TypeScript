---
description: findオペレーターは、条件を満たす最初の値を見つけて出力し、ストリームを完了させます。配列の検索やデータの存在確認に便利です。
---

# find - 条件を満たす最初の値を見つける

`find` オペレーターは、**条件を満たす最初の値**を見つけて出力し、即座にストリームを完了させます。値が見つからない場合は `undefined` を出力します。


## 🔰 基本構文と使い方

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// 出力: 8（最初の偶数）
```

**動作の流れ**:
1. 1, 3, 5, 7 をチェック → 条件を満たさない
2. 8 をチェック → 条件を満たす → 8 を出力して完了
3. 9, 10 は評価されない

[🌐 RxJS公式ドキュメント - `find`](https://rxjs.dev/api/operators/find)


## 🆚 first との対比

`find` と `first` は似ていますが、使い方が異なります。

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: 条件を満たす最初の値（条件はオプション）
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// 出力: 7

// find: 条件を満たす最初の値（条件は必須）
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// 出力: 7
```

| オペレーター | 条件指定 | 値が見つからない場合 | ユースケース |
|---|---|---|---|
| `first()` | オプション | エラー（`EmptyError`） | 最初の値を取得 |
| `first(predicate)` | オプション | エラー（`EmptyError`） | 条件付き取得 |
| `find(predicate)` | 必須 | `undefined` を出力 | 検索・存在確認 |


## 💡 典型的な活用パターン

1. **ユーザー検索**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface User {
     id: number;
     name: string;
     email: string;
   }

   const users$ = from([
     { id: 1, name: 'Alice', email: 'alice@example.com' },
     { id: 2, name: 'Bob', email: 'bob@example.com' },
     { id: 3, name: 'Charlie', email: 'charlie@example.com' }
   ] as User[]);

   // IDが2のユーザーを検索
   users$.pipe(
     find(user => user.id === 2)
   ).subscribe(user => {
     if (user) {
       console.log(`見つかりました: ${user.name}`);
     } else {
       console.log('ユーザーが見つかりません');
     }
   });
   // 出力: 見つかりました: Bob
   ```

2. **在庫確認**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface Product {
     id: string;
     name: string;
     stock: number;
   }

   const products$ = from([
     { id: 'A1', name: 'ノートPC', stock: 0 },
     { id: 'A2', name: 'マウス', stock: 15 },
     { id: 'A3', name: 'キーボード', stock: 8 }
   ] as Product[]);

   // 在庫切れの商品を見つける
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`在庫切れ: ${product.name}`);
     } else {
       console.log('すべて在庫あり');
     }
   });
   // 出力: 在庫切れ: ノートPC
   ```

3. **エラーログの検索**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 4, level: 'info' as const, message: 'Retry successful' }
   ] as LogEntry[]);

   // 最初のエラーを検索
   logs$.pipe(
     find(log => log.level === 'error')
   ).subscribe(log => {
     if (log) {
       console.log(`エラー検出: ${log.message} (時刻: ${log.timestamp})`);
     }
   });
   // 出力: エラー検出: Connection failed (時刻: 3)
   ```


## 🧠 実践コード例（商品検索）

在庫から特定の条件に合う商品を検索する例です。

```ts
import { from, fromEvent } from 'rxjs';
import { find } from 'rxjs';

interface Product {
  id: string;
  name: string;
  price: number;
  category: string;
}

const products: Product[] = [
  { id: 'P1', name: 'ワイヤレスマウス', price: 2980, category: 'PC周辺機器' },
  { id: 'P2', name: 'メカニカルキーボード', price: 8980, category: 'PC周辺機器' },
  { id: 'P3', name: 'USBメモリ 64GB', price: 1480, category: 'ストレージ' },
  { id: 'P4', name: 'モニター 27インチ', price: 29800, category: 'ディスプレイ' },
  { id: 'P5', name: 'ノートPCスタンド', price: 3980, category: 'PC周辺機器' }
];

// UI要素の作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = '商品検索';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = '最大価格を入力';
input.style.marginRight = '10px';
container.appendChild(input);

const searchButton = document.createElement('button');
searchButton.textContent = '検索';
container.appendChild(searchButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// 検索処理
fromEvent(searchButton, 'click').subscribe(() => {
  const maxPrice = parseInt(input.value);

  if (isNaN(maxPrice)) {
    result.textContent = '価格を入力してください';
    result.style.color = 'red';
    return;
  }

  from(products).pipe(
    find(product => product.price <= maxPrice)
  ).subscribe(product => {
    if (product) {
      result.innerHTML = `
        <strong>見つかりました！</strong><br>
        商品名: ${product.name}<br>
        価格: ¥${product.price.toLocaleString()}<br>
        カテゴリ: ${product.category}
      `;
      result.style.color = 'green';
    } else {
      result.textContent = `¥${maxPrice.toLocaleString()}以下の商品は見つかりませんでした`;
      result.style.color = 'orange';
    }
  });
});
```

このコードは、ユーザーが入力した価格以下の最初の商品を検索して表示します。


## 🎯 filter との違い

`find` と `filter` は異なる目的で使用されます。

```ts
import { from } from 'rxjs';
import { find, filter } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// filter: 条件に合う値をすべて出力
numbers$.pipe(
  filter(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter完了')
});
// 出力: 7, 8, 9, 10, filter完了

// find: 条件に合う最初の値のみ出力
numbers$.pipe(
  find(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('find完了')
});
// 出力: 7, find完了
```

| オペレーター | 出力数 | 完了タイミング | ユースケース |
|---|---|---|---|
| `filter(predicate)` | 条件に合うすべての値 | 元のストリーム完了時 | データの絞り込み |
| `find(predicate)` | 条件に合う最初の値のみ | 見つかった時点で即座に | 検索・存在確認 |


## 📋 型安全な使い方

TypeScript でジェネリクスを活用した型安全な実装例です。

```ts
import { Observable, from } from 'rxjs';
import { find } from 'rxjs';

interface Task {
  id: number;
  title: string;
  completed: boolean;
  priority: 'high' | 'medium' | 'low';
}

function findTaskById(
  tasks$: Observable<Task>,
  id: number
): Observable<Task | undefined> {
  return tasks$.pipe(
    find(task => task.id === id)
  );
}

function findFirstIncompleteTask(
  tasks$: Observable<Task>
): Observable<Task | undefined> {
  return tasks$.pipe(
    find(task => !task.completed)
  );
}

// 使用例
const tasks$ = from([
  { id: 1, title: 'タスクA', completed: true, priority: 'high' as const },
  { id: 2, title: 'タスクB', completed: false, priority: 'medium' as const },
  { id: 3, title: 'タスクC', completed: false, priority: 'low' as const }
] as Task[]);

// IDで検索
findTaskById(tasks$, 2).subscribe(task => {
  if (task) {
    console.log(`見つかりました: ${task.title}`);
  } else {
    console.log('タスクが見つかりません');
  }
});
// 出力: 見つかりました: タスクB

// 未完了タスクを検索
findFirstIncompleteTask(tasks$).subscribe(task => {
  if (task) {
    console.log(`次のタスク: ${task.title} (優先度: ${task.priority})`);
  }
});
// 出力: 次のタスク: タスクB (優先度: medium)
```


## 🔄 find と findIndex の違い

RxJSには `findIndex` オペレーターもあります。

```ts
import { from } from 'rxjs';
import { find, findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// find: 値を返す
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// 出力: 30

// findIndex: インデックスを返す
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// 出力: 2（30のインデックス）
```

| オペレーター | 返り値 | 見つからない場合 |
|---|---|---|
| `find(predicate)` | 値そのもの | `undefined` |
| `findIndex(predicate)` | インデックス（数値） | `-1` |


## ⚠️ よくある間違い

> [!NOTE]
> `find` は値が見つからない場合に `undefined` を出力します。エラーにはなりません。エラーが必要な場合は `first` を使用してください。

### 誤: 値が見つからない場合のエラー処理を期待

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ❌ 悪い例: エラーハンドリングを期待しているが呼ばれない
numbers$.pipe(
  find(n => n > 10)
).subscribe({
  next: console.log,
  error: err => console.log('エラー:', err) // 呼ばれない
});
// 出力: undefined
```

### 正: undefined のチェックまたは first を使用

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ✅ 良い例1: undefined をチェック
numbers$.pipe(
  find(n => n > 10)
).subscribe(result => {
  if (result !== undefined) {
    console.log('見つかった:', result);
  } else {
    console.log('見つかりませんでした');
  }
});
// 出力: 見つかりませんでした

// ✅ 良い例2: エラーが必要なら first を使用
numbers$.pipe(
  first(n => n > 10, 0) // デフォルト値を指定
).subscribe({
  next: console.log,
  error: err => console.log('エラー:', err.message)
});
// 出力: 0
```


## 🎓 まとめ

### find を使うべき場合
- ✅ 条件を満たす最初の値を検索したい場合
- ✅ 値の存在確認をしたい場合
- ✅ 見つからない場合に `undefined` で処理したい場合
- ✅ 配列やリストから特定の要素を探したい場合

### first を使うべき場合
- ✅ 最初の値を取得したい場合
- ✅ 値が見つからない場合にエラーを発行したい場合

### filter を使うべき場合
- ✅ 条件に合うすべての値が必要な場合
- ✅ データの絞り込みが目的の場合

### 注意点
- ⚠️ `find` は見つからない場合 `undefined` を出力（エラーではない）
- ⚠️ 条件を満たす最初の値で即座に完了する
- ⚠️ TypeScriptでは返り値が `T | undefined` 型になる


## 🚀 次のステップ

- **[first](./first)** - 最初の値を取得する方法を学ぶ
- **[filter](./filter)** - 条件に基づいてフィルタリングする方法を学ぶ
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - 条件を満たす最初の値のインデックスを取得する方法を学ぶ（公式ドキュメント）
- **[フィルタリングオペレーター実践例](./practical-use-cases)** - 実際のユースケースを学ぶ
