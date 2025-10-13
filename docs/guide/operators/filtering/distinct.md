---
description: distinctオペレーターは、すべての重複値を除去し、一度も出力されていないユニークな値のみを出力します。内部的にSetを使用して既出の値を記憶するため、無限ストリームでは注意が必要です。
---

# distinct - すべての重複値を除去する

`distinct` オペレーターは、Observable から発行されるすべての値を監視し、**過去に一度も出力されていない値のみ**を出力します。内部的に Set を使用して既出の値を記憶します。


## 🔰 基本構文と使い方

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

const numbers$ = of(1, 2, 1, 3, 2, 4, 1, 5);

numbers$.pipe(
  distinct()
).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5
```

- ストリーム全体で重複を除去します
- 一度出力された値は、その後何度出現しても無視されます
- `distinctUntilChanged` は**連続した**重複のみを除去しますが、`distinct` は**すべての**重複を除去します

[🌐 RxJS公式ドキュメント - `distinct`](https://rxjs.dev/api/operators/distinct)


## 🆚 distinctUntilChanged との違い

```ts
import { of } from 'rxjs';
import { distinct, distinctUntilChanged } from 'rxjs';

const values$ = of(1, 2, 1, 2, 3, 1, 2, 3);

// distinctUntilChanged: 連続した重複のみ除去
values$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// 出力: 1, 2, 1, 2, 3, 1, 2, 3

// distinct: すべての重複を除去
values$.pipe(
  distinct()
).subscribe(console.log);
// 出力: 1, 2, 3
```

| オペレーター | 除去対象 | ユースケース |
|---|---|---|
| `distinctUntilChanged` | 連続した重複のみ | 入力フィールド、センサーデータ |
| `distinct` | すべての重複 | ユニークな値のリスト、ID一覧 |


## 🎯 keySelector による比較カスタマイズ

オブジェクトの特定のプロパティで重複判定を行う場合、`keySelector` 関数を使用します。

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const users$ = of(
  { id: 1, name: 'Alice' } as User,
  { id: 2, name: 'Bob' } as User,
  { id: 1, name: 'Alice (updated)' } as User, // 同じID
  { id: 3, name: 'Charlie' } as User
);

users$.pipe(
  distinct(user => user.id) // IDで重複判定
).subscribe(console.log);
// 出力:
// { id: 1, name: 'Alice' }
// { id: 2, name: 'Bob' }
// { id: 3, name: 'Charlie' }
```


## 💡 典型的な活用パターン

1. **ユニークなID一覧の取得**
   ```ts
   import { from } from 'rxjs';
   import { distinct, map } from 'rxjs';

   interface Order {
     orderId: string;
     userId: number;
     amount: number;
   }

   const orders$ = from([
     { orderId: 'A1', userId: 1, amount: 100 },
     { orderId: 'A2', userId: 2, amount: 200 },
     { orderId: 'A3', userId: 1, amount: 150 },
     { orderId: 'A4', userId: 3, amount: 300 }
   ] as Order[]);

   // ユニークなユーザーIDのみを取得
   orders$.pipe(
     map(order => order.userId),
     distinct()
   ).subscribe(userId => {
     console.log(`User ID: ${userId}`);
   });
   // 出力: 1, 2, 3
   ```

2. **イベントログから固有のイベントタイプを抽出**
   ```ts
   import { fromEvent, merge } from 'rxjs';
   import { map, distinct, take } from 'rxjs';

   // UI要素を動的に作成
   const container = document.createElement('div');
   document.body.appendChild(container);

   const button1 = document.createElement('button');
   button1.textContent = 'Button 1';
   container.appendChild(button1);

   const button2 = document.createElement('button');
   button2.textContent = 'Button 2';
   container.appendChild(button2);

   const input = document.createElement('input');
   input.placeholder = '入力してください';
   container.appendChild(input);

   const log = document.createElement('div');
   log.style.marginTop = '10px';
   container.appendChild(log);

   // 複数のイベントストリームをマージして固有のイベントタイプを抽出
   const events$ = merge(
     fromEvent(button1, 'click').pipe(map(() => 'button1-click')),
     fromEvent(button2, 'click').pipe(map(() => 'button2-click')),
     fromEvent(input, 'input').pipe(map(() => 'input-change'))
   );

   events$.pipe(
     distinct(),
     take(3) // 3種類のイベントが揃ったら完了
   ).subscribe({
     next: (eventType) => {
       log.textContent += `Unique event: ${eventType}\n`;
       console.log(`Unique event: ${eventType}`);
     },
     complete: () => {
       log.textContent += 'すべてのイベントタイプを検出しました';
     }
   });
   ```


## 🧠 実践コード例（タグ入力）

ユーザーが入力したタグから重複を自動的に除去するUIの例です。

```ts
import { fromEvent, Subject } from 'rxjs';
import { map, distinct, scan } from 'rxjs';

// UI要素の作成
const container = document.createElement('div');
document.body.appendChild(container);

const tagInput = document.createElement('input');
tagInput.type = 'text';
tagInput.placeholder = 'タグを入力してEnter';
container.appendChild(tagInput);

const tagList = document.createElement('ul');
tagList.style.marginTop = '10px';
container.appendChild(tagList);

// タグ追加ストリーム
const tagSubject$ = new Subject<string>();

tagSubject$.pipe(
  map(tag => tag.trim().toLowerCase()),
  distinct() // 重複タグを除去
).subscribe(tag => {
  const li = document.createElement('li');
  li.textContent = tag;
  tagList.appendChild(li);
});

// Enter キーでタグを追加
fromEvent<KeyboardEvent>(tagInput, 'keydown').subscribe(event => {
  if (event.key === 'Enter') {
    const value = tagInput.value.trim();
    if (value) {
      tagSubject$.next(value);
      tagInput.value = '';
    }
  }
});
```

このコードは、同じタグを複数回入力しても、一度だけリストに追加されることを保証します。


## ⚠️ メモリ使用に関する注意

> [!WARNING]
> `distinct` オペレーターは、内部的に **Set** を使用してすべての既出値を記憶します。無限ストリームで使用すると、メモリリークの原因となる可能性があります。

### 問題: 無限ストリームでのメモリリーク

```ts
import { interval } from 'rxjs';
import { distinct, map } from 'rxjs';

// ❌ 悪い例: 無限ストリームで distinct を使用
interval(100).pipe(
  map(n => n % 10), // 0-9 のサイクル
  distinct() // 最初の 10 個だけ出力後、メモリに記憶し続ける
).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// その後は何も出力されないが、Setは保持され続ける
```

### 解決策: flushes パラメータでSetをクリア

```ts
import { interval, timer } from 'rxjs';
import { distinct, map } from 'rxjs';

// ✅ 良い例: 定期的にSetをクリア
interval(100).pipe(
  map(n => n % 5),
  distinct(
    value => value,
    timer(1000) // 1秒ごとにSetをクリア
  )
).subscribe(console.log);
// 1秒ごとに 0, 1, 2, 3, 4 が再出力される
```

### ベストプラクティス

1. **有限ストリームで使用する**: HTTP レスポンス、配列からの変換など
2. **flushes を使用する**: 無限ストリームの場合は定期的にクリア
3. **distinctUntilChanged を検討**: 連続した重複のみを除去する場合はこちらを使用


## 📋 型安全な使い方

TypeScript でジェネリクスを活用した型安全な実装例です。

```ts
import { Observable } from 'rxjs';
import { distinct, map } from 'rxjs';

interface Product {
  id: number;
  name: string;
  categoryId: number;
}

function getUniqueCategories(
  products$: Observable<Product>
): Observable<number> {
  return products$.pipe(
    distinct(product => product.categoryId)
  ).pipe(
    map(product => product.categoryId)
  );
}

// 使用例
import { of } from 'rxjs';

const products$ = of(
  { id: 1, name: 'Laptop', categoryId: 10 } as Product,
  { id: 2, name: 'Mouse', categoryId: 10 } as Product,
  { id: 3, name: 'Book', categoryId: 20 } as Product
);

getUniqueCategories(products$).subscribe(categoryId => {
  console.log(`Category ID: ${categoryId}`);
});
// 出力: 10, 20
```


## 🎓 まとめ

### distinct を使うべき場合
- ✅ ユニークな値のリストが必要な場合
- ✅ 有限のストリームで重複を除去したい場合
- ✅ ID一覧やカテゴリ一覧の作成

### distinctUntilChanged を使うべき場合
- ✅ 連続した重複のみを除去したい場合
- ✅ 入力フィールドの変更検知
- ✅ 無限ストリームでメモリを節約したい場合

### 注意点
- ⚠️ 無限ストリームでは `flushes` パラメータを使用してメモリリークを防ぐ
- ⚠️ 大量のユニークな値が流れる場合はメモリ使用量に注意
- ⚠️ パフォーマンスが重要な場合は、Set のサイズを監視する


## 🚀 次のステップ

- **[distinctUntilChanged](./distinctUntilChanged)** - 連続した重複のみを除去する方法を学ぶ
- **[distinctUntilKeyChanged](./distinctUntilKeyChanged)** - オブジェクトのキーで比較する方法を学ぶ
- **[filter](./filter)** - 条件に基づいてフィルタリングする方法を学ぶ
- **[フィルタリングオペレーター実践例](./practical-use-cases)** - 実際のユースケースを学ぶ
