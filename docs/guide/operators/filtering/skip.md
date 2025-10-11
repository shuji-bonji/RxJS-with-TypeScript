---
description: skipオペレーターは、Observableストリームから最初の指定された数の値をスキップし、それ以降の値のみを出力します。初期データを無視したい場合や、ウォームアップ期間をスキップする際に便利です。
---

# skip - 最初のN個の値をスキップする

`skip` オペレーターは、ストリームから**最初の指定された数**の値をスキップし、それ以降の値のみを出力します。


## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs/operators';

const source$ = interval(1000);

source$.pipe(
  skip(3)
).subscribe(console.log);
// 出力: 3, 4, 5, 6, 7, ...
```

- 最初の3件（0, 1, 2）をスキップします
- 4件目以降（3, 4, 5, ...）がすべて出力されます
- ストリームは元の完了タイミングで完了します

[🌐 RxJS公式ドキュメント - `skip`](https://rxjs.dev/api/operators/skip)


## 🆚 take との対比

`skip` と `take` は対照的な動作をします。

```ts
import { range } from 'rxjs';
import { skip, take } from 'rxjs/operators';

const numbers$ = range(0, 10); // 0から9まで

// take: 最初のN個を取得
numbers$.pipe(
  take(3)
).subscribe(console.log);
// 出力: 0, 1, 2

// skip: 最初のN個をスキップ
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// 出力: 3, 4, 5, 6, 7, 8, 9

// 組み合わせ: 最初の3個をスキップして、次の3個を取得
numbers$.pipe(
  skip(3),
  take(3)
).subscribe(console.log);
// 出力: 3, 4, 5
```

| オペレーター | 動作 | 完了タイミング |
|---|---|---|
| `take(n)` | 最初のn個を取得 | n個取得後に自動完了 |
| `skip(n)` | 最初のn個をスキップ | 元のストリームの完了時 |


## 💡 典型的な活用パターン

1. **初期値のスキップ**
   ```ts
   import { BehaviorSubject } from 'rxjs';
   import { skip } from 'rxjs/operators';

   const state$ = new BehaviorSubject<number>(0);

   // 初期値をスキップして、変更のみを監視
   state$.pipe(
     skip(1)
   ).subscribe(value => {
     console.log(`状態が変更されました: ${value}`);
   });

   state$.next(1); // 出力: 状態が変更されました: 1
   state$.next(2); // 出力: 状態が変更されました: 2
   ```

2. **ウォームアップ期間のスキップ**
   ```ts
   import { interval } from 'rxjs';
   import { skip, map } from 'rxjs/operators';

   // センサーデータのシミュレーション
   const sensorData$ = interval(100).pipe(
     map(() => Math.random() * 100)
   );

   // 最初の10件（1秒間）はキャリブレーション期間としてスキップ
   sensorData$.pipe(
     skip(10)
   ).subscribe(data => {
     console.log(`センサー値: ${data.toFixed(2)}`);
   });
   ```

3. **ページネーション**
   ```ts
   import { from } from 'rxjs';
   import { skip, take } from 'rxjs/operators';

   interface Item {
     id: number;
     name: string;
   }

   const allItems$ = from([
     { id: 1, name: 'Item 1' },
     { id: 2, name: 'Item 2' },
     { id: 3, name: 'Item 3' },
     { id: 4, name: 'Item 4' },
     { id: 5, name: 'Item 5' },
     { id: 6, name: 'Item 6' },
   ] as Item[]);

   const pageSize = 2;
   const pageNumber = 2; // 0-indexed

   // ページ2のアイテムを取得（アイテム5と6）
   allItems$.pipe(
     skip(pageNumber * pageSize),
     take(pageSize)
   ).subscribe(item => {
     console.log(item);
   });
   // 出力: { id: 5, name: 'Item 5' }, { id: 6, name: 'Item 6' }
   ```


## 🧠 実践コード例（カウンター）

最初の3回のクリックをスキップして、4回目以降のクリックのみカウントする例です。

```ts
import { fromEvent } from 'rxjs';
import { skip, scan } from 'rxjs/operators';

// UI要素の作成
const container = document.createElement('div');
document.body.appendChild(container);

const button = document.createElement('button');
button.textContent = 'クリック';
container.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'カウント: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = '最初の3回のクリックはスキップされます';
container.appendChild(message);

// クリックイベント
fromEvent(button, 'click').pipe(
  skip(3), // 最初の3回をスキップ
  scan((count) => count + 1, 0)
).subscribe(count => {
  counter.textContent = `カウント: ${count}`;
  if (count === 1) {
    message.textContent = '4回目のクリックからカウント開始！';
    message.style.color = 'green';
  }
});
```

このコードは、最初の3回のクリックを無視し、4回目のクリックから「1」としてカウントを開始します。


## 🎯 skip と skipWhile の違い

```ts
import { of } from 'rxjs';
import { skip, skipWhile } from 'rxjs/operators';

const numbers$ = of(1, 2, 3, 4, 5, 6);

// skip: 最初のN個を数で指定
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// 出力: 4, 5, 6

// skipWhile: 条件を満たす間スキップ
numbers$.pipe(
  skipWhile(n => n < 4)
).subscribe(console.log);
// 出力: 4, 5, 6
```

| オペレーター | スキップ条件 | ユースケース |
|---|---|---|
| `skip(n)` | 最初のn個を数でスキップ | 固定数のスキップ |
| `skipWhile(predicate)` | 条件を満たす間スキップ | 条件ベースのスキップ |
| `skipUntil(notifier$)` | 別のObservableが発火するまでスキップ | 時間ベースのスキップ |


## 📋 型安全な使い方

TypeScript でジェネリクスを活用した型安全な実装例です。

```ts
import { Observable, from } from 'rxjs';
import { skip, take } from 'rxjs/operators';

interface User {
  id: number;
  name: string;
  role: 'admin' | 'user';
}

function getPaginatedUsers(
  users$: Observable<User>,
  page: number,
  pageSize: number
): Observable<User> {
  return users$.pipe(
    skip(page * pageSize),
    take(pageSize)
  );
}

// 使用例
const users$ = from([
  { id: 1, name: 'Alice', role: 'admin' as const },
  { id: 2, name: 'Bob', role: 'user' as const },
  { id: 3, name: 'Charlie', role: 'user' as const },
  { id: 4, name: 'Dave', role: 'admin' as const },
  { id: 5, name: 'Eve', role: 'user' as const },
] as User[]);

// ページ1（2番目のページ、0-indexed）を取得
getPaginatedUsers(users$, 1, 2).subscribe(user => {
  console.log(`${user.name} (${user.role})`);
});
// 出力: Charlie (user), Dave (admin)
```


## ⚠️ よくある間違い

> [!NOTE]
> `skip` は最初のN個をスキップするだけで、ストリームを完了させません。無限ストリームでは `take` と組み合わせて終了条件を設定してください。

### 誤: 無限ストリームで skip のみを使用

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs/operators';

// ❌ 悪い例: 無限ストリームがそのまま続く
interval(1000).pipe(
  skip(5)
).subscribe(console.log);
// 5, 6, 7, 8, ... 永遠に続く
```

### 正: take と組み合わせて終了条件を設定

```ts
import { interval } from 'rxjs';
import { skip, take } from 'rxjs/operators';

// ✅ 良い例: スキップ後に取得数を制限
interval(1000).pipe(
  skip(5),
  take(3)
).subscribe({
  next: console.log,
  complete: () => console.log('完了')
});
// 5, 6, 7, 完了
```


## 🎓 まとめ

### skip を使うべき場合
- ✅ 初期値や最初のN個のデータを無視したい場合
- ✅ BehaviorSubject の初期値をスキップしたい場合
- ✅ ページネーションで特定ページのデータを取得したい場合
- ✅ センサーのキャリブレーション期間をスキップしたい場合

### take と組み合わせる場合
- ✅ 特定の範囲のデータのみを取得したい場合
- ✅ 無限ストリームから中間部分のデータを取得したい場合

### 注意点
- ⚠️ 無限ストリームでは `take` と組み合わせて終了条件を設定する
- ⚠️ `skip(0)` は元のストリームと同じ動作（何もスキップしない）
- ⚠️ スキップ数が総データ数より多い場合、何も出力されずに完了する


## 🚀 次のステップ

- **[take](./take)** - 最初のN個の値を取得する方法を学ぶ
- **[first](./first)** - 最初の値または条件を満たす最初の値を取得する方法を学ぶ
- **[last](./last)** - 最後の値を取得する方法を学ぶ
- **[filter](./filter)** - 条件に基づいてフィルタリングする方法を学ぶ
- **[フィルタリングオペレーター実践例](./practical-use-cases)** - 実際のユースケースを学ぶ
