---
description: filterオペレーターは、指定した条件関数に基づいてストリーム内の値を選別し、条件を満たす値だけを通過させるフィルタリングオペレーターです。フォーム入力のバリデーション、特定条件のデータ抽出、nullやundefinedの除外など、不要なデータを除外してストリームを効率化します。TypeScriptの型ガード関数（型述語）として活用でき、bufferとの違いや述語関数を純粋関数にする注意点も解説します。
---

# filter - 条件に合致する値だけを通過させる

`filter` オペレーターは、指定した条件関数に基づいてストリーム内の値を選別し、条件を満たす値だけを通過させます。

## 🔰 基本構文と使い方

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(console.log);
// 出力: 2, 4, 6, 8, 10
```

- 条件に合致した値のみが通過します。
- 配列の `Array.prototype.filter()` に似た動きですが、Observable上で逐次処理されます。

[🌐 RxJS公式ドキュメント - `filter`](https://rxjs.dev/api/operators/filter)

## 💡 典型的な活用パターン

- フォーム入力値のバリデーション
- 特定の型や構造を持つデータのみを許可
- センサーイベントやストリームデータのフィルタリング

## 🧠 実践コード例（UI付き）

入力された数値が偶数だった場合のみリアルタイムでリスト表示します。

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'filter の実践例:';
document.body.appendChild(title);

// 入力フィールド作成
const input = document.createElement('input');
input.type = 'number';
input.placeholder = '数値を入力';
input.style.marginBottom = '10px';
document.body.appendChild(input);

// 出力エリア作成
const output = document.createElement('div');
document.body.appendChild(output);

// 入力イベントストリーム
fromEvent(input, 'input')
  .pipe(
    map((e) => parseInt((e.target as HTMLInputElement).value, 10)),
    filter((n) => !isNaN(n) && n % 2 === 0)
  )
  .subscribe((evenNumber) => {
    const item = document.createElement('div');
    item.textContent = `偶数検出: ${evenNumber}`;
    output.prepend(item);
  });

```

- 数値が偶数である場合のみ、出力に表示されます。
- 奇数や無効な入力は無視されます。

> [!WARNING] 本番コードでの注意
> 上記サンプルは説明の簡略化のため `fromEvent` の購読解除を省略しています。実コードでは `takeUntil(destroy$)`、`take(N)`、もしくは `Subscription.unsubscribe()` で明示的にライフサイクル管理してください。詳細: [困難点克服: ライフサイクル管理](/guide/overcoming-difficulties/lifecycle-management.md)

## 🔍 buffer との違い

| オペレーター | 動作 | 出力 |
|:---|:---|:---|
| `filter` | 条件に**合致しない**値は破棄 | 個別の値 `T` |
| `buffer` | 値を配列に**蓄積** | 配列 `T[]` |

```ts
import { interval } from 'rxjs';
import { filter, buffer, take } from 'rxjs';

const source$ = interval(1000).pipe(take(5)); // 0, 1, 2, 3, 4

// filter - 条件に合致する値のみ通過
source$.pipe(
  filter(x => x % 2 === 0)
).subscribe(x => {
  console.log('filter:', x);
  // 出力: filter: 0
  // 出力: filter: 2
  // 出力: filter: 4
});

// buffer - 値を配列として蓄積
source$.pipe(
  buffer(interval(2500))
).subscribe(arr => {
  console.log('buffer:', arr);
  // 出力: buffer: [0, 1]
  // 出力: buffer: [2, 3, 4]
});
```

## ⚠️ 注意点

### 1. 述語関数は純粋関数にする

述語関数に副作用を持たせると、ストリームの再購読時に予期しない挙動を起こす可能性があります。

```ts
// ❌ 悪い例: 副作用あり
let counter = 0;
source$.pipe(
  filter(x => {
    counter++; // 副作用
    return x > 10;
  })
).subscribe();

// ✅ 良い例: 純粋関数
source$.pipe(
  filter(x => x > 10)
).subscribe();
```

### 2. 型ガード関数としての活用

TypeScript の型述語 (`x is T`) を返すように書くと、`filter` 通過後の型を絞り込めます。

```ts
import { Observable, of, filter } from 'rxjs';

interface User {
  id: number;
  name: string;
  email?: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob' }
);

// 型ガード関数として使用
users$.pipe(
  filter((user): user is User & { email: string } => user.email !== undefined)
).subscribe(user => {
  console.log(user.email.toLowerCase()); // email は string 型として推論される
});
```

> [!TIP] 型ガードの効果
> `user is User & { email: string }` という型述語を返すことで、`filter` 後の `user` は `email` が必須プロパティになります。`user.email.toLowerCase()` のような呼び出しが型エラーなく書けるようになります。

## 📚 関連オペレーター

- [take](/guide/operators/filtering/take) - 最初のN個の値のみ取得
- [first](/guide/operators/filtering/first) - 最初の値のみ取得（条件付きも可能）
- [distinct](/guide/operators/filtering/distinct) - 重複する値を除外
- [distinctUntilChanged](/guide/operators/filtering/distinctUntilChanged) - 直前の値と同じものを除外

## まとめ

`filter` オペレーターは RxJS の最も基本的なフィルタリングツールです。

- ✅ 条件に合致した値のみ通過
- ✅ 配列の `.filter()` と同じ感覚で使える
- ✅ TypeScript の型ガードとしても使える
- ⚠️ 述語関数は純粋関数にする
- ⚠️ 名前は似ているが `buffer` とは用途が異なる（個別値 vs 配列）
