---
description: pluckオペレーターは、オブジェクトストリーム内から特定のプロパティ値だけを取り出す変換演算子です。複雑なオブジェクト構造から必要なフィールドだけを簡単に抽出でき、APIレスポンスやイベントオブジェクトからの情報取得に便利です。
---

# pluck - オブジェクトから特定プロパティを抽出する

::: warning ⚠️ 非推奨（Deprecated）
`pluck`オペレーターは**RxJS 7で非推奨**となり、**RxJS 8で削除されました**。
代わりに`map`オペレーターを使用してください。

```typescript
// ❌ 非推奨（RxJS 8で削除済み）
.pipe(pluck('name'))

// ✅ 推奨
.pipe(map(obj => obj.name))
```
:::

`pluck`オペレーターは、オブジェクトストリーム内から特定のプロパティだけを取り出すために使用します。
複雑なオブジェクトの中から必要な値だけを簡単に取り出せる便利な変換オペレーターでしたが、現在は`map`の使用が推奨されています。


## 🔰 基本構文と使い方

### 非推奨の書き方（pluck）

```ts
import { from } from 'rxjs';
import { pluck } from 'rxjs/operators';

const users$ = from([
  { id: 1, name: '田中', age: 28 },
  { id: 2, name: '佐藤', age: 34 },
  { id: 3, name: '鈴木', age: 22 }
]);

users$.pipe(
  pluck('name')
).subscribe(console.log);
// 出力: 田中, 佐藤, 鈴木
```

### ✅ 推奨の書き方（map）

```ts
import { from } from 'rxjs';
import { map } from 'rxjs/operators';

const users$ = from([
  { id: 1, name: '田中', age: 28 },
  { id: 2, name: '佐藤', age: 34 },
  { id: 3, name: '鈴木', age: 22 }
]);

users$.pipe(
  map(user => user.name)
).subscribe(console.log);
// 出力: 田中, 佐藤, 鈴木
```

各オブジェクトから `name` プロパティのみを抽出して出力します。`map`を使うことで型安全性も保たれます。

[🌐 RxJS公式ドキュメント - `pluck`](https://rxjs.dev/api/operators/pluck)

## 💡 典型的な活用パターン

- APIレスポンスから特定のフィールドだけを抽出
- イベントオブジェクトから必要なプロパティを取得
- 入れ子になったオブジェクト構造からネストされた値を取り出す

## 🧠 実践コード例（UI付き）

マウスクリック位置（x座標）だけをリアルタイムで取得して表示する例です。

### ❌ 非推奨の書き方（pluck）

```ts
import { fromEvent } from 'rxjs';
import { pluck } from 'rxjs/operators';

// 出力エリア作成
const output = document.createElement('div');
output.style.marginTop = '20px';
document.body.appendChild(output);

// クリックイベントストリーム
fromEvent<MouseEvent>(document, 'click')
  .pipe(pluck('clientX'))
  .subscribe((x) => {
    output.textContent = `クリック位置 (X座標): ${x}`;
  });
```

### ✅ 推奨の書き方（map）

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs/operators';

// 出力エリア作成
const output = document.createElement('div');
output.style.marginTop = '20px';
document.body.appendChild(output);

// クリックイベントストリーム
fromEvent<MouseEvent>(document, 'click')
  .pipe(map(event => event.clientX))
  .subscribe((x) => {
    output.textContent = `クリック位置 (X座標): ${x}`;
  });
```

- クリックされた位置のX座標だけを抽出してリアルタイム表示します。
- `map`を使うことで、TypeScriptの型推論が効き、より安全なコードになります。

## 🔄 移行ガイド

### ネストされたプロパティの場合

```typescript
// ❌ 非推奨: pluck('user', 'profile', 'name')
.pipe(pluck('user', 'profile', 'name'))

// ✅ 推奨: map
.pipe(map(obj => obj.user?.profile?.name))
```

### 複数プロパティの抽出

```typescript
// ❌ 非推奨
.pipe(pluck('name'), pluck('length'))

// ✅ 推奨
.pipe(
  map(obj => obj.name),
  map(name => name.length)
)
```
