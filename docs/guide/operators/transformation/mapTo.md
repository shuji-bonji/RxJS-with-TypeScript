---
description: mapToオペレーターは、ストリーム内のすべての値を特定の固定値に変換し、ユーザー操作の検知や単純なトリガー処理に利用されます。
---

# mapTo - 常に固定の値を出力する

::: warning ⚠️ 非推奨（Deprecated）
`mapTo`オペレーターは**RxJS 7で非推奨**となり、**RxJS 9で削除される予定**です。
代わりに`map(() => value)`の形式を使用してください。

```typescript
// ❌ 非推奨
.pipe(mapTo('クリックされました！'))

// ✅ 推奨
.pipe(map(() => 'クリックされました！'))
```
:::

`mapTo`オペレーターは、入力ストリームに流れる値に関係なく、**常に同じ固定値を出力**します。
クリックやイベントストリームなど、発生を検知するだけで十分な場面でよく使われましたが、現在は`map`の使用が推奨されています。


## 🔰 基本構文と使い方

### ❌ 非推奨の書き方（mapTo）

```ts
import { fromEvent } from 'rxjs';
import { mapTo } from 'rxjs/operators';

const clicks$ = fromEvent(document, 'click');

clicks$
  .pipe(
    mapTo('クリックされました！')
  )
  .subscribe(console.log);

// 出力 (クリックごとに):
// クリックされました！
// クリックされました！
// ...
```

### ✅ 推奨の書き方（map）

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs/operators';

const clicks$ = fromEvent(document, 'click');

clicks$
  .pipe(
    map(() => 'クリックされました！')
  )
  .subscribe(console.log);

// 出力 (クリックごとに):
// クリックされました！
// クリックされました！
// ...
```

入力されたイベントの中身は無視され、常に「クリックされました！」という文字列だけが出力されます。

[🌐 RxJS公式ドキュメント - `mapTo`](https://rxjs.dev/api/operators/mapTo)

## 💡 典型的な活用パターン

- ユーザー操作を検知して、単一のアクションを発生させる
- イベント検知時に固定メッセージや通知を送信する
- インジケーターのON/OFFトリガーを簡素化する


## 🧠 実践コード例（UI付き）

ボタンをクリックすると、常に「イベント検知！」というメッセージを表示します。

### ❌ 非推奨の書き方（mapTo）

```ts
import { fromEvent } from 'rxjs';
import { mapTo } from 'rxjs/operators';

// ボタン作成
const button = document.createElement('button');
button.textContent = 'クリック';
document.body.appendChild(button);

// 出力領域
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// クリックイベントを検知
fromEvent(button, 'click')
  .pipe(mapTo('イベント検知！'))
  .subscribe((message) => {
    output.textContent = message;
    output.style.color = 'red';
    setTimeout(() => {
      output.style.color = 'black';
    }, 300);
  });
```

### ✅ 推奨の書き方（map）

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs/operators';

// ボタン作成
const button = document.createElement('button');
button.textContent = 'クリック';
document.body.appendChild(button);

// 出力領域
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// クリックイベントを検知
fromEvent(button, 'click')
  .pipe(map(() => 'イベント検知！'))
  .subscribe((message) => {
    output.textContent = message;
    output.style.color = 'red';
    setTimeout(() => {
      output.style.color = 'black';
    }, 300);
  });
```

- クリックごとに出力領域に「イベント検知！」と表示されます。
- 入力の内容は一切気にせず、単に**イベントが発生したことだけをトリガー**にしています。

## 🔄 移行ガイド

### 様々な値の場合

```typescript
// ❌ 非推奨: 固定値を返す
.pipe(mapTo(42))
.pipe(mapTo(true))
.pipe(mapTo({ status: 'success' }))

// ✅ 推奨: map(() => value)
.pipe(map(() => 42))
.pipe(map(() => true))
.pipe(map(() => ({ status: 'success' })))
```

### 型安全性の向上

```typescript
import { fromEvent } from 'rxjs';
import { map } from 'rxjs/operators';

// ボタン作成
const button = document.createElement('button');
button.textContent = 'クリック';
document.body.appendChild(button);

// 出力領域
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

interface ApiResponse {
  status: 'success' | 'error';
  timestamp: number;
}

// ✅ mapを使うことで型推論が効く
fromEvent(button, 'click')
  .pipe(
    map((): ApiResponse => ({
      status: 'success',
      timestamp: Date.now()
    }))
  )
  .subscribe(response => {
    // responseの型が正確に推論される
    console.log(response.status);
  });
```

