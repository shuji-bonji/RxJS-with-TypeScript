---
description: filterオペレーターは、指定した条件関数に基づいてストリーム内の値を選別し、条件を満たす値だけを通過させるフィルタリングオペレーターです。フォーム入力のバリデーション、特定条件のデータ抽出、nullやundefinedの除外など、不要なデータを除外してストリームを効率化します。TypeScriptの型ガードとしても活用できます。
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
