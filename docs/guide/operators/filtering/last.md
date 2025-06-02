---
description: lastオペレーターは、ストリームの完了時に最後の値、または条件に一致する最後の値だけを取り出すためのRxJS演算子です。
---

# last - 最後の値、または条件を満たす最後の値を取得する

`last` オペレーターは、ストリームから**最後の値**、または**条件を満たす最後の値**を取得し、ストリームを完了させます。


## 🔰 基本構文と使い方

```ts
import { from } from 'rxjs';
import { last } from 'rxjs/operators';

const numbers$ = from([1, 2, 3, 4, 5]);

// 最後の値だけを取得
numbers$.pipe(
  last()
).subscribe(console.log);

// 条件を満たす最後の値だけを取得
numbers$.pipe(
  last(n => n < 5)
).subscribe(console.log);

// 出力:
// 
// 4
```

- `last()` は、ストリーム完了時に**最後に発行された値**を出力します。
- 条件を渡すと、**条件を満たす最後の値**のみ取得できます。
- 条件に合う値が存在しない場合、エラーが発生します。

[🌐 RxJS公式ドキュメント - `last`](https://rxjs.dev/api/operators/last)


## 💡 典型的な活用パターン

- フィルタリングされたデータの最後の要素を取得
- ストリーム完了時の最新状態を取得
- セッションや操作ログの最後の重要な操作を取り出す


## 🧠 実践コード例（UI付き）

5件入力された複数の数値のうち、最後に5未満だった値を取得して表示します。

```ts
import { fromEvent } from 'rxjs';
import { map, filter, take, last } from 'rxjs/operators';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>last の実践例:</h3>';
document.body.appendChild(output);

// 入力フィールド作成
const input = document.createElement('input');
input.type = 'number';
input.placeholder = '数値を入力して Enter';
document.body.appendChild(input);

// 入力イベントストリーム
fromEvent<KeyboardEvent>(input, 'keydown')
  .pipe(
    filter((e) => e.key === 'Enter'),
    map(() => parseInt(input.value, 10)),
    take(5), // 最初の5件だけ取り込んだらcompleteする
    filter((n) => !isNaN(n) && n < 5), // 5未満だけ通す
    last() // 最後の5未満の値を取得
  )
  .subscribe({
    next: (value) => {
      const item = document.createElement('div');
      item.textContent = `5未満の最後の値: ${value}`;
      output.appendChild(item);
    },
    complete: () => {
      const complete = document.createElement('div');
      complete.textContent = '完了しました';
      complete.style.fontWeight = 'bold';
      output.appendChild(complete);
    },
  });

```
1. 数字を5回入力してEnter押す
2. 入力された数字の中から「5未満のもの」だけをピックアップ
3. 最後に入力された5未満の数値だけを表示
4. ストリームが自然にcompleteして終わる