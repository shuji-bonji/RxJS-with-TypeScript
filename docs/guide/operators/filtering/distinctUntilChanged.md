---
description: distinctUntilChangedオペレーターは、前回と同じ値が連続した場合にスキップし、変化があった値のみを出力することで効率的なデータ処理を可能にします。
---

# distinctUntilChanged - 連続した重複値を除去する

`distinctUntilChanged` オペレーターは、連続して同じ値が発行された場合に重複を除去し、直前の値と異なる場合のみ新しい値を出力します。
 

## 🔰 基本構文と使い方

```ts
import { from } from 'rxjs';
import { distinctUntilChanged } from 'rxjs';

const numbers$ = from([1, 1, 2, 2, 3, 1, 2, 3]);

numbers$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// 出力: 1, 2, 3, 1, 2, 3
```

- 直前と同じ値であれば無視されます。
- `Array.prototype.filter`のような一括処理ではなく、**逐次的に判断**します。

[🌐 RxJS公式ドキュメント - `distinctUntilChanged`](https://rxjs.dev/api/operators/distinctUntilChanged)
 

## 💡 典型的な活用パターン

- フォーム入力検知で、同じ入力値が連続した場合に無駄なリクエストを防ぐ
- センサーやイベントストリームの変化検出
- 状態管理における不要なUI再描画の防止
 

## 🧠 実践コード例（UI付き）

検索ボックスで、入力された文字列が**前回と異なる場合のみ**APIリクエストを送信するシミュレーションです。

```ts
import { fromEvent } from 'rxjs';
import { map, distinctUntilChanged } from 'rxjs';

// 出力エリア作成
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = '検索キーワードを入力';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// 入力ストリーム
fromEvent(searchInput, 'input')
  .pipe(
    distinctUntilChanged(),
    map((event) => (event.target as HTMLInputElement).value.trim())
  )
  .subscribe((keyword) => {
    resultArea.textContent = `検索値:${keyword}で実行`;
  });

```

- 入力文字が変わらなければリクエストされません。
- 効率的な検索処理やAPI通信最適化に活用できます。
 
