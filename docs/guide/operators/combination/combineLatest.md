---
description: combineLatestオペレーターを用いて複数のObservableの最新値を組み合わせる方法と、UIやフォーム入力への応用例を紹介します。
---

# combineLatest - 最新の値を組み合わせる

`combineLatest` オペレーターは、複数のObservableの**最新の値をすべてまとめて出力**します。  
いずれかのソースObservableから新しい値が発行されるたびに、すべての最新値をまとめた結果が発行されます。

## 🔰 基本構文と使い方

```ts
import { combineLatest, of } from 'rxjs';

const obs1$ = of('A', 'B', 'C');
const obs2$ = of(1, 2, 3);

combineLatest([obs1$, obs2$]).subscribe(([val1, val2]) => {
  console.log(val1, val2);
});

// 出力:
// C 1
// C 2
// C 3
```

- 各Observableが**少なくとも1つ値を発行してから**、組み合わせた値が出力されます。
- どちらか一方に新しい値が来るたびに、最新のペアが再出力されます。

[🌐 RxJS公式ドキュメント - `combineLatest`](https://rxjs.dev/api/index/function/combineLatest)


## 💡 典型的な活用パターン

- **フォーム入力のリアルタイム検証**（例: 名前とメールアドレスを同時監視）
- **複数ストリームの状態同期**（例: センサー値やデバイスステータスの統合）
- **依存関係のあるデータフェッチ**（例: ユーザーIDと設定IDの組み合わせ）

## 🧠 実践コード例（UI付き）

フォームの2つの入力フィールドの最新状態を常に組み合わせて表示します。

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith } from 'rxjs/operators';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>combineLatest の実践例:</h3>';
document.body.appendChild(output);

// フォームフィールド作成
const nameInput = document.createElement('input');
nameInput.placeholder = '名前を入力';
document.body.appendChild(nameInput);

const emailInput = document.createElement('input');
emailInput.placeholder = 'メールを入力';
document.body.appendChild(emailInput);

// 各入力のObservable
const name$ = fromEvent(nameInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

const email$ = fromEvent(emailInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

// 最新の入力値を結合
combineLatest([name$, email$]).subscribe(([name, email]) => {
  output.innerHTML = `
    <div><strong>名前:</strong> ${name}</div>
    <div><strong>メール:</strong> ${email}</div>
  `;
});
```

- いずれかのフィールドに入力すると、**最新の2つの入力状態が即座に表示**されます。
- `startWith('')` を使うことで、最初から組み合わせ結果を得られるようにしています。
