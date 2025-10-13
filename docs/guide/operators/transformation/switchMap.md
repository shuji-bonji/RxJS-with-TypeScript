---
description: switchMapは前のObservableをキャンセルして最新のObservableに切り替える演算子で、ライブサーチやナビゲーション切替などに最適です。
---

# switchMap - 前のObservableをキャンセルして最新のものに切り替える

`switchMap`オペレーターは、入力ストリームの各値に対して新しいObservableを生成し、**前回のObservableをキャンセルして最新のObservableだけに切り替え**ます。  
検索フォームのように、直近の入力だけを有効にしたいケースに最適です。

## 🔰 基本構文と使い方

```ts
import { of } from 'rxjs';
import { delay, switchMap } from 'rxjs';

of('A', 'B', 'C').pipe(
  switchMap(value =>
    of(`${value} 完了`).pipe(delay(1000))
  )
).subscribe(console.log);

// 出力例:
// C 完了
```

- 各値に対して新しいObservableを作成します。
- しかし、**新しい値が来た瞬間に前のObservableはキャンセル**されます。
- 最終的に`C`だけが出力されます。

[🌐 RxJS公式ドキュメント - `switchMap`](https://rxjs.dev/api/operators/switchMap)

## 💡 典型的な活用パターン

- 入力フォームのオートコンプリート
- ライブサーチ機能（最新の入力だけ有効）
- ナビゲーションやルーティング切り替え時のリソース読み込み
- ユーザーのアクションを最新のものに切り替えたい場合

## 🧠 実践コード例（UI付き）

検索ボックスに文字を入力すると、即座にAPIリクエストが送信され、**最後に入力したものだけの結果**を表示します。

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

// 入力フィールド作成
const searchInput = document.createElement('input');
searchInput.placeholder = 'ユーザー名で検索';
document.body.appendChild(searchInput);

// 出力領域
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// 入力イベント処理
fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  map(event => (event.target as HTMLInputElement).value.trim()),
  switchMap(term => {
    if (term === '') {
      return of([]);
    }
    return ajax.getJSON(`https://jsonplaceholder.typicode.com/users?username_like=${term}`);
  })
).subscribe(users => {
  output.innerHTML = '';

  (users as any[]).forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.username;
    output.appendChild(div);
  });
});
```

- 入力が変わるたびに前のリクエストはキャンセルされます。
- 最新の検索ワードにマッチしたユーザーだけが表示されます。
