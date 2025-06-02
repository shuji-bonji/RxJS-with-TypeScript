---
description: debounceTimeは連続した入力やイベントの中で、一定時間静止した後の最新値だけを発行するオペレーターで、検索ボックスなどに最適です。
---

# debounceTime - イベント発火を間引き、一定時間待機後に値を発行する

`debounceTime` オペレーターは、ストリーム内で値が発行された後、指定した時間だけ新しい値が発行されなかった場合に最後の値を出力します。  
ユーザー入力の検索ボックスなど、頻繁なイベントを抑制する場面で非常によく使われます。
 
## 🔰 基本構文と使い方

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs/operators';

const searchBox = document.createElement('input');
document.body.appendChild(searchBox);

fromEvent(searchBox, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value),
    debounceTime(300)
  )
  .subscribe(console.log);
```

- 入力イベントが発生した後、300ms以内にさらに入力がなければその値を発行します。
- 短時間に連続して発生するイベントをまとめる効果があります。

[🌐 RxJS公式ドキュメント - `debounceTime`](https://rxjs.dev/api/operators/debounceTime)
 
## 💡 典型的な活用パターン

- 検索ボックスで、ユーザーがタイピングを終了した後にリクエストを送信する
- ウィンドウリサイズイベントの最終的なサイズ取得
- スクロールイベントの最終位置取得
 
## 🧠 実践コード例（UI付き）

検索ボックスに文字を入力すると、300ms間入力が停止した時点で検索開始メッセージを表示します。

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs/operators';

// 出力エリア作成
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = '検索ワードを入力';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// 入力ストリーム
fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300)
).subscribe(value => {
  resultArea.textContent = `「${value}」の検索を開始しました`;
});
```

- 入力中はすぐに反応せず、
- 入力を止めて300ms後に最新の入力値で検索を開始します。
 