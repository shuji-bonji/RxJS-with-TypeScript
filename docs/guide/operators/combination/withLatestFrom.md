---
description: "withLatestFromは、メインのObservableが値を発行するたびに、別のストリームの最新値を組み合わせて出力するオペレーターです。フォーム送信時の最新状態取得、ボタンクリック時の入力値参照など、イベントと状態の組み合わせに活用できます。"
---

# withLatestFrom - 最新値を組み合わせる

`withLatestFrom` オペレーターは、**メインストリームの値が発行されるたびに**、  
別のストリームの**最新の値を組み合わせて**出力します。


## 🔰 基本構文と使い方

```ts
import { interval, fromEvent } from 'rxjs';
import { withLatestFrom, map, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
const timer$ = interval(1000);

clicks$
  .pipe(
    withLatestFrom(timer$),
    map(([click, timerValue]) => `クリック時点のカウンター: ${timerValue}`)
  )
  .subscribe(console.log);

// 出力:
// クリック時点のカウンター: 1
// クリック時点のカウンター: 2
// クリック時点のカウンター: 2
// クリック時点のカウンター: 5

```

- メインのObservable（ここではクリック）がトリガーとなり、
- サブのObservable（ここではカウンター）の**最新値**をその都度組み合わせて出力します。

[🌐 RxJS公式ドキュメント - `withLatestFrom`](https://rxjs.dev/api/index/function/withLatestFrom)


## 💡 典型的な活用パターン

- **ユーザーアクション時に最新状態を取得する**
- **リクエスト時にキャッシュデータを参照する**
- **イベントトリガー型のデータ結合**


## 🧠 実践コード例（UI付き）

2秒ごとに、入力フィールドの最新値を取得して表示する例です。

```ts
import { fromEvent, interval } from 'rxjs';
import { map, startWith, withLatestFrom } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'withLatestFrom 2秒ごとに最新入力取得:';
document.body.appendChild(title);

// 入力フィールド作成
const nameInput = document.createElement('input');
nameInput.placeholder = '名前を入力';
document.body.appendChild(nameInput);

// 出力エリア作成
const output = document.createElement('div');
document.body.appendChild(output);

// 入力Observable
const name$ = fromEvent(nameInput, 'input').pipe(
  map((e) => (e.target as HTMLInputElement).value),
  startWith('') // 最初から空文字を流す
);

// タイマー（2秒ごとに発火）
const timer$ = interval(2000);

// タイマーが発火するたびに、最新の入力値を取得
timer$.pipe(withLatestFrom(name$)).subscribe(([_, name]) => {
  const item = document.createElement('div');
  item.textContent = `2秒ごとの取得: 名前: ${name}`;
  output.prepend(item);
});

```

- ユーザーが入力を続けている間も、
- **2秒ごとに最新の入力内容が取得・表示**されます。
