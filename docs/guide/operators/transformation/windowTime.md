
description: windowTimeオペレーターは、一定時間ごとにObservableを分割し、それぞれの時間枠で発行された値を個別に処理できるようにする演算子です。


# 実用的なユースケース - 変換オペレーター編

ここでは、変換オペレーター (`map`, `mergeMap`, `switchMap`, `concatMap` など) を使った**実践的なユースケース**を紹介します。  
「理論」から「実戦」への橋渡しとなることを目指しています。



## 💡 典型的なシナリオ

### リスト変換 (map)

```ts
import { from } from 'rxjs';
import { map } from 'rxjs/operators';

from([1, 2, 3]).pipe(
  map(x => x * 10)
).subscribe(console.log);
// 出力: 10, 20, 30
```
- 単純な値変換に最適。
- ユーザー情報から表示用テキストを作成する場合などにも。



### 非同期リクエスト (mergeMap)

```ts
import { fromEvent, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs/operators';

const button = document.createElement('button');
button.textContent = '送信';
document.body.appendChild(button);

fromEvent(button, 'click').pipe(
  mergeMap(() => 
    of('リクエスト完了').pipe(delay(1000))
  )
).subscribe(console.log);
```
- 複数リクエストを**同時並行**で処理したい場合に。


### 入力フォームのライブサーチ (switchMap)

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, switchMap, map } from 'rxjs/operators';
import { ajax } from 'rxjs/ajax';

const input = document.createElement('input');
input.placeholder = 'ユーザーID (1-10)';
document.body.appendChild(input);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

fromEvent(input, 'input').pipe(
  debounceTime(300),
  map(e => (e.target as HTMLInputElement).value),
  switchMap(query =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/posts?userId=${query}`)
  )
).subscribe(result => {
  console.log(result);
  output.textContent = `ユーザー${(result as any[])[0]?.userId || '?'}の投稿: ${(result as any[]).length}件`;
});
```

**動作説明**:
- 入力例: `1` → ユーザーID 1の投稿一覧（10件）を取得
- 入力例: `2` → ユーザーID 2の投稿一覧（10件）を取得
- **最新の入力だけ**を対象にAPIリクエストを送るパターン
- 高速で「1」→「2」→「3」と入力しても、最後の「3」のリクエストだけが有効になる



### キュー型のリクエスト処理 (concatMap)

```ts
import { fromEvent, of } from 'rxjs';
import { concatMap, delay } from 'rxjs/operators';

const sendButton = document.createElement('button');
sendButton.textContent = 'キュー送信';
document.body.appendChild(sendButton);

fromEvent(sendButton, 'click').pipe(
  concatMap(() => 
    of('リクエスト完了').pipe(delay(1000))
  )
).subscribe(console.log);
```
- 順番を守って**直列で処理**したい場合に。



### 重複リクエスト防止 (exhaustMap)

```ts
import { fromEvent, of } from 'rxjs';
import { exhaustMap, delay } from 'rxjs/operators';

const singleSubmitButton = document.createElement('button');
singleSubmitButton.textContent = '単発送信';
document.body.appendChild(singleSubmitButton);

fromEvent(singleSubmitButton, 'click').pipe(
  exhaustMap(() => 
    of('リクエスト完了').pipe(delay(2000))
  )
).subscribe(console.log);
```
- 実行中に新たなリクエストを**無視**したい場合に。



## 📚 ポイントまとめ

- `map` → シンプルな値変換
- `mergeMap` → 並列処理
- `switchMap` → 最新だけ有効
- `concatMap` → 順番厳守
- `exhaustMap` → 実行中は無視

それぞれの特徴を理解して、シチュエーションに応じて使い分けましょう！


