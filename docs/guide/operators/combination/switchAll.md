---
description: switchAllは、Higher-order Observable（Observable of Observables）を受け取り、新しい内部Observableに切り替えて古いものをキャンセルするオペレーターです。
---

# switchAll - 新しい内部Observableに切り替え

`switchAll` オペレーターは、**Higher-order Observable**（Observable of Observables）を受け取り、
**新しい内部Observableが発行されるたびに切り替え**、古い内部Observableをキャンセルします。

## 🔰 基本構文と使い方

```ts
import { fromEvent, interval } from 'rxjs';
import { map, switchAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// クリックごとに新しいカウンターを開始（Higher-order Observable）
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// 新しいカウンターに切り替え（古いカウンターはキャンセル）
higherOrder$
  .pipe(switchAll())
  .subscribe(x => console.log(x));

// 出力（クリックを3回した場合）:
// 0（1つ目のカウンター）
// 1（1つ目のカウンター）
// ← ここでクリック（1つ目はキャンセル）
// 0（2つ目のカウンター）← 新しいカウンターに切り替え
// ← ここでクリック（2つ目はキャンセル）
// 0（3つ目のカウンター）← 新しいカウンターに切り替え
// 1（3つ目のカウンター）
// 2（3つ目のカウンター）
```

- Higher-order Observableから新しい内部Observableが発行されると**即座に切り替え**
- 前の内部Observableは**自動的にキャンセル**される
- 常に最新の内部Observableのみが実行される

[🌐 RxJS公式ドキュメント - `switchAll`](https://rxjs.dev/api/index/function/switchAll)

## 💡 典型的な活用パターン

- **検索機能（入力のたびに古い検索をキャンセル）**
- **オートコンプリート**
- **リアルタイムデータの更新（最新のデータソースに切り替え）**

## 🧠 実践コード例

検索入力のたびに古い検索をキャンセルして最新の検索のみ実行する例

```ts
import { fromEvent, of } from 'rxjs';
import { map, switchAll, debounceTime, delay } from 'rxjs';

const input = document.createElement('input');
input.placeholder = '検索キーワードを入力';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

let searchCount = 0;

// 入力イベントをデバウンス
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: 各入力値に対して模擬検索API呼び出し
const results$ = search$.pipe(
  map(query => {
    const id = ++searchCount;
    const start = Date.now();

    // 模擬検索API呼び出し（1秒の遅延）
    return of(`検索結果: "${query}"`).pipe(
      delay(1000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `[検索#${id}] ${msg} (${elapsed}秒)`;
      })
    );
  }),
  switchAll() // 新しい検索が開始されたら古い検索をキャンセル
);

results$.subscribe(result => {
  output.innerHTML = ''; // 前の結果をクリア
  const item = document.createElement('div');
  item.textContent = result;
  output.appendChild(item);
});
```

- ユーザーが入力を変更すると、**古い検索は自動的にキャンセル**されます
- 常に最新の検索結果のみが表示されます

## 🔄 関連オペレーター

| オペレーター | 説明 |
|---|---|
| `switchMap` | `map` + `switchAll` の短縮形（最もよく使われる） |
| [mergeAll](./mergeAll) | 全ての内部Observableを並行に購読 |
| [concatAll](./concatAll) | 内部Observableを順番に購読（前の完了を待つ） |
| [exhaustAll](./exhaustAll) | 実行中は新しい内部Observableを無視 |

## ⚠️ 注意点

### メモリリーク防止

`switchAll` は古い内部Observableを**自動的にキャンセル**するため、メモリリークを防ぐのに役立ちます。
検索やオートコンプリートなど、頻繁に新しいリクエストが発生する場合に最適です。

### 完了しない内部Observable

内部Observableが完了しなくても、新しい内部Observableが発行されれば自動的に切り替わります。

```ts
// interval は完了しないが、次のクリックで自動的にキャンセルされる
clicks$.pipe(
  map(() => interval(1000)), // 完了しない
  switchAll()
).subscribe();
```

### 最後の値のみ重要な場合に最適

古い処理の結果が不要で、**最新の結果のみが重要な場合**に `switchAll` を使用します。
全ての結果が必要な場合は [mergeAll](./mergeAll) を使用してください。
