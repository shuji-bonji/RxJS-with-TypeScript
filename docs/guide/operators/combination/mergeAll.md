---
description: mergeAllは、Higher-order Observable（Observable of Observables）を受け取り、全ての内部Observableを並行に購読して値を平坦化するオペレーターです。
---

# mergeAll - 全ての内部Observableを並行に平坦化

`mergeAll` オペレーターは、**Higher-order Observable**（Observable of Observables）を受け取り、
**全ての内部Observableを並行に購読**して、値を平坦化します。

## 🔰 基本構文と使い方

```ts
import { fromEvent, interval } from 'rxjs';
import { map, mergeAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// クリックごとに新しいカウンターを開始（Higher-order Observable）
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// 全てのカウンターを並行に購読
higherOrder$
  .pipe(mergeAll())
  .subscribe(x => console.log(x));

// 出力（クリックを3回した場合）:
// 0（1つ目のカウンター）
// 1（1つ目のカウンター）
// 0（2つ目のカウンター）← 並行実行
// 2（1つ目のカウンター）
// 1（2つ目のカウンター）
// 0（3つ目のカウンター）← 並行実行
// ...
```

- Higher-order Observableから発行される各内部Observableを**並行に購読**
- 全ての内部Observableからの値を**単一のストリームに結合**
- 並行購読数を制限することも可能（`mergeAll(2)` = 最大2つまで並行）

[🌐 RxJS公式ドキュメント - `mergeAll`](https://rxjs.dev/api/index/function/mergeAll)

## 💡 典型的な活用パターン

- **複数のAPI呼び出しを並行実行**
- **ユーザーアクションごとに独立したストリームを開始**
- **WebSocketやEventSourceなど複数のリアルタイム接続を統合**

## 🧠 実践コード例

入力変更のたびにAPI呼び出し（模擬）を並行実行する例

```ts
import { fromEvent, of } from 'rxjs';
import { map, mergeAll, delay, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = '検索キーワードを入力';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

// 入力イベントをデバウンス
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: 各入力値に対して模擬API呼び出し
const results$ = search$.pipe(
  map(query =>
    // 模擬API呼び出し（500msの遅延）
    of(`結果: "${query}"`).pipe(delay(500))
  ),
  mergeAll() // 全てのAPI呼び出しを並行実行
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- ユーザーが素早く入力を変更しても、**全てのAPI呼び出しが並行実行**されます
- 古い検索結果が新しい結果の後に表示される可能性があります（順序保証なし）

## 🔄 関連オペレーター

| オペレーター | 説明 |
|---|---|
| `mergeMap` | `map` + `mergeAll` の短縮形（よく使われる） |
| [concatAll](./concatAll) | 内部Observableを順番に購読（前の完了を待つ） |
| [switchAll](./switchAll) | 新しい内部Observableに切り替え（古いものをキャンセル） |
| [exhaustAll](./exhaustAll) | 実行中は新しい内部Observableを無視 |

## ⚠️ 注意点

### 並行購読数の制限

並行購読数を制限しないと、パフォーマンス問題が発生する可能性があります。

```ts
// 並行購読数を2つまでに制限
higherOrder$.pipe(
  mergeAll(2) // 最大2つまで並行実行
).subscribe();
```

### 順序保証なし

`mergeAll` は並行実行のため、**値の順序は保証されません**。
順序が重要な場合は [concatAll](./concatAll) を使用してください。
