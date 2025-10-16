---
description: exhaustAllは、Higher-order Observable（Observable of Observables）を受け取り、内部Observableが実行中の場合は新しいものを無視するオペレーターです。
---

# exhaustAll - 実行中は新しい内部Observableを無視

`exhaustAll` オペレーターは、**Higher-order Observable**（Observable of Observables）を受け取り、
**内部Observableが実行中の場合、新しい内部Observableを無視**します。

## 🔰 基本構文と使い方

```ts
import { fromEvent, interval } from 'rxjs';
import { map, exhaustAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// クリックごとに新しいカウンターを開始（Higher-order Observable）
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// カウンターが実行中の場合、新しいクリックを無視
higherOrder$
  .pipe(exhaustAll())
  .subscribe(x => console.log(x));

// 出力（クリックを連続で3回した場合）:
// 0（1つ目のカウンター）
// 1（1つ目のカウンター）
// ← ここでクリック（無視される：1つ目が実行中のため）
// 2（1つ目のカウンター）← 完了
// ← ここでクリック（受け付けられる：実行中のカウンターなし）
// 0（2つ目のカウンター）
// 1（2つ目のカウンター）
// 2（2つ目のカウンター）
```

- 内部Observableが実行中の場合、**新しい内部Observableは無視**される
- 実行中のObservableが**完了してから次を受け付ける**
- 二重実行を防止するのに最適

[🌐 RxJS公式ドキュメント - `exhaustAll`](https://rxjs.dev/api/index/function/exhaustAll)

## 💡 典型的な活用パターン

- **ダブルクリック防止（ボタン連打防止）**
- **ログインリクエストの二重送信防止**
- **保存処理の重複実行防止**

## 🧠 実践コード例

保存ボタンの二重クリックを防止する例

```ts
import { fromEvent, of } from 'rxjs';
import { map, exhaustAll, delay } from 'rxjs';

const saveButton = document.createElement('button');
saveButton.textContent = '保存';
document.body.appendChild(saveButton);

const output = document.createElement('div');
document.body.appendChild(output);

let saveCount = 0;

// ボタンクリックイベント
const clicks$ = fromEvent(saveButton, 'click');

// Higher-order Observable: 各クリックに対して模擬保存処理
const saves$ = clicks$.pipe(
  map(() => {
    const id = ++saveCount;
    const start = Date.now();

    // ボタンを一時的に無効化（視覚的フィードバック）
    saveButton.disabled = true;

    // 模擬保存処理（2秒の遅延）
    return of(`保存完了 #${id}`).pipe(
      delay(2000),
      map(msg => {
        saveButton.disabled = false;
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed}秒)`;
      })
    );
  }),
  exhaustAll() // 保存中は新しいクリックを無視
);

saves$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});

// 無視されたクリックをログ出力
clicks$.subscribe(() => {
  if (saveButton.disabled) {
    console.log('保存中のため、クリックを無視しました');
  }
});
```

- 保存処理中は**新しいクリックが無視**されます
- 保存完了後、次のクリックが受け付けられます

## 🔄 関連オペレーター

| オペレーター | 説明 |
|---|---|
| `exhaustMap` | `map` + `exhaustAll` の短縮形（よく使われる） |
| [mergeAll](./mergeAll) | 全ての内部Observableを並行に購読 |
| [concatAll](./concatAll) | 内部Observableを順番に購読（キューに溜める） |
| [switchAll](./switchAll) | 新しい内部Observableに切り替え（古いものをキャンセル） |

## 🔄 他のオペレーターとの比較

| オペレーター | 新しい内部Observableが発行されたら |
|---|---|
| `mergeAll` | 並行実行する |
| `concatAll` | キューに追加する（前の完了を待つ） |
| `switchAll` | 古いものをキャンセルして切り替える |
| `exhaustAll` | **無視する（実行中の完了を待つ）** |

## ⚠️ 注意点

### イベントの損失

`exhaustAll` は実行中のイベントを**完全に無視**するため、全てのイベントを処理したい場合は不適切です。

```ts
// ❌ 全てのクリックを記録したい場合は exhaustAll は不適切
// ✅ mergeAll または concatAll を使用
```

### UIフィードバック

ユーザーに「イベントが無視された」ことを視覚的に伝えることが重要です。

```ts
// ボタンを無効化
saveButton.disabled = true;

// トーストメッセージを表示
showToast('処理中です。しばらくお待ちください。');
```

### 適切な使用場面

#### `exhaustAll` が最適なケース
- ログイン処理（二重送信防止）
- 保存処理（重複実行防止）
- アニメーション（実行中は新しいアニメーションを開始しない）

#### `exhaustAll` が不適切なケース
- 検索処理（最新の検索を実行したい → `switchAll`）
- 全てのイベントを処理したい（→ `mergeAll` or `concatAll`）
