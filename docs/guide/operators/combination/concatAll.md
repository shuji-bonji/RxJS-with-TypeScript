---
description: concatAllは、Higher-order Observable（Observable of Observables）を受け取り、内部Observableを順番に購読して値を平坦化するオペレーターです。前のObservableが完了してから次を開始します。
---

# concatAll - 内部Observableを順番に平坦化

`concatAll` オペレーターは、**Higher-order Observable**（Observable of Observables）を受け取り、
**内部Observableを順番に購読**して、値を平坦化します。前のObservableが完了するまで次は開始しません。

## 🔰 基本構文と使い方

```ts
import { fromEvent, interval } from 'rxjs';
import { map, concatAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// クリックごとに新しいカウンターを開始（Higher-order Observable）
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// カウンターを順番に購読（前のカウンターが完了してから次を開始）
higherOrder$
  .pipe(concatAll())
  .subscribe(x => console.log(x));

// 出力（クリックを3回した場合）:
// 0（1つ目のカウンター）
// 1（1つ目のカウンター）
// 2（1つ目のカウンター）← 完了
// 0（2つ目のカウンター）← 1つ目の完了後に開始
// 1（2つ目のカウンター）
// 2（2つ目のカウンター）← 完了
// 0（3つ目のカウンター）← 2つ目の完了後に開始
// ...
```

- Higher-order Observableから発行される各内部Observableを**順番に購読**
- 前の内部Observableが**完了するまで次を開始しない**
- 値の順序が保証される

[🌐 RxJS公式ドキュメント - `concatAll`](https://rxjs.dev/api/index/function/concatAll)

## 💡 典型的な活用パターン

- **API呼び出しを順番に実行（前のリクエストが完了してから次を実行）**
- **アニメーションを順番に再生**
- **ファイルアップロードを順番に処理**

## 🧠 実践コード例

ボタンクリックごとにAPI呼び出し（模擬）を順番に実行する例

```ts
import { fromEvent, of } from 'rxjs';
import { map, concatAll, delay } from 'rxjs';

const button = document.createElement('button');
button.textContent = 'API呼び出し';
document.body.appendChild(button);

const output = document.createElement('div');
document.body.appendChild(output);

let callCount = 0;

// ボタンクリックイベント
const clicks$ = fromEvent(button, 'click');

// Higher-order Observable: 各クリックに対して模擬API呼び出し
const results$ = clicks$.pipe(
  map(() => {
    const id = ++callCount;
    const start = Date.now();

    // 模擬API呼び出し（2秒の遅延）
    return of(`API呼び出し #${id} 完了`).pipe(
      delay(2000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed}秒)`;
      })
    );
  }),
  concatAll() // 全てのAPI呼び出しを順番に実行
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- ボタンを連続クリックしても、**API呼び出しは順番に実行**されます
- 前のAPI呼び出しが完了してから次が開始されます

## 🔄 関連オペレーター

| オペレーター | 説明 |
|---|---|
| `concatMap` | `map` + `concatAll` の短縮形（よく使われる） |
| [mergeAll](./mergeAll) | 全ての内部Observableを並行に購読 |
| [switchAll](./switchAll) | 新しい内部Observableに切り替え（古いものをキャンセル） |
| [exhaustAll](./exhaustAll) | 実行中は新しい内部Observableを無視 |

## ⚠️ 注意点

### バックプレッシャー（滞留）

内部Observableの発行速度が完了速度より速い場合、**未処理のObservableがキューに溜まります**。

```ts
// 1秒ごとにクリック → 2秒かかるAPI呼び出し
// → キューに溜まり続ける可能性
```

この場合は以下の対策を検討
- `switchAll` を使用（最新のみ処理）
- `exhaustAll` を使用（実行中は無視）
- デバウンスやスロットリングを追加

### 無限Observableへの注意

前のObservableが**完了しない場合、次は永遠に開始されません**。

#### ❌ interval は完了しないので、2つ目のカウンターは開始されない
```ts
clicks$.pipe(
  map(() => interval(1000)), // 完了しない
  concatAll()
).subscribe();
```
#### ✅ take で完了させる
```ts
clicks$.pipe(
  map(() => interval(1000).pipe(take(3))), // 3回で完了
  concatAll()
).subscribe();
```
