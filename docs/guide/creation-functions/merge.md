---
description: merge Creation Functionは複数のObservableを同時に購読し、それぞれの値をリアルタイムに統合して出力するために使用されます。
---

# merge - 複数のストリームを同時に結合する

`merge` は、複数のObservableを同時に購読し、
それぞれのObservableから値が発行されるたびに、そのまま出力するCreation Functionです。

## 🔰 基本構文と使い方

```ts
import { merge, interval } from 'rxjs';
import { map, take } from 'rxjs/operators';

const source1$ = interval(1000).pipe(
  map(val => `ストリーム1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `ストリーム2: ${val}`),
  take(2)
);

merge(source1$, source2$).subscribe(console.log);
// 出力例:
// ストリーム1: 0
// ストリーム2: 0
// ストリーム1: 1
// ストリーム1: 2
// ストリーム2: 1
```

- すべてのObservableを同時に購読し、**発行された順**に値が流れてきます。
- 順番の保証はなく、**各Observableの発行タイミングに依存**します。


[🌐 RxJS公式ドキュメント - `merge`](https://rxjs.dev/api/index/function/merge)

## 💡 典型的な活用パターン

- **複数の非同期イベントを統合**する（例: ユーザー入力とバックエンド通知）
- **複数のデータストリームを単一ストリームに集約**する
- **リアルタイム更新とポーリングの統合**など、並列的な情報源の結合

## 🧠 実践コード例（UI付き）

クリックイベントとタイマーイベントをリアルタイムに結合して表示します。

```ts
import { merge, fromEvent, timer } from 'rxjs';
import { map } from 'rxjs/operators';

// 出力エリアを作成
const output = document.createElement('div');
output.innerHTML = '<h3>merge の実践例:</h3>';
document.body.appendChild(output);

// ボタン要素を作成
const button = document.createElement('button');
button.textContent = 'クリックしてイベント発火';
document.body.appendChild(button);

// クリックストリーム
const click$ = fromEvent(button, 'click').pipe(
  map(() => '✅ ボタンクリック検知')
);

// タイマーストリーム
const timer$ = timer(3000, 3000).pipe(
  map((val) => `⏰ タイマーイベント (${val})`)
);

// mergeして表示
merge(click$, timer$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- **ボタンをクリックすると即時にイベントが発生**し、
- **タイマーは3秒ごとに繰り返しイベントを発生**します。
- 2つの異なる種類のObservableを**リアルタイムで統合**できることが体験できます。


## 🔗 関連オペレーター

- **[mergeWith](/guide/operators/combination/mergeWith)** - Pipeable Operator版（パイプライン内で使用）
- **[mergeMap](/guide/operators/transformation/mergeMap)** - 各値を並列マッピングして結合
- **[concat](/guide/creation-functions/concat)** - 順次結合する Creation Function