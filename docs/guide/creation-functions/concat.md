---
description: concat Creation Functionで複数のObservableを順番に結合する方法と、ステップ実行やUI表示への活用方法を解説します。
---

# concat - 順番にストリームを結合する

`concat` は、複数のObservableを指定された順番に**順次実行**するCreation Functionです。  
前のObservableが`complete`してから、次のObservableの発行が始まります。

## 🔰 基本構文と使い方

```ts
import { concat, of, delay } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));

concat(obs1$, obs2$).subscribe(console.log);
// 出力: A → B → C → D
```

- すべての`obs1$`の発行が完了後、`obs2$`の発行が開始されます。
- 同時実行ではなく「順番に」ストリームを流すのがポイントです。

[🌐 RxJS公式ドキュメント - `concat`](https://rxjs.dev/api/index/function/concat)


## 💡 典型的な活用パターン

- **ステップバイステップ処理**：前工程が完了した後に次工程へ進めたい場合
- **順序保証されたAPIリクエスト**：一連の非同期操作を順番に実行したい場合
- **アニメーションや通知など、順番が重要なUIイベント**の制御

## 🧠 実践コード例（UI付き）

ローディングメッセージとデータリストを**順番に表示**する例です。

```ts
import { concat, of, timer } from 'rxjs';
import { map, take } from 'rxjs/operators';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>concat の実践例:</h3>';
document.body.appendChild(output);

// ローディングストリーム
const loading$ = timer(0, 1000).pipe(
  map((count) => `⏳ ローディング中... (${count + 1}s)`),
  take(3) // 3秒間だけ発行
);

// データリストストリーム
const data$ = of('🍎 Apple', '🍌 Banana', '🍇 Grape');

// concatして順番に表示
concat(loading$, data$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- 最初にローディングメッセージが3回表示され、
- その後にデータリストが順番に表示されます。
- **concat** を使うことで、自然な「段階的表示」が簡単に実現できます。


## 🔗 関連オペレーター

- **[concatWith](/guide/operators/combination/concatWith)** - Pipeable Operator版（パイプライン内で使用）
- **[concatMap](/guide/operators/transformation/concatMap)** - 各値を順次マッピングして結合
- **[merge](/guide/creation-functions/merge)** - 並行結合する Creation Function