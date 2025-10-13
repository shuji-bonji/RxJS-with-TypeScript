---
description: race Creation Functionは、複数のObservableのうち最初に値を発行したストリームのみを採用し、それ以降は他を無視する特殊な結合処理を実現します。
---

# race - 最初に値を発行したストリームを採用する

`race` は、複数のObservableのうち**最初に値を発行したObservableだけを生かし**、
他のObservableは無視する特殊な結合Creation Functionです。


## 🔰 基本構文と使い方

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'ゆっくり (5秒)'));
const fast$ = timer(2000).pipe(map(() => '速い (2秒)'));

race(slow$, fast$).subscribe(console.log);
// 出力: 速い (2秒)
```

- 最初に値を発行したObservableだけが勝者となり、その後のストリームを継続します。
- 他のObservableは無視されます。

[🌐 RxJS公式ドキュメント - `race`](https://rxjs.dev/api/index/function/race)


## 💡 典型的な活用パターン

- **複数のユーザーアクション（クリック、キー入力、スクロール）の早い方を処理する**
- **手動送信と自動保存など、複数のトリガーのうち早い方を採用する**
- **複数データ取得処理のうち、先に完了したデータを優先して表示する**

## 🧠 実践コード例（UI付き）

異なるタイミングで発火する3つのストリームから、最初に発行したものだけを採用するレースをシミュレートします。

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>race の実践例:</h3>';
document.body.appendChild(output);

// 異なるタイミングのObservable
const slow$ = timer(5000).pipe(map(() => 'ゆっくり (5秒後)'));
const medium$ = timer(3000).pipe(map(() => '普通 (3秒後)'));
const fast$ = timer(2000).pipe(map(() => '速い (2秒後)'));

const startTime = Date.now();

// レース開始メッセージ
const waiting = document.createElement('div');
waiting.textContent = 'レース開始...最初に発行されるストリームを待っています。';
output.appendChild(waiting);

// race実行
race(slow$, medium$, fast$).subscribe(winner => {
  const endTime = Date.now();
  const elapsed = ((endTime - startTime) / 1000).toFixed(2);

  const result = document.createElement('div');
  result.innerHTML = `<strong>勝者:</strong> ${winner} (経過時間: ${elapsed}秒)`;
  result.style.color = 'green';
  result.style.marginTop = '10px';
  output.appendChild(result);

  const explanation = document.createElement('div');
  explanation.textContent = '※ 最初に値を発行したObservableのみが選択されます。';
  explanation.style.marginTop = '5px';
  output.appendChild(explanation);
});
```

- 2秒後、最初に`fast$`が発行され、それ以降は`fast$`のみが出力されます。
- 他の`medium$`や`slow$`の発行は無視されます。


## 🔗 関連オペレーター

- **[raceWith](/guide/operators/combination/raceWith)** - Pipeable Operator版（パイプライン内で使用）
- **[timeout](/guide/operators/utility/timeout)** - タイムアウト専用オペレーター
- **[merge](/guide/creation-functions/merge)** - すべてのストリームをマージする Creation Function