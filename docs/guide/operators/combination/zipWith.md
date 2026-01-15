---
description: zipWithは元のObservableと他のObservableから対応する順番の値を揃えてペアにするRxJS結合オペレーターです。並列処理の結果を順序保証して結合、IDとデータの対応づけ、異なるタイミングで発行される関連データの同期など、順序が重要なストリーム結合に最適です。pipeable operator版でパイプライン内での使用が便利です。
---

# zipWith - 対応する値をペア化

`zipWith` オペレーターは、元のObservableと指定された他のObservableから発行される**対応する順番の値**をまとめて出力します。
すべてのObservableから1つずつ値が到着するまで待機し、揃ったタイミングでペアを作成します。
これは Creation Function の `zip` のPipeable Operator版です。

## 🔰 基本構文と使い方

```ts
import { of, interval } from 'rxjs';
import { zipWith, map, take } from 'rxjs';

const letters$ = of('A', 'B', 'C', 'D');
const numbers$ = interval(1000).pipe(
  map(val => val * 10),
  take(3)
);

letters$
  .pipe(zipWith(numbers$))
  .subscribe(([letter, number]) => {
    console.log(`${letter} - ${number}`);
  });

// 出力:
// A - 0
// B - 10
// C - 20
// (Dは対応する値がないため出力されない)
```

- 各Observableから**1つずつ値が揃ったタイミング**でペアが出力されます。
- 片方のObservableが完了すると、残りの値は破棄されます。

[🌐 RxJS公式ドキュメント - `zipWith`](https://rxjs.dev/api/operators/zipWith)


## 💡 典型的な活用パターン

- **並列処理の結果を順序保証して結合**：複数のAPI呼び出し結果をペア化
- **IDとデータの対応づけ**：ユーザーIDと対応するプロフィールデータの結合
- **ストリームの同期**：異なるタイミングで発行される関連データの同期


## 🧠 実践コード例（UI付き）

ユーザーIDリストと対応するユーザー名を順番にペア化して表示する例です。

```ts
import { from, of } from 'rxjs';
import { zipWith, delay, concatMap } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>zipWith の実践例:</h3>';
document.body.appendChild(output);

// ユーザーIDストリーム（即座に発行）
const userIds$ = from([101, 102, 103, 104]);

// ユーザー名ストリーム（1秒ごとに発行）
const userNames$ = from(['Alice', 'Bob', 'Carol']).pipe(
  concatMap(name => of(name).pipe(delay(1000)))
);

// zipして表示
userIds$
  .pipe(zipWith(userNames$))
  .subscribe(([id, name]) => {
    const item = document.createElement('div');
    item.textContent = `👤 ユーザーID ${id}: ${name}`;
    output.appendChild(item);
  });

// 出力:
// 👤 ユーザーID 101: Alice
// 👤 ユーザーID 102: Bob
// 👤 ユーザーID 103: Carol
// (104は対応する名前がないため表示されない)
```

- IDと名前が**1対1で対応**してペア化されます。
- 一方が完了すると、残りの値は破棄されます。


## 🔄 Creation Function `zip` との違い

### 基本的な違い

| | `zip` (Creation Function) | `zipWith` (Pipeable Operator) |
|:---|:---|:---|
| **使用場所** | 独立した関数として使用 | `.pipe()` チェーン内で使用 |
| **記述方法** | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **最初のストリーム** | すべて対等に扱う | メインストリームとして扱う |
| **利点** | シンプルで読みやすい | 他のオペレーターと組み合わせやすい |

### 使い分けの具体例

**シンプルなペア化だけなら Creation Function がおすすめ**

```ts
import { zip, of } from 'rxjs';

const questions$ = of('名前は？', '年齢は？', '住所は？');
const answers$ = of('太郎', '30', '東京');
const scores$ = of(10, 20, 30);

// シンプルで読みやすい
zip(questions$, answers$, scores$).subscribe(([q, a, s]) => {
  console.log(`Q: ${q}, A: ${a}, スコア: ${s}点`);
});
// 出力:
// Q: 名前は？, A: 太郎, スコア: 10点
// Q: 年齢は？, A: 30, スコア: 20点
// Q: 住所は？, A: 東京, スコア: 30点
```

**メインストリームに変換処理を加える場合は Pipeable Operator がおすすめ**

```ts
import { from, interval } from 'rxjs';
import { zipWith, map, take, filter } from 'rxjs';

// タスクリスト
const tasks$ = from([
  { id: 1, name: 'レポート作成', priority: 'high' },
  { id: 2, name: 'メール返信', priority: 'low' },
  { id: 3, name: '会議準備', priority: 'high' },
  { id: 4, name: '資料整理', priority: 'medium' }
]);

// 担当者リスト（1秒ごとに割り当て）
const assignees$ = from(['Alice', 'Bob', 'Carol', 'Dave']).pipe(
  zipWith(interval(1000).pipe(take(4))),
  map(([name]) => name)
);

// ✅ Pipeable Operator版 - 一つのパイプラインで完結
tasks$
  .pipe(
    filter(task => task.priority === 'high'),  // 高優先度のみ
    map(task => task.name),                     // タスク名を抽出
    zipWith(assignees$),                        // 担当者を割り当て
    map(([taskName, assignee]) => ({
      task: taskName,
      assignee,
      assignedAt: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(assignment => {
    console.log(`[${assignment.assignedAt}] ${assignment.task} → 担当: ${assignment.assignee}`);
  });
// 出力:
// [時刻] レポート作成 → 担当: Alice
// [時刻] 会議準備 → 担当: Bob

// ❌ Creation Function版 - 冗長になる
import { zip } from 'rxjs';
zip(
  tasks$.pipe(
    filter(task => task.priority === 'high'),
    map(task => task.name)
  ),
  assignees$
).pipe(
  map(([taskName, assignee]) => ({
    task: taskName,
    assignee,
    assignedAt: new Date().toLocaleTimeString()
  }))
).subscribe(assignment => {
  console.log(`[${assignment.assignedAt}] ${assignment.task} → 担当: ${assignment.assignee}`);
});
```

**順番が重要なデータの同期**

```ts
import { from } from 'rxjs';
import { zipWith, map, concatMap, delay } from 'rxjs';
import { of } from 'rxjs';

// UI作成
const output = document.createElement('div');
output.innerHTML = '<h3>クイズゲーム</h3>';
document.body.appendChild(output);

const questionArea = document.createElement('div');
questionArea.style.marginTop = '10px';
output.appendChild(questionArea);

// 問題リスト（即座に準備）
const questions$ = from([
  '日本の首都は？',
  '1+1は？',
  '地球は何番目の惑星？'
]);

// 回答リスト（ユーザー入力をシミュレート：2秒ごと）
const answers$ = from(['東京', '2', '3']).pipe(
  concatMap((answer, index) =>
    of(answer).pipe(delay((index + 1) * 2000))
  )
);

// 正解リスト
const correctAnswers$ = from(['東京', '2', '3']);

// ✅ Pipeable Operator版 - 問題をメインストリームとして処理
questions$
  .pipe(
    zipWith(answers$, correctAnswers$),
    map(([question, answer, correct], index) => ({
      no: index + 1,
      question,
      answer,
      correct,
      isCorrect: answer === correct
    }))
  )
  .subscribe(result => {
    const div = document.createElement('div');
    div.style.marginTop = '10px';
    div.style.padding = '10px';
    div.style.border = '1px solid #ccc';
    div.style.backgroundColor = result.isCorrect ? '#e8f5e9' : '#ffebee';
    div.innerHTML = `
      <strong>問題${result.no}:</strong> ${result.question}<br>
      <strong>回答:</strong> ${result.answer}<br>
      <strong>結果:</strong> ${result.isCorrect ? '✅ 正解！' : '❌ 不正解'}
    `;
    questionArea.appendChild(div);
  });
```

### まとめ

- **`zip`**: 複数のストリームを順番に対応づけるだけなら最適
- **`zipWith`**: メインストリームに対して変換や処理を加えながら他のストリームと順序保証して結合したい場合に最適


## ⚠️ 注意点

### 長さが異なる場合

短い方のObservableが完了すると、長い方の残りの値は破棄されます。

```ts
import { of } from 'rxjs';
import { zipWith } from 'rxjs';

const short$ = of(1, 2, 3);
const long$ = of('A', 'B', 'C', 'D', 'E');

short$.pipe(zipWith(long$)).subscribe(console.log);
// 出力: [1, 'A'], [2, 'B'], [3, 'C']
// 'D'と'E'は破棄される
```

### メモリの蓄積

片方のObservableが値を発行し続けると、もう一方が追いつくまで値がメモリに蓄積されます。

```ts
import { interval} from 'rxjs';
import { zipWith, take } from 'rxjs';

// 高速ストリーム（100msごと）
const fast$ = interval(100).pipe(take(10));

// 低速ストリーム（1秒ごと）
const slow$ = interval(1000).pipe(take(3));

fast$.pipe(zipWith(slow$)).subscribe(console.log);
// 出力: [0, 0] (1秒後), [1, 1] (2秒後), [2, 2] (3秒後)
// fast$の値がメモリに蓄積されて待機する
```

### combineLatestWith との違い

`zipWith` は対応する順番でペア化しますが、`combineLatestWith` は最新値を組み合わせます。

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(1000).pipe(take(3));
const source2$ = interval(1500).pipe(take(2));

// zipWith: 対応する順番でペア化
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// 出力: [0, 0], [1, 1]

// combineLatestWith: 最新値を組み合わせ
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// 出力: [0, 0], [1, 0], [2, 0], [2, 1]
```


## 📚 関連オペレーター

- **[zip](/guide/creation-functions/combination/zip)** - Creation Function版
- **[combineLatestWith](/guide/operators/combination/combineLatestWith)** - 最新値を組み合わせる
- **[withLatestFrom](/guide/operators/combination/withLatestFrom)** - メインストリームのみがトリガー
