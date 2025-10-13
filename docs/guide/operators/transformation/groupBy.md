---
description: groupByオペレーターは、ストリームの値を指定したキーに基づいてグループ化し、グループごとに別々のObservableを作成するRxJSの演算子で、データの分類や集約処理に活用されます。
---

# groupBy - キーに基づいて値をグループ化する

`groupBy`オペレーターは、ストリームから発行される値を**指定したキーに基づいてグループ化**し、各グループを個別のObservableとして出力します。
データをカテゴリ別に分類したり、グループごとに異なる処理を適用したりする場合に便利です。

## 🔰 基本構文と使い方

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

interface Person {
  name: string;
  age: number;
}

const people: Person[] = [
  { name: '太郎', age: 25 },
  { name: '花子', age: 30 },
  { name: '次郎', age: 25 },
  { name: '美咲', age: 30 },
  { name: '健太', age: 35 },
];

from(people).pipe(
  groupBy(person => person.age), // 年齢でグループ化
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(arr => ({ age: group.key, people: arr }))
    )
  )
).subscribe(result => {
  console.log(`年齢 ${result.age}:`, result.people);
});

// 出力:
// 年齢 25: [{name: '太郎', age: 25}, {name: '次郎', age: 25}]
// 年齢 30: [{name: '花子', age: 30}, {name: '美咲', age: 30}]
// 年齢 35: [{name: '健太', age: 35}]
```

- `groupBy(person => person.age)`で年齢をキーとしてグループ化
- 各グループは`GroupedObservable`として扱われ、`key`プロパティでグループのキーにアクセス可能
- `mergeMap`で各グループのObservableを処理

[🌐 RxJS公式ドキュメント - `groupBy`](https://rxjs.dev/api/operators/groupBy)

## 💡 典型的な活用パターン

- データのカテゴリ別分類
- グループごとの集計処理
- ログやイベントの種類別処理
- データのグルーピングと変換

## 🧠 実践コード例（UI付き）

ボタンをクリックすると、色別にグループ化して個数を表示する例です。

```ts
import { fromEvent, from } from 'rxjs';
import { groupBy, mergeMap, toArray, switchMap, map } from 'rxjs';

// ボタンを作成
const colors = ['赤', '青', '緑', '黄'];
colors.forEach(color => {
  const button = document.createElement('button');
  button.textContent = color;
  button.style.margin = '5px';
  button.style.padding = '10px';
  button.dataset.color = color;
  document.body.appendChild(button);
});

const calculateButton = document.createElement('button');
calculateButton.textContent = '集計する';
calculateButton.style.margin = '5px';
calculateButton.style.padding = '10px';
document.body.appendChild(calculateButton);

// 出力エリア作成
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// クリックされた色を記録
const clicks: string[] = [];

// 色ボタンのクリックイベント
fromEvent(document, 'click').subscribe((event: Event) => {
  const target = event.target as HTMLElement;
  const color = target.dataset.color;
  if (color) {
    clicks.push(color);
    output.innerHTML = `選択した色: ${clicks.join(', ')}`;
  }
});

// 集計ボタンのクリック時にグループ化して表示
fromEvent(calculateButton, 'click').pipe(
  switchMap(() =>
    from(clicks).pipe(
      groupBy(color => color),
      mergeMap(group =>
        group.pipe(
          toArray(),
          map(items => ({ color: group.key, count: items.length }))
        )
      ),
      toArray()
    )
  )
).subscribe(results => {
  if (results.length === 0) {
    output.innerHTML = '<p>まだ色が選択されていません</p>';
    return;
  }
  const resultText = results
    .map(r => `${r.color}: ${r.count}回`)
    .join('<br>');
  output.innerHTML = `<h3>集計結果</h3>${resultText}`;
});
```

- 色ボタンをクリックして色を選択
- 「集計する」ボタンで色別にグループ化して個数を表示
- `groupBy`で色ごとにグループ化し、各グループの要素数をカウント

## 🎯 カテゴリ別の集計例

商品をカテゴリ別に分類し、カテゴリごとの合計金額を計算する例です。

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, reduce, map } from 'rxjs';

interface Product {
  name: string;
  category: string;
  price: number;
}

const products: Product[] = [
  { name: 'りんご', category: '果物', price: 150 },
  { name: 'みかん', category: '果物', price: 100 },
  { name: 'にんじん', category: '野菜', price: 80 },
  { name: 'トマト', category: '野菜', price: 120 },
  { name: '牛乳', category: '乳製品', price: 200 },
  { name: 'チーズ', category: '乳製品', price: 300 },
];

from(products).pipe(
  groupBy(product => product.category),
  mergeMap(group =>
    group.pipe(
      reduce((total, product) => total + product.price, 0),
      map(total => ({ category: group.key, total }))
    )
  )
).subscribe(result => {
  console.log(`${result.category}: ${result.total}円`);
});

// 出力:
// 果物: 250円
// 野菜: 200円
// 乳製品: 500円
```

## 🎯 要素セレクターの使用例

グループ化する際に、値を変換することもできます。

```ts
import { from } from 'rxjs';
import { groupBy, map, mergeMap, toArray } from 'rxjs';

interface Student {
  name: string;
  grade: number;
  score: number;
}

const students: Student[] = [
  { name: '太郎', grade: 1, score: 85 },
  { name: '花子', grade: 2, score: 92 },
  { name: '次郎', grade: 1, score: 78 },
  { name: '美咲', grade: 2, score: 88 },
];

from(students).pipe(
  groupBy(
    student => student.grade,           // キーセレクター
    student => student.name             // 要素セレクター（名前だけを保持）
  ),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(names => ({ grade: group.key, students: names }))
    )
  )
).subscribe(result => {
  console.log(`${result.grade}年生:`, result.students.join(', '));
});

// 出力:
// 1年生: 太郎, 次郎
// 2年生: 花子, 美咲
```

- 第1引数：キーセレクター（グループ化の基準）
- 第2引数：要素セレクター（グループ内に保存する値）

## 🎯 型安全な groupBy の活用

TypeScriptの型推論を活用した例です。

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

type LogLevel = 'info' | 'warning' | 'error';

interface LogEntry {
  level: LogLevel;
  message: string;
  timestamp: number;
}

const logs: LogEntry[] = [
  { level: 'info', message: 'アプリ起動', timestamp: 1000 },
  { level: 'warning', message: '警告メッセージ', timestamp: 2000 },
  { level: 'error', message: 'エラー発生', timestamp: 3000 },
  { level: 'info', message: '処理完了', timestamp: 4000 },
  { level: 'error', message: '接続エラー', timestamp: 5000 },
];

from(logs).pipe(
  groupBy(log => log.level),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(entries => ({
        level: group.key,
        count: entries.length,
        messages: entries.map(e => e.message)
      }))
    )
  )
).subscribe(result => {
  console.log(`[${result.level.toUpperCase()}] ${result.count}件`);
  result.messages.forEach(msg => console.log(`  - ${msg}`));
});

// 出力:
// [INFO] 2件
//   - アプリ起動
//   - 処理完了
// [WARNING] 1件
//   - 警告メッセージ
// [ERROR] 2件
//   - エラー発生
//   - 接続エラー
```

## 🎯 グループごとに異なる処理を適用

各グループに対して異なる処理を適用する例です。

```ts
import { from, of } from 'rxjs';
import { groupBy, mergeMap, delay, map } from 'rxjs';

interface Task {
  id: number;
  priority: 'high' | 'medium' | 'low';
  name: string;
}

const tasks: Task[] = [
  { id: 1, priority: 'high', name: '緊急タスク' },
  { id: 2, priority: 'low', name: '後回しタスク' },
  { id: 3, priority: 'high', name: '重要タスク' },
  { id: 4, priority: 'medium', name: '通常タスク' },
];

from(tasks).pipe(
  groupBy(task => task.priority),
  mergeMap(group => {
    // 優先度に応じて遅延時間を設定
    const delayTime =
      group.key === 'high' ? 0 :
      group.key === 'medium' ? 1000 :
      2000;

    return group.pipe(
      delay(delayTime),
      map(task => ({ ...task, processedAt: Date.now() }))
    );
  })
).subscribe(task => {
  console.log(`[${task.priority}] ${task.name} を処理`);
});

// 出力（優先度順）:
// [high] 緊急タスク を処理
// [high] 重要タスク を処理
// （1秒後）
// [medium] 通常タスク を処理
// （さらに1秒後）
// [low] 後回しタスク を処理
```

## ⚠️ 注意点

### グループObservableの購読管理

`groupBy`は各グループに対してObservableを作成します。これらのObservableは適切に購読（subscribe）されないと、メモリリークの原因になります。

```ts
// ❌ 悪い例: グループObservableを購読しない
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd')
).subscribe(group => {
  // グループObservableを購読していない
  console.log('グループ:', group.key);
});
```

**対策**: 必ず`mergeMap`、`concatMap`、`switchMap`などで各グループを処理します。

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray } from 'rxjs';

// ✅ 良い例: 各グループを適切に処理
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd'),
  mergeMap(group =>
    group.pipe(toArray())
  )
).subscribe(console.log);
```

### グループの動的生成

新しいキーが出現するたびに新しいグループObservableが作成されます。キーの種類が多い場合は注意が必要です。

```ts
// キーの種類が無限に増える可能性がある例
fromEvent(document, 'click').pipe(
  groupBy(() => Math.random()) // 毎回異なるキー
).subscribe(); // メモリリークの危険
```

## 📚 関連オペレーター

- [`partition`](https://rxjs.dev/api/index/function/partition) - 条件で2つのObservableに分割
- [`reduce`](./reduce) - 最終的な集計結果を取得
- [`scan`](./scan) - 累積的な集計
- [`toArray`](../utility/toArray) - すべての値を配列にまとめる

## まとめ

`groupBy`オペレーターは、ストリームの値をキーに基づいてグループ化し、**各グループを個別のObservableとして扱う**ことができます。データの分類、カテゴリ別の集計、グループごとの異なる処理など、複雑なデータ処理に非常に便利です。ただし、各グループObservableは適切に購読する必要があり、通常は`mergeMap`などと組み合わせて使用します。
