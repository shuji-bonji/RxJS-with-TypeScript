---
description: findIndexオペレーターは、条件を満たす最初の値のインデックスを返すRxJSフィルタリングオペレーターです。見つからない場合は-1を返します。
---

# findIndex - 条件に一致するインデックスを取得

`findIndex` オペレーターは、**条件を満たす最初の値のインデックス**を返し、即座にストリームを完了させます。値が見つからない場合は `-1` を返します。

## 🔰 基本構文と使い方

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// 出力: 4（最初の偶数8のインデックス）
```

**動作の流れ**:
1. 1（インデックス0）→ 奇数、スキップ
2. 3（インデックス1）→ 奇数、スキップ
3. 5（インデックス2）→ 奇数、スキップ
4. 7（インデックス3）→ 奇数、スキップ
5. 8（インデックス4）→ 偶数、インデックス4を出力して完了

[🌐 RxJS公式ドキュメント - `findIndex`](https://rxjs.dev/api/operators/findIndex)

## 💡 典型的な活用パターン

- **配列内の位置特定**：特定条件を満たす要素の位置を取得
- **順序の確認**：ある条件を満たす要素が何番目に現れるか
- **データの並び替え**：インデックス情報を使った処理
- **存在チェック**：-1かどうかで存在確認

## 🧠 実践コード例1: タスクリストの検索

タスクリストから特定条件のタスクの位置を見つける例です。

```ts
import { from, fromEvent } from 'rxjs';
import { findIndex } from 'rxjs';

interface Task {
  id: number;
  title: string;
  priority: 'high' | 'medium' | 'low';
  completed: boolean;
}

const tasks: Task[] = [
  { id: 1, title: 'メール返信', priority: 'low', completed: true },
  { id: 2, title: '資料作成', priority: 'medium', completed: true },
  { id: 3, title: '会議準備', priority: 'high', completed: false },
  { id: 4, title: 'コードレビュー', priority: 'high', completed: false },
  { id: 5, title: 'ドキュメント更新', priority: 'low', completed: false }
];

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'タスク検索';
container.appendChild(title);

// タスクリスト表示
const taskList = document.createElement('ul');
taskList.style.listStyle = 'none';
taskList.style.padding = '0';
tasks.forEach((task, index) => {
  const li = document.createElement('li');
  li.style.padding = '5px';
  li.style.borderBottom = '1px solid #eee';
  const status = task.completed ? '✅' : '⬜';
  const priorityBadge = task.priority === 'high' ? '🔴' : task.priority === 'medium' ? '🟡' : '🟢';
  li.textContent = `[${index}] ${status} ${priorityBadge} ${task.title}`;
  taskList.appendChild(li);
});
container.appendChild(taskList);

// 検索ボタン
const buttonContainer = document.createElement('div');
buttonContainer.style.marginTop = '10px';
container.appendChild(buttonContainer);

const button1 = document.createElement('button');
button1.textContent = '最初の未完了タスクを検索';
button1.style.marginRight = '10px';
buttonContainer.appendChild(button1);

const button2 = document.createElement('button');
button2.textContent = '最初の高優先度タスクを検索';
buttonContainer.appendChild(button2);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// 最初の未完了タスクを検索
// 注: 本来は switchMap で平坦化するのが推奨パターンですが、
// ここでは可読性優先で subscribe をネストしています（本番コードでは switchMap を推奨）。
fromEvent(button1, 'click').subscribe(() => {
  // ネスト subscribe: 本来は switchMap での平坦化を推奨
  from(tasks).pipe(
    findIndex(task => !task.completed)
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ 見つかりました</strong><br>
        位置: インデックス ${index}<br>
        タスク: ${task.title}<br>
        優先度: ${task.priority}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ 未完了タスクは見つかりませんでした';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});

// 最初の高優先度タスクを検索
// 注: 本来は switchMap で平坦化するのが推奨パターン（本番コードでは switchMap を推奨）。
fromEvent(button2, 'click').subscribe(() => {
  // ネスト subscribe: 本来は switchMap での平坦化を推奨
  from(tasks).pipe(
    findIndex(task => task.priority === 'high')
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ 見つかりました</strong><br>
        位置: インデックス ${index}<br>
        タスク: ${task.title}<br>
        完了状態: ${task.completed ? '完了' : '未完了'}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ 高優先度タスクは見つかりませんでした';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});
```

- タスクリストから条件を満たす最初のタスクの位置を検索します。
- 見つからない場合は `-1` が返されます。

## 🎯 実践コード例2: リアルタイムデータの位置検出

ストリームから条件を満たす最初の値の位置を検出する例です。

```ts
import { interval } from 'rxjs';
import { findIndex, map, take } from 'rxjs';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'リアルタイムデータ検索';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = '50以上の値が出現する位置を検索中...';
container.appendChild(status);

const dataDisplay = document.createElement('div');
dataDisplay.style.marginTop = '10px';
dataDisplay.style.padding = '10px';
dataDisplay.style.border = '1px solid #ccc';
dataDisplay.style.maxHeight = '150px';
dataDisplay.style.overflow = 'auto';
container.appendChild(dataDisplay);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.fontWeight = 'bold';
container.appendChild(result);

// ランダムな値を生成（0〜100）
const data$ = interval(500).pipe(
  take(20),
  map(i => ({ index: i, value: Math.floor(Math.random() * 100) }))
);

// データ表示
data$.subscribe(data => {
  const div = document.createElement('div');
  const highlight = data.value >= 50 ? 'background-color: #fff9c4;' : '';
  div.style.cssText = `padding: 5px; ${highlight}`;
  div.textContent = `[${data.index}] 値: ${data.value}`;
  dataDisplay.appendChild(div);
  dataDisplay.scrollTop = dataDisplay.scrollHeight;
});

// 50以上の最初の値のインデックスを検索
data$.pipe(
  findIndex(data => data.value >= 50)
).subscribe(index => {
  status.textContent = '';
  if (index !== -1) {
    result.innerHTML = `
      ✅ 50以上の値が見つかりました<br>
      位置: インデックス ${index}
    `;
    result.style.color = 'green';
  } else {
    result.textContent = '❌ 50以上の値は見つかりませんでした';
    result.style.color = 'orange';
  }
});
```

- 0.5秒ごとに生成されるランダム値から、50以上の最初の値の位置を検出します。
- ハイライト表示で視覚的にわかりやすくしています。

## 🆚 類似オペレーターとの比較

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: 条件を満たす最初の値のインデックスを返す
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// 出力: 2（30のインデックス）

// find: 条件を満たす最初の値を返す
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// 出力: 30

// elementAt: 指定したインデックスの値を返す
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// 出力: 30
```

| オペレーター | 引数 | 返り値 | 見つからない場合 |
|:---|:---|:---|:---|
| `findIndex(predicate)` | 条件関数 | インデックス（数値） | `-1` |
| `find(predicate)` | 条件関数 | 値そのもの | `undefined` |
| `elementAt(index)` | インデックス | 値そのもの | エラー（デフォルト値なし） |

## 🔄 JavaScript の Array.findIndex() との比較

RxJS の `findIndex` は JavaScript の配列メソッド `Array.prototype.findIndex()` と似た動作をします。

```ts
// JavaScript の配列
const numbers = [10, 20, 30, 40, 50];
const index1 = numbers.findIndex(n => n > 25);
console.log(index1); // 2

// RxJS の Observable
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log); // 2
```

**主な違い**:
- **配列**: 同期的に即座に結果を返す
- **Observable**: 非同期で、ストリームから値が流れてくるのを待つ

## ⚠️ 注意点

### 1. 見つからない場合は -1 を返す

条件を満たす値がない場合、エラーではなく `-1` を返します。

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('条件を満たす値は見つかりませんでした');
  } else {
    console.log(`インデックス: ${index}`);
  }
});
// 出力: 条件を満たす値は見つかりませんでした
```

### 2. 最初に見つかった時点で完了

条件を満たす最初の値が見つかると、即座にストリームが完了します。

```ts
import { interval } from 'rxjs';
import { findIndex, tap } from 'rxjs';

interval(1000).pipe(
  tap(val => console.log(`値: ${val}`)),
  findIndex(n => n >= 3)
).subscribe(index => {
  console.log(`インデックス: ${index}`);
});
// 出力:
// 値: 0
// 値: 1
// 値: 2
// 値: 3
// インデックス: 3
```

### 3. TypeScript での型安全性

`findIndex` は常に `number` 型を返します。

```ts
import { Observable, from } from 'rxjs';
import { findIndex } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

function findFirstInactiveUserIndex(
  users$: Observable<User>
): Observable<number> {
  return users$.pipe(
    findIndex(user => !user.isActive)
  );
}

const users$ = from([
  { id: 1, name: 'Alice', isActive: true },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true }
]);

findFirstInactiveUserIndex(users$).subscribe(index => {
  // index は number 型
  if (index !== -1) {
    console.log(`最初の非アクティブユーザーはインデックス ${index} です`);
  }
});
// 出力: 最初の非アクティブユーザーはインデックス 1 です
```

### 4. インデックスは 0 から始まる

配列と同じく、インデックスは 0 から始まります。

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// 出力: 0（最初の要素）
```

## 📚 関連オペレーター

- **[find](./find)** - 条件を満たす最初の値を取得
- **[elementAt](./elementAt)** - 指定したインデックスの値を取得
- **[first](./first)** - 最初の値を取得
- **[filter](./filter)** - 条件を満たすすべての値を取得

## まとめ

`findIndex` オペレーターは、条件を満たす最初の値のインデックスを返します。

- ✅ JavaScript の `Array.findIndex()` と似た動作
- ✅ インデックス情報が必要な場合に最適
- ✅ 見つからない場合は `-1` を返す（エラーではない）
- ✅ 見つかった時点で即座に完了
- ⚠️ 返り値は常に `number` 型（-1 または 0 以上の整数）
- ⚠️ 値そのものが必要な場合は `find` を使用
