---
description: partitionは、1つのObservableを条件に基づいて2つのObservableに分割するRxJSのCreation Functionです。成功/失敗、有効/無効などの二分岐処理に最適です。
---

# partition - 条件で2つのストリームに分割する

`partition` は、1つのObservableを条件に基づいて**2つのObservableに分割**するCreation Functionです。
述語関数（predicate）で条件を指定し、条件を満たす値と満たさない値を別々のストリームとして取得できます。

## 🔰 基本構文と使い方

```ts
import { partition, of } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// 偶数と奇数に分割
const [evens$, odds$] = partition(source$, (value) => value % 2 === 0);

evens$.subscribe((value) => console.log('偶数:', value));
// 出力: 偶数: 2, 偶数: 4, 偶数: 6

odds$.subscribe((value) => console.log('奇数:', value));
// 出力: 奇数: 1, 奇数: 3, 奇数: 5
```

- `partition`は2つのObservableを含む**配列を返します**
- `[0]`: 条件を満たす値のストリーム
- `[1]`: 条件を満たさない値のストリーム

[🌐 RxJS公式ドキュメント - `partition`](https://rxjs.dev/api/index/function/partition)

## 💡 典型的な活用パターン

- **成功/失敗の分岐処理**（HTTPステータスコードによる振り分け）
- **イベントの振り分け**（左クリック/右クリック）
- **データの分類**（有効/無効、大人/子供など）
- **条件に基づくストリーム分割**

## 🧠 実践コード例（UI付き）

ボタンをクリックすると、クリック座標が画面の左半分か右半分かで処理を分岐します。

```ts
import { partition, fromEvent } from 'rxjs';
import { map } from 'rxjs';

// 出力エリア作成
const leftArea = document.createElement('div');
leftArea.innerHTML = '<h3>左クリック</h3><ul id="left-list"></ul>';
leftArea.style.float = 'left';
leftArea.style.width = '45%';
leftArea.style.padding = '10px';
leftArea.style.background = '#e3f2fd';
document.body.appendChild(leftArea);

const rightArea = document.createElement('div');
rightArea.innerHTML = '<h3>右クリック</h3><ul id="right-list"></ul>';
rightArea.style.float = 'right';
rightArea.style.width = '45%';
rightArea.style.padding = '10px';
rightArea.style.background = '#fce4ec';
document.body.appendChild(rightArea);

// クリックイベント
const clicks$ = fromEvent<MouseEvent>(document, 'click');

// 画面の中央X座標
const centerX = window.innerWidth / 2;

// 左半分と右半分に分割
const [leftClicks$, rightClicks$] = partition(
  clicks$,
  (event) => event.clientX < centerX
);

// 左クリックを処理
leftClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const leftList = document.getElementById('left-list')!;
  const li = document.createElement('li');
  li.textContent = `座標: (${pos.x}, ${pos.y})`;
  leftList.appendChild(li);
});

// 右クリックを処理
rightClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const rightList = document.getElementById('right-list')!;
  const li = document.createElement('li');
  li.textContent = `座標: (${pos.x}, ${pos.y})`;
  rightList.appendChild(li);
});
```

- 画面をクリックすると、クリック位置に応じて左右のリストに記録されます。
- 1つのソースから2つの独立したストリームを作成できます。

## 📊 実用例：APIレスポンスの分岐処理

HTTPステータスコードで成功と失敗を分割する例

```ts
import { partition, from, of } from 'rxjs';
import { mergeMap, map, catchError, share } from 'rxjs';

interface ApiResponse {
  status: number;
  data?: any;
  error?: string;
}

// ダミーAPI呼び出し
const apiCalls$ = from([
  fetch('/api/users/1'),
  fetch('/api/users/999'), // 存在しないユーザー
  fetch('/api/users/2'),
]);

// Responseを処理してApiResponseに変換
const responses$ = apiCalls$.pipe(
  mergeMap(fetchPromise => from(fetchPromise)),
  mergeMap(response =>
    from(response.json()).pipe(
      map(data => ({
        status: response.status,
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : (data.message || 'Error')
      } as ApiResponse)),
      catchError(err => of({
        status: response.status,
        data: undefined,
        error: err.message || 'Failed to parse response'
      } as ApiResponse))
    )
  ),
  share() // partitionの2回購読に対応
);

// 成功（200番台）と失敗（その他）に分割
const [success$, failure$] = partition(
  responses$,
  (response: ApiResponse) => response.status >= 200 && response.status < 300
);

// 成功レスポンスを処理
success$.subscribe((response) => {
  console.log('✅ 成功:', response.data);
  // UIに成功データを表示
});

// 失敗レスポンスを処理
failure$.subscribe((response) => {
  console.error('❌ 失敗:', response.error);
  // エラーメッセージを表示
});
```

## 🆚 filterとの比較

### 基本的な違い

| 方法 | 説明 | 出力 | ユースケース |
|------|------|------|--------------|
| `partition` | 1つのソースを2つのストリームに分割 | 2つのObservable | 両方のストリームを**同時に**使いたい場合 |
| `filter` | 条件を満たす値のみを通過させる | 1つのObservable | 1つのストリームだけが必要な場合 |

### 使い分けの具体例

**両方のストリームを同時に処理する場合は partition を使用**

```ts
import { partition, interval } from 'rxjs';
import { map, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ 成功</h4><ul id="success-list"></ul>';
successArea.style.float = 'left';
successArea.style.width = '45%';
output.appendChild(successArea);

const failureArea = document.createElement('div');
failureArea.innerHTML = '<h4 style="color: red;">❌ 失敗</h4><ul id="failure-list"></ul>';
failureArea.style.float = 'right';
failureArea.style.width = '45%';
output.appendChild(failureArea);

// ランダムな成功/失敗のストリーム
const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `タスク${i + 1}`
  }))
);

// ✅ partition - 成功と失敗を同時に処理
const [success$, failure$] = partition(tasks$, task => task.success);

success$.subscribe(task => {
  const successList = document.getElementById('success-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  successList.appendChild(li);
});

failure$.subscribe(task => {
  const failureList = document.getElementById('failure-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  failureList.appendChild(li);
});
```

**1つのストリームだけが必要な場合は filter を使用**

```ts
import { interval } from 'rxjs';
import { map, take, filter } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ 成功のみ表示</h4><ul id="success-only"></ul>';
output.appendChild(successArea);

const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `タスク${i + 1}`
  }))
);

// ✅ filter - 成功のみを処理（失敗は無視）
tasks$
  .pipe(filter(task => task.success))
  .subscribe(task => {
    const successList = document.getElementById('success-only')!;
    const li = document.createElement('li');
    li.textContent = task.message;
    successList.appendChild(li);
  });
```

**filter を2回使う vs partition の比較**

```ts
import { of } from 'rxjs';
import { filter } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// ❌ filter を2回使う - ソースが2回実行される可能性がある
const evens1$ = numbers$.pipe(filter(n => n % 2 === 0));
const odds1$ = numbers$.pipe(filter(n => n % 2 !== 0));

evens1$.subscribe(n => console.log('偶数:', n));
odds1$.subscribe(n => console.log('奇数:', n));
// 問題: numbers$がcold observableの場合、2回実行される

// ✅ partition を使う - 1回の実行で両方のストリームを作成
const [evens2$, odds2$] = partition(numbers$, n => n % 2 === 0);

evens2$.subscribe(n => console.log('偶数:', n));
odds2$.subscribe(n => console.log('奇数:', n));
// 利点: 1つのソースから効率的に2つのストリームを作成
```

**パイプライン内で分岐したい場合は filter を使用**

```ts
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

interface User {
  id: number;
  name: string;
  age: number;
  isActive: boolean;
}

const users$ = from([
  { id: 1, name: 'Alice', age: 25, isActive: true },
  { id: 2, name: 'Bob', age: 30, isActive: false },
  { id: 3, name: 'Carol', age: 35, isActive: true }
]);

// ❌ partition は Creation Function のため、パイプライン内で使えない
// users$.pipe(
//   map(user => user.name),
//   partition(name => name.startsWith('A')) // エラー
// );

// ✅ filter を使う - パイプライン内で使用可能
users$
  .pipe(
    filter(user => user.isActive),  // アクティブなユーザーのみ
    map(user => user.name)           // 名前を抽出
  )
  .subscribe(console.log);
// 出力: Alice, Carol
```

### まとめ

| 状況 | 推奨される方法 | 理由 |
|------|--------------|------|
| 成功と失敗を**両方とも**処理したい | `partition` | 1回のソース実行で2つのストリームを作成できる |
| 成功**だけ**を処理したい | `filter` | シンプルで分かりやすい |
| パイプライン内で条件分岐したい | `filter` | `partition`はCreation Functionのため使えない |
| 複雑な条件で3つ以上に分岐したい | `groupBy` | 複数のグループに分割できる |

## ⚠️ 注意点

### 1. 両方のストリームを購読する

`partition`で作成した2つのObservableは、**元のソースを共有**します。
両方を購読しないと、元のストリームが完全に処理されない可能性があります。

```ts
const [success$, failure$] = partition(source$, predicate);

// 両方を購読
success$.subscribe(handleSuccess);
failure$.subscribe(handleFailure);
```

### 2. ソースは2回実行される

`partition`は内部的に元のソースを2回購読します。
副作用がある場合は注意が必要です。

```ts
let callCount = 0;
const source$ = new Observable(observer => {
  callCount++;
  console.log(`購読回数: ${callCount}`);
  observer.next(1);
  observer.complete();
});

const [a$, b$] = partition(source$, n => n > 0);
a$.subscribe(); // 購読回数: 1
b$.subscribe(); // 購読回数: 2
```

副作用を避けるには、`share()`を使用します。

```ts
import { share } from 'rxjs';

const shared$ = source$.pipe(share());
const [a$, b$] = partition(shared$, n => n > 0);
```

### 3. Pipeable Operatorとして提供されていない

RxJS 7以降、`partition`は**Creation Functionのみ**で提供されています。
パイプライン内では使用できません。

```ts
// ❌ 不可能
source$.pipe(
  partition(n => n % 2 === 0) // エラー
);

// ✅ 正しい使い方
const [evens$, odds$] = partition(source$, n => n % 2 === 0);
```

## 💡 代替パターン

パイプライン内で分岐したい場合は、`filter`を使用します。

```ts
const source$ = of(1, 2, 3, 4, 5, 6);

const evens$ = source$.pipe(filter(n => n % 2 === 0));
const odds$ = source$.pipe(filter(n => n % 2 !== 0));

// または、shareでソースを共有
const shared$ = source$.pipe(share());
const evens$ = shared$.pipe(filter(n => n % 2 === 0));
const odds$ = shared$.pipe(filter(n => n % 2 !== 0));
```

## 🔗 関連オペレーター

- [`filter`](../operators/filtering/filter.md) - 条件を満たす値のみを通過
- [`groupBy`](../operators/transformation/groupBy.md) - 複数のグループに分割
- [`share`](../operators/multicasting/share.md) - ソースの共有

## 📝 まとめ

`partition`は、1つのObservableを条件に基づいて2つに分割する強力なツールです。

- ✅ 成功/失敗の分岐処理に最適
- ✅ 2つの独立したストリームを作成
- ⚠️ ソースは2回購読される（副作用に注意）
- ⚠️ Pipeable Operatorとしては提供されていない
