---
description: RxJSのscheduled()関数を使って、スケジューラーを指定してObservableを生成し、実行タイミングを制御する方法を、実践的なコード例とともに詳しく解説します。
---

# scheduled()

[📘 RxJS公式ドキュメント - scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` は、配列・Promise・Iterableなどのデータソースから Observable を生成する際に、スケジューラーを明示的に指定できる Creation Function です。実行タイミング（同期・非同期）を細かく制御でき、テストやUIパフォーマンス最適化に役立ちます。

## 基本的な使い方

### シンプルな配列の Observable 化

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// 配列を非同期で発行
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('購読開始');
observable$.subscribe({
  next: val => console.log('値:', val),
  complete: () => console.log('完了')
});
console.log('購読終了');

// 出力:
// 購読開始
// 購読終了
// 値: 1
// 値: 2
// 値: 3
// 完了
```

> [!IMPORTANT]
> **同期 vs 非同期の違い**
>
> `asyncScheduler` を使用すると、値の発行が非同期になります。そのため、「購読開始」→「購読終了」→「値: 1」の順で出力されます。

### from() との比較

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - デフォルトは同期的
console.log('=== from() ===');
from([1, 2, 3]).subscribe(val => console.log('値:', val));
console.log('購読終了');

// 出力:
// === from() ===
// 値: 1
// 値: 2
// 値: 3
// 購読終了

// scheduled() - 明示的に非同期
console.log('=== scheduled() ===');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('値:', val));
console.log('購読終了');

// 出力:
// === scheduled() ===
// 購読終了
// 値: 1
// 値: 2
// 値: 3
```

## スケジューラーの種類

RxJS には複数のスケジューラーが用意されており、用途に応じて使い分けます。

| スケジューラー | 実行タイミング | ベース技術 | 主な用途 |
|---------------|--------------|-----------|---------|
| `queueScheduler` | 同期（キュー方式） | 即座に実行 | デフォルト、同期的な処理 |
| `asyncScheduler` | 非同期 | `setTimeout` | UI最適化、長時間処理 |
| `asapScheduler` | 最速の非同期 | `Promise` (microtask) | 高優先度の非同期処理 |
| `animationFrameScheduler` | アニメーションフレーム | `requestAnimationFrame` | アニメーション、UI描画 |

### queueScheduler（同期実行）

```typescript
import { scheduled, queueScheduler } from 'rxjs';

console.log('開始');
scheduled([1, 2, 3], queueScheduler).subscribe(val => console.log('値:', val));
console.log('終了');

// 出力:
// 開始
// 値: 1
// 値: 2
// 値: 3
// 終了
```

### asyncScheduler（非同期実行）

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

console.log('開始');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('値:', val));
console.log('終了');

// 出力:
// 開始
// 終了
// 値: 1
// 値: 2
// 値: 3
```

### asapScheduler（マイクロタスク）

```typescript
import { scheduled, asapScheduler } from 'rxjs';

console.log('開始');
scheduled([1, 2, 3], asapScheduler).subscribe(val => console.log('値:', val));
console.log('終了');

// 出力:
// 開始
// 終了
// 値: 1
// 値: 2
// 値: 3
```

> [!TIP]
> **asyncScheduler vs asapScheduler**
>
> - `asyncScheduler`: `setTimeout` ベース（マクロタスク）
> - `asapScheduler`: `Promise` ベース（マイクロタスク）
>
> `asapScheduler` の方が早く実行されますが、両方とも非同期です。

### animationFrameScheduler（アニメーション）

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// アニメーションフレームごとに値を更新
const positions = [0, 50, 100, 150, 200];
const animation$ = scheduled(positions, animationFrameScheduler).pipe(
  map(pos => `位置: ${pos}px`)
);

animation$.subscribe(position => {
  console.log(position);
  // DOMの更新をここで行う
});

// 出力: (各アニメーションフレームで)
// 位置: 0px
// 位置: 50px
// 位置: 100px
// 位置: 150px
// 位置: 200px
```

## 実践的なパターン

### UIをブロックしない大量データ処理

```typescript
import { scheduled, asyncScheduler, map, bufferCount } from 'rxjs';
// 100万件のデータを処理
const largeArray = Array.from({ length: 1000000 }, (_, i) => i);

// ❌ 悪い例: 同期的に処理（UIがブロックされる）
// from(largeArray).subscribe(processData);

// ✅ 良い例: 非同期で処理（UIがブロックされない）
scheduled(largeArray, asyncScheduler).pipe(
  bufferCount(1000), // 1000件ずつバッチ処理
  map(batch => batch.reduce((sum, val) => sum + val, 0))
).subscribe({
  next: sum => console.log('バッチ合計:', sum),
  complete: () => console.log('処理完了')
});

console.log('UIは引き続き応答可能');
```

### Promiseとの組み合わせ

```typescript
import { scheduled, asyncScheduler, mergeMap } from 'rxjs';
interface User {
  id: number;
  name: string;
}

const userIds = [1, 2, 3, 4, 5];

// 複数のユーザーを非同期で取得
scheduled(userIds, asyncScheduler).pipe(
  mergeMap(id =>
    fetch(`https://api.example.com/users/${id}`).then(res => res.json())
  )
).subscribe({
  next: (user: User) => console.log('ユーザー:', user),
  error: error => console.error('エラー:', error),
  complete: () => console.log('全ユーザー取得完了')
});
```

### Iterableからの生成

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Setをスケジュール付きで変換
const uniqueNumbers = new Set([1, 2, 3, 4, 5]);
const observable$ = scheduled(uniqueNumbers, asyncScheduler);

observable$.subscribe({
  next: val => console.log('値:', val),
  complete: () => console.log('完了')
});

// Mapをスケジュール付きで変換
const userMap = new Map([
  [1, 'Alice'],
  [2, 'Bob'],
  [3, 'Charlie']
]);

scheduled(userMap, asyncScheduler).subscribe({
  next: ([id, name]) => console.log(`ID: ${id}, 名前: ${name}`),
  complete: () => console.log('完了')
});
```

## テストでの活用

`scheduled()` は、TestScheduler と組み合わせることで、時間制御が可能なテストを書けます。

### 基本的なテスト

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled } from 'rxjs';

describe('scheduled()', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('配列を順番に発行する', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler);
      const expected = '(abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

### 非同期処理のテスト

```typescript
import { scheduled, asyncScheduler, delay } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('非同期処理のテスト', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('遅延処理を仮想的にテスト', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler).pipe(
        delay(1000, testScheduler)
      );

      // 1000ms後に発行（仮想時間）
      const expected = '1000ms (abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

> [!TIP]
> **TestScheduler の利点**
>
> - 実際に時間を待たずにテストできる
> - 非同期処理を同期的にテストできる
> - テストの実行時間を大幅に短縮

## よくある使用例

### 1. ページネーション付きデータ取得

```typescript
import { scheduled, asyncScheduler, mergeMap, toArray } from 'rxjs';
interface Page {
  page: number;
  data: any[];
}

// ページ番号のリスト
const pages = [1, 2, 3, 4, 5];

// 各ページを非同期で取得
const allData$ = scheduled(pages, asyncScheduler).pipe(
  mergeMap(page =>
    fetch(`https://api.example.com/items?page=${page}`)
      .then(res => res.json())
  ),
  toArray() // 全ページのデータをまとめる
);

allData$.subscribe({
  next: data => console.log('全データ:', data),
  complete: () => console.log('取得完了')
});
```

### 2. バッチ処理

```typescript
import { scheduled, asyncScheduler, bufferCount, mergeMap, delay } from 'rxjs';
// 大量のタスクを1000件ずつ処理
const tasks = Array.from({ length: 10000 }, (_, i) => `Task-${i}`);

scheduled(tasks, asyncScheduler).pipe(
  bufferCount(1000), // 1000件ずつバッチ化
  mergeMap(batch => {
    console.log(`バッチ処理中: ${batch.length}件`);
    // バッチ処理を実行
    return processBatch(batch);
  })
).subscribe({
  complete: () => console.log('全バッチ処理完了')
});

function processBatch(batch: string[]): Promise<void> {
  // バッチ処理のロジック
  return Promise.resolve();
}
```

### 3. アニメーションの実装

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// 0から100までの値を生成
const frames = Array.from({ length: 100 }, (_, i) => i);

// アニメーションフレームごとに実行
const animation$ = scheduled(frames, animationFrameScheduler).pipe(
  map(frame => ({
    progress: frame / 100,
    position: frame * 5 // 0pxから500pxまで移動
  }))
);

animation$.subscribe({
  next: ({ progress, position }) => {
    const element = document.getElementById('animated-box');
    if (element) {
      element.style.transform = `translateX(${position}px)`;
      console.log(`進捗: ${(progress * 100).toFixed(0)}%`);
    }
  },
  complete: () => console.log('アニメーション完了')
});
```

### 4. 優先度付きタスク処理

```typescript
import { scheduled, asapScheduler, asyncScheduler } from 'rxjs';

// 高優先度タスク（asapScheduler = マイクロタスク）
const highPriorityTasks = ['緊急タスク1', '緊急タスク2'];
const highPriority$ = scheduled(highPriorityTasks, asapScheduler);

// 低優先度タスク（asyncScheduler = マクロタスク）
const lowPriorityTasks = ['通常タスク1', '通常タスク2'];
const lowPriority$ = scheduled(lowPriorityTasks, asyncScheduler);

console.log('タスク開始');

highPriority$.subscribe(task => console.log('高優先:', task));
lowPriority$.subscribe(task => console.log('低優先:', task));

console.log('タスク登録完了');

// 出力:
// タスク開始
// タスク登録完了
// 高優先: 緊急タスク1
// 高優先: 緊急タスク2
// 低優先: 通常タスク1
// 低優先: 通常タスク2
```

## scheduled() のオプション

`scheduled()` は以下のシグネチャを持ちます。

```typescript
function scheduled<T>(
  input: ObservableInput<T>,
  scheduler: SchedulerLike
): Observable<T>
```

### 対応する入力型

- **配列**: `T[]`
- **Promise**: `Promise<T>`
- **Iterable**: `Iterable<T>` (Set, Map, Generator など)
- **Observable**: `Observable<T>`
- **ArrayLike**: `ArrayLike<T>`

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// 配列
scheduled([1, 2, 3], asyncScheduler);

// Promise
scheduled(Promise.resolve('結果'), asyncScheduler);

// Set
scheduled(new Set([1, 2, 3]), asyncScheduler);

// Generator
function* generator() {
  yield 1;
  yield 2;
  yield 3;
}
scheduled(generator(), asyncScheduler);
```

## よくあるエラーと対処法

### 1. スケジューラーの指定忘れ

**エラー例:**
```typescript
// ❌ エラー: 第2引数が必要
const observable$ = scheduled([1, 2, 3]);
```

**対処法:**
```typescript
// ✅ 正しい: スケジューラーを指定
const observable$ = scheduled([1, 2, 3], asyncScheduler);
```

### 2. ブラウザ環境でのanimationFrameScheduler使用

**問題:**
Node.js環境では `requestAnimationFrame` が存在しないため、エラーになります。

**対処法:**
```typescript
import { scheduled, animationFrameScheduler, asyncScheduler } from 'rxjs';

// ブラウザ環境かチェック
const scheduler = typeof window !== 'undefined'
  ? animationFrameScheduler
  : asyncScheduler;

const observable$ = scheduled([1, 2, 3], scheduler);
```

### 3. 同期処理と非同期処理の混同

**問題:**
```typescript
// 非同期で実行されることを期待しているが、実際は同期的
scheduled([1, 2, 3], queueScheduler).subscribe(val => {
  console.log(val);
});
console.log('完了'); // ← この前に1, 2, 3が出力される
```

**対処法:**
```typescript
// 明確に非同期を指定
scheduled([1, 2, 3], asyncScheduler).subscribe(val => {
  console.log(val);
});
console.log('完了'); // ← この後に1, 2, 3が出力される
```

## from() との比較

| 特徴 | from() | scheduled() |
|------|--------|-------------|
| スケジューラー指定 | ❌ 不可（デフォルトのみ） | ✅ 明示的に指定可能 |
| 同期・非同期制御 | ❌ 制御不可 | ✅ 制御可能 |
| テスト容易性 | 普通 | ✅ TestSchedulerで時間制御可能 |
| シンプルさ | ✅ シンプル | やや複雑 |
| 使用場面 | 基本的な変換 | 実行タイミング制御が必要な場合 |

> [!TIP]
> **使い分けのポイント**
>
> - **基本的には `from()` を使う**: スケジューラー制御が不要な場合
> - **`scheduled()` を使う場合**:
>   - UI のブロッキングを避けたい
>   - テストで時間制御が必要
>   - アニメーションの実装
>   - 優先度付きタスク処理

## ベストプラクティス

### 1. 大量データ処理では asyncScheduler を使用

```typescript
// ✅ 良い例: UIをブロックしない
scheduled(largeArray, asyncScheduler).pipe(
  map(processHeavyTask)
).subscribe();
```

### 2. テストでは TestScheduler を使用

```typescript
// ✅ 良い例: 時間を仮想的に制御
testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

### 3. アニメーションでは animationFrameScheduler を使用

```typescript
// ✅ 良い例: ブラウザの再描画タイミングに合わせる
scheduled(frames, animationFrameScheduler).subscribe(updateUI);
```

### 4. 環境に応じたスケジューラー選択

```typescript
// ✅ 良い例: 環境に応じて切り替え
const scheduler = process.env.NODE_ENV === 'test'
  ? queueScheduler
  : asyncScheduler;

const source$ = scheduled(data, scheduler);
```

## まとめ

`scheduled()` は、スケジューラーを明示的に指定して Observable を生成する Creation Function です。

**主な特徴:**
- 実行タイミング（同期・非同期）を明示的に制御
- 複数のスケジューラーから選択可能
- TestScheduler でテストが容易
- UIのブロッキング回避に有効

**使用場面:**
- 大量データの非同期処理
- アニメーションの実装
- テストでの時間制御
- 優先度付きタスク処理

**注意点:**
- 必ずスケジューラーを指定する
- 環境に応じて適切なスケジューラーを選択
- from() との使い分けを理解する

**推奨される使い方:**
- UI最適化: `asyncScheduler`
- アニメーション: `animationFrameScheduler`
- テスト: `TestScheduler`
- 高優先度: `asapScheduler`

## 関連ページ

- [using()](/guide/creation-functions/control/using) - リソース制御付きObservable
- [制御系 Creation Functions](/guide/creation-functions/control/) - scheduled() と using() の比較
- [スケジューラーの種類](/guide/schedulers/types) - スケジューラーの詳細
- [from()](/guide/creation-functions/basic/from) - 基本的なObservable生成

## 参考リソース

- [RxJS公式ドキュメント - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [RxJS公式ドキュメント - Scheduler](https://rxjs.dev/guide/scheduler)
- [RxJS公式ドキュメント - TestScheduler](https://rxjs.dev/api/testing/TestScheduler)
