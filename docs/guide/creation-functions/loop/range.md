---
description: "range() - 指定した開始値から連続する整数を順番に発行するCreation Function。for文の代わりとなる宣言的な連番生成方法です。generate()との使い分け、TypeScriptでの型推論を活用した実装、map()との組み合わせパターンを実践的なコード例で解説します。"
---

# range() - 数値範囲の生成

`range()`は、指定した開始値から指定個数の連続する整数を発行する、for文的なCreation Functionです。

## 概要

`range()`は開始値と個数を指定して、連続する整数をObservableとして発行します。従来の`for`文を置き換える宣言的な方法として、連番生成やバッチ処理に使用されます。

**シグネチャ**:
```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

**パラメーター**:
- `start`: 開始値（この値から発行開始）
- `count`: 発行する値の個数（省略時は0から`start`未満まで）
- `scheduler`: 値を発行するスケジューラー（省略時は同期的に発行）

**公式ドキュメント**: [📘 RxJS公式: range()](https://rxjs.dev/api/index/function/range)

## 基本的な使い方

### パターン1: 開始値と個数を指定

最も一般的な使用方法です。

```typescript
import { range } from 'rxjs';

// 1から5個の連番を生成（1, 2, 3, 4, 5）
range(1, 5).subscribe({
  next: value => console.log('値:', value),
  complete: () => console.log('完了')
});

// 出力:
// 値: 1
// 値: 2
// 値: 3
// 値: 4
// 値: 5
// 完了
```

### パターン2: 0から始まる連番

開始値を0にすることで、配列のインデックスのような連番を生成できます。

```typescript
import { range } from 'rxjs';

// 0から10個の連番（0, 1, 2, ..., 9）
range(0, 10).subscribe(console.log);
// 出力: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

### パターン3: 負の数からスタート

負の数からも生成可能です。

```typescript
import { range } from 'rxjs';

// -3から5個の連番（-3, -2, -1, 0, 1）
range(-3, 5).subscribe(console.log);
// 出力: -3, -2, -1, 0, 1
```

## 重要な特徴

### 1. 同期的に発行

`range()`はデフォルトでは購読と同時に、すべての値を**同期的に**発行します。

```typescript
import { range } from 'rxjs';

console.log('購読前');

range(1, 3).subscribe(value => console.log('値:', value));

console.log('購読後');

// 出力:
// 購読前
// 値: 1
// 値: 2
// 値: 3
// 購読後
```

### 2. 即座に完了

すべての値を発行後、即座に`complete`を通知します。

```typescript
import { range } from 'rxjs';

range(1, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('完了！')
});

// 出力: 1, 2, 3, 完了！
```

### 3. for文との等価性

`range(start, count)`は以下のfor文と等価です。

```typescript
// 命令的なfor文
for (let i = start; i < start + count; i++) {
  console.log(i);
}

// 宣言的なrange()
range(start, count).subscribe(console.log);
```

## 実践的なユースケース

### 1. バッチ処理

複数のタスクを順次実行する際に使用します。

```typescript
import { range, of, Observable, concatMap, delay, map } from 'rxjs';
// データ処理をシミュレートする関数
function processItem(index: number): Observable<string> {
  return of(index).pipe(
    delay(100), // 100msの処理時間をシミュレート
    map(i => `アイテム${i}の処理結果`)
  );
}

// 10件のデータを順次処理（各処理間に1秒の遅延）
range(1, 10).pipe(
  concatMap(index =>
    processItem(index).pipe(delay(1000))
  )
).subscribe({
  next: result => console.log(`処理完了: ${result}`),
  complete: () => console.log('全処理完了')
});

// 出力:
// 処理完了: アイテム1の処理結果 (約1.1秒後)
// 処理完了: アイテム2の処理結果 (約2.1秒後)
// ...
// 処理完了: アイテム10の処理結果 (約10.1秒後)
// 全処理完了
```

### 2. ページネーション

複数ページのデータを順次取得します。

```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// ページデータ取得をシミュレートする関数
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`アイテム${page}-1`, `アイテム${page}-2`, `アイテム${page}-3`]
  }).pipe(
    delay(500) // API呼び出しをシミュレート
  );
}

function fetchAllPages(totalPages: number) {
  return range(1, totalPages).pipe(
    concatMap(page => fetchPage(page))
  );
}

fetchAllPages(5).subscribe({
  next: (data: PageData) => console.log(`ページ ${data.page}:`, data.items),
  complete: () => console.log('全ページ取得完了')
});

// 出力:
// ページ 1: ['アイテム1-1', 'アイテム1-2', 'アイテム1-3']
// ページ 2: ['アイテム2-1', 'アイテム2-2', 'アイテム2-3']
// ページ 3: ['アイテム3-1', 'アイテム3-2', 'アイテム3-3']
// ページ 4: ['アイテム4-1', 'アイテム4-2', 'アイテム4-3']
// ページ 5: ['アイテム5-1', 'アイテム5-2', 'アイテム5-3']
// 全ページ取得完了
```

### 3. 配列インデックスの処理

配列の各要素を処理する際に、インデックスベースのループとして使用します。

```typescript
import { range, map } from 'rxjs';
const items = ['Apple', 'Banana', 'Cherry', 'Date', 'Elderberry'];

range(0, items.length).pipe(
  map(index => ({ index, item: items[index] }))
).subscribe(({ index, item }) => {
  console.log(`[${index}] ${item}`);
});

// 出力:
// [0] Apple
// [1] Banana
// [2] Cherry
// [3] Date
// [4] Elderberry
```

### 4. テストデータの生成

単体テストでモックデータを生成する際に便利です。

```typescript
import { range, map, toArray } from 'rxjs';
// ユーザーデータのモック生成
range(1, 100).pipe(
  map(id => ({
    id,
    name: `User${id}`,
    email: `user${id}@example.com`
  })),
  toArray()
).subscribe(users => {
  console.log(`${users.length}件のユーザーを生成`);
  // テストで使用
});
```

### 5. リトライ処理のカウンター

エラー時のリトライ回数を制御します。

```typescript
import { range, throwError, concat, of, Observable, mergeMap, delay, catchError, map, toArray } from 'rxjs';
// データ取得をシミュレートする関数（ランダムに失敗する）
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.7; // 30%の確率で成功

  return of(shouldFail).pipe(
    delay(300),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('データ取得失敗'))
        : of('データ取得成功')
    )
  );
}

function fetchWithRetry(maxRetries: number = 3) {
  return concat(
    range(0, maxRetries).pipe(
      map(attempt => {
        console.log(`試行 ${attempt + 1}/${maxRetries}`);
        return fetchData().pipe(
          catchError((error: unknown) => {
            if (attempt === maxRetries - 1) {
              return throwError(() => new Error('最大リトライ回数に到達'));
            }
            return throwError(() => error);
          }),
          delay(Math.pow(2, attempt) * 1000) // 指数バックオフ
        );
      }),
      toArray()
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('結果:', result),
  error: err => console.error('エラー:', err.message)
});

// 出力例:
// 試行 1/3
// 試行 2/3
// 結果: データ取得成功
```

## スケジューラーによる非同期化

大量のデータを処理する場合、スケジューラーを指定して非同期実行できます。

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
console.log('開始');

// 100万個の数値を非同期で発行
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe({
  next: val => {
    if (val % 100000 === 0) {
      console.log(`進捗: ${val}`);
    }
  },
  complete: () => console.log('完了')
});

console.log('購読後（非同期なので即座に実行される）');

// 出力:
// 開始
// 購読後（非同期なので即座に実行される）
// 進捗: 100000
// 進捗: 200000
// ...
// 完了
```

> [!TIP]
> **スケジューラーの活用**:
> - 大量データ処理でUIをブロックしない
> - テストでの時間制御（TestScheduler）
> - Node.js環境でのイベントループ制御

詳しくは [スケジューラーの種類と使い分け](/guide/schedulers/types) を参照してください。

## 他の Creation Functions との比較

### range() vs of()

```typescript
import { range, of } from 'rxjs';

// range() - 連続した整数
range(1, 3).subscribe(console.log);
// 出力: 1, 2, 3

// of() - 任意の値を列挙
of(1, 2, 3).subscribe(console.log);
// 出力: 1, 2, 3

// 違い: range()は連番のみ、of()は任意の値を指定可能
of(1, 10, 100).subscribe(console.log);
// 出力: 1, 10, 100 (range()では不可能)
```

### range() vs from()

```typescript
import { range, from } from 'rxjs';

// range() - 連番を生成
range(1, 5).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5

// from() - 配列から生成（事前に配列を作成する必要がある）
from([1, 2, 3, 4, 5]).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5

// range()の利点: 配列を事前にメモリに確保しない
range(1, 1000000); // メモリ効率的
from(Array.from({ length: 1000000 }, (_, i) => i + 1)); // 配列がメモリに乗る
```

### range() vs generate()

```typescript
import { range, generate } from 'rxjs';

// range() - シンプルな連番
range(1, 5).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5

// generate() - 同じことを複雑に書いた例
generate(
  1,                    // 初期値
  x => x <= 5,          // 継続条件
  x => x + 1            // イテレーション
).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5

// generate()の利点: 複雑な条件や状態管理が可能
generate(
  1,
  x => x <= 100,
  x => x * 2  // 2倍ずつ増加
).subscribe(console.log);
// 出力: 1, 2, 4, 8, 16, 32, 64
// (range()では不可能)
```

> [!TIP]
> **選択基準**:
> - **連番が必要** → `range()`
> - **任意の値を列挙** → `of()`
> - **既存の配列/Promise** → `from()`
> - **複雑な条件/ステップ** → `generate()`

## パフォーマンスに関する注意

`range()`は同期的に値を発行するため、大量の値を生成する場合はパフォーマンスに注意が必要です。

> [!WARNING]
> **大量データの取り扱い**:
> ```typescript
> // ❌ 悪い例: 100万個の値を同期的に発行（UIがブロックされる）
> range(1, 1000000).subscribe(console.log);
>
> // ✅ 良い例1: スケジューラーで非同期化
> range(1, 1000000).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ 良い例2: バッファリングで分割
> range(1, 1000000).pipe(
>   bufferCount(1000)
> ).subscribe(batch => console.log(`${batch.length}件処理`));
> ```

## from()配列との使い分け

```typescript
import { range, from } from 'rxjs';

// 連番が必要な場合 → range()が簡潔
range(0, 10).subscribe(console.log);

// 配列を作成してから変換する必要はない（非効率）
from(Array.from({ length: 10 }, (_, i) => i)).subscribe(console.log);

// 既存の配列がある場合 → from()を使用
const existingArray = [5, 10, 15, 20];
from(existingArray).subscribe(console.log);
```

## エラーハンドリング

`range()`自体はエラーを発行しませんが、パイプラインでエラーが発生する可能性があります。

```typescript
import { range, of, map, catchError } from 'rxjs';
range(1, 10).pipe(
  map(n => {
    if (n === 5) {
      throw new Error('5でエラー');
    }
    return n * 2;
  }),
  catchError((error: unknown) => {
    const message = error instanceof Error ? error.message : String(error);
    console.error('エラー発生:', message);
    return of(-1); // デフォルト値を返す
  })
).subscribe(console.log);

// 出力: 2, 4, 6, 8, -1
```

## まとめ

`range()`は、連続する整数を生成するシンプルで強力なCreation Functionです。

> [!IMPORTANT]
> **range()の特徴**:
> - ✅ 連番生成に最適（for文の代替）
> - ✅ バッチ処理、ページネーション、テストデータ生成に便利
> - ✅ メモリ効率的（配列を事前に作成しない）
> - ⚠️ 大量データは非同期化を検討
> - ⚠️ 複雑な条件は`generate()`を使用

## 関連項目

- [generate()](/guide/creation-functions/loop/generate) - 汎用的なループ生成
- [of()](/guide/creation-functions/basic/of) - 任意の値を列挙
- [from()](/guide/creation-functions/basic/from) - 配列やPromiseから変換
- [interval()](/guide/creation-functions/basic/interval) - 定期的な値の発行

## 参考リソース

- [RxJS公式: range()](https://rxjs.dev/api/index/function/range)
- [Learn RxJS: range](https://www.learnrxjs.io/learn-rxjs/operators/creation/range)
