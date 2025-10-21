---
description: of() - 指定した値を順番に発行するCreation Function。最もシンプルなObservable作成方法で、テストデータやモック作成に最適です。
---

# of() - 値の順次発行

`of()`は、指定した値を1つずつ順番に発行する、最もシンプルなCreation Functionです。

## 概要

`of()`は引数として渡された値を、購読と同時に順番に発行し、すべての値を発行後に即座に完了します。テストコードやモックデータの作成に頻繁に使用されます。

**シグネチャ**:
```typescript
function of<T>(...args: T[]): Observable<T>
```

**公式ドキュメント**: [📘 RxJS公式: of()](https://rxjs.dev/api/index/function/of)

## 基本的な使い方

`of()`は複数の値をカンマ区切りで渡すことができます。

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('値:', value),
  error: err => console.error('エラー:', err),
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

## 重要な特徴

### 1. 同期的に発行

`of()`は購読と同時に、すべての値を**同期的に**発行します。

```typescript
import { of } from 'rxjs';

console.log('購読前');

of('A', 'B', 'C').subscribe(value => console.log('値:', value));

console.log('購読後');

// 出力:
// 購読前
// 値: A
// 値: B
// 値: C
// 購読後
```

### 2. 即座に完了

すべての値を発行後、即座に`complete`を通知します。

```typescript
import { of } from 'rxjs';

of(1, 2, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('完了！')
});

// 出力: 1, 2, 3, 完了！
```

### 3. 任意の型の値を発行可能

プリミティブ型からオブジェクト、配列まで、任意の型の値を発行できます。

```typescript
import { of } from 'rxjs';

// プリミティブ型
of(42, 'hello', true).subscribe(console.log);

// オブジェクト
of(
  { id: 1, name: 'Alice' },
  { id: 2, name: 'Bob' }
).subscribe(console.log);

// 配列（配列そのものを1つの値として発行）
of([1, 2, 3], [4, 5, 6]).subscribe(console.log);
// 出力: [1, 2, 3], [4, 5, 6]
```

### 4. Cold Observable

`of()`は**Cold Observable**です。購読するたびに独立した実行が開始されます。

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3);

// 1回目の購読
values$.subscribe(val => console.log('購読者A:', val));

// 2回目の購読（独立して実行される）
values$.subscribe(val => console.log('購読者B:', val));

// 出力:
// 購読者A: 1
// 購読者A: 2
// 購読者A: 3
// 購読者B: 1
// 購読者B: 2
// 購読者B: 3
```

> [!NOTE]
> **Cold Observableの特徴**
> - 購読するたびに独立した実行が開始されます
> - 各購読者は独自のデータストリームを受け取ります
> - データの共有が必要な場合は、`share()`などでHot化する必要があります
>
> 詳しくは [コールドObservableとホットObservable](/guide/observables/cold-and-hot-observables) を参照してください。

## of() vs from() の違い

`of()`と`from()`は、配列を扱う際に動作が異なります。これはよくある混乱ポイントです。

```typescript
import { of, from } from 'rxjs';

// of() - 配列を1つの値として発行
of([1, 2, 3]).subscribe(console.log);
// 出力: [1, 2, 3]

// from() - 配列の各要素を個別に発行
from([1, 2, 3]).subscribe(console.log);
// 出力: 1, 2, 3
```

> [!IMPORTANT]
> **使い分けの基準**:
> - 配列そのものを発行したい → `of([1, 2, 3])`
> - 配列の各要素を個別に発行したい → `from([1, 2, 3])`

## 実践的なユースケース

### 1. テストデータ・モック作成

`of()`は、テストコードでモックデータを作成する際に最も頻繁に使用されます。

```typescript
import { of } from 'rxjs';

// ユーザーデータのモック
function getMockUser$() {
  return of({
    id: 1,
    name: 'Test User',
    email: 'test@example.com'
  });
}

// テストで使用
getMockUser$().subscribe(user => {
  console.log('User:', user.name); // User: Test User
});
```

### 2. デフォルト値の提供

エラー時のフォールバック値や、デフォルト値を提供する際に使用します。

```typescript
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

function fetchData(id: number) {
  if (id < 0) {
    return throwError(() => new Error('Invalid ID'));
  }
  return of({ id, data: 'some data' });
}

fetchData(-1).pipe(
  catchError(err => {
    console.error('エラー:', err.message);
    return of({ id: 0, data: 'default data' }); // デフォルト値
  })
).subscribe(result => console.log(result));
// 出力: エラー: Invalid ID
//       { id: 0, data: 'default data' }
```

### 3. 複数の値を段階的に発行

複数のステップを順番に実行する際に使用します。

```typescript
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('Loading...', 'Processing...', 'Done!').pipe(
  concatMap(message => of(message).pipe(delay(1000)))
).subscribe(console.log);

// 出力（1秒ごと）:
// Loading...
// Processing...
// Done!
```

### 4. 条件分岐での値の返却

`iif()`や`switchMap()`と組み合わせて、条件に応じた値を返します。

```typescript
import { of, iif } from 'rxjs';

const isAuthenticated = true;

iif(
  () => isAuthenticated,
  of('Welcome back!'),
  of('Please log in')
).subscribe(console.log);
// 出力: Welcome back!
```

## パイプラインでの使用

`of()`はパイプラインの開始点として、または途中でデータを注入する際に使用されます。

```typescript
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

of(1, 2, 3, 4, 5).pipe(
  filter(n => n % 2 === 0),  // 偶数のみ
  map(n => n * 10)           // 10倍
).subscribe(console.log);
// 出力: 20, 40
```

## よくある間違い

### 1. 配列をそのまま渡す

```typescript
// ❌ 間違い - 配列全体が1つの値として発行される
of([1, 2, 3]).subscribe(console.log);
// 出力: [1, 2, 3]

// ✅ 正しい - 各要素を個別に発行したい場合はfrom()を使う
from([1, 2, 3]).subscribe(console.log);
// 出力: 1, 2, 3

// ✅ または、スプレッド構文を使う
of(...[1, 2, 3]).subscribe(console.log);
// 出力: 1, 2, 3
```

### 2. 非同期処理と混同

`of()`は同期的に発行します。非同期処理にはならないので注意が必要です。

```typescript
// ❌ これは非同期にならない
of(fetchDataFromAPI()).subscribe(console.log);
// fetchDataFromAPI()は即座に実行され、そのPromiseオブジェクトが発行される

// ✅ Promiseをストリーム化する場合はfrom()を使う
from(fetchDataFromAPI()).subscribe(console.log);
```

## パフォーマンスの考慮事項

`of()`は非常に軽量で、パフォーマンスのオーバーヘッドはほとんどありません。ただし、大量の値を発行する場合は、以下の点に注意してください。

> [!TIP]
> 大量の値（数千以上）を順次発行する場合は、`from()`や`range()`の使用を検討してください。

## 関連するCreation Functions

| Function | 違い | 使い分け |
|----------|------|----------|
| **[from()](/guide/creation-functions/basic/from)** | 配列やPromiseから変換 | イテラブルやPromiseをストリーム化 |
| **range()** | 数値の範囲を生成 | 連続した数値を発行 |
| **EMPTY** | 何も発行せず即座に完了 | 空のストリームが必要な場合 |

## まとめ

- `of()`は指定した値を順番に発行する最もシンプルなCreation Function
- 購読と同時に同期的に発行し、即座に完了する
- テストデータやモック作成に最適
- 配列を渡すと配列そのものが発行される（`from()`と異なる）
- 非同期処理には`from()`を使用する

## 次のステップ

- [from() - 配列・Promise等から変換](/guide/creation-functions/basic/from)
- [結合系 Creation Functions](/guide/creation-functions/combination/)
- [基本作成系の概要に戻る](/guide/creation-functions/basic/)
