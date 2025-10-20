---
description: RxJSのCreation Functions（Observable作成関数）について、Pipeable Operatorとの違い、基本的な使い方、3つのカテゴリ（結合系・選択分割系・条件分岐系）を網羅的に解説します。
---

# Creation Functions

RxJSでは、Observableを作成するための**Creation Functions**と、既存のObservableを変換する**Pipeable Operators**という2つの異なる形式があります。

このページでは、Creation Functionsの基本概念と、3つの主要なカテゴリについて解説します。

## Creation Functions とは

**Creation Functions**は、新しいObservableを作成するための関数です。

```typescript
import { of, from, interval } from 'rxjs';

// Creation Functionとして使用
const obs1$ = of(1, 2, 3);
const obs2$ = from([4, 5, 6]);
const obs3$ = interval(1000);
```

これらは`rxjs`パッケージから直接インポートし、関数として呼び出してObservableを生成します。

## Pipeable Operator との違い

Creation FunctionsとPipeable Operatorsは、用途と使い方が異なります。以下の表で両者の違いを確認してください。

| 特徴 | Creation Function | Pipeable Operator |
|------|-------------------|-------------------|
| **用途** | 新しいObservableを作成 | 既存のObservableを変換 |
| **インポート元** | `rxjs` | `rxjs/operators` |
| **使用方法** | 関数として直接呼び出し | `.pipe()` 内で使用 |
| **例** | `concat(obs1$, obs2$)` | `obs1$.pipe(concatWith(obs2$))` |

### Creation Function の例

Creation Functionは、複数のObservableを直接結合する際に使用します。

```typescript
import { concat, of } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Creation Function として使用
concat(obs1$, obs2$).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5, 6
```

### Pipeable Operator の例

Pipeable Operatorは、既存のObservableに対して変換処理を追加する際に使用します。

```typescript
import { of } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Pipeable Operator として使用
obs1$.pipe(
  concatWith(obs2$)
).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5, 6
```

## 使い分けの基準

Creation FunctionとPipeable Operatorの選択は、以下の基準で判断します。

### Creation Function を使うべき場合

Creation Functionは、複数のObservableを同じレベルで操作する場合や、最初からObservableを作成する場合に適しています。

- **複数のObservableを同じレベルで結合する場合**
  ```typescript
  concat(obs1$, obs2$, obs3$)
  merge(click$, hover$, scroll$)
  ```

- **最初からObservableを作成する場合**
  ```typescript
  of(1, 2, 3)
  from([1, 2, 3])
  interval(1000)
  ```

### Pipeable Operator を使うべき場合

Pipeable Operatorは、既存のObservableに処理を追加する場合や、複数の操作を連鎖させる場合に適しています。

- **既存のObservableに処理を追加する場合**
  ```typescript
  obs1$.pipe(
    map(x => x * 2),
    concatWith(obs2$),
    filter(x => x > 5)
  )
  ```

- **パイプラインとして複数の操作を連鎖させる場合**

## 基本的な Creation Functions（復習）

2章で学んだ基本的なCreation Functionsを振り返ります。これらは単一のObservableを作成するための基礎的な関数です。

| Function | 説明 | 例 |
|----------|------|-----|
| `of` | 指定した値を順番に発行 | `of(1, 2, 3)` |
| `from` | 配列、Promise等から変換 | `from([1, 2, 3])` |
| `interval` | 指定間隔で連続発行 | `interval(1000)` |
| `timer` | 遅延後に発行開始 | `timer(1000, 500)` |

詳細は [Observableの作成方法](/guide/observables/creation) を参照してください。

## Creation Functions の3つのカテゴリ

この章では、複数のObservableを操作するための高度なCreation Functionsを3つのカテゴリに分けて学習します。

### 1. [結合系 Creation Functions](/guide/creation-functions/combination/)

複数のObservableを1つのObservableに結合します。結合方法によって、値の発行タイミングや順序が異なります。

**主要な関数**: concat, merge, combineLatest, zip, forkJoin

**代表的なユースケース**:
- ステップバイステップ処理（concat）
- 複数イベントの統合（merge）
- フォーム入力の同期（combineLatest）
- 並列API呼び出しの完了待ち（forkJoin）

→ [結合系 Creation Functionsの詳細を見る](/guide/creation-functions/combination/)

### 2. [選択・分割系 Creation Functions](/guide/creation-functions/selection/)

複数のObservableから1つを選択したり、1つのObservableを複数に分割します。

**主要な関数**: race, partition

**代表的なユースケース**:
- 複数データソースの競争（race）
- 成功/失敗の分岐処理（partition）

→ [選択・分割系 Creation Functionsの詳細を見る](/guide/creation-functions/selection/)

### 3. [条件分岐系 Creation Functions](/guide/creation-functions/conditional/)

条件に基づいてObservableを選択したり、購読時に動的にObservableを生成します。

**主要な関数**: iif, defer

**代表的なユースケース**:
- ログイン状態による処理分岐（iif）
- 動的なObservable作成（defer）

→ [条件分岐系 Creation Functionsの詳細を見る](/guide/creation-functions/conditional/)

## Pipeable Operator との対応関係

多くのCreation Functionsには、対応するPipeable Operatorが存在します。パイプラインの中で使用する場合は、`~With`系のオペレーターを使います。

| Creation Function | Pipeable Operator | 備考 |
|-------------------|-------------------|------|
| `concat(a$, b$)` | `a$.pipe(`**[concatWith](/guide/operators/combination/concatWith)**`(b$))` | RxJS 7+ |
| `merge(a$, b$)` | `a$.pipe(`**[mergeWith](/guide/operators/combination/mergeWith)**`(b$))` | RxJS 7+ |
| `zip(a$, b$)` | `a$.pipe(`**[zipWith](/guide/operators/combination/zipWith)**`(b$))` | RxJS 7+ |
| `combineLatest([a$, b$])` | `a$.pipe(`**[combineLatestWith](/guide/operators/combination/combineLatestWith)**`(b$))` | RxJS 7+ |
| `race(a$, b$)` | `a$.pipe(`**[raceWith](/guide/operators/combination/raceWith)**`(b$))` | RxJS 7+ |

> [!NOTE]
> RxJS 7以降、**[concatWith](/guide/operators/combination/concatWith)**, **[mergeWith](/guide/operators/combination/mergeWith)**, **[zipWith](/guide/operators/combination/zipWith)**, **[combineLatestWith](/guide/operators/combination/combineLatestWith)**, **[raceWith](/guide/operators/combination/raceWith)** などの`~With`系オペレーターが追加され、Pipeable Operatorとしても使いやすくなりました。

## どちらを使うべきか？

Creation FunctionとPipeable Operatorの選択は、コンテキストによって異なります。

### Creation Function を推奨

複数のObservableを同じレベルで操作する場合は、Creation Functionを使うことでコードが簡潔になります。

```typescript
// ✅ 複数のObservableを同じレベルで結合
const combined$ = merge(
  fromEvent(button1, 'click'),
  fromEvent(button2, 'click'),
  fromEvent(button3, 'click')
);
```

### Pipeable Operator を推奨

パイプラインの一部として操作を追加する場合は、Pipeable Operatorを使うことで処理の流れが明確になります。

```typescript
// ✅ パイプラインの一部として結合
const result$ = source$.pipe(
  map(x => x * 2),
  mergeWith(other$),
  filter(x => x > 10)
);
```

## まとめ

- **Creation Functions**: 新しいObservableを作成する関数
- **Pipeable Operators**: 既存のObservableを変換する関数
- Creation Functionsは3つのカテゴリに分類される。
  - **結合系**: 複数を1つにまとめる
  - **選択・分割系**: 選択または分割する
  - **条件分岐系**: 条件に応じて動的に生成する
- パイプラインの中では`~With`系のPipeable Operatorを使う

## 次のステップ

各カテゴリの詳細を学ぶには、以下のリンクから進んでください。

1. **[結合系 Creation Functions](/guide/creation-functions/combination/)** - concat, merge, combineLatest, zip, forkJoin
2. **[選択・分割系 Creation Functions](/guide/creation-functions/selection/)** - race, partition
3. **[条件分岐系 Creation Functions](/guide/creation-functions/conditional/)** - iif, defer

各ページで、Creation Functionの詳細な動作と実践例を学ぶことができます。
