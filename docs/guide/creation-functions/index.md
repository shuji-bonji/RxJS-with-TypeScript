---
description: RxJSのCreation Functions（Observable作成関数）について、Pipeable Operatorとの違い、基本的な使い方、結合系Creation Functionsを網羅的に解説します。
---

# Creation Functions - Observableの作成と結合

RxJSでは、Observableを作成するための**Creation Functions**と、既存のObservableを変換する**Pipeable Operators**という2つの異なる形式があります。

このページでは、Creation Functionsの基本概念と、代表的な結合系Creation Functionsについて解説します。

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

| 特徴 | Creation Function | Pipeable Operator |
|------|-------------------|-------------------|
| **用途** | 新しいObservableを作成 | 既存のObservableを変換 |
| **インポート元** | `rxjs` | `rxjs/operators` |
| **使用方法** | 関数として直接呼び出し | `.pipe()` 内で使用 |
| **例** | `concat(obs1$, obs2$)` | `obs1$.pipe(concatWith(obs2$))` |

### Creation Function の例

```typescript
import { concat, of } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Creation Function として使用
concat(obs1$, obs2$).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5, 6
```

### Pipeable Operator の例

```typescript
import { of } from 'rxjs';
import { concatWith } from 'rxjs/operators';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Pipeable Operator として使用
obs1$.pipe(
  concatWith(obs2$)
).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5, 6
```

## 使い分けの基準

### Creation Function を使うべき場合

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

2章で学んだ基本的なCreation Functionsを振り返ります。

| Function | 説明 | 例 |
|----------|------|-----|
| `of` | 指定した値を順番に発行 | `of(1, 2, 3)` |
| `from` | 配列、Promise等から変換 | `from([1, 2, 3])` |
| `interval` | 指定間隔で連続発行 | `interval(1000)` |
| `timer` | 遅延後に発行開始 | `timer(1000, 500)` |

詳細は [Observableの作成方法](/guide/observables/creation) を参照してください。

## 結合系 Creation Functions

複数のObservableを結合するための主要なCreation Functionsです。

| Function | 説明 | ユースケース |
|----------|------|-------------|
| **[concat](/guide/creation-functions/concat)** | 順次結合（前が完了後、次が開始） | ステップバイステップ処理 |
| **[merge](/guide/creation-functions/merge)** | 並行結合（同時購読、発行順に出力） | 複数イベントの統合 |
| **[combineLatest](/guide/creation-functions/combineLatest)** | 最新値を組み合わせ | フォーム入力の同期 |
| **[zip](/guide/creation-functions/zip)** | 対応する値をペア化 | リクエストとレスポンスの対応 |
| **[race](/guide/creation-functions/race)** | 最初に発行したものを採用 | 複数データソースの競争 |
| **[forkJoin](/guide/creation-functions/forkJoin)** | すべての完了を待って最終値を結合 | 並列API呼び出しの完了待ち |
| **partition** | 条件で2つに分割 | 成功/失敗の分岐処理 |

各Creation Functionの詳細は、リンクをクリックして参照してください。

> [!NOTE]
> `partition` のドキュメントは準備中です。

## Pipeable Operator との対応関係

多くの結合系Creation Functionsには、対応するPipeable Operatorが存在します。

| Creation Function | Pipeable Operator | 備考 |
|-------------------|-------------------|------|
| `concat(a$, b$)` | `a$.pipe(concatWith(b$))` | RxJS 7+ |
| `merge(a$, b$)` | `a$.pipe(mergeWith(b$))` | RxJS 7+ |
| `zip(a$, b$)` | `a$.pipe(zipWith(b$))` | RxJS 7+ |
| `combineLatest([a$, b$])` | `a$.pipe(combineLatestWith(b$))` | RxJS 7+ |
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` | RxJS 7+ |

> [!NOTE]
> RxJS 7以降、`concatWith`, `mergeWith`, `zipWith`などの`~With`系オペレーターが追加され、Pipeable Operatorとしても使いやすくなりました。

## どちらを使うべきか？

### Creation Function を推奨

```typescript
// ✅ 複数のObservableを同じレベルで結合
const combined$ = merge(
  fromEvent(button1, 'click'),
  fromEvent(button2, 'click'),
  fromEvent(button3, 'click')
);
```

### Pipeable Operator を推奨

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
- 結合系Creation Functionsは用途に応じて使い分ける
- パイプラインの中では`~With`系のPipeable Operatorを使う

次のセクションでは、各Creation Functionの詳細な使い方と実践例を学びます。
