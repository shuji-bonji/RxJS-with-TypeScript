---
description: RxJSの結合オペレーター（Pipeable Operators）を使って複数のObservableを組み合わせる方法を解説します。withLatestFromなどのPipeable形式の演算子の使い分けと活用法を紹介します。
---

# 結合オペレーター（Pipeable Operators）

RxJS の結合（Combination）オペレーターは、複数の Observable を組み合わせて新しいストリームを作り出すための強力なツールです。

> [!IMPORTANT]
> このページでは **Pipeable Operators（パイプライン内で使用する形式）** を扱います。
>
> **Creation Functions（複数のObservableから新しいObservableを作成する形式）** については、
> [3章 Creation Functions](/guide/creation-functions/) を参照してください。

## Creation Functions vs Pipeable Operators

結合に関連する機能は、2つの形式で提供されています。

### Creation Functions（3章で解説）

複数のObservableを引数として受け取り、新しいObservableを作成。

```typescript
import { concat, merge, combineLatest, zip, race, forkJoin } from 'rxjs';

// Creation Function として使用
const combined$ = concat(obs1$, obs2$, obs3$);
const merged$ = merge(source1$, source2$);
```

詳細は [Creation Functions](/guide/creation-functions/) を参照。

### Pipeable Operators（このページで解説）

既存のObservableに対して `.pipe()` 内で使用。

```typescript
import { concatWith, mergeWith, combineLatestWith } from 'rxjs/operators';

// Pipeable Operator として使用
const result$ = source$.pipe(
  map(x => x * 2),
  concatWith(other$),
  filter(x => x > 10)
);
```

## Pipeable Operators 一覧

### ◾ このページで扱うオペレーター

|オペレーター|説明|
|---|---|
|[withLatestFrom](./withLatestFrom)|メインObservableの発行に応じて、最新の他ストリームの値を組み合わせます|

### ◾ Creation Functions として提供されるもの

以下は主に Creation Function として使用されます（[3章](/guide/creation-functions/)参照）。

|Function|説明|Pipeable版|
|---|---|---|
|[concat](./concat)|順番に結合|`concatWith` (RxJS 7+)|
|[merge](./merge)|並行結合|`mergeWith` (RxJS 7+)|
|[combineLatest](./combineLatest)|最新値を組み合わせ|`combineLatestWith` (RxJS 7+)|
|[zip](./zip)|対応する値をペア化|`zipWith` (RxJS 7+)|
|[race](./race)|最速のストリームを採用|`raceWith` (RxJS 7+)|
|[forkJoin](./forkJoin)|すべての完了を待つ|（Pipeable版なし）|

## さらに実践的に学びたい方へ

結合オペレーターを使ったリアルなシナリオ例は、  
[実用的なユースケース](./practical-use-cases.md) にて詳しく紹介しています。