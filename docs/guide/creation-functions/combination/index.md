---
description: 複数のObservableを1つに結合するCreation Functionsについて解説します。concat、merge、combineLatest、zip、forkJoinの使い分けと実践例を学びます。
---

# 結合系 Creation Functions

複数のObservableを1つのObservableに結合するための主要なCreation Functionsです。

## 結合系 Creation Functions とは

結合系のCreation Functionsは、複数のObservableを受け取り、それらを1つのObservableストリームにまとめます。結合方法によって、値の発行タイミングや順序が異なります。

以下の表で、各Creation Functionの特徴と使い分けを確認してください。

## 主要な結合系 Creation Functions

| Function | 説明 | ユースケース |
|----------|------|-------------|
| **[concat](/guide/creation-functions/combination/concat)** | 順次結合（前が完了後、次が開始） | ステップバイステップ処理 |
| **[merge](/guide/creation-functions/combination/merge)** | 並行結合（同時購読、発行順に出力） | 複数イベントの統合 |
| **[combineLatest](/guide/creation-functions/combination/combineLatest)** | 最新値を組み合わせ | フォーム入力の同期 |
| **[zip](/guide/creation-functions/combination/zip)** | 対応する値をペア化 | リクエストとレスポンスの対応 |
| **[forkJoin](/guide/creation-functions/combination/forkJoin)** | すべての完了を待って最終値を結合 | 並列API呼び出しの完了待ち |

## 使い分けの基準

結合系Creation Functionsの選択は、以下の観点で判断します。

### 1. 実行タイミング

- **順次実行**: `concat` - 前のObservableが完了してから次を開始
- **並行実行**: `merge`, `combineLatest`, `zip`, `forkJoin` - すべてのObservableを同時に購読

### 2. 値の発行方法

- **すべての値を発行**: `concat`, `merge` - 各Observableから発行されたすべての値を出力
- **最新値を組み合わせ**: `combineLatest` - いずれかが値を発行するたびに、すべての最新値を組み合わせて発行
- **対応する値をペア化**: `zip` - 各Observableの対応する位置の値をペア化して発行
- **最終値のみ**: `forkJoin` - すべてのObservableが完了した時点で、各最終値を配列で発行

### 3. 完了のタイミング

- **すべて完了後**: `concat`, `forkJoin` - すべてのObservableが完了するまで待つ
- **最短のストリームで完了**: `zip` - いずれか1つが完了した時点で、残りの値がペアになれなくなるため完了
- **完了しない**: `merge`, `combineLatest` - いずれかが完了しても、他が継続していれば完了しない

## Cold から Hot への変換

上記の表に示した通り、**全ての結合系Creation Functionsは Cold Observable を生成します**。購読するたびに独立した実行が開始されます。

しかし、マルチキャスト系オペレーター（`share()`, `shareReplay()`, `publish()` など）を使用することで、**Cold Observable を Hot Observable に変換**できます。

### 実践例：HTTPリクエストの共有

```typescript
import { merge, interval } from 'rxjs';
import { map, take, share } from 'rxjs';

// ❄️ Cold - 購読ごとに独立したHTTPリクエスト
const coldApi$ = merge(
  interval(1000).pipe(map(() => 'Source A'), take(3)),
  interval(1500).pipe(map(() => 'Source B'), take(2))
);

coldApi$.subscribe(val => console.log('購読者1:', val));
coldApi$.subscribe(val => console.log('購読者2:', val));
// → 各購読者が独立した interval を実行（2倍のリクエスト）

// 🔥 Hot - 購読者間で実行を共有
const hotApi$ = merge(
  interval(1000).pipe(map(() => 'Source A'), take(3)),
  interval(1500).pipe(map(() => 'Source B'), take(2))
).pipe(share());

hotApi$.subscribe(val => console.log('購読者1:', val));
hotApi$.subscribe(val => console.log('購読者2:', val));
// → 1つの interval を共有（リクエストは1回のみ）
```

> [!TIP]
> **Hot化が必要なケース**:
> - 複数のコンポーネントで同じAPI結果を共有
> - `forkJoin` で並列リクエストした結果を複数箇所で使用
> - `combineLatest` で状態を管理し、複数の購読者に配信
>
> 詳しくは [基本作成系 - Cold から Hot への変換](/guide/creation-functions/basic/#cold-から-hot-への変換) を参照してください。

## Pipeable Operator との対応関係

結合系Creation Functionsには、対応するPipeable Operatorが存在します。パイプラインの中で使用する場合は、`~With`系のオペレーターを使います。

| Creation Function | Pipeable Operator |
|-------------------|-------------------|
| `concat(a$, b$)` | `a$.pipe(concatWith(b$))` |
| `merge(a$, b$)` | `a$.pipe(mergeWith(b$))` |
| `zip(a$, b$)` | `a$.pipe(zipWith(b$))` |
| `combineLatest([a$, b$])` | `a$.pipe(combineLatestWith(b$))` |

## 次のステップ

各Creation Functionの詳細な動作と実践例を学ぶには、上記の表からリンクをクリックしてください。

また、[選択・分割系 Creation Functions](/guide/creation-functions/selection/)や[条件分岐系 Creation Functions](/guide/creation-functions/conditional/)も併せて学習することで、Creation Functionsの全体像を理解できます。
