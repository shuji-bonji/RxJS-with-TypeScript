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
- **いずれか完了時**: `zip` - いずれか1つが完了したら完了
- **完了しない**: `merge`, `combineLatest` - いずれかが完了しても、他が継続していれば完了しない

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
