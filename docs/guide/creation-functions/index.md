---
description: RxJSのCreation Functions（Observable作成関数）について、Pipeable Operatorとの違い、基本的な使い方、7つのカテゴリ（基本作成系・ループ生成系・HTTP通信系・結合系・選択分割系・条件分岐系・制御系）を網羅的に解説します。
---

# Creation Functions

RxJSでは、Observableを作成するための**Creation Functions**と、既存のObservableを変換する**Pipeable Operators**という2つの異なる形式があります。

このページでは、Creation Functionsの基本概念と、7つの主要なカテゴリについて解説します。

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

## Creation Functions のカテゴリ

この章では、Creation Functionsを7つのカテゴリに分けて学習します。

### 全カテゴリ一覧

以下の表で、すべてのカテゴリと含まれる関数を確認できます。各関数名をクリックすると詳細ページに移動します。

| カテゴリ | 説明 | 主要な関数 | 代表的なユースケース |
|---------|------|-----------|-------------------|
| **[基本作成系](/guide/creation-functions/basic/)** | 最も基本的で頻繁に使用される関数。データ、配列、イベント、時間ベースのObservableを作成 | [of](/guide/creation-functions/basic/of), [from](/guide/creation-functions/basic/from), [fromEvent](/guide/creation-functions/basic/fromEvent), [interval](/guide/creation-functions/basic/interval), [timer](/guide/creation-functions/basic/timer) | 固定値のテスト、既存データのストリーム化、DOMイベント処理、ポーリング、遅延実行 |
| **[ループ生成系](/guide/creation-functions/loop/)** | for文やwhile文のようなループ処理をObservableで表現 | [range](/guide/creation-functions/loop/range), [generate](/guide/creation-functions/loop/generate) | 連番生成、バッチ処理、複雑な状態遷移、数学的計算 |
| **[HTTP通信系](/guide/creation-functions/http-communication/)** | HTTP通信をObservableとして扱う | [ajax](/guide/creation-functions/http-communication/ajax), [fromFetch](/guide/creation-functions/http-communication/fromFetch) | XMLHttpRequestベースのHTTP通信、Fetch APIベースのHTTP通信、REST API呼び出し |
| **[結合系](/guide/creation-functions/combination/)** | 複数のObservableを1つに結合。結合方法によって発行タイミングや順序が異なる | [concat](/guide/creation-functions/combination/concat), [merge](/guide/creation-functions/combination/merge), [combineLatest](/guide/creation-functions/combination/combineLatest), [zip](/guide/creation-functions/combination/zip), [forkJoin](/guide/creation-functions/combination/forkJoin) | ステップバイステップ処理、複数イベントの統合、フォーム入力の同期、並列API呼び出しの完了待ち |
| **[選択・分割系](/guide/creation-functions/selection/)** | 複数のObservableから1つを選択、または1つのObservableを複数に分割 | [race](/guide/creation-functions/selection/race), [partition](/guide/creation-functions/selection/partition) | 複数データソースの競争、成功/失敗の分岐処理 |
| **[条件分岐系](/guide/creation-functions/conditional/)** | 条件に基づいてObservableを選択、または購読時に動的に生成 | [iif](/guide/creation-functions/conditional/iif), [defer](/guide/creation-functions/conditional/defer) | ログイン状態による処理分岐、動的なObservable作成、遅延評価 |
| **[制御系](/guide/creation-functions/control/)** | Observableの実行タイミングやリソース管理を制御 | [scheduled](/guide/creation-functions/control/scheduled), [using](/guide/creation-functions/control/using) | スケジューラーによる実行タイミング制御、リソースのライフサイクル管理、メモリリーク防止 |

> [!TIP]
> **学習の順序**
>
> 初学者は以下の順序で学習することをお勧めします。
> 1. **基本作成系** - RxJSの基本となる関数
> 2. **結合系** - 複数のストリームを扱う基礎
> 3. **HTTP通信系** - 実践的なAPI連携
> 4. その他のカテゴリ - 必要に応じて学習

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

- **Creation Functions**: Observableを作成したり、結合する関数
- **Pipeable Operators**: 既存のObservableを変換する関数
- Creation Functionsは7つのカテゴリに分類される
  1. **基本作成系**: データ、配列、イベント、時間ベースのObservableを作成
  2. **ループ生成系**: 繰り返し処理をObservableで表現
  3. **HTTP通信系**: HTTP通信をObservableとして扱う
  4. **結合系**: 複数を1つにまとめる
  5. **選択・分割系**: 選択または分割する
  6. **条件分岐系**: 条件に応じて動的に生成する
  7. **制御系**: 実行タイミングやリソース管理を制御
- パイプラインの中では`~With`系のPipeable Operatorを使う
- 各カテゴリには複数の関数が含まれ、用途に応じて使い分ける

## 次のステップ

各カテゴリの詳細を学ぶには、以下のリンクから進んでください。

1. **[基本作成系 Creation Functions](/guide/creation-functions/basic/)** - of, from, fromEvent, interval, timer
2. **[ループ生成系 Creation Functions](/guide/creation-functions/loop/)** - range, generate
3. **[HTTP通信系 Creation Functions](/guide/creation-functions/http-communication/)** - ajax, fromFetch
4. **[結合系 Creation Functions](/guide/creation-functions/combination/)** - concat, merge, combineLatest, zip, forkJoin
5. **[選択・分割系 Creation Functions](/guide/creation-functions/selection/)** - race, partition
6. **[条件分岐系 Creation Functions](/guide/creation-functions/conditional/)** - iif, defer
7. **[制御系 Creation Functions](/guide/creation-functions/control/)** - scheduled, using

各ページで、Creation Functionの詳細な動作と実践例を学ぶことができます。

## 参考リソース

- [RxJS公式ドキュメント - Creation Functions](https://rxjs.dev/guide/operators#creation-operators-list)
- [Learn RxJS - Creation Operators](https://www.learnrxjs.io/learn-rxjs/operators/creation)
