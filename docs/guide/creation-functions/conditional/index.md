---
description: "条件に基づいてObservableを選択・作成するCreation Functions（iifとdefer）について解説。iifは三項演算子的な条件分岐、deferは購読時評価を実現します。それぞれの使い分け、TypeScriptでの型安全な実装、実践的なユースケースを紹介します。"
---

# 条件分岐系 Creation Functions

条件に基づいてObservableを選択したり、購読時に動的にObservableを生成するCreation Functionsです。

## 条件分岐系 Creation Functions とは

条件分岐系のCreation Functionsは、以下の役割を持ちます。

- **条件による選択**: 条件に応じて異なるObservableを選択する
- **遅延生成**: 購読時に動的にObservableを生成する

これらは、静的にObservableを作成・結合する他のCreation Functionsとは異なり、**実行時の条件や状態**に基づいて振る舞いを変えることができます。

> [!NOTE]
> `iif`と`defer`は以前「条件オペレーター」として分類されていましたが、これらは**Creation Functions**（Observable作成関数）であり、Pipeable Operatorではありません。

## 主要な条件分岐系 Creation Functions

| Function | 説明 | ユースケース |
|----------|------|-------------|
| **[iif](/guide/creation-functions/conditional/iif)** | 条件に応じて2つのObservableのどちらかを選ぶ | ログイン状態による処理分岐 |
| **[defer](/guide/creation-functions/conditional/defer)** | 購読時にObservableを遅延生成 | 動的なObservable作成 |

## 使い分けの基準

### iif - 条件による2つの分岐

`iif`は、条件関数の結果に応じて、2つのObservableのうちどちらかを選択します。条件は**購読時**に評価されます。

**構文**:
```typescript
iif(
  () => condition,  // 条件関数（購読時に評価）
  trueObservable,   // true の場合のObservable
  falseObservable   // false の場合のObservable
)
```

**ユースケース**:
- ログイン状態に応じた処理分岐
- キャッシュの有無による処理切り替え
- 環境変数による動作変更

```typescript
import { iif, of } from 'rxjs';

const isAuthenticated = () => Math.random() > 0.5;

const data$ = iif(
  isAuthenticated,
  of('Authenticated data'),
  of('Public data')
);

data$.subscribe(console.log);
// 出力: 'Authenticated data' または 'Public data'（購読時の条件による）
```

### defer - 購読時の遅延生成

`defer`は、購読が発生するたびにObservableを生成します。これにより、購読時点の状態に基づいてObservableの振る舞いを変えることができます。

**構文**:
```typescript
defer(() => {
  // 購読時に実行される
  return someObservable;
})
```

**ユースケース**:
- 購読時の最新状態を反映したObservable生成
- 毎回異なるランダム値を生成
- 購読ごとに異なる処理を実行

```typescript
import { defer, of } from 'rxjs';

// 購読時に現在時刻を取得
const timestamp$ = defer(() => of(new Date().toISOString()));

setTimeout(() => {
  timestamp$.subscribe(time => console.log('First:', time));
}, 1000);

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Second:', time));
}, 2000);

// 出力:
// First: 2024-10-21T01:00:00.000Z
// Second: 2024-10-21T01:00:01.000Z
// ※購読時刻が異なるため、異なる時刻が出力される
```

## iif vs defer の違い

| 特徴 | iif | defer |
|------|-----|-------|
| **選択肢** | 2つのObservableから選択 | 任意のObservableを生成 |
| **評価タイミング** | 購読時に条件を評価 | 購読時に関数を実行 |
| **用途** | 条件分岐 | 動的生成 |

## パイプラインでの使用

条件分岐系Creation Functionsは、他のオペレーターと組み合わせて使用できます。

```typescript
import { defer, of } from 'rxjs';
import { switchMap } from 'rxjs';

// ユーザーIDからユーザー情報を取得
const userId$ = of(123);

userId$.pipe(
  switchMap(id =>
    defer(() => {
      // 購読時に最新のキャッシュをチェック
      const cached = cache.get(id);
      return cached ? of(cached) : fetchUser(id);
    })
  )
).subscribe(console.log);
```

## Cold から Hot への変換

上記の表に示した通り、**全ての条件分岐系Creation Functionsは Cold Observable を生成します**。購読するたびに条件評価や生成関数が実行されます。

マルチキャスト系オペレーター（`share()`, `shareReplay()` など）を使用することで、Cold Observable を Hot Observable に変換できます。

### 実践例：条件分岐結果の共有

```typescript
import { iif, of, interval } from 'rxjs';
import { take, share } from 'rxjs';

const condition = () => Math.random() > 0.5;

// ❄️ Cold - 購読ごとに条件を再評価
const coldIif$ = iif(
  condition,
  of('Condition is true'),
  interval(1000).pipe(take(3))
);

coldIif$.subscribe(val => console.log('購読者1:', val));
coldIif$.subscribe(val => console.log('購読者2:', val));
// → 各購読者が独立して条件評価（異なる結果の可能性）

// 🔥 Hot - 購読者間で条件評価結果を共有
const hotIif$ = iif(
  condition,
  of('Condition is true'),
  interval(1000).pipe(take(3))
).pipe(share());

hotIif$.subscribe(val => console.log('購読者1:', val));
hotIif$.subscribe(val => console.log('購読者2:', val));
// → 条件評価は1回のみ、結果を共有
```

> [!TIP]
> 詳しくは [基本作成系 - Cold から Hot への変換](/guide/creation-functions/basic/#cold-から-hot-への変換) を参照してください。

## 次のステップ

各Creation Functionの詳細な動作と実践例を学ぶには、上記の表からリンクをクリックしてください。

また、[結合系 Creation Functions](/guide/creation-functions/combination/)や[選択・分割系 Creation Functions](/guide/creation-functions/selection/)も併せて学習することで、Creation Functionsの全体像を理解できます。
