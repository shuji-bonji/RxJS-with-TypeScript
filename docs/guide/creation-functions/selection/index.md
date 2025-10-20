---
description: 複数のObservableから1つを選択したり、1つのObservableを複数に分割するCreation Functionsについて解説します。raceとpartitionの使い分けと実践例を学びます。
---

# 選択・分割系 Creation Functions

複数のObservableから1つを選択したり、1つのObservableを条件に応じて複数に分割するCreation Functionsです。

## 選択・分割系 Creation Functions とは

選択・分割系のCreation Functionsは、結合系とは異なり、以下の役割を持ちます。

- **選択**: 複数のObservableの中から、特定の条件を満たすものを選ぶ
- **分割**: 1つのObservableを条件に応じて複数のObservableに分ける

これらは「複数を1つにまとめる」結合とは逆方向、または異なる観点での操作を行います。

## 主要な選択・分割系 Creation Functions

| Function | 説明 | ユースケース |
|----------|------|-------------|
| **[race](/guide/creation-functions/selection/race)** | 最初に発行したものを採用 | 複数データソースの競争 |
| **[partition](/guide/creation-functions/selection/partition)** | 条件で2つに分割 | 成功/失敗の分岐処理 |

## 使い分けの基準

### race - 最速のObservableを選択

`race`は、複数のObservableを同時に購読し、**最初に値を発行したObservable**を採用します。採用されなかったObservableは自動的にunsubscribeされます。

**ユースケース**:
- 複数のAPIエンドポイントから最速のレスポンスを採用
- タイムアウト処理（本来の処理 vs タイマー）
- キャッシュと実際のAPI呼び出しの競争

```typescript
import { race, timer } from 'rxjs';
import { mapTo } from 'rxjs/operators';

// 複数のデータソースから最速のものを採用
const fast$ = timer(1000).pipe(mapTo('Fast API'));
const slow$ = timer(3000).pipe(mapTo('Slow API'));

race(fast$, slow$).subscribe(console.log);
// 出力: 'Fast API' (1秒後に出力され、slow$はキャンセルされる)
```

### partition - 条件で分割

`partition`は、1つのObservableを条件関数に基づいて**2つのObservable**に分割します。返り値は`[trueの場合, falseの場合]`という配列です。

**ユースケース**:
- 成功と失敗の分離
- 偶数と奇数の分離
- 有効なデータと無効なデータの分離

```typescript
import { of, partition } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// 偶数と奇数に分割
const [even$, odd$] = partition(source$, n => n % 2 === 0);

even$.subscribe(val => console.log('Even:', val));
// 出力: Even: 2, Even: 4, Even: 6

odd$.subscribe(val => console.log('Odd:', val));
// 出力: Odd: 1, Odd: 3, Odd: 5
```

## Pipeable Operator との対応関係

選択・分割系Creation Functionsにも、対応するPipeable Operatorが存在します。

| Creation Function | Pipeable Operator |
|-------------------|-------------------|
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(source$, predicate)` | パイプライン内では使えない（Creation Functionのみ） |

> [!NOTE]
> `partition`にはPipeable Operator版が存在しません。分割が必要な場合は、Creation Functionとして使用するか、`filter`を2回使って手動で分割します。

## 次のステップ

各Creation Functionの詳細な動作と実践例を学ぶには、上記の表からリンクをクリックしてください。

また、[結合系 Creation Functions](/guide/creation-functions/combination/)や[条件分岐系 Creation Functions](/guide/creation-functions/conditional/)も併せて学習することで、Creation Functionsの全体像を理解できます。
