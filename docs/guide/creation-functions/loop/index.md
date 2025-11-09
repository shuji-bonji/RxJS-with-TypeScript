---
description: ループ的に値を生成するCreation Functionsについて解説します。range、generateを使い、for文やwhile文のような繰り返し処理をObservableストリームとして実装する方法を学びます。連番生成からカスタム条件に基づく複雑な状態遷移まで、TypeScriptの型推論を活かした宣言的なループ処理を実現できます。
---

# ループ生成系 Creation Functions

for文やwhile文のようなループ処理をObservableとして表現するためのCreation Functionsです。

## ループ生成系 Creation Functions とは

ループ生成系のCreation Functionsは、繰り返し処理をリアクティブに実現します。従来の命令的なループ（`for`文、`while`文）を宣言的なObservableストリームに置き換えることで、RxJSのオペレーターチェーンと組み合わせた柔軟な処理が可能になります。

以下の表で、各Creation Functionの特徴と使い分けを確認してください。

## 主要なループ生成系 Creation Functions

| Function | 説明 | ユースケース |
|----------|------|-------------|
| **[range](/guide/creation-functions/loop/range)** | 数値の範囲を生成（for文的） | 連番生成、バッチ処理 |
| **[generate](/guide/creation-functions/loop/generate)** | 汎用的なループ生成（while文的） | 条件付き繰り返し、複雑な状態遷移 |

## 使い分けの基準

ループ生成系Creation Functionsの選択は、以下の観点で判断します。

### 1. 生成パターン

- **数値の連続**: `range()` - 開始値と終了値を指定するだけのシンプルな連番生成
- **複雑な条件**: `generate()` - 初期値、条件、イテレーション、結果選択を自由に制御

### 2. ループの種類

- **for文的なループ**: `range()` - `for (let i = start; i <= end; i++)`
- **while文的なループ**: `generate()` - `while (condition) { ... }`

### 3. 柔軟性

- **シンプルで十分**: `range()` - 数値の連続が必要な場合
- **高度な制御が必要**: `generate()` - カスタム状態管理、条件分岐、ステップ制御

## 実践的な使用例

### range() - 連番生成

シンプルな連番生成には`range()`が最適です。

```typescript
import { range, map } from 'rxjs';
// 1から5までの連番を生成
range(1, 5).subscribe(console.log);
// 出力: 1, 2, 3, 4, 5

// バッチ処理での活用
range(0, 10).pipe(
  map(i => `処理${i + 1}`)
).subscribe(console.log);
// 出力: 処理1, 処理2, ..., 処理10
```

### generate() - 条件付きループ

複雑な条件やカスタム状態管理が必要な場合は`generate()`を使用します。

```typescript
import { generate } from 'rxjs';

// フィボナッチ数列の生成（最初の10項）
generate(
  { current: 0, next: 1, count: 0 },  // 初期状態
  state => state.count < 10,           // 継続条件
  state => ({                          // 状態更新
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // 結果選択
).subscribe(console.log);
// 出力: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## 命令的ループとの比較

従来の命令的なループとRxJSのループ生成系Creation Functionsの比較です。

### 命令的なfor文

```typescript
// 従来のfor文
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### 宣言的なrange()

```typescript
import { range, map, toArray } from 'rxjs';
// RxJSのrange()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

> [!TIP]
> **宣言的アプローチの利点**:
> - パイプライン処理で可読性向上
> - エラーハンドリングが統一的
> - 非同期処理との組み合わせが容易
> - キャンセルや中断が簡単（`takeUntil()`など）

## Cold から Hot への変換

上記の表に示した通り、**全てのループ生成系Creation Functionsは Cold Observable を生成します**。購読するたびに独立した実行が開始されます。

しかし、マルチキャスト系オペレーター（`share()`, `shareReplay()` など）を使用することで、**Cold Observable を Hot Observable に変換**できます。

### 実践例：計算結果の共有

```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - 購読ごとに独立した計算
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('計算中:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('購読者1:', val));
cold$.subscribe(val => console.log('購読者2:', val));
// → 計算が2回実行される（2000回の計算）

// 🔥 Hot - 購読者間で計算結果を共有
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('計算中:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('購読者1:', val));
hot$.subscribe(val => console.log('購読者2:', val));
// → 計算は1回のみ実行される（1000回の計算）
```

> [!TIP]
> **Hot化が必要なケース**:
> - 高コストな計算を複数箇所で使用
> - バッチ処理の結果を複数コンポーネントで共有
> - ページネーション処理の結果を複数のUIコンポーネントで表示
>
> 詳しくは [基本作成系 - Cold から Hot への変換](/guide/creation-functions/basic/#cold-から-hot-への変換) を参照してください。

## 非同期処理との組み合わせ

ループ生成系Creation Functionsは、非同期処理と組み合わせることで強力な機能を発揮します。

### API呼び出しの連続実行

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
    items: [`データ${page}-1`, `データ${page}-2`, `データ${page}-3`]
  }).pipe(
    delay(300) // API呼び出しをシミュレート
  );
}

// ページ1から10までを順次取得（各リクエスト間に1秒の遅延）
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe(
  data => console.log(`ページ ${data.page} 取得:`, data.items),
  err => console.error('エラー:', err)
);
```

### リトライ処理での活用

```typescript
import { range, throwError, of, Observable, mergeMap, retryWhen, delay } from 'rxjs';
// データ取得をシミュレートする関数（ランダムに失敗する）
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // 40%の確率で成功

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('データ取得失敗'))
        : of('データ取得成功')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          // 最大3回リトライ
          if (index >= 3) {
            return throwError(() => error);
          }
          console.log(`リトライ ${index + 1}/3`);
          // 指数バックオフ: 1秒、2秒、4秒
          return range(0, 1).pipe(delay(Math.pow(2, index) * 1000));
        })
      )
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('結果:', result),
  error: err => console.error('エラー:', err.message)
});

// 出力例:
// リトライ 1/3
// リトライ 2/3
// 結果: データ取得成功
```

## Pipeable Operator との関係

ループ生成系Creation Functionsには、直接対応するPipeable Operatorはありません。これらは常にCreation Functionとして使用されます。

ただし、以下のようなオペレーターと組み合わせることで、より高度な処理が可能になります。

| 組み合わせるオペレーター | 用途 |
|-------------------|------|
| `map()` | 各値を変換 |
| `filter()` | 条件に一致する値のみ通過 |
| `take()`, `skip()` | 値の個数を制御 |
| `concatMap()`, `mergeMap()` | 各値で非同期処理を実行 |
| `toArray()` | すべての値を配列にまとめる |

## パフォーマンスに関する注意

ループ生成系Creation Functionsは同期的に値を発行するため、大量の値を生成する場合はパフォーマンスに注意が必要です。

> [!WARNING]
> **大量データの取り扱い**:
> - `range(1, 1000000)` のような大量データは、すべて同期的に発行されメモリを消費
> - 必要に応じて `bufferCount()` や `windowCount()` でバッファリング
> - または `scheduled()` でスケジューラーを指定し、非同期実行に変更

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// 非同期スケジューラーで実行
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## 次のステップ

各Creation Functionの詳細な動作と実践例を学ぶには、上記の表からリンクをクリックしてください。

また、[基本作成系 Creation Functions](/guide/creation-functions/basic/)、[結合系 Creation Functions](/guide/creation-functions/combination/)、[選択・分割系 Creation Functions](/guide/creation-functions/selection/)、[条件分岐系 Creation Functions](/guide/creation-functions/conditional/)も併せて学習することで、Creation Functionsの全体像を理解できます。
