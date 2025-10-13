---
description: concatWithは、元のObservableの完了後に他のObservableを順番に結合するPipeable Operatorです。
---

# concatWith - パイプライン内で順番にストリームを結合する

`concatWith` オペレーターは、元のObservableが`complete`した後、指定された他のObservableを**順番に結合**します。
これは Creation Function の `concat` のPipeable Operator版です。

## 🔰 基本構文と使い方

```ts
import { of, delay } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));
const obs3$ = of('E', 'F').pipe(delay(100));

obs1$
  .pipe(concatWith(obs2$, obs3$))
  .subscribe(console.log);

// 出力: A → B → C → D → E → F
```

- `obs1$` が完了してから `obs2$` が開始され、`obs2$` が完了してから `obs3$` が開始されます。
- `.pipe()` チェーン内で使用できるため、他のオペレーターと組み合わせやすくなります。

[🌐 RxJS公式ドキュメント - `concatWith`](https://rxjs.dev/api/operators/concatWith)


## 💡 典型的な活用パターン

- **パイプライン内での順次処理**：変換後のストリームに追加データを順番に結合
- **完了後のフォローアップ処理**：メイン処理完了後にクリーンアップや通知を追加
- **段階的なデータロード**：初期データ取得後に追加データを順次取得


## 🧠 実践コード例（UI付き）

メインの検索結果を表示した後、関連する推奨アイテムを順番に表示する例です。

```ts
import { of, delay } from 'rxjs';
import { concatWith, map } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>concatWith の実践例:</h3>';
document.body.appendChild(output);

// メイン検索結果
const searchResults$ = of('🔍 検索結果1', '🔍 検索結果2', '🔍 検索結果3').pipe(
  delay(500)
);

// 推奨アイテム1
const recommendations1$ = of('💡 おすすめ商品A', '💡 おすすめ商品B').pipe(
  delay(300)
);

// 推奨アイテム2
const recommendations2$ = of('⭐ 人気商品X', '⭐ 人気商品Y').pipe(
  delay(300)
);

// 順番に結合して表示
searchResults$
  .pipe(
    concatWith(recommendations1$, recommendations2$),
    map((value, index) => `${index + 1}. ${value}`)
  )
  .subscribe((value) => {
    const item = document.createElement('div');
    item.textContent = value;
    output.appendChild(item);
  });
```

- 最初に検索結果が表示され、
- その後に推奨商品が順番に表示されます。
- パイプライン内で `map` などの他のオペレーターと組み合わせて使用できます。


## 🔄 Creation Function `concat` との違い

### 基本的な違い

| | `concat` (Creation Function) | `concatWith` (Pipeable Operator) |
|:---|:---|:---|
| **使用場所** | 独立した関数として使用 | `.pipe()` チェーン内で使用 |
| **記述方法** | `concat(obs1$, obs2$, obs3$)` | `obs1$.pipe(concatWith(obs2$, obs3$))` |
| **最初のストリーム** | すべて対等に扱う | メインストリームとして扱う |
| **利点** | シンプルで読みやすい | 他のオペレーターと組み合わせやすい |

### 使い分けの具体例

**シンプルな結合だけなら Creation Function がおすすめ**

```ts
import { concat, of } from 'rxjs';

const part1$ = of('A', 'B');
const part2$ = of('C', 'D');
const part3$ = of('E', 'F');

// シンプルで読みやすい
concat(part1$, part2$, part3$).subscribe(console.log);
// 出力: A → B → C → D → E → F
```

**途中で変換が必要なら Pipeable Operator がおすすめ**

```ts
import { of } from 'rxjs';
import { concatWith, map, filter } from 'rxjs';

const userData$ = of({ name: 'Alice', age: 30 }, { name: 'Bob', age: 25 });
const additionalData$ = of({ name: 'Charlie', age: 35 });

// ❌ Creation Function版 - 冗長になる
import { concat } from 'rxjs';
concat(
  userData$.pipe(
    filter(user => user.age >= 30),
    map(user => user.name)
  ),
  additionalData$.pipe(map(user => user.name))
).subscribe(console.log);

// ✅ Pipeable Operator版 - 一つのパイプラインで完結
userData$
  .pipe(
    filter(user => user.age >= 30),  // 30歳以上のみ
    map(user => user.name),          // 名前だけ抽出
    concatWith(
      additionalData$.pipe(map(user => user.name))
    )
  )
  .subscribe(console.log);
// 出力: Alice → Charlie
```

**メインストリームに後続処理を追加する場合**

```ts
import { fromEvent, of } from 'rxjs';
import { concatWith, take, mapTo } from 'rxjs';

// ボタンと出力エリア作成
const button = document.createElement('button');
button.textContent = '3回クリックしてください';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

const clicks$ = fromEvent(button, 'click');

// ✅ Pipeable Operator版 - メインストリームの延長として自然
clicks$
  .pipe(
    take(3),                          // 最初の3クリックを取得
    mapTo('クリックされました'),
    concatWith(of('完了しました'))    // 完了後に追加メッセージ
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = message;
    output.appendChild(div);
  });

// 同じ動作をCreation Function版で書くと...
// ❌ Creation Function版 - メインストリームを分けて書く必要がある
import { concat } from 'rxjs';
concat(
  clicks$.pipe(
    take(3),
    mapTo('クリックされました')
  ),
  of('完了しました')
).subscribe(console.log);
```

### まとめ

- **`concat`**: 複数のストリームをシンプルに結合するだけなら最適
- **`concatWith`**: メインストリームに対して変換や処理を加えながら後続を追加したい場合に最適


## ⚠️ 注意点

### 完了待ちによる遅延

元のObservableが完了しない限り、次のObservableは開始されません。

```ts
import { interval, of } from 'rxjs';
import { concatWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),              // 3つで完了させる
  concatWith(of('完了'))
).subscribe(console.log);
// 出力: 0 → 1 → 2 → 完了
```

### エラー処理

前のObservableでエラーが発生すると、後続のObservableは実行されません。

```ts
import { throwError, of } from 'rxjs';
import { concatWith, catchError } from 'rxjs';

throwError(() => new Error('エラー発生'))
  .pipe(
    catchError(err => of('エラーを回復')),
    concatWith(of('次の処理'))
  )
  .subscribe(console.log);
// 出力: エラーを回復 → 次の処理
```


## 📚 関連オペレーター

- **[concat](/guide/creation-functions/concat)** - Creation Function版
- **[mergeWith](/guide/operators/combination/mergeWith)** - 並列結合するPipeable版
- **[concatMap](/guide/operators/transformation/concatMap)** - 各値を順次マッピング
