---
description: combineLatestWithは元のObservableと他のObservableの最新値を組み合わせて出力するRxJS結合オペレーターです。フォーム入力のリアルタイム検証、複数の状態の同期、計算結果のリアルタイム更新など、依存する複数のストリームの最新値を常に監視したい場面に最適です。pipeable operator版でパイプライン内での使用が便利です。
---

# combineLatestWith - 最新値を結合

`combineLatestWith` オペレーターは、元のObservableと指定された他のObservableの**最新の値をすべてまとめて出力**します。
いずれかのObservableから新しい値が発行されるたびに、すべての最新値をまとめた結果が発行されます。
これは Creation Function の `combineLatest` のPipeable Operator版です。

## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// 出力例:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

- 各Observableが**少なくとも1つ値を発行してから**、組み合わせた値が出力されます。
- どちらか一方に新しい値が来るたびに、最新のペアが再出力されます。

[🌐 RxJS公式ドキュメント - `combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)


## 💡 典型的な活用パターン

- **フォーム入力のリアルタイム検証**：複数フィールドの最新状態を常に監視
- **依存する複数の状態の同期**：設定値とユーザー入力の組み合わせ
- **計算結果のリアルタイム更新**：複数の入力値から結果を即座に算出


## 🧠 実践コード例（UI付き）

価格と数量の入力から、リアルタイムで合計金額を計算する例です。

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>combineLatestWith の実践例:</h3>';
document.body.appendChild(output);

// 入力フィールド作成
const priceInput = document.createElement('input');
priceInput.type = 'number';
priceInput.placeholder = '単価';
priceInput.value = '100';
document.body.appendChild(priceInput);

const quantityInput = document.createElement('input');
quantityInput.type = 'number';
quantityInput.placeholder = '数量';
quantityInput.value = '1';
document.body.appendChild(quantityInput);

// 結果表示エリア
const result = document.createElement('div');
result.style.fontSize = '20px';
result.style.marginTop = '10px';
document.body.appendChild(result);

// 各入力のObservable
const price$ = fromEvent(priceInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(100)
);

const quantity$ = fromEvent(quantityInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(1)
);

// 最新の値を組み合わせて計算
price$
  .pipe(
    combineLatestWith(quantity$),
    map(([price, quantity]) => price * quantity)
  )
  .subscribe((total) => {
    result.innerHTML = `<strong>合計金額: ¥${total.toLocaleString()}</strong>`;
  });
```

- いずれかのフィールドに入力すると、**最新の2つの値から合計が即座に再計算**されます。
- `startWith()` を使うことで、最初から組み合わせ結果を得られるようにしています。


## 🔄 Creation Function `combineLatest` との違い

### 基本的な違い

| | `combineLatest` (Creation Function) | `combineLatestWith` (Pipeable Operator) |
|:---|:---|:---|
| **使用場所** | 独立した関数として使用 | `.pipe()` チェーン内で使用 |
| **記述方法** | `combineLatest([obs1$, obs2$])` | `obs1$.pipe(combineLatestWith(obs2$))` |
| **最初のストリーム** | すべて対等に扱う | メインストリームとして扱う |
| **返り値** | 配列 `[val1, val2]` | タプル `[val1, val2]` |
| **利点** | シンプルで読みやすい | 他のオペレーターと組み合わせやすい |

### 使い分けの具体例

**シンプルな組み合わせだけなら Creation Function がおすすめ**

```ts
import { combineLatest, of } from 'rxjs';

const firstName$ = of('太郎');
const lastName$ = of('山田');
const age$ = of(30);

// シンプルで読みやすい
combineLatest([firstName$, lastName$, age$]).subscribe(([first, last, age]) => {
  console.log(`${last} ${first}さん (${age}歳)`);
});
// 出力: 山田 太郎さん (30歳)
```

**メインストリームに変換処理を加える場合は Pipeable Operator がおすすめ**

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, debounceTime } from 'rxjs';

const searchInput = document.createElement('input');
searchInput.placeholder = '検索...';
document.body.appendChild(searchInput);

const categorySelect = document.createElement('select');
categorySelect.innerHTML = '<option>すべて</option><option>本</option><option>DVD</option>';
document.body.appendChild(categorySelect);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// メインストリーム: 検索キーワード
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  debounceTime(300),  // 入力後300ms待つ
  startWith('')
);

// サブストリーム: カテゴリー選択
const category$ = fromEvent(categorySelect, 'change').pipe(
  map(e => (e.target as HTMLSelectElement).value),
  startWith('すべて')
);

// ✅ Pipeable Operator版 - 一つのパイプラインで完結
searchTerm$
  .pipe(
    map(term => term.toLowerCase()),  // 小文字に変換
    combineLatestWith(category$),
    map(([term, category]) => ({
      term,
      category,
      timestamp: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(result => {
    output.textContent = `検索: "${result.term}" カテゴリー: ${result.category} [${result.timestamp}]`;
  });

// ❌ Creation Function版 - 冗長になる
import { combineLatest } from 'rxjs';
combineLatest([
  searchTerm$.pipe(map(term => term.toLowerCase())),
  category$
]).pipe(
  map(([term, category]) => ({
    term,
    category,
    timestamp: new Date().toLocaleTimeString()
  }))
).subscribe(result => {
  output.textContent = `検索: "${result.term}" カテゴリー: ${result.category} [${result.timestamp}]`;
});
```

**複数の設定値を組み合わせる場合**

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// スライダー作成
const redSlider = document.createElement('input');
redSlider.type = 'range';
redSlider.min = '0';
redSlider.max = '255';
redSlider.value = '255';
document.body.appendChild(document.createTextNode('Red: '));
document.body.appendChild(redSlider);
document.body.appendChild(document.createElement('br'));

const greenSlider = document.createElement('input');
greenSlider.type = 'range';
greenSlider.min = '0';
greenSlider.max = '255';
greenSlider.value = '0';
document.body.appendChild(document.createTextNode('Green: '));
document.body.appendChild(greenSlider);
document.body.appendChild(document.createElement('br'));

const blueSlider = document.createElement('input');
blueSlider.type = 'range';
blueSlider.min = '0';
blueSlider.max = '255';
blueSlider.value = '0';
document.body.appendChild(document.createTextNode('Blue: '));
document.body.appendChild(blueSlider);

const colorBox = document.createElement('div');
colorBox.style.width = '200px';
colorBox.style.height = '100px';
colorBox.style.marginTop = '10px';
colorBox.style.border = '1px solid #ccc';
document.body.appendChild(colorBox);

// メインストリーム: Red
const red$ = fromEvent(redSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(255)
);

// ✅ Pipeable Operator版 - Redをメインに他の色を組み合わせる
red$
  .pipe(
    combineLatestWith(
      fromEvent(greenSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      ),
      fromEvent(blueSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      )
    ),
    map(([r, g, b]) => `rgb(${r}, ${g}, ${b})`)
  )
  .subscribe(color => {
    colorBox.style.backgroundColor = color;
    colorBox.textContent = color;
    colorBox.style.display = 'flex';
    colorBox.style.alignItems = 'center';
    colorBox.style.justifyContent = 'center';
    colorBox.style.color = '#fff';
    colorBox.style.textShadow = '1px 1px 2px #000';
  });
```

### まとめ

- **`combineLatest`**: 複数のストリームをシンプルに組み合わせるだけなら最適
- **`combineLatestWith`**: メインストリームに対して変換や処理を加えながら他のストリームの最新値を組み合わせたい場合に最適


## ⚠️ 注意点

### 初期値が揃うまで発行されない

すべてのObservableが少なくとも1つ値を発行するまで、結果は出力されません。

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER)  // 値を発行しないObservable
).subscribe(console.log);
// 出力なし（NEVERが値を発行しないため）
```

この場合、`startWith()` で初期値を与えることで解決できます。

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take, startWith } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER.pipe(startWith(null)))
).subscribe(console.log);
// 出力: [0, null] → [1, null] → [2, null]
```

### 頻繁な再発行に注意

いずれかのストリームが頻繁に値を発行すると、結果も頻繁に再発行されます。

```ts
import { interval } from 'rxjs';
import { combineLatestWith } from 'rxjs';

// 100msごとに発行されるストリーム
const fast$ = interval(100);
const slow$ = interval(1000);

fast$.pipe(
  combineLatestWith(slow$)
).subscribe(console.log);
// slow$が発行されるたびに、fast$の最新値と組み合わされる
// → パフォーマンスに注意が必要
```

### エラー処理

いずれかのObservableでエラーが発生すると、全体がエラーで終了します。

```ts
import { throwError, interval } from 'rxjs';
import { combineLatestWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  combineLatestWith(
    throwError(() => new Error('エラー発生')).pipe(
      catchError((err: unknown) => of('回復'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error(err.message)
});
// 出力: [0, '回復'] → [1, '回復']
```


## 📚 関連オペレーター

- **[combineLatest](/guide/creation-functions/combination/combineLatest)** - Creation Function版
- **[withLatestFrom](/guide/operators/combination/withLatestFrom)** - メインストリームのみがトリガー
- **[zipWith](/guide/operators/combination/zipWith)** - 対応する値をペア化
