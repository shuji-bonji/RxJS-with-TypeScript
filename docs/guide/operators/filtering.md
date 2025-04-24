# フィルタリングオペレーター

RxJSのフィルタリングオペレーターは、ストリームから必要なデータだけを選び取り、不要なデータを流さないための重要なツールです。これにより、アプリケーションの効率やパフォーマンスが大幅に向上します。

フィルタリングオペレーターは、ストリーム内の値を選別して、特定の条件を満たすもののみを通過させるRxJSのオペレーターです。データの流れを制御し、必要な値だけを処理することで、効率的なデータ処理パイプラインを構築できます。

## 基本的なフィルタリングオペレーター

### filter

`filter`オペレーターは、与えられた条件関数に基づいて値をフィルタリングします。条件を満たす値のみが出力ストリームに含まれます。

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs/operators';

// 出力を表示するための要素を作成
const output = document.createElement('div');
document.body.appendChild(output);

// 数値の配列
const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

// 偶数のみをフィルタリング
numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(value => {
  const item = document.createElement('div');
  item.textContent = `偶数: ${value}`;
  output.appendChild(item);
});

// 出力:
// 偶数: 2
// 偶数: 4
// 偶数: 6
// 偶数: 8
// 偶数: 10
```
> [!NOTE]  
> 💡 `filter()` は配列の `Array.prototype.filter()` に似ていますが、ストリーム内の値に対して逐次的に評価を行う点で異なります。非同期データに対してもシームレスに適用できます。

### take

`take`オペレーターは、指定した数の値だけを取得し、それ以降の値は無視します。

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs/operators';

// 出力要素を作成
const takeOutput = document.createElement('div');
document.body.appendChild(takeOutput);

// 1秒ごとに値を発行
const source$ = interval(1000);

// 最初の5つの値だけを取得
source$.pipe(
  take(5)
).subscribe({
  next: value => {
    const item = document.createElement('div');
    item.textContent = `値: ${value}`;
    takeOutput.appendChild(item);
  },
  complete: () => {
    const complete = document.createElement('div');
    complete.textContent = '完了しました';
    complete.style.fontWeight = 'bold';
    takeOutput.appendChild(complete);
  }
});

// 出力:
// 値: 0
// 値: 1
// 値: 2
// 値: 3
// 値: 4
// 完了しました
```

> [!NOTE]
> ✅ `take()` は、最初の数件だけを処理したいときに便利です。たとえば「最初の5件だけUIに表示する」といった用途で役立ちます。

### first

`first`オペレーターは、ストリームの最初の値だけを取得します。オプションで条件関数を指定すると、その条件を満たす最初の値を取得できます。

```ts
import { from } from 'rxjs';
import { first } from 'rxjs/operators';

// 出力要素を作成
const firstOutput = document.createElement('div');
document.body.appendChild(firstOutput);

const numbers$ = from([1, 2, 3, 4, 5]);

// 最初の値だけを取得
numbers$.pipe(
  first()
).subscribe(value => {
  const item = document.createElement('div');
  item.textContent = `最初の値: ${value}`;
  firstOutput.appendChild(item);
});

// 条件を満たす最初の値を取得
numbers$.pipe(
  first(n => n > 3)
).subscribe(value => {
  const item = document.createElement('div');
  item.textContent = `3より大きい最初の値: ${value}`;
  firstOutput.appendChild(item);
});

// 出力:
// 最初の値: 1
// 3より大きい最初の値: 4
```

> [!NOTE]
> ✅ `first()` は、「最初に条件を満たす1件」だけを取得したいときに活用されます。

### last

`last`オペレーターは、ストリームの最後の値だけを取得します。オプションで条件関数を指定することもできます。

```ts
import { from } from 'rxjs';
import { last } from 'rxjs/operators';

// 出力要素を作成
const lastOutput = document.createElement('div');
document.body.appendChild(lastOutput);

const numbers$ = from([1, 2, 3, 4, 5]);

// 最後の値だけを取得
numbers$.pipe(
  last()
).subscribe(value => {
  const item = document.createElement('div');
  item.textContent = `最後の値: ${value}`;
  lastOutput.appendChild(item);
});

// 条件を満たす最後の値を取得
numbers$.pipe(
  last(n => n < 5)
).subscribe(value => {
  const item = document.createElement('div');
  item.textContent = `5未満の最後の値: ${value}`;
  lastOutput.appendChild(item);
});

// 出力:
// 最後の値: 5
// 5未満の最後の値: 4
```

> [!NOTE]
> ✅ `last()` は、ストリームの完了時に「最後の値」または「最後に条件を満たした値」だけを取り出すのに便利です。

## 時間ベースのフィルタリングオペレーター

### debounceTime

`debounceTime`オペレーターは、指定した時間内に新しい値が発行されなかった場合のみ、最後の値を出力します。入力フィールドの検索などで頻繁に使用されます。

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs/operators';

// 入力フィールドと結果表示領域を作成
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = '検索語を入力...';
container.appendChild(searchInput);

const searchResults = document.createElement('div');
searchResults.style.marginTop = '10px';
container.appendChild(searchResults);

// 入力イベントを購読
fromEvent(searchInput, 'input').pipe(
  // 入力値を取得
  map(event => (event.target as HTMLInputElement).value),
  // 300ミリ秒間入力がなければ値を出力
  debounceTime(300)
).subscribe(value => {
  searchResults.textContent = `「${value}」の検索結果を取得中...`;
  // 実際の検索API呼び出しなど
});

// 使い方の説明
const instruction = document.createElement('p');
instruction.textContent = '上の入力欄に入力してください。入力が300ms停止すると検索が実行されます。';
container.appendChild(instruction);
```
> [!NOTE]
> 🔍 `debounceTime()` は検索ボックスなどのユーザー入力に最適で、「最後の入力から300ms以内に次の入力がなければ実行」という形で頻繁なイベント発火を抑制します。

### throttleTime

`throttleTime`オペレーターは、指定した時間ごとに最初の値のみを出力し、その間の値はスキップします。スクロールイベントやマウス移動など、頻繁に発生するイベントの処理に適しています。

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs/operators';

// マウス座標表示エリアを作成
const mouseContainer = document.createElement('div');
mouseContainer.style.height = '200px';
mouseContainer.style.border = '1px solid #ccc';
mouseContainer.style.padding = '10px';
mouseContainer.textContent = 'マウスをこの領域内で動かしてください';
document.body.appendChild(mouseContainer);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
document.body.appendChild(positionDisplay);

// マウス移動イベント
const mouseMoves$ = fromEvent(mouseContainer, 'mousemove');

// マウス位置を取得
mouseMoves$.pipe(
  map(event => ({
    x: (event as MouseEvent).clientX,
    y: (event as MouseEvent).clientY
  })),
  // 100ミリ秒ごとに1回だけ値を出力
  throttleTime(100)
).subscribe(position => {
  positionDisplay.textContent = `マウス位置: X=${position.x}, Y=${position.y}`;
});
```
> [!NOTE]
> 🔍 `throttleTime()` はスクロールイベントやマウス移動など、処理頻度を制限したい場面に最適です。

## 条件ベースのフィルタリングオペレーター

### distinctUntilChanged

`distinctUntilChanged`オペレーターは、連続して同じ値が発行された場合に、重複を除去します。直前の値と異なる場合のみ値を出力します。

```ts
import { from } from 'rxjs';
import { distinctUntilChanged } from 'rxjs/operators';

// 出力領域を作成
const distinctOutput = document.createElement('div');
document.body.appendChild(distinctOutput);

const title = document.createElement('h3');
title.textContent = 'distinctUntilChanged の例';
distinctOutput.appendChild(title);

const numbers$ = from([1, 1, 2, 2, 3, 1, 2, 3]);

// 連続した重複値を除去
numbers$.pipe(
  distinctUntilChanged()
).subscribe(value => {
  const item = document.createElement('div');
  item.textContent = `値: ${value}`;
  distinctOutput.appendChild(item);
});

// 出力:
// 値: 1
// 値: 2
// 値: 3
// 値: 1
// 値: 2
// 値: 3
```
> [!NOTE]
> 📌 同じ値が連続して流れるときに無駄な処理やAPI呼び出しを抑える用途で使います。フォームのバリデーションや検索キーワードの入力検知などによく使われます。

### distinctUntilKeyChanged

`distinctUntilKeyChanged`オペレーターは、オブジェクトの特定のプロパティが前回の値と異なる場合のみ、値を出力します。

```ts
import { from } from 'rxjs';
import { distinctUntilKeyChanged } from 'rxjs/operators';

// 出力領域を作成
const keyChangedOutput = document.createElement('div');
document.body.appendChild(keyChangedOutput);

const keyTitle = document.createElement('h3');
keyTitle.textContent = 'distinctUntilKeyChanged の例';
keyChangedOutput.appendChild(keyTitle);

const users = [
  { id: 1, name: '田中' },
  { id: 2, name: '田中' }, // nameが前回と同じ
  { id: 3, name: '佐藤' },
  { id: 4, name: '鈴木' },
  { id: 5, name: '鈴木' }, // nameが前回と同じ
  { id: 6, name: '田中' }
];

// nameプロパティの変更のみを検出
from(users).pipe(
  distinctUntilKeyChanged('name')
).subscribe(user => {
  const item = document.createElement('div');
  item.textContent = `ユーザー: ID=${user.id}, 名前=${user.name}`;
  keyChangedOutput.appendChild(item);
});

// 出力:
// ユーザー: ID=1, 名前=田中
// ユーザー: ID=3, 名前=佐藤
// ユーザー: ID=4, 名前=鈴木
// ユーザー: ID=6, 名前=田中
```

## 実践的なユースケース

### ユーザー入力の処理

検索入力のフィルタリングとdebounce処理の例を見てみましょう。

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter } from 'rxjs/operators';

// UI要素を作成
const searchContainer = document.createElement('div');
document.body.appendChild(searchContainer);

const searchTitle = document.createElement('h2');
searchTitle.textContent = '検索フィルタリングの例';
searchContainer.appendChild(searchTitle);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = '検索語を入力（3文字以上）...';
searchInput.style.padding = '8px';
searchInput.style.width = '300px';
searchContainer.appendChild(searchInput);

const resultsContainer = document.createElement('div');
resultsContainer.style.marginTop = '10px';
resultsContainer.style.padding = '10px';
resultsContainer.style.border = '1px solid #eee';
resultsContainer.style.minHeight = '100px';
searchContainer.appendChild(resultsContainer);

// 検索処理
fromEvent(searchInput, 'input').pipe(
  // 入力値を取得
  map(event => (event.target as HTMLInputElement).value.trim()),
  // 300ms間入力がないときに処理
  debounceTime(300),
  // 前回と同じ値なら無視
  distinctUntilChanged(),
  // 空の検索を無視
  filter(term => term.length >= 3)
).subscribe(searchTerm => {
  resultsContainer.innerHTML = '';
  
  const searchInfo = document.createElement('div');
  searchInfo.textContent = `「${searchTerm}」の検索結果を表示中...`;
  resultsContainer.appendChild(searchInfo);
  
  // 検索結果のシミュレーション
  setTimeout(() => {
    const results = ['結果1', '結果2', '結果3'].map(item => 
      `${item}: ${searchTerm}に関連する情報`
    );
    
    resultsContainer.innerHTML = '';
    
    if (results.length === 0) {
      const noResults = document.createElement('div');
      noResults.textContent = '結果が見つかりませんでした';
      resultsContainer.appendChild(noResults);
    } else {
      results.forEach(result => {
        const resultItem = document.createElement('div');
        resultItem.textContent = result;
        resultItem.style.padding = '5px';
        resultItem.style.margin = '5px 0';
        resultItem.style.backgroundColor = '#f5f5f5';
        resultsContainer.appendChild(resultItem);
      });
    }
  }, 500);
});

// 使い方の説明
const searchInstructions = document.createElement('p');
searchInstructions.textContent = '3文字以上入力すると検索が実行されます。入力後300ms待機します。同じ検索語は無視されます。';
searchContainer.appendChild(searchInstructions);
```

### 無限スクロールのシミュレーション

```ts
import { fromEvent } from 'rxjs';
import { map, filter, throttleTime, distinctUntilChanged, tap, scan } from 'rxjs/operators';

// UI要素を作成
const scrollContainer = document.createElement('div');
document.body.appendChild(scrollContainer);

const scrollTitle = document.createElement('h2');
scrollTitle.textContent = '無限スクロールのシミュレーション';
scrollContainer.appendChild(scrollTitle);

// スクロール可能なコンテナ
const scrollableArea = document.createElement('div');
scrollableArea.style.height = '200px';
scrollableArea.style.overflow = 'auto';
scrollableArea.style.border = '1px solid #ccc';
scrollableArea.style.padding = '10px';
scrollContainer.appendChild(scrollableArea);

// アイテムリスト
const itemsList = document.createElement('div');
itemsList.id = 'items-list';
scrollableArea.appendChild(itemsList);

// ローディングインジケータ
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'データを読み込み中...';
loadingIndicator.style.display = 'none';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.backgroundColor = '#f0f0f0';
loadingIndicator.style.textAlign = 'center';
scrollableArea.appendChild(loadingIndicator);

// 最初のアイテムを追加
function addItems(page) {
  for (let i = 1; i <= 10; i++) {
    const item = document.createElement('div');
    item.className = 'item';
    item.textContent = `アイテム ${(page - 1) * 10 + i}`;
    item.style.padding = '10px';
    item.style.margin = '5px 0';
    item.style.backgroundColor = '#f9f9f9';
    item.style.borderRadius = '4px';
    itemsList.appendChild(item);
  }
}

// 最初のページを表示
addItems(1);

// スクロールイベント
const scroll$ = fromEvent(scrollableArea, 'scroll');

// 無限スクロール実装
scroll$.pipe(
  // 200msごとに1回だけ処理
  throttleTime(200),
  // スクロール位置を計算
  map(() => {
    const scrollTop = scrollableArea.scrollTop;
    const scrollHeight = scrollableArea.scrollHeight;
    const clientHeight = scrollableArea.clientHeight;
    
    // スクロール位置のパーセンテージ
    return {
      scrollPercentage: (scrollTop / (scrollHeight - clientHeight)) * 100
    };
  }),
  // 前回と同じ位置なら無視
  distinctUntilChanged((prev, current) => 
    Math.floor(prev.scrollPercentage) === Math.floor(current.scrollPercentage)
  ),
  // 80%以上スクロールしたときのみ
  filter(({ scrollPercentage }) => scrollPercentage > 80),
  // ページ数を追跡
  scan((currentPage) => currentPage + 1, 1),
  // 最大5ページまで
  filter(page => page <= 5)
).subscribe(page => {
  // ローディング表示
  loadingIndicator.style.display = 'block';
  
  // データ取得を擬似的に遅延させる
  setTimeout(() => {
    loadingIndicator.style.display = 'none';
    addItems(page);
    
    // 最後のページでメッセージを表示
    if (page === 5) {
      const endMessage = document.createElement('div');
      endMessage.textContent = 'すべてのデータを読み込みました';
      endMessage.style.padding = '10px';
      endMessage.style.textAlign = 'center';
      endMessage.style.color = '#666';
      itemsList.appendChild(endMessage);
    }
  }, 800);
});

// 使い方の説明
const scrollInstructions = document.createElement('p');
scrollInstructions.textContent = '上のエリアをスクロールして下さい。80%以上スクロールすると新しいアイテムが読み込まれます。';
scrollContainer.appendChild(scrollInstructions);
```
## フィルタリングオペレーターの選び方

| やりたいこと | オペレーター | 説明 |
|--------------|--------------|------|
| 条件に合う値だけ通す | `filter` | 最も基本的なフィルタ |
| 最初の数件だけ取得 | `take`, `first` | 初期値だけ処理したいときに便利 |
| 一定時間入力が止まるまで待つ | `debounceTime` | 入力ボックスに最適 |
| 一定間隔で値を処理 | `throttleTime` | スクロールやマウス移動など |
| 同じ値の連続を無視 | `distinctUntilChanged` | 重複入力や再検索を防止 |

## まとめ

フィルタリングオペレーターは、データストリームから必要な情報だけを選択し、不要なデータを除外するための強力なツールです。主なポイントは以下の通りです。

1. 基本的なフィルタリングには`filter`、`take`、`skip`などのオペレーターを使用
2. 時間ベースのフィルタリングには`debounceTime`、`throttleTime`などを使用
3. 重複値の除外には`distinctUntilChanged`や`distinctUntilKeyChanged`が有用
4. 複数のフィルタリングオペレーターを組み合わせることで、複雑なデータフロー制御が可能

これらのオペレーターを使いこなすことで、特に非同期イベント駆動のUIやAPI通信などにおいて、効率的かつ宣言的なコードを作成できます。適切なフィルタリングは、不要な処理の削減やユーザーエクスペリエンスの向上に直結します。
