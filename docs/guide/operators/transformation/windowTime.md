---
description: "windowTimeオペレーターは一定時間ごとにObservableを分割し、各時間枠で発行された値を個別のObservableとして処理できます。時間ベースのストリーム分割、バッチ処理、bufferTimeとの違い、TypeScriptでの型安全な実装を解説します。"
---

# windowTime - 一定時間ごとにObservableを分割する

`windowTime`オペレーターは、**一定時間ごと**にソースObservableの値をグループ化し、そのグループを**新しいObservableとして出力**します。
`bufferTime`が配列を返すのに対し、`windowTime`は**Observable\<T>を返す**ため、各ウィンドウに対してさらにオペレーターを適用できます。

## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, take } from 'rxjs';

// 100msごとに値を発行
const source$ = interval(100);

source$.pipe(
  windowTime(1000), // 1秒ごとにウィンドウを作成
  take(3),          // 最初の3つのウィンドウのみ
  mergeAll()        // 各ウィンドウをフラット化
).subscribe(value => {
  console.log('値:', value);
});

// 出力:
// 1秒目: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// 2秒目: 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
// 3秒目: 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
```

- 指定した時間（1000ms）ごとに新しいウィンドウ（Observable）が作成されます。
- 各ウィンドウは独立したObservableとして処理できます。

[🌐 RxJS公式ドキュメント - `windowTime`](https://rxjs.dev/api/operators/windowTime)

## 💡 典型的な活用パターン

- **時間ベースのバッチ処理**：一定時間ごとにデータをまとめて処理
- **リアルタイムデータの集計**：毎秒のイベント数をカウント
- **パフォーマンス監視**：一定時間ごとのメトリクス収集
- **時系列データの分析**：時間枠ごとの統計処理

## 🔍 bufferTime との違い

| オペレーター | 出力 | ユースケース |
|:---|:---|:---|
| `bufferTime` | **配列 (T[])** | グループ化された値をまとめて処理 |
| `windowTime` | **Observable\<T>** | 時間枠ごとに異なるストリーム処理 |

```ts
import { interval } from 'rxjs';
import { bufferTime, windowTime, take } from 'rxjs';

const source$ = interval(100);

// bufferTime - 配列として出力
source$.pipe(
  bufferTime(1000),
  take(2)
).subscribe(values => {
  console.log('バッファ（配列）:', values);
  // 出力: バッファ（配列）: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// windowTime - Observable として出力
source$.pipe(
  windowTime(1000),
  take(2)
).subscribe(window$ => {
  console.log('ウィンドウ（Observable）:', window$);
  // ネスト subscribe: window 系オペレーターの仕様上必要なパターン
  // （windowTime が発行する内側 Observable を消費するために必要）
  window$.subscribe(value => {
    console.log('  値:', value);
  });
});
```

## 🧠 実践コード例1: 毎秒のクリック数をカウント

ボタンのクリック数を1秒ごとに集計する例です。

```ts
import { fromEvent } from 'rxjs';
import { windowTime, map, mergeAll, scan } from 'rxjs';

// ボタン作成
const button = document.createElement('button');
button.textContent = 'クリック';
document.body.appendChild(button);

// 出力エリア
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// クリックイベント
const clicks$ = fromEvent(button, 'click');

let windowNumber = 0;

clicks$.pipe(
  windowTime(1000), // 1秒ごとにウィンドウを作成
  map(window$ => {
    ++windowNumber;

    // 各ウィンドウ内のクリック数をカウント
    return window$.pipe(
      scan(count => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  const timestamp = new Date().toLocaleTimeString();
  output.textContent = `[${timestamp}] ウィンドウ ${windowNumber}: ${count}回クリック`;
});
```

- 1秒ごとに新しいウィンドウが作成されます。
- 各ウィンドウ内のクリック数がリアルタイムでカウントされます。

## 🎯 実践コード例2: 時間枠ごとの統計処理

各時間枠の値の合計と平均を計算する例です。

```ts
import { interval } from 'rxjs';
import { windowTime, map, mergeMap, toArray, take } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>時間枠ごとの統計処理（1秒ごと）</h3>';
document.body.appendChild(output);

const table = document.createElement('table');
table.style.borderCollapse = 'collapse';
table.style.marginTop = '10px';
table.innerHTML = `
  <thead>
    <tr style="background: #f0f0f0;">
      <th style="border: 1px solid #ccc; padding: 8px;">ウィンドウ</th>
      <th style="border: 1px solid #ccc; padding: 8px;">件数</th>
      <th style="border: 1px solid #ccc; padding: 8px;">合計</th>
      <th style="border: 1px solid #ccc; padding: 8px;">平均</th>
    </tr>
  </thead>
  <tbody id="stats-body"></tbody>
`;
output.appendChild(table);

const source$ = interval(100).pipe(
  map(() => Math.floor(Math.random() * 100)) // ランダムな値
);

let windowNumber = 0;

source$.pipe(
  windowTime(1000), // 1秒ごと
  take(5),          // 5つのウィンドウのみ
  mergeMap(window$ => {
    const current = ++windowNumber;

    // 各ウィンドウの値を配列に変換して統計処理
    return window$.pipe(
      toArray(),
      map(values => ({
        window: current,
        count: values.length,
        sum: values.reduce((a, b) => a + b, 0),
        avg: values.length > 0
          ? (values.reduce((a, b) => a + b, 0) / values.length).toFixed(2)
          : 0
      }))
    );
  })
).subscribe(stats => {
  const tbody = document.getElementById('stats-body')!;
  const row = document.createElement('tr');
  row.innerHTML = `
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.window}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.count}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.sum}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.avg}</td>
  `;
  tbody.appendChild(row);
});
```

- 各ウィンドウの統計情報を個別に計算できます。
- ウィンドウごとに異なる処理を適用可能です。
- テーブル形式で統計データが視覚的に表示されます。

## 📊 オーバーラップするウィンドウ（windowCreationInterval）

第2引数に`windowCreationInterval`を指定すると、ウィンドウを重複させることができます。

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray, take, map } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>オーバーラップするウィンドウ</h3>';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.marginTop = '10px';
document.body.appendChild(output);

const source$ = interval(100);

source$.pipe(
  windowTime(
    2000,  // ウィンドウの長さ: 2秒
    1000   // ウィンドウの作成間隔: 1秒
  ),
  take(3),
  mergeMap((window$, index) =>
    window$.pipe(
      toArray(),
      map(values => ({ window: index + 1, values }))
    )
  )
).subscribe(result => {
  const div = document.createElement('div');
  div.style.marginTop = '10px';
  div.style.padding = '5px';
  div.style.backgroundColor = '#f5f5f5';
  div.style.borderLeft = '3px solid #4CAF50';

  const title = document.createElement('strong');
  title.textContent = `ウィンドウ ${result.window}:`;
  div.appendChild(title);

  div.appendChild(document.createElement('br'));

  const values = document.createElement('span');
  values.textContent = `値: [${result.values.join(', ')}]`;
  div.appendChild(values);

  div.appendChild(document.createElement('br'));

  const info = document.createElement('span');
  info.style.color = '#666';
  info.textContent = `(${result.values.length}個の値、${(result.window - 1)}秒〜${(result.window + 1)}秒)`;
  div.appendChild(info);

  output.appendChild(div);

  // Chrome対策: レンダリングを強制
  void output.offsetHeight;
});
```

**動作の説明：**
- **ウィンドウ 1**: 0秒〜2秒の値 `[0, 1, 2, ..., 19]` (20個)
- **ウィンドウ 2**: 1秒〜3秒の値 `[10, 11, 12, ..., 29]` (20個) ← 値10-19がウィンドウ1と重複
- **ウィンドウ 3**: 2秒〜4秒の値 `[20, 21, 22, ..., 39]` (20個) ← 値20-29がウィンドウ2と重複

- ウィンドウの長さ（2秒）より短い間隔（1秒）で新しいウィンドウを作成すると、重複が発生します。
- スライディングウィンドウの実装に便利です。

## 🎯 実用例: リアルタイムイベント監視

```ts
import { fromEvent } from 'rxjs';
import { windowTime, mergeMap, toArray, map } from 'rxjs';

// 出力エリア
const output = document.createElement('div');
output.innerHTML = '<h3>マウス移動監視（5秒ごと）</h3>';
document.body.appendChild(output);

const list = document.createElement('ul');
output.appendChild(list);

// マウス移動イベント
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  windowTime(5000), // 5秒ごと
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(events => ({
        count: events.length,
        timestamp: new Date().toLocaleTimeString()
      }))
    )
  )
).subscribe(result => {
  const item = document.createElement('li');
  item.textContent = `[${result.timestamp}] マウス移動: ${result.count}回`;
  list.insertBefore(item, list.firstChild);

  // 最大10件まで表示
  while (list.children.length > 10) {
    list.removeChild(list.lastChild!);
  }
});
```

## ⚠️ 注意点

### 1. ウィンドウのサブスクリプション管理

各ウィンドウは独立したObservableなので、明示的に購読する必要があります。

```ts
source$.pipe(
  windowTime(1000)
).subscribe(window$ => {
  // ネスト subscribe: window 系オペレーターの仕様上必要なパターン
  // ウィンドウ自体を購読しないと値は流れない
  window$.subscribe(value => {
    console.log('値:', value);
  });
});
```

または、`mergeAll()`, `concatAll()`, `switchAll()`などを使用してフラット化します。

```ts
source$.pipe(
  windowTime(1000),
  mergeAll() // すべてのウィンドウをマージ
).subscribe(value => {
  console.log('値:', value);
});
```

### 2. メモリ管理

長時間実行される場合、適切に購読解除することが重要です。

```ts
import { takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

const destroy$ = new Subject<void>();

source$.pipe(
  windowTime(1000),
  mergeAll(),
  takeUntil(destroy$) // 破棄時に購読解除
).subscribe();

// コンポーネント破棄時など
destroy$.next();
destroy$.complete();
```

### 3. 最大値の指定（maxWindowSize）

第3引数で各ウィンドウの最大値数を制限できます。

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray } from 'rxjs';

interval(100).pipe(
  windowTime(
    2000,      // ウィンドウの長さ: 2秒
    undefined, // ウィンドウ作成間隔: デフォルト（重複なし）
    5          // 最大値数: 5個まで
  ),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('ウィンドウ:', values);
  // 最大5個の値のみが含まれる
});
```

## 🆚 window系オペレーターの比較

| オペレーター | 区切りのタイミング | ユースケース |
|:---|:---|:---|
| `window` | 別のObservableの発行 | イベント駆動型の分割 |
| `windowTime` | **一定時間** | **時間ベースの分割** |
| `windowCount` | 一定個数 | 個数ベースの分割 |
| `windowToggle` | 開始・終了のObservable | 動的な開始/終了制御 |
| `windowWhen` | 動的なクロージング条件 | ウィンドウごとに異なる終了条件 |

## 📚 関連オペレーター

- **[bufferTime](./bufferTime)** - 配列として値をまとめる（windowTimeの配列版）
- **[window](./window)** - Observable発行でウィンドウ分割
- **[windowCount](./windowCount)** - 個数ベースでウィンドウ分割
- **[windowToggle](./windowToggle)** - 開始・終了のObservableでウィンドウ制御
- **[windowWhen](./windowWhen)** - 動的なクロージング条件でウィンドウ分割

## まとめ

`windowTime`オペレーターは、時間ベースでストリームを分割し、各時間枠を独立したObservableとして処理できる強力なツールです。

- ✅ 一定時間ごとに自動的にウィンドウを作成
- ✅ 各ウィンドウに対して異なる処理を適用可能
- ✅ スライディングウィンドウ（重複）にも対応
- ✅ リアルタイムデータの集計・分析に最適
- ⚠️ サブスクリプション管理が必要
- ⚠️ メモリ管理に注意
