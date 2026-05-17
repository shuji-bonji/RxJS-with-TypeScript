---
description: "mergeWithは、元のObservableと他のObservableを同時に購読して並列に結合するPipeable Operatorです。複数のイベントソースを統合してリアルタイムに処理する場合に活用できます。merge()との違い、TypeScriptでの型安全な実装を解説します。"
---

# mergeWith - 複数ストリームを並行結合

`mergeWith` オペレーターは、元のObservableと指定された他のObservableを**同時に購読**し、
それぞれから発行される値をリアルタイムに統合します。
これは Creation Function の `merge` のPipeable Operator版です。

## 🔰 基本構文と使い方

```ts
import { interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `ストリーム1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `ストリーム2: ${val}`),
  take(2)
);

source1$
  .pipe(mergeWith(source2$))
  .subscribe(console.log);

// 出力例:
// ストリーム1: 0
// ストリーム2: 0
// ストリーム1: 1
// ストリーム1: 2
// ストリーム2: 1
```

- すべてのObservableを同時に購読し、**発行された順**に値が流れます。
- 順番の保証はなく、**各Observableの発行タイミングに依存**します。

[🌐 RxJS公式ドキュメント - `mergeWith`](https://rxjs.dev/api/operators/mergeWith)


## 💡 典型的な活用パターン

- **複数のイベントソースを統合**：ユーザー操作と自動更新の統合
- **並列データフェッチの結合**：複数のAPIからの応答を単一ストリームに集約
- **リアルタイム更新のマージ**：WebSocketとポーリングを統合


## 🧠 実践コード例（UI付き）

ユーザーのクリックイベントと自動更新タイマーを統合して通知を表示する例です。

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>mergeWith の実践例:</h3>';
document.body.appendChild(output);

// ボタン作成
const button = document.createElement('button');
button.textContent = '手動更新';
document.body.appendChild(button);

// クリックストリーム
const manualUpdate$ = fromEvent(button, 'click').pipe(
  map(() => '👆 手動更新が実行されました')
);

// 自動更新タイマー（5秒ごと）
const autoUpdate$ = interval(5000).pipe(
  map(val => `🔄 自動更新 #${val + 1}`),
  take(3)
);

// 両方を統合して表示
manualUpdate$
  .pipe(mergeWith(autoUpdate$))
  .subscribe((value) => {
    const timestamp = new Date().toLocaleTimeString();
    const item = document.createElement('div');
    item.textContent = `[${timestamp}] ${value}`;
    output.appendChild(item);
  });
```

- ボタンをクリックすると即座に手動更新が表示され、
- 5秒ごとに自動更新も並行して実行されます。
- 両方のイベントがリアルタイムで統合されます。


## 🔄 Creation Function `merge` との違い

### 基本的な違い

| | `merge` (Creation Function) | `mergeWith` (Pipeable Operator) |
|:---|:---|:---|
| **使用場所** | 独立した関数として使用 | `.pipe()` チェーン内で使用 |
| **記述方法** | `merge(obs1$, obs2$, obs3$)` | `obs1$.pipe(mergeWith(obs2$, obs3$))` |
| **最初のストリーム** | すべて対等に扱う | メインストリームとして扱う |
| **利点** | シンプルで読みやすい | 他のオペレーターと組み合わせやすい |

### 使い分けの具体例

**シンプルなマージだけなら Creation Function がおすすめ**

```ts
import { merge, fromEvent } from 'rxjs';
import { map } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(map(() => 'クリック'));
const moves$ = fromEvent(document, 'mousemove').pipe(map(() => 'マウス移動'));
const keypress$ = fromEvent(document, 'keypress').pipe(map(() => 'キー入力'));

// シンプルで読みやすい
merge(clicks$, moves$, keypress$).subscribe(console.log);
// 出力: いずれかのイベントが発生した順に表示
```

**メインストリームに変換処理を加える場合は Pipeable Operator がおすすめ**

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, filter, throttleTime } from 'rxjs';

const userClicks$ = fromEvent(document, 'click');
const autoRefresh$ = interval(30000); // 30秒ごと

// ✅ Pipeable Operator版 - 一つのパイプラインで完結
userClicks$
  .pipe(
    throttleTime(1000),           // 連打防止
    map(() => ({ source: 'user', timestamp: Date.now() })),
    mergeWith(
      autoRefresh$.pipe(
        map(() => ({ source: 'auto', timestamp: Date.now() }))
      )
    ),
    filter(event => event.timestamp > Date.now() - 60000)  // 1分以内のみ
  )
  .subscribe(event => {
    console.log(`${event.source}更新: ${new Date(event.timestamp).toLocaleTimeString()}`);
  });

// ❌ Creation Function版 - 冗長になる
import { merge } from 'rxjs';
merge(
  userClicks$.pipe(
    throttleTime(1000),
    map(() => ({ source: 'user', timestamp: Date.now() }))
  ),
  autoRefresh$.pipe(
    map(() => ({ source: 'auto', timestamp: Date.now() }))
  )
).pipe(
  filter(event => event.timestamp > Date.now() - 60000)
).subscribe(event => {
  console.log(`${event.source}更新: ${new Date(event.timestamp).toLocaleTimeString()}`);
});
```

**複数のデータソースを統合する場合**

```ts
import { fromEvent, timer } from 'rxjs';
import { mergeWith, map, startWith } from 'rxjs';

// ボタン作成
const saveButton = document.createElement('button');
saveButton.textContent = '保存';
document.body.appendChild(saveButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// メインストリーム: ユーザーの保存操作
const manualSave$ = fromEvent(saveButton, 'click').pipe(
  map(() => '💾 手動保存')
);

// ✅ Pipeable Operator版 - メインストリームに自動保存を追加
manualSave$
  .pipe(
    startWith('📝 編集開始'),
    mergeWith(
      timer(10000, 10000).pipe(map(() => '⏰ 自動保存'))  // 10秒ごとに自動保存
    )
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
    output.appendChild(div);
  });
```

### まとめ

- **`merge`**: 複数のストリームを対等に統合するだけなら最適
- **`mergeWith`**: メインストリームに対して変換や処理を加えながら他のストリームを統合したい場合に最適


## ⚠️ 注意点

### 完了タイミング

すべてのObservableが完了するまで、結合されたストリームは完了しません。

```ts
import { of, interval, NEVER } from 'rxjs';
import { mergeWith, take } from 'rxjs';

of(1, 2, 3).pipe(
  mergeWith(
    interval(1000).pipe(take(2)),
    // NEVER  // ← これを追加すると永遠に完了しない
  )
).subscribe({
  next: console.log,
  complete: () => console.log('✅ 完了')
});
// 出力: 1 → 2 → 3 → 0 → 1 → ✅ 完了
```

### 並行実行数の制御

デフォルトではすべてのストリームを同時実行しますが、`mergeMap`との組み合わせで制御できます。

```ts
import { from, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

from([1, 2, 3, 4, 5]).pipe(
  mergeMap(
    val => of(val).pipe(delay(1000)),
    2  // 最大2つまで並行実行
  )
).subscribe(console.log);
```

### エラー処理

いずれかのObservableでエラーが発生すると、全体がエラーで終了します。

```ts
import { throwError, interval } from 'rxjs';
import { mergeWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  mergeWith(
    throwError(() => new Error('エラー発生')).pipe(
      catchError((err: unknown) => of('エラーを回復'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('エラー:', err.message)
});
// 出力: 0 → エラーを回復 → 1
```


## 📚 関連オペレーター

- **[merge](/guide/creation-functions/combination/merge)** - Creation Function版
- **[concatWith](/guide/operators/combination/concatWith)** - 順次結合するPipeable版
- **[mergeMap](/guide/operators/transformation/mergeMap)** - 各値を並列マッピング
