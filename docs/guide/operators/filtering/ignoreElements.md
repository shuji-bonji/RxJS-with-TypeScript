---
description: ignoreElementsオペレーターは、すべての値を無視して完了とエラーのみを通すRxJSフィルタリングオペレーターです。処理の完了を待つ場合に便利です。
---

# ignoreElements - すべての値を無視して完了/エラーのみを通す

`ignoreElements` オペレーターは、ソースObservableから発行される**すべての値を無視**し、**完了通知とエラー通知のみ**を下流に通します。

## 🔰 基本構文と使い方

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs/operators';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('値:', value), // 呼ばれない
  complete: () => console.log('完了しました')
});
// 出力: 完了しました
```

**動作の流れ**:
1. 1, 2, 3, 4, 5 がすべて無視される
2. 完了通知のみが下流に伝わる

[🌐 RxJS公式ドキュメント - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 典型的な活用パターン

- **処理の完了待ち**：値は不要で完了だけを知りたい場合
- **副作用のみ実行**：tap で副作用を実行し、値は無視
- **エラーハンドリング**：エラーのみを捕捉したい場合
- **シーケンスの同期**：複数の処理の完了を待つ

## 🧠 実践コード例1: 初期化処理の完了待ち

複数の初期化処理が完了するのを待つ例です。

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs/operators';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'アプリケーション初期化';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// ステータスログを追加する関数
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// 初期化処理1: データベース接続
const initDatabase$ = from(['DB接続中...', 'テーブル確認中...', 'DB準備完了']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // 値は無視、完了のみ通知
);

// 初期化処理2: 設定ファイル読み込み
const loadConfig$ = from(['設定ファイル読み込み中...', '設定解析中...', '設定適用完了']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// 初期化処理3: ユーザー認証
const authenticate$ = from(['認証情報確認中...', 'トークン検証中...', '認証完了']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// すべての初期化処理を実行
addLog('初期化開始...', 'orange');

forkJoin([
  initDatabase$,
  loadConfig$,
  authenticate$
]).subscribe({
  complete: () => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#e8f5e9';
    completeMessage.style.color = 'green';
    completeMessage.style.fontWeight = 'bold';
    completeMessage.textContent = '✅ すべての初期化が完了しました！アプリケーションを起動できます。';
    addLog('アプリケーション起動', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ 初期化エラー: ${err.message}`;
  }
});
```

- 各初期化処理の詳細なログは表示されますが、値は無視されます。
- すべての処理が完了した時点で、完了メッセージが表示されます。

## 🎯 実践コード例2: ファイルアップロードの完了待ち

複数ファイルのアップロード進捗を表示しつつ、完了のみを通知する例です。

```ts
import { from, of, concat } from 'rxjs';
import { ignoreElements, tap, delay, mergeMap } from 'rxjs/operators';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'ファイルアップロード';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'アップロード開始';
container.appendChild(button);

const progressArea = document.createElement('div');
progressArea.style.marginTop = '10px';
container.appendChild(progressArea);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.display = 'none';
container.appendChild(result);

interface FileUpload {
  name: string;
  size: number;
}

const files: FileUpload[] = [
  { name: 'document.pdf', size: 2500 },
  { name: 'image.jpg', size: 1800 },
  { name: 'video.mp4', size: 5000 }
];

// ファイルアップロード処理（進捗表示付き）
function uploadFile(file: FileUpload) {
  const fileDiv = document.createElement('div');
  fileDiv.style.marginTop = '5px';
  fileDiv.style.padding = '5px';
  fileDiv.style.border = '1px solid #ccc';
  progressArea.appendChild(fileDiv);

  const progressSteps = [0, 25, 50, 75, 100];

  return from(progressSteps).pipe(
    delay(200),
    tap(progress => {
      fileDiv.textContent = `📄 ${file.name} (${file.size}KB) - ${progress}%`;
      if (progress === 100) {
        fileDiv.style.backgroundColor = '#e8f5e9';
      }
    }),
    ignoreElements() // 進捗値は無視、完了のみ通知
  );
}

button.addEventListener('click', () => {
  button.disabled = true;
  progressArea.innerHTML = '';
  result.style.display = 'none';

  // すべてのファイルを順次アップロード
  from(files).pipe(
    mergeMap(file => uploadFile(file), 2) // 最大2つ並行
  ).subscribe({
    complete: () => {
      result.style.display = 'block';
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
      result.innerHTML = `
        <strong>✅ アップロード完了</strong><br>
        ${files.length}個のファイルがアップロードされました
      `;
      button.disabled = false;
    },
    error: err => {
      result.style.display = 'block';
      result.style.backgroundColor = '#ffebee';
      result.style.color = 'red';
      result.textContent = `❌ エラー: ${err.message}`;
      button.disabled = false;
    }
  });
});
```

- 各ファイルの進捗は表示されますが、進捗値そのものは下流に流れません。
- すべてのアップロードが完了した時点で、完了メッセージが表示されます。

## 🆚 類似オペレーターとの比較

### ignoreElements vs filter(() => false) vs take(0)

```ts
import { of } from 'rxjs';
import { ignoreElements, filter, take } from 'rxjs/operators';

const source$ = of(1, 2, 3);

// ignoreElements: すべての値を無視、完了は通す
source$.pipe(
  ignoreElements()
).subscribe({
  next: v => console.log('値:', v),
  complete: () => console.log('ignoreElements: 完了')
});
// 出力: ignoreElements: 完了

// filter(() => false): すべての値をフィルタリング、完了は通す
source$.pipe(
  filter(() => false)
).subscribe({
  next: v => console.log('値:', v),
  complete: () => console.log('filter: 完了')
});
// 出力: filter: 完了

// take(0): 即座に完了
source$.pipe(
  take(0)
).subscribe({
  next: v => console.log('値:', v),
  complete: () => console.log('take(0): 完了')
});
// 出力: take(0): 完了
```

| オペレーター | 値の処理 | 完了通知 | ユースケース |
|:---|:---|:---|:---|
| `ignoreElements()` | すべて無視 | 通す | **完了のみ必要**（推奨） |
| `filter(() => false)` | すべてフィルタリング | 通す | 条件フィルタリング（偶然すべて除外） |
| `take(0)` | 即座に完了 | 通す | 即座に完了させたい |

**推奨**: 意図的にすべての値を無視したい場合は `ignoreElements()` を使用してください。コードの意図が明確になります。

## 🔄 エラー通知の扱い

`ignoreElements` は値を無視しますが、**エラー通知は通します**。

```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs/operators';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('エラー発生'))
).pipe(
  ignoreElements()
);

// 成功ケース
success$.subscribe({
  next: v => console.log('値:', v),
  complete: () => console.log('✅ 完了'),
  error: err => console.error('❌ エラー:', err.message)
});
// 出力: ✅ 完了

// エラーケース
error$.subscribe({
  next: v => console.log('値:', v),
  complete: () => console.log('✅ 完了'),
  error: err => console.error('❌ エラー:', err.message)
});
// 出力: ❌ エラー: エラー発生
```

## ⚠️ 注意点

### 1. 副作用は実行される

`ignoreElements` は値を無視しますが、副作用（`tap` など）は実行されます。

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs/operators';

of(1, 2, 3).pipe(
  tap(v => console.log('副作用:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('値:', v),
  complete: () => console.log('完了')
});
// 出力:
// 副作用: 1
// 副作用: 2
// 副作用: 3
// 完了
```

### 2. 無限Observable での使用

無限Observableで使用すると、完了が来ないため永遠に購読が続きます。

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs/operators';

// ❌ 悪い例: 完了しない
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('完了') // 呼ばれない
});

// ✅ 良い例: take で完了させる
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('完了') // 5秒後に呼ばれる
});
```

### 3. TypeScript での型

`ignoreElements` の返り値は `Observable<never>` 型です。

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs/operators';

const numbers$: Observable<number> = of(1, 2, 3);

// ignoreElements の結果は Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value は never 型なので、このブロックは実行されない
    console.log(value);
  },
  complete: () => console.log('完了のみ')
});
```

### 4. 完了が保証されない場合

ソースが完了しない場合、`ignoreElements` も完了しません。

```ts
import { NEVER } from 'rxjs';
import { ignoreElements } from 'rxjs/operators';

// ❌ NEVERは完了もエラーも発行しない
NEVER.pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('完了') // 呼ばれない
});
```

## 💡 実用的な組み合わせパターン

### パターン1: 初期化シーケンス

```ts
import { of, concat } from 'rxjs';
import { tap, ignoreElements, delay } from 'rxjs/operators';

const initStep1$ = of('Step 1').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

const initStep2$ = of('Step 2').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

const initStep3$ = of('Step 3').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

// すべてのステップを順次実行
concat(initStep1$, initStep2$, initStep3$).subscribe({
  complete: () => console.log('✅ すべての初期化完了')
});
```

### パターン2: クリーンアップ処理

```ts
import { from, of } from 'rxjs';
import { tap, ignoreElements, mergeMap } from 'rxjs/operators';

interface Resource {
  id: number;
  name: string;
}

const resources: Resource[] = [
  { id: 1, name: 'Database' },
  { id: 2, name: 'Cache' },
  { id: 3, name: 'Logger' }
];

from(resources).pipe(
  mergeMap(resource =>
    of(resource).pipe(
      tap(() => console.log(`🧹 ${resource.name} をクリーンアップ中...`)),
      ignoreElements()
    )
  )
).subscribe({
  complete: () => console.log('✅ すべてのリソースをクリーンアップしました')
});
```

## 📚 関連オペレーター

- **[filter](./filter)** - 条件に基づいて値をフィルタリング
- **[take](./take)** - 最初のN個の値のみ取得
- **[skip](./skip)** - 最初のN個の値をスキップ
- **[tap](../utility/tap)** - 副作用を実行

## まとめ

`ignoreElements` オペレーターは、すべての値を無視して完了とエラーのみを通します。

- ✅ 完了の通知のみ必要な場合に最適
- ✅ 副作用（tap）は実行される
- ✅ エラー通知も通す
- ✅ `filter(() => false)` より意図が明確
- ⚠️ 無限Observableでは完了しない
- ⚠️ 返り値の型は `Observable<never>`
- ⚠️ 値は完全に無視されるが、副作用は実行される
