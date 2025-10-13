---
description: timeoutWithオペレーターは指定時間内に値が発行されない場合に別のObservableに切り替え、タイムアウト時のフォールバック処理を提供します。
---

# timeoutWith - タイムアウトとフォールバック

`timeoutWith` オペレーターは、指定時間内に値が発行されない場合に**別のObservableに切り替え**ます。`timeout`オペレーターがエラーを発生させるのに対し、`timeoutWith`はフォールバック処理を提供します。

## 🔰 基本構文・動作

タイムアウト時に切り替える代替Observableを指定します。

```ts
import { of, timer } from 'rxjs';
import { timeoutWith, concatMap } from 'rxjs/operators';

// 3秒かかる処理
of('データ')
  .pipe(
    concatMap(v => timer(3000).pipe(map(() => v))),
    timeoutWith(2000, of('フォールバック'))  // 2秒でタイムアウト
  )
  .subscribe(console.log);
// 出力: フォールバック
```

指定時間内に値が発行されない場合、代替Observableに切り替わります。

[🌐 RxJS公式ドキュメント - timeoutWith](https://rxjs.dev/api/operators/timeoutWith)

> [!NOTE]
> RxJS 7.x以降では、`timeout`オペレーターに`with`オプションが追加され、`timeoutWith`の機能を代替できます。ただし、`timeoutWith`も引き続き利用可能です。

## 💡 典型的な活用例

- **APIのフォールバック**: タイムアウト時にキャッシュデータを使用
- **リトライ代替**: タイムアウト時に別の処理パスへ切り替え
- **ユーザー通知**: タイムアウト時に警告メッセージを表示
- **デフォルト値の提供**: 応答がない場合のデフォルトデータ

## 🧪 実践コード例1: APIフォールバック

APIリクエストがタイムアウトした場合にキャッシュデータを使用する例です。

```ts
import { of, throwError, timer } from 'rxjs';
import { timeoutWith, mergeMap, delay } from 'rxjs/operators';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'timeoutWith - APIフォールバック';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'データを取得';
button.style.marginBottom = '10px';
container.appendChild(button);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.appendChild(logItem);
}

// キャッシュデータ（フォールバック用）
const cachedData = of({
  data: 'キャッシュされたデータ（古い）',
  timestamp: '2024-01-01 00:00:00'
});

// 遅いAPIをシミュレート
function slowAPI(delayMs: number) {
  return timer(delayMs).pipe(
    mergeMap(() => of({
      data: '最新のAPIデータ',
      timestamp: new Date().toLocaleString('ja-JP')
    }))
  );
}

button.addEventListener('click', () => {
  output.innerHTML = '';
  addLog('APIリクエスト開始...', '#e3f2fd');

  slowAPI(3000)  // 3秒かかる
    .pipe(
      timeoutWith(2000, cachedData)  // 2秒でタイムアウト
    )
    .subscribe({
      next: result => {
        const isCache = result.data.includes('キャッシュ');
        const color = isCache ? '#fff9c4' : '#c8e6c9';
        const label = isCache ? '⚠️ タイムアウト - キャッシュデータ' : '✅ 取得成功';

        addLog(label, color);
        addLog(`データ: ${result.data}`, '#f5f5f5');
        addLog(`タイムスタンプ: ${result.timestamp}`, '#f5f5f5');
      },
      error: err => addLog(`❌ エラー: ${err.message}`, '#ffcdd2')
    });
});
```

- APIが2秒以内に応答すれば最新データを使用
- タイムアウトした場合はキャッシュデータにフォールバック
- ユーザーにデータソースを明示

## 🧪 実践コード例2: 複数のフォールバック戦略

優先順位付きの複数フォールバックを実装する例です。

```ts
import { of, timer, throwError } from 'rxjs';
import { timeoutWith, mergeMap, tap } from 'rxjs/operators';

// UI作成
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'timeoutWith - 多段フォールバック';
container2.appendChild(title2);

const button2 = document.createElement('button');
button2.textContent = 'データ取得（3段階フォールバック）';
button2.style.marginBottom = '10px';
container2.appendChild(button2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '250px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.marginBottom = '2px';
  logItem.style.backgroundColor = color;
  logItem.style.fontSize = '14px';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

// データソース関数
function primaryAPI() {
  addLog2('1️⃣ プライマリAPIにアクセス中...', '#e3f2fd');
  return timer(3000).pipe(
    mergeMap(() => of({ source: 'Primary API', data: 'primary-data' }))
  );
}

function secondaryAPI() {
  addLog2('2️⃣ セカンダリAPIにフォールバック...', '#fff9c4');
  return timer(3000).pipe(
    mergeMap(() => of({ source: 'Secondary API', data: 'secondary-data' }))
  );
}

function cachedData() {
  addLog2('3️⃣ キャッシュデータを使用', '#ffccbc');
  return of({ source: 'Cache', data: 'cached-data' });
}

button2.addEventListener('click', () => {
  output2.innerHTML = '';

  primaryAPI()
    .pipe(
      timeoutWith(1000, secondaryAPI()),  // 1秒でセカンダリへ
      timeoutWith(1500, cachedData())     // さらに1.5秒でキャッシュへ
    )
    .subscribe({
      next: result => {
        const colors: Record<string, string> = {
          'Primary API': '#c8e6c9',
          'Secondary API': '#fff9c4',
          'Cache': '#ffccbc'
        };

        addLog2(
          `✅ データ取得成功: ${result.source} から ${result.data}`,
          colors[result.source]
        );
      },
      error: err => addLog2(`❌ エラー: ${err.message}`, '#ffcdd2')
    });
});
```

- プライマリAPIが1秒以内に応答しなければセカンダリへ
- セカンダリも1.5秒以内に応答しなければキャッシュへ
- 段階的なフォールバック戦略

## 🆚 timeout との比較

```ts
import { timer, of, throwError } from 'rxjs';
import { timeout, timeoutWith, catchError } from 'rxjs/operators';

// timeout - エラーを発生
timer(2000)
  .pipe(
    timeout(1000)
  )
  .subscribe({
    next: console.log,
    error: err => console.error('タイムアウトエラー:', err.message)
  });
// エラー: Timeout has occurred

// timeoutWith - 代替Observableに切り替え
timer(2000)
  .pipe(
    timeoutWith(1000, of('フォールバック'))
  )
  .subscribe({
    next: v => console.log('値:', v),
    error: () => {}  // 呼ばれない
  });
// 値: フォールバック
```

| オペレーター | タイムアウト時の動作 | ユースケース |
|:---|:---|:---|
| `timeout` | エラーを発生 | エラーハンドリングが必要な場合 |
| `timeoutWith` | 代替Observableに切り替え | フォールバックデータを提供する場合 |

> [!TIP]
> `timeout`の詳細については、[timeout](./timeout.md)を参照してください。

## ⚠️ 注意点

### 1. 代替Observableは購読時に評価される

```ts
import { timer, of } from 'rxjs';
import { timeoutWith } from 'rxjs/operators';

let callCount = 0;

// ❌ 悪い例: 購読時に毎回評価される
timer(2000)
  .pipe(
    timeoutWith(1000, of(++callCount))
  )
  .subscribe();

// ✅ 良い例: defer を使用
import { defer } from 'rxjs';

timer(2000)
  .pipe(
    timeoutWith(1000, defer(() => of(++callCount)))
  )
  .subscribe();
```

### 2. 代替Observableもタイムアウトする可能性

```ts
import { timer } from 'rxjs';
import { timeoutWith } from 'rxjs/operators';

timer(3000)
  .pipe(
    timeoutWith(1000,
      timer(3000)  // この代替Observableも遅い
    ),
    timeoutWith(1500, of('最終フォールバック'))
  )
  .subscribe(console.log);
// 出力: 最終フォールバック
```

### 3. エラーは発生しない

`timeoutWith`を使用すると、タイムアウトによるエラーは発生しません。

```ts
import { timer, of } from 'rxjs';
import { timeoutWith } from 'rxjs/operators';

timer(2000)
  .pipe(
    timeoutWith(1000, of('フォールバック'))
  )
  .subscribe({
    next: console.log,
    error: () => console.log('エラー'),      // 呼ばれない
    complete: () => console.log('完了')
  });
// 出力:
// フォールバック
// 完了
```

## 実践的な組み合わせ例

```ts
import { timer, of } from 'rxjs';
import { timeoutWith, retry, catchError } from 'rxjs/operators';

// リトライとフォールバックの組み合わせ
function apiWithRetryAndFallback() {
  return timer(3000).pipe(
    mergeMap(() => {
      // ランダムに失敗
      if (Math.random() < 0.7) {
        return throwError(() => new Error('API Error'));
      }
      return of('API Success');
    }),
    retry(2),                                    // 2回リトライ
    timeoutWith(5000, of('Cached Data')),       // 5秒でフォールバック
    catchError(() => of('Error Fallback'))       // エラー時のフォールバック
  );
}

apiWithRetryAndFallback().subscribe(console.log);
// 可能性のある出力:
// - 'API Success' (成功)
// - 'Cached Data' (タイムアウト)
// - 'Error Fallback' (エラー)
```

## 📚 関連オペレーター

- **[timeout](./timeout)** - タイムアウト制御（エラー発生）
- **[catchError](../../error-handling/retry-catch)** - エラーハンドリング
- **[retry](./retry)** - エラー時の再試行
- **[switchMap](../transformation/switchMap)** - Observable の切り替え

## ✅ まとめ

`timeoutWith` オペレーターは、タイムアウト時に代替Observableに切り替えます。

- ✅ タイムアウト時のフォールバック処理
- ✅ エラーを発生させずにデータを提供
- ✅ 多段階のフォールバック戦略が可能
- ✅ APIリクエストのキャッシュ活用に最適
- ⚠️ 代替Observableは購読時に評価される
- ⚠️ 代替Observableもタイムアウトする可能性
- ⚠️ タイムアウトによるエラーは発生しない
