---
description: repeatWhenオペレーターは完了時に別のObservableの発行を待って再購読を行い、条件付きリピート処理や動的な再試行間隔の実装に活用します。
---

# repeatWhen - 条件付きリピート

`repeatWhen` オペレーターは、ソースObservableが完了したときに**別のObservableが値を発行するまで待ってから再購読**します。動的な再試行間隔や条件に応じたリピート処理を実装できます。

## 🔰 基本構文・動作

完了通知を受け取るとnotifierファクトリー関数が呼ばれ、そのObservableが値を発行したタイミングで再購読します。

```ts
import { of } from 'rxjs';
import { repeatWhen, delay, take } from 'rxjs';

of(1, 2, 3)
  .pipe(
    repeatWhen(notifications =>
      notifications.pipe(
        delay(1000),  // 1秒待ってから再購読
        take(2)       // 2回だけリピート
      )
    )
  )
  .subscribe(console.log);
// 出力: 1, 2, 3 (初回)
// (1秒後) 1, 2, 3 (1回目)
// (1秒後) 1, 2, 3 (2回目)
```

notifierファクトリー関数は、完了通知のストリームを受け取り、再購読のタイミングを制御するObservableを返します。

[🌐 RxJS公式ドキュメント - repeatWhen](https://rxjs.dev/api/operators/repeatWhen)

> [!NOTE]
> RxJS 7.x以降では、`repeat`オペレーターに`delay`オプションが追加され、基本的なリピート遅延は`repeat({ count: 3, delay: 1000 })`で実現できます。`repeatWhen`はより複雑な条件制御が必要な場合に使用します。

## 💡 典型的な活用例

- **動的な再試行間隔**: 指数バックオフなど、条件に応じた間隔変更
- **ユーザー操作による再実行**: ボタンクリックで再購読
- **外部イベントによるリピート**: 他のストリームの状態に応じてリピート
- **条件付き繰り返し**: 特定の条件を満たす場合のみ再実行

## 🧪 実践コード例1: 指数バックオフ

リピート回数が増えるごとに待機時間を延ばす例です。

```ts
import { of, timer } from 'rxjs';
import { repeatWhen, mergeMap, take, tap } from 'rxjs';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'repeatWhen - 指数バックオフ';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '250px';
output.style.overflow = 'auto';
container.appendChild(output);

function addLog(message: string, color: string = '#e3f2fd') {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('ja-JP', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = `[${timestamp}] ${message}`;
  output.appendChild(logItem);
}

let executionCount = 0;

of('データ取得')
  .pipe(
    tap(() => {
      executionCount++;
      addLog(`${executionCount}回目の実行`, '#c8e6c9');
    }),
    repeatWhen(notifications =>
      notifications.pipe(
        mergeMap((_, index) => {
          // 指数バックオフ: 1秒 → 2秒 → 4秒 → 8秒
          const delayTime = Math.pow(2, index) * 1000;
          addLog(`次の実行まで ${delayTime}ms 待機...`, '#fff9c4');
          return timer(delayTime);
        }),
        take(3)  // 3回リピート
      )
    )
  )
  .subscribe({
    next: data => addLog(`受信: ${data}`, '#e3f2fd'),
    complete: () => addLog('--- すべての実行が完了 ---', '#c8e6c9')
  });
```

- 1回目実行後、1秒待機
- 2回目実行後、2秒待機
- 3回目実行後、4秒待機
- 4回目実行後、完了

## 🧪 実践コード例2: ユーザー操作による再実行

ボタンクリックで処理を再実行する例です。

```ts
import { of, fromEvent, Subject } from 'rxjs';
import { repeatWhen, tap, delay } from 'rxjs';

// UI作成
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'repeatWhen - ボタンクリックで再実行';
container2.appendChild(title2);

const button = document.createElement('button');
button.textContent = '再実行';
button.style.marginBottom = '10px';
button.style.padding = '10px 20px';
button.style.fontSize = '16px';
container2.appendChild(button);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

const repeatSubject = new Subject<void>();

let count = 0;

function addLog2(message: string, color: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('ja-JP', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.marginBottom = '2px';
  logItem.style.backgroundColor = color;
  logItem.style.fontSize = '14px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);

  // 自動スクロール
  output2.scrollTop = output2.scrollHeight;
}

// ボタンクリックでSubjectに通知
fromEvent(button, 'click').subscribe(() => {
  addLog2('ボタンがクリックされました', '#fff9c4');
  repeatSubject.next();
});

addLog2('初回実行開始...', '#e3f2fd');

of('タスク実行中...')
  .pipe(
    delay(500),  // 処理をシミュレート
    tap(() => {
      count++;
      addLog2(`タスク完了 (${count}回目)`, '#c8e6c9');
      addLog2('「再実行」ボタンをクリックしてください', '#e3f2fd');
    }),
    repeatWhen(() => repeatSubject)  // Subjectが発行したら再購読
  )
  .subscribe({
    next: message => addLog2(message, '#e3f2fd')
  });
```

- 初回実行後、ボタンクリック待ち
- ボタンをクリックすると再実行
- 何度でも再実行可能

## 🆚 repeat との比較

```ts
import { of, timer } from 'rxjs';
import { repeat, repeatWhen, delay } from 'rxjs';

// repeat - シンプルなリピート
of(1, 2, 3)
  .pipe(
    repeat(3)  // 3回リピート（即座に）
  )
  .subscribe(console.log);
// 出力: 1, 2, 3, 1, 2, 3, 1, 2, 3 (即座に)

// repeat with delay (RxJS 7+)
of(1, 2, 3)
  .pipe(
    repeat({ count: 3, delay: 1000 })  // 1秒間隔で3回
  )
  .subscribe(console.log);
// 出力: 1, 2, 3 (1秒後) 1, 2, 3 (1秒後) 1, 2, 3

// repeatWhen - 動的な制御
of(1, 2, 3)
  .pipe(
    repeatWhen(notifications =>
      notifications.pipe(
        delay((_, index) => timer(index * 1000))  // 動的な遅延
      )
    )
  )
  .subscribe(console.log);
// 出力: 1, 2, 3 (0秒後) 1, 2, 3 (1秒後) 1, 2, 3 (2秒後)...
```

| オペレーター | リピート制御 | 遅延制御 | ユースケース |
|:---|:---|:---|:---|
| `repeat` | 固定回数 | 固定間隔 | シンプルなリピート |
| `repeat({ delay })` | 固定回数 | 固定間隔 | 固定間隔のリピート |
| `repeatWhen` | 動的 | 動的 | 複雑な条件制御 |

> [!TIP]
> `repeat`の詳細については、[repeat](./repeat.md)を参照してください。

## ⚠️ 注意点

### 1. notifierが完了するとストリームも完了

```ts
import { of } from 'rxjs';
import { repeatWhen, delay, take } from 'rxjs';

of(1, 2, 3)
  .pipe(
    repeatWhen(notifications =>
      notifications.pipe(
        delay(1000),
        take(2)  // notifierが2回発行後に完了
      )
    )
  )
  .subscribe({
    next: console.log,
    complete: () => console.log('完了')
  });
// 出力: 1, 2, 3 (初回)
// (1秒後) 1, 2, 3 (1回目)
// (1秒後) 1, 2, 3 (2回目)
// 完了
```

notifierが完了すると、ソースObservableも完了します。

### 2. notifierがエラーを発生させるとストリームもエラー

```ts
import { of, throwError } from 'rxjs';
import { repeatWhen, mergeMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    repeatWhen(notifications =>
      notifications.pipe(
        mergeMap((_, index) => {
          if (index >= 2) {
            return throwError(() => new Error('リピート制限'));
          }
          return of(null);
        })
      )
    )
  )
  .subscribe({
    next: console.log,
    error: err => console.error('エラー:', err.message)
  });
// 出力: 1, 2, 3, 1, 2, 3, 1, 2, 3
// エラー: リピート制限
```

### 3. 無限リピートに注意

```ts
import { of } from 'rxjs';
import { repeatWhen, delay } from 'rxjs';

// ❌ 悪い例: 無限リピート
of(1, 2, 3)
  .pipe(
    repeatWhen(notifications =>
      notifications.pipe(
        delay(1000)  // take()がないため無限リピート
      )
    )
  )
  .subscribe(console.log);
// 永遠に繰り返される

// ✅ 良い例: 制限を設ける
of(1, 2, 3)
  .pipe(
    repeatWhen(notifications =>
      notifications.pipe(
        delay(1000),
        take(5)  // 5回でストップ
      )
    )
  )
  .subscribe(console.log);
```

### 4. エラー発生時はrepeatWhenは動作しない

`repeatWhen`は完了時のリピートのみを扱います。エラー時の再試行には`retryWhen`を使用します。

```ts
import { of, throwError, concat } from 'rxjs';
import { repeatWhen, delay } from 'rxjs';

concat(
  of(1, 2),
  throwError(() => new Error('エラー'))
)
  .pipe(
    repeatWhen(notifications =>
      notifications.pipe(delay(1000))  // 呼ばれない
    )
  )
  .subscribe({
    next: console.log,
    error: err => console.error('エラー:', err.message)
  });
// 出力: 1, 2
// エラー: エラー
```

## 実践的な組み合わせ例

```ts
import { of, timer } from 'rxjs';
import { repeatWhen, mergeMap, tap, take } from 'rxjs';

// 段階的にリピート間隔を変更する例
let attempt = 0;

of('API呼び出し')
  .pipe(
    tap(() => {
      attempt++;
      console.log(`試行 ${attempt}`);
    }),
    repeatWhen(notifications =>
      notifications.pipe(
        mergeMap((_, index) => {
          // 最初の3回: 1秒間隔
          // 次の3回: 5秒間隔
          // それ以降: 10秒間隔
          let delay: number;
          if (index < 3) {
            delay = 1000;
          } else if (index < 6) {
            delay = 5000;
          } else {
            delay = 10000;
          }

          console.log(`次の試行まで ${delay}ms 待機`);
          return timer(delay);
        }),
        take(9)  // 合計10回実行（初回 + 9回リピート）
      )
    )
  )
  .subscribe({
    next: console.log,
    complete: () => console.log('完了')
  });
```

## 📚 関連オペレーター

- **[repeat](./repeat)** - シンプルなリピート制御
- **[retry](./retry)** - エラー時の再試行
- **[retryWhen](./retryWhen)** - 条件付き再試行
- **[delay](./delay)** - 固定時間の遅延

## ✅ まとめ

`repeatWhen` オペレーターは、条件付きで再購読を制御します。

- ✅ 動的な再試行間隔を実装可能
- ✅ 外部イベントによるリピート制御
- ✅ 指数バックオフなど複雑なパターンに対応
- ✅ ユーザー操作による再実行に活用
- ⚠️ notifierの完了/エラーがストリームに影響
- ⚠️ 無限リピートに注意
- ⚠️ エラー時はrepeatWhenは動作しない（retryWhen使用）
