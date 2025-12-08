---
description: "delayWhenオペレーターは各値の遅延タイミングを個別のObservableで動的に制御します。条件に応じた柔軟な遅延処理、リトライ時の指数バックオフ、ユーザー操作待ち、API呼び出しの間隔調整など、実践的なパターンをTypeScriptコード例で解説します。"
---

# delayWhen - 動的な遅延制御

`delayWhen` オペレーターは、各値の遅延時間を**個別のObservableで動的に決定**します。`delay`オペレーターが固定時間の遅延を提供するのに対し、`delayWhen`は値ごとに異なる遅延を適用できます。

## 🔰 基本構文・動作

各値に対して遅延を決定するObservableを返す関数を指定します。

```ts
import { of, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    delayWhen(value => {
      const delayTime = value === 'B' ? 2000 : 1000;
      return timer(delayTime);
    })
  )
  .subscribe(console.log);
// 出力:
// A (1秒後)
// C (1秒後)
// B (2秒後)
```

この例では、値 `'B'` だけ2秒の遅延が適用され、他は1秒の遅延になります。

[🌐 RxJS公式ドキュメント - delayWhen](https://rxjs.dev/api/index/function/delayWhen)

## 💡 典型的な活用例

- **値に応じた遅延**: 優先度やタイプに基づいて遅延時間を変更
- **外部イベントによる遅延**: ユーザーの操作や他のストリームの完了を待つ
- **条件付き遅延**: 特定の値だけ遅延させる
- **非同期タイミング制御**: APIレスポンスやデータの準備完了を待つ

## 🧪 実践コード例1: 優先度による遅延

タスクの優先度に応じて処理タイミングを制御する例です。

```ts
import { from, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'delayWhen - 優先度による遅延';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '300px';
output.style.overflow = 'auto';
container.appendChild(output);

interface Task {
  id: number;
  name: string;
  priority: 'high' | 'medium' | 'low';
}

const tasks: Task[] = [
  { id: 1, name: 'タスクA', priority: 'low' },
  { id: 2, name: 'タスクB', priority: 'high' },
  { id: 3, name: 'タスクC', priority: 'medium' },
  { id: 4, name: 'タスクD', priority: 'high' },
  { id: 5, name: 'タスクE', priority: 'low' }
];

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const time = now.toLocaleTimeString('ja-JP', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${time}] ${message}`;
  output.appendChild(logItem);
}

addLog('処理開始', '#e3f2fd');

from(tasks)
  .pipe(
    delayWhen(task => {
      // 優先度に応じて遅延時間を設定
      let delayTime: number;
      switch (task.priority) {
        case 'high':
          delayTime = 500;  // 高優先度: 0.5秒
          break;
        case 'medium':
          delayTime = 1500; // 中優先度: 1.5秒
          break;
        case 'low':
          delayTime = 3000; // 低優先度: 3秒
          break;
      }
      return timer(delayTime);
    })
  )
  .subscribe({
    next: task => {
      const colors = {
        high: '#c8e6c9',
        medium: '#fff9c4',
        low: '#ffccbc'
      };
      addLog(
        `${task.name} (優先度: ${task.priority}) を処理`,
        colors[task.priority]
      );
    },
    complete: () => {
      addLog('すべてのタスクが完了', '#e3f2fd');
    }
  });
```

- 高優先度のタスクは0.5秒後に処理
- 中優先度は1.5秒後、低優先度は3秒後に処理
- タスクの重要度に応じた処理順序を実現

## 🧪 実践コード例2: 外部イベントによる遅延

ユーザーのクリックを待ってから値を発行する例です。

```ts
import { of, fromEvent } from 'rxjs';
import { delayWhen, take, tap } from 'rxjs';

// UI作成
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'delayWhen - クリック待機';
container2.appendChild(title2);

const button = document.createElement('button');
button.textContent = 'クリックして次の値を表示';
button.style.marginBottom = '10px';
container2.appendChild(button);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.minHeight = '100px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

let clickCount = 0;

of('メッセージ1', 'メッセージ2', 'メッセージ3')
  .pipe(
    tap(msg => {
      addLog2(`待機中: ${msg} (ボタンをクリックしてください)`);
      button.textContent = `クリックして「${msg}」を表示`;
    }),
    delayWhen(() => {
      // クリックイベントが発生するまで遅延
      return fromEvent(button, 'click').pipe(take(1));
    })
  )
  .subscribe({
    next: msg => {
      clickCount++;
      addLog2(`✅ 表示: ${msg}`);
      if (clickCount < 3) {
        button.disabled = false;
      } else {
        button.textContent = '完了';
        button.disabled = true;
      }
    },
    complete: () => {
      addLog2('--- すべてのメッセージが表示されました ---');
    }
  });
```

- 各値はユーザーのクリックを待ってから発行される
- 外部イベントをトリガーとした遅延制御が可能
- インタラクティブなシーケンス処理に応用できる

## 🆚 delay との比較

```ts
import { of, timer } from 'rxjs';
import { delay, delayWhen } from 'rxjs';

// delay - 固定時間の遅延
of(1, 2, 3)
  .pipe(delay(1000))
  .subscribe(console.log);
// すべての値が1秒遅延

// delayWhen - 値ごとに異なる遅延
of(1, 2, 3)
  .pipe(
    delayWhen(value => timer(value * 1000))
  )
  .subscribe(console.log);
// 1は1秒後、2は2秒後、3は3秒後
```

| オペレーター | 遅延の制御 | ユースケース |
|:---|:---|:---|
| `delay` | 固定時間 | シンプルな一律遅延 |
| `delayWhen` | 動的（値ごと） | 条件付き遅延、外部イベント待機 |

## ⚠️ 注意点

### 1. 遅延Observableは毎回新しく生成される

```ts
// ❌ 悪い例: 同じObservableインスタンスを使い回し
const delayObs$ = timer(1000);
source$.pipe(
  delayWhen(() => delayObs$)  // 2回目以降動作しない
).subscribe();

// ✅ 良い例: 毎回新しいObservableを生成
source$.pipe(
  delayWhen(() => timer(1000))
).subscribe();
```

### 2. 遅延Observableが完了しない場合

```ts
import { of, NEVER } from 'rxjs';
import { delayWhen } from 'rxjs';

// ❌ 悪い例: NEVERを返すと永遠に遅延
of(1, 2, 3)
  .pipe(
    delayWhen(() => NEVER)  // 値が発行されない
  )
  .subscribe(console.log);
// 何も出力されない
```

遅延Observableは必ず値を発行するか完了する必要があります。

### 3. エラーハンドリング

遅延Observable内でエラーが発生すると、ストリーム全体がエラーになります。

```ts
import { of, throwError, timer, delayWhen } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delayWhen(value => {
      if (value === 2) {
        return throwError(() => new Error('遅延エラー'));
      }
      return timer(1000);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('エラー:', err.message)
  });
// 出力: 1
// エラー: 遅延エラー
```

## 📚 関連オペレーター

- **[delay](./delay)** - 固定時間の遅延
- **[debounceTime](../filtering/debounceTime)** - 入力停止後に遅延
- **[throttleTime](../filtering/throttleTime)** - 一定期間ごとに値を通過
- **[timeout](./timeout)** - タイムアウト制御

## ✅ まとめ

`delayWhen` オペレーターは、各値の遅延タイミングを動的に制御します。

- ✅ 値ごとに異なる遅延を適用可能
- ✅ 外部イベントやObservableによる遅延制御
- ✅ 優先度やタイプに応じた処理タイミングの調整
- ⚠️ 遅延Observableは毎回新しく生成する必要がある
- ⚠️ 遅延Observableは必ず完了するか値を発行する必要がある
- ⚠️ エラーハンドリングに注意
