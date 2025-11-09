---
description: dematerializeはNotificationオブジェクトを通常の通知（next, error, complete）に戻し、materializeの逆変換を行うRxJSユーティリティオペレーターです。通知の加工後の復元、エラーのフィルタリングや変換、通知の順序変更やバッファリングなど、通知をデータとして処理した後に元の形式に戻したい場面に最適です。
---

# dematerialize - 通知オブジェクトの復元

`dematerialize` オペレーターは、**Notification オブジェクトを通常の通知（next, error, complete）に変換**します。`materialize` の逆変換を行い、データ化された通知を元の形式に戻します。

## 🔰 基本構文・動作

Notificationオブジェクトのストリームを通常のストリームに戻します。

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),     // Notificationオブジェクトに変換
    dematerialize()    // 元に戻す
  )
  .subscribe({
    next: v => console.log('値:', v),
    complete: () => console.log('完了')
  });
// 出力:
// 値: 1
// 値: 2
// 値: 3
// 完了
```

[🌐 RxJS公式ドキュメント - dematerialize](https://rxjs.dev/api/index/function/dematerialize)

## 💡 典型的な活用例

- **通知の加工後に復元**: materializeで処理した後、元の形式に戻す
- **エラーのフィルタリング**: 特定のエラーだけを除外
- **通知の順序変更**: 通知をデータとして並べ替えた後に復元
- **デバッグ後の復元**: ログ出力などの処理後に通常動作に戻す

## 🧪 実践コード例1: エラーの選択的フィルタリング

特定のエラーだけを除外し、それ以外は通常通り処理する例です。

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize, filter } from 'rxjs';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'dematerialize - エラーフィルタリング';
container.appendChild(title);

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

// エラーを含むストリーム
const source$ = concat(
  of(1, 2),
  throwError(() => new Error('無視すべきエラー')),
  of(3, 4),
  throwError(() => new Error('重大なエラー')),
  of(5)
);

source$
  .pipe(
    materialize(),
    filter(notification => {
      // "無視すべきエラー"だけをフィルタリング
      if (notification.kind === 'E') {
        const errorMessage = notification.error?.message || '';
        if (errorMessage.includes('無視すべき')) {
          addLog(`🔇 無視: ${errorMessage}`, '#fff9c4');
          return false;  // このエラーは除外
        }
      }
      return true;
    }),
    dematerialize()  // 元の形式に戻す
  )
  .subscribe({
    next: v => addLog(`✅ 値: ${v}`, '#c8e6c9'),
    error: err => addLog(`❌ エラー: ${err.message}`, '#ffcdd2'),
    complete: () => addLog('完了', '#e3f2fd')
  });
```

- "無視すべきエラー"は除外され、ストリームは継続
- "重大なエラー"は通常通りエラーハンドラに渡される
- エラーの選択的処理が可能

## 🧪 実践コード例2: 通知の遅延処理

通知を一時的にバッファリングしてから復元する例です。

```ts
import { from, interval, take, delay } from 'rxjs';
import { materialize, dematerialize, bufferTime, concatMap } from 'rxjs';

// UI作成
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'dematerialize - バッファリングと遅延';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('ja-JP', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('開始 - 1秒ごとに値を発行、2秒ごとにまとめて処理');

interval(1000)
  .pipe(
    take(6),
    materialize(),
    bufferTime(2000),      // 2秒ごとにバッファ
    concatMap(notifications => {
      addLog2(`--- ${notifications.length}個の通知をバッファから処理 ---`);
      return from(notifications).pipe(
        delay(500),        // 各通知を0.5秒遅延
        dematerialize()    // 元の形式に戻す
      );
    })
  )
  .subscribe({
    next: v => addLog2(`値: ${v}`),
    complete: () => addLog2('完了')
  });
```

- 通知を2秒ごとにバッファリング
- バッファから取り出して遅延処理
- `dematerialize`で元のストリームとして復元

## 🆚 materialize との関係

```ts
import { of } from 'rxjs';
import { materialize, dematerialize, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),           // Notificationに変換
    map(notification => {
      // Notificationオブジェクトとして処理
      console.log('kind:', notification.kind);
      return notification;
    }),
    dematerialize()          // 元に戻す
  )
  .subscribe(v => console.log('値:', v));
// 出力:
// kind: N
// 値: 1
// kind: N
// 値: 2
// kind: N
// 値: 3
// kind: C
```

| 処理の流れ | 説明 |
|:---|:---|
| 元のストリーム | 通常の値（next）、エラー（error）、完了（complete） |
| ↓ `materialize()` | Notificationオブジェクトのストリーム |
| 中間処理 | Notificationとして加工・フィルタリング |
| ↓ `dematerialize()` | 通常のストリームに復元 |
| 最終的なストリーム | 通常の値、エラー、完了 |

## ⚠️ 注意点

### 1. エラー通知は実際のエラーに変換される

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// 各Observableをmaterialize()で通知オブジェクトに変換
concat(
  of(1).pipe(materialize()),
  throwError(() => new Error('エラー')).pipe(materialize()),
  of(2).pipe(materialize())  // エラー後なので実行されない
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('値:', v),
    error: err => console.log('エラー:', err.message)
  });
// 出力:
// 値: 1
// エラー: エラー
```

エラー通知に到達すると、ストリームはエラーで中断されます。

### 2. 完了通知でストリームが完了する

```ts
import { of, EMPTY, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// 各Observableをmaterialize()で通知オブジェクトに変換
concat(
  of(1).pipe(materialize()),
  of(2).pipe(materialize()),
  EMPTY.pipe(materialize()),  // 完了通知
  of(3).pipe(materialize())   // 完了後なので実行されない
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('値:', v),
    complete: () => console.log('完了')
  });
// 出力:
// 値: 1
// 値: 2
// 完了
```

完了通知以降の値は発行されません。

### 3. 不正なNotificationオブジェクト

`dematerialize`は正しいNotificationオブジェクトを期待します。

```ts
import { of } from 'rxjs';
import { dematerialize } from 'rxjs';

// ❌ 通常の値をdematerializeに渡すとエラー
of(1, 2, 3)
  .pipe(
    dematerialize()  // Notificationオブジェクトではない
  )
  .subscribe({
    next: console.log,
    error: err => console.error('エラー:', err.message)
  });
// エラーが発生
```

## 実践的な組み合わせ例

```ts
import { interval, throwError, of, concat } from 'rxjs';
import { materialize, dematerialize, take, mergeMap, map } from 'rxjs';

// エラーをwarningに変換する例
interval(500)
  .pipe(
    take(10),
    mergeMap(value => {
      // 5の時だけエラーを発生
      if (value === 5) {
        return throwError(() => new Error(`値${value}でエラー`));
      }
      return of(value);
    }),
    materialize(),
    map(notification => {
      // エラーをwarningメッセージに変換
      if (notification.kind === 'E') {
        console.warn('Warning:', notification.error?.message);
        // エラーの代わりに特別な値を発行（materialize()で生成）
        return { kind: 'N' as const, value: -1 };
      }
      return notification;
    }),
    dematerialize()
  )
  .subscribe({
    next: v => console.log('値:', v),
    error: err => console.error('エラー:', err),  // 呼ばれない
    complete: () => console.log('完了')
  });
// 出力:
// 値: 0, 1, 2, 3, 4
// Warning: 値5でエラー
// 値: -1  (エラーの代わり)
// 値: 6, 7, 8, 9
// 完了
```

## 📚 関連オペレーター

- **[materialize](./materialize)** - 通知をNotificationオブジェクトに変換
- **[catchError](../../error-handling/retry-catch)** - エラーハンドリング
- **[retry](./retry)** - エラー時の再試行

## ✅ まとめ

`dematerialize` オペレーターは、Notificationオブジェクトを通常の通知に戻します。

- ✅ `materialize`の逆変換
- ✅ 通知を加工した後に元の形式に復元
- ✅ エラーのフィルタリングや変換が可能
- ✅ 通知の順序変更やバッファリングに活用
- ⚠️ エラー通知は実際のエラーとして動作
- ⚠️ 完了通知でストリームが完了
- ⚠️ 正しいNotificationオブジェクトが必要
