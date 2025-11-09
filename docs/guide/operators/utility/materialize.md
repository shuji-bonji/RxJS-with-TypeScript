---
description: materializeはObservableの通知（next, error, complete）をNotificationオブジェクトに変換するRxJSユーティリティオペレーターです。エラーをデータとして扱う、通知のデバッグとロギング、メタ情報の記録など、通知そのものを操作したい場面に最適です。dematerializeで元の形式に戻せ、TypeScriptの型推論により型安全な通知処理が可能です。
---

# materialize - 通知のオブジェクト化

`materialize` オペレーターは、Observable の**通知（next, error, complete）を Notification オブジェクトに変換**します。これにより、値だけでなくエラーや完了もデータとして扱えるようになります。

## 🔰 基本構文・動作

通常のストリームをNotificationオブジェクトのストリームに変換します。

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

of(1, 2, 3)
  .pipe(materialize())
  .subscribe(notification => {
    console.log(notification);
  });
// 出力:
// Notification { kind: 'N', value: 1, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 2, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 3, error: undefined, hasValue: true }
// Notification { kind: 'C', value: undefined, error: undefined, hasValue: false }
```

Notificationオブジェクトの `kind` プロパティ:
- `'N'`: next（値の発行）
- `'E'`: error（エラー）
- `'C'`: complete（完了）

[🌐 RxJS公式ドキュメント - materialize](https://rxjs.dev/api/index/function/materialize)

## 💡 典型的な活用例

- **エラーのデータ化**: エラーをストリームの一部として扱う
- **デバッグとロギング**: 通知の詳細な追跡
- **メタ情報の記録**: いつ、どのような通知が発生したか記録
- **エラーを含むストリームの結合**: 複数のストリームのエラーを統一的に処理

## 🧪 実践コード例1: エラーをデータとして扱う

通常はストリームを中断するエラーを、データとして扱い続行する例です。

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, map } from 'rxjs';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'materialize - エラーのデータ化';
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

// 通常のエラーハンドリング（ストリーム中断）
addLog('--- 通常のエラーハンドリング ---', '#e3f2fd');
concat(
  of(1, 2),
  throwError(() => new Error('エラー発生')),
  of(3, 4)  // ここは実行されない
).subscribe({
  next: v => addLog(`値: ${v}`, '#c8e6c9'),
  error: err => addLog(`❌ エラー: ${err.message}`, '#ffcdd2'),
  complete: () => addLog('完了', '#e3f2fd')
});

// materializeを使用（ストリーム継続）
setTimeout(() => {
  addLog('--- materializeを使用 ---', '#e3f2fd');

  concat(
    of(1, 2),
    throwError(() => new Error('エラー発生')),
    of(3, 4)
  )
    .pipe(
      materialize(),
      map(notification => {
        if (notification.kind === 'N') {
          return `値: ${notification.value}`;
        } else if (notification.kind === 'E') {
          return `エラー（データ化）: ${notification.error?.message}`;
        } else {
          return '完了';
        }
      })
    )
    .subscribe({
      next: msg => {
        const color = msg.includes('エラー') ? '#fff9c4' : '#c8e6c9';
        addLog(msg, color);
      },
      complete: () => addLog('ストリーム完了', '#e3f2fd')
    });
}, 1000);
```

- 通常のエラーはストリームを中断
- `materialize`を使うとエラーもデータとして扱い、ストリームが継続

## 🧪 実践コード例2: デバッグ用ロギング

すべての通知を詳細にログ出力する例です。

```ts
import { interval, throwError } from 'rxjs';
import { materialize, take, mergeMap } from 'rxjs';

// UI作成
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'materialize - デバッグロギング';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '250px';
output2.style.overflow = 'auto';
output2.style.fontFamily = 'monospace';
output2.style.fontSize = '12px';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('ja-JP', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.marginBottom = '2px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

interval(500)
  .pipe(
    take(5),
    mergeMap(value => {
      // 値が3の時にエラーを発生させる
      if (value === 3) {
        return throwError(() => new Error('値3でエラー'));
      }
      return of(value);
    }),
    materialize()
  )
  .subscribe({
    next: notification => {
      switch (notification.kind) {
        case 'N':
          addLog2(`[NEXT] value: ${notification.value}`);
          break;
        case 'E':
          addLog2(`[ERROR] ${notification.error?.message}`);
          break;
        case 'C':
          addLog2('[COMPLETE]');
          break;
      }
    },
    complete: () => {
      addLog2('--- Observer完了 ---');
    }
  });
```

- すべての通知タイプ（next, error, complete）を統一的にログ出力
- タイムスタンプ付きで通知の発生順序を追跡
- デバッグやモニタリングに有用

## 🆚 通常のストリームとの比較

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

// 通常のストリーム
of(1, 2, 3).subscribe({
  next: v => console.log('値:', v),
  complete: () => console.log('完了')
});
// 出力:
// 値: 1
// 値: 2
// 値: 3
// 完了

// materializeを使用
of(1, 2, 3)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('通知:', n),
    complete: () => console.log('完了')
  });
// 出力:
// 通知: Notification { kind: 'N', value: 1, ... }
// 通知: Notification { kind: 'N', value: 2, ... }
// 通知: Notification { kind: 'N', value: 3, ... }
// 通知: Notification { kind: 'C', ... }
// 完了
```

## Notificationオブジェクトの操作

```ts
import { of } from 'rxjs';
import { materialize, map } from 'rxjs';

of(10, 20, 30)
  .pipe(
    materialize(),
    map(notification => {
      // Notificationオブジェクトのプロパティ
      return {
        kind: notification.kind,           // 'N', 'E', 'C'
        hasValue: notification.hasValue,   // 値を持つか
        value: notification.value,         // 値（nextの場合）
        error: notification.error          // エラー（errorの場合）
      };
    })
  )
  .subscribe(console.log);
// 出力:
// { kind: 'N', hasValue: true, value: 10, error: undefined }
// { kind: 'N', hasValue: true, value: 20, error: undefined }
// { kind: 'N', hasValue: true, value: 30, error: undefined }
// { kind: 'C', hasValue: false, value: undefined, error: undefined }
```

## ⚠️ 注意点

### 1. エラーはストリームを中断しない

`materialize` を使用すると、エラーもデータとして扱われ、ストリームは中断されません。

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize } from 'rxjs';

concat(
  of(1),
  throwError(() => new Error('エラー')),
  of(2)
)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('通知:', n.kind),
    error: () => console.log('エラーハンドラ'),  // 呼ばれない
    complete: () => console.log('完了')
  });
// 出力:
// 通知: N
// 通知: E  ← エラーもnextとして扱われる
// 完了
```

### 2. dematerialize との組み合わせ

`materialize` で変換したストリームは、`dematerialize` で元に戻せます。

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),
    // ここで何らかの処理
    dematerialize()  // 元に戻す
  )
  .subscribe(console.log);
// 出力: 1, 2, 3
```

### 3. パフォーマンスへの影響

Notificationオブジェクトの生成にはオーバーヘッドがあります。本番環境では必要な場合のみ使用してください。

## 📚 関連オペレーター

- **[dematerialize](./dematerialize)** - Notificationオブジェクトを通常の通知に戻す
- **[tap](./tap)** - 副作用の実行（デバッグ用途）
- **[catchError](../../error-handling/retry-catch)** - エラーハンドリング

## ✅ まとめ

`materialize` オペレーターは、通知をNotificationオブジェクトに変換します。

- ✅ エラーをデータとして扱える
- ✅ デバッグとロギングに有用
- ✅ 通知のメタ情報を記録できる
- ✅ `dematerialize` で元に戻せる
- ⚠️ エラーはストリームを中断しなくなる
- ⚠️ パフォーマンスオーバーヘッドに注意
