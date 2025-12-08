---
description: "delayオペレーターはObservable内の各値の発行タイミングを指定時間だけ遅らせます。UI演出、レート制限、非同期処理の制御、テスト時の遅延シミュレーションに効果的です。delayWhenとの違い、TypeScriptでの型安全な実装を実践的なコード例で解説します。"
---

# delay - 値の遅延

`delay` オペレーターは、ストリーム内の各値の発行を指定した時間だけ遅延させるために使用します。  
アニメーションの演出や、ユーザーへのフィードバック表示のタイミング調整などに役立ちます。


## 🔰 基本構文・動作

一定時間後に値を発行する最小構成です。

```ts
import { of } from 'rxjs';
import { delay } from 'rxjs';

of('Hello')
  .pipe(
    delay(1000) // 1秒後に値を発行
  )
  .subscribe(console.log);
// 出力:
// Hello
```

この例では、`of('Hello')` によって作られた値が 1秒遅れて `subscribe()` で受け取られます。

[🌐 RxJS公式ドキュメント - delay](https://rxjs.dev/api/index/function/delay)

## 💡 典型的な活用例

複数の値を発行する場面で、delayを用いて発行タイミングを調整する例です。

```ts
import { of } from 'rxjs';
import { delay, concatMap } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    concatMap(
      (val, index) => of(val).pipe(delay(1000 * index)) // Aは即時、Bは1秒後、Cは2秒後
    )
  )
  .subscribe(console.log);
// 出力:
// A
// B
```

このように、`concatMap` と組み合わせることで、値ごとに個別の遅延を設けることも可能です。


## 🧪 実践コード例（UI付き）

```ts
import { of } from 'rxjs';
import { delay, tap } from 'rxjs';

// 出力表示エリア
const delayOutput = document.createElement('div');
delayOutput.innerHTML = '<h3>delay の例:</h3>';
document.body.appendChild(delayOutput);

// 現在時刻を表示する関数
function addTimeLog(message: string) {
  const now = new Date();
  const time =
    now.toLocaleTimeString('ja-JP', { hour12: false }) +
    '.' +
    now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `${time}: ${message}`;
  delayOutput.appendChild(logItem);
}

// 開始時刻を記録
addTimeLog('開始');

// 値のシーケンス
of('A', 'B', 'C')
  .pipe(
    tap((val) => addTimeLog(`値 ${val} が発行される前`)),
    delay(1000), // 1秒遅延
    tap((val) => addTimeLog(`値 ${val} が1秒後に発行された`))
  )
  .subscribe();
```


## ✅ まとめ

- `delay`は**Observableの出力タイミングを制御**するための演算子
- 一定の遅延をかけたり、`concatMap` と組み合わせて**値ごとの遅延制御**ができる
- UIへの出力やタイマー演出など、**UX改善のための非同期調整**に便利
