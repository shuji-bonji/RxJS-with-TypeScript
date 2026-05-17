---
description: throttleTimeオペレーターは、指定時間間隔内で最初の値だけを通過させ、それ以降の値を無視することで高頻度イベントを効率的に間引きます。スクロールやマウス移動などのリアルタイムイベント最適化に最適です。
---

# throttleTime - 最初の値を通し時間で制限

`throttleTime` オペレーターは、最初に発行された値を通過させ、指定した時間間隔内に発行された後続の値を無視します。  
一定時間ごとに最新の値を出すわけではなく、**最初に受け取った値だけを通し、その後の間は無視する**動きです。

スクロールイベントやマウス移動イベントなど、発火頻度の高いストリームを間引きたいときに有効です。
 

## 🔰 基本構文と使い方

```ts
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

fromEvent(document, 'click')
  .pipe(throttleTime(2000))
  .subscribe(() => console.log('クリックされました！'));

```

- 2秒間隔で最初のクリックイベントだけを受け取り、それ以降のクリックは無視します。

[🌐 RxJS公式ドキュメント - `throttleTime`](https://rxjs.dev/api/operators/throttleTime)

> [!WARNING] 本番コードでの注意
> 上記サンプルは説明の簡略化のため `fromEvent` の購読解除を省略しています。実コードでは `takeUntil(destroy$)`、`take(N)`、もしくは `Subscription.unsubscribe()` で明示的にライフサイクル管理してください。詳細: [困難点克服: ライフサイクル管理](/guide/overcoming-difficulties/lifecycle-management.md)
 

## 💡 典型的な活用パターン

- スクロールやマウス移動のイベントハンドリング最適化
- ボタン連打による多重送信防止
- リアルタイムデータストリームの間引き
 

## 🧠 実践コード例（UI付き）

マウスを動かしたとき、位置情報を100ミリ秒ごとに表示します。

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

// 出力エリア作成
const container = document.createElement('div');
container.style.height = '200px';
container.style.border = '1px solid #ccc';
container.style.padding = '10px';
container.textContent = 'マウスをこの領域内で動かしてください';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
document.body.appendChild(positionDisplay);

// マウス移動イベント
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => ({
    x: event.clientX,
    y: event.clientY
  })),
  throttleTime(100)
).subscribe(position => {
  positionDisplay.textContent = `マウス位置: X=${position.x}, Y=${position.y}`;
});
```

- 頻繁に発火するマウス移動イベントを100msごとに制限して、最新の位置だけを表示します。
 
