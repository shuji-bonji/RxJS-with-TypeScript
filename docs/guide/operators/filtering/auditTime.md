---
description: auditTimeオペレーターは、値が発行されたら指定時間待機し、その期間内の最後の値を出力します。throttleTimeと似ていますが、最初ではなく最後の値を出力する点が異なります。
---

# auditTime - 指定時間後に最後の値を発行する

`auditTime` オペレーターは、値が発行されたら**指定時間待機**し、その期間内の**最後の値**を出力します。その後、次の値が来るまで待機します。


## 🔰 基本構文と使い方

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('クリック！'));
```

**動作の流れ**:
1. 最初のクリックが発生
2. 1秒間待機（この間のクリックは記録されるが出力されない）
3. 1秒後に最後のクリックを出力
4. 次のクリックを待つ

[🌐 RxJS公式ドキュメント - `auditTime`](https://rxjs.dev/api/operators/auditTime)


## 🆚 throttleTime との対比

`throttleTime` と `auditTime` は似ていますが、出力する値が異なります。

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: 最初の値を出力
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// 出力: 0, 4, 8（各期間の最初の値）

// auditTime: 最後の値を出力
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// 出力: 3, 6, 9（各期間の最後の値）
```

**タイムライン比較**:
```
ソース:     0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (最初)   (最初)   (最初)

audit:      -------3--------6--------9----|
                  (最後)   (最後)   (最後)
```

| オペレーター | 出力する値 | 出力タイミング | ユースケース |
|---|---|---|---|
| `throttleTime(ms)` | 期間内の**最初**の値 | 値受信時 | 即座の反応が必要 |
| `auditTime(ms)` | 期間内の**最後**の値 | 期間終了時 | 最新の状態が必要 |
| `debounceTime(ms)` | 静寂後の**最後**の値 | 入力停止後 | 入力完了を待つ |


## 💡 典型的な活用パターン

1. **ウィンドウリサイズの最適化**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // 200ms間隔で最新のサイズを取得
   ).subscribe(() => {
     console.log(`ウィンドウサイズ: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **スクロール位置の追跡**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map } from 'rxjs';

   fromEvent(window, 'scroll').pipe(
     auditTime(100),
     map(() => ({
       scrollY: window.scrollY,
       scrollX: window.scrollX
     }))
   ).subscribe(position => {
     console.log(`スクロール位置: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```

3. **スムーズなドラッグ移動**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map, takeUntil, switchMap } from 'rxjs';

   // ドラッグ可能な要素を作成
   const box = document.createElement('div');
   box.style.width = '100px';
   box.style.height = '100px';
   box.style.backgroundColor = '#3498db';
   box.style.position = 'absolute';
   box.style.cursor = 'move';
   box.style.left = '100px';
   box.style.top = '100px';
   box.textContent = 'ドラッグ';
   box.style.display = 'flex';
   box.style.alignItems = 'center';
   box.style.justifyContent = 'center';
   box.style.color = 'white';
   document.body.appendChild(box);

   const mouseDown$ = fromEvent<MouseEvent>(box, 'mousedown');
   const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
   const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

   // ドラッグ操作の実装
   mouseDown$.pipe(
     switchMap(startEvent => {
       const startX = startEvent.clientX - box.offsetLeft;
       const startY = startEvent.clientY - box.offsetTop;

       return mouseMove$.pipe(
         auditTime(16), // 約60FPS（16ms）で位置更新
         map(moveEvent => ({
           x: moveEvent.clientX - startX,
           y: moveEvent.clientY - startY
         })),
         takeUntil(mouseUp$)
       );
     })
   ).subscribe(position => {
     box.style.left = `${position.x}px`;
     box.style.top = `${position.y}px`;
   });
   ```


## 🧠 実践コード例（マウス追跡）

マウスの動きを追跡し、一定間隔で最新の位置を表示する例です。

```ts
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// UI要素の作成
const container = document.createElement('div');
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relative';
container.textContent = 'マウスをこの領域内で動かしてください';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
positionDisplay.style.fontFamily = 'monospace';
document.body.appendChild(positionDisplay);

const dot = document.createElement('div');
dot.style.width = '10px';
dot.style.height = '10px';
dot.style.borderRadius = '50%';
dot.style.backgroundColor = '#e74c3c';
dot.style.position = 'absolute';
dot.style.display = 'none';
container.appendChild(dot);

// マウス移動イベント
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // 100msごとに最新の位置を取得
).subscribe(position => {
  positionDisplay.textContent = `最新位置（100ms間隔）: X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // ドットを最新位置に移動
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = 'block';
});
```

このコードは、マウスが頻繁に動いても、100msごとに最新の位置だけを取得して表示します。


## 🎯 debounceTime との違い

`auditTime` と `debounceTime` は**どちらも最後の値を出力**しますが、**タイミングが全く異なります**。

### 決定的な違い

| オペレーター | 動作 | 使い分け |
|---|---|---|
| `auditTime(ms)` | 値が来たら**ms後に必ず出力**（入力が続いていても） | 定期的にサンプリングしたい |
| `debounceTime(ms)` | **入力が止まってから**ms後に出力 | 入力完了を待ちたい |

### 具体例：検索入力での違い

```ts
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = '検索ワード入力';
document.body.appendChild(input);

// auditTime: 入力中でも300msごとに検索実行
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → 検索:', input.value);
});

// debounceTime: 入力停止後300ms待ってから検索実行
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → 検索:', input.value);
});
```

### タイムラインで見る違い

ユーザーが「ab」→「abc」→「abcd」と素早く入力した場合:

```
入力イベント:   a--b--c--d------------|
              ↓
auditTime:    ------c-----d----------|
            (300ms後) (300ms後)
            → 「abc」で検索、「abcd」で検索（計2回）

debounceTime: --------------------d-|
                              (停止後300ms)
            → 「abcd」で検索（計1回のみ）
```

**わかりやすい覚え方**:
- **`auditTime`**: 「定期的に監査（audit）する」→ 一定間隔で必ず確認
- **`debounceTime`**: 「落ち着く（debounce）まで待つ」→ 静かになるまで待つ

### 実用的な使い分け

```ts
// ✅ auditTime が適切な場合
// - スクロール位置の追跡（ずっとスクロールしていても定期的に取得したい）
fromEvent(window, 'scroll').pipe(
  auditTime(100)  // 100msごとに最新位置を取得
).subscribe(/* ... */);

// ✅ debounceTime が適切な場合
// - 検索ボックス（入力が完了してから検索したい）
fromEvent(searchInput, 'input').pipe(
  debounceTime(300)  // 入力停止後300ms待つ
).subscribe(/* ... */);
```


## 📋 型安全な使い方

TypeScript でジェネリクスを活用した型安全な実装例です。

```ts
import { Observable, fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

interface MousePosition {
  x: number;
  y: number;
  timestamp: number;
}

function trackMousePosition(
  element: HTMLElement,
  intervalMs: number
): Observable<MousePosition> {
  return fromEvent<MouseEvent>(element, 'mousemove').pipe(
    auditTime(intervalMs),
    map(event => ({
      x: event.clientX,
      y: event.clientY,
      timestamp: Date.now()
    } as MousePosition))
  );
}

// 使用例
const canvas = document.createElement('div');
canvas.style.width = '400px';
canvas.style.height = '300px';
canvas.style.border = '1px solid black';
document.body.appendChild(canvas);

trackMousePosition(canvas, 200).subscribe(position => {
  console.log(`位置: (${position.x}, ${position.y}) at ${position.timestamp}`);
});
```


## 🔄 auditTime と throttleTime の組み合わせ

特定のシナリオでは両方を組み合わせることもできます。

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(100).pipe(take(50));

// throttleTime → auditTime の順序
source$.pipe(
  throttleTime(1000),  // 1秒ごとに最初の値を通す
  auditTime(500)       // その後500ms待って最後の値を出力
).subscribe(console.log);
```


## ⚠️ よくある間違い

> [!WARNING]
> `auditTime` と `debounceTime` は動作が異なります。検索入力などユーザーが入力を**停止するのを待つ**場合は `debounceTime` を使用してください。`auditTime` は入力中も一定間隔で値を発行します。

### 誤: auditTime と debounceTime を混同

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

// 検索入力欄を作成
const input = document.createElement('input');
input.type = 'text';
input.placeholder = '検索...';
document.body.appendChild(input);

// ❌ 悪い例: 検索入力に auditTime を使用
fromEvent(input, 'input').pipe(
  auditTime(300) // 入力中も300msごとに検索が実行される
).subscribe(() => {
  console.log('検索実行');
});
```

### 正: debounceTime を使用

```ts
import { fromEvent } from 'rxjs';
import { debounceTime } from 'rxjs';

// 検索入力欄を作成
const input = document.createElement('input');
input.type = 'text';
input.placeholder = '検索...';
document.body.appendChild(input);

// ✅ 良い例: 検索入力に debounceTime を使用
fromEvent(input, 'input').pipe(
  debounceTime(300) // 入力停止後300ms待ってから検索
).subscribe(() => {
  console.log('検索実行', input.value);
});
```


## 🎓 まとめ

### auditTime を使うべき場合
- ✅ 一定間隔で最新の値が必要な場合
- ✅ スクロール、リサイズ、マウス移動などの高頻度イベント
- ✅ 定期的なサンプリングが必要な場合
- ✅ 最新の状態を反映したい場合

### throttleTime を使うべき場合
- ✅ 即座の反応が必要な場合
- ✅ 最初の値で処理を開始したい場合
- ✅ ボタン連打防止

### debounceTime を使うべき場合
- ✅ 入力完了を待ちたい場合
- ✅ 検索、オートコンプリート
- ✅ ユーザーが入力を止めるまで待つ

### 注意点
- ⚠️ `auditTime` は期間内の最後の値のみを出力（中間の値は破棄される）
- ⚠️ 短い間隔で設定すると、あまり効果がない
- ⚠️ 用途に応じて `throttleTime` か `debounceTime` の方が適切な場合がある


## 🚀 次のステップ

- **[throttleTime](./throttleTime)** - 最初の値を通す方法を学ぶ
- **[debounceTime](./debounceTime)** - 入力停止後に値を発行する方法を学ぶ
- **[filter](./filter)** - 条件に基づいてフィルタリングする方法を学ぶ
- **[フィルタリングオペレーター実践例](./practical-use-cases)** - 実際のユースケースを学ぶ
