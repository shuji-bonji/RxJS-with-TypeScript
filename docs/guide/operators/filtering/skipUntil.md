---
description: skipUntilオペレーターは、別のObservableが値を発行するまで元のObservableの値をすべてスキップし、発行後は通常通り値を出力します。時間ベースの遅延開始や、特定イベント発生後の処理に便利です。
---

# skipUntil - 発火までスキップ

`skipUntil` オペレーターは、**指定した Observable（通知トリガー）が最初の値を発行するまで、元の Observable からの値をすべてスキップ**します。通知トリガーが発行されたタイミング以降は、通常通り値を出力します。


## 🔰 基本構文と使い方

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // 0.5秒ごとに値を発行
const notifier$ = timer(2000); // 2秒後に値を発行

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// 出力: 4, 5, 6, 7, 8, ...
// （最初の2秒間の値 0, 1, 2, 3 はスキップされる）
```

**動作の流れ**:
1. `source$` が 0, 1, 2, 3 を発行 → すべてスキップ
2. 2秒後に `notifier$` が値を発行
3. 以降の `source$` の値（4, 5, 6, ...）は通常通り出力

[🌐 RxJS公式ドキュメント - `skipUntil`](https://rxjs.dev/api/operators/skipUntil)


## 🆚 takeUntil との対比

`skipUntil` と `takeUntil` は対照的な動作をします。

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500); // 0.5秒ごとに値を発行
const notifier$ = timer(2000); // 2秒後に値を発行

// takeUntil: 通知が来るまで値を取得
source$.pipe(
  takeUntil(notifier$)
).subscribe(console.log);
// 出力: 0, 1, 2, 3（2秒後に停止）

// skipUntil: 通知が来るまで値をスキップ
source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// 出力: 4, 5, 6, 7, ...（2秒後から開始）
```

| オペレーター | 動作 | 完了タイミング |
|---|---|---|
| `takeUntil(notifier$)` | 通知が来るまで値を**取得** | 通知時に自動完了 |
| `skipUntil(notifier$)` | 通知が来るまで値を**スキップ** | 元のストリームの完了時 |


## 💡 典型的な活用パターン

1. **ユーザー認証後にデータを処理開始**
   ```ts
   import { interval, Subject } from 'rxjs';
   import { skipUntil } from 'rxjs';

   const authenticated$ = new Subject<void>();
   const dataStream$ = interval(1000);

   // 認証完了までデータをスキップ
   dataStream$.pipe(
     skipUntil(authenticated$)
   ).subscribe(data => {
     console.log(`データ処理: ${data}`);
   });

   // 3秒後に認証完了
   setTimeout(() => {
     console.log('認証完了！');
     authenticated$.next();
   }, 3000);
   // 3秒後から「データ処理: 3」「データ処理: 4」...と出力
   ```

2. **初期ロード完了後にイベント処理を開始**
   ```ts
   import { fromEvent, BehaviorSubject } from 'rxjs';
   import { filter, skipUntil } from 'rxjs';

   const appReady$ = new BehaviorSubject<boolean>(false);
   const button = document.createElement('button');
   button.textContent = 'クリック';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');

   // アプリの準備が完了するまでクリックを無視
   clicks$.pipe(
     skipUntil(appReady$.pipe(filter(ready => ready)))
   ).subscribe(() => {
     console.log('クリックが処理されました');
   });

   // 2秒後にアプリが準備完了
   setTimeout(() => {
     console.log('アプリ準備完了');
     appReady$.next(true);
   }, 2000);
   ```

3. **タイマーベースの遅延開始**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { skipUntil, scan } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'カウント';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');
   const startTime$ = timer(3000); // 3秒後

   // 3秒経過するまでクリックをカウントしない
   clicks$.pipe(
     skipUntil(startTime$),
     scan(count => count + 1, 0)
   ).subscribe(count => {
     console.log(`カウント: ${count}`);
   });

   console.log('3秒後からカウント開始...');
   ```


## 🧠 実践コード例（ゲームのカウントダウン）

ゲーム開始前のカウントダウン中はクリックを無視し、カウントダウン終了後にクリックを有効にする例です。

```ts
import { fromEvent, timer, interval } from 'rxjs';
import { skipUntil, take, scan } from 'rxjs';

// UI要素の作成
const container = document.createElement('div');
document.body.appendChild(container);

const countdown = document.createElement('div');
countdown.style.fontSize = '24px';
countdown.style.marginBottom = '10px';
countdown.textContent = 'カウントダウン中...';
container.appendChild(countdown);

const button = document.createElement('button');
button.textContent = 'クリック！';
button.disabled = true;
container.appendChild(button);

const scoreDisplay = document.createElement('div');
scoreDisplay.style.marginTop = '10px';
scoreDisplay.textContent = 'スコア: 0';
container.appendChild(scoreDisplay);

// カウントダウン（3秒）
const countdownTimer$ = interval(1000).pipe(take(3));
countdownTimer$.subscribe({
  next: (n) => {
    countdown.textContent = `開始まで ${3 - n} 秒...`;
  },
  complete: () => {
    countdown.textContent = 'ゲーム開始！';
    button.disabled = false;
  }
});

// ゲーム開始の通知
const gameStart$ = timer(3000);

// クリックイベント（ゲーム開始までスキップ）
const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  skipUntil(gameStart$),
  scan(score => score + 10, 0)
).subscribe(score => {
  scoreDisplay.textContent = `スコア: ${score}`;
});
```

このコードでは、カウントダウンの3秒間はクリックが無視され、カウントダウン終了後のクリックのみスコアに反映されます。


## 🎯 skip と skipUntil の違い

```ts
import { interval, timer } from 'rxjs';
import { skip, skipUntil } from 'rxjs';

const source$ = interval(500);

// skip: 最初のN個を数でスキップ
source$.pipe(
  skip(3)
).subscribe(console.log);
// 出力: 3, 4, 5, 6, ...

// skipUntil: 別のObservableが発火するまでスキップ
source$.pipe(
  skipUntil(timer(1500))
).subscribe(console.log);
// 出力: 3, 4, 5, 6, ...（結果は同じだが、制御方法が異なる）
```

| オペレーター | スキップ条件 | ユースケース |
|---|---|---|
| `skip(n)` | 最初のn個を数でスキップ | 固定数のスキップ |
| `skipWhile(predicate)` | 条件を満たす間スキップ | 条件ベースのスキップ |
| `skipUntil(notifier$)` | 別のObservableが発火するまでスキップ | イベント/時間ベースのスキップ |


## 📋 型安全な使い方

TypeScript でジェネリクスを活用した型安全な実装例です。

```ts
import { Observable, Subject, fromEvent } from 'rxjs';
import { skipUntil, map } from 'rxjs';

interface GameState {
  status: 'waiting' | 'ready' | 'playing' | 'finished';
}

interface ClickEvent {
  timestamp: number;
  x: number;
  y: number;
}

class Game {
  private gameReady$ = new Subject<void>();
  private state: GameState = { status: 'waiting' };

  startGame(element: HTMLElement): Observable<ClickEvent> {
    const clicks$ = fromEvent<MouseEvent>(element, 'click').pipe(
      map(event => ({
        timestamp: Date.now(),
        x: event.clientX,
        y: event.clientY
      } as ClickEvent)),
      skipUntil(this.gameReady$)
    );

    // 準備完了を通知
    setTimeout(() => {
      this.state = { status: 'ready' };
      this.gameReady$.next();
      console.log('ゲーム準備完了！');
    }, 2000);

    return clicks$;
  }
}

// 使用例
const game = new Game();
const canvas = document.createElement('div');
canvas.style.width = '300px';
canvas.style.height = '200px';
canvas.style.border = '1px solid black';
canvas.textContent = 'クリックしてください';
document.body.appendChild(canvas);

game.startGame(canvas).subscribe(click => {
  console.log(`クリック位置: (${click.x}, ${click.y})`);
});
```


## 🔄 skipUntil と takeUntil の組み合わせ

特定の期間だけ値を取得したい場合、両方を組み合わせます。

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500);
const start$ = timer(2000); // 2秒後に開始
const stop$ = timer(5000);  // 5秒後に停止

source$.pipe(
  skipUntil(start$), // 2秒後まではスキップ
  takeUntil(stop$)   // 5秒後に停止
).subscribe({
  next: console.log,
  complete: () => console.log('完了')
});
// 出力: 4, 5, 6, 7, 8, 9, 完了
// （2秒〜5秒の間の値のみ取得）
```

**タイムライン**:
```
0s    1s    2s    3s    4s    5s
|-----|-----|-----|-----|-----|
0  1  2  3  4  5  6  7  8  9  10
      ↑           ↑
   skip開始    take終了
   （4から）  （9まで）
```


## ⚠️ よくある間違い

> [!IMPORTANT]
> `skipUntil` は通知 Observable の**最初の発火のみ**が有効です。2回目以降の発火は無視されます。

### 誤: 通知Observableが複数回発火する

```ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject<void>();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ❌ 悪い例: 複数回nextを呼んでも効果は最初の1回のみ
setTimeout(() => notifier$.next(), 1000);
setTimeout(() => notifier$.next(), 2000); // これは無意味
```

### 正: 最初の発火のみが有効であることを理解する

```ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject<void>();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ✅ 良い例: 1回だけnextを呼ぶ
setTimeout(() => {
  console.log('スキップ終了');
  notifier$.next();
  notifier$.complete(); // 完了させるのがベストプラクティス
}, 1000);
```


## 🎓 まとめ

### skipUntil を使うべき場合
- ✅ 特定のイベント発生後に処理を開始したい場合
- ✅ 初期化完了後にユーザー操作を有効にしたい場合
- ✅ 時間ベースの遅延開始が必要な場合
- ✅ 認証完了後にデータ処理を開始したい場合

### takeUntil との組み合わせ
- ✅ 特定の期間だけ値を取得したい場合（skipUntil + takeUntil）

### 注意点
- ⚠️ 通知Observableの最初の発火のみが有効
- ⚠️ 通知Observableが発火しない場合、すべての値がスキップされ続ける
- ⚠️ 元のストリームが完了するまでsubscriptionは維持される


## 🚀 次のステップ

- **[skip](./skip)** - 最初のN個の値をスキップする方法を学ぶ
- **[take](./take)** - 最初のN個の値を取得する方法を学ぶ
- **[takeUntil](../utility/takeUntil)** - 別のObservableが発火するまで値を取得する方法を学ぶ
- **[filter](./filter)** - 条件に基づいてフィルタリングする方法を学ぶ
- **[フィルタリングオペレーター実践例](./practical-use-cases)** - 実際のユースケースを学ぶ
