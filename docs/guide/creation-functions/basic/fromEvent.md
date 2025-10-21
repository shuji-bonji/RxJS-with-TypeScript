---
description: fromEvent() - DOMイベントやEventEmitterをObservableに変換するCreation Function。イベント駆動型プログラミングをストリーム化します。
---

# fromEvent() - イベントをObservableに変換

`fromEvent()`は、DOMイベントやNode.js EventEmitterなどのイベントソースを、Observableストリームに変換するCreation Functionです。

## 概要

`fromEvent()`は、イベントベースの非同期処理をRxJSのパイプラインで扱えるようにします。購読時に自動的にイベントリスナーを登録し、購読解除時には自動的にリスナーを削除するため、メモリリークのリスクを大幅に軽減できます。

**シグネチャ**:
```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

**公式ドキュメント**: [📘 RxJS公式: fromEvent()](https://rxjs.dev/api/index/function/fromEvent)

## 基本的な使い方

DOMイベントをObservableとして扱う最もシンプルな例です。

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('ボタンがクリックされました:', event);
});

// クリックするたびにイベントが発行される
```

## 重要な特徴

### 1. 自動的なリスナー登録・解除

`fromEvent()`は、購読時にイベントリスナーを登録し、購読解除時に自動的にリスナーを削除します。

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent<MouseEvent>(document, 'click');

const subscription = clicks$.subscribe(event => {
  console.log('クリック位置:', event.clientX, event.clientY);
});

// 5秒後に購読解除（イベントリスナーも自動削除）
setTimeout(() => {
  subscription.unsubscribe();
  console.log('購読解除しました');
}, 5000);
```

> [!IMPORTANT]
> **メモリリーク防止**
>
> `unsubscribe()`を呼ぶと、内部で`removeEventListener()`が自動的に実行されます。これにより、手動でリスナーを削除する必要がなく、メモリリークのリスクが大幅に軽減されます。

### 2. Cold Observable（各購読が独立したリスナーを登録）

`fromEvent()`によって作成されるObservableは**Cold Observable**です。購読するたびに、独立したイベントリスナーが登録されます。

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// 購読1 - リスナーAを登録
clicks$.subscribe(() => console.log('Observer 1: クリック'));

// 1秒後に購読2を追加 - リスナーBを独立して登録
setTimeout(() => {
  clicks$.subscribe(() => console.log('Observer 2: クリック'));
}, 1000);

// 1回のクリックで両方のリスナーが発火
// これは各購読が独立したリスナーを持つ証拠
```

> [!NOTE]
> **Cold Observableの証明**
>
> 購読するたびに新しいイベントリスナーが登録され、購読解除時に削除されます。これはCold Observableの特徴です。ただし、イベントソース（DOM要素など）は外部にあり共有されるため、「購読前のイベントは受け取れない」というHot的な性質も持ちます。

### 3. TypeScriptの型サポート

イベントの型を明示的に指定できます。

```typescript
import { fromEvent } from 'rxjs';

const input = document.createElement('input');
input.type = 'text';
document.body.appendChild(input);
const input$ = fromEvent<InputEvent>(input, 'input');

input$.subscribe(event => {
  // eventの型はInputEvent
  const target = event.target as HTMLInputElement;
  console.log('入力値:', target.value);
});
```

### 4. Cold Observable

`fromEvent()`は**Cold Observable**です。購読するたびに独立した実行が開始されます。

```typescript
import { fromEvent } from 'rxjs';

const button = document.createElement('button');
button.innerText = "購読";
document.body.appendChild(button);

const clicks$ = fromEvent(document, 'click');

// 1回目の購読 - イベントリスナーが追加される
clicks$.subscribe(() => console.log('購読者A'));

// 2回目の購読 - 別のイベントリスナーが追加される
clicks$.subscribe(() => console.log('購読者B'));

// 1回クリックすると両方のリスナーが発火
// 出力:
// 購読者A
// 購読者B
```

> [!NOTE]
> **Cold Observableの特徴**
> - 購読するたびに独立した実行が開始されます
> - 各購読者は独自のデータストリームを受け取ります
> - 購読ごとに独立したイベントリスナーが登録されます。unsubscribeで自動的にリスナーが解除されます。
>
> 詳しくは [コールドObservableとホットObservable](/guide/observables/cold-and-hot-observables) を参照してください。

## 実践的なユースケース

### 1. クリックイベントの処理

ボタンクリックを制御し、連続クリックを防止します。

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const button = document.createElement('button');
button.innerText = "submit";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  debounceTime(300), // 300ms以内の連続クリックを無視
  map(() => '送信中...')
).subscribe(message => {
  console.log(message);
  // API呼び出しなどの処理
});
```

### 2. フォーム入力のリアルタイム検証

入力イベントをストリーム化し、リアルタイムでバリデーションを実行します。

```typescript
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'email: ';
const emailInput = document.createElement('input');
label.appendChild(emailInput);
document.body.appendChild(label);
const email$ = fromEvent<InputEvent>(emailInput, 'input');

email$.pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(500), // 入力が止まってから500ms後に処理
  distinctUntilChanged() // 値が変わった時のみ
).subscribe(email => {
  console.log('検証対象:', email);
  // メールアドレスのバリデーション処理
  validateEmail(email);
});

function validateEmail(email: string): void {
  const isValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  console.log(isValid ? '有効なメールアドレス' : '無効なメールアドレス');
}
```

### 3. ドラッグ&ドロップの実装

マウスイベントを組み合わせて、ドラッグ&ドロップを実装します。

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, takeUntil, map } from 'rxjs';

// ドラッグ可能な要素を作成
const element = document.createElement('div');
element.style.width = '100px';
element.style.height = '100px';
element.style.backgroundColor = '#333';
element.style.position = 'absolute'; // 絶対配置に設定
element.style.left = '50px'; // 初期位置
element.style.top = '50px';
element.style.cursor = 'move'; // ドラッグ可能なカーソル
document.body.appendChild(element);

const mousedown$ = fromEvent<MouseEvent>(element, 'mousedown');
const mousemove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseup$ = fromEvent<MouseEvent>(document, 'mouseup');

mousedown$.pipe(
  switchMap(startEvent => {
    // 要素内のクリック位置を記録
    const startX = startEvent.clientX - element.offsetLeft;
    const startY = startEvent.clientY - element.offsetTop;

    return mousemove$.pipe(
      map(moveEvent => ({
        left: moveEvent.clientX - startX,
        top: moveEvent.clientY - startY
      })),
      takeUntil(mouseup$) // マウスアップで終了
    );
  })
).subscribe(({ left, top }) => {
  // 要素の位置を更新
  element.style.left = `${left}px`;
  element.style.top = `${top}px`;
});
```

### 4. スクロールイベントの監視

無限スクロールやスクロール位置の追跡に使用します。

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

const scroll$ = fromEvent(window, 'scroll');

scroll$.pipe(
  throttleTime(200), // 200msごとに1回のみ処理
  map(() => window.scrollY)
).subscribe(scrollPosition => {
  console.log('スクロール位置:', scrollPosition);

  // ページ下部に到達したら追加コンテンツを読み込む
  if (scrollPosition + window.innerHeight >= document.body.scrollHeight - 100) {
    console.log('追加コンテンツを読み込み');
    // loadMoreContent();
  }
});
```

## パイプラインでの使用

`fromEvent()`は、イベントストリームを起点としたパイプライン処理に最適です。

```typescript
import { fromEvent } from 'rxjs';
import { map, filter, scan } from 'rxjs';

const button = document.createElement('button');
button.innerText = "カウンター";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  filter((event: Event) => {
    // Shiftキーを押しながらのクリックのみカウント
    return (event as MouseEvent).shiftKey;
  }),
  scan((count, _) => count + 1, 0),
  map(count => `クリック回数: ${count}`)
).subscribe(message => console.log(message));
```

## よくある間違い

### 1. 購読解除を忘れる

#### ❌ 間違い - 購読解除を忘れるとメモリリークの原因に

```typescript
import { fromEvent } from 'rxjs';

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  clicks$.subscribe(console.log); // 購読解除されない！
}

setupEventListener();
```

#### ✅ 正しい - 必ず購読解除する

```typescript
import { fromEvent } from 'rxjs';
import { Subscription } from 'rxjs';

let subscription: Subscription;

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  subscription = clicks$.subscribe(console.log);
}

function cleanup() {
  if (subscription) {
    subscription.unsubscribe();
  }
}

setupEventListener();
// コンポーネント破棄時などにcleanup()を呼ぶ
```

> [!WARNING]
> **メモリリークに注意**
>
> SPAやコンポーネントベースのフレームワークでは、コンポーネント破棄時に必ず購読解除してください。購読解除を忘れると、イベントリスナーが残り続けてメモリリークの原因になります。

### 2. 複数のイベントリスナーを重複登録


#### ❌ 間違い - 同じイベントに複数回購読すると、複数のリスナーが登録される
```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Observer 1'));
clicks$.subscribe(() => console.log('Observer 2'));
// クリックすると両方のログが表示される（2つのリスナーが登録されている）
```

#### ✅ 正しい - 必要に応じてshare()でマルチキャスト
```ts
import { fromEvent } from 'rxjs';
import { share } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(share());

clicks$.subscribe(() => console.log('Observer 1'));
clicks$.subscribe(() => console.log('Observer 2'));
// 1つのリスナーが共有される
```

## パフォーマンスの考慮事項

高頻度で発火するイベント（scroll, mousemove, resize等）を扱う際は、パフォーマンスに注意が必要です。

> [!TIP]
> **高頻度イベントの最適化**:
> - `throttleTime()` - 一定時間ごとに1回のみ処理
> - `debounceTime()` - 入力が止まってから処理
> - `distinctUntilChanged()` - 値が変わった時のみ処理

#### ❌ パフォーマンス問題 - リサイズのたびに処理

```typescript
import { fromEvent } from 'rxjs';

const resize$ = fromEvent(window, 'resize');

resize$.subscribe(() => {
  console.log('リサイズ処理'); // 高負荷処理
});
```

#### ✅ 最適化 - 200msごとに1回のみ処理
```typescript
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

const resize$ = fromEvent(window, 'resize');
resize$.pipe(
  throttleTime(200)
).subscribe(() => {
  console.log('リサイズ処理'); // 負荷軽減
});
```

## 関連するCreation Functions

| Function | 違い | 使い分け |
|----------|------|----------|
| **[from()](/guide/creation-functions/basic/from)** | 配列・Promiseから変換 | イベント以外のデータをストリーム化 |
| **[interval()](/guide/creation-functions/basic/interval)** | 一定間隔で発行 | 定期的な処理が必要 |
| **fromEventPattern()** | カスタムイベント登録 | EventEmitter以外の独自イベントシステム |

## まとめ

- `fromEvent()`はDOMイベントやEventEmitterをObservableに変換
- 購読時にリスナー登録、購読解除時に自動削除（メモリリーク防止）
- Hot Observableとして動作
- 必ず購読解除を実行してメモリリークを防ぐ
- 高頻度イベントは`throttleTime()`や`debounceTime()`で最適化

## 次のステップ

- [interval() - 一定間隔で値を発行](/guide/creation-functions/basic/interval)
- [timer() - 遅延後に発行開始](/guide/creation-functions/basic/timer)
- [基本作成系の概要に戻る](/guide/creation-functions/basic/)
