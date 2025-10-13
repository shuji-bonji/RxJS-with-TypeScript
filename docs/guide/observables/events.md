---
description: fromEventを使ってDOMイベントをObservableとして扱う方法を解説します。クリック、マウス移動、キーボードイベントのストリーム化から、ドラッグ&ドロップ、フォーム検証など複雑なUI処理の実装まで実践的な例を紹介します。
---

# イベントのストリーム化

ここでは、RxJSでのObservableの作成方法について、基本的な構文から実践的な用途までを網羅的に紹介します。

## 従来のイベント処理とRxJSの比較

### クリックイベント
#### ◇ 従来のDOMイベント処理

```ts
document.addEventListener('click', (event) => {
  console.log('クリックされました:', event);
});

// 処理結果:
// クリックされました: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

#### ◆　RxJSを使用したイベント処理

```ts
import { fromEvent } from 'rxjs';

// クリックイベントのストリーム化
const clicks$ = fromEvent(document, 'click');
clicks$.subscribe(event => console.log('RxJSクリック:', event));

// 処理結果:
// RxJSクリック: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### マウス移動イベント
#### ◇ 従来のDOMイベント処理
```ts
document.addEventListener('mousemove', (event) => {
  console.log('マウス位置:', event.clientX, event.clientY);
});

// 処理結果:
// マウス位置: 4 357
// マウス位置: 879 148
// マウス位置: 879 148
```

#### ◆　RxJSを使用したイベント処理

```ts
import { fromEvent } from 'rxjs';
import { map, throttleTime } from 'rxjs';

// マウス移動イベントのストリーム化（スロットリング付き）
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  throttleTime(100), // 100ミリ秒ごとに制限
  map(event => ({ x: event.clientX, y: event.clientY }))
);
mouseMove$.subscribe(position => console.log('マウス位置:', position));

// 処理結果:
// マウス位置: {x: 177, y: 453}
// マウス位置: {x: 1239, y: 297}
```

### キーボードイベント
#### ◇ 従来のDOMイベント処理
```ts
document.addEventListener('keydown', (event) => {
  console.log('キーが押されました:', event.key);
});

// 処理結果:
// キーが押されました: h
// キーが押されました: o
// キーが押されました: g
// キーが押されました: e
```

#### ◆　RxJSを使用したイベント処理

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// キーボードイベントのストリーム化
const keyDown$ = fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key)
);
keyDown$.subscribe(key => console.log('押されたキー:', key));

// 処理結果:
// 押されたキー: h
// 押されたキー: o
// 押されたキー: g
// 押されたキー: e
```


## fromEvent の使い方と応用

`fromEvent`は、DOMイベントをObservableに変換する最も一般的な方法です。`fromEvent` は最も基本的なイベント → Observable 変換関数で、RxJSを使ったイベント処理の出発点となります。

### 基本的な使用方法
```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe((event) => console.log('RxJSクリック:', event));

// 処理結果:
// RxJSクリック: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```


### イベントターゲットと型の指定
```ts
import { fromEvent } from 'rxjs';

const myButton = document.querySelector('#myButton')!;
const buttonClicks$ = fromEvent<MouseEvent>(myButton, 'click');
buttonClicks$.subscribe((event) => console.log('myButtonクリック:', event));

// 処理結果:
// myButtonクリック: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### オプションの指定（キャプチャフェーズでリスニング）
```ts
import { fromEvent } from 'rxjs';

const capturedClicks$ = fromEvent(document, 'click', { capture: true });
capturedClicks$.subscribe((event) => console.log('ページ クリック:', event));

// 処理結果:
// ページ クリック: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!NOTE]
> DOMイベントの伝播には「キャプチャ」と「バブリング」の2つのフェーズがあります。
> 通常は「バブリング」（イベントが子要素から親要素へ伝播）ですが、`capture: true` を指定すると「キャプチャフェーズ」（親要素から子要素へ伝播）でリスニングします。
> これにより、子要素が処理する前に、親要素でイベントを検出することができます。

## 複数のイベントソースの処理

RxJSでは複数のイベントソースを `merge` や `combineLatest` によって統合し、共通のロジックに集約することが可能です。

```ts
import { fromEvent, merge } from 'rxjs';
import { map } from 'rxjs';

// 複数のボタンからのクリックを統合
const button1Clicks$ = fromEvent(document.querySelector('#button1')!, 'click')
  .pipe(map(() => 'ボタン1がクリックされました'));
  
const button2Clicks$ = fromEvent(document.querySelector('#button2')!, 'click')
  .pipe(map(() => 'ボタン2がクリックされました'));

// 両方のイベントストリームをマージ
const allButtonClicks$ = merge(button1Clicks$, button2Clicks$);
allButtonClicks$.subscribe(message => console.log(message));
```

#### 実行結果
```
ボタン1がクリックされました
```
```
ボタン2がクリックされました
```


## イベントストリームの変換と操作

イベントをストリーム化する利点は、RxJSのオペレータを使用して変換や操作が簡単に行えることです。

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  debounceTime,
  distinctUntilChanged,
} from 'rxjs';

// 入力フィールドの値の変更を監視
const input$ = fromEvent<InputEvent>(
  document.querySelector('#searchInput')!,
  'input'
).pipe(
  map((event) => (event.target as HTMLInputElement).value),
  filter((text) => text.length > 2), // 3文字以上の場合のみ処理
  debounceTime(300), // 300ms間隔を空ける（タイピング中は発火しない）
  distinctUntilChanged() // 前回と同じ値なら発火しない
);

input$.subscribe((searchText) => {
  console.log('検索テキスト:', searchText);
  // ここで検索APIを呼び出すなどの処理
});

```

#### 実行結果
```sh
検索テキスト: abc
検索テキスト: abcd
```
このように、入力イベントなどをストリームとして扱うことで、UIの反応性や保守性を大きく向上させることができます。

## ドラッグ&ドロップの実装例

複数イベントの組み合わせを使った例として、マウスのドラッグ操作をObservableで管理してみましょう。

```ts
import { fromEvent } from 'rxjs';
import { map, switchMap, takeUntil, tap } from 'rxjs';

function implementDragAndDrop(element: HTMLElement) {
  // マウスダウンイベントのストリーム
  const mouseDown$ = fromEvent<MouseEvent>(element, 'mousedown');
  
  // ドキュメント上のマウス移動イベントのストリーム
  const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
  
  // ドキュメント上のマウスアップイベントのストリーム
  const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

  // ドラッグ処理
  const drag$ = mouseDown$.pipe(
    tap(event => {
      // ブラウザのデフォルトドラッグ処理を防止
      event.preventDefault();
    }),
    switchMap(startEvent => {
      // 初期位置の記録
      const initialX = startEvent.clientX;
      const initialY = startEvent.clientY;
      const elementX = parseInt(element.style.left || '0', 10);
      const elementY = parseInt(element.style.top || '0', 10);
      
      // マウス移動のストリームを返す（mouseUpまで）
      return mouseMove$.pipe(
        map(moveEvent => ({
          x: elementX + (moveEvent.clientX - initialX),
          y: elementY + (moveEvent.clientY - initialY)
        })),
        takeUntil(mouseUp$) // マウスアップで終了
      );
    })
  );

  // 購読して位置を更新
  drag$.subscribe(position => {
    element.style.left = `${position.x}px`;
    element.style.top = `${position.y}px`;
    console.log(`${element.style.left}, ${element.style.top}`);
  });
}

// 使用例
const draggableElement = document.querySelector('#draggable') as HTMLElement;
implementDragAndDrop(draggableElement);
```

#### 実行結果
```
1px, 0px
1px, -1px
0px, -2px
0px, -3px
0px, -4px
```

## フォーム入力の監視と検証

フォームバリデーションのような典型的なUI処理も、Observableを使えばより宣言的に、安全に記述できます。

```ts
import { fromEvent, combineLatest } from 'rxjs';
import { map, startWith, debounceTime } from 'rxjs';

function validateForm() {
  // 入力フィールドの参照
  const usernameInput = document.querySelector('#username') as HTMLInputElement;
  const passwordInput = document.querySelector('#password') as HTMLInputElement;
  const submitButton = document.querySelector('#submit') as HTMLButtonElement;

  // 入力フィールドの変更ストリーム
  const username$ = fromEvent<InputEvent>(usernameInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // 初期値
  );

  const password$ = fromEvent<InputEvent>(passwordInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // 初期値
  );

  // 両方の入力を組み合わせて検証
  const formValid$ = combineLatest([username$, password$]).pipe(
    debounceTime(300),
    map(([username, password]) => {
      return username.length >= 3 && password.length >= 6;
    })
  );

  // フォームの検証状態に基づいてボタンの有効/無効を切り替え
  formValid$.subscribe(isValid => {
    submitButton.disabled = !isValid;
  });

  // フォーム送信の処理
  const submit$ = fromEvent(submitButton, 'click');
  submit$.subscribe(() => {
    console.log('フォーム送信:', {
      username: usernameInput.value,
      password: passwordInput.value
    });
    // ここで実際の送信処理を行う
  });
}

// 使用例
validateForm();
```
#### 実行結果
```
フォーム送信: {username: 'testuser', password: '123456'}
```

## イベント一覧へのリンク

すべてのイベントとその `fromEvent` での利用可否の一覧については、以下のリンクから参照できます。

➡️ **[イベント一覧表](./events-list.md)**

この一覧では、JavaScript の標準イベントが `fromEvent` に対応しているかどうかが明確に示されているため、RxJS を使ったリアクティブプログラミングの際に便利です。


## fromEvent で利用できないイベント {#cannot-used-fromEvent}

`fromEvent` は DOM の `EventTarget` インターフェースに依存しています。そのため、以下のイベントは直接 `fromEvent` で扱うことができません。これらは特定のオブジェクトに紐付いているか、独自のイベントリスナーを持っているためです。

| イベント名         | 型                    | 理由                                             |
| ------------------ | --------------------- | ------------------------------------------------ |
| `beforeunload`    | `BeforeUnloadEvent`  | ウィンドウが閉じられる前に実行するイベントで、DOM イベントリスナーではなくブラウザの動作に依存 |
| `unload`          | `Event`              | ページが完全に閉じるとリスナーも削除されるため、RxJS の Observable では無効 |
| `message`         | `MessageEvent`       | ServiceWorker や WebWorker からのメッセージは `fromEvent` で直接キャプチャできない |
| `popstate`        | `PopStateEvent`      | `history.pushState` や `replaceState` の変更は手動でハンドリングが必要 |
| `storage`         | `StorageEvent`       | `localStorage` の変更は `fromEvent` では監視できない（`window.addEventListener` 経由が必要） |
| `languagechange`  | `Event`              | ブラウザ設定の変更は `window` オブジェクトの動作に依存する |
| `fetch`           | `Event`              | `fetch` の進行状況（`onprogress` など）は通常の DOM イベントではない |
| `WebSocket`       | `Event`              | `onmessage`, `onopen`, `onclose` は独自のイベントリスナーを持つ |
| `ServiceWorker`   | `Event`              | `message`, `install`, `activate` などは `fromEvent` で扱えない |

### 代替手段
これらのイベントを監視する場合、次の方法を利用してください。

- `window.addEventListener('message', callback)`
- `window.addEventListener('popstate', callback)`
- `window.addEventListener('storage', callback)`
- `WebSocket` の場合、`ws.addEventListener('message', callback)`
- `ServiceWorker` の場合、`navigator.serviceWorker.addEventListener('message', callback)`

RxJS でラップする場合、`fromEvent` の代わりに次のように Observable を手動で生成できます。

```typescript
import { Observable } from 'rxjs';

const message$ = new Observable<MessageEvent>(observer => {
  const handler = (event: MessageEvent) => observer.next(event);
  window.addEventListener('message', handler);

  // 解除処理
  return () => window.removeEventListener('message', handler);
});

message$.subscribe(event => {
  console.log('Message received:', event.data);
});
```

## まとめとベストプラクティス

本記事では、イベントをObservable化することで得られる利点と具体的な応用方法を見てきました。

RxJSを使ったイベント処理は、以下のようなメリットがあります。

- 宣言的かつ構造化されたイベント管理が可能
- `pipe()` とオペレーターを通じて、イベントに対するフィルタリングや変換、遅延処理が容易
- 複数のイベントソースの統合や複雑な状態の制御も明確に表現できる
- `subscribe` によるサイドエフェクトの集中管理

### ベストプラクティス

- UI部品ごとの `fromEvent` は適切に `unsubscribe`（`takeUntil` などの使用）
- DOM参照は null チェックや `!` 明示で安定化
- ストリームは細かく分割し、`switchMap` や `mergeMap` の使い分けを意識する
- バックエンド通信との組み合わせは `exhaustMap` や `concatMap` などで制御可能

RxJSによるイベントストリーム化は、単なるクリックやキーダウン処理を超えて、**リアクティブなUI構築全体の基本設計思想**となります。

