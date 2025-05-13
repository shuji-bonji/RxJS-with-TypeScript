# イベント一覧表

## 1. マウスイベント

| JavaScript イベント名 | HTML 属性 | 型 | 説明 | fromEventで利用可否 |
|---|---|---|---|---|
| click | onclick | MouseEvent | 要素がクリックされたとき | ✅ |
| dblclick | ondblclick | MouseEvent | 要素がダブルクリックされたとき | ✅ |
| mousedown | onmousedown | MouseEvent | マウスボタンが押されたとき | ✅ |
| mouseup | onmouseup | MouseEvent | マウスボタンが離されたとき | ✅ |
| mousemove | onmousemove | MouseEvent | マウスが動いたとき | ✅ |
| mouseover | onmouseover | MouseEvent | マウスが要素の上に来たとき | ✅ |
| mouseout | onmouseout | MouseEvent | マウスが要素の外に出たとき | ✅ |
| mouseenter | onmouseenter | MouseEvent | マウスが要素に入ったとき（バブリングなし）| ✅ |
| mouseleave | onmouseleave | MouseEvent | マウスが要素から出たとき（バブリングなし）| ✅ |
| contextmenu | oncontextmenu | MouseEvent | 右クリックメニューが開かれたとき | ✅ |


## 2. ポインターイベント

| JavaScript イベント名 | HTML 属性 | 型 | 説明 | fromEventで利用可否 |
|---|---|---|---|---|
| pointerdown | onpointerdown | PointerEvent | ポインター（タッチ、ペン、マウス）が押されたとき | ✅ |
| pointerup | onpointerup | PointerEvent | ポインターが離されたとき | ✅ |
| pointermove | onpointermove | PointerEvent | ポインターが動いたとき | ✅ |
| pointerover | onpointerover | PointerEvent | ポインターが要素の上に来たとき | ✅ |
| pointerout | onpointerout | PointerEvent | ポインターが要素の外に出たとき | ✅ |
| pointerenter | onpointerenter | PointerEvent | ポインターが要素に入ったとき（バブリングなし）| ✅ |
| pointerleave | onpointerleave | PointerEvent | ポインターが要素から出たとき（バブリングなし）| ✅ |
| pointercancel | onpointercancel | PointerEvent | ポインター操作がキャンセルされたとき | ✅ |
| gotpointercapture | ongotpointercapture | PointerEvent | ポインターのキャプチャを取得したとき | ✅ |
| lostpointercapture | onlostpointercapture | PointerEvent | ポインターのキャプチャを失ったとき | ✅ |


## 3. タッチイベント

| JavaScript イベント名 | HTML 属性 | 型 | 説明 | fromEventで利用可否 |
|---|---|---|---|---|
| touchstart | ontouchstart | TouchEvent | 画面がタッチされたとき | ✅ |
| touchmove | ontouchmove | TouchEvent | タッチした指が移動したとき | ✅ |
| touchend | ontouchend | TouchEvent | タッチが終了したとき | ✅ |
| touchcancel | ontouchcancel | TouchEvent | タッチがキャンセルされたとき | ✅ |


## 4. キーボードイベント

| JavaScript イベント名 | HTML 属性 | 型 | 説明 | fromEventで利用可否 |
|---|---|---|---|---|
| keydown | onkeydown | KeyboardEvent | キーが押されたとき | ✅ |
| keypress | onkeypress | KeyboardEvent | キーが押されている間（非推奨） | ✅ |
| keyup | onkeyup | KeyboardEvent | キーが離されたとき | ✅ |


## 5. フォーカス関連イベント

| JavaScript イベント名 | HTML 属性 | 型 | 説明 | fromEventで利用可否 |
|---|---|---|---|---|
| focus | onfocus | FocusEvent | 要素にフォーカスが当たったとき | ✅ |
| blur | onblur | FocusEvent | 要素からフォーカスが外れたとき | ✅ |
| focusin | onfocusin | FocusEvent | 要素または子要素にフォーカスが入ったとき | ✅ |
| focusout | onfocusout | FocusEvent | 要素または子要素からフォーカスが外れたとき | ✅ |


## 6. フォームイベント

| JavaScript イベント名 | HTML 属性 | 型 | 説明 | fromEventで利用可否 |
|---|---|---|---|---|
| change | onchange | Event | 入力内容が変更されたとき | ✅ |
| input | oninput | InputEvent | 入力フィールドの値が変更されたとき | ✅ |
| submit | onsubmit | SubmitEvent | フォームが送信されたとき | ✅ |
| reset | onreset | Event | フォームがリセットされたとき | ✅ |
| select | onselect | Event | テキストが選択されたとき | ✅ |


## 7. ドラッグ & ドロップイベント

| JavaScript イベント名 | HTML 属性 | 型 | 説明 | fromEventで利用可否 |
|---|---|---|---|---|
| drag | ondrag | DragEvent | 要素がドラッグされている間 | ✅ |
| dragstart | ondragstart | DragEvent | ドラッグが開始されたとき | ✅ |
| dragend | ondragend | DragEvent | ドラッグが終了したとき | ✅ |
| dragover | ondragover | DragEvent | ドラッグした要素が他の要素の上にあるとき | ✅ |
| dragenter | ondragenter | DragEvent | ドラッグした要素がターゲットに入ったとき | ✅ |
| dragleave | ondragleave | DragEvent | ドラッグした要素がターゲットから外れたとき | ✅ |
| drop | ondrop | DragEvent | ドラッグされた要素がドロップされたとき | ✅ |


## 8. ウィンドウ & ドキュメントイベント

| JavaScript イベント名 | HTML 属性 | 型 | 説明 | fromEventで利用可否 |
|---|---|---|---|---|
| load | onload | Event | ページが完全に読み込まれたとき | ✅ |
| resize | onresize | UIEvent | ウィンドウサイズが変更されたとき | ✅ |
| scroll | onscroll | Event | ページがスクロールされたとき | ✅ |
| unload | onunload | Event | ページが閉じられたとき | ❌ |
| beforeunload | onbeforeunload | BeforeUnloadEvent | ページが閉じられる直前 | ❌ |
| error | onerror | ErrorEvent | エラーが発生したとき | ✅ |
| visibilitychange | onvisibilitychange | Event | ページの表示状態が変わったとき（タブの切り替えなど） | ✅ |
| fullscreenchange | onfullscreenchange | Event | フルスクリーンの状態が変化したとき | ✅ |


## 9. メディアイベント

| JavaScript イベント名 | HTML 属性 | 型 | 説明 | fromEventで利用可否 |
|---|---|---|---|---|
| play | onplay | Event | メディアの再生が開始されたとき | ✅ |
| pause | onpause | Event | メディアの再生が一時停止されたとき | ✅ |
| ended | onended | Event | メディアの再生が終了したとき | ✅ |
| volumechange | onvolumechange | Event | メディアの音量が変更されたとき | ✅ |
| seeking | onseeking | Event | メディアのシークが開始されたとき | ✅ |
| seeked | onseeked | Event | メディアのシークが完了したとき | ✅ |
| timeupdate | ontimeupdate | Event | メディアの再生時間が更新されたとき | ✅ |


## 10. アニメーション & トランジションイベント

| JavaScript イベント名 | HTML 属性 | 型 | 説明 | fromEventで利用可否 |
|---|---|---|---|---|
| animationstart | onanimationstart | AnimationEvent | アニメーションが開始されたとき | ✅ |
| animationend | onanimationend | AnimationEvent | アニメーションが終了したとき | ✅ |
| animationiteration | onanimationiteration | AnimationEvent | アニメーションが繰り返されたとき | ✅ |
| transitionstart | ontransitionstart | TransitionEvent | CSSトランジションが開始されたとき | ✅ |
| transitionend | ontransitionend | TransitionEvent | CSSトランジションが終了したとき | ✅ |


## 11. その他のイベント

| JavaScript イベント名 | HTML 属性 | 型 | 説明 | fromEventで利用可否 |
|---|---|---|---|---|
| wheel | onwheel | WheelEvent | マウスホイールが回転したとき | ✅ |
| abort | onabort | UIEvent | リソースの読み込みが中断されたとき | ✅ |
| hashchange | onhashchange | HashChangeEvent | URL のハッシュ（`#section1` など）が変更されたとき | ✅ |
| message | onmessage | MessageEvent | Web Workers や iframe からメッセージを受信したとき | ❌ |
| online | ononline | Event | ネットワークがオンライン状態に戻ったとき | ✅ |
| offline | onoffline | Event | ネットワークがオフラインになったとき | ✅ |
| popstate | onpopstate | PopStateEvent | `history.pushState` や `history.back` による状態変化時 | ❌ |
| storage | onstorage | StorageEvent | `localStorage` または `sessionStorage` が変更されたとき | ❌ |
| languagechange | onlanguagechange | Event | 言語設定が変更されたとき（ブラウザの設定変更） | ❌ |



