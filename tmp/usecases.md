# ⚡ RxJSが活躍するWebサービス分野と実装サンプル集

RxJS（Reactive Extensions for JavaScript）は、**「時間とともに変化するデータ」を安全に扱うためのライブラリ**です。  
Angularだけでなく、PWA、リアルタイム通信、AIストリーミングなど、幅広い領域で活用できます。


## 🛰️ 1. リアルタイム通信・ストリーミング系

### 📡 主な用途
- WebSocketやServer-Sent Events(SSE)のメッセージ購読  
- チャット、通知、株価更新、IoTセンサー値の監視  
- WebRTCデータチャンネル制御  

### 💻 サンプルコード
```ts
import { webSocket } from 'rxjs/webSocket';
import { filter, map } from 'rxjs/operators';

const socket$ = webSocket('wss://example.com/chat');

socket$
  .pipe(
    filter(msg => msg.room === 'general'),
    map(msg => `[${msg.user}] ${msg.text}`)
  )
  .subscribe(console.log);
````

### 💡 ポイント

* `webSocket()`で双方向通信をストリーム化
* `filter`, `map`, `takeUntil`でリアルタイム制御
* 通信停止や再接続もRxJSで安全に制御可能


## 🧩 2. UI／状態管理・フォーム制御系

### 🪄 主な用途

* 入力フォームの依存更新、検索候補補完
* コンポーネント間通信、Undo/Redo処理
* SignalsやStore（NgRx／Svelte $store）との統合

### 💻 サンプルコード

```ts
import { fromEvent, combineLatest } from 'rxjs';
import { debounceTime, map, switchMap } from 'rxjs/operators';

const search$ = fromEvent(document.getElementById('search'), 'input')
  .pipe(map(e => e.target.value));

const sort$ = fromEvent(document.getElementById('sort'), 'change')
  .pipe(map(e => e.target.value));

combineLatest([search$, sort$])
  .pipe(
    debounceTime(300),
    switchMap(([term, order]) => fetch(`/api?q=${term}&sort=${order}`).then(res => res.json()))
  )
  .subscribe(renderResults);
```

### 💡 ポイント

* UIイベントをリアクティブに組み合わせ可能
* Angularなら`FormControl.valueChanges`でも同様の構成が使える
* Svelteでも`fromEvent`を使えば同じ思想で構築可能


## 📦 3. オフライン／同期・PWA系

### 🔁 主な用途

* オフラインキャッシュ管理（Service Worker連携）
* ネットワーク状態監視、再同期制御
* IndexedDBとの非同期同期処理

### 💻 サンプルコード

```ts
import { fromEvent, merge, mapTo } from 'rxjs';

const online$ = fromEvent(window, 'online').pipe(mapTo(true));
const offline$ = fromEvent(window, 'offline').pipe(mapTo(false));

merge(online$, offline$).subscribe(isOnline => {
  console.log('Network status:', isOnline ? 'Online' : 'Offline');
});
```

### 💡 ポイント

* オンライン・オフラインイベントを統一的に扱える
* PWAアプリのバックグラウンド同期制御にも応用可能
* キャッシュ更新のトリガー処理もRxJSで制御しやすい


## 🤖 4. AI／ストリーミングAPI／バックエンド処理系

### 🧠 主な用途

* OpenAIのストリーミングレスポンス（SSE）処理
* Webhookや非同期イベントの逐次処理
* NestJS（RxJS標準搭載）でのリアクティブAPI設計

### 💻 サンプルコード

```ts
import { from, fromEventPattern } from 'rxjs';
import { map, mergeMap } from 'rxjs/operators';

const stream$ = from(fetch('/api/ai/stream'));

stream$
  .pipe(
    mergeMap(res => res.body),
    map(chunk => decodeText(chunk))
  )
  .subscribe(text => updateUI(text));
```

### 💡 ポイント

* OpenAIなどのトークン逐次出力に最適
* Node.js（NestJS, Fastifyなど）とも親和性高い
* AI応答をストリームとしてUIに反映できる


## 🧱 5. アーキテクチャ／設計パターン系

### 🧩 主な用途

* DDD（ドメイン駆動設計）におけるイベントフロー管理
* クリーンアーキテクチャのデータフロー分離
* UI層〜ドメイン層〜データ層をストリームで接続

### 💻 サンプルコード（Angular/Nest構成例）

```ts
// service layer
loadUser$ = this.http.get<User[]>('/api/users').pipe(shareReplay(1));

// component
this.loadUser$.subscribe(users => this.users.set(users));
```

### 💡 ポイント

* `shareReplay`でキャッシュ化し、複数購読でも再呼び出し不要
* `combineLatest`で依存関係を明示的に設計可能
* SignalsやStoreと併用で、**「リアクティブな設計思想」**を体現できる


## 🌐 まとめ：分野別マッピング

| カテゴリ     | 活用領域                   | キーワード            |
| -------- | ---------------------- | ---------------- |
| 📡 通信    | WebSocket, SSE, WebRTC | リアルタイムチャット・IoT監視 |
| 🧩 UI/UX | SPA, Dashboard, Form   | 入力制御・状態管理        |
| 📦 同期    | PWA, Mobile            | キャッシュ・再同期        |
| 🤖 AI    | Streaming API, Chat    | トークン逐次出力・フロー制御   |
| 🧱 設計    | Clean Architecture     | 時間安全・テスト容易       |


## 🔗 関連リンク

* [RxJS公式ドキュメント](https://rxjs.dev/)
* [Learn RxJS Operators](https://www.learnrxjs.io/)
* [RxJS Marbles Testing Guide](https://rxjs.dev/guide/testing/marble-testing)
* [あなたのサイト：RxJS with TypeScript](https://shuji-bonji.github.io/RxJS-with-TypeScript/)

