---
description: Cold ObservableとHot Observableの違いを詳しく解説します。購読ごとのデータストリームの独立性、shareやshareReplayによるColdからHotへの変換方法、APIリクエストのキャッシュなど実践的な活用例を紹介します。
---
# コールドObservableとホットObservable

RxJSを使用する上で重要な概念の一つが、「コールドObservable」と「ホットObservable」の区別です。この違いを理解することは、効率的なObservableの使い方を習得するために不可欠です。

## なぜCold/Hotの理解が重要か

Cold/Hotの違いを理解していないと、以下のような問題に直面します。

- **意図しない重複実行** - API呼び出しが複数回実行される
- **メモリリーク** - 購読を適切に管理できない
- **パフォーマンス問題** - 不要な処理が繰り返される
- **データ不整合** - 期待したデータが受け取れない

## Cold vs Hotの違い（比較表）

まず、全体像を把握しましょう。

| 比較項目 | Cold Observable | Hot Observable |
|----------|------------------|----------------|
| **購読なしでの実行** | 実行されない（購読されて初めて実行） | 実行される（subscribeされなくとも値を流す） |
| **データ発行タイミング** | `subscribe()` された時に開始 | 発行側のタイミングで開始（購読と無関係） |
| **実行の再利用** | 毎回新たに実行される | 既存のストリームを複数で共有 |
| **データの一貫性** | 各購読で独立した値を受け取る | 途中から購読すると過去の値を受け取れない |
| **主な使用例** | HTTPリクエスト、非同期処理 | UIイベント、WebSocket、リアルタイム通信 |
| **使用場面** | 各処理が独立している場合 | 状態共有、イベントブロードキャスト |

**判断基準：** 各購読者に対して処理を再実行すべきか？それともストリームを共有すべきか？

## コールドObservable

### 特徴

- **購読されるたびに新しいデータストリームが作成される**
- **購読されるまでデータの発行を開始しない（遅延実行）**
- **すべてのサブスクライバーは、Observableの最初から全データを受け取る**

Cold Observableは、subscribeするたびに新しい実行コンテキストが生成されます。
これはHTTPリクエストや非同期処理など、毎回新しい処理が必要な場合に適しています。

### コード例

```typescript
import { Observable } from 'rxjs';

// コールドObservableの例
const cold$ = new Observable<number>(subscriber => {
  console.log('データソースの作成 - 新しい購読');
  const randomValue = Math.random();
  subscriber.next(randomValue);
  subscriber.complete();
});

// 1回目の購読
console.log('--- 1回目の購読 ---');
cold$.subscribe(value => console.log('購読者1:', value));

// 2回目の購読（異なるデータが生成される）
console.log('--- 2回目の購読 ---');
cold$.subscribe(value => console.log('購読者2:', value));
```

#### 実行結果
```sh
--- 1回目の購読 ---
データソースの作成 - 新しい購読
購読者1: 0.259632...
--- 2回目の購読 ---
データソースの作成 - 新しい購読  ← 再実行される
購読者2: 0.744322...  ← 異なる値
```

> [!TIP] 重要なポイント
> 購読するたびに「データソースの作成」が実行され、異なる値が生成されます。

### よくあるコールドObservable（見分け方）

以下のObservableは通常Coldです。

```typescript
import { of, from, interval, timer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Creation Functions
of(1, 2, 3)                    // Cold
from([1, 2, 3])                // Cold
from(fetch('/api/data'))       // Cold

// 時間オペレーター
interval(1000)                 // Cold
timer(1000)                    // Cold

// HTTP リクエスト
ajax('/api/users')             // Cold
```

> [!TIP] ルール
> Creation Functions、時間オペレーター、HTTPリクエストは基本的にCold

## ホットObservable

### 特徴

- **subscribeされなくとも値を流す（購読の有無に関わらず実行される）**
- **購読を開始した時点以降のデータのみを受け取る**
- **一つのデータソースが複数のサブスクライバーに共有される**

Hot Observableは、ストリームの発行タイミングが購読とは無関係であり、購読者は途中から参加することになります。

### コード例

```typescript
import { Subject } from 'rxjs';

// ホットObservableの例（Subject使用）
const hot$ = new Subject<number>();

// 最初の購読
console.log('--- 購読者1 開始 ---');
hot$.subscribe(value => console.log('購読者1:', value));

// データ発行
hot$.next(1);
hot$.next(2);

// 2番目の購読（後からの購読）
console.log('--- 購読者2 開始 ---');
hot$.subscribe(value => console.log('購読者2:', value));

// さらにデータ発行
hot$.next(3);
hot$.next(4);

hot$.complete();
```

#### 実行結果
```sh
--- 購読者1 開始 ---
購読者1: 1
購読者1: 2
--- 購読者2 開始 ---
購読者1: 3
購読者2: 3  ← 購読2は3から参加（1, 2は受け取れない）
購読者1: 4
購読者2: 4
```

> [!TIP] 重要なポイント
> 購読者2は途中から参加したため、過去の値（1, 2）は受け取れません。

### よくあるホットObservable（見分け方）

以下のObservableは常にHotです。

```typescript
import { Subject, BehaviorSubject, ReplaySubject, fromEvent } from 'rxjs';
import { webSocket } from 'rxjs/webSocket';

// Subject系
new Subject()                  // Hot
new BehaviorSubject(0)         // Hot
new ReplaySubject(1)           // Hot

// DOM イベント
fromEvent(button, 'click')     // Hot
fromEvent(window, 'resize')    // Hot

// WebSocket
webSocket('ws://localhost:8080') // Hot
```

> [!TIP] ルール
> Subject系、DOMイベント、WebSocketは常にHot

## コールドObservableをホットに変換する方法

RxJSでは、Cold ObservableをHotに変換する手段として主に以下が使われます。

- `share()` - 簡易ホット化（推奨）
- `shareReplay()` - 過去の値をキャッシュしてホット化
- ~~`multicast()`~~ - 非推奨（RxJS v7で非推奨、v8で削除）

### share()オペレーター

`share()`は、コールドObservableをホットObservableに変換する最も一般的な方法です。

```typescript
import { interval } from 'rxjs';
import { share, take } from 'rxjs';

// HTTP呼び出しをシミュレート
const makeHttpRequest = () => {
  console.log('HTTP呼び出し実行!');
  return interval(1000).pipe(take(3));
};

// ❌ コールドObservable（共有なし）
const cold$ = makeHttpRequest();

cold$.subscribe(val => console.log('購読者1:', val));
cold$.subscribe(val => console.log('購読者2:', val));
// → HTTP呼び出しが2回実行される

// ✅ ホットObservable（share使用）
const shared$ = makeHttpRequest().pipe(share());

shared$.subscribe(val => console.log('共有購読者1:', val));
shared$.subscribe(val => console.log('共有購読者2:', val));
// → HTTP呼び出しは1回だけ、結果を共有
```

**実行結果（Cold）：**
```sh
HTTP呼び出し実行!  ← 1回目
購読者1: 0
HTTP呼び出し実行!  ← 2回目（重複！）
購読者2: 0
...
```

**実行結果（Hot）：**
```sh
HTTP呼び出し実行!  ← 1回だけ
共有購読者1: 0
共有購読者2: 0  ← 同じストリームを共有
...
```

> [!NOTE] ユースケース
> - 複数コンポーネントで同じAPI結果を使う
> - 副作用（HTTP呼び出しなど）の重複を避ける

### shareReplay()オペレーター

`shareReplay()`は`share()`の拡張版で、**過去の値をキャッシュ**して新しいサブスクライバーに再生します。

```typescript
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

const request$ = interval(1000).pipe(
  take(3),
  shareReplay(2)  // 最後の2つの値をキャッシュ
);

// 1回目の購読
request$.subscribe(val => console.log('購読者1:', val));

// 3.5秒後に2回目の購読（ストリーム完了後）
setTimeout(() => {
  console.log('--- 購読者2開始（完了後） ---');
  request$.subscribe(val => console.log('購読者2:', val));
}, 3500);
```

#### 実行結果
```sh
購読者1: 0
購読者1: 1
購読者1: 2
--- 購読者2開始（完了後） ---
購読者2: 1  ← キャッシュされた値（最後の2つ）
購読者2: 2  ← キャッシュされた値
```

> [!NOTE] ユースケース
> - API結果のキャッシュ
> - 初期状態の共有（最新の1件のみキャッシュ）
> - 遅延購読者への過去データ提供

> [!WARNING] shareReplayの注意点
> `shareReplay()`は購読が0になってもキャッシュを保持し続けるため、メモリリークの原因になることがあります。詳しくは [Chapter 10: shareReplayの誤用](/guide/anti-patterns/common-mistakes#4-sharereplay-の誤用) を参照してください。

### multicast()について

> [!NOTE]
> `multicast()` は柔軟ですが、RxJS v7で非推奨となり、v8で削除されました。現在は `share()` や `shareReplay()` を使用してください。詳しくは [share()オペレーター解説](/guide/operators/multicasting/share) を参照してください。

## 実践的な例: APIキャッシュサービス

実際のアプリケーションでよくあるパターン：複数のコンポーネントが同じAPIデータを必要とする場合。

```typescript
import { Observable, of, throwError } from 'rxjs';
import { catchError, shareReplay, delay, tap } from 'rxjs';

// シンプルなキャッシュサービス
class UserService {
  private cache$: Observable<User[]> | null = null;

  getUsers(): Observable<User[]> {
    // キャッシュがあればそれを返す
    if (this.cache$) {
      console.log('キャッシュから返却');
      return this.cache$;
    }

    // 新しいリクエストを作成しキャッシュ
    console.log('新規リクエスト実行');
    this.cache$ = this.fetchUsersFromAPI().pipe(
      catchError(err => {
        this.cache$ = null;  // エラー時はキャッシュをクリア
        return throwError(() => err);
      }),
      shareReplay(1)  // 最後の結果をキャッシュ
    );

    return this.cache$;
  }

  private fetchUsersFromAPI(): Observable<User[]> {
    // 実際のAPIリクエストをシミュレート
    return of([
      { id: 1, name: '山田太郎' },
      { id: 2, name: '佐藤花子' }
    ]).pipe(
      delay(1000),
      tap(() => console.log('APIからデータ受信'))
    );
  }

  clearCache(): void {
    this.cache$ = null;
    console.log('キャッシュクリア');
  }
}

interface User {
  id: number;
  name: string;
}

// 使用例
const userService = new UserService();

// コンポーネント1: データを要求
userService.getUsers().subscribe(users =>
  console.log('コンポーネント1:', users)
);

// コンポーネント2: 2秒後にデータを要求
setTimeout(() => {
  userService.getUsers().subscribe(users =>
    console.log('コンポーネント2:', users)
  );
}, 2000);

// キャッシュをクリアして再度要求
setTimeout(() => {
  userService.clearCache();
  userService.getUsers().subscribe(users =>
    console.log('コンポーネント3:', users)
  );
}, 4000);
```

#### 実行結果
```sh
新規リクエスト実行
APIからデータ受信
コンポーネント1: [{id: 1, name: '山田太郎'}, {id: 2, name: '佐藤花子'}]
キャッシュから返却  ← API呼び出しなし
コンポーネント2: [{id: 1, name: '山田太郎'}, {id: 2, name: '佐藤花子'}]
キャッシュクリア
新規リクエスト実行  ← 再度API呼び出し
APIからデータ受信
コンポーネント3: [{id: 1, name: '山田太郎'}, {id: 2, name: '佐藤花子'}]
```

**ポイント：**
- `shareReplay(1)`で最後のレスポンスをキャッシュ
- 複数のコンポーネントがデータを共有（API呼び出しは1回のみ）
- エラー時やクリア時は適切にキャッシュを破棄

## 使用するタイミング

<div class="comparison-cards">

::: tip コールド
#### こんな時に使う
- 各サブスクライバーが独自のデータセットを必要とする場合
- 新しく開始するプロセスやアクションを表現する場合
- 副作用の重複が問題にならない場合

#### 例
- フォーム送信のたびに新しいPOSTリクエストを送る
- ユーザーごとに異なるタイマーが必要
- 各購読で独立した計算を実行
:::

::: tip ホット
#### こんな時に使う
- 複数のコンポーネント間でデータを共有する場合
- リソースを節約したい場合（例：HTTP呼び出しの回数を減らす）
- イベントストリームを表現する場合
- ステート管理やサービス間の通信

#### 例
- アプリケーション全体で共有する設定情報
- ユーザーのログイン状態
- リアルタイムメッセージ（WebSocket）
- DOMイベント（クリック、スクロールなど）
:::

</div>

## まとめ

コールドObservableとホットObservableを理解し、適切に使い分けることは、効率的なRxJSアプリケーションを構築するための重要なスキルです。

::: tip 重要なポイント
- **Cold Observable**: 購読されて初めて動き出すストリーム（購読ごとに独立実行）
- **Hot Observable**: すでに動いているストリームを共有（複数購読で同じ実行）
- **share()**: ColdをHotに変換する最も簡単な方法
- **shareReplay()**: 過去の値をキャッシュしてHotに変換（API結果の共有に便利）
:::

::: tip 設計判断の基準
- 複数のサブスクライバー間でデータを共有する必要があるか？
- 過去の値をキャッシュし、新しいサブスクライバーに提供する必要があるか？
- 副作用（HTTPリクエストなど）の重複をどのように管理するか？
:::

これらの考慮事項を元に、適切なObservableの種類とオペレーターを選択することで、効率的で堅牢なリアクティブアプリケーションを構築できます。

## 関連セクション

- **[share()オペレーター](/guide/operators/multicasting/share)** - share()の詳細解説
- **[shareReplayの誤用](/guide/anti-patterns/common-mistakes#4-sharereplay-の誤用)** - よくある間違いと対処法
- **[Subject](/guide/subjects/what-is-subject)** - HotなSubjectの理解

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>
