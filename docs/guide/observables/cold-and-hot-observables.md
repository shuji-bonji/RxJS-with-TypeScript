---
description: Cold ObservableとHot Observableの違いを詳しく解説します。購読ごとのデータストリームの独立性、shareやshareReplayによるColdからHotへの変換方法、APIリクエストのキャッシュなど実践的な活用例を紹介します。
---
# コールドObservableとホットObservable

ここでは、RxJSにおける Cold Observable と Hot Observable の違いと、それぞれの振る舞いや使用例を比較しながら解説します。

RxJSを使用する上で重要な概念の一つが、「コールドObservable」と「ホットObservable」の区別です。この違いを理解することは、効率的なObservableの使い方を習得するために不可欠です。


## コールドObservableとは

コールドObservableは、各サブスクライバー（購読者）に対して独立したデータストリームを提供します。

### 特徴

- 購読されるたびに新しいデータストリームが作成される
- 購読されるまでデータの発行を開始しない（遅延実行）
- すべてのサブスクライバーは、Observableの最初から全データを受け取る

Cold Observableは、subscribeするたびに新しい実行コンテキストが生成されます。  
これはHTTPリクエストや非同期処理など、毎回新しい処理が必要な場合に適しています。

#### 例

```ts
import { Observable } from 'rxjs';

// コールドObservableの例
const cold$ = new Observable<number>(subscriber => {
  console.log('データソースの作成 - 新しい購読');
  const randomValue = Math.random();
  subscriber.next(randomValue);
  subscriber.next(randomValue + 1);
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
購読者1: 0.25963210979373885
購読者1: 1.2596321097937389
--- 2回目の購読 ---
データソースの作成 - 新しい購読
購読者2: 0.7443227343912574
購読者2: 1.7443227343912575
```

### 一般的なコールドObservable

- `of()`, `from()`, `range()` などのクリエーションオペレーター
- `ajax()` によるHTTPリクエスト
- `interval()`, `timer()` などの時間オペレーター

## ホットObservableとは

ホットObservableは、すべてのサブスクライバーが同一のデータストリームを共有します。

### 特徴

- データストリームは購読の有無に関わらず発行される可能性がある
- 購読を開始した時点以降のデータのみを受け取る
- 一つのデータソースが複数のサブスクライバーに共有される

Hot Observableは、ストリームの発行タイミングが購読とは無関係であり、購読者は途中から参加することになります。代表的な例としては、`Subject` や `fromEvent()` などがあります。

### 例

```ts
import { Subject } from 'rxjs';

// ホットObservableの例（Subject使用）
const hot$ = new Subject<number>();

// 最初の購読
console.log('--- 購読者1 開始 ---');
hot$.subscribe(value => console.log('購読者1:', value));

// データ発行
console.log('--- データ発行 1, 2 ---');
hot$.next(1);
hot$.next(2);

// 2番目の購読（後からの購読）
console.log('--- 購読者2 開始 ---');
hot$.subscribe(value => console.log('購読者2:', value));

// さらにデータ発行
console.log('--- データ発行 3, 4 ---');
hot$.next(3);
hot$.next(4);

hot$.complete();
```

#### 実行結果
```sh
--- 購読者1 開始 ---
--- データ発行 1, 2 ---
購読者1: 1
購読者1: 2
--- 購読者2 開始 ---
--- データ発行 3, 4 ---
購読者1: 3
購読者2: 3
購読者1: 4
購読者2: 4
```

### 一般的なホットObservable

- `Subject` およびその派生クラス（`BehaviorSubject`, `ReplaySubject`など）
- `fromEvent()` によるDOMイベント
- WebSocket接続
- ブロードキャストチャネル

## Cold ObservableとHot Observableの違い

Cold Observable と Hot Observable の違いを、以下の表にまとめます。

| 比較項目 | Cold Observable | Hot Observable |
|----------|------------------|----------------|
| データ発行タイミング | `subscribe()` された時に開始 | 発行側のタイミングで開始（購読と無関係） |
| 実行の再利用 | 毎回新たに実行される | 既存のストリームを複数で共有 |
| データの一貫性 | 各購読で独立した値を受け取る | 途中から購読すると過去の値を受け取れない（unless replayed） |
| 主な使用例 | HTTPリクエスト、非同期処理、`of`, `from`, `interval` | UIイベント、WebSocket、`Subject`, `fromEvent` |
| 使用場面 | 各処理が独立している場合 | 状態共有、イベントブロードキャスト、最適化したリソース利用 |

> 💡 判断基準：各購読者に対して処理を再実行すべきか？それともストリームを共有すべきか？

## コールドObservableをホットに変換する方法

RxJSでは、Cold ObservableをHotに変換する手段として主に以下の2つが使われます。

- `share()`：内部的に`multicast`と`refCount`を組み合わせた簡易ホット化演算子
- `multicast()`：低レベルなマルチキャスト変換。明示的にSubjectを渡す必要がある


### share()オペレーター

`share()`オペレーターは、コールドObservableをホットObservableに変換する最も一般的な方法です。
- 複数購読を共有したいが、内部の複雑さを意識したくない場合に便利
- 自動で購読開始・停止の制御（refCount）を行う

```ts
import { interval, Observable } from 'rxjs';
import { share, take, tap } from 'rxjs/operators';

// HTTP呼び出しをシミュレート
function makeHttpRequest(): Observable<number> {
  console.log('HTTP呼び出し実行!'); // 副作用: 実行確認用
  return interval(1000).pipe(
    take(3),
    tap(x => console.log('データ取得:', x))
  );
}

// コールドObservableの場合（共有なし）
console.log('--- コールドObservable ---');
const cold$ = makeHttpRequest();

// 1回目の購読
cold$.subscribe(val => console.log('購読者1:', val));

// 2秒後に2回目の購読
setTimeout(() => {
  cold$.subscribe(val => console.log('購読者2:', val));
}, 2000);

// share()でホット化した場合
setTimeout(() => {
  console.log('--- ホットObservable (share使用) ---');
  const shared$ = makeHttpRequest().pipe(
    share()
  );
  
  // 1回目の購読
  shared$.subscribe(val => console.log('共有購読者1:', val));
  
  // 2秒後に2回目の購読
  setTimeout(() => {
    shared$.subscribe(val => console.log('共有購読者2:', val));
  }, 2000);
}, 5000);
```

#### 実行結果
```sh
--- コールドObservable ---
HTTP呼び出し実行!
データ取得: 0
購読者1: 0
データ取得: 1
購読者1: 1
データ取得: 2
購読者1: 2
データ取得: 0 👈 2回目のHTTPリクエストが発生
購読者2: 0
データ取得: 1
購読者2: 1
データ取得: 2
購読者2: 2
--- ホットObservable (share使用) ---
HTTP呼び出し実行!
データ取得: 0
共有購読者1: 0
データ取得: 1
共有購読者1: 1
データ取得: 2
共有購読者1: 2
共有購読者2: 2 👈 値を受け取れない（ストリームはすでに完了している）
```

この変換は、複数コンポーネントで同じストリームを共有したい場合や、複数購読で副作用を避けたい場合に特に有用です。

### shareReplay()オペレーター

`shareReplay()`は`share()`の拡張版で、指定した数の過去の値をキャッシュして新しいサブスクライバーに再生します。

```ts
import { interval } from 'rxjs';
import { shareReplay, take, tap } from 'rxjs/operators';

// HTTP呼び出しをシミュレート
const request$ = interval(1000).pipe(
  take(3),
  tap(x => console.log('データ取得:', x)),
  // 最後の2つの値をキャッシュし再生
  shareReplay(2)
);

// 1回目の購読
request$.subscribe(val => console.log('購読者1:', val));

// 3.5秒後に2回目の購読（ストリーム完了後）
setTimeout(() => {
  console.log('購読者2開始 - キャッシュされた値を取得');
  request$.subscribe(val => console.log('購読者2:', val));
}, 3500);
```

#### 実行結果

```sh
データ取得: 0
購読者1: 0
データ取得: 1
購読者1: 1
データ取得: 2
購読者1: 2
購読者2開始 - キャッシュされた値を取得
購読者2: 1 👈　キャッシュされた値（最後の2つ）
購読者2: 2 👈　キャッシュされた値（最後の2つ）
```

### `multicast()`（柔軟だが複雑）

```ts
import { interval, Subject } from 'rxjs';
import { multicast, refCount, take } from 'rxjs/operators';

const source$ = interval(1000).pipe(take(3));

const multicasted$ = source$.pipe(
  multicast(() => new Subject()),
  refCount()
);

multicasted$.subscribe(val => console.log('購読者1:', val));
setTimeout(() => {
  multicasted$.subscribe(val => console.log('購読者2:', val));
}, 1500);
```

> [!NOTE]
> `multicast()` は柔軟ですが、`share()` の方が簡潔で実用的なケースが多いため、通常は `share()` を使用するのが推奨されます。

実際のアプリケーション開発では、コールドObservableをホットに変換して複数のサブスクライバー間でデータを共有したい場合があります。

## 使用するタイミング

### コールドObservableを使用するタイミング

- 各サブスクライバーが独自のデータセットを必要とする場合
- 新しく開始するプロセスやアクションを表現する場合
- 副作用の重複が問題にならない場合

### ホットObservableを使用するタイミング

- 複数のコンポーネント間でデータを共有する場合
- リソースを節約したい場合（例：HTTP呼び出しの回数を減らす）
- イベントストリームを表現する場合
- ステート管理やサービス間の通信

## 実践的な例: キャッシュとデータ共有

リアルワールドのアプリケーションでの使用例として、コンポーネント間でのデータ共有とキャッシュを実装してみましょう。

```ts
import { Observable, of, throwError } from 'rxjs';
import { catchError, shareReplay, tap, delay } from 'rxjs/operators';

// データサービスのシミュレーション
class UserService {
  private cache$: Observable<any> | null = null;
  
  // ユーザーデータを取得するメソッド
  getUsers(): Observable<any[]> {
    // キャッシュが存在する場合はそれを返す
    if (this.cache$) {
      console.log('キャッシュしたデータを返します');
      return this.cache$;
    }
    
    // 新しいリクエストを作成しキャッシュ
    console.log('新しいリクエストを実行します');
    this.cache$ = this.fetchUsersFromAPI().pipe(
      // エラー処理（キャッシュをクリア）
      catchError(err => {
        this.cache$ = null;
        return throwError(() => err);
      }),
      // リプレイ用にキャッシュ（期限なし）
      shareReplay(1)
    );
    
    return this.cache$;
  }
  
  // APIからユーザーデータを取得する（シミュレーション）
  private fetchUsersFromAPI(): Observable<any[]> {
    console.log('API呼び出し実行');
    // APIリクエストをシミュレート
    return of([
      { id: 1, name: '山田太郎' },
      { id: 2, name: '佐藤花子' }
    ]).pipe(
      delay(1000), // 1秒の遅延でAPIリクエストをシミュレート
      tap(() => console.log('APIからデータを受信'))
    );
  }
  
  // キャッシュをクリア
  clearCache(): void {
    this.cache$ = null;
    console.log('キャッシュをクリアしました');
  }
}

// 使用例
const userService = new UserService();

// 複数のコンポーネントがデータを要求
console.log('コンポーネント1: ユーザーデータを要求');
userService.getUsers().subscribe(users => {
  console.log('コンポーネント1: ユーザーデータを受信', users);
});

// 少し遅れて別のコンポーネントがデータを要求
setTimeout(() => {
  console.log('コンポーネント2: ユーザーデータを要求');
  userService.getUsers().subscribe(users => {
    console.log('コンポーネント2: ユーザーデータを受信', users);
  });
}, 2000);

// キャッシュをクリアして再度データを要求
setTimeout(() => {
  userService.clearCache();
  console.log('コンポーネント3: ユーザーデータを要求');
  userService.getUsers().subscribe(users => {
    console.log('コンポーネント3: ユーザーデータを受信', users);
  });
}, 4000);
```

#### 実行結果
```sh
コンポーネント1: ユーザーデータを要求
新しいリクエストを実行します
API呼び出し実行
APIからデータを受信
コンポーネント1: ユーザーデータを受信 (2) [{…}, {…}]
コンポーネント2: ユーザーデータを要求
キャッシュしたデータを返します
コンポーネント2: ユーザーデータを受信 (2) [{…}, {…}]
キャッシュをクリアしました
コンポーネント3: ユーザーデータを要求
新しいリクエストを実行します
API呼び出し実行
APIからデータを受信
コンポーネント3: ユーザーデータを受信 (2) [{…}, {…}]
```

この例では、`shareReplay(1)`を使用して最後のレスポンスをキャッシュし、複数のコンポーネントがデータを共有できるようにしています。これにより、不要なAPI呼び出しを防ぎ、パフォーマンスを向上させることができます。

## まとめ

コールドObservableとホットObservableを理解し、適切に使い分けることは、効率的なRxJSアプリケーションを構築するための重要なスキルです。特に、`share()`や`shareReplay()`などのオペレーターを活用することで、リソース消費を最適化しながら複数のコンシューマー間でデータを共有できます。


- Cold Observable: 「観測されて初めて動き出す」ストリーム  
（つまり、subscribe() されてから実行される）
- Hot Observable: 「すでに動いている」ストリーム  
（前から実行されている可能性がある）


ColdとHotの選択は、処理の性質（再実行されるべきか？共有されるべきか？）に依存します。意図的に使い分けることで、ストリームの設計がより柔軟になります。

設計判断を行う際は、以下の点を考慮しましょう。

- 複数のサブスクライバー間でデータを共有する必要があるか？
- 過去の値をキャッシュし、新しいサブスクライバーに提供する必要があるか？
- 副作用（HTTPリクエストなど）の重複をどのように管理するか？

これらの考慮事項を元に、適切なObservableの種類とオペレーターを選択することで、効率的で堅牢なリアクティブアプリケーションを構築できます。