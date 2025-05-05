# Subjectの種類

RxJSには基本的な`Subject`の他に、特定のユースケースに特化した複数の派生クラスが用意されています。それぞれ異なる動作特性を持ち、適切な状況で活用することでより効果的なリアクティブプログラミングが可能になります。

ここでは、主要な4種類のSubjectとその特性、活用シーンを詳しく解説します。

## 基本となる4種類のSubject

| 種類 | 特徴 | 主なユースケース |
|------|------|----------------|
| [`Subject`](#subject) | 最もシンプルなSubject<br>購読後の値のみを受け取る | イベント通知、マルチキャスト |
| [`BehaviorSubject`](#behaviorsubject) | 最新の値を保持し、新規購読時に即座に提供 | 状態管理、UIコンポーネントの現在値 |
| [`ReplaySubject`](#replaysubject) | 指定した数の過去の値を新規購読者に再生 | 操作履歴、最近の更新情報 |
| [`AsyncSubject`](#asyncsubject) | 完了時の最後の値のみを発行 | HTTP/APIリクエストの結果 |

## 標準`Subject` {#subject}

[📘 RxJS公式: Subject](https://rxjs.dev/api/index/class/Subject)

最もシンプルなタイプのSubjectで、購読後に発生した値のみを受け取ります。

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

// 初期値はなく、購読時に何も受け取らない
subject.subscribe(value => console.log('Observer 1:', value));

subject.next(1);
subject.next(2);

// 2回目の購読（購読後の値のみ受け取る）
subject.subscribe(value => console.log('Observer 2:', value));

subject.next(3);
subject.complete();
```

#### 実行結果
```
Observer 1: 1
Observer 1: 2
Observer 1: 3
Observer 2: 3
```

## BehaviorSubject  {#behaviorsubject}

[📘 RxJS公式: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

初期値を必要とし、常に最新の値を保持します。  
新しい購読者は購読時に直ちに最新の値を受け取ります。

```ts
import { BehaviorSubject } from 'rxjs';

// 初期値 0 で作成
const behaviorSubject = new BehaviorSubject<number>(0);

// 初期値を即座に受け取る
behaviorSubject.subscribe(value => console.log('Observer 1:', value));

behaviorSubject.next(1);
behaviorSubject.next(2);

// 2回目の購読（最新値2を即座に受け取る）
behaviorSubject.subscribe(value => console.log('Observer 2:', value));

behaviorSubject.next(3);
behaviorSubject.complete();
```

#### 実行結果
```
Observer 1: 0
Observer 1: 1
Observer 1: 2
Observer 2: 2
Observer 1: 3
Observer 2: 3
```

### BehaviorSubjectの活用例

#### ユーザー認証状態の管理結果

```ts
import { BehaviorSubject } from 'rxjs';

interface User {
  id: string;
  name: string;
}

// 初期値はnull（未ログイン状態）
const currentUser$ = new BehaviorSubject<User | null>(null);

// コンポーネント等でログイン状態を監視
currentUser$.subscribe(user => {
  if (user) {
    console.log(`ログイン中: ${user.name}`);
  } else {
    console.log('未ログイン状態');
  }
});

// ログイン処理
function login(user: User) {
  currentUser$.next(user);
}

// ログアウト処理
function logout() {
  currentUser$.next(null);
}

// 使用例
console.log('アプリケーション起動');
// → 未ログイン状態

login({ id: 'user123', name: '山田太郎' });
// → ログイン中: 山田太郎

logout();
// → 未ログイン状態
```

#### 実行結果
```sh
未ログイン状態
アプリケーション起動
ログイン中: 山田太郎
未ログイン状態
```

## `ReplaySubject` {#replaysubject}
[📘 RxJS公式: ReplaySubject](https://rxjs.dev/api/index/class/ReplaySubject)

指定した数の過去の値を記憶し、新しい購読者に再送信します。  
バッファサイズとタイムウィンドウを設定可能です。

```ts
import { ReplaySubject } from 'rxjs';

// 直近3つの値をバッファリング
const replaySubject = new ReplaySubject<number>(3);

replaySubject.next(1);
replaySubject.next(2);
replaySubject.next(3);
replaySubject.next(4);

// 購読開始（直近3つの値2,3,4を受け取る）
replaySubject.subscribe(value => console.log('Observer 1:', value));

replaySubject.next(5);

// 2回目の購読（直近3つの値3,4,5を受け取る）
replaySubject.subscribe(value => console.log('Observer 2:', value));

replaySubject.complete();
```

#### 実行結果
```
Observer 1: 2
Observer 1: 3
Observer 1: 4
Observer 1: 5
Observer 2: 3
Observer 2: 4
Observer 2: 5
```

### タイムウィンドウ付きReplaySubject

時間ベースでバッファリングすることも可能です。

```ts
import { ReplaySubject } from 'rxjs';

// 最大5つの値、かつ500ms以内の値をバッファリング
const timeWindowSubject = new ReplaySubject<number>(5, 500);

timeWindowSubject.next(1);

setTimeout(() => {
  timeWindowSubject.next(2);
  
  // 1000ms後に購読（500msの時間枠を超えたので1は受け取らない）
  setTimeout(() => {
    timeWindowSubject.subscribe(value => console.log('受信:', value));
  }, 1000);
}, 100);
```

#### 実行結果
```
受信: 2
```

### ReplaySubjectの活用例

#### 最近の検索履歴の管理

```ts
import { ReplaySubject } from 'rxjs';

// 最新5件の検索クエリを保持
const searchHistory$ = new ReplaySubject<string>(5);

// 検索実行関数
function search(query: string) {
  console.log(`検索実行: ${query}`);
  searchHistory$.next(query);
  // 実際の検索処理...
}

// 検索履歴を表示するコンポーネント
function showSearchHistory() {
  console.log('--- 検索履歴 ---');
  searchHistory$.subscribe(query => {
    console.log(query);
  });
}

// 使用例
search('TypeScript');
search('RxJS');
search('Angular');
search('React');

showSearchHistory();
// 最新5件（この場合は4件）の検索履歴を表示
```

#### 実行結果
```sh
検索実行: TypeScript
検索実行: RxJS
検索実行: Angular
検索実行: React
--- 検索履歴 ---
TypeScript
RxJS
Angular
React
```

## `AsyncSubject` {#asyncsubject}
[📘 RxJS公式: AsyncSubject](https://rxjs.dev/api/index/class/AsyncSubject)

完了時に最後の値のみを発行するSubjectです。完了前の値は発行されません。

```ts
import { AsyncSubject } from 'rxjs';

const asyncSubject = new AsyncSubject<number>();

asyncSubject.subscribe(value => console.log('Observer 1:', value));

asyncSubject.next(1);
asyncSubject.next(2);
asyncSubject.next(3);

// 購読のタイミングに関わらず、最後の値のみ受け取る
asyncSubject.subscribe(value => console.log('Observer 2:', value));

asyncSubject.next(4);
asyncSubject.complete(); // 完了時に最後の値(4)が発行される
```

#### 実行結果
```
Observer 1: 4
Observer 2: 4
```

### AsyncSubjectの活用例

#### APIリクエストの結果共有結果

```ts
import { AsyncSubject } from 'rxjs';

interface ApiResponse {
  data: any;
  status: number;
}

function fetchData(url: string) {
  const subject = new AsyncSubject<ApiResponse>();
  
  // APIリクエストをシミュレート
  console.log(`APIリクエスト: ${url}`);
  setTimeout(() => {
    const response = {
      data: { id: 1, name: 'サンプルデータ' },
      status: 200
    };
    
    subject.next(response);
    subject.complete();
  }, 1000);
  
  return subject;
}

// 使用例
const data$ = fetchData('/api/users/1');

// 複数のコンポーネントが同じリクエスト結果を共有できる
data$.subscribe(response => {
  console.log('コンポーネント1:', response.data);
});

setTimeout(() => {
  data$.subscribe(response => {
    console.log('コンポーネント2:', response.data);
  });
}, 1500); // 完了後でも値を受け取れる
```

#### 実行結果
```sh
APIリクエスト: /api/users/1
コンポーネント1: {id: 1, name: 'サンプルデータ'}
コンポーネント2: {id: 1, name: 'サンプルデータ'}
```

## 各Subjectの比較と選択ガイド

各Subjectのタイプを選択する際に役立つポイントをまとめます。

### Subjectの選び方

|type|選択基準|
|---|---|
|`Subject`|単純なイベント通知やマルチキャスト配信に使用|
|`BehaviorSubject`|<li>常に初期値が必要なケース </li><li>現在の状態を表すデータ（ユーザー状態、設定、フラグなど） </li><li>UIコンポーネントの現在値</li>|
|`ReplaySubject`|<li>直近の操作履歴を保持する必要がある場合 </li><li>後から参加した購読者に過去のデータを提供したい場合  </li><li>バッファされたデータストリーム</li>|
|`AsyncSubject`|<li>最終結果のみが重要な場合（APIレスポンスなど） </li><li>途中経過は不要で完了時の値だけを共有したい場合</li>|

### 選択の判断フロー

1. 完了時の最後の値だけが必要 ⇨ `AsyncSubject`
2. 直近N個の値が必要 ⇨ `ReplaySubject`
3. 現在の状態/値が常に必要 ⇨ `BehaviorSubject`
4. それ以外（純粋なイベント通知など） ⇨ `Subject`

## アプリケーション設計における活用パターン

### モジュール間通信の例

```ts
// アプリケーション全体の状態管理サービス
class AppStateService {
  // 現在のユーザー（初期値必須なのでBehaviorSubject）
  private userSubject = new BehaviorSubject<User | null>(null);
  // 読み取り専用のObservableとして公開
  readonly user$ = this.userSubject.asObservable();
  
  // 通知（単純なイベント通知なのでSubject）
  private notificationSubject = new Subject<Notification>();
  readonly notifications$ = this.notificationSubject.asObservable();
  
  // 最近の検索（履歴が必要なのでReplaySubject）
  private searchHistorySubject = new ReplaySubject<string>(10);
  readonly searchHistory$ = this.searchHistorySubject.asObservable();
  
  // API呼び出し結果キャッシュ（最終結果のみ必要なのでAsyncSubject）
  private readonly apiCaches = new Map<string, AsyncSubject<any>>();
  
  // メソッド例
  setUser(user: User | null) {
    this.userSubject.next(user);
  }
  
  notify(notification: Notification) {
    this.notificationSubject.next(notification);
  }
  
  addSearch(query: string) {
    this.searchHistorySubject.next(query);
  }
  
  // API結果のキャッシュ
  fetchData(url: string): Observable<any> {
    if (!this.apiCaches.has(url)) {
      const subject = new AsyncSubject<any>();
      this.apiCaches.set(url, subject);
      
      // 実際のAPI呼び出し
      fetch(url)
        .then(res => res.json())
        .then(data => {
          subject.next(data);
          subject.complete();
        })
        .catch(err => {
          subject.error(err);
        });
    }
    
    return this.apiCaches.get(url)!.asObservable();
  }
}
```

## まとめ

RxJSのSubjectは、さまざまなユースケースに対応できる強力なツールです。各タイプの特性を理解し、適切に活用することで、効率的でメンテナンス性の高いリアクティブアプリケーションを構築できます。

- `Subject`: 最もシンプルで、基本的なマルチキャスト機能を提供
- `BehaviorSubject`: 現在の状態を常に保持し、新規購読者にすぐに提供
- `ReplaySubject`: 直近の値の履歴を保持し、後から参加した購読者にも提供
- `AsyncSubject`: 完了時の最終値のみを発行

状態管理、イベント通知、データ共有などあらゆる場面で適切な`Subject`を選択することが、効率的なリアクティブプログラミングの鍵となります。