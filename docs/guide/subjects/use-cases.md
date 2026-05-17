---
description: Subjectファミリー（Subject、BehaviorSubject、ReplaySubject、AsyncSubject）を活用した状態管理、イベントバス、グローバル通知システム、データキャッシュ、リアクティブフォーム管理など、実践的なユースケースを豊富なTypeScriptコード例で解説します。各Subjectタイプの特性を理解し、適切な場面で活用できるようになります。
---

# Subjectのユースケース

RxJSのSubjectは様々な実用的なシナリオで活用できます。ここでは、Subjectファミリー（Subject、BehaviorSubject、ReplaySubject、AsyncSubject）の実践的な使用例を紹介し、それぞれが最適な場面について解説します。

## 状態管理パターン

### シンプルなストアの実装

`BehaviorSubject` を使って、アプリケーションの状態を保持・更新・購読できるシンプルなストアを実装します。

```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// 初期状態
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // BehaviorSubjectで状態を管理
  private state$ = new BehaviorSubject<AppState>(initialState);
  
  // 状態の読み取り用メソッド
  getState() {
    return this.state$.getValue();
  }
  
  // 指定したプロパティをObservableとして取得
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }
  
  // 状態の更新
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }
  
  // 状態をObservableとして公開
  get state() {
    return this.state$.asObservable();
  }
}

// 使用例
const store = new Store();

// 状態を監視
store.select('user').subscribe(user => {
  console.log('ユーザー状態変更:', user?.name, user?.role);
});

// テーマ変更を監視
store.select('theme').subscribe(theme => {
  console.log('テーマ変更:', theme);
  document.body.className = theme; // UIに反映
});

// 状態の更新
store.setState({ user: { name: '山田太郎', role: 'admin' } });
store.setState({ theme: 'dark' });
```

#### 実行結果
```sh
ユーザー状態変更: undefined undefined
テーマ変更: light
ユーザー状態変更: 山田太郎 admin
テーマ変更: light
ユーザー状態変更: 山田太郎 admin
テーマ変更: dark
```

このパターンは小規模なアプリケーションや、NgRxやReduxのような大規模な状態管理ライブラリを使わない場合に便利です。

## コンポーネント間通信

### イベントバスの実装

通知タイプごとに異なるデータ型を扱える `Subject` ベースのイベントバスを実装し、コンポーネント間通信を行います。

```ts
import { Subject } from 'rxjs';
import { filter, map } from 'rxjs';

type EventPayloadMap = {
  USER_LOGIN: { username: string; timestamp: number };
  DATA_UPDATED: any;
  NOTIFICATION: string;
};

// イベント型の定義
type EventType = keyof EventPayloadMap;

interface AppEvent<K extends EventType> {
  type: K;
  payload: EventPayloadMap[K];
}

// イベントバスサービス
class EventBusService {
  private eventSubject = new Subject<AppEvent<unknown>>();

  emit<K extends EventType>(type: K, payload: EventPayloadMap[K]): void {
    this.eventSubject.next({ type, payload });
  }

  // 特定タイプのイベントを購読
  on<K extends EventType>(type: K) {
    return this.eventSubject.pipe(
      filter((event): event is AppEvent<K> => event.type === type),
      map((event) => event.payload)
    );
  }
}
// 使用例）コンポーネント間通信
const eventBus = new EventBusService();

// ヘッダーコンポーネント（通知を表示）
eventBus.on('NOTIFICATION').subscribe((message) => {
  console.log('ヘッダー: 通知を表示:', message);
});

// ユーザーコンポーネント（ログイン状態を監視）
eventBus.on('USER_LOGIN').subscribe((user) => {
  console.log('ユーザーコンポーネント: ログイン検出:', user.username);
});

// 設定コンポーネント（データ更新を監視）
eventBus.on('DATA_UPDATED').subscribe((data) => {
  console.log('設定コンポーネント: データ更新:', data);
});

// イベント発行
eventBus.emit('USER_LOGIN', { username: 'user123', timestamp: Date.now() });
eventBus.emit('NOTIFICATION', '新しいメッセージがあります');
```

#### 実行結果
```sh
ユーザーコンポーネント: ログイン検出: user123
ヘッダー: 通知を表示: 新しいメッセージがあります
```

イベントバスパターンは、疎結合なコンポーネント間通信を実現する優れた方法です。特に階層の離れたコンポーネント間の通信に適しています。

> [!CAUTION]
> 💡 実アプリケーションでは、購読解除（`unsubscribe()`）を行わないとメモリリークにつながる可能性があります。`takeUntil()` などを使った解除処理も検討してください。

## APIデータキャッシング

### リクエスト結果の共有とキャッシュ

`AsyncSubject` を使って、HTTPリクエストのような一度だけ発行されるデータの共有とキャッシュを実現します。

```ts
import { Observable, AsyncSubject, of, throwError } from 'rxjs';
import { tap, catchError, delay } from 'rxjs';

class ApiCacheService {
  private cache = new Map<string, AsyncSubject<any>>();

  fetchData<T>(url: string): Observable<T> {
    // キャッシュに存在する場合はそれを返す
    if (this.cache.has(url)) {
      console.log(`キャッシュからデータ取得: ${url}`);
      return this.cache.get(url)!.asObservable() as Observable<T>;
    }

    // キャッシュがない場合は新しいリクエストを作成
    console.log(`APIリクエスト実行: ${url}`);
    const subject = new AsyncSubject<T>();
    this.cache.set(url, subject);

    // APIリクエストをシミュレート
    this.makeRequest<T>(url)
      .pipe(
        tap((data) => {
          subject.next(data);
          subject.complete();
        }),
        catchError((error: unknown) => {
          // エラー時はキャッシュから削除
          this.cache.delete(url);
          subject.error(error);
          return throwError(() => error);
        })
      )
      .subscribe();

    return subject.asObservable();
  }

  // 実際のAPIリクエスト処理
  private makeRequest<T>(url: string): Observable<T> {
    // 実際のアプリではfetchやHTTPクライアントを使用
    return of({
      data: 'サンプルデータ',
      timestamp: Date.now(),
    } as unknown as T).pipe(
      tap(() => console.log('API応答受信')),
      // ランダムな遅延をシミュレート
      delay(Math.random() * 1000 + 500)
    );
  }

  // キャッシュをクリア
  clearCache(url?: string): void {
    if (url) {
      this.cache.delete(url);
    } else {
      this.cache.clear();
    }
    console.log('キャッシュをクリアしました');
  }
}

// 使用例
const apiCache = new ApiCacheService();

// 複数のコンポーネントが同じAPIデータを要求
apiCache.fetchData('/api/products').subscribe((data) => {
  console.log('コンポーネント1: データ受信', data);
});

// 少し後に別のコンポーネントも同じデータを要求（キャッシュから取得）
setTimeout(() => {
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('コンポーネント2: データ受信', data);
  });
}, 1000);

// キャッシュクリア後に再度リクエスト
setTimeout(() => {
  apiCache.clearCache();
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('コンポーネント3: データ受信（キャッシュクリア後）', data);
  });
}, 2000);
```

#### 実行結果
```sh
APIリクエスト実行: /api/products
API応答受信
コンポーネント1: データ受信 {data: 'サンプルデータ', timestamp: 1745405703582}
キャッシュからデータ取得: /api/products
コンポーネント2: データ受信 {data: 'サンプルデータ', timestamp: 1745405703582}
キャッシュをクリアしました
APIリクエスト実行: /api/products
API応答受信
コンポーネント3: データ受信（キャッシュクリア後） {data: 'サンプルデータ', timestamp: 1745405705585}
```

AsyncSubjectを使ったこのパターンは、完了時の最後の値のみが重要なAPIリクエストに最適です。また、同一リクエストの重複発行を防止します。

> [!TIP]
> 💡 `AsyncSubject` は `error()` が呼び出された場合、値は発行されず、`error` のみが通知されるため注意が必要です。


## フォーム管理

`BehaviorSubject` を使って、リアクティブフォームの現在値とバリデーション状態を管理します。
### フォーム値の双方向バインディング

```ts
import { BehaviorSubject } from 'rxjs';
import { debounceTime, distinctUntilChanged } from 'rxjs';

interface UserForm {
  name: string;
  email: string;
  age: number;
}

class ReactiveForm {
  // 初期値を持つBehaviorSubject
  private formSubject = new BehaviorSubject<UserForm>({
    name: '',
    email: '',
    age: 0
  });
  
  // 公開用のObservable
  formValues$ = this.formSubject.asObservable();
  
  // バリデーション結果
  private validSubject = new BehaviorSubject<boolean>(false);
  valid$ = this.validSubject.asObservable();
  
  constructor() {
    // 値変更時にバリデーション実行
    this.formValues$.pipe(
      debounceTime(300),
      distinctUntilChanged((prev, curr) => JSON.stringify(prev) === JSON.stringify(curr))
    ).subscribe(form => {
      this.validateForm(form);
    });
  }
  
  // フィールド値の更新
  updateField<K extends keyof UserForm>(field: K, value: UserForm[K]) {
    const currentForm = this.formSubject.getValue();
    this.formSubject.next({
      ...currentForm,
      [field]: value
    });
  }
  
  // フォームの取得
  getForm(): UserForm {
    return this.formSubject.getValue();
  }
  
  // バリデーション
  private validateForm(form: UserForm) {
    const isValid = 
      form.name.length > 0 && 
      form.email.includes('@') &&
      form.age > 0;
      
    this.validSubject.next(isValid);
  }
  
  // フォーム送信
  submit() {
    if (this.validSubject.getValue()) {
      console.log('フォーム送信:', this.getForm());
      // APIリクエストなど
    } else {
      console.error('フォームが無効です');
    }
  }
}

// 使用例
const form = new ReactiveForm();

// フォーム値を監視
form.formValues$.subscribe(values => {
  console.log('フォーム値変更:', values);
  // UIの更新処理など
});

// バリデーション状態を監視
form.valid$.subscribe(isValid => {
  console.log('フォーム有効性:', isValid);
  // 送信ボタンの有効/無効切り替えなど
});

// ユーザー入力をシミュレート
form.updateField('name', '山田太郎');
form.updateField('email', 'yamada@example.com');
form.updateField('age', 30);

// フォーム送信
form.submit();
```

#### 実行結果
```sh
フォーム値変更: {name: '', email: '', age: 0}
フォーム有効性: false
フォーム値変更: {name: '山田太郎', email: '', age: 0}
フォーム値変更: {name: '山田太郎', email: 'yamada@example.com', age: 0}
フォーム値変更: {name: '山田太郎', email: 'yamada@example.com', age: 30}
フォームが無効です
submit @ 
（匿名） @ このエラーを分析
フォーム有効性: true
```


このパターンは、リアクティブフォームの実装に特に有用です。BehaviorSubjectが現在の値を常に保持するため、フォームの状態管理に最適です。

## ロギングと履歴

`ReplaySubject` を使って、過去の操作履歴を保持・再表示できるログ管理機構を構築します。
### 操作履歴の管理

```ts
import { Observable, ReplaySubject } from 'rxjs';
import { tap } from 'rxjs';

interface LogEntry {
  action: string;
  timestamp: number;
  data?: any;
}

class ActivityLogger {
  // 最新10件のログを保持
  private logSubject = new ReplaySubject<LogEntry>(10);
  logs$ = this.logSubject.asObservable();
  
  // ログエントリを追加
  log(action: string, data?: any) {
    const entry: LogEntry = {
      action,
      timestamp: Date.now(),
      data
    };
    
    this.logSubject.next(entry);
    console.log(`ログ記録: ${action}`, data);
  }
  
  // 別のObservableをラップしてログを記録
  wrapWithLogging<T>(source$: Observable<T>, actionName: string): Observable<T> {
    return source$.pipe(
      tap(data => this.log(actionName, data))
    );
  }
}

// 使用例
const logger = new ActivityLogger();

// ログを監視（UIに表示するなど）
logger.logs$.subscribe(log => {
  const time = new Date(log.timestamp).toLocaleTimeString();
  console.log(`[${time}] ${log.action}`);
});

// 様々な操作をログに記録
logger.log('アプリケーション起動');
logger.log('ユーザーログイン', { userId: 'user123' });

// 少し後に、新しいコンポーネントが過去のログを含めて購読開始
setTimeout(() => {
  console.log('--- 履歴ビューアが過去のログを含めて表示 ---');
  logger.logs$.subscribe(log => {
    const time = new Date(log.timestamp).toLocaleTimeString();
    console.log(`履歴: [${time}] ${log.action}`);
  });
  
  // さらにログ追加
  logger.log('データ更新', { itemId: 456 });
}, 1000);
```
#### 実行結果
```sh
[19:58:40] アプリケーション起動
ログ記録: アプリケーション起動 undefined
[19:58:40] ユーザーログイン
ログ記録: ユーザーログイン {userId: 'user123'}
--- 履歴ビューアが過去のログを含めて表示 ---
履歴: [19:58:40] アプリケーション起動
履歴: [19:58:40] ユーザーログイン
[19:58:41] データ更新
履歴: [19:58:41] データ更新
ログ記録: データ更新 {itemId: 456}
```

ReplaySubjectを使用すると、新しい購読者に過去のログエントリを提供できるため、履歴管理に最適です。ユーザー操作の追跡やデバッグ情報の収集に役立ちます。

> [!IMPORTANT]
> ⚠️ `ReplaySubject` にバッファサイズを指定しない場合、すべての値がメモリに保持され続けるため、大量データや長時間動作するアプリでは注意が必要です。

## 非同期処理の管理

`Subject` と `BehaviorSubject` を使って、複数タスクの進捗状況とアクティブ状態をリアルタイムに管理します。
### 長時間実行タスクの進捗管理

```ts
import { Subject, BehaviorSubject } from 'rxjs';

interface TaskProgress {
  taskId: string;
  progress: number; // 0-100
  status: 'pending' | 'running' | 'completed' | 'error';
  message?: string;
}

class TaskManager {
  // タスク進捗の通知
  private progressSubject = new Subject<TaskProgress>();
  progress$ = this.progressSubject.asObservable();
  
  // 現在実行中のタスク
  private activeTasksSubject = new BehaviorSubject<string[]>([]);
  activeTasks$ = this.activeTasksSubject.asObservable();
  
  // タスクを開始
  startTask(taskId: string, taskFn: (update: (progress: number) => void) => Promise<any>) {
    // アクティブタスクリストに追加
    const currentTasks = this.activeTasksSubject.getValue();
    this.activeTasksSubject.next([...currentTasks, taskId]);
    
    // 初期進捗通知
    this.progressSubject.next({
      taskId,
      progress: 0,
      status: 'running'
    });
    
    // 進捗更新用の関数
    const updateProgress = (progress: number) => {
      this.progressSubject.next({
        taskId,
        progress,
        status: 'running'
      });
    };
    
    // タスク実行
    return taskFn(updateProgress)
      .then(result => {
        // 完了通知
        this.progressSubject.next({
          taskId,
          progress: 100,
          status: 'completed'
        });
        return result;
      })
      .catch(error => {
        // エラー通知
        this.progressSubject.next({
          taskId,
          progress: 0,
          status: 'error',
          message: error.message
        });
        throw error;
      })
      .finally(() => {
        // アクティブタスクリストから削除
        const tasks = this.activeTasksSubject.getValue();
        this.activeTasksSubject.next(tasks.filter(id => id !== taskId));
      });
  }
}

// 使用例
const taskManager = new TaskManager();

// 進捗バーUIなどで進捗を表示
taskManager.progress$.subscribe(progress => {
  console.log(`タスク ${progress.taskId}: ${progress.progress}% - ${progress.status}`);
  
  // UI更新コード
  // progressBar.setValue(progress.progress);
  // statusLabel.setText(progress.status);
});

// アクティブタスク数を表示
taskManager.activeTasks$.subscribe(tasks => {
  console.log(`実行中のタスク数: ${tasks.length}`);
});

// 長時間実行タスクのシミュレーション
taskManager.startTask('file-upload', (update) => {
  return new Promise((resolve) => {
    let progress = 0;
    
    // 進捗シミュレーション
    const interval = setInterval(() => {
      progress += 10;
      update(progress);
      
      if (progress >= 100) {
        clearInterval(interval);
        resolve('アップロード完了');
      }
    }, 500);
  });
});
```

#### 実行結果
```sh
実行中のタスク数: 0
実行中のタスク数: 1
タスク file-upload: 0% - running
タスク file-upload: 10% - running
タスク file-upload: 20% - running
タスク file-upload: 30% - running
タスク file-upload: 40% - running
タスク file-upload: 50% - running
タスク file-upload: 60% - running
タスク file-upload: 70% - running
タスク file-upload: 80% - running
タスク file-upload: 90% - running
タスク file-upload: 100% - running
タスク file-upload: 100% - completed
実行中のタスク数: 0
```

このパターンでは、Subjectを使って長時間実行タスクの進捗状況をリアルタイムで通知します。ファイルアップロード、データ処理、バックグラウンド操作などの進捗表示に適しています。

## リアルタイム更新

WebSocket の接続状態・受信メッセージ・再接続制御を、複数の Subject を使って管理します。
### WebSocketストリームの管理

```ts
import { Subject, BehaviorSubject, timer, Observable } from 'rxjs';
import { takeUntil, filter, map } from 'rxjs';

interface WebSocketMessage {
  type: string;
  data: any;
}

class WebSocketService {
  private socket: WebSocket | null = null;
  private url: string;

  // 接続状態
  private connectionStatusSubject = new BehaviorSubject<boolean>(false);
  connectionStatus$ = this.connectionStatusSubject.asObservable();

  // メッセージストリーム
  private messagesSubject = new Subject<WebSocketMessage>();
  messages$ = this.messagesSubject.asObservable();

  // 接続終了用Subject
  private destroySubject = new Subject<void>();

  constructor(url: string) {
    this.url = url;
  }

  // WebSocket接続を開始
  connect(): void {
    if (this.socket) {
      return; // 既に接続済み
    }

    this.socket = new WebSocket(this.url);

    // イベントハンドラの設定
    this.socket.addEventListener('open', () => {
      console.log('WebSocket接続確立');
      this.connectionStatusSubject.next(true);
    });

    this.socket.addEventListener('message', (event) => {
      try {
        const message = JSON.parse(event.data) as WebSocketMessage;
        this.messagesSubject.next(message);
      } catch (e) {
        console.error('メッセージ解析エラー:', e);
      }
    });

    this.socket.addEventListener('close', () => {
      console.log('WebSocket接続終了');
      this.connectionStatusSubject.next(false);
      this.socket = null;

      // 自動再接続
      this.reconnect();
    });

    this.socket.addEventListener('error', (error) => {
      console.error('WebSocketエラー:', error);
      this.connectionStatusSubject.next(false);
    });
  }

  // 再接続ロジック
  private reconnect(): void {
    // destroyが呼ばれていなければ再接続
    timer(3000)
      .pipe(takeUntil(this.destroySubject))
      .subscribe(() => {
        console.log('WebSocket再接続試行...');
        this.connect();
      });
  }

  // メッセージ送信
  send(type: string, data: any): void {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      const message: WebSocketMessage = { type, data };
      this.socket.send(JSON.stringify(message));
    } else {
      console.error('WebSocketが接続されていません');
    }
  }

  // 特定タイプのメッセージのみを取得
  getMessagesOfType<T>(type: string): Observable<T> {
    return this.messages$.pipe(
      filter((msg) => msg.type === type),
      map((msg) => msg.data as T)
    );
  }

  // 接続解除
  disconnect(): void {
    this.destroySubject.next();
    this.destroySubject.complete();

    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
  }
}

// 使用例
const wsService = new WebSocketService('wss://echo.websocket.org');

// 接続状態を監視
wsService.connectionStatus$.subscribe((isConnected) => {
  console.log('接続状態:', isConnected ? 'オンライン' : 'オフライン');
  // UI更新など
});

// すべてのメッセージを監視
wsService.messages$.subscribe((message) => {
  console.log('受信メッセージ:', message);
});

// 特定タイプのメッセージのみ監視
wsService
  .getMessagesOfType<{ price: number }>('stock-update')
  .subscribe((stockData) => {
    console.log(`株価更新: ${stockData.price}`);
  });

// 接続開始
wsService.connect();

// メッセージ送信
setTimeout(() => {
  wsService.send('chat-message', { text: 'こんにちは！' });
}, 1000);

// アプリケーション終了時
// wsService.disconnect();
```

#### 実行結果
```sh
接続状態: オフライン
WebSocket接続確立
接続状態: オンライン
メッセージ解析エラー: SyntaxError: Unexpected token 'R', "Request se"... is not valid JSON
  at JSON.parse (<anonymous>)
  at WebSocket.<anonymous> (:30)
（匿名） @ このエラーを分析
受信メッセージ: {type: 'chat-message', data: {…}}
```

このWebSocket管理パターンは、リアルタイム通信を必要とするアプリケーションに最適です。Subjectを使って接続状態とメッセージフローを管理し、複数のコンポーネントで共有できます。

## Subjectの選び方の指針

| ユースケース | 推奨Subject | 解説 |
|--------------|-------------|------|
| イベント通知・通信 | `Subject` | シンプルな一方向通信に適する |
| 現在値の保持・状態管理 | `BehaviorSubject` | 初期値が必要で、最新値を常に取得可能 |
| 履歴付きストリーム・ログ | `ReplaySubject` | 過去の値も購読者に提供できる |
| 最終値の一括提供・レスポンス共有 | `AsyncSubject` | 完了時に最後の値のみを通知 |

> 💡 変数名の末尾に `$` を付けるのは、Observableであることを表すRxJSの一般的な命名慣習です。

## まとめ

RxJSのSubjectファミリーは、以下のような様々なユースケースに対応する強力なツールです。

- **BehaviorSubject**: 状態管理、フォーム管理、現在値の表示
- **Subject**: イベント通知、コンポーネント間通信
- **ReplaySubject**: 履歴管理、操作ログ、遅延参加コンポーネント
- **AsyncSubject**: APIレスポンスのキャッシュ、計算結果の共有

これらのパターンを適切に組み合わせることで、リアクティブで保守性の高いアプリケーションを構築できます。特にメモリリークを防ぐために、適切なタイミングでの購読解除を忘れないよう注意しましょう。