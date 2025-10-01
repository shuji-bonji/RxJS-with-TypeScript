---
description: RxJSのマルチキャスティングの仕組みを詳しく解説します。Subjectを使った基本パターン、shareとshareReplayオペレーターの使い分け、APIリクエストの共有やキャッシュ、状態管理など実践的な設計パターンを紹介します。
---

# マルチキャスティングの仕組み

マルチキャスティングは、一つのObservableからのデータストリームを複数の購読者（Observer）に効率的に配信する手法です。  
RxJSでは、Subjectやオペレーターによってこれを実現できます。

## マルチキャスティングとは

通常のObservable（Cold Observable）は、購読されるたびに新しいデータストリームを作成します。これは、複数の購読者がいる場合、同じ処理が複数回実行されることを意味します。

マルチキャスティングを使用すると、データソースを一度だけ実行し、その結果を複数の購読者に配信できます。これは特に以下の場合に重要です。

- HTTP/APIリクエストを重複して呼び出したくない
- 高コストな操作（計算や副作用）を一度だけ実行したい
- アプリケーション状態を複数のコンポーネントで共有する

## マルチキャスティングの基本パターン

### Subjectを使用した基本的なマルチキャスト

```ts
import { Observable, Subject } from 'rxjs';
import { tap } from 'rxjs/operators';

// データソース（コールドObservable）
function createDataSource(): Observable<number> {
  return new Observable<number>(observer => {
    console.log('データソース: 接続');
    // データ生成ロジック（高コストな操作を想定）
    const id = setInterval(() => {
      const value = Math.round(Math.random() * 100);
      console.log(`データソース: 値を生成 -> ${value}`);
      observer.next(value);
    }, 1000);
    
    // クリーンアップ関数
    return () => {
      console.log('データソース: 切断');
      clearInterval(id);
    };
  });
}

// マルチキャスト実装
function multicast() {
  // 元のデータソース
  const source$ = createDataSource().pipe(
    tap(value => console.log(`ソース処理: ${value}`))
  );
  
  // マルチキャスト用のSubject
  const subject = new Subject<number>();
  
  // ソースをSubjectに接続
  const subscription = source$.subscribe(subject);
  
  // 複数の購読者がSubjectを購読
  console.log('Observer 1 購読開始');
  const subscription1 = subject.subscribe(value => console.log(`Observer 1: ${value}`));
  
  // 3秒後に別の購読者を追加
  setTimeout(() => {
    console.log('Observer 2 購読開始');
    const subscription2 = subject.subscribe(value => console.log(`Observer 2: ${value}`));
    
    // 5秒後に全ての購読を終了
    setTimeout(() => {
      console.log('全ての購読を終了');
      subscription.unsubscribe();
      subscription1.unsubscribe();
      subscription2.unsubscribe();
    }, 5000);
  }, 3000);
}

// 実行
multicast();
```

#### 実行結果
```
データソース: 接続
Observer 1 購読開始
データソース: 値を生成 -> 71
ソース処理: 71
Observer 1: 71
データソース: 値を生成 -> 79
ソース処理: 79
Observer 1: 79
データソース: 値を生成 -> 63
ソース処理: 63
Observer 1: 63
Observer 2 購読開始
データソース: 値を生成 -> 49
ソース処理: 49
Observer 1: 49
Observer 2: 49
データソース: 値を生成 -> 94
ソース処理: 94
Observer 1: 94
Observer 2: 94
データソース: 値を生成 -> 89
ソース処理: 89
Observer 1: 89
Observer 2: 89
データソース: 値を生成 -> 10
ソース処理: 10
Observer 1: 10
Observer 2: 10
データソース: 値を生成 -> 68
ソース処理: 68
Observer 1: 68
Observer 2: 68
全ての購読を終了
データソース: 切断
```

## マルチキャスト演算子

RxJSでは、マルチキャスティングを実装するための専用演算子が用意されています。

### `share()` 演算子
[📘 RxJS公式: share()](https://rxjs.dev/api/index/function/share)

最も簡単にマルチキャストを実装できる演算子です。  
内部的には`multicast()`と`refCount()`を組み合わせています。

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs/operators';

// インターバルでカウントするObservable
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`ソース: ${value}`)),
  share() // マルチキャストを有効化
);

// 最初の購読者
console.log('Observer 1 購読開始');
const subscription1 = source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 2.5秒後に2人目の購読者を追加
setTimeout(() => {
  console.log('Observer 2 購読開始');
  const subscription2 = source$.subscribe(value => console.log(`Observer 2: ${value}`));
  
  // 5秒後に購読者1を解除
  setTimeout(() => {
    console.log('Observer 1 購読解除');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

#### 実行結果
```
Observer 1 購読開始
ソース: 0
Observer 1: 0
Observer 2 購読開始
ソース: 1
Observer 1: 1
Observer 2: 1
ソース: 2
Observer 1: 2
Observer 2: 2
ソース: 3
Observer 1: 3
Observer 2: 3
Observer 1 購読解除
ソース: 4
Observer 2: 4
```

### `share()` の詳細な制御

`refCount()`の代わりに、RxJS 7以降では `share()` にオプションを渡すことで、より明確に挙動を制御できます。

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs/operators';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`ソース: ${value}`)),
  share({
    resetOnError: true,
    resetOnComplete: true,
    resetOnRefCountZero: true,
  })
);

// 最初の購読者
console.log('Observer 1 購読開始');
const subscription1 = source$.subscribe((value) =>
  console.log(`Observer 1: ${value}`)
);

// 2.5秒後に2人目の購読者を追加
setTimeout(() => {
  console.log('Observer 2 購読開始');
  const subscription2 = source$.subscribe((value) =>
    console.log(`Observer 2: ${value}`)
  );

  setTimeout(() => {
    console.log('Observer 1 購読解除');
    subscription1.unsubscribe();
  }, 1500);
}, 2500);
```

#### 実行結果
```
Observer 1 購読開始
ソース: 0
Observer 1: 0
ソース: 1
Observer 1: 1
Observer 2 購読開始
ソース: 2
Observer 1: 2
Observer 2: 2
ソース: 3
Observer 1: 3
Observer 2: 3
Observer 1 購読解除
ソース: 4
Observer 2: 4
ソース: 5
Observer 2: 5
```

この方法により、ストリームの終了や購読者がゼロになったときの挙動を明確に制御できます。

### `shareReplay()` 演算子

[📘 RxJS公式: shareReplay()](https://rxjs.dev/api/index/function/shareReplay)

`share()`と同様ですが、指定した数の過去の値を記憶し、後から参加した購読者にも提供します。

```ts
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs/operators';

// shareReplayを使用（バッファサイズ2）
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`ソース: ${value}`)),
  shareReplay(2) // 直近2つの値をバッファリング
);

// 最初の購読者
console.log('Observer 1 購読開始');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 3.5秒後に2人目の購読者を追加
setTimeout(() => {
  console.log('Observer 2 購読開始 - 最新の2つの値を受け取る');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

#### 実行結果
```
Observer 1 購読開始
ソース: 0
Observer 1: 0
ソース: 1
Observer 1: 1
Observer 2 購読開始 - 最新の2つの値を受け取る
Observer 2: 0
Observer 2: 1
ソース: 2
Observer 1: 2
Observer 2: 2
ソース: 3
Observer 1: 3
Observer 2: 3
ソース: 4
Observer 1: 4
Observer 2: 4
```

## マルチキャスティングにおけるタイミングとライフサイクル

マルチキャストストリームのライフサイクルを理解することは重要です。特に`share()`演算子を使用した場合、以下の挙動に注意が必要です。

1. 最初の購読者: `share()` は、最初の購読が行われたタイミングでソース Observable への接続を開始します。
2. 全ての購読者が解除される: `share({ resetOnRefCountZero: true })` の設定がある場合、購読者が0になるとソースへの接続が解除されます。
3. 完了またはエラー: デフォルトでは、`share()` は complete または error が発生すると内部状態をリセットします（resetOnComplete/resetOnError が true の場合）。
4. 再購読: ストリームがリセットされた後に再度購読が行われると、新しい Observable として再構築されます。

このように、`share()` のオプションにより、購読数や完了状態によってストリームの開始・停止・再生成のタイミングが制御されます。

## 実践的なユースケース

### APIリクエストの共有

同じAPIエンドポイントへの重複リクエストを回避する例。

```ts
import { Observable, of, throwError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, catchError, shareReplay, tap } from 'rxjs/operators';

// APIサービスのシミュレーション
class UserService {
  private cache = new Map<string, Observable<any>>();
  
  getUser(id: string): Observable<any> {
    // キャッシュにある場合はそれを返す
    if (this.cache.has(id)) {
      console.log(`キャッシュからユーザーID ${id} を取得`);
      return this.cache.get(id)!;
    }
    
    // 新しいリクエストを作成
    console.log(`APIからユーザーID ${id} を取得`);
    const request$ = ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${id}`).pipe(
      tap(response => console.log('APIレスポンス:', response)),
      catchError(error => {
        console.error('APIエラー:', error);
        // キャッシュから削除
        this.cache.delete(id);
        return throwError(() => new Error('ユーザー取得に失敗しました'));
      }),
      // shareReplayで共有化（完了後も値をキャッシュ）
      shareReplay(1)
    );
    
    // キャッシュに保存
    this.cache.set(id, request$);
    return request$;
  }
}

// 使用例
const userService = new UserService();

// 複数のコンポーネントが同じユーザーデータを要求
console.log('コンポーネント1: ユーザーデータを要求');
userService.getUser('1').subscribe(user => {
  console.log('コンポーネント1: ユーザーデータを受信', user);
});

// 少し遅れて別のコンポーネントも同じデータを要求
setTimeout(() => {
  console.log('コンポーネント2: 同じユーザーデータを要求');
  userService.getUser('1').subscribe(user => {
    console.log('コンポーネント2: ユーザーデータを受信', user);
  });
}, 1000);

// 別のユーザーを要求
setTimeout(() => {
  console.log('コンポーネント3: 別のユーザーデータを要求');
  userService.getUser('2').subscribe(user => {
    console.log('コンポーネント3: ユーザーデータを受信', user);
  });
}, 2000);
```

#### 実行結果
```
コンポーネント1: ユーザーデータを要求
APIからユーザーID 1 を取得
APIレスポンス: {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
コンポーネント1: ユーザーデータを受信 {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
コンポーネント2: 同じユーザーデータを要求
キャッシュからユーザーID 1 を取得
コンポーネント2: ユーザーデータを受信 {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
コンポーネント3: 別のユーザーデータを要求
APIからユーザーID 2 を取得
APIレスポンス: {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
コンポーネント3: ユーザーデータを受信 {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
```

## マルチキャスティングの設計パターン

### シングルトンオブザーバブル

アプリケーション全体で単一のオブザーバブルを共有するパターンです。

```ts
import { Subject } from 'rxjs';

// アプリケーション全体のグローバル状態管理
class AppState {
  // シングルトンインスタンス
  private static instance: AppState;

  // グローバルな通知ストリーム
  private notificationsSubject = new Subject<string>();

  // 公開用のオブザーバブル（read-only）
  readonly notifications$ = this.notificationsSubject.asObservable();

  // シングルトンアクセス
  static getInstance(): AppState {
    if (!AppState.instance) {
      AppState.instance = new AppState();
    }
    return AppState.instance;
  }

  // 通知を送信するメソッド
  notify(message: string): void {
    this.notificationsSubject.next(message);
  }
}

// 使用例
const appState = AppState.getInstance();

// 通知を監視（複数のコンポーネントから）
appState.notifications$.subscribe((msg) =>
  console.log('コンポーネントA:', msg)
);
appState.notifications$.subscribe((msg) =>
  console.log('コンポーネントB:', msg)
);

// 通知を送信
appState.notify('システム更新が利用可能です');
```

#### 実行結果
```ts
コンポーネントA: システム更新が利用可能です
コンポーネントB: システム更新が利用可能です
```

## まとめ

マルチキャスティングは、RxJSアプリケーションの効率性とパフォーマンスを向上させる重要な技術です。主なポイントは以下の通りです。

- マルチキャスティングにより、一つのデータソースを複数の購読者で共有できる
- `share()`、`shareReplay()`、`publish()`などの演算子を使って実装可能
- APIリクエストの重複を避けたり、計算コストの高い処理を最適化できる
- 状態管理やコンポーネント間の通信に役立つ

適切なマルチキャスト戦略を選択することで、アプリケーションの応答性と効率性を高めながら、コード量を減らし保守性を向上させることができます。