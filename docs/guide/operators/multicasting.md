# マルチキャスティング

マルチキャスティングは、RxJSにおいて一つのObservableソースから複数の購読者（Observer）に対して同じデータを効率的に配信するための重要なコンセプトです。これを理解することで、パフォーマンスを向上させたり、重複したリソース消費を避けたりすることができます。

## マルチキャスティングとは

通常のObservable（Cold Observable）は、各購読者が独自の実行を持ちます。つまり、各購読者がサブスクライブするたびにデータストリームが新たに生成され、処理が実行されます。

これに対し、マルチキャスティングされたObservable（Hot Observable）は、複数の購読者間で同一のデータストリームを共有します。これにより、以下のような利点があります：

- 重複したデータ生成処理を防止
- リソース（APIリクエスト、タイマー）の効率的利用
- 複数コンポーネント間での一貫した状態の共有

## Cold ObservableとHot Observableの違い

マルチキャスティングを理解するには、まず「Cold」と「Hot」の違いを理解する必要があります：

```ts
import { Observable, Subject } from 'rxjs';

// Cold Observable - 各購読者が独自の実行を持つ
const cold$ = new Observable<number>(subscriber => {
  const random = Math.random();
  console.log('Cold Observable実行');
  subscriber.next(random);
});

// Cold Observableを2回購読
cold$.subscribe(val => console.log('購読者1:', val));
cold$.subscribe(val => console.log('購読者2:', val));

// Hot Observable (Subject使用) - 実行が共有される
const hot$ = new Subject<number>();
console.log('Hot Observable作成');

// Hot Observableを2回購読
hot$.subscribe(val => console.log('購読者A:', val));
hot$.subscribe(val => console.log('購読者B:', val));

// 値を発行
hot$.next(Math.random());
```

実行結果：
```
Cold Observable実行
購読者1: 0.123456789
Cold Observable実行
購読者2: 0.987654321  // 別の乱数
Hot Observable作成
購読者A: 0.555555555
購読者B: 0.555555555  // 同じ乱数
```

## マルチキャスト演算子

RxJSでは、Cold ObservableをHotに変換するための専用オペレーターがいくつか用意されています。

### share オペレーター

最も簡単にマルチキャストを実装できるオペレーターです。内部的に`Subject`と`refCount`を使用しています。

```ts
import { interval } from 'rxjs';
import { share, take, tap } from 'rxjs/operators';

// Observableを作成（APIリクエストをシミュレート）
const source$ = interval(1000).pipe(
  take(3),
  tap(x => console.log('ソース値:', x)),
  share()  // マルチキャスト化
);

// 複数の購読
console.log('購読者1開始');
const sub1 = source$.subscribe(val => console.log('購読者1:', val));

// 1.5秒後に2人目の購読者を追加
setTimeout(() => {
  console.log('購読者2開始');
  const sub2 = source$.subscribe(val => console.log('購読者2:', val));
}, 1500);
```

実行結果：
```
購読者1開始
ソース値: 0
購読者1: 0
ソース値: 1
購読者1: 1
購読者2開始
ソース値: 2
購読者1: 2
購読者2: 2
```

ここで注目すべき点：
- ソース値の生成（`tap`内のログ）は一度だけ
- 2人目の購読者は途中から参加するため、以前の値は受け取れない
- すべての購読者がいなくなると実行は停止（`refCount`の効果）

### shareReplay オペレーター

`share`と同様ですが、指定した数の過去の値を記録し、新しい購読者にも提供します。

```ts
import { interval } from 'rxjs';
import { shareReplay, take, tap } from 'rxjs/operators';

const source$ = interval(1000).pipe(
  take(3),
  tap(x => console.log('ソース値:', x)),
  shareReplay(2)  // 直近2つの値をバッファ
);

console.log('購読者1開始');
source$.subscribe(val => console.log('購読者1:', val));

// 2.5秒後に2人目の購読者を追加（最初の値はバッファされていない）
setTimeout(() => {
  console.log('購読者2開始');
  source$.subscribe(val => console.log('購読者2:', val));
}, 2500);
```

実行結果：
```
購読者1開始
ソース値: 0
購読者1: 0
ソース値: 1
購読者1: 1
ソース値: 2
購読者1: 2
購読者2開始
購読者2: 1  // バッファされた値
購読者2: 2  // バッファされた値
```

### publish と refCount

`publish`と`refCount`を組み合わせることで、より細かい制御が可能です。

```ts
import { interval } from 'rxjs';
import { publish, refCount, take, tap } from 'rxjs/operators';

const source$ = interval(1000).pipe(
  take(3),
  tap(x => console.log('ソース値:', x)),
  publish(),
  refCount()
);

// 使用方法はshareと同様
console.log('購読者1開始');
const sub1 = source$.subscribe(val => console.log('購読者1:', val));

setTimeout(() => {
  console.log('購読者2開始');
  const sub2 = source$.subscribe(val => console.log('購読者2:', val));
}, 1500);
```

## multicast オペレーター

`multicast`は最も低レベルなマルチキャスト演算子で、より高度な制御が可能です。明示的に`Subject`を渡すことができます。

```ts
import { interval, Subject } from 'rxjs';
import { multicast, refCount, take, tap } from 'rxjs/operators';

// Subject/BehaviorSubject/ReplaySubjectなど任意のタイプを指定可能
const subject = new Subject<number>();

const source$ = interval(1000).pipe(
  take(3),
  tap(x => console.log('ソース値:', x)),
  multicast(subject),  // 明示的にSubjectを渡す
  refCount()
);

console.log('購読者1開始');
const sub1 = source$.subscribe(val => console.log('購読者1:', val));

setTimeout(() => {
  console.log('購読者2開始');
  const sub2 = source$.subscribe(val => console.log('購読者2:', val));
}, 1500);
```

## 実践的な例: APIデータ共有

複数のコンポーネントが同じAPIデータを必要とする場合の例です。

```ts
import { Observable, of, throwError } from 'rxjs';
import { shareReplay, tap, catchError, delay } from 'rxjs/operators';

// データサービスクラス
class UserService {
  private cachedUsers$: Observable<any[]> | null = null;
  
  getUsers(): Observable<any[]> {
    // キャッシュがあれば再利用
    if (this.cachedUsers$) {
      console.log('キャッシュからユーザーデータを返します');
      return this.cachedUsers$;
    }
    
    // キャッシュがなければAPIリクエスト
    console.log('APIからユーザーデータを取得します');
    this.cachedUsers$ = this.fetchUsersFromAPI().pipe(
      // エラーハンドリング
      catchError(err => {
        console.error('APIエラー:', err);
        this.cachedUsers$ = null;  // キャッシュクリア
        return throwError(() => new Error('ユーザー取得に失敗しました'));
      }),
      // 完了後もキャッシュ維持
      shareReplay(1)
    );
    
    return this.cachedUsers$;
  }
  
  // APIリクエストをシミュレート
  private fetchUsersFromAPI(): Observable<any[]> {
    return of([
      { id: 1, name: '山田太郎' },
      { id: 2, name: '佐藤花子' },
      { id: 3, name: '鈴木一郎' }
    ]).pipe(
      delay(2000),  // APIレイテンシをシミュレート
      tap(() => console.log('APIレスポンス受信'))
    );
  }
  
  // キャッシュをクリア（データ更新時など）
  clearCache(): void {
    this.cachedUsers$ = null;
    console.log('キャッシュをクリアしました');
  }
}

// 使用例
const userService = new UserService();

// コンポーネント1がデータをリクエスト
console.log('コンポーネント1: ユーザー一覧を要求');
userService.getUsers().subscribe({
  next: users => console.log('コンポーネント1: ユーザー数', users.length),
  error: err => console.error('コンポーネント1エラー:', err)
});

// 1秒後にコンポーネント2も同じデータをリクエスト
setTimeout(() => {
  console.log('コンポーネント2: ユーザー一覧を要求');
  userService.getUsers().subscribe({
    next: users => console.log('コンポーネント2: ユーザー数', users.length),
    error: err => console.error('コンポーネント2エラー:', err)
  });
}, 1000);

// 3秒後にキャッシュをクリアして再取得
setTimeout(() => {
  userService.clearCache();
  console.log('コンポーネント3: ユーザー一覧を要求');
  userService.getUsers().subscribe({
    next: users => console.log('コンポーネント3: ユーザー数', users.length),
    error: err => console.error('コンポーネント3エラー:', err)
  });
}, 5000);
```

実行結果：
```
コンポーネント1: ユーザー一覧を要求
APIからユーザーデータを取得します
コンポーネント2: ユーザー一覧を要求
キャッシュからユーザーデータを返します
APIレスポンス受信
コンポーネント1: ユーザー数 3
コンポーネント2: ユーザー数 3
キャッシュをクリアしました
コンポーネント3: ユーザー一覧を要求
APIからユーザーデータを取得します
APIレスポンス受信
コンポーネント3: ユーザー数 3
```

## マルチキャスティングパターンの比較

| オペレーター | 特徴 | ユースケース |
|------------|------|-------------|
| `share()` | 基本的なマルチキャスト | 複数コンポーネントでの同時利用 |
| `shareReplay(n)` | 過去n個の値をバッファ | 遅延購読/状態共有 |
| `publish() + refCount()` | より細かい制御が可能 | 高度な制御が必要な場合 |
| `multicast(() => new Subject())` | 完全なカスタマイズ | 特殊なSubjectタイプが必要な場合 |

## マルチキャスティング使用時の注意点

1. **タイミングの把握**：購読開始時期によって受け取る値が異なることを理解する
2. **ライフサイクル管理**：特に`refCount`を使う場合、購読者がゼロになるとストリームが完了する
3. **エラー処理**：マルチキャストされたObservableでエラーが発生すると、全購読者に影響する
4. **メモリ管理**：`shareReplay`などを使う場合、メモリリークに注意する

## まとめ

マルチキャスティングは、RxJSアプリケーションのパフォーマンスと設計に大きく影響する重要な概念です。適切なマルチキャスト戦略を選択することで、以下のようなメリットが得られます：

- 重複APIリクエストの削減
- コンポーネント間の状態共有の簡素化
- リソース使用量の最適化
- アプリケーション全体のパフォーマンス向上

複雑なアプリケーションを構築する際は、Cold/Hotの概念とマルチキャスティングを理解し、状況に応じて適切なオペレーターを選択することが重要です。