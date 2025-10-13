---
description: finalizeとcompleteを用いて、RxJSにおけるストリームの完了処理とリソース解放を効果的に行う方法を解説します。
---
# finalize と complete - リソース解放とストリームの完了処理

RxJSでは、ストリームの終了とリソース解放を適切に管理することが重要です。このページでは、`finalize`オペレーターと`complete`通知の仕組みについて解説します。

## finalize - リソース解放のためのオペレーター

`finalize`オペレーターは、Observableが**完了・エラー・アンサブスクライブのいずれかで終了したとき**に、指定したクリーンアップコードを実行するためのオペレーターです。  
finalizeは**ストリームの終了時に必ず一度だけ**呼び出され、複数回呼ばれることはありません。

[🌐 RxJS公式ドキュメント - finalize](https://rxjs.dev/api/index/function/finalize)

### finalize の基本的な使い方

```ts
import { of } from 'rxjs';
import { finalize, tap } from 'rxjs';

// ローディング状態を管理する変数
let isLoading = true;

// 成功するストリーム
of('データ')
  .pipe(
    tap((data) => console.log('データ処理中:', data)),
    // 成功・失敗・キャンセルのいずれの場合でも実行される
    finalize(() => {
      isLoading = false;
      console.log('ローディング状態リセット:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('値:', value),
    complete: () => console.log('完了'),
  });

// 出力:
// データ処理中: データ
// 値: データ
// 完了
// ローディング状態リセット: false
```

### エラー発生時の finalize

```ts
import { throwError } from 'rxjs';
import { finalize, catchError } from 'rxjs';

let isLoading = true;

throwError(() => new Error('データ取得エラー'))
  .pipe(
    catchError((err) => {
      console.error('エラー処理:', err.message);
      throw err; // エラーを再スロー
    }),
    finalize(() => {
      isLoading = false;
      console.log('エラー後のリソース解放:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('値:', value),
    error: (err) => console.error('サブスクライバーでのエラー:', err.message),
    complete: () => console.log('完了'), // エラー時は呼ばれない
  });

// 出力:
// エラー処理: データ取得エラー
// サブスクライバーでのエラー: データ取得エラー
// エラー後のリソース解放: false
```

### アンサブスクライブ時のfinalize

```ts
import { interval } from 'rxjs';
import { finalize } from 'rxjs';

let resource = 'アクティブ';

// 1秒ごとにカウント
const subscription = interval(1000)
  .pipe(
    finalize(() => {
      resource = '解放済み';
      console.log('リソース状態:', resource);
    })
  )
  .subscribe((count) => {
    console.log('カウント:', count);

    // 3回カウントしたら手動でアンサブスクライブ
    if (count >= 2) {
      subscription.unsubscribe();
    }
  });

// 出力:
// カウント: 0
// カウント: 1
// カウント: 2
// リソース状態: 解放済み
```

finalizeは、エラー発生時だけでなく正常完了時や、手動でのアンサブスクライブ（unsubscribe）時にも確実にクリーンアップ処理を行いたい場合に有効です。

## complete - ストリームの正常終了通知

Observableが正常に終了すると、Observerの`complete`コールバックが呼び出されます。これはObservableのライフサイクルの最後のステップです。

### 自動的なcomplete

いくつかのObservableは、特定の条件が満たされると自動的に完了します。

```ts
import { of } from 'rxjs';
import { take } from 'rxjs';

// 有限のシーケンスは自動的に完了する
of(1, 2, 3).subscribe({
  next: (value) => console.log('値:', value),
  complete: () => console.log('有限ストリーム完了'),
});

// interval + takeで制限したストリーム
interval(1000)
  .pipe(
    take(3) // 3つ値を取得したら完了
  )
  .subscribe({
    next: (value) => console.log('カウント:', value),
    complete: () => console.log('制限付きストリーム完了'),
  });

// 出力:
// 値: 1
// 値: 2
// 値: 3
// 有限ストリーム完了
// カウント: 0
// カウント: 1
// カウント: 2
// 制限付きストリーム完了

```

### 手動でのcomplete

Subjectやカスタムのものではcompleteを手動で呼び出せます。

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.subscribe({
  next: (value) => console.log('値:', value),
  complete: () => console.log('Subject完了'),
});

subject.next(1);
subject.next(2);
subject.complete(); // 手動で完了
subject.next(3); // 完了後は無視される

// 出力:
// 値: 1
// 値: 2
// Subject完了
```

## finalize と complete の違い

重要な違いを理解しておきましょう。

1. **実行タイミング**
   - `complete`: Observableが**正常に完了したとき**のみ呼ばれる
   - `finalize`: Observableが**完了・エラー・アンサブスクライブのいずれか**で終了したときに呼ばれる

2. **用途**
   - `complete`: 正常終了時の通知を受け取る（成功したときの処理）
   - `finalize`: リソース解放やクリーンアップを確実に行う（成功・失敗にかかわらず必ず実行したい処理）

## 実践的なユースケース

### API呼び出しとローディング状態の管理

```ts
import { ajax } from 'rxjs/ajax';
import { finalize, catchError } from 'rxjs';
import { of } from 'rxjs';

// ローディング状態
let isLoading = false;

function fetchData(id: string) {
  // ローディング開始
  isLoading = true;
  const loading = document.createElement('p');
  loading.style.display = 'block';
  document.body.appendChild(loading);
  // document.getElementById('loading')!.style.display = 'block';

  // APIリクエスト
  return ajax.getJSON(`https://jsonplaceholder.typicode.com/posts/${id}`).pipe(
    catchError((error) => {
      console.error('APIエラー:', error);
      return of({ error: true, message: 'データの取得に失敗しました' });
    }),
    // 成功・失敗にかかわらずローディングを終了
    finalize(() => {
      isLoading = false;
      loading!.style.display = 'none';
      console.log('ローディング状態リセット完了');
    })
  );
}

// 使用例
fetchData('123').subscribe({
  next: (data) => console.log('データ:', data),
  complete: () => console.log('データ取得完了'),
});

// 出力:
//  APIエラー: AjaxErrorImpl {message: 'ajax error', name: 'AjaxError', xhr: XMLHttpRequest, request: {…}, status: 0, …}
//  データ: {error: true, message: 'データの取得に失敗しました'}
//  データ取得完了
//  ローディング状態リセット完了
//   GET https://jsonplaceholder.typicode.com/posts/123 net::ERR_NAME_NOT_RESOLVED
```

### リソースのクリーンアップ

```ts
import { interval } from 'rxjs';
import { finalize, takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

class ResourceManager {
  private destroy$ = new Subject<void>();
  private timerId: number | null = null;

  constructor() {
    // 何らかのリソースの初期化
    this.timerId = window.setTimeout(() => console.log('タイマー実行'), 10000);

    // 定期的な処理
    interval(1000)
      .pipe(
        // コンポーネント破棄時に停止
        takeUntil(this.destroy$),
        // 確実にリソース解放
        finalize(() => {
          console.log('インターバル停止');
        })
      )
      .subscribe((count) => {
        console.log('実行中...', count);
      });
  }

  dispose() {
    // 破棄処理
    if (this.timerId) {
      window.clearTimeout(this.timerId);
      this.timerId = null;
    }

    // ストリームの停止シグナル
    this.destroy$.next();
    this.destroy$.complete();

    console.log('リソースマネージャー破棄完了');
  }
}

// 使用例
const manager = new ResourceManager();

// 5秒後に破棄
setTimeout(() => {
  manager.dispose();
}, 5000);

// 出力:
// 実行中... 0
// 実行中... 1
// 実行中... 2
// 実行中... 3
// 実行中... 4
// インターバル停止
// リソースマネージャー破棄完了
```

[📘 RxJS公式: takeUntil()](https://rxjs.dev/api/index/function/takeUntil)

## ベストプラクティス

1. **常にリソースを解放する**: `finalize`を使ってストリームが終了する際のクリーンアップを保証する
2. **ローディング状態の管理**: `finalize`を使って必ずローディング状態をリセットする
3. **コンポーネントのライフサイクル管理**: `takeUntil` と `finalize` を組み合わせて、コンポーネント破棄時にリソースをクリーンアップする（特にAngularなどではこのパターンが推奨されます）
4. **エラー処理との併用**: `catchError`と`finalize`を組み合わせて、エラー後のフォールバック処理と確実なクリーンアップを実現する
5. **完了ステータスの把握**: `complete`コールバックを使って、ストリームが正常に完了したかどうかを判断する

## まとめ

`finalize`と`complete`は、RxJSでのリソース管理と処理完了のための重要なツールです。`finalize`はストリームがどのように終了しても確実に実行されるため、リソース解放に最適です。一方、`complete`は正常終了時の処理を行いたい場合に使用します。これらを適切に組み合わせることで、メモリリークを防ぎ、信頼性の高いアプリケーションを構築できます。