---
description: RxJSの包括的なエラー処理戦略を解説します。catchError、retry、retryWhen、finalizeオペレーターの組み合わせ方、指数バックオフによる再試行、エラー時のリソース解放、フォールバック処理など実践的なパターンを紹介します。
---
# RxJSのエラー処理戦略

RxJSにおけるエラー処理は、リアクティブプログラミングの重要な側面です。適切なエラー処理を実装することで、アプリケーションの堅牢性と信頼性が向上します。このドキュメントでは、RxJSで使用できる様々なエラー処理戦略について説明します。

## 基本パターン

RxJSでは、Observableのライフサイクルの一部としてエラーを処理します。基本的なエラー処理には以下の方法があります。

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

// エラーを発生させるObservable
const error$ = throwError(() => new Error('エラーが発生しました')); // RxJS 7以降、関数形式推奨

// 基本的なエラーハンドリング
error$
  .pipe(
    catchError((error) => {
      console.error('エラーをキャッチ:', error.message);
      return of('エラー後のフォールバック値');
    })
  )
  .subscribe({
    next: (value) => console.log('値:', value),
    error: (err) => console.error('ハンドリングされなかったエラー:', err),
    complete: () => console.log('完了'),
  });

// 出力:
// エラーをキャッチ: エラーが発生しました
// 値: エラー後のフォールバック値
// 完了
```

## 様々なエラー処理戦略

### 1. エラーを捕捉して代替値を提供

`catchError`オペレーターを使用して、エラーを捕捉し、代替値や代替ストリームを提供します。

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

const source$ = throwError(() => new Error('データ取得エラー'));

source$.pipe(
  catchError(error => {
    console.error('エラー発生:', error.message);
    // 代替データを返す
    return of({ isError: true, data: [], message: 'デフォルトデータを表示します' });
  })
).subscribe(data => console.log('結果:', data));

// 出力:
// エラー発生: データ取得エラー
// 結果: {isError: true, data: Array(0), message: 'デフォルトデータを表示します'}
```

### 2. エラーが発生したら再試行

`retry`や`retryWhen`オペレーターを使用して、エラーが発生した場合にストリームを再試行します。

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, tap } from 'rxjs';

let attemptCount = 0;

interval(1000).pipe(
  mergeMap(val => {
    if (++attemptCount <= 2) {
      return throwError(() => new Error(`エラー #${attemptCount}`));
    }
    return of('成功！');
  }),
  tap(() => console.log('実行:', attemptCount)),
  retry(2), // 最大2回再試行
).subscribe({
  next: value => console.log('値:', value),
  error: err => console.error('最終エラー:', err.message),
});

// 出力:
// 実行: 3
// 値: 成功！
// 実行: 4
// 値: 成功！
// 実行: 5
// ...
```

### 3. 指数バックオフによる再試行

ネットワークリクエストなどでは、再試行の間隔を徐々に増やす「指数バックオフ」が効果的です。

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, tap, concatMap, catchError } from 'rxjs';

function fetchWithRetry() {
  let retryCount = 0;

  return throwError(() => new Error('ネットワークエラー')).pipe(
    retryWhen((errors) =>
      errors.pipe(
        // エラー回数をカウント
        tap((error) => console.log('エラー発生:', error.message)),
        // 指数バックオフで遅延
        concatMap(() => {
          retryCount++;
          const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
          console.log(`${retryCount}回目の再試行を${delayMs}ms後に実行`);
          return timer(delayMs);
        }),
        // 最大5回まで再試行
        tap(() => {
          if (retryCount >= 5) {
            throw new Error('最大再試行回数を超えました');
          }
        })
      )
    ),
    // 最終的なフォールバック
    catchError((error) => {
      console.error('すべての再試行が失敗:', error.message);
      return of({
        error: true,
        message: '接続に失敗しました。後ほど再試行してください。',
      });
    })
  );
}

fetchWithRetry().subscribe({
  next: (result) => console.log('結果:', result),
  error: (err) => console.error('ハンドリングされなかったエラー:', err),
});

// 出力:
// エラー発生: ネットワークエラー
// 1回目の再試行を2000ms後に実行
// エラー発生: ネットワークエラー
// 2回目の再試行を4000ms後に実行
// エラー発生: ネットワークエラー
// 3回目の再試行を8000ms後に実行
// エラー発生: ネットワークエラー
// 4回目の再試行を10000ms後に実行
// エラー発生: ネットワークエラー
// 5回目の再試行を10000ms後に実行
// すべての再試行が失敗: 最大再試行回数を超えました
// 結果: {error: true, message: '接続に失敗しました。後ほど再試行してください。'}
```

### 4. エラー発生時のリソース解放

`finalize`オペレーターを使って、ストリームが**完了またはエラー**で終了したときにリソースを解放します。
finalizeは、エラー発生時だけでなく正常完了時にも確実にクリーンアップ処理を行いたい場合に有効です。

```ts
import { throwError } from 'rxjs';
import { catchError, finalize } from 'rxjs';

let isLoading = true;

throwError(() => new Error('処理エラー'))
  .pipe(
    catchError((error) => {
      console.error('エラー処理:', error.message);
      return throwError(() => error); // エラーを再スロー
    }),
    finalize(() => {
      isLoading = false;
      console.log('ローディング状態をリセット:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('値:', value),
    error: (err) => console.error('最終エラー:', err.message),
    complete: () => console.log('完了'),
  });

// 出力:
// エラー処理: 処理エラー
// 最終エラー: 処理エラー
// ローディング状態をリセット: false
```

## エラー処理パターン

### UI要素の表示制御を含むエラー処理

```ts
import { of, throwError } from 'rxjs';
import { catchError, finalize, tap } from 'rxjs';

function fetchData(shouldFail = false) {
  // ローディング表示
  showLoadingIndicator();

  // データ取得（成功またはエラー）
  return (
    shouldFail
      ? throwError(() => new Error('APIエラー'))
      : of({ name: 'データ', value: 42 })
  ).pipe(
    tap((data) => {
      // 成功時の処理
      updateUI(data);
    }),
    catchError((error) => {
      // エラー時のUI更新
      showErrorMessage(error.message);
      // 空のデータまたはデフォルト値を返す
      return of({ name: 'デフォルト', value: 0 });
    }),
    finalize(() => {
      // 成功・エラーに関わらずローディング表示を消す
      hideLoadingIndicator();
    })
  );
}

// UI操作用のヘルパー関数
function showLoadingIndicator() {
  console.log('ローディング表示');
}
function hideLoadingIndicator() {
  console.log('ローディング非表示');
}
function updateUI(data: { name: string; value: number }) {
  console.log('UI更新:', data);
}
function showErrorMessage(message: any) {
  console.log('エラー表示:', message);
}

// 使用例
fetchData(true).subscribe();

// 出力:
// ローディング表示
// エラー表示: APIエラー
// ローディング非表示
```

### 複数のエラーソースの処理

```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, map } from 'rxjs';

// 複数のAPIリクエストをシミュレート
function getUser() {
  return of({ id: 1, name: '山田太郎' });
}

function getPosts() {
  return throwError(() => new Error('投稿取得エラー'));
}

function getComments() {
  return throwError(() => new Error('コメント取得エラー'));
}

// すべてのデータを取得し、部分的なエラーを許容
forkJoin({
  user: getUser().pipe(
    catchError((error) => {
      console.error('ユーザー取得エラー:', error.message);
      return of(null); // エラー時はnullを返す
    })
  ),
  posts: getPosts().pipe(
    catchError((error) => {
      console.error('投稿取得エラー:', error.message);
      return of([]); // エラー時は空配列を返す
    })
  ),
  comments: getComments().pipe(
    catchError((error) => {
      console.error('コメント取得エラー:', error.message);
      return of([]); // エラー時は空配列を返す
    })
  ),
})
  .pipe(
    map((result) => ({
      ...result,
      // 部分的にエラーがあったかを示すフラグを追加
      hasErrors:
        !result.user ||
        result.posts.length === 0 ||
        result.comments.length === 0,
    }))
  )
  .subscribe((data) => {
    console.log('最終結果:', data);

    if (data.hasErrors) {
      console.log(
        '一部のデータ取得に失敗しましたが、利用可能なデータを表示します'
      );
    }
  });

// 出力:
// 投稿取得エラー: 投稿取得エラー
// コメント取得エラー: コメント取得エラー
// 最終結果: {user: {…}, posts: Array(0), comments: Array(0), hasErrors: true}
// 一部のデータ取得に失敗しましたが、利用可能なデータを表示します
```

## エラー処理のベストプラクティス

1. **エラーを常にキャッチする**: Observable チェーンでは、必ずエラーハンドリングを追加しましょう。特に長時間実行されるストリームでは重要です。

2. **意味のあるエラーメッセージを提供する**: エラーオブジェクトには、発生場所や原因を特定するのに役立つ情報を含めましょう。

3. **リソースを適切に解放する**: `finalize` を使用して、成功・失敗に関わらずリソースが解放されるようにしましょう。

4. **再試行戦略を考慮する**: 特にネットワーク操作では、適切な再試行戦略を実装すると信頼性が向上します。

5. **ユーザーフレンドリーなエラーハンドリング**: UIでは、技術的なエラーメッセージをそのまま表示するのではなく、ユーザーが理解できる情報を提供しましょう。

```ts
// 例：ユーザーフレンドリーなエラーメッセージへの変換
function getErrorMessage(error: any): string {
  if (error.status === 401) {
    return 'セッションが期限切れです。再度ログインしてください。';
  } else if (error.status === 404) {
    return '要求されたリソースが見つかりませんでした。';
  } else if (error.status >= 500) {
    return 'サーバーエラーが発生しました。後ほど再試行してください。';
  }
  return '予期しないエラーが発生しました。';
}
```

## まとめ

RxJSにおけるエラー処理は、アプリケーションの堅牢性を確保するための重要な部分です。`catchError`、`retry`、`finalize`などのオペレーターを適切に組み合わせることで、様々なエラーシナリオに対応できます。エラーを単にキャッチするだけでなく、ユーザー体験を向上させるために総合的なエラー処理戦略を設計しましょう。

## 🔗 関連セクション

- **[よくある間違いと対処法](/guide/anti-patterns/common-mistakes#9-エラーの握りつぶし)** - エラー処理に関するアンチパターンを確認
- **[retry と catchError](/guide/error-handling/retry-catch)** - より詳細な活用方法を解説