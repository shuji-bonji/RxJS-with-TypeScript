---
description: retryとcatchErrorオペレーターを組み合わせた堅牢なエラー処理戦略を解説します。一時的な障害の再試行、指数バックオフパターン、適切なフォールバック処理など実践的なAPIリクエストの例を通して学びます。
---
# retry と catchError - 効果的なエラー処理の組み合わせ

RxJSにおけるエラー処理の中核となる二つのオペレーター、`retry`と`catchError`について詳しく解説します。これらを組み合わせることで、堅牢なエラー処理戦略を実現できます。

## retry - 失敗時の再試行（基本パターン）

`retry`オペレーターは、ストリームでエラーが発生した場合に、**指定した回数だけストリームの実行を再開**するためのオペレーターです。ネットワークリクエストなど、一時的に失敗する可能性がある操作に特に有効です。

[🌐 RxJS公式ドキュメント - retry](https://rxjs.dev/api/index/function/retry)

### 基本パターン

```ts
import { Observable, of } from 'rxjs';
import { retry, map } from 'rxjs';

// ランダムにエラーが発生する関数
function getDataWithRandomError(): Observable<string> {
  return of('データ').pipe(
    map(() => {
      if (Math.random() < 0.7) {
        throw new Error('ランダムエラー発生');
      }
      return 'データ取得成功!';
    })
  );
}

// 最大3回まで再試行
getDataWithRandomError()
  .pipe(retry(3))
  .subscribe({
    next: (data) => console.log('成功:', data),
    error: (err) => console.error('エラー (3回の再試行後):', err.message),
  });

// 出力:
// 成功: データ取得成功!
// エラー (3回の再試行後): ランダムエラー発生 ⇦ 3回失敗したときに表示
```

### リアルタイムでの再試行状況の監視

```ts
import { Observable, of } from 'rxjs';
import { retry, tap, catchError, map } from 'rxjs';

let attempts = 0;

function simulateFlakyRequest(): Observable<string> {
  return of('リクエスト').pipe(
    tap(() => {
      attempts++;
      console.log(`試行 #${attempts}`);
    }),
    map(() => {
      if (attempts < 3) {
        throw new Error(`エラー #${attempts}`);
      }
      return '成功！';
    })
  );
}

simulateFlakyRequest()
  .pipe(
    retry(3),
    catchError((error) => {
      console.log('すべての再試行が失敗:', error.message);
      return of('フォールバック値');
    })
  )
  .subscribe({
    next: (result) => console.log('最終結果:', result),
    complete: () => console.log('完了'),
  });

// 出力:
// 試行 #1
// 試行 #2
// 試行 #3
// 最終結果: 成功！
// 完了
```

> [!NOTE] リトライタイミングとスケジューラー
> `retry` オペレーターで遅延時間を指定する場合（`retry({ delay: 1000 })` など）、内部的に **asyncScheduler** が使用されます。スケジューラーを活用することで、リトライのタイミングを細かく制御したり、テスト時に仮想時間を使用したりできます。
>
> 詳しくは[スケジューラーの種類と使い分け - エラーリトライの制御](/guide/schedulers/types#エラーリトライの制御)を参照してください。

## catchError - エラー捕捉と代替処理（基本パターン）

`catchError`オペレーターは、ストリーム内で発生したエラーを捕捉し、**代替のObservableを返す**ことでエラーを処理します。これにより、エラーが発生してもストリームが中断されずに処理を継続できます。

[🌐 RxJS公式ドキュメント - catchError](https://rxjs.dev/api/index/function/catchError)

### 基本パターン

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('API呼び出しエラー')) // RxJS 7以降、関数形式推奨
  .pipe(
    catchError((error) => {
      console.error('エラー発生:', error.message);
      return of('エラー発生時のデフォルト値');
    })
  )
  .subscribe({
    next: (value) => console.log('値:', value),
    complete: () => console.log('完了'),
  });

// 出力:
// エラー発生: API呼び出しエラー
// 値: エラー発生時のデフォルト値
// 完了
```

### エラーの再スロー

エラーを記録したあとで再度スローしたい場合

```ts
import { throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('元のエラー')) // RxJS 7以降、関数形式推奨
  .pipe(
    catchError((error) => {
      console.error('エラーをログ記録:', error.message);
      // エラーを再スロー
      return throwError(() => new Error('変換されたエラー'));
    })
  )
  .subscribe({
    next: (value) => console.log('値:', value),
    error: (err) => console.error('最終エラー:', err.message),
    complete: () => console.log('完了'),
  });

// 出力:
// エラーをログ記録: 元のエラー
// 最終エラー: 変換されたエラー
```

## retry と catchError の組み合わせ

実際のアプリケーションでは、`retry`と`catchError`を組み合わせて使用するのが一般的です。この組み合わせにより、一時的なエラーを再試行で解決しつつ、最終的に失敗した場合はフォールバック値を提供できます。

```ts
import { of, throwError } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

function fetchData() {
  // エラーを発生させるObservable
  return throwError(() => new Error('ネットワークエラー')) // RxJS 7以降、関数形式推奨
    .pipe(
    // デバッグ用
    tap(() => console.log('データ取得を試行')),
    // 最大3回再試行
    retry(3),
    // すべての再試行が失敗した場合
    catchError((error) => {
      console.error('すべての再試行が失敗:', error.message);
      // デフォルト値を返す
      return of({
        error: true,
        data: null,
        message: 'データ取得に失敗しました',
      });
    })
  );
}

fetchData().subscribe({
  next: (result) => console.log('結果:', result),
  complete: () => console.log('処理完了'),
});

// 出力:
// すべての再試行が失敗: ネットワークエラー
// 結果: {error: true, data: null, message: 'データ取得に失敗しました'}
// 処理完了
```

## 高度な再試行戦略: retryWhen

より柔軟な再試行戦略が必要な場合は、`retryWhen`オペレーターを使用できます。これにより、再試行のタイミングやロジックをカスタマイズできます。


[🌐 RxJS公式ドキュメント - retryWhen](https://rxjs.dev/api/index/function/retryWhen)

### 指数バックオフによる再試行

ネットワークリクエストの再試行では、指数バックオフパターン（再試行間隔を徐々に長くする）が一般的です。これにより、サーバーへの負荷を軽減しつつ、一時的な問題が解決するのを待つことができます。

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
          // timer は内部的に asyncScheduler を使用
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
```

> [!TIP] スケジューラーを活用した詳細なリトライ制御
> 上記の例では `timer()` を使用していますが、より高度な制御が必要な場合は、スケジューラーを明示的に指定することで、リトライのタイミングを細かく調整したり、テスト時に仮想時間を使用したりできます。
>
> 詳しくは[スケジューラーの種類と使い分け - エラーリトライの制御](/guide/schedulers/types#エラーリトライの制御)を参照してください。

## リトライのデバッグ

リトライ処理をデバッグする際、試行回数や各試行の結果を追跡することが重要です。以下では、リトライ状況をリアルタイムで監視する実用的な方法を紹介します。

### 方法1: tap の error コールバック（基本）

`tap`オペレーターの`error`コールバックを使用することで、エラー発生時に試行回数をカウントできます。

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('一時的なエラー'))
  .pipe(
    tap({
      error: () => {
        attemptCount++;
        console.log(`試行回数: ${attemptCount}`);
      }
    }),
    retry(2),
    catchError((error) => {
      console.log(`最終試行回数: ${attemptCount}`);
      return of(`最終エラー: ${error.message}`);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('購読エラー:', err)
  });

// 出力:
// 試行回数: 1
// 試行回数: 2
// 試行回数: 3
// 最終試行回数: 3
// 最終エラー: 一時的なエラー
```

> [!NOTE] throwError での制限
> `throwError`は値を発行せずに即座にエラーを出すため、`tap`の`next`コールバックは実行されません。`error`コールバックを使用する必要があります。

### 方法2: retryWhen で詳細に追跡（推奨）

より詳細な情報（試行回数、遅延時間、エラー内容）を追跡するには、`retryWhen`を使用します。

```typescript
import { throwError, of, timer, retryWhen, mergeMap, catchError } from 'rxjs';
throwError(() => new Error('一時的なエラー'))
  .pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
          console.log(`🔄 リトライ ${retryCount}回目`);
          console.log(`   エラー: ${error.message}`);

          if (retryCount > 2) {
            console.log(`❌ 最大リトライ回数に到達`);
            console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
            throw error;
          }

          const delayMs = 1000;
          console.log(`⏳ ${delayMs}ms後に再試行...`);
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

          return timer(delayMs);
        })
      )
    ),
    catchError((error) => {
      console.log(`\n最終結果: すべてのリトライが失敗`);
      return of(`最終エラー: ${error.message}`);
    })
  )
  .subscribe(result => console.log('結果:', result));

// 出力:
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 リトライ 1回目
//    エラー: 一時的なエラー
// ⏳ 1000ms後に再試行...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// （1秒待機）
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 リトライ 2回目
//    エラー: 一時的なエラー
// ⏳ 1000ms後に再試行...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// （1秒待機）
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 リトライ 3回目
//    エラー: 一時的なエラー
// ❌ 最大リトライ回数に到達
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
//
// 最終結果: すべてのリトライが失敗
// 結果: 最終エラー: 一時的なエラー
```

### 方法3: カスタムObservableで試行回数を追跡

実際のAPIリクエストなど、値を発行するObservableの場合は、カスタムObservableで試行回数を管理できます。

```typescript
import { Observable, of, retry, catchError } from 'rxjs';
let attemptCount = 0;

// 試行回数をカウントできるObservable
const retryableStream$ = new Observable(subscriber => {
  attemptCount++;
  console.log(`[試行 ${attemptCount}回目]`);

  // 最初の2回は失敗、3回目で成功
  if (attemptCount < 3) {
    subscriber.error(new Error(`失敗 (試行${attemptCount})`));
  } else {
    subscriber.next('成功データ');
    subscriber.complete();
  }
});

retryableStream$
  .pipe(
    retry(2),
    catchError((error) => {
      console.log(`[完了] 合計${attemptCount}回試行して失敗`);
      return of(`最終エラー: ${error.message}`);
    })
  )
  .subscribe({
    next: data => console.log('[結果]', data),
    complete: () => console.log('[完了]')
  });

// 出力:
// [試行 1回目]
// [試行 2回目]
// [試行 3回目]
// [結果] 成功データ
// [完了]
```

### 方法4: 指数バックオフとログ記録

実用的なAPIリクエストでの詳細ログ記録パターンです。

```typescript
import { timer, throwError, of, retryWhen, mergeMap, catchError, finalize } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchWithRetryLogging(url: string, maxRetries = 3) {
  let startTime = Date.now();

  return ajax.getJSON(url).pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          const elapsed = Date.now() - startTime;

          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
          console.log(`🔄 リトライ情報`);
          console.log(`   回数: ${retryCount}/${maxRetries}`);
          console.log(`   エラー: ${error.message || error.status}`);
          console.log(`   経過時間: ${elapsed}ms`);

          if (retryCount >= maxRetries) {
            console.log(`❌ 最大リトライ回数に到達`);
            console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
            throw error;
          }

          // 指数バックオフ
          const delayMs = Math.min(1000 * Math.pow(2, index), 10000);
          console.log(`⏳ ${delayMs}ms後に再試行...`);
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

          return timer(delayMs);
        })
      )
    ),
    catchError((error) => {
      const totalTime = Date.now() - startTime;
      console.log(`\n❌ 最終的に失敗 (合計時間: ${totalTime}ms)`);
      return of({ error: true, message: 'データ取得失敗' });
    }),
    finalize(() => {
      const totalTime = Date.now() - startTime;
      console.log(`\n✅ 処理完了 (合計時間: ${totalTime}ms)`);
    })
  );
}

// 使用例
fetchWithRetryLogging('https://jsonplaceholder.typicode.com/users/1').subscribe({
  next: data => console.log('データ:', data),
  error: err => console.error('エラー:', err)
});
```

### 方法5: RxJS 7.4+ の retry 設定オブジェクト

RxJS 7.4以降では、`retry`に設定オブジェクトを渡すことができます。

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('一時的なエラー'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`試行 ${attemptCount}回目`);
      },
      error: (err) => console.log(`エラー発生:`, err.message)
    }),
    retry({
      count: 2,
      delay: 1000, // 1秒待ってリトライ（内部で asyncScheduler を使用）
      resetOnSuccess: true
    }),
    catchError((error) => {
      console.log(`最終的に失敗（合計${attemptCount}回試行）`);
      return of(`最終エラー: ${error.message}`);
    })
  )
  .subscribe(result => console.log('結果:', result));

// 出力:
// 試行 1回目
// エラー発生: 一時的なエラー
// 試行 2回目
// エラー発生: 一時的なエラー
// 試行 3回目
// エラー発生: 一時的なエラー
// 最終的に失敗（合計3回試行）
// 結果: 最終エラー: 一時的なエラー
```

> [!TIP] リトライデバッグの推奨アプローチ
> - **開発中**: 方法2（retryWhen）または方法4（詳細ログ）が最適
> - **本番環境**: 方法4をベースに、エラー監視サービスへのログ送信を追加
> - **シンプルなケース**: 方法1（tap の error）または方法5（retry設定）で十分
>
> **関連情報**:
> - リトライのタイミング制御については[スケジューラーの種類と使い分け - エラーリトライの制御](/guide/schedulers/types#エラーリトライの制御)を参照
> - デバッグ手法の全体像は[RxJSのデバッグ手法 - リトライの試行回数を追跡](/guide/debugging/#シナリオ6-リトライの試行回数を追跡したい)を参照

## 実際のアプリケーションでの使用例：APIリクエスト

実際のAPIリクエストでこれらのオペレーターを活用する例です。

```ts
import { Observable, of } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { retry, catchError, finalize, tap } from 'rxjs';

// ローディング状態
let isLoading = false;

function fetchUserData(userId: string): Observable<any> {
  isLoading = true;

  return ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`).pipe(
    // リクエストのデバッグ
    tap((response) => console.log('APIレスポンス:', response)),
    // ネットワークエラーは最大2回再試行
    retry(2),
    // エラーハンドリング
    catchError((error) => {
      if (error.status === 404) {
        return of({ error: true, message: 'ユーザーが見つかりません' });
      } else if (error.status >= 500) {
        return of({ error: true, message: 'サーバーエラーが発生しました' });
      }
      return of({ error: true, message: '不明なエラーが発生しました' });
    }),
    // 成功・失敗に関わらず必ず実行
    finalize(() => {
      isLoading = false;
      console.log('ローディング完了');
    })
  );
}

// 使用例
fetchUserData('123').subscribe({
  next: (data) => {
    if (data.error) {
      // エラー情報の表示
      console.error('エラー:', data.message);
    } else {
      // データの表示
      console.log('ユーザーデータ:', data);
    }
  },
});
 
  
// 出力:
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// 不明なエラーが発生しました
// ローディング完了
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
```

## ベストプラクティス

### いつ retry を使うべきか

- **一時的なエラー**が予想される場合（ネットワーク接続問題など）
- **サーバー側の一時的な問題**（高負荷やタイムアウトなど）
- 再試行で**解決する可能性がある**エラーの場合

### いつ retry を使うべきでないか

- **認証エラー**（401, 403）- 再試行しても解決しない
- **リソースが存在しない**（404）- 再試行しても見つからない
- **バリデーションエラー**（400）- リクエスト自体に問題がある
- **クライアント側のプログラムエラー** - 再試行は無意味

### catchError の効果的な使用法

- エラーの**種類に応じて異なる処理**を行う
- ユーザーに**わかりやすいメッセージ**を提供する
- 適切な場合は**フォールバックデータ**を返す
- 必要に応じて**エラーを変換**する

## まとめ

`retry`と`catchError`を組み合わせることで、堅牢なエラー処理が可能になります。一時的なエラーは再試行によって回復を試み、永続的なエラーは適切にフォールバック処理を施すことで、ユーザー体験を向上させることができます。実際のアプリケーションでは、エラーの性質に応じて適切な戦略を選択し、フォールバックメカニズムを提供することが重要です。

次のセクションでは、リソース解放のための`finalize`オペレーターと、ストリームの完了処理について解説します。