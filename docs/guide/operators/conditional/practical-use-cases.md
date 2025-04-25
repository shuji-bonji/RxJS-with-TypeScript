
# 実践的なユースケース

RxJSの条件オペレーターを活用すると、動的な状態に応じてストリームの分岐や切り替えが可能になります。  
この章では、実際に動作するUI付きコードを通して、各演算子の活用パターンを体験できます。

## 条件に基づく異なるデータソースの選択

```ts
import { iif, of, EMPTY } from 'rxjs';
import { switchMap, tap, catchError, retry } from 'rxjs/operators';

// UIの作成
const appContainer = document.createElement('div');
appContainer.innerHTML = '<h3>データソース選択アプリ:</h3>';
document.body.appendChild(appContainer);

// オプション選択
const optionsDiv = document.createElement('div');
optionsDiv.style.marginBottom = '15px';
appContainer.appendChild(optionsDiv);

// チェックボックス（オフラインモード）
const offlineCheck = document.createElement('input');
offlineCheck.type = 'checkbox';
offlineCheck.id = 'offlineMode';
optionsDiv.appendChild(offlineCheck);

const offlineLabel = document.createElement('label');
offlineLabel.htmlFor = 'offlineMode';
offlineLabel.textContent = 'オフラインモード';
offlineLabel.style.marginLeft = '5px';
optionsDiv.appendChild(offlineLabel);

// 検索ID入力
const idInput = document.createElement('input');
idInput.type = 'number';
idInput.placeholder = 'ID (1-10)';
idInput.min = '1';
idInput.max = '10';
idInput.style.marginLeft = '15px';
idInput.style.width = '80px';
optionsDiv.appendChild(idInput);

// 検索ボタン
const searchButton = document.createElement('button');
searchButton.textContent = '検索';
searchButton.style.marginLeft = '10px';
optionsDiv.appendChild(searchButton);

// 結果エリア
const resultsArea = document.createElement('div');
resultsArea.style.padding = '15px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.borderRadius = '5px';
resultsArea.style.backgroundColor = '#f9f9f9';
resultsArea.style.minHeight = '150px';
appContainer.appendChild(resultsArea);

type User = {
  lastUpdated?: Date;
  fromCache?: boolean;
  id: number;
  name: string;
  email: string;
};
type ErrorResult = {
  error: boolean;
  message: string;
};

// オフラインデータ（キャッシュ）
const cachedData: Record<number, User> = {
  1: { id: 1, name: '山田太郎', email: 'yamada@example.com' },
  2: { id: 2, name: '佐藤花子', email: 'sato@example.com' },
  3: { id: 3, name: '鈴木一郎', email: 'suzuki@example.com' },
};

// オンラインAPIシミュレーション
function fetchUserFromApi(id: number) {
  console.log(`APIからユーザーID ${id} を取得中...`);

  // 50%の確率で失敗するAPI
  const shouldFail = Math.random() < 0.5;

  if (shouldFail) {
    return of(null).pipe(
      tap(() => console.log('API呼び出しに失敗')),
      switchMap(() =>
        EMPTY.pipe(
          tap(() => {
            throw new Error('ネットワークエラー');
          })
        )
      ),
      catchError((err) => {
        throw new Error('APIリクエストに失敗しました');
      })
    );
  }

  // 成功時は1秒後にデータを返す
  return of({
    id: id,
    name: `オンラインユーザー${id}`,
    email: `user${id}@example.com`,
    lastUpdated: new Date().toISOString(),
  }).pipe(
    tap(() => console.log('API呼び出しに成功')),
    // 1秒の遅延をシミュレート
    switchMap((data) => {
      return new Promise((resolve) => {
        setTimeout(() => resolve(data), 1000);
      });
    })
  );
}

// キャッシュからユーザーを取得
function getUserFromCache(id: number) {
  console.log(`キャッシュからユーザーID ${id} を取得中...`);

  return iif(
    () => id in cachedData,
    of({ ...cachedData[id], fromCache: true }),
    EMPTY.pipe(
      tap(() => {
        throw new Error('キャッシュにユーザーが見つかりません');
      })
    )
  );
}

// 検索ボタンクリック
searchButton.addEventListener('click', () => {
  const id = parseInt(idInput.value, 10);
  const isOffline = offlineCheck.checked;

  // 入力検証
  if (isNaN(id) || id < 1 || id > 10) {
    resultsArea.innerHTML =
      '<p style="color: red;">有効なID (1-10) を入力してください</p>';
    return;
  }

  // ローディング表示
  resultsArea.innerHTML = '<p>データを取得中...</p>';

  // オフラインモードに基づいてデータソースを選択
  iif(
    () => isOffline,
    getUserFromCache(id).pipe(
      catchError((err) => {
        console.error('キャッシュエラー:', err);
        return of({ error: err.message });
      })
    ),
    fetchUserFromApi(id).pipe(
      retry(2), // 最大2回再試行
      catchError((err) => {
        console.error('APIエラー:', err);

        // APIが失敗したらキャッシュをフォールバックとして使用
        return getUserFromCache(id).pipe(
          catchError(() =>
            of({ error: 'オンラインAPIとキャッシュの両方が失敗しました' })
          )
        );
      })
    )
  ).subscribe({
    next: (result: any) => {
      if ('error' in result) {
        resultsArea.innerHTML = `<p style="color: red;">エラー: ${result.message}</p>`;
      } else {
        const source = result.fromCache
          ? '<span style="color: orange;">(キャッシュから)</span>'
          : '<span style="color: green;">(APIから)</span>';

        resultsArea.innerHTML = `
          <h4>ユーザー情報 ${source}</h4>
          <p><strong>ID:</strong> ${result.id}</p>
          <p><strong>名前:</strong> ${result.name}</p>
          <p><strong>メール:</strong> ${result.email}</p>
          ${
            result.lastUpdated
              ? `<p><small>最終更新: ${new Date(
                  result.lastUpdated
                ).toLocaleString()}</small></p>`
              : ''
          }
        `;
      }
    },
    error: (err) => {
      resultsArea.innerHTML = `<p style="color: red;">エラー: ${err.message}</p>`;
    },
  });
});

// 初期メッセージ
resultsArea.innerHTML = '<p>ボタンをクリックしてデータを取得してください</p>';


```



## 実行時の分岐とフォールバック戦略

`iif` を用いたこの例では、ユーザーの操作や状態に応じてデータソースを「オフラインキャッシュ」「オンラインAPI」から動的に切り替えています。  
また、`catchError` と `retry` を組み合わせることで、失敗時のリトライやフォールバック先の定義も行えます。

特に次のようなユースケースに適しています：

- ネットワークが不安定な環境でのオフライン対応
- キャッシュの活用とオンラインリクエストの切り替え
- API失敗時の自動リトライと代替ルートへの切り替え

## パフォーマンス最適化パターン

より複雑なシナリオでは、条件演算子を組み合わせて最適化されたデータ取得パターンを実装できます。

```ts
import { fromEvent, Observable, of, throwError, timer } from 'rxjs';
import {
  switchMap,
  catchError,
  map,
  tap,
  debounceTime,
  distinctUntilChanged,
  withLatestFrom,
  delay,
  startWith,
} from 'rxjs/operators';

// UI要素の作成
const optimizationContainer = document.createElement('div');
optimizationContainer.innerHTML = '<h3>高度な条件付きデータ取得:</h3>';
document.body.appendChild(optimizationContainer);

// 検索UI
const searchInputGroup = document.createElement('div');
searchInputGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(searchInputGroup);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'ユーザーIDを入力 (1-10)';
searchInput.style.padding = '8px';
searchInput.style.width = '180px';
searchInputGroup.appendChild(searchInput);

const searchButton = document.createElement('button');
searchButton.textContent = '検索';
searchButton.style.marginLeft = '10px';
searchButton.style.padding = '8px 16px';
searchInputGroup.appendChild(searchButton);

// オプション設定
const optionsGroup = document.createElement('div');
optionsGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(optionsGroup);

const cacheCheckbox = document.createElement('input');
cacheCheckbox.type = 'checkbox';
cacheCheckbox.id = 'useCache';
cacheCheckbox.checked = true;
optionsGroup.appendChild(cacheCheckbox);

const cacheLabel = document.createElement('label');
cacheLabel.htmlFor = 'useCache';
cacheLabel.textContent = 'キャッシュを使用';
cacheLabel.style.marginRight = '15px';
optionsGroup.appendChild(cacheLabel);

const forceCheckbox = document.createElement('input');
forceCheckbox.type = 'checkbox';
forceCheckbox.id = 'forceRefresh';
optionsGroup.appendChild(forceCheckbox);

const forceLabel = document.createElement('label');
forceLabel.htmlFor = 'forceRefresh';
forceLabel.textContent = '強制再取得';
optionsGroup.appendChild(forceLabel);

// 結果表示エリア
const optimizedResults = document.createElement('div');
optimizedResults.style.padding = '15px';
optimizedResults.style.border = '1px solid #ddd';
optimizedResults.style.borderRadius = '5px';
optimizedResults.style.minHeight = '150px';
optimizedResults.style.backgroundColor = '#f9f9f9';
optimizationContainer.appendChild(optimizedResults);

// キャッシュ管理
const cache = new Map<string, { data: any; timestamp: number }>();
const CACHE_EXPIRY = 30000; // 30秒

// ユーザーデータをシミュレート取得するAPI
function fetchUserData(id: string, forceRefresh: boolean): Observable<any> {
  // 無効なID
  if (!id || isNaN(Number(id)) || Number(id) < 1 || Number(id) > 10) {
    return throwError(
      () => new Error('無効なユーザーID: 1～10の数値を入力してください')
    );
  }

  const cacheKey = `user-${id}`;
  const cachedItem = cache.get(cacheKey);
  const now = Date.now();

  // キャッシュチェック（期限内かつ強制再取得でない場合）
  if (
    !forceRefresh &&
    cachedItem &&
    now - cachedItem.timestamp < CACHE_EXPIRY
  ) {
    console.log(`キャッシュから取得: ${id}`);
    return of({
      ...cachedItem.data,
      fromCache: true,
    }).pipe(delay(100)); // 高速レスポンスをシミュレート
  }

  // APIリクエストをシミュレート
  console.log(`APIからデータ取得: ${id}`);

  // 25%の確率でランダムにエラー発生
  const shouldFail = Math.random() < 0.25;

  if (shouldFail) {
    return timer(1500).pipe(
      switchMap(() =>
        throwError(() => new Error('APIリクエストに失敗しました'))
      )
    );
  }

  // 成功レスポンス
  return timer(1500).pipe(
    map(() => {
      const userData = {
        id: Number(id),
        name: `ユーザー${id}`,
        email: `user${id}@example.com`,
        lastUpdated: now,
        fromCache: false,
      };

      // キャッシュに保存
      cache.set(cacheKey, {
        data: userData,
        timestamp: now,
      });

      return userData;
    })
  );
}

// 検索条件の変更を監視
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map((event) => (event.target as HTMLInputElement).value.trim()),
  debounceTime(300),
  distinctUntilChanged()
);

// キャッシュ設定の変更を監視
const useCache$ = fromEvent(cacheCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// 強制再取得の変更を監視
const forceRefresh$ = fromEvent(forceCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(false)
);

// 検索ボタンのクリックイベント
const searchClick$ = fromEvent(searchButton, 'click');

// 検索実行
searchClick$
  .pipe(
    // 現在の入力値、キャッシュ設定、強制再取得設定を取得
    withLatestFrom(
      searchTerm$,
      useCache$,
      forceRefresh$,
      (_, term, useCache, forceRefresh) => ({
        term,
        useCache,
        forceRefresh,
      })
    ),
    tap(() => {
      // 検索開始表示
      optimizedResults.innerHTML = '<p>検索中...</p>';
    }),
    // iif()を使用した条件付きストリーム
    switchMap(({ term, useCache, forceRefresh }) => {
      // 検索語が空の場合
      if (!term) {
        return of({ error: '検索語を入力してください' });
      }

      // キャッシュ無効の場合
      if (!useCache) {
        return fetchUserData(term, true);
      }

      // 通常の検索（キャッシュ使用＆必要に応じて強制再取得）
      return fetchUserData(term, forceRefresh);
    }),
    // エラー処理
    catchError((err) => {
      return of({ error: err.message });
    })
  )
  .subscribe({
    next: (result) => {
      if ('error' in result) {
        // エラー表示
        optimizedResults.innerHTML = `
        <p style="color: red;">エラー: ${result.error}</p>
      `;
      } else {
        // データ表示
        const source = result.fromCache
          ? '<span style="color: orange;">(キャッシュから)</span>'
          : '<span style="color: green;">(APIから)</span>';

        optimizedResults.innerHTML = `
        <h4>ユーザー情報 ${source}</h4>
        <p><strong>ID:</strong> ${result.id}</p>
        <p><strong>名前:</strong> ${result.name}</p>
        <p><strong>メール:</strong> ${result.email}</p>
        ${
          result.lastUpdated
            ? `<p><small>最終更新: ${new Date(
                result.lastUpdated
              ).toLocaleString()}</small></p>`
            : ''
        }
      `;
      }
    },
  });

// 初期メッセージ
optimizedResults.innerHTML =
  '<p>ユーザーIDを入力して検索ボタンをクリックしてください</p>';

```


---

## オペレーターの選択ガイド

条件オペレーターは見た目が似ているものも多く、混乱しやすいですが、それぞれ明確な適用目的があります。  
以下は、典型的な判断フローと特徴の比較です。

## 条件オペレーターの選び方

| オペレーター | ユースケース | 特徴 |
|------------|------------|------|
| `iif` | 実行時に1つのストリームを選択する | 2つの選択肢から条件に基づいて1つを選択 |
| `partition` | ストリームを条件で2つに分ける | 元ストリームを条件でTrue/Falseに分割 |
| `throwIfEmpty` | 空のストリームを検出する | 値が一つも発行されない場合にエラーを投げる |
| `defaultIfEmpty` | 空の場合にデフォルト値を使用 | ストリームが空の場合のフォールバック値を提供 |

### 選択の判断フロー

1. **選択肢が2つあるか？**
   - Yes → `iif`を使用
   - No → 次へ

2. **ストリームを分割したいか？**
   - Yes → `partition`を使用
   - No → 次へ

3. **空ストリームに対処したいか？**
   - Yes → 空ストリームをエラーとして扱いたいか？
     - Yes → `throwIfEmpty`
     - No → `defaultIfEmpty`
   - No → 次へ

4. **単純に条件に基づいて値をフィルタリングしたいか？**
   - Yes → `filter`オペレーターを使用（基本的なフィルタリングオペレーター）
   - No → 目的を再検討

## まとめ

条件オペレーターは、ストリームのフローを制御し、特定の条件に基づいて処理を分岐させるための強力なツールです。主なポイントは次のとおりです。

1. **決定に基づくリアクティブフロー**：条件オペレーターを使用することで、イベントやデータの状態に応じて処理を動的に変更できます。
2. **エラー処理の強化**：条件オペレーターは、エラー処理戦略の重要な部分として機能し、例外ケースに対するグレースフルな対応を可能にします。
3. **最適化の機会**：条件付き実行により、不必要な処理を回避し、特にネットワークリクエストやハードウェアアクセスといったコストの高い操作を最適化できます。
4. **複雑なアプリケーションフロー**：複数の条件オペレーターを組み合わせることで、複雑なビジネスロジックや状態管理を宣言的に表現できます。

条件オペレーターは、RxJSを使用したエラー処理、キャッシュ戦略、フォールバックメカニズム、および条件付き実行パターンを実装する際に特に価値を発揮します。他のオペレーターと組み合わせることで、宣言的かつ型安全な方法で複雑なアプリケーションフローを構築できます。