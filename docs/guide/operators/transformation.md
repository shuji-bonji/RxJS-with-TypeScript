# 変換オペレーター

変換オペレーターは、RxJSのパイプライン内でデータを変形・加工するための最も基本的なオペレーターです。これらは、ストリーム内の各値を新しい値や形式に変換することで、データの流れをより扱いやすく、目的に適した形にします。

## 基本的な変換オペレーター

### map

`map`オペレーターは、配列の`map`メソッドと類似した機能を持ち、ストリーム内の各値に関数を適用して新しい値に変換します。

```ts
import { of } from 'rxjs';
import { map } from 'rxjs/operators';

// 数値のストリーム
const numbers$ = of(1, 2, 3, 4, 5);

// 各数値を2倍にする
numbers$.pipe(
  map(n => n * 2)
).subscribe(value => console.log(value));

// 出力:
// 2
// 4
// 6
// 8
// 10
```

### scan

`scan`オペレーターは累積器（アキュムレーター）のような働きをし、前回の結果と現在の値を組み合わせて新しい値を生成します。

```ts
import { of } from 'rxjs';
import { scan } from 'rxjs/operators';

// 数値のストリーム
const numbers$ = of(1, 2, 3, 4, 5);

// 各値を累積
numbers$.pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(value => console.log(value));

// 出力:
// 1  (0 + 1)
// 3  (1 + 2)
// 6  (3 + 3)
// 10 (6 + 4)
// 15 (10 + 5)
```

### pluck

`pluck`オペレーターは、オブジェクトからプロパティを抽出します。オブジェクトのストリームから特定のプロパティのみを取り出したい場合に便利です。

```ts
import { from } from 'rxjs';
import { pluck } from 'rxjs/operators';

// ユーザーオブジェクトのストリーム
const users$ = from([
  { id: 1, name: '田中', age: 28 },
  { id: 2, name: '佐藤', age: 34 },
  { id: 3, name: '鈴木', age: 22 }
]);

// 名前のみを抽出
users$.pipe(
  pluck('name')
).subscribe(name => console.log(name));

// 出力:
// 田中
// 佐藤
// 鈴木
```

### mapTo

`mapTo`オペレーターは、入力値に関係なく常に同じ固定値を出力します。

```ts
import { fromEvent } from 'rxjs';
import { mapTo } from 'rxjs/operators';

// クリックイベント
const clicks$ = fromEvent(document, 'click');

// クリックごとに固定値を出力
clicks$.pipe(
  mapTo('クリックされました！')
).subscribe(message => console.log(message));

// 出力 (クリックごとに):
// クリックされました！
// クリックされました！
// クリックされました！
```

## 高度な変換オペレーター

### mergeMap (flatMap)

`mergeMap`（別名`flatMap`）オペレーターは、各値を新しいObservableに変換し、それらを平坦化（フラット化）して一つのストリームにします。ネストされたAPI呼び出しなど、非同期処理のチェーンに特に便利です。

```ts
import { fromEvent, mergeMap, of, delay } from 'rxjs';

// ボタンを作成
const button = document.createElement('button');
button.textContent = 'クリックしてリクエスト送信';
document.body.appendChild(button);

// クリックイベントをストリーム化
fromEvent(button, 'click')
  .pipe(
    mergeMap((event, index) => {
      const requestId = index + 1;
      console.log(`リクエスト ${requestId} 開始`);
      return of(`レスポンス ${requestId}`).pipe(delay(2000));
    })
  )
  .subscribe((response) => {
    const div = document.createElement('div');
    div.textContent = `✅ ${response}`;
    document.body.appendChild(div);
  });
// 出力:
// リクエスト 1 開始
// ページ表示:
// ✅ レスポンス 1
```

この例は、`mergeMap` によって各クリックから非同期処理が**並列で開始される**様子を実演するコードです。リクエストが完了するまで待たずに次のリクエストを処理し始める点が、`mergeMap` の特性をよく示しています。


### switchMap

`switchMap`は`mergeMap`に似ていますが、新しい内部Observableが始まるたびに前の内部Observableをキャンセルします。これは検索バーの自動補完のような、最新の結果だけが必要なケースに最適です。

```ts
import { fromEvent } from 'rxjs';
import { switchMap, debounceTime, map } from 'rxjs/operators';
import { ajax } from 'rxjs/ajax';

// 検索入力フィールド
const searchInput = document.createElement('input');
document.body.appendChild(searchInput);

// 検索処理
fromEvent(searchInput, 'input')
  .pipe(
    debounceTime(300), // 300ms間入力がなければ処理
    map((event) => (event.target as HTMLInputElement).value.trim()),
    switchMap((term) => {
      if (term === '') {
        return []; // 空の入力は空結果
      }
      // 前のリクエストはキャンセルされる
      return ajax.getJSON(
        `https://jsonplaceholder.typicode.com/users?username_like=${term}`
      );
    })
  )
  .subscribe((users) => {
    // 結果を表示
    console.log('検索結果:', users);
  });

// 出力:
// 検索結果: (6) [{…}, {…}, {…}, {…}, {…}, {…}]
```

### concatMap

`concatMap`は`mergeMap`と似ていますが、順序を保証します。内部Observableは前の内部Observableが完了するまで実行されません。順序が重要な処理に使用します。

```ts
import { from } from 'rxjs';
import { concatMap, delay, map } from 'rxjs/operators';
import { ajax } from 'rxjs/ajax';

type User = {
  id: number;
  name: string;
  username: string;
  email: string;
};

// ユーザーIDのリスト
const userIds$ = from([1, 2, 3, 4, 5]);

// 各ユーザーのデータを順番に取得
userIds$
  .pipe(
    concatMap((id) => {
      console.log(`ユーザーID ${id} の情報を取得中...`);
      // API呼び出しをシミュレート
      return ajax
        .getJSON(`https://jsonplaceholder.typicode.com/users/${id}`)
        .pipe(
          delay(1000) // 1秒遅延
        );
    })
  )
  .subscribe((user) => {
    console.log('取得したユーザー:', (user as User).name);
  });

// 出力:
// ユーザーID 1 の情報を取得中...
// 取得したユーザー: Leanne Graham
// ユーザーID 2 の情報を取得中...
// 取得したユーザー: Ervin Howell
// ユーザーID 3 の情報を取得中...
// 取得したユーザー: Clementine Bauch
// ユーザーID 4 の情報を取得中...
// 取得したユーザー: Patricia Lebsack
// ユーザーID 5 の情報を取得中...
// 取得したユーザー: Chelsey Dietrich
```

### exhaustMap

`exhaustMap`は、内部Observableが実行中の間、新しい値を無視します。内部Observableが完了した後に初めて新しい値を処理します。これは、重複クリックによる多重送信を防止するようなケースに適しています。

```ts
import { fromEvent } from 'rxjs';
import { exhaustMap, tap, delay } from 'rxjs/operators';
import { ajax } from 'rxjs/ajax';

// 送信ボタン
const submitButton = document.createElement('button');
submitButton.innerText = '送信';
document.body.appendChild(submitButton);

// 送信処理
fromEvent(submitButton, 'click')
  .pipe(
    tap(() => console.log('フォーム送信ボタンがクリックされました')),
    exhaustMap(() => {
      console.log('送信処理開始...');
      // 送信処理をシミュレート
      return ajax
        .post('https://jsonplaceholder.typicode.com/posts', {
          title: 'foo',
          body: 'bar',
          userId: 1,
        })
        .pipe(
          delay(2000) // 2秒の処理時間をシミュレート
        );
    })
  )
  .subscribe(
    (response) => console.log('送信成功:', response),
    (error) => console.error('送信エラー:', error)
  );

// この間にボタンを複数回クリックしても、最初の処理が終わるまで無視される

// 出力:
// フォーム送信ボタンがクリックされました
// 送信処理開始...
// 送信成功: AjaxResponse2 {originalEvent: ProgressEvent, xhr: XMLHttpRequest, request: {…}, type: 'download_load', status: 201, …}
```

## プロパティ変換オペレーター

### buffer系オペレーター

`buffer`系オペレーターは、値を一定の条件でバッファリングしてから一度に出力します。

#### bufferTime

```ts
import { interval } from 'rxjs';
import { bufferTime } from 'rxjs/operators';

// 100msごとに値を発行
const source$ = interval(100);

// 1秒間の値をバッファリング
source$.pipe(
  bufferTime(1000)
).subscribe(buffer => {
  console.log('1秒間に収集された値:', buffer);
});

// 出力:
// 1秒間に収集された値: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// 1秒間に収集された値: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
// ...
```

#### bufferCount

```ts
import { interval } from 'rxjs';
import { bufferCount } from 'rxjs/operators';

// 100msごとに値を発行
const source$ = interval(100);

// 5つの値ごとにバッファリング
source$.pipe(
  bufferCount(5)
).subscribe(buffer => {
  console.log('5つごとの値:', buffer);
});

// 出力:
// 5つごとの値: [0, 1, 2, 3, 4]
// 5つごとの値: [5, 6, 7, 8, 9]
// ...
```

### windowTime

`windowTime`は`bufferTime`に似ていますが、値の配列ではなく、Observableを返します。

```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, map, count } from 'rxjs/operators';

// 100msごとに値を発行
const source$ = interval(100);

// 1秒間の値をウィンドウ化
source$
  .pipe(
    windowTime(1000),
    map((window) =>
      window.pipe(
        // 各ウィンドウ内の値を数える
        count()
      )
    ),
    mergeAll()
  )
  .subscribe((count) => {
    console.log('1秒間の値の数:', count);
  });

// 出力:
// 1秒間の値の数: 10
// 1秒間の値の数: 10
// ...

```

## 実用的な変換パターン

### ユーザー入力のバリデーションと変換

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs/operators';

// 入力フィールド
const emailInput = document.createElement('input');
const emailStatus = document.createElement('p');
document.body.appendChild(emailInput);
document.body.appendChild(emailStatus);

// メールアドレスのバリデーション関数
function isValidEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

// 入力処理
fromEvent(emailInput, 'input')
  .pipe(
    debounceTime(400),
    map((event) => (event.target as HTMLInputElement).value.trim()),
    distinctUntilChanged(),
    map((email) => {
      if (!email) {
        return {
          isValid: false,
          message: 'メールアドレスを入力してください',
          value: email,
        };
      }

      if (!isValidEmail(email)) {
        return {
          isValid: false,
          message: '有効なメールアドレスを入力してください',
          value: email,
        };
      }

      return {
        isValid: true,
        message: 'メールアドレスは有効です',
        value: email,
      };
    })
  )
  .subscribe((result) => {
    if (result.isValid) {
      emailStatus.textContent = '✓ ' + result.message;
      emailStatus.className = 'valid';
    } else {
      emailStatus.textContent = '✗ ' + result.message;
      emailStatus.className = 'invalid';
    }
  });
```

### オブジェクト配列の変換と集計

```ts
import { from } from 'rxjs';
import { map, toArray } from 'rxjs/operators';

// 売上データ
const sales = [
  { product: 'ノートPC', price: 120000, quantity: 3 },
  { product: 'タブレット', price: 45000, quantity: 7 },
  { product: 'スマートフォン', price: 85000, quantity: 4 },
  { product: 'マウス', price: 3500, quantity: 12 },
  { product: 'キーボード', price: 6500, quantity: 8 },
];

// データ変換と集計
from(sales)
  .pipe(
    // 各商品の合計金額を計算
    map((item) => ({
      product: item.product,
      price: item.price,
      quantity: item.quantity,
      total: item.price * item.quantity,
    })),
    // 税込価格を追加
    map((item) => ({
      ...item,
      totalWithTax: Math.round(item.total * 1.1),
    })),
    // 配列に戻す
    toArray(),
    // 合計金額を計算
    map((items) => {
      const grandTotal = items.reduce((sum, item) => sum + item.total, 0);
      const grandTotalWithTax = items.reduce(
        (sum, item) => sum + item.totalWithTax,
        0
      );
      return {
        items,
        grandTotal,
        grandTotalWithTax,
      };
    })
  )
  .subscribe((result) => {
    console.log('商品詳細:', result.items);
    console.log('合計金額(税抜):', result.grandTotal);
    console.log('合計金額(税込):', result.grandTotalWithTax);
  });
// 出力:
// 商品詳細: (5) [{…}, {…}, {…}, {…}, {…}]
// 合計金額(税抜): 1109000
// 合計金額(税込): 1219900
```

### JSONデータの正規化

```ts
import { ajax } from 'rxjs/ajax';
import { map } from 'rxjs/operators';

const resultBox = document.createElement('div');
resultBox.id = 'normalized-results';
document.body.appendChild(resultBox);

ajax
  .getJSON<any[]>('https://jsonplaceholder.typicode.com/users')
  .pipe(
    map((users) => {
      // IDをキーとするオブジェクトに変換
      const normalizedUsers: Record<number, any> = {};
      const userIds: number[] = [];

      users.forEach((user) => {
        normalizedUsers[user.id] = {
          ...user,
          // ネストしたオブジェクトを平坦化
          companyName: user.company.name,
          city: user.address.city,
          street: user.address.street,
          // 不要なネストを削除
          company: undefined,
          address: undefined,
        };
        userIds.push(user.id);
      });

      return {
        entities: normalizedUsers,
        ids: userIds,
      };
    })
  )
  .subscribe((result) => {
    const title = document.createElement('h3');
    title.textContent = '正規化されたユーザーデータ';
    resultBox.appendChild(title);

    result.ids.forEach((id) => {
      const user = result.entities[id];
      const div = document.createElement('div');
      div.innerHTML = `
      <strong>${user.name}</strong><br>
      ユーザー名: @${user.username}<br>
      Email: ${user.email}<br>
      会社: ${user.companyName}<br>
      住所: ${user.city}, ${user.street}<br><br>
    `;
      resultBox.appendChild(div);
    });

    // 特定のIDのユーザーに素早くアクセス可能
    console.log('ユーザーID 3:', result.entities[3]);
  });

```

## 複数の変換の組み合わせ

実際のアプリケーションでは、複数の変換オペレーターを組み合わせて使用することが一般的です。

```ts
import { fromEvent, timer } from 'rxjs';
import {
  switchMap,
  map,
  tap,
  debounceTime,
  takeUntil,
  distinctUntilChanged,
} from 'rxjs/operators';
import { ajax } from 'rxjs/ajax';

type User = {
  id: number;
  name: string;
  username: string;
  email: string;
  company: {
    name: string;
  };
};

// 検索入力
const searchInput = document.createElement('input');
const resultsContainer = document.createElement('p');
const loadingIndicator = document.createElement('p');

document.body.append(searchInput);
document.body.append(resultsContainer);
document.body.append(loadingIndicator);

// 検索処理
fromEvent(searchInput, 'input')
  .pipe(
    // 入力値を取得
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // 300ms待機
    debounceTime(300),
    // 同じ値なら無視
    distinctUntilChanged(),
    // ローディング表示
    tap(() => {
      loadingIndicator.style.display = 'block';
      resultsContainer.innerHTML = '';
    }),
    // APIリクエスト（前のリクエストはキャンセル）
    switchMap((term) => {
      // 空の入力は結果なし
      if (term === '') {
        return [];
      }

      // タイムアウト処理（5秒）
      const timeout$ = timer(5000).pipe(
        tap(() => console.warn('APIレスポンスがタイムアウトしました')),
        map(() => [{ error: 'タイムアウト' }])
      );

      // API呼び出し
      const response$ = ajax
        .getJSON(
          `https://jsonplaceholder.typicode.com/users?username_like=${term}`
        )
        .pipe(
          // 結果を加工
          map((users) =>
            (users as User[]).map((user) => ({
              id: user.id,
              name: user.name,
              username: user.username,
              email: user.email,
              company: user.company.name,
            }))
          ),
          // タイムアウトまでに完了
          takeUntil(timeout$)
        );

      return response$;
    }),
    // ローディング終了
    tap(() => {
      loadingIndicator.style.display = 'none';
    })
  )
  .subscribe((result) => {
    loadingIndicator.style.display = 'none';

    if (Array.isArray(result)) {
      if (result.length === 0) {
        resultsContainer.innerHTML =
          '<div class="no-results">ユーザーが見つかりませんでした</div>';
      } else {
        resultsContainer.innerHTML = result
          .map(
            (user) => `
          <div class="user-card">
            <h3>${user.name}</h3>
            <p>@${user.username}</p>
            <p>${user.email}</p>
            <p>会社: ${user.company}</p>
          </div>
        `
          )
          .join('');
      }
    } else {
      resultsContainer.innerHTML = `<div class="error">⚠️ ${result}</div>`;
    }
  });

```

## まとめ

変換オペレーターはRxJSの中で最も頻繁に使用されるオペレーターの一つであり、非同期データ処理パイプラインの構築に欠かせないツールです。適切なオペレーターを選択することで、複雑なデータフローも宣言的かつ効率的に処理できます。

### 主なポイント

1. 基本的な値の変換は`map`オペレーターを使用
2. 複数の値を集約するには`scan`や`reduce`を使用
3. ネストされた非同期処理には`mergeMap`、`switchMap`、`concatMap`、`exhaustMap`から用途に合わせて選択
4. 複数の値をグループ化するには`buffer`系オペレーターを使用
5. 実際のアプリケーションでは複数のオペレーターを組み合わせて使用

変換オペレーターを適切に組み合わせることで、コードの可読性を保ちながら、複雑なデータ処理ロジックを簡潔に表現できます。