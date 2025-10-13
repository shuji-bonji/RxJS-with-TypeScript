---
description: 実際のアプリケーションで役立つRxJSの変換オペレーターの使い方を、ユーザー入力処理やAPIレスポンス整形、データ集計など具体的な事例で解説します。
---

# 実用的な変換パターン

変換オペレーターは、RxJSにおいて最も頻繁に使用されるオペレーター群の一つです。  
リアクティブプログラミングにおいて、データを柔軟に加工・変形するために不可欠な役割を果たします。

このセクションでは、典型的な実践例を紹介しながら、変換オペレーターの活用パターンを整理します。


## 💬 典型的な活用パターン

| パターン | 代表的なオペレーター | 説明 |
|:---|:---|:---|
| 値の単純変換 | `map` | 各値に変換関数を適用 |
| 累積・集計処理 | `scan`, `reduce` | 値を逐次的に蓄積 |
| ネスト非同期処理 | `mergeMap`, `switchMap`, `concatMap`, `exhaustMap` | Observableを生成・結合 |
| バッチ処理・グループ化 | `bufferTime`, `bufferCount`, `windowTime` | まとめて処理・分割管理 |
| プロパティ抽出 | `pluck` | オブジェクトから特定フィールド抽出 |


## ユーザー入力のバリデーションと変換

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

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

## オブジェクト配列の変換と集計

```ts
import { from } from 'rxjs';
import { map, toArray } from 'rxjs';

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

## JSONデータの正規化

```ts
import { ajax } from 'rxjs/ajax';
import { map } from 'rxjs';

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
} from 'rxjs';
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

## 🧠 まとめ

- 単純な変換は `map`
- 非同期を扱うなら `mergeMap`・`switchMap`・`concatMap`・`exhaustMap`
- バッチ処理は `bufferTime`・`bufferCount`
- プロパティ抽出は `pluck`
- 実際のアプリでは**これらを組み合わせることが常態**

変換オペレーターをマスターすると、複雑な非同期データフローも  
直感的かつ宣言的に扱えるようになります！
