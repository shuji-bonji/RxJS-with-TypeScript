# TypeScriptとRxJSの基本連携

この文書では、TypeScriptでRxJSを使い始めるための基本的な連携方法について説明します。

## このドキュメントで学べること

- TypeScriptでのObservable型の扱い方
- 型安全なRxJSプログラミングの基礎
- TypeScriptの型システムがRxJSにもたらす利点

TypeScriptとRxJSを組み合わせることで、型安全性を保ちながら非同期プログラミングを行うことができます。この文書では、TypeScriptでRxJSを基本的に活用するための方法を紹介します。

## 型定義の活用

### Observable型の指定

TypeScriptでRxJSを使用する最大の利点は、Observableに流れる値の型を明示的に定義できることです。

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs/operators';

// 明示的な型定義
const numbers$: Observable<number> = of(1, 2, 3);

// ジェネリック型を使用した変換
interface User {
  id: number;
  name: string;
}

const users$: Observable<User> = of(
  { id: 1, name: '山田' },
  { id: 2, name: '佐藤' }
);

// 型が変換されるオペレーション
const userNames$: Observable<string> = users$.pipe(
  map(user => user.name)
);
```

TypeScriptでは、変数名に`$`サフィックスを付けることで、それがObservableであることを視覚的に示す慣習があります。これは必須ではありませんが、コードの読みやすさを向上させるのに役立ちます。

## インターフェースと型エイリアス

複雑なデータ構造を扱う場合、インターフェースや型エイリアスを定義すると便利です。

```ts
// イベントの型定義
interface AppEvent {
  type: string;
  payload: unknown;
}

// 特定のイベント型
interface UserLoginEvent extends AppEvent {
  type: 'USER_LOGIN';
  payload: {
    userId: string;
    timestamp: number;
  };
}

interface DataUpdateEvent extends AppEvent {
  type: 'DATA_UPDATE';
  payload: {
    items: Array<{ id: string; value: number }>;
  };
}

// イベントの型合成
type ApplicationEvent = UserLoginEvent | DataUpdateEvent;

// イベントバスの実装
const eventBus$ = new Subject<ApplicationEvent>();

// 型安全なイベント発行
eventBus$.next({
  type: 'USER_LOGIN',
  payload: {
    userId: 'user123',
    timestamp: Date.now()
  }
});
```

## RxJSのimportの最適化

RxJSをTypeScriptプロジェクトで使用する際、importの方法も重要です。

```ts
// 推奨される方法
import { Observable, of, from } from 'rxjs';
import { map, filter, catchError } from 'rxjs/operators';
```

## 型の恩恵を受けるシンプルな例

例として、ユーザー検索機能を作成してみましょう。TypeScriptの型システムによって、コード補完や型チェックが可能になります。

```ts
import { fromEvent, Observable } from 'rxjs';
import { map, filter, debounceTime, switchMap } from 'rxjs/operators';

// ユーザー型の定義
interface User {
  id: number;
  name: string;
  email: string;
}

// APIリクエスト関数
function searchUsers(term: string): Observable<User[]> {
  // 実際のAPIリクエストの代わりに、モックデータを返す
  return of([
    { id: 1, name: '山田太郎', email: 'yamada@example.com' },
    { id: 2, name: '佐藤花子', email: 'sato@example.com' }
  ]).pipe(
    // 検索語に基づいてフィルタリング
    map(users => users.filter(user => 
      user.name.includes(term) || user.email.includes(term)
    ))
  );
}

// 検索フォームの設定
const searchInput = document.querySelector<HTMLInputElement>('#search');
if (searchInput) {
  // 入力イベントのストリーム
  const searchTerms$ = fromEvent<InputEvent>(searchInput, 'input').pipe(
    map(event => (event.target as HTMLInputElement).value),
    filter(term => term.length > 2),  // 3文字以上の場合のみ
    debounceTime(300)  // タイピング中のリクエスト防止
  );

  // 検索結果の購読
  const subscription = searchTerms$.pipe(
    switchMap(term => searchUsers(term))
  ).subscribe({
    next: (users) => {
      console.log('検索結果:', users);
      // ここで結果をUIに表示
    },
    error: (err) => console.error('検索エラー:', err)
  });

  // コンポーネントのクリーンアップ時に購読解除
  // componentWillUnmount() {
  //   subscription.unsubscribe();
  // }
}
```

## tsconfig.jsonの基本設定

RxJSとTypeScriptを効果的に使用するには、適切なtsconfig.jsonの設定が重要です。

```json
{
  "compilerOptions": {
    "target": "es2020",
    "module": "esnext",
    "moduleResolution": "node",
    "strict": true,
    "noImplicitAny": true,
    "strictNullChecks": true,
    "esModuleInterop": true,
    "sourceMap": true,
    "lib": ["es2020", "dom"]
  }
}
```

### 特に重要な設定

- `strict`: 型チェックを厳格にします
- `noImplicitAny`: 暗黙のany型を禁止します
- `strictNullChecks`: null/undefinedを明示的に扱う必要があります

これらの設定により、TypeScriptの型システムの恩恵を最大限に受けることができます。

## まとめ

TypeScriptとRxJSを組み合わせることで、次のような利点があります。

- 型安全な非同期プログラミング
- IDEの支援による開発効率の向上
- コンパイル時の型チェックによるエラーの早期発見
- 自己文書化されたコード

高度な型機能やカスタムオペレーターの定義など、さらに進んだトピックについては、「7. TypeScriptとRxJSの高度な連携」セクションで詳しく説明します。
