---
description: TypeScriptでRxJSを使う際によくある15の間違いとその対処法を、実際のコード例とともに詳しく解説します。Subjectの外部公開、ネストしたsubscribe、メモリリーク、shareReplayの誤用など、実践的なアンチパターンを学びましょう。
---

# よくある間違いと対処法

このページでは、TypeScriptでRxJSを使う際によく見られる15のアンチパターンと、それぞれの解決策を詳しく解説します。

## 目次

1. [Subject の外部公開](#1-subject-の外部公開)
2. [ネストした subscribe（コールバック地獄）](#2-ネストした-subscribe-コールバック地獄)
3. [unsubscribe 忘れ（メモリリーク）](#3-unsubscribe-忘れ-メモリリーク)
4. [shareReplay の誤用](#4-sharereplay-の誤用)
5. [map での副作用](#5-map-での副作用)
6. [Cold/Hot Observable の違いの無視](#6-cold-hot-observable-の違いの無視)
7. [Promise と Observable の不適切な混在](#7-promise-と-observable-の不適切な混在)
8. [バックプレッシャーの無視](#8-バックプレッシャーの無視)
9. [エラーの握りつぶし](#9-エラーの握りつぶし)
10. [DOM イベントサブスクリプションのリーク](#10-dom-イベントサブスクリプションのリーク)
11. [型安全性の欠如（any の多用）](#11-型安全性の欠如-any-の多用)
12. [不適切なオペレーター選択](#12-不適切なオペレーター選択)
13. [過度な複雑化](#13-過度な複雑化)
14. [subscribe 内での状態変更](#14-subscribe-内での状態変更)
15. [テストの欠如](#15-テストの欠如)


## 1. Subject の外部公開

### 問題

`Subject` をそのまま公開すると、外部から `next()` を呼ばれてしまい、状態管理が予測不可能になります。

### ❌ 悪い例

```ts
import { Subject } from 'rxjs';

// Subject をそのまま export
export const cartChanged$ = new Subject<void>();

// 別のファイルから誰でも next() を呼べてしまう
cartChanged$.next(); // 予期しないタイミングで呼ばれる可能性
```

### ✅ 良い例

```ts
import { BehaviorSubject, Observable } from 'rxjs';

class CartStore {
  private readonly _items$ = new BehaviorSubject<string[]>([]);

  // 読み取り専用の Observable として公開
  readonly items$: Observable<string[]> = this._items$.asObservable();

  // 状態変更は専用メソッドで制御
  add(item: string): void {
    this._items$.next([...this._items$.value, item]);
  }

  remove(item: string): void {
    this._items$.next(
      this._items$.value.filter(i => i !== item)
    );
  }
}

export const cartStore = new CartStore();
```

### 解説

- `asObservable()` で読み取り専用の `Observable` に変換
- 状態変更は専用メソッド経由でのみ可能にする
- 変更のトレーサビリティが向上し、デバッグが容易に


## 2. ネストした subscribe（コールバック地獄）

### 問題

`subscribe` の中でさらに `subscribe` を呼ぶと、コールバック地獄に陥り、エラー処理やキャンセル処理が複雑になります。

### ❌ 悪い例

```ts
import { of } from 'rxjs';

// API 呼び出しのシミュレーション
function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
}

// ネストした subscribe
apiA().subscribe(a => {
  apiB(a.id).subscribe(b => {
    apiC(b.token).subscribe(result => {
      console.log('done', result);
    });
  });
});
```

### ✅ 良い例

```ts
import { of } from 'rxjs';
import { switchMap } from 'rxjs';

function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
};


// 高階オペレーターを使ってフラット化
apiA().pipe(
  switchMap(a => apiB(a.id)),
  switchMap(b => apiC(b.token))
).subscribe(result => {
  console.log('done', result);
});
```

### 解説

- `switchMap`、`mergeMap`、`concatMap` などの高階オペレーターを使用
- エラー処理が一箇所で可能
- 購読解除も一度で済む
- コードの可読性が向上


## 3. unsubscribe 忘れ（メモリリーク）

### 問題

無限ストリーム（イベントリスナーなど）の購読を解除しないと、メモリリークが発生します。

### ❌ 悪い例

```ts
import { fromEvent } from 'rxjs';

// コンポーネントの初期化時
function setupResizeHandler() {
  fromEvent(window, 'resize').subscribe(() => {
    console.log('resized');
  });
  // 購読を解除していない！
}

// コンポーネントが破棄されてもイベントリスナーが残り続ける
```

### ✅ 良い例

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeUntil, finalize } from 'rxjs';

class MyComponent {
  private readonly destroy$ = new Subject<void>();

  ngOnInit(): void {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$),
      finalize(() => console.log('cleanup'))
    ).subscribe(() => {
      console.log('resized');
    });
  }

  ngOnDestroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### ✅ 別の良い例（Subscription を使う方法）

```ts
import { fromEvent, Subscription } from 'rxjs';

class MyComponent {
  private subscription = new Subscription();

  ngOnInit(): void {
    this.subscription.add(
      fromEvent(window, 'resize').subscribe(() => {
        console.log('resized');
      })
    );
  }

  ngOnDestroy(): void {
    this.subscription.unsubscribe();
  }
}
```

### 解説

- `takeUntil` パターンが推奨される（宣言的で明確）
- `Subscription` を使った手動管理も有効
- コンポーネント破棄時に必ず購読解除を実行


## 4. shareReplay の誤用

### 問題

`shareReplay` の動作を理解せずに使うと、古いデータが再生されたり、メモリリークが発生したりします。

### ❌ 悪い例

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// バッファサイズを無制限にしてしまう
const shared$ = interval(1000).pipe(
  shareReplay() // デフォルトは無制限バッファ
);

// 購読者がいなくなっても値がメモリに残り続ける
```

### ✅ 良い例

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// バッファサイズと参照カウントを明示的に指定
const shared$ = interval(1000).pipe(
  take(10),
  shareReplay({
    bufferSize: 1,
    refCount: true // 購読者がいなくなったらリソース解放
  })
);
```

### 解説

- `bufferSize` を明示的に指定（通常は 1）
- `refCount: true` で購読者がいなくなったら自動解放
- HTTP リクエストなど、完了するストリームでは `shareReplay({ bufferSize: 1, refCount: true })` が安全


## 5. map での副作用

### 問題

`map` オペレーター内で状態を変更すると、予測不可能な動作を引き起こします。

### ❌ 悪い例

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

let counter = 0;

const source$ = of(1, 2, 3).pipe(
  map(value => {
    counter++; // 副作用！
    return value * 2;
  })
);

source$.subscribe(console.log);
source$.subscribe(console.log); // counter が予期せず増加
```

### ✅ 良い例

```ts
import { of } from 'rxjs';
import { map, tap, scan } from 'rxjs';

// 純粋な変換のみ
const source$ = of(1, 2, 3).pipe(
  map(value => value * 2)
);

// 副作用は tap で分離
const withLogging$ = source$.pipe(
  tap(value => console.log('Processing:', value))
);

// 状態の蓄積は scan を使う
const withCounter$ = of(1, 2, 3).pipe(
  scan((acc, value) => ({ count: acc.count + 1, value }), { count: 0, value: 0 })
);
```

### 解説

- `map` は純粋関数として使用
- 副作用（ログ、API 呼び出しなど）は `tap` に分離
- 状態の蓄積は `scan` や `reduce` を使用


## 6. Cold/Hot Observable の違いの無視

### 問題

Observable が Cold か Hot かを理解せずに使うと、重複実行や予期しない動作を引き起こします。

### ❌ 悪い例

```ts
import { ajax } from 'rxjs/ajax';

// Cold Observable - 購読ごとに HTTP リクエストが実行される
const data$ = ajax.getJSON('https://api.example.com/data');

data$.subscribe(console.log); // リクエスト 1
data$.subscribe(console.log); // リクエスト 2（無駄な重複）
```

### ✅ 良い例

```ts
import { ajax } from 'rxjs/ajax';
import { shareReplay } from 'rxjs';

// Hot Observable に変換して共有
const data$ = ajax.getJSON('https://api.example.com/data').pipe(
  shareReplay({ bufferSize: 1, refCount: true })
);

data$.subscribe(console.log); // リクエスト 1
data$.subscribe(console.log); // キャッシュされた結果を使用
```

### 解説

- Cold Observable: 購読ごとに実行される（`of`, `from`, `fromEvent`, `ajax` など）
- Hot Observable: 購読に関係なく実行される（`Subject`, マルチキャスト化したObservable など）
- `share` / `shareReplay` で Cold を Hot に変換可能


## 7. Promise と Observable の不適切な混在

### 問題

Promise と Observable を適切に変換せずに混在させると、エラーハンドリングやキャンセル処理が不完全になります。

### ❌ 悪い例

```ts
import { from } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Promise をそのまま使っている
from(fetchData()).subscribe(data => {
  fetchData().then(moreData => { // ネストした Promise
    console.log(data, moreData);
  });
});
```

### ✅ 良い例

```ts
import { from } from 'rxjs';
import { switchMap } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Promise を Observable に変換して統一
from(fetchData()).pipe(
  switchMap(() => from(fetchData()))
).subscribe(moreData => {
  console.log(moreData);
});
```

### 解説

- `from` で Promise を Observable に変換
- Observable パイプライン内で統一的に処理
- エラーハンドリングとキャンセルが容易に


## 8. バックプレッシャーの無視

### 問題

高頻度で発生するイベントを制御せずに処理すると、パフォーマンスが低下します。

### ❌ 悪い例

```ts
import { fromEvent } from 'rxjs';

// 入力イベントをそのまま処理
fromEvent(document.getElementById('search'), 'input').subscribe(event => {
  // 入力のたびに API 呼び出し（過負荷）
  searchAPI((event.target as HTMLInputElement).value);
});

function searchAPI(query: string): void {
  console.log('Searching for:', query);
}
```

### ✅ 良い例

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, distinctUntilChanged, map, switchMap } from 'rxjs';

// デバウンスとキャンセルを適用
fromEvent(document.getElementById('search'), 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // 300ms 待機
  distinctUntilChanged(), // 値が変わった時のみ
  switchMap(query => searchAPI(query)) // 古いリクエストはキャンセル
).subscribe(results => {
  console.log('Results:', results);
});
```

### 解説

- `debounceTime` で一定時間待機
- `throttleTime` で最大頻度を制限
- `distinctUntilChanged` で重複を除外
- `switchMap` で古いリクエストをキャンセル


## 9. エラーの握りつぶし

### 問題

エラーを適切に処理しないと、デバッグが困難になり、ユーザー体験が低下します。

### ❌ 悪い例

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

// エラーを無視
ajax.getJSON('https://api.example.com/data').pipe(
  catchError(() => of(null)) // エラー情報が失われる
).subscribe(data => {
  console.log(data); // null が来ても原因不明
});
```

### ✅ 良い例

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

interface ApiResponse {
  data: unknown;
  error?: string;
}

ajax.getJSON<ApiResponse>('https://api.example.com/data').pipe(
  catchError(error => {
    console.error('API Error:', error);
    // ユーザーに通知
    showErrorToast('データの取得に失敗しました');
    // エラー情報を含む代替値を返す
    return of({ data: null, error: error.message } as ApiResponse);
  })
).subscribe((response) => {
  if (response.error) {
    console.log('Fallback mode due to:', response.error);
  }
});

function showErrorToast(message: string): void {
  console.log('Toast:', message);
}
```

### 解説

- エラーをログに記録
- ユーザーにフィードバックを提供
- エラー情報を含む代替値を返す
- リトライ戦略を検討（`retry`, `retryWhen`）


## 10. DOM イベントサブスクリプションのリーク

### 問題

DOM イベントリスナーを適切に解放しないと、メモリリークが発生します。

### ❌ 悪い例

```ts
import { fromEvent } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;

  constructor() {
    this.button = document.createElement('button');

    // イベントリスナーを登録
    fromEvent(this.button, 'click').subscribe(() => {
      console.log('clicked');
    });

    // 購読解除していない
  }

  destroy(): void {
    this.button.remove();
    // リスナーが残ったまま
  }
}
```

### ✅ 良い例

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;
  private readonly destroy$ = new Subject<void>();

  constructor() {
    this.button = document.createElement('button');

    fromEvent(this.button, 'click').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => {
      console.log('clicked');
    });
  }

  destroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
    this.button.remove();
  }
}
```

### 解説

- `takeUntil` パターンで確実に購読解除
- コンポーネント破棄時に `destroy$` を発火
- DOM 要素削除前にリスナーを解放


## 11. 型安全性の欠如（any の多用）

### 問題

`any` を多用すると、TypeScript の型チェックが無効化され、実行時エラーが発生しやすくなります。

### ❌ 悪い例

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

function fetchUser(): Observable<any> {
  return new Observable(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// 型チェックが効かない
fetchUser().pipe(
  map(user => user.naem) // タイポ！実行時まで気づかない
).subscribe(console.log);
```

### ✅ 良い例

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

interface User {
  name: string;
  age: number;
}

function fetchUser(): Observable<User> {
  return new Observable<User>(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// 型チェックが効く
fetchUser().pipe(
  map(user => user.name) // コンパイル時にエラー検出
).subscribe(console.log);
```

### 解説

- インターフェースや型エイリアスを定義
- `Observable<T>` の型パラメータを明示
- TypeScript の型推論を最大限活用


## 12. 不適切なオペレーター選択

### 問題

目的に合わないオペレーターを使うと、非効率だったり予期しない動作を引き起こします。

### ❌ 悪い例

```ts
import { fromEvent } from 'rxjs';
import { mergeMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// ボタンクリックごとに検索（古いリクエストがキャンセルされない）
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  mergeMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### ✅ 良い例

```ts
import { fromEvent } from 'rxjs';
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// 最新のリクエストのみを処理（古いリクエストは自動キャンセル）
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  switchMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### 主要な高階オペレーターの使い分け

| オペレーター | 用途 |
|---|---|
| `switchMap` | 最新のストリームのみ処理（検索、オートコンプリート） |
| `mergeMap` | 並列処理（順序不問） |
| `concatMap` | 順次処理（順序が重要） |
| `exhaustMap` | 実行中は新しい入力を無視（ボタン連打防止） |

### 解説

- 各オペレーターの挙動を理解
- ユースケースに応じた適切な選択
- 詳細は [変換オペレーター](/guide/operators/transformation/) を参照


## 13. 過度な複雑化

### 問題

シンプルに書ける処理を、RxJS で過度に複雑化してしまうケース。

### ❌ 悪い例

```ts
import { Observable, of } from 'rxjs';
import { map, mergeMap, toArray } from 'rxjs';

// 単純な配列変換を RxJS で複雑化
function doubleNumbers(numbers: number[]): Observable<number[]> {
  return of(numbers).pipe(
    mergeMap(arr => of(...arr)),
    map(n => n * 2),
    toArray()
  );
}
```

### ✅ 良い例

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// 配列処理は普通の JavaScript で十分
function doubleNumbers(numbers: number[]): number[] {
  return numbers.map(n => n * 2);
}

// RxJS は非同期・イベント駆動の処理に使う
const button = document.getElementById('calc-btn') as HTMLButtonElement;
const numbers = [1, 2, 3, 4, 5];

fromEvent(button, 'click').pipe(
  map(() => doubleNumbers(numbers))
).subscribe(result => console.log(result));
```

### 解説

- RxJS は非同期処理やイベントストリームに使う
- 同期的な配列処理は通常の JavaScript で十分
- 複雑さとメリットのバランスを考慮


## 14. subscribe 内での状態変更

### 問題

`subscribe` 内で直接状態を変更すると、テストが困難になり、バグの原因になります。

### ❌ 悪い例

```ts
import { interval } from 'rxjs';

class Counter {
  count = 0;

  start(): void {
    interval(1000).subscribe(() => {
      this.count++; // subscribe 内で状態変更
      this.updateUI();
    });
  }

  updateUI(): void {
    console.log('Count:', this.count);
  }
}
```

### ✅ 良い例

```ts
import { interval, BehaviorSubject } from 'rxjs';
import { scan, tap } from 'rxjs';

class Counter {
  private readonly count$ = new BehaviorSubject<number>(0);

  start(): void {
    interval(1000).pipe(
      scan(acc => acc + 1, 0),
      tap(count => this.count$.next(count))
    ).subscribe();

    // UI は count$ を購読
    this.count$.subscribe(count => this.updateUI(count));
  }

  updateUI(count: number): void {
    console.log('Count:', count);
  }
}
```

### 解説

- 状態は `BehaviorSubject` や `scan` で管理
- `subscribe` はトリガーとして使用
- テスタブルでリアクティブな設計


## 15. テストの欠如

### 問題

RxJS のコードをテストせずに本番環境にデプロイすると、リグレッションが発生しやすくなります。

### ❌ 悪い例

```ts
import { interval } from 'rxjs';
import { map, filter } from 'rxjs';

// テストなしでデプロイ
export function getEvenNumbers() {
  return interval(1000).pipe(
    filter(n => n % 2 === 0),
    map(n => n * 2)
  );
}
```

### ✅ 良い例

```ts
import { TestScheduler } from 'rxjs/testing';
import { getEvenNumbers } from './numbers';

describe('getEvenNumbers', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('should emit only even numbers doubled', () => {
    scheduler.run(({ expectObservable }) => {
      const expected = '1s 0 1s 4 1s 8';
      expectObservable(getEvenNumbers()).toBe(expected);
    });
  });
});
```

### 解説

- `TestScheduler` でマーブルテストを実施
- 非同期処理を同期的にテスト可能
- 詳細は [テスト手法](/guide/testing/unit-tests) を参照


## まとめ

これらの15のアンチパターンを理解し、避けることで、より堅牢で保守性の高いRxJSコードを書けるようになります。

## 参考文献

このアンチパターン集は、以下の信頼できるソースを参考に作成されています。

### 公式ドキュメント・リポジトリ
- **[RxJS 公式ドキュメント](https://rxjs.dev/)** - オペレーターとAPIの公式リファレンス
- **[GitHub Issue #5931](https://github.com/ReactiveX/rxjs/issues/5931)** - shareReplayのメモリリーク問題に関する議論

### アンチパターンとベストプラクティス
- **[RxJS in Angular - Antipattern 1: Nested subscriptions](https://www.thinktecture.com/en/angular/rxjs-antipattern-1-nested-subs/)** - Thinktecture AG
- **[RxJS in Angular - Antipattern 2: Stateful Streams](https://www.thinktecture.com/en/angular/rxjs-antipattern-2-state/)** - Thinktecture AG
- **[RxJS Best Practices in Angular 16 (2025)](https://www.infoq.com/articles/rxjs-angular16-best-practices/)** - InfoQ (2025年5月)
- **[RxJS: Why memory leaks occur when using a Subject](https://angularindepth.com/posts/1433/rxjs-why-memory-leaks-occur-when-using-a-subject)** - Angular In Depth
- **[RxJS Antipatterns](https://brianflove.com/posts/2017-11-01-ngrx-anti-patterns/)** - Brian Love

### 追加リソース
- **[Learn RxJS](https://www.learnrxjs.io/)** - オペレーターとパターンの実践的ガイド
- **[RxJS Marbles](https://rxmarbles.com/)** - オペレーターの視覚的な理解

## コードレビューに活用

自分のコードがアンチパターンに該当していないか確認しましょう。

👉 **[アンチパターン回避チェックリスト](./checklist)** - 15の確認項目でコードを見直す

各チェック項目から、このページの対応するアンチパターンの詳細へ直接ジャンプできます。

## 次のステップ

- **[エラーハンドリング](/guide/error-handling/strategies)** - より詳細なエラー処理戦略を学ぶ
- **[テスト手法](/guide/testing/unit-tests)** - RxJS コードの効果的なテスト方法を習得
- **[オペレーターの理解](/guide/operators/)** - 各オペレーターの詳細な使い方を学ぶ

これらのベストプラクティスを日々のコーディングに取り入れて、品質の高いRxJSコードを書いていきましょう！
