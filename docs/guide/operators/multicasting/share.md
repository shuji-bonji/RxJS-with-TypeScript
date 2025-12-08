---
description: "share()オペレーターを使ったマルチキャスティングの実装方法を解説。複数の購読者で同じObservableを共有し、重複する処理（API呼び出し、計算）を削減します。shareReplay()との違い、Cold/Hot変換、TypeScriptでの型安全な実装を紹介します。"
---

# share - Observableを複数の購読者で共有する

`share()`オペレーターは、RxJSで最も簡単にマルチキャスティングを実装できる演算子です。
複数の購読者が同じデータソースを共有することで、重複する処理（APIリクエスト、計算処理など）を削減できます。

[📘 RxJS公式ドキュメント - `share()`](https://rxjs.dev/api/index/function/share)

## 🔰 基本的な使い方

```typescript
import { interval, share, take, tap } from 'rxjs';

// インターバルでカウントするObservable
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`ソース: ${value}`)),
  share() // マルチキャストを有効化
);

// 最初の購読者
console.log('Observer 1 購読開始');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1: ${value}`)
);

// 2.5秒後に2人目の購読者を追加
setTimeout(() => {
  console.log('Observer 2 購読開始');
  source$.subscribe(value =>
    console.log(`Observer 2: ${value}`)
  );

  // 2.5秒後に購読者1を解除
  setTimeout(() => {
    console.log('Observer 1 購読解除');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

### 実行結果

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
```

**重要なポイント**:
- ソースの処理（`tap`）は1回だけ実行される
- すべての購読者が同じ値を受け取る
- 途中参加した購読者は、参加後の値のみを受け取る

## 💡 share() の仕組み

`share()`は、RxJSの標準的なマルチキャスティングオペレーターです。内部的にはSubjectを使用して複数の購読者にブロードキャストします。

> [!NOTE]
> **RxJS v7以降の変更**: 以前は`multicast()`と`refCount()`の組み合わせとして説明されていましたが、これらのオペレーターはv7で非推奨、v8で削除されました。現在は`share()`が標準的なマルチキャスティング方法です。詳細は[RxJS公式ドキュメント - Multicasting](https://rxjs.dev/deprecations/multicasting)を参照してください。

**動作の流れ**:
- **最初の購読時**: ソースObservableへの接続を開始し、内部Subjectを作成
- **購読者が追加**: 既存の接続を共有（Subjectを通じて値をブロードキャスト）
- **すべての購読者が解除**: ソースへの接続を切断（`resetOnRefCountZero: true`の場合）
- **再購読**: 新しい接続として開始（リセット設定による）

## 🎯 詳細な制御オプション（RxJS 7+）

RxJS 7以降では、`share()`にオプションを渡すことで挙動を細かく制御できます。

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`ソース: ${value}`)),
  share({
    resetOnError: true,       // エラー時にリセット
    resetOnComplete: true,     // 完了時にリセット
    resetOnRefCountZero: true, // 購読者が0になったらリセット
  })
);
```

### オプションの詳細

| オプション | デフォルト | 説明 |
|-----------|----------|------|
| `resetOnError` | `true` | エラー発生時に内部状態をリセット |
| `resetOnComplete` | `true` | ストリーム完了時に内部状態をリセット |
| `resetOnRefCountZero` | `true` | 購読者が0になったら接続を切断 |
| `connector` | `() => new Subject()` | カスタムSubjectを指定 |

### connectorオプションを使った高度な制御

`connector`オプションを使用することで、`shareReplay`と同等の挙動を実現できます。

```typescript
import { interval, ReplaySubject } from 'rxjs';
import { take, share, tap } from 'rxjs';

// ReplaySubjectを使って最新1件をバッファ
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`ソース: ${value}`)),
  share({
    connector: () => new ReplaySubject(1),
    resetOnError: false,
    resetOnComplete: false,
    resetOnRefCountZero: false
  })
);

// 最初の購読者
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 2.5秒後に購読（過去1件を受け取る）
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 2500);
```

**実行結果**:
```
ソース: 0
Observer 1: 0
ソース: 1
Observer 1: 1
Observer 2: 1  // ← 途中参加でも直前の値を受け取る
ソース: 2
Observer 1: 2
Observer 2: 2
...
```

> [!TIP]
> この方法は`shareReplay(1)`の代替として使用できます。`resetOnRefCountZero: false`を設定することで、参照カウントが0になっても接続を維持し、`shareReplay`の「永続的なキャッシュ」の問題を回避できます。

## 📊 share()なしとの比較

### ❌ share()を使わない場合（Cold Observable）

```typescript
import { interval, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`ソース: ${value}`))
);

// 購読者1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 購読者2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**実行結果**:
```
ソース: 0
Observer 1: 0
ソース: 1
Observer 1: 1
ソース: 0    // ← 新しいストリームが開始される
Observer 2: 0
ソース: 2
Observer 1: 2
ソース: 1
Observer 2: 1
ソース: 2
Observer 2: 2
```

各購読者が独立したストリームを持ち、ソース処理が重複実行されます。

### ✅ share()を使った場合（Hot Observable）

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`ソース: ${value}`)),
  share()
);

// 購読者1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 購読者2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**実行結果**:
```
ソース: 0
Observer 1: 0
ソース: 1
Observer 1: 1
Observer 2: 1  // ← 同じストリームを共有
ソース: 2
Observer 1: 2
Observer 2: 2
```

## 💼 実践的なユースケース

### APIリクエストの重複防止

```typescript
import { ajax } from 'rxjs/ajax';
import { share, tap } from 'rxjs';

// ユーザー情報を取得するObservable
const getUser$ = ajax.getJSON('https://jsonplaceholder.typicode.com/users/1').pipe(
  tap(() => console.log('APIリクエスト実行')),
  share() // 複数コンポーネントでの重複リクエストを防ぐ
);

// コンポーネント1
getUser$.subscribe(user => console.log('コンポーネント1:', user));

// コンポーネント2（ほぼ同時にリクエスト）
getUser$.subscribe(user => console.log('コンポーネント2:', user));

// 結果: APIリクエストは1回だけ実行される
```

### 定期的なデータ取得の共有

```typescript
import { timer, share, switchMap, tap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// 5秒ごとにTODOリストを取得（APIリクエストは共有）
const sharedTodos$ = timer(0, 5000).pipe(
  tap(() => console.log('APIリクエスト実行')),
  switchMap(() => ajax.getJSON('https://jsonplaceholder.typicode.com/todos?_limit=3')),
  share() // 複数の購読者でAPIリクエストを共有
);

// 複数のコンポーネントで同じデータストリームを使用
sharedTodos$.subscribe(todos => console.log('コンポーネントA:', todos));
sharedTodos$.subscribe(todos => console.log('コンポーネントB:', todos));

// 結果: 5秒ごとにAPIリクエストは1回だけ実行され、両コンポーネントが同じデータを受け取る
```

## ⚠️ 注意点

1. **タイミングに注意**: 途中参加した購読者は、過去の値を受け取れない
2. **エラー伝播**: エラーが発生すると、すべての購読者に影響する
3. **メモリ管理**: 購読を適切に解除しないとメモリリークの原因になる

## 🔄 関連オペレーター

- **[shareReplay()](/guide/operators/multicasting/shareReplay)** - 過去の値をバッファリングして後続の購読者にも提供
- **[Subject](/guide/subjects/what-is-subject)** - マルチキャスティングの基礎となるクラス

> [!WARNING]
> **非推奨のオペレーター**: `publish()`, `multicast()`, `refCount()`などの旧マルチキャスティングAPIはRxJS v7で非推奨、v8で削除されました。これらの代わりに`share()`または`connectable()`/`connect()`を使用してください。

## まとめ

`share()`オペレーターは、
- 複数の購読者で同じObservableを共有
- APIリクエストや重い処理の重複実行を防ぐ
- 簡単に使えるマルチキャスティングの基本
- RxJS 7+では細かい制御オプションが利用可能

複数のコンポーネントが同じデータソースを必要とする場合に、`share()`を使うことでパフォーマンスを大幅に改善できます。
