---
description: 基本的なObservable作成のためのCreation Functionsについて解説します。of、from、fromEvent、interval、timerを使い、単一値、配列、Promise、イベント、タイマーなど多様なデータソースからObservableを作成する方法を学びます。TypeScriptの型安全性とともに実装でき、RxJSの基礎となる重要な概念です。
---

# 基本作成系 Creation Functions

最も基本的で頻繁に使用されるCreation Functionsです。データ、配列、イベント、時間ベースのObservableを簡単に作成できます。

## 基本作成系 Creation Functions とは

基本作成系のCreation Functionsは、さまざまなデータソースから単一のObservableを作成するための関数です。これらはRxJSを使う上で最も基礎となる関数群で、ほぼすべてのRxJSコードで使用されます。

以下の表で、各Creation Functionの特徴と使い分けを確認してください。

## 主要な基本作成系 Creation Functions

| Function | 説明 | ユースケース |
|----------|------|-------------|
| **[of](/guide/creation-functions/basic/of)** | 指定した値を順番に発行 | 固定値のテスト、モック作成 |
| **[from](/guide/creation-functions/basic/from)** | 配列、Promise等から変換 | 既存データのストリーム化 |
| **[fromEvent](/guide/creation-functions/basic/fromEvent)** | イベントをObservableに変換 | DOM イベント、Node.js EventEmitter |
| **[interval](/guide/creation-functions/basic/interval)** | 指定間隔で連続発行 | ポーリング、定期実行 |
| **[timer](/guide/creation-functions/basic/timer)** | 遅延後に発行開始 | 遅延実行、タイムアウト |

## 使い分けの基準

基本作成系Creation Functionsの選択は、データソースの種類によって決まります。

### 1. データの種類

- **静的な値**: `of()` - 直接値を指定してObservableを作成
- **配列やイテラブル**: `from()` - 既存のコレクションをストリームに変換
- **Promise**: `from()` - 非同期処理をObservableに変換
- **イベント**: `fromEvent()` - イベントリスナーをObservableに変換
- **時間ベース**: `interval()`, `timer()` - 時間経過に基づいて値を発行

### 2. 発行のタイミング

- **即座に発行**: `of()`, `from()` - 購読と同時に値を発行開始
- **イベント発生時**: `fromEvent()` - イベントが発生するたびに発行
- **定期的に発行**: `interval()` - 一定間隔で連続発行
- **遅延後に発行**: `timer()` - 指定時間後に発行開始

### 3. 完了のタイミング

- **すぐに完了**: `of()`, `from()` - すべての値を発行後に完了
- **完了しない**: `fromEvent()`, `interval()` - unsubscribeまで継続
- **1回だけ発行して完了**: `timer(delay)` - 1つの値を発行後に完了

## 実践的な使用例

### of() - 固定値のテスト

```typescript
import { of } from 'rxjs';

// テストデータを作成
const mockUser$ = of({ id: 1, name: 'Test User' });

mockUser$.subscribe(user => console.log(user));
// 出力: { id: 1, name: 'Test User' }
```

### from() - 配列をストリーム化

```typescript
import { from } from 'rxjs';
import { map } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

numbers$.pipe(
  map(n => n * 2)
).subscribe(console.log);
// 出力: 2, 4, 6, 8, 10
```

### fromEvent() - クリックイベント

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Button clicked!'));
```

### interval() - ポーリング

```typescript
import { interval } from 'rxjs';
import { switchMap } from 'rxjs';

// 5秒ごとにAPIをポーリング
interval(5000).pipe(
  switchMap(() => fetchData())
).subscribe(data => console.log('Updated:', data));
```

### timer() - 遅延実行

```typescript
import { timer } from 'rxjs';

// 3秒後に実行
timer(3000).subscribe(() => console.log('3 seconds passed'));
```

## メモリリークに注意

基本作成系Creation Functionsを使用する際は、適切な購読解除が重要です。

> [!WARNING]
> `fromEvent()`, `interval()`, および周期的に発行する `timer(delay, period)` は完了しないため、コンポーネントの破棄時に必ず`unsubscribe()`するか、`takeUntil()`などで自動解除する必要があります。
>
> 注: `timer(delay)` のように第2引数を省略した場合は1回発行後に自動完了します。

```typescript
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => console.log('Window resized'));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Cold から Hot への変換

上記の表に示した通り、**全ての基本作成系Creation Functionsは Cold Observable を生成します**。購読するたびに独立した実行が開始されます。

しかし、以下のマルチキャスト系オペレーターを使用することで、**Cold Observable を Hot Observable に変換**できます。

### Hot化する条件とオペレーター

| オペレーター | 動作 | ユースケース |
|-------------|------|-------------|
| **share()** | マルチキャスト + 自動接続/切断 | 複数購読者でHTTPリクエストを共有 |
| **shareReplay(n)** | 最新n個の値をキャッシュして新規購読者に配信 | APIレスポンスのキャッシュ |
| **publish() + connect()** | 手動でマルチキャスト開始 | 購読者が揃ってから実行開始 |
| **multicast(subject)** | カスタムSubjectでマルチキャスト | 高度な制御が必要な場合 |

### 実践例

```typescript
import { interval } from 'rxjs';
import { take, share } from 'rxjs';

// ❄️ Cold - 購読ごとに独立したタイマー
const cold$ = interval(1000).pipe(take(3));

cold$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  cold$.subscribe(val => console.log('B:', val));
}, 1500);

// 出力:
// A: 0 (0秒後)
// A: 1 (1秒後)
// B: 0 (1.5秒後) ← Bは独立して0から開始
// A: 2 (2秒後)
// B: 1 (2.5秒後)

// 🔥 Hot - 購読者間でタイマーを共有
const hot$ = interval(1000).pipe(take(3), share());

hot$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  hot$.subscribe(val => console.log('B:', val));
}, 1500);

// 出力:
// A: 0 (0秒後)
// A: 1 (1秒後)
// A: 2, B: 2 (2秒後) ← Bは途中から参加、同じ値を受け取る
```

> [!TIP]
> **Hot化が必要なケース**:
> - HTTPリクエストを複数の購読者で共有したい
> - WebSocketやサーバー接続を1つだけ維持したい
> - 高コストな計算結果を複数箇所で使いたい
>
> 詳しくは **Subjectとマルチキャスト** の章（Chapter 5）を参照してください。

## Pipeable Operator との関係

基本作成系Creation Functionsには、直接対応するPipeable Operatorはありません。これらは常にCreation Functionとして使用されます。

ただし、以下のようなパターンでPipeable Operatorと組み合わせて使用されます。

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, switchMap } from 'rxjs';

// ユーザー入力 → 300ms待機 → API呼び出し
fromEvent(input, 'input').pipe(
  debounceTime(300),
  switchMap(event => fetchSuggestions(event.target.value))
).subscribe(suggestions => console.log(suggestions));
```

## 次のステップ

各Creation Functionの詳細な動作と実践例を学ぶには、上記の表からリンクをクリックしてください。

また、[結合系 Creation Functions](/guide/creation-functions/combination/)、[選択・分割系 Creation Functions](/guide/creation-functions/selection/)、[条件分岐系 Creation Functions](/guide/creation-functions/conditional/)も併せて学習することで、Creation Functionsの全体像を理解できます。
