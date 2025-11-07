---
description: RxJSの制御系 Creation Functions である scheduled と using の概要、違い、使い分けのガイドラインについて解説します。
---

# 制御系 Creation Functions

RxJSでは、Observable の実行タイミングやリソース管理を細かく制御するための Creation Functions が提供されています。このセクションでは、`scheduled()` と `using()` の2つの関数について詳しく解説します。

## 制御系 Creation Functions とは

制御系 Creation Functions は、Observable の動作をより細かく制御するための関数群です。実行タイミングの制御（スケジューラー）やリソースのライフサイクル管理など、高度なユースケースに対応します。

### 主な特徴

- **実行タイミングの制御**: スケジューラーを使って同期・非同期実行を切り替え
- **リソース管理**: Observable のライフサイクルに合わせた自動リソース解放
- **テスト容易性**: スケジューラーを切り替えることでテストが容易に
- **パフォーマンス最適化**: 実行タイミングを制御してUIのブロッキングを回避

## 制御系 Creation Functions の一覧

| 関数 | 説明 | 主な用途 |
|------|------|---------|
| [scheduled()](/guide/creation-functions/control/scheduled) | スケジューラーを指定してObservableを生成 | 実行タイミングの制御、テスト |
| [using()](/guide/creation-functions/control/using) | リソース制御付きObservable | WebSocket、ファイルハンドルなどのリソース管理 |

## scheduled() の基本

`scheduled()` は、既存のデータソース（配列、Promise、Iterableなど）から Observable を生成する際に、スケジューラーを明示的に指定できる関数です。

### 基本的な使い方

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// 配列を非同期で発行
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('購読開始');
observable$.subscribe({
  next: val => console.log('値:', val),
  complete: () => console.log('完了')
});
console.log('購読終了');

// 出力:
// 購読開始
// 購読終了
// 値: 1
// 値: 2
// 値: 3
// 完了
```

> [!NOTE]
> `asyncScheduler` を使用すると、値の発行が非同期になります。これにより、購読処理がメインスレッドをブロックすることなく実行されます。

## using() の基本

`using()` は、Observable のライフサイクルに合わせてリソースを自動的に作成・解放する関数です。購読開始時にリソースを作成し、購読終了時（`complete` または `unsubscribe`）に自動的に解放します。

### 基本的な使い方

```typescript
import { using, interval, Subscription, take } from 'rxjs';
const resource$ = using(
  // リソースファクトリー: 購読開始時に実行
  () => {
    console.log('リソース作成');
    return new Subscription(() => console.log('リソース解放'));
  },
  // Observable ファクトリー: リソースを使ってObservableを作成
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('値:', value),
  complete: () => console.log('完了')
});

// 出力:
// リソース作成
// 値: 0
// 値: 1
// 値: 2
// 完了
// リソース解放
```

> [!IMPORTANT]
> `using()` は、購読終了時に自動的にリソースを解放するため、メモリリークを防ぐことができます。

## scheduled() vs using() の比較

| 特徴 | scheduled() | using() |
|------|-------------|---------|
| 主な目的 | 実行タイミングの制御 | リソースのライフサイクル管理 |
| スケジューラー | ✅ 明示的に指定可能 | ❌ 指定不可 |
| リソース管理 | ❌ 手動管理が必要 | ✅ 自動管理 |
| 使用場面 | テスト、UI最適化 | WebSocket、ファイルハンドル |
| 複雑度 | シンプル | やや複雑 |

## 使い分けのガイドライン

### scheduled() を選ぶべき場合

1. **実行タイミングを制御したい**
   - 同期処理を非同期に変更したい
   - UIのブロッキングを回避したい

2. **テストで時間制御が必要**
   - TestScheduler と組み合わせて時間を制御
   - 非同期処理を同期的にテストしたい

3. **既存のデータソースをObservable化**
   - 配列、Promise、Iterable を Observable に変換
   - スケジューラーを明示的に指定したい

### using() を選ぶべき場合

1. **リソースの自動解放が必要**
   - WebSocket接続の管理
   - ファイルハンドルの管理
   - タイマーの自動クリーンアップ

2. **メモリリークを防ぎたい**
   - リソースの解放忘れを防ぐ
   - 購読終了時の確実なクリーンアップ

3. **複雑なリソース管理**
   - 複数のリソースを一括管理
   - リソースの依存関係を管理

## 実践的な使用例

### scheduled() の使用例

```typescript
import { scheduled, asyncScheduler, queueScheduler } from 'rxjs';

// 大量のデータを非同期で処理（UIをブロックしない）
const largeArray = Array.from({ length: 10000 }, (_, i) => i);
const async$ = scheduled(largeArray, asyncScheduler);

async$.subscribe(value => {
  // 重い処理をここで実行
  // UIはブロックされない
});

// テストでは同期的に実行
const sync$ = scheduled(largeArray, queueScheduler);
```

### using() の使用例

```typescript
import { using, timer } from 'rxjs';

// WebSocket接続を自動管理
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    console.log('WebSocket接続開始');
    return {
      unsubscribe: () => {
        ws.close();
        console.log('WebSocket接続終了');
      }
    };
  },
  () => timer(0, 1000) // 1秒ごとにメッセージを受信
);
```

## スケジューラーの種類（scheduled() 用）

| スケジューラー | 説明 | 使用場面 |
|---------------|------|---------|
| `queueScheduler` | 同期実行（キュー方式） | デフォルト、同期的な処理 |
| `asyncScheduler` | 非同期実行（setTimeout） | UI最適化、長時間処理 |
| `asapScheduler` | 最速の非同期実行（Promise） | 高優先度の非同期処理 |
| `animationFrameScheduler` | アニメーションフレーム | アニメーション、UI描画 |

> [!TIP]
> スケジューラーについて詳しくは、[スケジューラーの種類](/guide/schedulers/types) を参照してください。

## よくある質問

### Q1: scheduled() と from() の違いは？

**A:** `from()` は内部的にデフォルトのスケジューラー（同期的）を使用します。`scheduled()` はスケジューラーを明示的に指定できるため、実行タイミングを細かく制御できます。

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - 同期的に実行
const sync$ = from([1, 2, 3]);

// scheduled() - 非同期的に実行
const async$ = scheduled([1, 2, 3], asyncScheduler);
```

### Q2: using() はいつ使うべきですか？

**A:** リソースの解放忘れを防ぎたい場合に使用します。特に以下のケースで有効です。
- WebSocket、EventSource などのネットワーク接続
- ファイルハンドル、データベース接続
- 手動で `clearInterval()` や `clearTimeout()` が必要な処理

### Q3: scheduled() でテストが簡単になる理由は？

**A:** TestScheduler を使うことで、時間経過を仮想的にコントロールできます。非同期処理を同期的にテストでき、テストの実行時間を大幅に短縮できます。

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled, asyncScheduler } from 'rxjs';

const testScheduler = new TestScheduler((actual, expected) => {
  expect(actual).toEqual(expected);
});

testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

## ベストプラクティス

### 1. scheduled() でUIブロッキングを回避

```typescript
// ❌ 悪い例: 大量のデータを同期的に処理
from(largeArray).subscribe(processHeavyTask);

// ✅ 良い例: asyncScheduler で非同期処理
scheduled(largeArray, asyncScheduler).subscribe(processHeavyTask);
```

### 2. using() でリソースを確実に解放

```typescript
// ❌ 悪い例: 手動でリソース管理
const ws = new WebSocket('wss://example.com');
const source$ = interval(1000);
source$.subscribe(() => ws.send('ping'));
// unsubscribe 忘れでリソースリーク

// ✅ 良い例: using() で自動管理
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return { unsubscribe: () => ws.close() };
  },
  () => interval(1000).pipe(tap(() => ws.send('ping')))
);
```

### 3. テストでは適切なスケジューラーを使用

```typescript
// ✅ 良い例: テストでは TestScheduler
const testScheduler = new TestScheduler(...);
const source$ = scheduled([1, 2, 3], testScheduler);

// ✅ 良い例: 本番では asyncScheduler
const source$ = scheduled([1, 2, 3], asyncScheduler);
```

## まとめ

制御系 Creation Functions は、Observable の動作を細かく制御するための高度な機能です。

**scheduled():**
- 実行タイミング（同期・非同期）を明示的に制御
- テストでの時間制御に便利
- UIのブロッキング回避に有効

**using():**
- リソースのライフサイクルを自動管理
- メモリリークを防ぐ
- WebSocketなどの接続管理に最適

適切に使い分けることで、より堅牢でパフォーマンスの高いRxJSアプリケーションを構築できます。

## 次のステップ

各関数の詳細な使い方については、以下のページを参照してください。

- [scheduled() の詳細](/guide/creation-functions/control/scheduled) - スケジューラーを指定してObservableを生成
- [using() の詳細](/guide/creation-functions/control/using) - リソース制御付きObservable

## 参考リソース

- [RxJS公式ドキュメント - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [RxJS公式ドキュメント - using](https://rxjs.dev/api/index/function/using)
- [RxJS公式ドキュメント - Scheduler](https://rxjs.dev/guide/scheduler)
- [スケジューラーの種類](/guide/schedulers/types)
