---
description: "RxJSのデバッグ手法について、tap()による値の追跡、console.logの効果的な配置、RxJS DevTools拡張機能、カスタムデバッグオペレーター作成、パフォーマンス計測まで、実践的なデバッグ戦略を体系的に解説。値が流れない問題の特定方法も紹介します。"
---

# RxJSのデバッグ手法

RxJSのデバッグは、非同期ストリームの性質上、従来の同期的なデバッグ手法とは異なるアプローチが必要です。

このページでは、RxJSアプリケーションをデバッグするための基本戦略と、詳細なデバッグ手法へのナビゲーションを提供します。

## デバッグ手法の概要

RxJSのデバッグは以下の4つのアプローチに分類されます。

| アプローチ | 内容 | 詳細ページ |
|----------|------|-----------|
| **基本戦略** | tap オペレーター、開発者ツール、RxJS DevTools | 本ページで解説 |
| **よくあるシナリオ** | 値が流れてこない、メモリリーク、エラーの見逃しなど6つの典型的問題 | [→ 詳細](/guide/debugging/common-scenarios) |
| **カスタムツール** | 名前付きストリーム、デバッグオペレーター、パフォーマンス計測 | [→ 詳細](/guide/debugging/custom-tools) |
| **パフォーマンス** | 購読数監視、再評価検出、メモリ使用量確認、ベストプラクティス | [→ 詳細](/guide/debugging/performance) |

## デバッグの基本戦略

### 1. `tap` オペレーターでのログ出力

`tap` オペレーターは、ストリームの値に副作用を与えずに観察できる、最も基本的なデバッグ手法です。

```ts
import { interval } from 'rxjs';
import { map, filter, tap } from 'rxjs';

interval(1000)
  .pipe(
    tap(value => console.log('🔵 元の値:', value)),
    map(x => x * 2),
    tap(value => console.log('🟢 map後:', value)),
    filter(x => x > 5),
    tap(value => console.log('🟡 filter後:', value))
  )
  .subscribe(value => console.log('✅ 最終値:', value));

// 出力:
// 🔵 元の値: 0
// 🟢 map後: 0
// 🔵 元の値: 1
// 🟢 map後: 2
// 🔵 元の値: 2
// 🟢 map後: 4
// 🔵 元の値: 3
// 🟢 map後: 6
// 🟡 filter後: 6
// ✅ 最終値: 6
```

#### ポイント
- パイプラインの各ステップに `tap` を挿入することで、データの流れを追跡できる
- 絵文字やラベルを使うことで、ログの視認性を向上させる
- `tap` は値を変更しないため、安全にデバッグログを挿入できる

### 2. 詳細なログ情報の出力

より詳細なデバッグ情報を取得するには、Observer オブジェクトを使用します。

```ts
import { of, throwError, concat } from 'rxjs';
import { tap } from 'rxjs';

const debug = (tag: string) =>
  tap({
    next: value => console.log(`[${tag}] next:`, value),
    error: error => console.error(`[${tag}] error:`, error),
    complete: () => console.log(`[${tag}] complete`)
  });

// 正常なストリーム
of(1, 2, 3)
  .pipe(debug('正常'))
  .subscribe();

// 出力:
// [正常] next: 1
// [正常] next: 2
// [正常] next: 3
// [正常] complete

// エラーを含むストリーム
concat(
  of(1, 2),
  throwError(() => new Error('エラー発生'))
)
  .pipe(debug('エラー'))
  .subscribe({
    error: () => {} // エラーハンドリング
  });

// 出力:
// [エラー] next: 1
// [エラー] next: 2
// [エラー] error: Error: エラー発生
```

### 3. 開発者ツールでの確認

ブラウザの開発者ツールを活用したデバッグ手法です。

```ts
import { fromEvent, timer } from 'rxjs';
import { map, tap, debounceTime } from 'rxjs';

// デバッグ用のヘルパー関数
function tapDebugger<T>(label: string) {
  return tap<T>({
    next: value => {
      console.group(`🔍 ${label}`);
      console.log('Value:', value);
      console.log('Type:', typeof value);
      console.log('Timestamp:', new Date().toISOString());
      console.trace('Stack trace');
      console.groupEnd();
    }
  });
}

// ボタンクリックイベントのデバッグ
const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tapDebugger('Click Event'),
      debounceTime(300),
      tapDebugger('After Debounce'),
      map(() => ({ timestamp: Date.now() }))
    )
    .subscribe(data => console.log('📤 送信:', data));
}
```

#### 開発者ツールの活用
- `console.group()` でログをグループ化
- `console.trace()` でスタックトレースを表示
- `console.table()` で配列やオブジェクトを見やすく表示
- ブレークポイントを `tap` 内に設置

### 4. RxJS DevTools の活用

RxJS DevTools は、ブラウザ拡張機能として提供されるデバッグツールです。

#### インストール
- Chrome: [RxJS DevTools - Chrome Web Store](https://chrome.google.com/webstore)
- Firefox: [RxJS DevTools - Firefox Add-ons](https://addons.mozilla.org/)

#### 主な機能
- Observable の購読状態の可視化
- ストリームの値のタイムライン表示
- メモリリークの検出
- パフォーマンス分析

#### 使用例

```ts
import { interval } from 'rxjs';
import { take, map } from 'rxjs';

// 開発環境でのみデバッグを有効化
// ビルドツールによって異なる環境変数の判定方法
const isDevelopment =
  // Vite: import.meta.env.DEV
  // webpack: process.env.NODE_ENV === 'development'
  // 手動設定: グローバル変数を使用
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

const stream$ = interval(1000).pipe(
  take(5),
  map(x => x * 2)
);

if (isDevelopment) {
  // DevTools で観察可能にする
  stream$.subscribe({
    next: value => console.log('DevTools:', value)
  });
}
```

## 詳細なデバッグ手法

基本戦略を理解したら、以下の詳細ページで具体的なデバッグ手法を学習してください。

### よくあるデバッグシナリオ

実際の開発で遭遇する6つの典型的な問題とその解決方法

- シナリオ1: 値が流れてこない
- シナリオ2: 期待と異なる値が出力される
- シナリオ3: 購読が完了しない（無限ストリーム）
- シナリオ4: メモリリーク（購読解除忘れ）
- シナリオ5: エラーが発生しているのに気づかない
- シナリオ6: リトライの試行回数を追跡したい

[→ よくあるデバッグシナリオを見る](/guide/debugging/common-scenarios)

### カスタムデバッグツール

プロジェクトの要件に合わせた独自のデバッグツールの作成方法

- 名前付きストリームのデバッグ（tagStream）
- カスタムデバッグオペレーターの作成
- パフォーマンス計測用オペレーター（measure）

[→ カスタムデバッグツールを見る](/guide/debugging/custom-tools)

### パフォーマンスデバッグ

アプリケーションの最適化とベストプラクティス

- 購読数の確認と追跡
- 不要な再評価の検出（shareReplay）
- メモリ使用量の監視
- デバッグ環境の構築
- 型安全なデバッグ
- エラー境界の設定

[→ パフォーマンスデバッグを見る](/guide/debugging/performance)

## まとめ

RxJSのデバッグは、以下のポイントを押さえることで効率的に行えます。

### 基本戦略
- ✅ `tap` オペレーターでストリームの各段階を観察
- ✅ 開発者ツールを活用した詳細なログ出力
- ✅ RxJS DevTools でストリームを可視化

### よくあるシナリオ
- ✅ 値が流れてこない → 購読忘れ、フィルタリング条件の確認
- ✅ 期待と異なる値 → オペレーターの順序、参照の共有に注意
- ✅ 購読が完了しない → 無限ストリームに `take` や `takeUntil` を使用
- ✅ メモリリーク → `takeUntil` パターンで自動購読解除
- ✅ エラーの見逃し → 適切なエラーハンドリングの実装

### デバッグツール
- ✅ カスタムデバッグオペレーターで柔軟なデバッグ
- ✅ 名前付きストリームで複数ストリームを追跡
- ✅ パフォーマンス計測でボトルネックを特定

### パフォーマンス
- ✅ 購読数の監視でメモリリークを防止
- ✅ 不要な再計算を `shareReplay` で回避
- ✅ メモリ使用量を定期的に確認

これらの手法を組み合わせることで、RxJSアプリケーションのデバッグを効率的に行うことができます。

## 関連ページ

- [エラーハンドリング](/guide/error-handling/strategies) - エラー処理の戦略
- [テスト手法](/guide/testing/unit-tests) - RxJSのテスト方法
- [RxJSアンチパターン集](/guide/anti-patterns/) - よくある間違いと対処法
- [パイプライン](/guide/operators/pipeline) - オペレーターの連鎖
