---
description: RxJSコードを書く際に確認すべきアンチパターン回避チェックリスト。15のベストプラクティスを確認し、堅牢で保守性の高いコードを実現しましょう。
---

# アンチパターン回避チェックリスト

このチェックリストを使って、自分のRxJSコードがアンチパターンに該当していないか確認しましょう。各項目をクリックすると、詳しい解説とコード例を確認できます。

## チェック項目

### 🔴 重大な問題の回避

- [ ] **[Subject は `asObservable()` で読み取り専用として公開している](./common-mistakes#1-subject-の外部公開)**
  - `Subject` を直接 export せず、`asObservable()` で Observable として公開
  - 状態変更は専用メソッド経由でのみ可能にする

- [ ] **[ネストした `subscribe` を避け、高階オペレーターを使っている](./common-mistakes#2-ネストした-subscribe-コールバック地獄)**
  - `subscribe` 内で別の `subscribe` を呼ばない
  - `switchMap`、`mergeMap`、`concatMap` などでフラット化

- [ ] **[無限ストリームは `takeUntil` や `unsubscribe` で確実に解除している](./common-mistakes#3-unsubscribe-忘れ-メモリリーク)**
  - イベントリスナーなど無限に続くストリームは必ず購読解除
  - `takeUntil` パターンまたは `Subscription` の管理

- [ ] **[`shareReplay` は `bufferSize` と `refCount` を明示している](./common-mistakes#4-sharereplay-の誤用)**
  - `shareReplay({ bufferSize: 1, refCount: true })` の形式を使用
  - メモリリークを防ぐために参照カウントを有効化

### 🟡 注意が必要な問題の回避

- [ ] **[`map` は純粋関数として使い、副作用は `tap` に分離している](./common-mistakes#5-map-での副作用)**
  - `map` 内で状態変更やログ出力をしない
  - 副作用は `tap` オペレーターで明示的に分離

- [ ] **[Cold/Hot Observable の違いを理解し、適切に使い分けている](./common-mistakes#6-cold-hot-observable-の違いの無視)**
  - HTTP リクエストなどは `shareReplay` で Hot に変換
  - 購読ごとに実行されるべきか、共有すべきかを判断

- [ ] **[Promise は `from` で Observable に変換して統一している](./common-mistakes#7-promise-と-observable-の不適切な混在)**
  - Promise と Observable を混在させない
  - `from()` で Observable に変換して統一的に処理

- [ ] **[高頻度イベントは `debounceTime` や `throttleTime` で制御している](./common-mistakes#8-バックプレッシャーの無視)**
  - 検索入力などは `debounceTime` で制御
  - スクロールイベントなどは `throttleTime` で制御
  - `distinctUntilChanged` で重複を除外

### 🔵 コード品質の向上

- [ ] **[エラーは適切にログ記録し、ユーザーにフィードバックしている](./common-mistakes#9-エラーの握りつぶし)**
  - `catchError` でエラーをキャッチし、適切に処理
  - ユーザーに分かりやすいエラーメッセージを表示
  - 必要に応じて `retry` / `retryWhen` で再試行

- [ ] **[DOM イベントリスナーは適切に解放している](./common-mistakes#10-dom-イベントサブスクリプションのリーク)**
  - `fromEvent` の購読は必ず解除
  - コンポーネント破棄時に `takeUntil` で自動解除

- [ ] **[`any` を避け、型安全性を確保している](./common-mistakes#11-型安全性の欠如-any-の多用)**
  - インターフェースや型エイリアスを定義
  - `Observable<T>` の型パラメータを明示
  - TypeScript の型推論を活用

- [ ] **[目的に合った適切なオペレーターを選択している](./common-mistakes#12-不適切なオペレーター選択)**
  - 検索: `switchMap`（最新のみ）
  - 並列処理: `mergeMap`
  - 順次処理: `concatMap`
  - 連打防止: `exhaustMap`

- [ ] **[シンプルな処理は RxJS を使わず、通常の JavaScript で書いている](./common-mistakes#13-過度な複雑化)**
  - 配列処理などは通常の JavaScript で十分
  - RxJS は非同期処理やイベントストリームに使用

- [ ] **[`subscribe` 内で直接状態変更せず、リアクティブに管理している](./common-mistakes#14-subscribe-内での状態変更)**
  - `BehaviorSubject` や `scan` で状態を管理
  - `subscribe` は最終的なトリガーとして使用

- [ ] **[RxJS コードのテストを書いている](./common-mistakes#15-テストの欠如)**
  - `TestScheduler` でマーブルテストを実施
  - 非同期処理を同期的にテスト可能に

## 使い方

### 1. コードレビュー時

新しいコードを書いた後、このチェックリストを使って自己レビューを実施しましょう。

### 2. プルリクエスト時

プルリクエストのテンプレートにこのチェックリストを含めることで、レビュアーと共通の基準で確認できます。

### 3. 定期的な見直し

既存のコードベースに対して定期的にこのチェックリストを使い、アンチパターンが混入していないか確認しましょう。

### 4. チーム内での共有

チームメンバーと共有し、RxJS のベストプラクティスを統一しましょう。

## 関連リソース

- **[よくある間違いと対処法](./common-mistakes)** - 各アンチパターンの詳細な解説とコード例
- **[アンチパターン集トップ](./index)** - アンチパターン一覧と学習の進め方
- **[エラーハンドリング](/guide/error-handling/strategies)** - エラー処理のベストプラクティス
- **[テスト手法](/guide/testing/unit-tests)** - RxJS コードのテスト方法

## チェックリスト活用のコツ

1. **全項目を一度に完璧にしようとしない**
   - まずは重大な問題（🔴）から優先的に対処
   - 段階的に改善していく

2. **チーム内で優先度を決める**
   - プロジェクトの特性に応じて重要度を調整
   - カスタマイズしたチェックリストを作成

3. **自動化を検討**
   - ESLint などの静的解析ツールで自動チェック
   - CI/CD パイプラインに組み込む

4. **定期的に更新**
   - RxJS のバージョンアップに応じて更新
   - チームの経験から得た知見を反映

---

**重要**: このチェックリストは完璧なコードを書くためのものではなく、よくある問題を避けるためのガイドです。プロジェクトの文脈に応じて柔軟に活用してください。
