---
description: TypeScript環境でRxJSを体系的に学ぶための学習ガイドです。Observableの基礎からSubject、各種オペレーター、エラー処理、スケジューラー、テスト手法まで段階的かつ実践的に解説します。各セクションは独立して参照可能です。
---

# ガイド

TypeScript環境でRxJSを体系的に学ぶためのガイドです。  
以下の各セクションを順に進めることで、RxJSの基礎から応用まで体系的に理解することができます。

## 目次

### 1. RxJS入門
- [はじめに](/guide/introduction)
- [学習用の実行環境構築](/guide/starter-kid.md)
- [RxJSとは何か](/guide/basics/what-is-rxjs)
- [ストリームとは？](/guide/basics/what-is-a-stream)

### 2. Observableの基礎
- [Observableとは](/guide/observables/what-is-observable)
- [Observableの作成方法](/guide/observables/creation)
- [イベントのストリーム化](/guide/observables/events)
- [Observableのライフサイクル](/guide/observables/observable-lifecycle)
- [コールドObservableとホットObservable](/guide/observables/cold-and-hot-observables)

### 3. Creation Functions
- [Creation Functionsとは](/guide/creation-functions/)
- [concat - 順次結合](/guide/creation-functions/concat)
- [merge - 並行結合](/guide/creation-functions/merge)
- [combineLatest - 最新値の組み合わせ](/guide/creation-functions/combineLatest)
- [zip - 対応する値のペア化](/guide/creation-functions/zip)
- [race - 最速のストリームを採用](/guide/creation-functions/race)
- [forkJoin - すべての完了を待つ](/guide/creation-functions/forkJoin)

### 4. オペレーターの理解
- [オペレーターの概要](/guide/operators/)
- [パイプラインの概念](/guide/operators/pipeline)

### 5. Subjectとマルチキャスト
- [Subjectとは](/guide/subjects/what-is-subject)
- [Subjectの種類](/guide/subjects/types-of-subject)
- [Multicastingの仕組み](/guide/subjects/multicasting)
- [Subjectのユースケース](/guide/subjects/use-cases)

### 6. エラーハンドリング
- [エラー処理戦略](/guide/error-handling/strategies)
- [retry と catchError](/guide/error-handling/retry-catch)
- [finalize と complete](/guide/error-handling/finalize)

### 7. スケジューラーの活用
- [非同期処理の制御](/guide/schedulers/async-control)
- [スケジューラーの種類と使い分け](/guide/schedulers/types)
- [補足:タスクとスケジューラーの基礎知識](/guide/schedulers/task-and-scheduler-basics)

### 8. RxJSのデバッグ手法
- [デバッグ手法の概要](/guide/debugging/)

### 9. テスト手法
- [RxJSのユニットテスト](/guide/testing/unit-tests)
- [TestSchedulerの活用](/guide/testing/test-scheduler)
- [マーブルテスト](/guide/testing/marble-testing)

### 10. RxJSアンチパターン集
- [アンチパターンとは](/guide/anti-patterns/)
- [よくある間違いと対処法](/guide/anti-patterns/common-mistakes)
- [アンチパターン回避チェックリスト](/guide/anti-patterns/checklist)

---

> [!NOTE]
> このガイドは、段階的かつ体系的にRxJSの理解を深めるために構成されています。
> 必要に応じて各セクションを自由に参照してください。
