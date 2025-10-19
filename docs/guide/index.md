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
- [PromiseとRxJSの違い](/guide/basics/promise-vs-rxjs)

### 2. Observableの基礎
- [Observableとは](/guide/observables/what-is-observable)
- [Observableの作成方法](/guide/observables/creation)
- [イベントのストリーム化](/guide/observables/events)
- [fromEvent で利用できないイベント](/guide/observables/events#cannot-used-fromEvent)
- [イベント一覧](/guide/observables/events-list)
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
- [partition - 条件で2つに分割](/guide/creation-functions/partition)
- [iif - 条件による分岐](/guide/creation-functions/iif)
- [defer - 遅延生成](/guide/creation-functions/defer)

### 4. オペレーターの理解
- [オペレーターの概要](/guide/operators/)
- [パイプラインの概念](/guide/operators/pipeline)
- [変換オペレーター](/guide/operators/transformation/) - map, scan, mergeMap, switchMap, buffer系、window系など
- [フィルタリングオペレーター](/guide/operators/filtering/) - filter, take, debounceTime, throttleTime, distinctなど
- [結合オペレーター](/guide/operators/combination/) - concatWith, mergeWith, withLatestFrom, *Allオペレーターなど
- [ユーティリティオペレーター](/guide/operators/utility/) - tap, delay, retry, finalize, takeUntilなど
- [条件オペレーター](/guide/operators/conditional/) - defaultIfEmpty, every, isEmptyなど
- [マルチキャスティング](/guide/operators/multicasting/) - share, shareReplayなど

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
- [PromiseとObservableの混在](/guide/anti-patterns/promise-observable-mixing)
- [ワンライナー地獄と段階分離構文](/guide/anti-patterns/one-liner-hell)
- [アンチパターン回避チェックリスト](/guide/anti-patterns/checklist)

### 11. RxJS困難点克服
- [なぜRxJSは難しいのか](/guide/overcoming-difficulties/)
- [概念理解の壁](/guide/overcoming-difficulties/conceptual-understanding)
- [ライフサイクル管理の壁](/guide/overcoming-difficulties/lifecycle-management)
- [オペレーター選択の迷い](/guide/overcoming-difficulties/operator-selection)
- [タイミングと順序の理解](/guide/overcoming-difficulties/timing-and-order)
- [状態管理の難しさ](/guide/overcoming-difficulties/state-and-sharing)
- [複数ストリーム組み合わせ](/guide/overcoming-difficulties/stream-combination)
- [デバッグの壁](/guide/overcoming-difficulties/debugging-guide)

---

> [!NOTE]
> このガイドは、段階的かつ体系的にRxJSの理解を深めるために構成されています。
> 必要に応じて各セクションを自由に参照してください。
