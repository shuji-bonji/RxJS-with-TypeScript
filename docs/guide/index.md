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
- [基本作成系](/guide/creation-functions/basic/) - of, from, fromEvent, interval, timer
- [ループ生成系](/guide/creation-functions/loop/) - range, generate
- [HTTP通信系](/guide/creation-functions/http-communication/) - ajax, fromFetch
- [結合系](/guide/creation-functions/combination/) - concat, merge, combineLatest, zip, forkJoin
- [選択・分割系](/guide/creation-functions/selection/) - race, partition
- [条件分岐系](/guide/creation-functions/conditional/) - iif, defer
- [制御系](/guide/creation-functions/control/) - scheduled, using

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
- [エラーハンドリングの2つの場所](/guide/error-handling/error-handling-locations)
- [try-catchとRxJSの統合](/guide/error-handling/try-catch-integration)
- [retry と catchError](/guide/error-handling/retry-catch)
- [finalize と complete](/guide/error-handling/finalize)

### 7. スケジューラーの活用
- [非同期処理の制御](/guide/schedulers/async-control)
- [スケジューラーの種類と使い分け](/guide/schedulers/types)
- [補足:タスクとスケジューラーの基礎知識](/guide/schedulers/task-and-scheduler-basics)

### 8. RxJSのデバッグ手法
- [デバッグ手法の概要](/guide/debugging/)
- [よくあるデバッグシナリオ](/guide/debugging/common-scenarios)
- [カスタムデバッグツール](/guide/debugging/custom-tools)
- [パフォーマンスデバッグ](/guide/debugging/performance)

### 9. テスト手法
- [RxJSのユニットテスト](/guide/testing/unit-tests)
- [TestSchedulerの活用](/guide/testing/test-scheduler)
- [マーブルテスト](/guide/testing/marble-testing)

### 10. RxJSアンチパターン集
- [アンチパターンとは](/guide/anti-patterns/)
- [よくある間違いと対処法](/guide/anti-patterns/common-mistakes)
- [subscribe内のif文ネスト地獄](/guide/anti-patterns/subscribe-if-hell)
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

### 13. 実践パターン集
- [実践パターン集の概要](/guide/practical-patterns/)
- [UIイベント処理](/guide/practical-patterns/ui-events) - クリック、スクロール、ドラッグ&ドロップなど
- [API呼び出し](/guide/practical-patterns/api-calls) - HTTP通信、並列/直列処理、エラーハンドリング
- [フォーム処理](/guide/practical-patterns/form-handling) - リアルタイムバリデーション、自動保存、複数フィールド連携
- [高度なフォームパターン](/guide/practical-patterns/advanced-form-patterns) - JSON Patch、大規模フォーム自動保存、Undo/Redo、共同編集
- [リアルタイムデータ処理](/guide/practical-patterns/real-time-data) - WebSocket、SSE、Polling、接続管理
- [キャッシュ戦略](/guide/practical-patterns/caching-strategies) - データキャッシュ、TTL、無効化、オフライン対応
- [エラーハンドリング実践](/guide/practical-patterns/error-handling-patterns) - API呼び出しエラー、リトライ戦略、グローバルエラー処理
- [subscribe内の条件分岐](/guide/practical-patterns/subscribe-branching) - subscribe内での分岐を避ける、パイプライン内での分岐方法

### 付録
- [付録の概要](/guide/appendix/)
- [組み込み開発とリアクティブプログラミング](/guide/appendix/embedded-reactive-programming)
- [ReactiveX以外のリアクティブ的手法](/guide/appendix/reactive-patterns-beyond-rxjs)
- [リアクティブアーキテクチャ全体マップ](/guide/appendix/reactive-architecture-map)

---

> [!NOTE]
> このガイドは、段階的かつ体系的にRxJSの理解を深めるために構成されています。
> 必要に応じて各セクションを自由に参照してください。
