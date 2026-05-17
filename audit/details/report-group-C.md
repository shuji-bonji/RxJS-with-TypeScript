# Group C レポート（creation-functions, overcoming-difficulties, practical-patterns）

## docs/guide/creation-functions/basic/from.md

### lint_rxjs
- `prefer-observer` (7件): 個別コールバックでの subscribe を Observer オブジェクトに置き換え推奨
- `no-implicit-any-catch` (1件): catchError の error パラメータに `unknown` 型注釈を追加推奨

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 5 subscribe / 0 unsubscribe、subscription (medium) 完了オペレータ（takeUntil/take/first）なし

### analyze_operators
- 主な指摘: concatMap + mergeMap 混在で複雑化の可能性。`share()/shareReplay()` 追加や `takeUntil()` 検討推奨

---

## docs/guide/creation-functions/basic/fromEvent.md

### lint_rxjs
- `prefer-observer` (3件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 9 subscribe / 2 unsubscribe、subscription (medium) 完了オペレータなし、operator (high) fromEvent() の DOM イベントリスナー未解除リスク

### analyze_operators
- 主な指摘: 9 演算子チェーン (debounceTime, map, distinctUntilChanged, switchMap, takeUntil, throttleTime, filter, scan, share)。`catchError` 等のエラーハンドリング追加検討

---

## docs/guide/creation-functions/basic/index.md

### lint_rxjs
- `prefer-observer` (5件)

### detect_memory_leak
- ステータス: ✅ Clean（takeUntil(notifier$) と take(N) を検出）

### analyze_operators
- 主な指摘: 良好な構成 (switchMap/take/share/takeUntil/debounceTime/map)

---

## docs/guide/creation-functions/basic/interval.md

### lint_rxjs
- `prefer-observer` (10件)
- `no-implicit-any-catch` (1件)

### detect_memory_leak
- ステータス: ✅ Clean（take(N) と takeWhile() を検出）

### analyze_operators
- 主な指摘: 適切な構成 (switchMap, catchError, map, takeWhile, filter, scan, share)

---

## docs/guide/creation-functions/basic/of.md

### lint_rxjs
- `prefer-observer` (4件)
- `no-implicit-any-catch` (1件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 10 subscribe / 0 unsubscribe、subscription (medium) 完了オペレータなし

### analyze_operators
- 主な指摘: catchError, concatMap, delay, filter, map の組合せ。`share()/shareReplay()` 追加検討

---

## docs/guide/creation-functions/basic/timer.md

### lint_rxjs
- `no-unsafe-takeuntil` (2件, error): takeUntil の後に map → takeUntil は subscribe 直前（share/finalize 以外）に置くべき
- `prefer-observer` (5件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subject (medium) 2 Subjects 作成 / 0 complete() 呼び出し

### analyze_operators
- 主な指摘: switchMap, retry, map, takeUntil, take, scan, share の組合せ。良好

---

## docs/guide/creation-functions/combination/combineLatest.md

### lint_rxjs
- `prefer-observer` (2件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 4/0、subscription (medium) 完了オペレータなし、operator (high) 無限 interval/timer に take/takeUntil なし、subject (medium) 2 Subjects / 0 complete()、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: debounceTime → switchMap → startWith のチェーン。`share()/shareReplay()` と `catchError()` 追加検討

---

## docs/guide/creation-functions/combination/concat.md

### lint_rxjs
- `prefer-observer` (2件)

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 検出）

### analyze_operators
- 主な指摘: map, take, share の良好なチェーン

---

## docs/guide/creation-functions/combination/forkJoin-vs-combineLatest.md

### lint_rxjs
- `prefer-observer` (2件)

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 検出）

### analyze_operators
- 主な指摘: of, interval, combineLatest, forkJoin を含む創造関数。take で適切に管理

---

## docs/guide/creation-functions/combination/forkJoin.md

### lint_rxjs
- `prefer-observer` (1件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 2/0、subscription (medium) 完了オペレータなし

### analyze_operators
- 主な指摘: forkJoin のみのシンプルな構成

---

## docs/guide/creation-functions/combination/index.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 4/0、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: concatWith, mergeWith, map, filter のチェーン。`takeUntil()` 検討

---

## docs/guide/creation-functions/combination/merge.md

### lint_rxjs
- `prefer-observer` (2件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 4/0、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: map, take, share の標準的チェーン

---

## docs/guide/creation-functions/combination/zip.md

### lint_rxjs
- `prefer-observer` (1件)

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 検出）

### analyze_operators
- 主な指摘: シンプルな zip 使用例 (map, take)

---

## docs/guide/creation-functions/conditional/defer.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 5/0、subscription (medium) 完了オペレータなし

### analyze_operators
- (hasPipe=false のため実行なし)

---

## docs/guide/creation-functions/conditional/iif.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 5/0、subscription (medium) 完了オペレータなし

### analyze_operators
- (hasPipe=false のため実行なし)

---

## docs/guide/creation-functions/conditional/index.md

### lint_rxjs
- `prefer-observer` (6件)

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 検出）

### analyze_operators
- 主な指摘: switchMap, take, share の組合せ。`catchError()` 追加検討

---

## docs/guide/creation-functions/control/index.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 検出）

### analyze_operators
- 主な指摘: scheduled, using を使った適切な構成 (take, tap)

---

## docs/guide/creation-functions/control/scheduled.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 7/0、subscription (medium) 完了オペレータなし

### analyze_operators
- 主な指摘: 6 演算子チェーン (bufferCount, map, reduce, mergeMap, toArray, delay)。`takeUntil()` と `catchError()` 検討

---

## docs/guide/creation-functions/control/using.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subject (medium) 1 Subject / 0 complete()

### analyze_operators
- 主な指摘: using の典型例 (take 使用)

---

## docs/guide/creation-functions/http-communication/ajax.md

### lint_rxjs
- `no-implicit-any-catch` (1件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 6/1、subscription (medium) 完了オペレータなし、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: timeout, retry, catchError, map, switchMap の良好なチェーン。`share()/shareReplay()` 検討

---

## docs/guide/creation-functions/http-communication/fromFetch.md

### lint_rxjs
- `no-implicit-any-catch` (3件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 6/0、subscription (medium) 完了オペレータなし

### analyze_operators
- 主な指摘: switchMap, catchError, retry, timeout, map の標準的なエラーハンドリング構成

---

## docs/guide/creation-functions/http-communication/index.md

### lint_rxjs
- `no-implicit-any-catch` (1件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 5/1、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: timeout, retry, catchError, map, switchMap の組合せ。`share()/shareReplay()` 検討

---

## docs/guide/creation-functions/index.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 4/0、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: concatWith, mergeWith の Pipeable Operator と merge Creation Function の対比。良好

---

## docs/guide/creation-functions/loop/generate.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 検出）

### analyze_operators
- 主な指摘: take, concatMap, delay のシンプルなチェーン

---

## docs/guide/creation-functions/loop/index.md

### lint_rxjs
- `prefer-observer` (1件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 8/0、subscription (medium) 完了オペレータなし

### analyze_operators
- 主な指摘: map, toArray, share, concatMap, delay, retryWhen, mergeMap, observeOn。**`retryWhen` は deprecated (7.3.0以降) → `retry({ delay: ... })` 推奨**。`takeUntil()` 検討

---

## docs/guide/creation-functions/loop/range.md

### lint_rxjs
- `prefer-observer` (1件)
- `no-implicit-any-catch` (1件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 9/0、subscription (medium) 完了オペレータなし

### analyze_operators
- 主な指摘: concatMap, delay, map, toArray, catchError の組合せ。`share()/shareReplay()` 検討

---

## docs/guide/creation-functions/selection/index.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 7/0、subscription (medium) 完了オペレータなし

### analyze_operators
- 主な指摘: **`mapTo` は deprecated (7.2.0以降) → `map(() => value)` 推奨**。share の併用は良好

---

## docs/guide/creation-functions/selection/partition.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 検出）

### analyze_operators
- 主な指摘: mergeMap, share, take, map の組合せ。`catchError()` 追加検討

---

## docs/guide/creation-functions/selection/race.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 2/0、subscription (medium) 完了オペレータなし

### analyze_operators
- 主な指摘: race と map のシンプルな構成

---

## docs/guide/overcoming-difficulties/conceptual-understanding.md

### lint_rxjs
- `no-nested-subscribe` (1件, error): ネストした subscribe → flattening オペレータで置換推奨
- `prefer-observer` (4件)

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 検出）

### analyze_operators
- 主な指摘: take, share, mergeMap, filter, map, toArray, throttleTime の組合せ。`catchError()` 追加検討

---

## docs/guide/overcoming-difficulties/debugging-guide.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 8/0、operator (high) 無限 interval に limit operator なし

### analyze_operators
- 主な指摘: tap, filter, map, mergeMap, delay, concatMap の組合せ。複数 flattening operator 混在。`share()/shareReplay()` と `takeUntil()` 検討

---

## docs/guide/overcoming-difficulties/index.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 1/0、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: シンプルな map のみ（index ページのためコード少）

---

## docs/guide/overcoming-difficulties/lifecycle-management.md

### lint_rxjs
- `prefer-observer` (3件)

### detect_memory_leak
- ステータス: ✅ Clean（takeUntil(notifier$) と take(N) を検出、Angular ライフサイクル下で適切）

### analyze_operators
- 主な指摘: filter, take, takeUntil, map の組合せ。教育的に良好な構成

---

## docs/guide/overcoming-difficulties/operator-selection.md

### lint_rxjs
- `prefer-observer` (2件)
- `no-implicit-any-catch` (1件)

### detect_memory_leak
- ステータス: ✅ Clean（Angular ライフサイクル下で takeUntil(notifier$) を検出）

### analyze_operators
- 主な指摘: debounceTime, map, distinctUntilChanged, switchMap, retry, catchError, mergeMap, concatMap, exhaustMap の網羅的構成。複数 flattening operator は教育目的（比較）。良好

---

## docs/guide/overcoming-difficulties/state-and-sharing.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 8/0、subject (medium) 5 Subjects / 0 complete()

### analyze_operators
- 主な指摘: filter, map, share, shareReplay の組合せ。`shareReplay()` の buffer 制限警告。`takeUntil()` 検討

---

## docs/guide/overcoming-difficulties/stream-combination.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 7/0、operator (high) 無限 interval/timer に limit なし、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: map, debounceTime, switchMap, startWith の組合せ。combineLatest, merge, forkJoin, zip, race を網羅。`share()/shareReplay()`、`catchError()`、`takeUntil()` 検討

---

## docs/guide/overcoming-difficulties/timing-and-order.md

### lint_rxjs
- `prefer-observer` (1件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subject (medium) 1 Subject / 0 complete()

### analyze_operators
- 主な指摘: debounceTime, distinctUntilChanged, switchMap, throttleTime, withLatestFrom, mergeMap, delay の組合せ。複数 flattening operator 警告。`share()/shareReplay()` 検討

---

## docs/guide/practical-patterns/advanced-form-patterns.md

### lint_rxjs
- `prefer-observer` (1件)
- `no-implicit-any-catch` (1件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 3/0、subject (medium) 4 Subjects / 0 complete()

### analyze_operators
- 主な指摘: pairwise, map, filter, bufferTime, concatMap, catchError, debounceTime の組合せ。map + filter 結合で最適化可能。`share()/shareReplay()` と `takeUntil()` 検討

---

## docs/guide/practical-patterns/api-calls.md

### lint_rxjs
- `no-implicit-any-catch` (3件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 7/0、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: timeout, catchError, map, debounceTime, distinctUntilChanged, switchMap, retry, exhaustMap, shareReplay の網羅的構成。複数 flattening operator 警告。`shareReplay()` buffer 制限警告。`takeUntil()` 検討

---

## docs/guide/practical-patterns/caching-strategies.md

### lint_rxjs
- `no-sharereplay` (4件, error): **`shareReplay(1)` 等の数値引数は危険 → `shareReplay({ bufferSize: 1, refCount: true })` を使用すべき**
- `no-implicit-any-catch` (1件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 3/0、subject (medium) 1 Subject / 0 complete()、operator (low) shareReplay() refCount なしのリスク

### analyze_operators
- 主な指摘: tap, shareReplay, catchError, switchMap, map の組合せ。`shareReplay()` buffer 制限警告。`takeUntil()` 検討

---

## docs/guide/practical-patterns/error-handling-patterns.md

### lint_rxjs
- `no-implicit-any-catch` (2件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 3/0、subject (medium) 1 Subject / 0 complete()

### analyze_operators
- 主な指摘: retry, catchError, mergeMap の標準的なエラーハンドリング構成。`share()/shareReplay()` 検討

---

## docs/guide/practical-patterns/form-handling.md

### lint_rxjs
- `no-implicit-any-catch` (1件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 4/0、subject (medium) 1 Subject / 0 complete()、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: map, tap, switchMap, debounceTime, distinctUntilChanged, filter の組合せ。`share()/shareReplay()`、`catchError()`、`takeUntil()` 検討

---

## docs/guide/practical-patterns/index.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 1/0、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: map, switchMap のシンプルな構成（index ページのためコード少）

---

## docs/guide/practical-patterns/real-time-data.md

### lint_rxjs
- `no-unsafe-subject-next` (1件): 型付き Subject に値なしで .next() を呼び出している
- `no-implicit-any-catch` (1件)

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 4/0、subject (medium) 3 Subjects / 1 complete()、operator (high) 無限 interval (polling) に limit なし

### analyze_operators
- 主な指摘: map, retry, switchMap, catchError, filter, scan の組合せ。WebSocket/SSE/Polling の典型的構成。`share()/shareReplay()` と `takeUntil()` 検討

---

## docs/guide/practical-patterns/subscribe-branching.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscription (high) 6/0、subject (medium) 2 Subjects / 0 complete()、operator (high) fromEvent() リスナー未解除リスク

### analyze_operators
- 主な指摘: filter, switchMap, catchError, mergeMap の組合せ。複数 flattening operator 警告。`share()/shareReplay()` と `takeUntil()` 検討

---

## docs/guide/practical-patterns/ui-events.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（takeUntil(notifier$) パターンを検出）

### analyze_operators
- 主な指摘: throttleTime, debounceTime, map, distinctUntilChanged, switchMap, takeUntil, scan, filter の網羅的構成。良好。`share()/shareReplay()` と `catchError()` 追加検討
