# Group D レポート（その他章）

## docs/guide/anti-patterns/common-mistakes.md

### lint_rxjs
- 🔴 no-nested-subscribe (×3): ネストした subscribe を検出。switchMap/mergeMap/concatMap への置換推奨
- 🔴 no-unsafe-takeuntil: takeUntil の後に finalize 以外のオペレーター。takeUntil は最後に配置
- 🔴 no-ignored-replay-buffer: `shareReplay()` を設定オブジェクトなしで使用。`{ bufferSize: 1, refCount: true }` を推奨
- 🟡 no-unsafe-subject-next (×3): 型付き Subject の `.next()` を引数なしで呼び出し
- 🟡 no-implicit-any-catch: catchError のエラー引数に明示的な型を

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - 4 Subject 中 2 つしか complete() が呼ばれていない
  - shareReplay() に refCount オプションがない

### analyze_operators
- 主な指摘: shareReplay() がバッファ制限なしでメモリ問題のリスク、mergeMap/switchMap 併用で複雑性

---

## docs/guide/anti-patterns/flag-management.md

### lint_rxjs
- 🔴 no-sharereplay: `shareReplay(1)` の数値引数は安全でない。設定オブジェクトで refCount を指定
- 🟡 no-unsafe-subject-next: 型付き Subject の `.next()` を引数なしで呼び出し
- 🟡 no-implicit-any-catch: catchError のエラー引数に明示的な型を

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 購読 1 件に対し unsubscribe なし、完了オペレーター不使用、2 Subject 中 0 件しか complete() なし、shareReplay() に refCount なし

### analyze_operators
- 主な指摘: shareReplay() のバッファ制限なし、takeUntil() でクリーンアップ推奨

---

## docs/guide/anti-patterns/one-liner-hell.md

### lint_rxjs
- 🟡 prefer-observer: `subscribe(fn, errfn, completefn)` 形式は非推奨。Observer オブジェクト形式へ

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 3 subscribe に対して unsubscribe なし、完了オペレーター不使用、1 Subject 中 0 件しか complete() なし、fromEvent のリスナーが解放されない可能性

### analyze_operators
- 主な指摘: shareReplay() のバッファ制限なし、map + filter を一つの操作にまとめる検討余地

---

## docs/guide/anti-patterns/promise-observable-mixing.md

### lint_rxjs
- 🔴 no-topromise: `toPromise()` は非推奨。`firstValueFrom()`/`lastValueFrom()` を使用
- 🔴 no-async-subscribe: `subscribe` に async 関数を渡すのは不可。`switchMap` 等を使用
- 🔴 no-nested-subscribe: ネストした subscribe を検出
- 🟡 no-implicit-any-catch: catchError のエラー引数の型

### detect_memory_leak
- ステータス: ✅ Clean（takeUntil パターン検出のため）

### analyze_operators
- 主な指摘: shareReplay() のバッファ制限なし

---

## docs/guide/anti-patterns/subscribe-if-hell.md

### lint_rxjs
- 🔴 no-nested-subscribe: ネストした subscribe（アンチパターン例として意図的）
- 🟡 no-implicit-any-catch: catchError のエラー引数の型

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 4 subscribe に対して unsubscribe なし、完了オペレーター不使用

### analyze_operators
- 主な指摘: share()/shareReplay()、takeUntil() の使用検討

---

## docs/guide/appendix/embedded-reactive-programming.md

### lint_rxjs
- 🟡 no-unsafe-subject-next: 型付き Subject の `.next()` を引数なしで呼び出し
- 🟡 no-implicit-any-catch: catchError のエラー引数の型

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 4 Subject 中 1 件しか complete() なし

### analyze_operators
- 主な指摘: Rate limiting オプションの選択肢を説明

---

## docs/guide/appendix/reactive-architecture-map.md

### lint_rxjs
- 🔴 no-async-subscribe: subscribe に async 関数を渡している
- 🟡 no-implicit-any-catch: catchError のエラー引数の型

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 5 subscribe に対して unsubscribe なし、interval/timer の限定オペレーター不使用、fromEvent のリスナーが解放されない可能性

### analyze_operators
- 主な指摘: mergeMap/switchMap の併用で複雑性

---

## docs/guide/appendix/reactive-patterns-beyond-rxjs.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 5 subscribe に対して unsubscribe なし、interval の限定オペレーター不使用、2 Subject 中 0 件しか complete() なし、fromEvent のリスナーが解放されない可能性

### analyze_operators
- 主な指摘: map + filter を統合検討、takeUntil() でクリーンアップ

---

## docs/guide/appendix/reactive-programming-reconsidered.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 4 subscribe に対して unsubscribe なし、完了オペレーター不使用、1 Subject 中 0 件しか complete() なし

### analyze_operators
- 主な指摘: mergeMap/switchMap/concatMap の併用で複雑性、map + filter 統合検討

---

## docs/guide/appendix/rxjs-and-reactive-streams-ecosystem.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) パターン検出のため）

### analyze_operators
- 主な指摘: share()/shareReplay()、catchError()/retry() の追加検討

---

## docs/guide/basics/promise-vs-rxjs.md

### lint_rxjs
- 🟡 no-implicit-any-catch (×2): catchError のエラー引数の型

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 5 subscribe に対して unsubscribe 1 件のみ、完了オペレーター不使用、fromEvent のリスナーが解放されない可能性

### analyze_operators
- 主な指摘: switchMap/exhaustMap の併用で複雑性

---

## docs/guide/basics/what-is-rxjs.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 4 subscribe に対して unsubscribe なし、完了オペレーター不使用、fromEvent のリスナーが解放されない可能性

### analyze_operators
- 主な指摘: share()/shareReplay()、catchError()/retry()、takeUntil() の追加検討

---

## docs/guide/debugging/common-scenarios.md

### lint_rxjs
- 🔴 no-unsafe-takeuntil (×3): takeUntil の後に他のオペレーターが続く
- 🟡 no-redundant-notify: complete() 後の next() は無効
- 🟡 no-unsafe-subject-next: 型付き Subject の `.next()` を引数なしで呼び出し
- 🟡 no-implicit-any-catch: catchError のエラー引数の型

### detect_memory_leak
- ステータス: ✅ Clean（takeUntil(notifier$)、take(N) パターン検出のため）

### analyze_operators
- 主な指摘: concatMap/mergeMap の併用で複雑性

---

## docs/guide/debugging/custom-tools.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) パターン検出のため）

### analyze_operators
- 主な指摘: 基本的なオペレーターのみで問題なし

---

## docs/guide/debugging/index.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) パターン検出のため）

### analyze_operators
- 主な指摘: map + filter を統合検討

---

## docs/guide/debugging/performance.md

### lint_rxjs
- 🔴 no-sharereplay: `shareReplay(3)` の数値引数は安全でない
- 🟡 no-implicit-any-catch: catchError のエラー引数の型

### detect_memory_leak
- ステータス: ✅ Clean（take(N) パターン検出のため、shareReplay() の refCount オプション推奨）

### analyze_operators
- 主な指摘: shareReplay() のバッファ制限なし

---

## docs/guide/error-handling/error-handling-locations.md

### lint_rxjs
- 🟡 no-implicit-any-catch (×11): catchError のエラー引数の型を多数の箇所で指定推奨

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 3 subscribe に対して unsubscribe なし、完了オペレーター不使用

### analyze_operators
- 主な指摘: share()/shareReplay()、takeUntil() の追加検討

---

## docs/guide/error-handling/finalize.md

### lint_rxjs
- 🔴 no-unsafe-takeuntil: takeUntil の後に他のオペレーターが続く（finalize 以外）
- 🟡 no-implicit-any-catch (×2): catchError のエラー引数の型
- 🟡 no-redundant-notify: complete() 後の next() は無効
- 🟡 no-unsafe-subject-next: 型付き Subject の `.next()` を引数なしで呼び出し

### detect_memory_leak
- ステータス: ✅ Clean（takeUntil(notifier$)、take(N) パターン検出のため）

### analyze_operators
- 主な指摘: 基本的なオペレーター中心で問題なし

---

## docs/guide/error-handling/retry-catch.md

### lint_rxjs
- 🟡 no-implicit-any-catch (×5): catchError のエラー引数の型

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 4 subscribe に対して unsubscribe なし、完了オペレーター不使用

### analyze_operators
- 主な指摘:
  - **retryWhen は v7.3.0 から非推奨**。`retry({ delay: (error, retryCount) => ... })` を使用
  - share()/shareReplay()、takeUntil() の追加検討

---

## docs/guide/error-handling/strategies.md

### lint_rxjs
- 🟡 no-implicit-any-catch (×5): catchError のエラー引数の型

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 5 subscribe に対して unsubscribe なし、完了オペレーター不使用、interval/timer の限定オペレーター不使用

### analyze_operators
- 主な指摘:
  - **retryWhen は非推奨**
  - mergeMap/concatMap の併用で複雑性

---

## docs/guide/error-handling/try-catch-integration.md

### lint_rxjs
- 🟡 no-implicit-any-catch (×4): catchError のエラー引数の型

### detect_memory_leak
- ステータス: ✅ Clean（take(N) パターン検出のため）

### analyze_operators
- 主な指摘: share()/shareReplay() の追加検討

---

## docs/guide/observables/cold-and-hot-observables.md

### lint_rxjs
- 🔴 no-sharereplay (×2): `shareReplay(2)`、`shareReplay(1)` の数値引数は安全でない
- 🟡 prefer-observer (×2): `subscribe(fn, errfn, completefn)` 形式は非推奨
- 🟡 no-implicit-any-catch: catchError のエラー引数の型

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 4 Subject 中 0 件しか complete() なし、shareReplay() に refCount なし

### analyze_operators
- 主な指摘: shareReplay() のバッファ制限なし

---

## docs/guide/observables/creation.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 12 subscribe に対して unsubscribe なし、完了オペレーター不使用、interval/timer の限定オペレーター不使用、fromEvent のリスナーが解放されない可能性

### analyze_operators
- 主な指摘: share()/shareReplay()、catchError()/retry() の追加検討

---

## docs/guide/observables/events-list.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 2 subscribe に対して unsubscribe なし、完了オペレーター不使用、fromEvent のリスナーが解放されない可能性

### analyze_operators
- (hasPipe=false のため未実行)

---

## docs/guide/observables/events.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（takeUntil(notifier$) パターン検出のため）

### analyze_operators
- 主な指摘: map + filter を統合検討、share()/shareReplay()、catchError()/retry() の追加検討

---

## docs/guide/observables/observable-lifecycle.md

### lint_rxjs
- 🟡 no-implicit-any-catch: catchError のエラー引数の型

### detect_memory_leak
- ステータス: ✅ Clean（takeUntil(notifier$)、take(N) パターン検出のため）

### analyze_operators
- 主な指摘: 基本的なオペレーター中心で問題なし

---

## docs/guide/observables/observer-vs-subscriber.md

### lint_rxjs
- 🟡 prefer-observer: `subscribe(fn, errfn, completefn)` 形式は非推奨

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 3 subscribe に対して unsubscribe なし、完了オペレーター不使用、interval/timer の限定オペレーター不使用

### analyze_operators
- (hasPipe=false のため未実行)

---

## docs/guide/observables/what-is-observable.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 2 subscribe に対して unsubscribe なし、完了オペレーター不使用

### analyze_operators
- 主な指摘: 問題なし

---

## docs/guide/schedulers/async-control.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) パターン検出のため）

### analyze_operators
- 主な指摘: share()/shareReplay()、catchError()/retry() の追加検討

---

## docs/guide/schedulers/task-and-scheduler-basics.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 1 subscribe に対して unsubscribe なし、完了オペレーター不使用

### analyze_operators
- 主な指摘: 問題なし

---

## docs/guide/schedulers/types.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) パターン検出のため）

### analyze_operators
- 主な指摘:
  - **retryWhen は非推奨**
  - share()/shareReplay() の追加検討

---

## docs/guide/starter-kid.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) パターン検出のため）

### analyze_operators
- 主な指摘: 問題なし

---

## docs/guide/subjects/multicasting.md

### lint_rxjs
- 🔴 no-sharereplay (×2): `shareReplay(2)`、`shareReplay(1)` の数値引数は安全でない
- 🟡 prefer-observer (×3): `subscribe(fn, errfn, completefn)` 形式は非推奨
- 🟡 no-implicit-any-catch: catchError のエラー引数の型

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 2 Subject 中 0 件しか complete() なし、shareReplay() に refCount なし

### analyze_operators
- 主な指摘: shareReplay() のバッファ制限なし

---

## docs/guide/subjects/types-of-subject.md

### lint_rxjs
- 🟡 no-redundant-notify (×2): complete() 後の next() は無効

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 4 subscribe に対して unsubscribe なし、完了オペレーター不使用、4 Subject 中 2 件しか complete() なし

### analyze_operators
- (hasPipe=false のため未実行)

---

## docs/guide/subjects/use-cases.md

### lint_rxjs
- 🟡 no-implicit-any-catch: catchError のエラー引数の型
- 🟡 no-unsafe-subject-next: 型付き Subject の `.next()` を引数なしで呼び出し

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 7 Subject 中 1 件しか complete() なし

### analyze_operators
- 主な指摘: map + filter を統合検討

---

## docs/guide/subjects/what-is-subject.md

### lint_rxjs
- 🟡 no-redundant-notify: complete() 後の next() は無効

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 3 Subject 中 1 件しか complete() なし

### analyze_operators
- 主な指摘: 問題なし

---

## docs/guide/testing/marble-testing.md

### lint_rxjs
- No issues

### detect_memory_leak
- (hasSubscribe=false のため未実行)

### analyze_operators
- 主な指摘: Rate limiting オプションの説明（debounceTime/throttleTime/auditTime/sampleTime）

---

## docs/guide/testing/test-scheduler.md

### lint_rxjs
- No issues

### detect_memory_leak
- (hasSubscribe=false のため未実行)

### analyze_operators
- 主な指摘: 基本的なオペレーター中心で問題なし

---

## docs/guide/testing/unit-tests.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) パターン検出のため）

### analyze_operators
- 主な指摘: 基本的なオペレーター中心で問題なし

---

## docs/guide/typescript-advanced/_typeScript-and-rxjs-integration.md

### lint_rxjs
- 🟡 no-implicit-any-catch: catchError のエラー引数の型

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: 5 subscribe に対して unsubscribe なし、完了オペレーター不使用、3 Subject 中 0 件しか complete() なし

### analyze_operators
- 主な指摘: map + filter を統合検討、share()/shareReplay()、takeUntil() の追加検討

---

## docs/guide/typescript-advanced/type-safety.md

### lint_rxjs
- No issues

### detect_memory_leak
- (hasSubscribe=false のため未実行)

### analyze_operators
- (hasPipe=false のため未実行)

---
