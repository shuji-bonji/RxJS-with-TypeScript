# Group B レポート（operators 後半）

対象: 41 記事（operators/multicasting, operators/pipeline.md, operators/transformation/*, operators/utility/*）

凡例:
- lint_rxjs: recommended config (23 rules) で検出した issue
- detect_memory_leak: 自動クリーンアップ (take/takeUntil) があれば Clean、それ以外は無限ストリーム/fromEvent 等の警告
- analyze_operators: 主要オペレーターと改善提案

---

## docs/guide/operators/multicasting/share.md

### lint_rxjs
- prefer-observer (warning) x4: .subscribe(value => ...) をオブジェクト形式 .subscribe({ next: ... }) にすべき（heuristic）

### detect_memory_leak
- ステータス: Clean（take(N) で自動完了パターンを検出）

### analyze_operators
- 主要オペレーター: take → tap → share → switchMap / 生成関数 interval, timer
- catchError() または retry() の追加を提案

---

## docs/guide/operators/multicasting/shareReplay.md

### lint_rxjs
- no-sharereplay (error) x4: shareReplay(N) の数値引数は安全でない。{ bufferSize, refCount: true } を使うべき
- no-ignored-replay-buffer (error): shareReplay() を引数なしで呼ぶとメモリ問題のリスク
- prefer-observer (warning) x4

### detect_memory_leak
- ステータス: Clean（take(N) 検出）。ただし shareReplay() 無引数のコードは別途警告候補

### analyze_operators
- 主要オペレーター: take → tap → shareReplay → delay → share
- パフォーマンス警告: shareReplay() を buffer size 未指定で使うとメモリ問題のリスク（ドキュメント内の悪い例として意図的）

---

## docs/guide/operators/pipeline.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- subscribe 2件あり unsubscribe なし、fromEvent の購読解除欠如、完了オペレーター(take/takeUntil)欠如

### analyze_operators
- 主要オペレーター: filter → map → switchMap → debounceTime → tap（多数）
- share()/shareReplay(), catchError(), takeUntil() の追加提案

---

## docs/guide/operators/transformation/buffer.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- 無限 interval/timer（限定オペレーター無し）、fromEvent の解除欠如、subscribe 3件すべて unsubscribe なし

### analyze_operators
- 主要オペレーター: buffer / 生成関数 fromEvent, interval, timer, merge, race

---

## docs/guide/operators/transformation/bufferCount.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- 無限 interval、fromEvent 解除欠如、subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: bufferCount → map

---

## docs/guide/operators/transformation/bufferTime.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- 無限 interval、fromEvent 解除欠如、subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: bufferTime

---

## docs/guide/operators/transformation/bufferToggle.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- 無限 interval/timer、fromEvent 解除欠如、Subject の complete() 欠如、subscribe 4件 unsubscribe なし

### analyze_operators
- 主要オペレーター: bufferToggle → take → map → bufferWhen

---

## docs/guide/operators/transformation/bufferWhen.md

### lint_rxjs
- prefer-observer (warning) x2

### detect_memory_leak
- ステータス: Potential leaks
- 無限 interval/timer、fromEvent 解除欠如、subscribe 4件 unsubscribe なし

### analyze_operators
- 主要オペレーター: bufferWhen → take → bufferToggle → map → share → buffer → bufferTime → bufferCount

---

## docs/guide/operators/transformation/concatMap.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- fromEvent 解除欠如、subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: concatMap → delay
- share()/shareReplay(), catchError() の追加提案

---

## docs/guide/operators/transformation/exhaustMap.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- fromEvent 解除欠如、subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: exhaustMap → delay

---

## docs/guide/operators/transformation/expand.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: expand → take → mergeMap
- 代替案: switchMap/concatMap/exhaustMap の使い分け提案、catchError() 提案

---

## docs/guide/operators/transformation/groupBy.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- fromEvent 解除欠如、subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: groupBy → mergeMap → toArray → map → switchMap
- パフォーマンス警告: 複数のフラット化オペレーター (mergeMap, switchMap) は複雑化のリスク

---

## docs/guide/operators/transformation/map.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- fromEvent 解除欠如、subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: map

---

## docs/guide/operators/transformation/mergeMap.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- fromEvent 解除欠如、subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: mergeMap → delay
- 代替案: switchMap/concatMap/exhaustMap

---

## docs/guide/operators/transformation/mergeScan.md

### lint_rxjs
- prefer-observer (warning) x1

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: take → mergeScan → scan → delay → map → catchError

---

## docs/guide/operators/transformation/pairwise.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: take → pairwise → map → throttleTime
- Rate limiting 代替案: debounceTime / throttleTime / auditTime / sampleTime

---

## docs/guide/operators/transformation/practical-use-cases.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- fromEvent 解除欠如、subscribe 3件 unsubscribe なし

### analyze_operators
- 主要オペレーター: debounceTime → map → distinctUntilChanged → toArray
- takeUntil() 提案

---

## docs/guide/operators/transformation/reduce.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: reduce → scan → switchMap → take
- share()/shareReplay(), catchError() 提案

---

## docs/guide/operators/transformation/scan.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- fromEvent 解除欠如、subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: scan → tap

---

## docs/guide/operators/transformation/switchMap.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- fromEvent 解除欠如、subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: switchMap → delay → debounceTime → map
- share(), catchError(), takeUntil() 提案

---

## docs/guide/operators/transformation/window.md

### lint_rxjs
- no-nested-subscribe (error) x1: window の内側で window$.subscribe(...) をネスト購読している（フラット化演算子 mergeAll/switchAll 推奨）

### detect_memory_leak
- ステータス: Potential leaks
- 無限 interval/timer、fromEvent 解除欠如、subscribe 4件 unsubscribe なし

### analyze_operators
- 主要オペレーター: window → mergeAll → buffer

---

## docs/guide/operators/transformation/windowCount.md

### lint_rxjs
- no-nested-subscribe (error) x2: 入れ子の subscribe（windowCount の windows を直接購読）

### detect_memory_leak
- ステータス: Potential leaks
- 無限 interval、subscribe 3件 unsubscribe なし

### analyze_operators
- 主要オペレーター: windowCount → mergeAll → bufferCount → map → reduce

---

## docs/guide/operators/transformation/windowTime.md

### lint_rxjs
- no-nested-subscribe (error) x2: 入れ子の subscribe

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: windowTime → take → mergeAll → bufferTime → map → scan

---

## docs/guide/operators/transformation/windowToggle.md

### lint_rxjs
- no-nested-subscribe (error) x1: window$ を直接 subscribe している

### detect_memory_leak
- ステータス: Potential leaks
- 無限 interval、fromEvent 解除欠如、subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: windowToggle → mergeAll → bufferToggle → mergeMap → toArray

---

## docs/guide/operators/transformation/windowWhen.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: windowWhen → take → mergeAll → map → mergeMap → toArray

---

## docs/guide/operators/utility/delay.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- subscribe 3件 unsubscribe なし、完了オペレーター欠如

### analyze_operators
- 主要オペレーター: delay → concatMap → tap
- share()/shareReplay(), catchError() 提案

---

## docs/guide/operators/utility/delayWhen.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- fromEvent 解除欠如、subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: delayWhen → tap → take → delay

---

## docs/guide/operators/utility/dematerialize.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- subscribe 2件 unsubscribe なし

### analyze_operators
- 主要オペレーター: materialize → dematerialize → filter

---

## docs/guide/operators/utility/finalize.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: finalize → tap → delay → take

---

## docs/guide/operators/utility/materialize.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: materialize → map → take → mergeMap
- catchError(), share() 提案

---

## docs/guide/operators/utility/observeOn.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: observeOn → bufferCount → tap → take → map / 生成関数 range (loop)

---

## docs/guide/operators/utility/practical-use-cases.md

### lint_rxjs
- no-implicit-any-catch (warning) x1: catchError(err => ...) の型注釈不足

### detect_memory_leak
- ステータス: Potential leaks
- fromEvent 解除欠如、subscribe 1件 unsubscribe なし

### analyze_operators
- 主要オペレーター: map → startWith → debounceTime → tap → delay → catchError → finalize
- takeUntil() 提案

---

## docs/guide/operators/utility/repeat.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean

### analyze_operators
- 主要オペレーター: repeat → tap → delay

---

## docs/guide/operators/utility/retry.md

### lint_rxjs
- no-implicit-any-catch (warning) x1: catchError(error => ...) の型注釈不足

### detect_memory_leak
- ステータス: Potential leaks
- subscribe 2件 unsubscribe なし、無限 interval、完了オペレーター欠如

### analyze_operators
- 主要オペレーター: retry → catchError → tap → mergeMap
- share()/shareReplay(), takeUntil() 提案

---

## docs/guide/operators/utility/startWith.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: startWith → scan → take

---

## docs/guide/operators/utility/subscribeOn.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: subscribeOn → take → tap

---

## docs/guide/operators/utility/takeUntil.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（takeUntil(notifier$) 検出）

### analyze_operators
- 主要オペレーター: takeUntil

---

## docs/guide/operators/utility/tap.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Potential leaks
- subscribe 2件 unsubscribe なし、完了オペレーター欠如

### analyze_operators
- 主要オペレーター: tap → map

---

## docs/guide/operators/utility/timeout.md

### lint_rxjs
- no-implicit-any-catch (warning) x5: 複数の catchError(err => ...) の型注釈不足

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: delay → timeout → catchError → take

---

## docs/guide/operators/utility/timestamp.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: take → timestamp → pairwise → map

---

## docs/guide/operators/utility/toArray.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: Clean（take(N) 検出）

### analyze_operators
- 主要オペレーター: toArray → take → delayWhen → delay

---

## サマリ

### Lint 全体傾向（41 記事中）
- no-sharereplay / no-ignored-replay-buffer (error): shareReplay.md（5件）— 教材として意図的に "良くない例" を含む
- no-nested-subscribe (error): window.md, windowCount.md, windowTime.md, windowToggle.md（合計6件）— ネストされた subscribe をフラット化演算子で置き換える説明があると望ましい
- prefer-observer (warning): share.md, shareReplay.md, bufferWhen.md, mergeScan.md（合計11件）— .subscribe({ next, error, complete }) 形式への移行推奨
- no-implicit-any-catch (warning): practical-use-cases.md(utility), retry.md, timeout.md（合計7件）— catchError((err: unknown) => ...) への型注釈追加推奨

### Memory leak 全体傾向
- Clean（take/takeUntil による自動完了パターン検出）: 17 記事
- Potential leaks（fromEvent 解除欠如、無限 interval、subscribe 解除欠如等）: 24 記事
- 多くは "教材としてのデモコード" のため実害は限定的だが、takeUntil(destroy$) パターンの明示を強化する余地あり

### Analyze 全体傾向
- transformation/window 系・buffer 系オペレーターは複雑な組み合わせが多く、catchError や share の追加提案が共通
- 高次マッピング系（mergeMap, switchMap, concatMap, exhaustMap）の使い分け提案がほぼ全記事で表示
