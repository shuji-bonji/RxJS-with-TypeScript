# Group A レポート（operators 前半）

## docs/guide/operators/combination/combineLatestAll.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean
- 検出された問題: なし（take(N)による自動クリーンアップを検出）

### analyze_operators
- 主な指摘: なし。Higher-order Observable を combineLatestAll でフラット化する典型例

---

## docs/guide/operators/combination/combineLatestWith.md

### lint_rxjs
- no-implicit-any-catch (warning): catchError の err パラメータに `unknown` 型注釈を追加するべき

### detect_memory_leak
- ステータス: ✅ Clean

### analyze_operators
- 主な指摘: チェーンが長め（map→take→combineLatestWith→debounceTime→startWith）。debounceTime の代替として throttleTime / auditTime / sampleTime も検討余地あり

---

## docs/guide/operators/combination/concatAll.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean

### analyze_operators
- 主な指摘: なし

---

## docs/guide/operators/combination/concatWith.md

### lint_rxjs
- no-implicit-any-catch (warning): catchError の err パラメータに型注釈推奨

### detect_memory_leak
- ステータス: ✅ Clean

### analyze_operators
- 主な指摘: **mapTo が deprecated**（since 7.2.0）。`map(() => value)` への置き換えを推奨

---

## docs/guide/operators/combination/exhaustAll.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean

### analyze_operators
- 主な指摘: なし

---

## docs/guide/operators/combination/index.md

### lint_rxjs
- No issues

### detect_memory_leak
- subscribe を含まないため未実施

### analyze_operators
- 主な指摘: 概念紹介ページ。Creation Function 版と Pipeable Operator 版の対応関係を示す例として妥当

---

## docs/guide/operators/combination/mergeAll.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean

### analyze_operators
- 主な指摘: なし

---

## docs/guide/operators/combination/mergeWith.md

### lint_rxjs
- no-implicit-any-catch (warning): catchError の err に型注釈推奨

### detect_memory_leak
- ステータス: ✅ Clean

### analyze_operators
- 主な指摘: 8 オペレーターの長いチェーン。map+filter 統合余地あり。share/shareReplay 検討の提案あり

---

## docs/guide/operators/combination/practical-use-cases.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 呼び出しに対し unsubscribe なし（複数箇所）
  - 完了オペレーターなしの subscription
  - fromEvent によるイベントリスナーが解除されない可能性

### analyze_operators
- 主な指摘: 多数の creation/operator を組み合わせた包括的サンプル。switchMap + debounceTime のレート制御パターンが含まれ妥当。share/shareReplay の活用余地あり

---

## docs/guide/operators/combination/raceWith.md

### lint_rxjs
- no-implicit-any-catch (warning × 2): catchError の err に型注釈推奨

### detect_memory_leak
- ステータス: ✅ Clean

### analyze_operators
- 主な指摘: switchMap と mergeMap が混在 → フラット化オペレータ複数による複雑さの注意。takeUntil 検討提案

---

## docs/guide/operators/combination/switchAll.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean

### analyze_operators
- 主な指摘: debounceTime + switchAll の検索パターンとして適切

---

## docs/guide/operators/combination/withLatestFrom.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 2 回に対して unsubscribe なし
  - 完了オペレーターなし
  - 無限 interval/timer に limit operator 未付与（`timer$ = interval(1000)`）
  - fromEvent のリスナー解除に注意

### analyze_operators
- 主な指摘: チェーン自体は短くシンプル

---

## docs/guide/operators/combination/zipAll.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean

### analyze_operators
- 主な指摘: なし。バックプレッシャー説明と整合

---

## docs/guide/operators/combination/zipWith.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean

### analyze_operators
- 主な指摘: combineLatestWith との対比例を含み教育的に充実

---

## docs/guide/operators/conditional/defaultIfEmpty.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 呼び出しに対して unsubscribe なし
  - 完了オペレーター（take/takeUntil/first）なしの subscription
- 補足: EMPTY/from([]) は即完了するため実害は少ないが、教育目的としては明示があると望ましい

### analyze_operators
- 主な指摘: なし。defaultIfEmpty + delay の典型例

---

## docs/guide/operators/conditional/every.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 多数に対して unsubscribe なし
  - 完了オペレーターなし
  - fromEvent によるイベントリスナーが解除されない可能性

### analyze_operators
- 主な指摘: every オペレータ単体は短いチェーン。フォームバリデーション実例として combineLatest + every の組み合わせが妥当

---

## docs/guide/operators/conditional/isEmpty.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 多数に対して unsubscribe なし
  - fromEvent のリスナー解除に注意

### analyze_operators
- 主な指摘: switchMap + debounceTime の検索パターンを含む。share/shareReplay や catchError/retry の追加検討提案あり

---

## docs/guide/operators/conditional/practical-use-cases.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks（同様パターン）
- 検出された問題: subscribe に unsubscribe が無い・fromEvent 解除なし

### analyze_operators
- 主な指摘: defaultIfEmpty / every / isEmpty を組み合わせた実例集。条件オペレータの統合的なチェーンとして妥当

---

## docs/guide/operators/filtering/audit.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 6 件に対して unsubscribe なし
  - 完了オペレーターなし
  - 無限 interval/timer に limit operator なし（audit の notifier の interval/timer は問題ないが、メイン側にも適用される指摘）
  - fromEvent のリスナー解除に注意

### analyze_operators
- 主な指摘: audit/auditTime の比較として適切。レート制御の代替案（debounceTime/throttleTime/sampleTime）が提示される

---

## docs/guide/operators/filtering/auditTime.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 自動クリーンアップ検出）

### analyze_operators
- 主な指摘: auditTime + throttleTime + debounceTime の対比例を含む。各レート制御オペレータの違いを示す例として教育的に充実

---

## docs/guide/operators/filtering/debounceTime.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 1 件に対し unsubscribe なし
  - 完了オペレーターなし
  - fromEvent のリスナー解除に注意

### analyze_operators
- 主な指摘: シンプルなチェーン（map → debounceTime）。レート制御の代替案提示あり

---

## docs/guide/operators/filtering/distinct.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 5 件に対し unsubscribe なし
  - 完了オペレーターなし
  - 無限 interval（ストリームの内部例として "悪い例" を含む）
  - Subject が作成されているが complete() なし
  - fromEvent のリスナー解除に注意
- 補足: 記事自体に「無限ストリーム + distinct」のメモリリーク警告と timer(1000) によるリセット例が含まれており教育的に妥当

### analyze_operators
- 主な指摘: distinct と distinctUntilChanged の対比あり

---

## docs/guide/operators/filtering/distinctUntilChanged.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscribe 2 件に unsubscribe なし、完了オペレーターなし、fromEvent のリスナー解除注意

### analyze_operators
- 主な指摘: シンプルなチェーン

---

## docs/guide/operators/filtering/distinctUntilKeyChanged.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscribe に unsubscribe なし、fromEvent のリスナー解除注意

### analyze_operators
- 主な指摘: scan を組み合わせた累積状態の例として妥当。takeUntil 検討提案あり

---

## docs/guide/operators/filtering/elementAt.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 4 件に unsubscribe なし
  - 完了オペレーターなし
  - 無限 interval に limit operator なし
  - fromEvent のリスナー解除注意

### analyze_operators
- 主な指摘: elementAt + map のシンプルな構成

---

## docs/guide/operators/filtering/filter.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscribe に unsubscribe なし、fromEvent のリスナー解除注意

### analyze_operators
- 主な指摘: 最小構成のチェーン（filter → map）

---

## docs/guide/operators/filtering/find.md

### lint_rxjs
- **no-nested-subscribe (error)**: ネストされた subscribe が検出された（fromEvent click 内で from(products).subscribe を呼ぶパターン）。switchMap などのフラット化オペレータへのリファクタリング推奨

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscribe 3 件に unsubscribe なし、fromEvent のリスナー解除注意

### analyze_operators
- 主な指摘: find と first/filter の比較あり

---

## docs/guide/operators/filtering/findIndex.md

### lint_rxjs
- **no-nested-subscribe (error × 2)**: ネストされた subscribe が複数箇所で検出された（ボタンの click subscribe 内で from(tasks).subscribe を呼ぶパターン）。switchMap などのフラット化推奨

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscribe 3 件に unsubscribe なし、fromEvent のリスナー解除注意

### analyze_operators
- 主な指摘: findIndex 単体のシンプル構成

---

## docs/guide/operators/filtering/first.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（first() 自動クリーンアップを検出）

### analyze_operators
- 主な指摘: 最小構成。first() の役割が分かりやすい

---

## docs/guide/operators/filtering/ignoreElements.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題: subscribe 2 件に unsubscribe なし
- 補足: forkJoin で結合された ignoreElements ストリームは完了で終わるため、現実的なリーク影響は小さい

### analyze_operators
- 主な指摘: ignoreElements + tap + delay + forkJoin の初期化パターンとして妥当

---

## docs/guide/operators/filtering/last.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) と last() の自動クリーンアップ）

### analyze_operators
- 主な指摘: filter → take → last の組み合わせがシンプルで分かりやすい

---

## docs/guide/operators/filtering/practical-use-cases.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 2 件に unsubscribe なし
  - 完了オペレーターなし
  - fromEvent のリスナー解除注意

### analyze_operators
- 主な指摘: 6 オペレーターのチェーン。スクロールページング・検索のレート制御の代替案提示あり。takeUntil 検討提案

---

## docs/guide/operators/filtering/sampleTime.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 3 件に unsubscribe なし
  - 無限 interval（sensor$ シミュレート）に limit operator なし
  - fromEvent のリスナー解除注意

### analyze_operators
- 主な指摘: sampleTime + map のシンプル構成

---

## docs/guide/operators/filtering/skip.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 自動クリーンアップ）
- 補足: 記事内に「無限 interval + skip」コードあり。実行時はメモリ／CPU の継続消費に注意したい

### analyze_operators
- 主な指摘: skip / take / skipWhile の比較ができる構成

---

## docs/guide/operators/filtering/skipLast.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 自動クリーンアップ）

### analyze_operators
- 主な指摘: concatMap + take + skipLast の構成。share/shareReplay と catchError/retry の検討提案

---

## docs/guide/operators/filtering/skipUntil.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（takeUntil + take(N) 自動クリーンアップ）

### analyze_operators
- 主な指摘: skipUntil と takeUntil の対比例として教育的に良い構成

---

## docs/guide/operators/filtering/skipWhile.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 自動クリーンアップ）

### analyze_operators
- 主な指摘: skipWhile + take + map の組み合わせ。温度センサーシミュレーションの例として妥当

---

## docs/guide/operators/filtering/take.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（take(N) 自動クリーンアップ）

### analyze_operators
- 主な指摘: 最小構成（take 単体）

---

## docs/guide/operators/filtering/takeLast.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 3 件に unsubscribe なし
  - 完了オペレーターなし
  - Subject の complete() なし
  - fromEvent のリスナー解除注意
- 補足: takeLast はソース完了で発火するため、Subject に complete() を呼ばないとレポートされる値が出ない。記事内で複数 subscribe しているが、Subject の inputs$ が complete() されない場合の挙動に注意

### analyze_operators
- 主な指摘: takeLast 単体

---

## docs/guide/operators/filtering/takeWhile.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ✅ Clean（takeWhile() 自動クリーンアップ）

### analyze_operators
- 主な指摘: takeWhile + switchMap + scan の組み合わせ。share/shareReplay と catchError/retry の検討提案

---

## docs/guide/operators/filtering/throttleTime.md

### lint_rxjs
- No issues

### detect_memory_leak
- ステータス: ⚠️ Potential leaks
- 検出された問題:
  - subscribe 2 件に unsubscribe なし
  - 完了オペレーターなし
  - fromEvent のリスナー解除注意

### analyze_operators
- 主な指摘: throttleTime + map のシンプル構成。レート制御の代替案提示あり

---
