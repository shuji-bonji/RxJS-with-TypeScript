# Issue #34: 網羅性チェックレポート

**生成日**: 2026-05-18

**基準**: 行数比 70%未満を翻訳ギャップとして検出 / セクション数差も検出


## サマリ

| 言語 | CRITICAL (50%未満) | WARN (50-70%) | SECTION 差 | INFO | 合計 |
|------|---|---|---|---|---|
| en | 1 | 7 | 4 | 0 | **13** |
| fr | 6 | 9 | 3 | 0 | **18** |
| de | 8 | 3 | 5 | 0 | **16** |
| it | 2 | 8 | 7 | 0 | **18** |
| es | 15 | 0 | 3 | 0 | **20** |
| nl | 3 | 9 | 5 | 0 | **17** |
| pt | 1 | 7 | 4 | 0 | **13** |

## 言語別詳細

### en

**CRITICAL** (1 ファイル)

| ファイル | JA 行 | en 行 | 比率 | JA h2 | en h2 |
|---|---|---|---|---|---|
| `operators/filtering/findIndex.md` | 410 | 191 | 47% | 9 | 6 |

**WARN** (7 ファイル)

| ファイル | JA 行 | en 行 | 比率 | JA h2 | en h2 |
|---|---|---|---|---|---|
| `operators/filtering/find.md` | 440 | 243 | 55% | 10 | 7 |
| `operators/filtering/sampleTime.md` | 366 | 201 | 55% | 9 | 8 |
| `operators/filtering/ignoreElements.md` | 468 | 262 | 56% | 10 | 8 |
| `operators/filtering/takeLast.md` | 396 | 232 | 59% | 10 | 7 |
| `operators/filtering/auditTime.md` | 410 | 244 | 60% | 10 | 7 |
| `operators/filtering/elementAt.md` | 339 | 205 | 60% | 8 | 7 |
| `operators/filtering/audit.md` | 294 | 199 | 68% | 9 | 8 |

**SECTION** (4 ファイル)

| ファイル | JA 行 | en 行 | 比率 | JA h2 | en h2 |
|---|---|---|---|---|---|
| `operators/combination/combineLatestAll.md` | 162 | 119 | 73% | 6 | 5 |
| `operators/utility/timestamp.md` | 372 | 289 | 78% | 9 | 8 |
| `practical-patterns/ui-events.md` | 1182 | 953 | 81% | 14 | 13 |
| `creation-functions/selection/index.md` | 125 | 139 | 111% | 6 | 7 |

**EXTRA** (1 ファイル)

| ファイル | JA 行 | en 行 | 比率 | JA h2 | en h2 |
|---|---|---|---|---|---|
| `operators/filtering/filter.md` | 76 | 178 | 234% | 3 | 7 |

### fr

**CRITICAL** (6 ファイル)

| ファイル | JA 行 | fr 行 | 比率 | JA h2 | fr h2 |
|---|---|---|---|---|---|
| `creation-functions/combination/forkJoin-vs-combineLatest.md` | 350 | 105 | 30% | 9 | 5 |
| `operators/transformation/bufferWhen.md` | 501 | 197 | 39% | 9 | 7 |
| `operators/transformation/windowWhen.md` | 390 | 156 | 40% | 11 | 8 |
| `operators/transformation/windowToggle.md` | 394 | 161 | 41% | 12 | 8 |
| `operators/transformation/bufferToggle.md` | 483 | 215 | 45% | 10 | 8 |
| `operators/transformation/windowTime.md` | 403 | 196 | 49% | 11 | 8 |

**WARN** (9 ファイル)

| ファイル | JA 行 | fr 行 | 比率 | JA h2 | fr h2 |
|---|---|---|---|---|---|
| `operators/utility/materialize.md` | 316 | 165 | 52% | 9 | 7 |
| `operators/transformation/groupBy.md` | 371 | 196 | 53% | 10 | 7 |
| `operators/transformation/window.md` | 328 | 178 | 54% | 10 | 7 |
| `operators/transformation/windowCount.md` | 302 | 163 | 54% | 11 | 8 |
| `operators/transformation/reduce.md` | 304 | 186 | 61% | 10 | 7 |
| `operators/utility/dematerialize.md` | 337 | 205 | 61% | 9 | 7 |
| `operators/transformation/pairwise.md` | 270 | 174 | 64% | 10 | 8 |
| `operators/combination/combineLatestWith.md` | 359 | 238 | 66% | 6 | 6 |
| `operators/combination/raceWith.md` | 369 | 249 | 67% | 6 | 6 |

**SECTION** (3 ファイル)

| ファイル | JA 行 | fr 行 | 比率 | JA h2 | fr h2 |
|---|---|---|---|---|---|
| `operators/utility/timestamp.md` | 372 | 289 | 78% | 9 | 8 |
| `creation-functions/selection/index.md` | 125 | 139 | 111% | 6 | 7 |
| `practical-patterns/caching-strategies.md` | 962 | 1345 | 140% | 11 | 13 |

### de

**CRITICAL** (8 ファイル)

| ファイル | JA 行 | de 行 | 比率 | JA h2 | de h2 |
|---|---|---|---|---|---|
| `operators/filtering/elementAt.md` | 339 | 58 | 17% | 8 | 4 |
| `operators/filtering/find.md` | 440 | 84 | 19% | 10 | 5 |
| `operators/filtering/ignoreElements.md` | 468 | 103 | 22% | 10 | 5 |
| `operators/filtering/findIndex.md` | 410 | 93 | 23% | 9 | 5 |
| `operators/filtering/skipUntil.md` | 385 | 92 | 24% | 10 | 5 |
| `operators/filtering/sampleTime.md` | 366 | 97 | 27% | 9 | 5 |
| `creation-functions/combination/forkJoin-vs-combineLatest.md` | 350 | 105 | 30% | 9 | 5 |
| `operators/filtering/skipWhile.md` | 485 | 207 | 43% | 9 | 6 |

**WARN** (3 ファイル)

| ファイル | JA 行 | de 行 | 比率 | JA h2 | de h2 |
|---|---|---|---|---|---|
| `appendix/reactive-patterns-beyond-rxjs.md` | 659 | 334 | 51% | 14 | 12 |
| `operators/filtering/skipLast.md` | 475 | 247 | 52% | 9 | 7 |
| `appendix/rxjs-and-reactive-streams-ecosystem.md` | 644 | 380 | 59% | 10 | 9 |

**SECTION** (5 ファイル)

| ファイル | JA 行 | de 行 | 比率 | JA h2 | de h2 |
|---|---|---|---|---|---|
| `creation-functions/basic/fromEvent.md` | 420 | 349 | 83% | 10 | 9 |
| `creation-functions/basic/interval.md` | 412 | 355 | 86% | 11 | 10 |
| `creation-functions/basic/timer.md` | 427 | 401 | 94% | 11 | 10 |
| `creation-functions/combination/forkJoin.md` | 89 | 91 | 102% | 3 | 4 |
| `overcoming-difficulties/debugging-guide.md` | 988 | 1007 | 102% | 9 | 10 |

### it

**CRITICAL** (2 ファイル)

| ファイル | JA 行 | it 行 | 比率 | JA h2 | it h2 |
|---|---|---|---|---|---|
| `creation-functions/combination/forkJoin-vs-combineLatest.md` | 350 | 105 | 30% | 9 | 5 |
| `operators/filtering/findIndex.md` | 410 | 191 | 47% | 9 | 6 |

**WARN** (8 ファイル)

| ファイル | JA 行 | it 行 | 比率 | JA h2 | it h2 |
|---|---|---|---|---|---|
| `operators/transformation/expand.md` | 692 | 347 | 50% | 10 | 10 |
| `operators/filtering/find.md` | 440 | 243 | 55% | 10 | 7 |
| `operators/filtering/sampleTime.md` | 366 | 201 | 55% | 9 | 8 |
| `operators/filtering/ignoreElements.md` | 468 | 262 | 56% | 10 | 8 |
| `operators/filtering/takeLast.md` | 396 | 232 | 59% | 10 | 7 |
| `operators/filtering/auditTime.md` | 410 | 244 | 60% | 10 | 7 |
| `operators/filtering/elementAt.md` | 339 | 205 | 60% | 8 | 7 |
| `operators/filtering/audit.md` | 294 | 199 | 68% | 9 | 8 |

**SECTION** (7 ファイル)

| ファイル | JA 行 | it 行 | 比率 | JA h2 | it h2 |
|---|---|---|---|---|---|
| `creation-functions/loop/generate.md` | 613 | 427 | 70% | 12 | 9 |
| `operators/combination/combineLatestAll.md` | 162 | 119 | 73% | 6 | 5 |
| `operators/utility/timestamp.md` | 372 | 289 | 78% | 9 | 8 |
| `creation-functions/loop/range.md` | 491 | 423 | 86% | 12 | 11 |
| `operators/transformation/groupBy.md` | 371 | 375 | 101% | 10 | 11 |
| `creation-functions/selection/index.md` | 125 | 139 | 111% | 6 | 7 |
| `subjects/use-cases.md` | 788 | 933 | 118% | 9 | 10 |

**EXTRA** (1 ファイル)

| ファイル | JA 行 | it 行 | 比率 | JA h2 | it h2 |
|---|---|---|---|---|---|
| `operators/transformation/mergeScan.md` | 199 | 356 | 179% | 7 | 9 |

### es

**CRITICAL** (15 ファイル)

| ファイル | JA 行 | es 行 | 比率 | JA h2 | es h2 |
|---|---|---|---|---|---|
| `operators/filtering/auditTime.md` | 410 | 37 | 9% | 10 | 2 |
| `operators/filtering/skipLast.md` | 475 | 58 | 12% | 9 | 4 |
| `operators/filtering/skipWhile.md` | 485 | 60 | 12% | 9 | 4 |
| `operators/filtering/ignoreElements.md` | 468 | 70 | 15% | 10 | 5 |
| `operators/filtering/find.md` | 440 | 69 | 16% | 10 | 3 |
| `operators/filtering/sampleTime.md` | 366 | 60 | 16% | 9 | 4 |
| `operators/filtering/elementAt.md` | 339 | 56 | 17% | 8 | 4 |
| `operators/filtering/skipUntil.md` | 385 | 70 | 18% | 10 | 3 |
| `operators/filtering/takeLast.md` | 396 | 74 | 19% | 10 | 4 |
| `operators/filtering/takeWhile.md` | 385 | 75 | 19% | 11 | 3 |
| `operators/filtering/distinct.md` | 324 | 64 | 20% | 9 | 3 |
| `operators/filtering/audit.md` | 294 | 66 | 22% | 9 | 5 |
| `operators/filtering/findIndex.md` | 410 | 92 | 22% | 9 | 5 |
| `creation-functions/combination/forkJoin-vs-combineLatest.md` | 350 | 105 | 30% | 9 | 5 |
| `operators/filtering/skip.md` | 312 | 150 | 48% | 9 | 5 |

**SECTION** (3 ファイル)

| ファイル | JA 行 | es 行 | 比率 | JA h2 | es h2 |
|---|---|---|---|---|---|
| `operators/combination/combineLatestAll.md` | 162 | 119 | 73% | 6 | 5 |
| `operators/utility/timestamp.md` | 372 | 289 | 78% | 9 | 8 |
| `practical-patterns/index.md` | 272 | 239 | 88% | 9 | 8 |

**EXTRA** (2 ファイル)

| ファイル | JA 行 | es 行 | 比率 | JA h2 | es h2 |
|---|---|---|---|---|---|
| `operators/filtering/filter.md` | 76 | 178 | 234% | 3 | 7 |
| `operators/transformation/scan.md` | 66 | 283 | 429% | 3 | 10 |

### nl

**CRITICAL** (3 ファイル)

| ファイル | JA 行 | nl 行 | 比率 | JA h2 | nl h2 |
|---|---|---|---|---|---|
| `creation-functions/combination/forkJoin-vs-combineLatest.md` | 350 | 105 | 30% | 9 | 5 |
| `operators/filtering/findIndex.md` | 410 | 191 | 47% | 9 | 6 |
| `operators/filtering/skipLast.md` | 475 | 233 | 49% | 9 | 7 |

**WARN** (9 ファイル)

| ファイル | JA 行 | nl 行 | 比率 | JA h2 | nl h2 |
|---|---|---|---|---|---|
| `operators/transformation/expand.md` | 692 | 367 | 53% | 10 | 7 |
| `operators/filtering/find.md` | 440 | 243 | 55% | 10 | 7 |
| `operators/filtering/sampleTime.md` | 366 | 201 | 55% | 9 | 8 |
| `operators/filtering/ignoreElements.md` | 468 | 262 | 56% | 10 | 8 |
| `operators/filtering/takeLast.md` | 396 | 232 | 59% | 10 | 7 |
| `operators/filtering/auditTime.md` | 410 | 244 | 60% | 10 | 7 |
| `operators/filtering/elementAt.md` | 339 | 205 | 60% | 8 | 7 |
| `operators/filtering/audit.md` | 294 | 199 | 68% | 9 | 8 |
| `operators/transformation/bufferToggle.md` | 483 | 332 | 69% | 10 | 8 |

**SECTION** (5 ファイル)

| ファイル | JA 行 | nl 行 | 比率 | JA h2 | nl h2 |
|---|---|---|---|---|---|
| `operators/combination/combineLatestAll.md` | 162 | 119 | 73% | 6 | 5 |
| `operators/utility/timestamp.md` | 372 | 289 | 78% | 9 | 8 |
| `operators/filtering/skipUntil.md` | 385 | 304 | 79% | 10 | 9 |
| `practical-patterns/ui-events.md` | 1182 | 946 | 80% | 14 | 13 |
| `creation-functions/selection/index.md` | 125 | 139 | 111% | 6 | 7 |

### pt

**CRITICAL** (1 ファイル)

| ファイル | JA 行 | pt 行 | 比率 | JA h2 | pt h2 |
|---|---|---|---|---|---|
| `operators/filtering/findIndex.md` | 410 | 191 | 47% | 9 | 6 |

**WARN** (7 ファイル)

| ファイル | JA 行 | pt 行 | 比率 | JA h2 | pt h2 |
|---|---|---|---|---|---|
| `operators/filtering/find.md` | 440 | 243 | 55% | 10 | 7 |
| `operators/filtering/sampleTime.md` | 366 | 201 | 55% | 9 | 8 |
| `operators/filtering/ignoreElements.md` | 468 | 262 | 56% | 10 | 8 |
| `operators/filtering/takeLast.md` | 396 | 232 | 59% | 10 | 7 |
| `operators/filtering/auditTime.md` | 410 | 244 | 60% | 10 | 7 |
| `operators/filtering/elementAt.md` | 339 | 205 | 60% | 8 | 7 |
| `operators/filtering/audit.md` | 294 | 199 | 68% | 9 | 8 |

**SECTION** (4 ファイル)

| ファイル | JA 行 | pt 行 | 比率 | JA h2 | pt h2 |
|---|---|---|---|---|---|
| `operators/combination/combineLatestAll.md` | 162 | 119 | 73% | 6 | 5 |
| `operators/utility/timestamp.md` | 372 | 289 | 78% | 9 | 8 |
| `practical-patterns/ui-events.md` | 1182 | 946 | 80% | 14 | 13 |
| `creation-functions/selection/index.md` | 125 | 139 | 111% | 6 | 7 |

**EXTRA** (1 ファイル)

| ファイル | JA 行 | pt 行 | 比率 | JA h2 | pt h2 |
|---|---|---|---|---|---|
| `operators/filtering/filter.md` | 76 | 178 | 234% | 3 | 7 |

## 横断的ギャップ（複数言語で同じファイルに問題）

| ファイル | 問題のある言語 | 言語数 |
|---|---|---|
| `operators/filtering/elementAt.md` | en, de, it, es, nl, pt | 6 |
| `operators/filtering/find.md` | en, de, it, es, nl, pt | 6 |
| `operators/filtering/findIndex.md` | en, de, it, es, nl, pt | 6 |
| `operators/filtering/ignoreElements.md` | en, de, it, es, nl, pt | 6 |
| `operators/filtering/sampleTime.md` | en, de, it, es, nl, pt | 6 |
| `creation-functions/combination/forkJoin-vs-combineLatest.md` | fr, de, it, es, nl | 5 |
| `operators/filtering/audit.md` | en, it, es, nl, pt | 5 |
| `operators/filtering/auditTime.md` | en, it, es, nl, pt | 5 |
| `operators/filtering/takeLast.md` | en, it, es, nl, pt | 5 |
| `operators/filtering/skipLast.md` | de, es, nl | 3 |
| `operators/filtering/skipUntil.md` | de, es | 2 |
| `operators/filtering/skipWhile.md` | de, es | 2 |
| `operators/transformation/bufferToggle.md` | fr, nl | 2 |
| `operators/transformation/expand.md` | it, nl | 2 |
