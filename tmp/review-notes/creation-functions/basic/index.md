# Review: docs/guide/creation-functions/basic/index.md & docs/en/guide/creation-functions/basic/index.md

## Findings
1. メモリリーク注意書き（JA line 113 / EN line 113）で `fromEvent()`, `interval()`, `timer()` が「完了しない」とまとめられていますが、`timer(dueTime)` は 1 回で完了します。第2引数ありの `timer` や `interval` のみに限定するなど、正確な説明が必要です（EN 版も同じ）。

## Status
- 技術面: ⚠️
- 翻訳面: ⚠️
