# Review: docs/guide/observables/creation.md & docs/en/guide/observables/creation.md

## Findings
1. `fromEvent()` の注意書き（JA lines 196-199 / EN lines 190-193）が「DOM 以外では使えない」「Node.js では使用不可」と断言していますが、実際には DOM 以外にも `EventTarget` 実装や Node.js の `EventEmitter` などを扱えます。読者が Node.js で使えないと誤解するため、対応可能なターゲットの説明に改めてください（英語版も同様）。

## Status
- 技術面: ⚠️
- 翻訳面: ✅
