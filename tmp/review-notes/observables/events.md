# Review: docs/guide/observables/events.md & docs/en/guide/observables/events.md

## Findings
1. キーボードイベントの例で、英語版は `filter(key => key.length === 1)` を入れて modifier キー等を弾いていますが、日本語版は `map` のみでコードが異なります。挙動が変わるため、どちらかに合わせてください。

## Status
- 技術面: ⚠️（コード挙動差）
- 翻訳面: ⚠️
