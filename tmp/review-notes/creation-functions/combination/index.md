# Review: docs/guide/creation-functions/combination/index.md & docs/en/guide/creation-functions/combination/index.md

## Findings
1. 完了タイミングの説明（JA line 41-45 / EN line 41-45）が「zip はいずれか 1 つが完了したら完了」となっていますが、実際には最も短い Observable が尽きた時点（＝「完了したストリームに対して残りの値が揃わなくなった時点」）で完了します。現行の記述だと “1 本でも完了した瞬間に zip が即終了する” と誤解されるため、正しい挙動に沿った説明へ修正が必要です。

## Status
- 技術面: ⚠️
- 翻訳面: ⚠️（同じ説明が英語版にも存在）
