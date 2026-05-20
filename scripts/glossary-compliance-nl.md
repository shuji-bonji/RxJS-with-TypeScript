# Dutch (オランダ語) 用語遵守チェック結果

## サマリー

| 項目 | 値 |
|------|-----|
| 検査ファイル数 | 181 |
| 違反のあるファイル数 | 5 |
| 違反総数 | 21 |

### 違反タイプ別

| タイプ | 件数 |
|--------|------|
| 🔴 no_translate (固有名詞消失) | 15 |
| 🟡 callouts JA 残存 | 1 |
| 🟠 code_jp 残存 (コード内日本語) | 5 |

## 違反詳細 (Top 30)

### `schedulers/types.md` (違反 11 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `timer` | 3 | 0 | 0 |
| `range` | 3 | 0 | 0 |
| `mergeMap` | 8 | 0 | 0 |
| `tap` | 5 | 0 | 0 |
| `count` | 5 | 0 | 0 |
| `Promise` | 4 | 0 | 0 |

**code_jp 残存 (コードブロック内):**

- L331: `注`
- L333: `これは概念を示す疑似コードです`
- L336: `文字列として扱う`
- L341: `高速な応答が必要なメッセージ処理`
- L349: `メッセージ受信`

### `practical-patterns/api-calls.md` (違反 4 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `Subject` | 4 | 0 | 0 |
| `exhaustMap` | 3 | 0 | 0 |
| `count` | 6 | 0 | 0 |
| `Promise` | 6 | 0 | 0 |

### `practical-patterns/ui-events.md` (違反 4 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `of` | 7 | 0 | 0 |
| `timer` | 4 | 0 | 0 |
| `buffer` | 4 | 0 | 0 |
| `catchError` | 4 | 0 | 0 |

### `operators/filtering/filter.md` (違反 1 件)

**callouts JA 残存:**

- L74: `> [!WARNING] 本番コードでの注意`

### `operators/filtering/skipUntil.md` (違反 1 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `Observable` | 3 | 0 | 0 |


---

生成日時: 2026-05-20T08:16:00.690Z
