# Italian (イタリア語) 用語遵守チェック結果

## サマリー

| 項目 | 値 |
|------|-----|
| 検査ファイル数 | 181 |
| 違反のあるファイル数 | 7 |
| 違反総数 | 45 |

### 違反タイプ別

| タイプ | 件数 |
|--------|------|
| 🔴 no_translate (固有名詞消失) | 17 |
| 🟡 callouts JA 残存 | 2 |
| 🟠 code_jp 残存 (コード内日本語) | 26 |

## 違反詳細 (Top 30)

### `schedulers/types.md` (違反 29 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `range` | 3 | 0 | 0 |
| `tap` | 5 | 0 | 0 |
| `Promise` | 4 | 0 | 0 |

**code_jp 残存 (コードブロック内):**

- L402: `推奨`
- L402: `形式`
- L404: `回まで再試行`
- L406: `指数バックオフ`
- L406: `秒`
- ... 他 21 件

### `practical-patterns/api-calls.md` (違反 6 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `Subject` | 4 | 0 | 0 |
| `exhaustMap` | 3 | 0 | 0 |
| `takeUntil` | 6 | 0 | 0 |
| `shareReplay` | 3 | 0 | 0 |
| `count` | 6 | 0 | 0 |
| `Promise` | 6 | 0 | 0 |

### `operators/filtering/elementAt.md` (違反 3 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `take` | 4 | 0 | 0 |
| `skip` | 4 | 0 | 0 |
| `first` | 5 | 0 | 0 |

### `creation-functions/loop/range.md` (違反 2 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `toArray` | 4 | 0 | 0 |
| `error` | 4 | 0 | 0 |

### `operators/filtering/sampleTime.md` (違反 2 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `sample` | 6 | 0 | 0 |

**callouts JA 残存:**

- L30: `> [!WARNING] 本番コードでの注意`

### `subjects/use-cases.md` (違反 2 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `takeUntil` | 3 | 0 | 0 |
| `WebSocket` | 11 | 0 | 0 |

### `operators/filtering/filter.md` (違反 1 件)

**callouts JA 残存:**

- L74: `> [!WARNING] 本番コードでの注意`


---

生成日時: 2026-05-20T12:30:23.688Z
