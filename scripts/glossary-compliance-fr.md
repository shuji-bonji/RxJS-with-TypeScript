# French (フランス語) 用語遵守チェック結果

## サマリー

| 項目 | 値 |
|------|-----|
| 検査ファイル数 | 181 |
| 違反のあるファイル数 | 6 |
| 違反総数 | 36 |

### 違反タイプ別

| タイプ | 件数 |
|--------|------|
| 🔴 no_translate (固有名詞消失) | 8 |
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

### `creation-functions/basic/fromEvent.md` (違反 2 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `window` | 5 | 0 | 0 |
| `count` | 5 | 0 | 0 |

### `creation-functions/basic/timer.md` (違反 2 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `Subject` | 3 | 0 | 0 |
| `map` | 6 | 0 | 0 |

### `creation-functions/index.md` (違反 1 件)

**no_translate 違反:**

| 用語 | JA出現 | 翻訳出現 | 比率 |
|------|--------|----------|------|
| `Observable` | 13 | 0 | 0 |

### `operators/filtering/filter.md` (違反 1 件)

**callouts JA 残存:**

- L74: `> [!WARNING] 本番コードでの注意`

### `operators/filtering/sampleTime.md` (違反 1 件)

**callouts JA 残存:**

- L30: `> [!WARNING] 本番コードでの注意`


---

生成日時: 2026-05-20T12:30:23.261Z
