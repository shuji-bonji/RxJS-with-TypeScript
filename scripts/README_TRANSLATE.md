# 翻訳スクリプト 使い方ガイド

`scripts/translate_files.py` は Issue #34（網羅性チェック）で検出された翻訳ギャップを DeepL API で補完するためのローカル実行用スクリプトです。

## クイックスタート

```bash
# 1. 依存インストール
pip install deepl --break-system-packages

# 2. DeepL API キーを設定
export DEEPL_AUTH_KEY="your-deepl-api-key"

# 3. まずドライランで消費文字数を確認
python scripts/translate_files.py --dry-run

# 4. 1 ファイル × 1 言語で試行
python scripts/translate_files.py --file operators/filtering/elementAt.md --lang fr

# 5. 結果を確認したら全件実行
python scripts/translate_files.py
```

## 前提条件

- **Python 3.10+**
- **DeepL API キー** — [DeepL アカウント](https://www.deepl.com/account/usage) から取得
- **deepl パッケージ** — `pip install deepl`
- **Glossary 配置** — `.claude/skills/rxjs-glossary/glossary.json` がリポジトリに存在すること

## オプション

| オプション | 説明 |
|-----------|------|
| `--file FILE` / `-f FILE` | 特定のファイルだけ処理 (例: `operators/filtering/audit.md`) |
| `--files A,B,C` | 複数ファイルをカンマ区切り |
| `--lang LANG` / `-l LANG` | 特定の言語だけ (`fr`, `de`, `it`, `es`, `nl`, `pt`) |
| `--dry-run` / `-n` | DeepL を実際には呼ばず、消費文字数のみ表示 |

## 対象ファイル (デフォルト)

Issue #34 で「横断的に翻訳不足」と判定された 8 ファイル：

1. `operators/filtering/audit.md` (一部 fr のみ完全、要再生成)
2. `operators/filtering/auditTime.md`
3. `operators/filtering/elementAt.md`
4. `operators/filtering/find.md`
5. `operators/filtering/findIndex.md`
6. `operators/filtering/ignoreElements.md`
7. `operators/filtering/sampleTime.md`
8. `operators/filtering/takeLast.md`

## 対象言語

- `fr` (French)
- `de` (German)
- `it` (Italian)
- `es` (Spanish)
- `nl` (Dutch)
- `pt` (Portuguese)

## ワークフロー

スクリプトは Skill `rxjs-vitepress-i18n` の手順に従って以下を自動化：

1. **保護パッケージ生成** — JA Markdown から Frontmatter / コードブロック / Mermaid / テーブル / VitePress callout を抽出して placeholder に置換
2. **DeepL 翻訳** — 1 言語あたり 4-5 call:
   - Frontmatter `description`
   - 保護版本文
   - 各テーブルのセル (改行区切り)
   - コード内日本語フラグメント (改行区切り)
3. **後処理・組み立て**:
   - プレースホルダー破損修正 (`TABLA_N`, `CODICE_N` 等を `___TABLE_N___` に戻す)
   - 末尾ピリオド除去
   - プレースホルダー復元 (大文字小文字無視)
   - 内部リンクパス `/guide/` → `/<lang>/guide/`
   - 言語別のパス汚染修正 (例: `/guida/funzioni di creazione/` → `/it/guide/creation-functions/`)
   - Glossary によるコード内日本語の置換 (固有名詞保護)
   - 語順問題の修正 (`valeur1émission` → `Émet valeur 1` など)

## 消費量見積もり

| 対象 | 1 言語あたり | 6 言語合計 |
|------|------------|-----------|
| audit.md (294行) | ~10,000 字 | ~60,000 字 |
| auditTime.md (410行) | ~14,000 字 | ~85,000 字 |
| elementAt.md (339行) | ~11,000 字 | ~65,000 字 |
| find.md (440行) | ~15,000 字 | ~90,000 字 |
| findIndex.md (410行) | ~14,000 字 | ~85,000 字 |
| ignoreElements.md (468行) | ~16,000 字 | ~95,000 字 |
| sampleTime.md (366行) | ~12,000 字 | ~75,000 字 |
| takeLast.md (396行) | ~13,000 字 | ~80,000 字 |
| **合計** | | **~635,000 字** |

DeepL Free 版の 100 万文字制限の **約 63%** を消費する想定。
`--dry-run` で事前にチェックしてから本実行してください。

## 失敗時の対処

### `deepl パッケージが必要です` エラー

```bash
pip install deepl
# 環境による：
pip install --break-system-packages deepl   # PEP 668 環境
pip install --user deepl                      # ユーザー領域
```

### `DEEPL_AUTH_KEY が設定されていません` エラー

```bash
# Bash/Zsh
export DEEPL_AUTH_KEY="your-deepl-api-key"

# Fish
set -x DEEPL_AUTH_KEY "your-deepl-api-key"

# 永続化（zsh の場合）
echo 'export DEEPL_AUTH_KEY="your-deepl-api-key"' >> ~/.zshrc
source ~/.zshrc
```

### Glossary 警告

```
⚠️  Glossary が見つからない: .../glossary.json
```

`.claude/skills/rxjs-glossary/glossary.json` を Cowork outputs から配置してください。詳細は `outputs/skills/INSTALL.md` を参照。

### 翻訳結果に残存日本語がある

スクリプト実行ログに `⚠️  残存JP N` と表示される場合、Glossary でカバーできなかった新しい日本語パターンがあることを示します。

対応：
1. 残存している日本語を確認 (`grep -P '[぀-ゟ゠-ヿ一-鿿]' docs/<lang>/guide/<file>` 等)
2. `glossary.json` の該当言語の `code_jp` に追記
3. スクリプトを再実行

## 実行後の確認

```bash
# 行数比較 (網羅性チェック)
for lang in fr de it es nl pt; do
  for file in audit auditTime elementAt find findIndex ignoreElements sampleTime takeLast; do
    ja=$(wc -l < "docs/guide/operators/filtering/$file.md")
    l=$(wc -l < "docs/$lang/guide/operators/filtering/$file.md")
    echo "  $lang/$file: JA $ja → $lang $l (${l}/$ja $(echo "scale=0; $l*100/$ja" | bc)%)"
  done
done

# 残存日本語チェック
for lang in fr de it es nl pt; do
  for file in audit auditTime elementAt find findIndex ignoreElements sampleTime takeLast; do
    path="docs/$lang/guide/operators/filtering/$file.md"
    n=$(grep -P '[぀-ゟ゠-ヿ一-鿿]' "$path" 2>/dev/null | wc -l)
    [ "$n" -gt 0 ] && echo "  ⚠️  $lang/$file: 残存 JP 行 $n"
  done
done

# ビルド検証
npm run docs:build
```

## 関連リンク

- Skill: `.claude/skills/rxjs-vitepress-i18n/SKILL.md`
- Glossary: `.claude/skills/rxjs-glossary/SKILL.md` / `glossary.json`
- 網羅性レポート: `audit/issue34-coverage-report.md`
- CLAUDE.md "Translation Workflow & i18n Maintenance" セクション
- CHANGELOG.md "Fifth Release" — Phase 1〜3 で確立した修正パターン
