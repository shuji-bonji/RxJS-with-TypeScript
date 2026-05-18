# 翻訳品質評価ガイド (Issue #33)

本リポジトリの 6 言語 (fr/de/it/es/nl/pt) 翻訳の品質を、**XCOMET スコア** と **辞書遵守** の 2 軸で評価する。

## 評価レイヤーの分担

| レイヤー | スクリプト | 評価軸 | 出力 |
|---------|-----------|--------|------|
| **L1: XCOMET 評価** | `auto-evaluate.cjs` | 翻訳の流暢性・意味の保存（ニューラルモデル） | `evaluation-results-{lang}.json`<br>`evaluation-report-{lang}.md` |
| **L2: 辞書遵守チェック** | `validate-glossary-compliance.cjs` | 固有名詞消失・callout 残存・コード内 JA 残存 | `glossary-compliance-{lang}.json`<br>`glossary-compliance-{lang}.md` |

両者は補完関係。XCOMET が捕捉できない用語コンプライアンスを L2 が検出する。

---

## クイックスタート

### 1. 翻訳ペア抽出（ドキュメント変更後に必要）

ドキュメント (`docs/`) が更新されたら、まず翻訳ペアを再抽出する。

```bash
# 7 言語まとめて
for lang in en fr de it es nl pt; do
  node scripts/extract-translation-pairs-v2.cjs $lang
done
```

各 `scripts/translation-pairs-{lang}.json` が更新される。

### 2. XCOMET MCP Python server 起動

[xCOMET MCP](https://github.com/) がローカルで起動している必要がある。`mcp.json` 等で設定済みの場合、Claude Code 等から呼ばれた時点で自動起動する。手動起動する場合：

```bash
# XCOMET モデルがロード済み・GPU 推奨
# サーバーは http://127.0.0.1:<port>/batch_evaluate を提供
```

`auto-evaluate.cjs` は `lsof` で listening port を検出し、`/health` で `model_loaded: true` を確認する。

### 3. L1: XCOMET 評価実行

```bash
# 単一言語
node scripts/auto-evaluate.cjs fr

# 全 7 言語
for lang in en fr de it es nl pt; do
  node scripts/auto-evaluate.cjs $lang
done
```

各 `scripts/evaluation-results-{lang}.json` が更新される。**既評価ファイルはスキップ**されるため、再評価したい場合は対象ファイルのエントリを削除する。

### 4. レポート生成

```bash
# 単一言語
node scripts/generate-evaluation-report.cjs fr

# 全 7 言語
node scripts/generate-evaluation-report.cjs all
```

`scripts/evaluation-report-{lang}.md` が更新される。

### 5. L2: 辞書遵守チェック

```bash
# 単一言語
node scripts/validate-glossary-compliance.cjs fr

# 全 6 言語
node scripts/validate-glossary-compliance.cjs all

# 特定ファイルだけ詳細表示
node scripts/validate-glossary-compliance.cjs fr --file operators/filtering/audit.md
```

`scripts/glossary-compliance-{lang}.json` と `.md` が生成される。

---

## 評価結果の見方

### XCOMET スコア基準

| スコア帯 | マーク | 意味 |
|---------|--------|------|
| ≥ 0.95 | 🟢 | 優秀 |
| 0.85〜0.94 | 🔵 | 良好 |
| 0.70〜0.84 | 🟡 | 要改善 |
| < 0.70 | 🔴 | 要修正 (DeepL 再翻訳推奨) |

### 辞書違反タイプ

| タイプ | 説明 | 対応 |
|--------|------|------|
| 🔴 `no_translate` (固有名詞消失) | JA に 3 回以上出現する固有名詞 (forkJoin 等) が翻訳版で完全消失 | 該当ファイルを再翻訳または手動補完 |
| 🟡 `callouts_ja_residual` (callout JA 残存) | `> [!WARNING]` 見出しに JA フレーズが残存 | `fix_broken_translations.py` で部分対応可、それ以外は手動 |
| 🟠 `code_jp_residual` (コード内 JA) | コードブロック内コメントに JA 残存 | DeepL Glossary の `code_jp` に追加 → 再翻訳 |

---

## 既知の制約

### XCOMET MCP の利用条件
- **GPU 推奨**: CPU でも動くが、1 ファイル数十秒〜数分かかる
- **モデル容量**: XCOMET-XL は ~14 GB ディスク + 10-16 GB RAM
- **モデルロード時間**: 初回起動で 1〜2 分

### auto-evaluate.cjs の挙動
- セクション数が JA/翻訳版で異なるファイルは `extract-translation-pairs-v2.cjs` でスキップされる (`skippedFiles` カウント)
- スキップされたファイルは XCOMET 評価対象に入らない → Issue #34 の網羅性チェックで別途検出

### validate-glossary-compliance.cjs の判定方針
- `no_translate` 違反: **完全消失 (count=0)** かつ **JA に 3 回以上出現** の場合のみフラグ
  - 部分的な不足は「翻訳カバレッジ問題」として Issue #34 の網羅性チェックに委譲
- カウントは **コードブロック内・インラインコード内のみ** (散文中の英単語誤検出を排除)

---

## トラブルシューティング

### XCOMET server が見つからない

```
Error: xCOMET Python server not found.
```

→ MCP サーバーが起動しているか確認。`lsof -i -P | grep python3 | grep LISTEN` で port を確認。

### 重複 frontmatter エラー

ビルド時に Vue の補間エラーが出る、または validate-glossary-compliance で大量の `code_jp_residual` 違反が L300 以降に出現する場合：

```bash
python3 scripts/fix_broken_translations.py
```

中盤の重複 JA frontmatter を除去する。

### glossary 違反が大量に出る場合

`no_translate` 違反が 100 件以上出る場合、その言語の該当ファイルが**翻訳カバレッジ不足**の可能性が高い (Issue #34 の対象)。XCOMET スコアと突き合わせて、両方低いファイルから優先的に再翻訳する。

---

## ワークフロー全体図

```mermaid
flowchart TD
    A[docs/ 更新] --> B[extract-translation-pairs-v2.cjs]
    B --> C{translation-pairs-{lang}.json}
    C --> D[auto-evaluate.cjs]
    D --> E{evaluation-results-{lang}.json}
    E --> F[generate-evaluation-report.cjs]
    F --> G[evaluation-report-{lang}.md]

    A --> H[validate-glossary-compliance.cjs]
    H --> I{glossary-compliance-{lang}.json/.md}

    G -.スコア < 0.70 のファイル.-> J[translate_files.py で再翻訳]
    I -.code_jp 違反多数.-> K[glossary.json 拡充 → 再翻訳]
    J --> A
    K --> A
```

---

## 履歴的に重要なポイント (Issue #32-#34)

1. **Issue #32 (RxJS MCP 監査)** で 337 ファイル変更後は `translation-pairs-*.json` を必ず再抽出
2. **Issue #34 (網羅性)** で `translate_files.py` 実行後は **`fix_broken_translations.py` も必ず実行**
   - プレースホルダー復元 (`___CODE_N___` 等)
   - 重複 frontmatter 除去 (中盤に JA description が残るバグ)
   - 先頭空行除去
3. **Issue #33 (本タスク)** は Issue #32/#34 が完了してから評価を実行する
