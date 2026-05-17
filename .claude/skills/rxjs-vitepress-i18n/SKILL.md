---
name: rxjs-vitepress-i18n
description: |
  RxJS-with-TypeScript プロジェクト (VitePress 多言語ドキュメント) の DeepL を使った翻訳ワークフロー。
  Markdown のコードブロック・Mermaid・テーブル・frontmatter・callout を保護してから DeepL に投入し、
  後処理で組み立てる手順。Issue #34 (網羅性チェック) で検出された翻訳不足ファイルを 6 言語 (fr/de/it/es/nl/pt)
  に補完する場面で使用する。Claude 駆動 MCP 経由ではなく、ローカル Python スクリプト + DeepL API 直叩きを推奨。
---

# RxJS-with-TypeScript 翻訳ワークフロー

## 適用範囲

- **対象ディレクトリ**: `docs/guide/` 配下の Markdown ファイル (181 ファイル)
- **対象言語**: en, fr, de, it, es, nl, pt
- **JA が原本**: 全言語は JA からの翻訳

## 全体フロー

```
JA Markdown
    ↓ Step 1: 保護パッケージ生成
保護版テキスト + blocks（コード/Mermaid/テーブル/frontmatter/callout）
    ↓ Step 2: DeepL 翻訳 (1ファイルあたり 4-5 call/言語)
各言語の翻訳結果
    ↓ Step 3: 後処理 (プレースホルダー復元・固有名詞修正・語順修正・リンクパス変換)
各言語の Markdown 完成版
```

## Step 1: 保護パッケージ生成

JA ファイルから次の要素を抽出してプレースホルダー `___<TYPE>_<N>___` に置換する：

1. **Frontmatter** (`---\n...\n---`) — `___FM_0___`
2. **コードブロック** (` ```lang\n...\n``` `) — `___CODE_N___`
3. **Markdown テーブル** (連続する `|...|` 行群) — `___TABLE_N___`
4. **VitePress callout ヘッダー** (`> [!XXX]` 行) — `___CALLOUT_N___`

### Python サンプル

```python
import re, json
from pathlib import Path

def generate_package(rel_path, repo_root):
    src = repo_root / "docs" / "guide" / rel_path
    text = src.read_text(encoding='utf-8')
    blocks = []

    def save(content, prefix):
        blocks.append(content)
        return f"\n\n___{prefix}_{len(blocks)-1}___\n\n"

    # Frontmatter
    fm = re.match(r'^---\n.*?\n---\n', text, re.DOTALL)
    if fm:
        text = save(fm.group(0), 'FM') + text[fm.end():]

    # Code blocks
    text = re.sub(r'```[^\n]*\n.*?\n```', lambda m: save(m.group(0), 'CODE'), text, flags=re.DOTALL)

    # Tables
    text = re.sub(r'((?:^\|.*\|\s*$\n?)+)', lambda m: save(m.group(0), 'TABLE'), text, flags=re.MULTILINE)

    # Callout headers
    text = re.sub(r'^> \[![A-Z]+\][^\n]*\n', lambda m: save(m.group(0), 'CALLOUT'), text, flags=re.MULTILINE)

    text = re.sub(r'\n{3,}', '\n\n', text)
    return {'body': text, 'blocks': blocks}
```

## Step 2: DeepL 翻訳

各言語につき 4-5 回の DeepL 呼び出し：

| # | 対象 | 内容 |
|---|------|------|
| 1 | Frontmatter `description` | `frontmatter` から抽出した description テキスト |
| 2 | Body (protected) | `___CODE_N___` 等のプレースホルダーが入った本文 |
| 3 | Table cells | 各テーブルのセルを改行区切りで連結 (テーブルごとに 1 call) |
| 4 | Code 内日本語 fragments | コードブロック内に含まれる日本語フラグメントを改行区切りで連結 |

### DeepL API 使用例

```python
import deepl
translator = deepl.Translator(os.getenv('DEEPL_AUTH_KEY'))

result = translator.translate_text(
    text=body_protected,
    source_lang='JA',
    target_lang='FR',  # FR, DE, IT, ES, NL, PT-BR
).text
```

### 注意：DeepL の制約

- **Markdown テーブルの `|` を句読点と誤認識する** → 必ず保護してセル単位で翻訳
- **プレースホルダーの大文字小文字を変える** (`___TABLE_N___` → `TABLA_N` (es), `TAVOLA_N` (it), `TABEL_N` (nl), `CODICE_N` (it), `CÓDIGO_N` (es/pt), `OPROEP_N`/`UITROEP_N` (nl), `AUFRUF_N`/`Aufruf_N` (de)) → 後処理で復元
- **末尾にピリオドを追加する** (`___CODE_1___` → `___CODE_1___.`) → 後処理で除去
- **固有名詞を翻訳してしまう** (`forkJoin` → `fourcheJoindre` 等) → 別途修正、`rxjs-glossary` Skill 参照

## Step 3: 後処理

### 3-1. プレースホルダー破損修正

```python
# 言語別の破損パターンを元に戻す
text = re.sub(r'\b(TAVOLA|TABELLA|TABLA|TABEL)_(\d+)(?:___|\b)', r'___TABLE_\2___', text)
text = re.sub(r'\b(CODICE|CÓDIGO|CODIGO)_(\d+)(?:___|\b)', r'___CODE_\2___', text)
text = re.sub(r'\b(OPROEP|UITROEP|CHIAMATA|LLAMADA|AUFRUF|CHAMADA)_(\d+)(?:___|\b)', r'___CALLOUT_\2___', text, flags=re.IGNORECASE)
```

### 3-2. 末尾ピリオド除去

```python
text = re.sub(r'(___[A-Za-z]+_\d+___)\.', r'\1', text)
```

### 3-3. プレースホルダー復元（大文字小文字無視）

```python
def restore(m):
    return blocks[int(m.group(1))]
text = re.sub(r'___(?:FM|CODE|TABLE|CALLOUT)_(\d+)___', restore, text, flags=re.IGNORECASE)
```

### 3-4. 内部リンクパスの言語化

```python
text = re.sub(r'\]\(/guide/', f'](/{lang}/guide/', text)

# DeepL が path を翻訳してしまうケースの修正
# 例: it で /guida/funzioni di creazione/ → /it/guide/creation-functions/
text = re.sub(r'/guida/funzioni di creazione/', r'/it/guide/creation-functions/', text)  # it
text = re.sub(r'/guía/creación-funciones/', r'/es/guide/creation-functions/', text)  # es
text = re.sub(r'/guía/', r'/es/guide/', text)  # es
text = re.sub(r'/guida/', r'/it/guide/', text)  # it
```

### 3-5. テーブル再構築

各言語のテーブルセル翻訳結果から Markdown テーブルを再構築：

```python
def build_table(cells, n_cols):
    n_rows = len(cells) // n_cols
    lines = ["| " + " | ".join(cells[:n_cols]) + " |",
             "|" + "|".join(["---"] * n_cols) + "|"]
    for r in range(1, n_rows):
        lines.append("| " + " | ".join(cells[r*n_cols:(r+1)*n_cols]) + " |")
    return "\n".join(lines) + "\n"
```

### 3-6. コード内日本語の置換

`rxjs-glossary` Skill の言語別マッピング (`code_jp` 辞書) を使用：

```python
for jp, translated in sorted(code_jp_mapping.items(), key=lambda x: -len(x[0])):
    code = code.replace(jp, translated)
```

**長いものから置換する**のがポイント（短いものから置換すると、長い fragment が部分的に置換されて壊れる）。

### 3-7. 語順問題の修正

Fragment 単位の翻訳では、日本語の助詞・修飾語順がフランス語/ドイツ語などで不自然になる：

| 言語 | 問題パターン | 修正 |
|------|-------------|------|
| fr | `valeur1émission` → `Émet valeur 1` | 正規表現で `r'valeur(\d+)émission'` → `r'Émet valeur \1'` |
| de | `Wert1Ausgabe` → `Ausgabe Wert 1` | `r'Wert(\d+)Ausgabe'` → `r'Ausgabe Wert \1'` |
| it | `valore1emissione` → `emette valore 1` | `r'valore(\d+)emissione'` → `r'emette valore \1'` |
| es | `valor1emisión` → `emite valor 1` | `r'valor(\d+)emisión'` → `r'emite valor \1'` |
| nl | `waarde1uitgifte` → `geeft waarde 1 uit` | `r'waarde(\d+)uitgifte'` → `r'geeft waarde \1 uit'` |
| pt | `valor1emissão` → `emite valor 1` | `r'valor(\d+)emissão'` → `r'emite valor \1'` |

### 3-8. 余分な空行整理

```python
text = re.sub(r'\n{3,}', '\n\n', text)
```

## ファイル別の DeepL 消費量目安

| ファイル | JA行数 | 1言語あたり | 6言語合計 |
|---------|-------|------------|----------|
| `forkJoin-vs-combineLatest.md` | 349 | ~12,000 文字 | ~70,000 文字 |
| `audit.md` | 294 | ~10,000 文字 | ~60,000 文字 |
| `find.md` | 440 | ~15,000 文字 | ~90,000 文字 |
| `findIndex.md` | 410 | ~14,000 文字 | ~85,000 文字 |
| `ignoreElements.md` | 468 | ~16,000 文字 | ~95,000 文字 |
| `auditTime.md` | 410 | ~14,000 文字 | ~85,000 文字 |
| `takeLast.md` | 396 | ~13,000 文字 | ~80,000 文字 |
| `elementAt.md` | 339 | ~11,000 文字 | ~65,000 文字 |
| `sampleTime.md` | 366 | ~12,000 文字 | ~75,000 文字 |

DeepL Free 版の 100 万文字制限を超えるリスクがあるため、ファイル別に消費を監視する。

## 推奨実行環境

**Claude 駆動 (MCP 経由) は非推奨**：
- 1 ファイル/セッションが限界 (コンテキスト窓圧迫)
- 6 言語分の翻訳結果保持と組み立てが困難

**ローカル Python スクリプト推奨**：
- `scripts/translate_files.py` として配置
- `DEEPL_AUTH_KEY` 環境変数で API キー設定
- `deepl-python` SDK 使用 (`pip install deepl`)
- ファイル/言語ごとにアトミックに処理
- 失敗時の再実行が容易

## サンプル：完全自動化スクリプト雛形

```python
#!/usr/bin/env python3
"""scripts/translate_files.py - DeepL で多言語に翻訳"""
import os, re, json
from pathlib import Path
import deepl

REPO = Path(__file__).parent.parent
TRANSLATOR = deepl.Translator(os.getenv('DEEPL_AUTH_KEY'))
LANGS = {'fr': 'FR', 'de': 'DE', 'it': 'IT', 'es': 'ES', 'nl': 'NL', 'pt': 'PT-BR'}

# rxjs-glossary Skill から code_jp マッピングと固有名詞リストを読み込み
GLOSSARY = json.loads(
    (REPO / '.claude' / 'skills' / 'rxjs-glossary' / 'glossary.json').read_text()
)

def translate_file(rel_path, target_lang):
    pkg = generate_package(rel_path, REPO)

    # 1. fm description
    fm_desc_jp = extract_fm_desc(pkg['blocks'][0])
    fm_desc_t = TRANSLATOR.translate_text(fm_desc_jp, source_lang='JA', target_lang=LANGS[target_lang]).text

    # 2. body
    body_t = TRANSLATOR.translate_text(pkg['body'], source_lang='JA', target_lang=LANGS[target_lang]).text

    # 3. tables
    tables_t = {}
    for idx, cells in pkg['table_cells'].items():
        joined = '\n'.join(cells)
        t = TRANSLATOR.translate_text(joined, source_lang='JA', target_lang=LANGS[target_lang]).text
        tables_t[idx] = t.split('\n')

    # 4. code_jp (glossary がカバーしない場合のみ)
    missing = [f for f in pkg['code_jp_fragments'] if f not in GLOSSARY[target_lang]['code_jp']]
    if missing:
        joined = '\n'.join(missing)
        t = TRANSLATOR.translate_text(joined, source_lang='JA', target_lang=LANGS[target_lang]).text
        for jp, tr in zip(missing, t.split('\n')):
            GLOSSARY[target_lang]['code_jp'][jp] = tr  # 学習・更新

    # 5. 組み立て
    text = assemble(pkg, body_t, fm_desc_t, tables_t, GLOSSARY[target_lang])

    # 6. 書き出し
    out = REPO / 'docs' / target_lang / 'guide' / rel_path
    out.write_text(text, encoding='utf-8')
    print(f"  {target_lang}/{rel_path} ({len(text)} chars)")

if __name__ == '__main__':
    FILES = [
        "operators/filtering/audit.md",
        "operators/filtering/auditTime.md",
        "operators/filtering/elementAt.md",
        "operators/filtering/find.md",
        "operators/filtering/findIndex.md",
        "operators/filtering/ignoreElements.md",
        "operators/filtering/sampleTime.md",
        "operators/filtering/takeLast.md",
    ]
    for f in FILES:
        for lang in LANGS:
            translate_file(f, lang)
```

## 既存実装の参照

成功事例として参照可能：

- `docs/{fr,de,it,es,nl,pt}/guide/creation-functions/combination/forkJoin-vs-combineLatest.md` — 6 言語完全翻訳済み
- `docs/fr/guide/operators/filtering/audit.md` — fr のみ完全
- `audit/issue34-coverage-report.md` — 網羅性レポート

## 関連 Skill

- `rxjs-glossary` — 固有名詞・各言語の定型訳マッピング（このワークフローと併用必須）

## 配置場所

このプロジェクト固有の Skill として、リポジトリ内の `.claude/skills/rxjs-vitepress-i18n/SKILL.md` に配置することを推奨。
GitHub にコミットすれば他デバイス・他セッションからも参照可能。
