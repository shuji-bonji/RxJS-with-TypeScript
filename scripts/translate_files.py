#!/usr/bin/env python3
"""
scripts/translate_files.py
==========================

RxJS-with-TypeScript の多言語ドキュメントを DeepL API で翻訳するスクリプト。

## 前提

- Python 3.10+
- DeepL API キー (環境変数 `DEEPL_AUTH_KEY`)
- pip install deepl
- Skill `rxjs-vitepress-i18n` と `rxjs-glossary` の手順に従う

## 使い方

```bash
# 環境変数設定
export DEEPL_AUTH_KEY="your-deepl-api-key"

# 依存インストール
pip install deepl

# 全 7 ファイル × 6 言語 (デフォルト)
python scripts/translate_files.py

# 特定のファイルだけ
python scripts/translate_files.py --file operators/filtering/audit.md

# 特定の言語だけ
python scripts/translate_files.py --lang fr

# ドライラン (DeepL 呼び出し無し、消費文字数のみ表示)
python scripts/translate_files.py --dry-run

# 指定 RELATIVE ファイル + 言語
python scripts/translate_files.py --file operators/filtering/elementAt.md --lang de
```

## ワークフロー（Skill `rxjs-vitepress-i18n` と同一手順）

1. JA ファイルを保護パッケージ化（Frontmatter / コード / テーブル / callout を placeholder 化）
2. DeepL API で 4-5 セクションを翻訳
3. 後処理で組み立て・固有名詞修正・語順修正
4. `docs/{lang}/guide/{rel_path}` に書き出し

参考実装: `docs/fr/guide/creation-functions/combination/forkJoin-vs-combineLatest.md`
"""

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path
from typing import Optional

try:
    import deepl
except ImportError:
    print("ERROR: deepl パッケージが必要です。`pip install deepl` を実行してください。", file=sys.stderr)
    sys.exit(1)


# ============================================================
# 設定
# ============================================================

REPO = Path(__file__).resolve().parent.parent
SKILLS_DIR = REPO / '.claude' / 'skills'
GLOSSARY_PATH = SKILLS_DIR / 'rxjs-glossary' / 'glossary.json'
DEEPL_GLOSSARY_IDS_PATH = SKILLS_DIR / 'rxjs-glossary' / 'deepl_glossary_ids.json'

# 言語コード: ディレクトリ名 → DeepL ターゲット言語コード
LANGS = {
    'en': 'EN-US',
    'fr': 'FR',
    'de': 'DE',
    'it': 'IT',
    'es': 'ES',
    'nl': 'NL',
    'pt': 'PT-BR',
}

# Issue #34 の残作業対象ファイル (CLAUDE.md と audit/issue34-coverage-report.md 参照)
DEFAULT_FILES = [
    "operators/filtering/audit.md",       # 部分実装中 (本文段落のみ fr 流用)
    "operators/filtering/auditTime.md",
    "operators/filtering/elementAt.md",
    "operators/filtering/find.md",
    "operators/filtering/findIndex.md",
    "operators/filtering/ignoreElements.md",
    "operators/filtering/sampleTime.md",
    "operators/filtering/takeLast.md",
]


# ============================================================
# Step 1: 保護パッケージ生成
# ============================================================

def generate_package(rel_path: str) -> dict:
    """JA ファイルを保護パッケージに変換"""
    src = REPO / 'docs' / 'guide' / rel_path
    if not src.exists():
        raise FileNotFoundError(f"JA ファイルが見つからない: {src}")

    text = src.read_text(encoding='utf-8')
    blocks: list[str] = []

    def save(content: str, prefix: str) -> str:
        blocks.append(content)
        return f"\n\n___{prefix}_{len(blocks)-1}___\n\n"

    # 1. Frontmatter
    fm_match = re.match(r'^---\n.*?\n---\n', text, re.DOTALL)
    if fm_match:
        text = save(fm_match.group(0), 'FM') + text[fm_match.end():]

    # 2. Code blocks
    text = re.sub(r'```[^\n]*\n.*?\n```', lambda m: save(m.group(0), 'CODE'),
                  text, flags=re.DOTALL)

    # 3. Markdown tables
    text = re.sub(r'((?:^\|.*\|\s*$\n?)+)', lambda m: save(m.group(0), 'TABLE'),
                  text, flags=re.MULTILINE)

    # 4. VitePress callout headers
    text = re.sub(r'^> \[![A-Z]+\][^\n]*\n', lambda m: save(m.group(0), 'CALLOUT'),
                  text, flags=re.MULTILINE)

    text = re.sub(r'\n{3,}', '\n\n', text)

    # frontmatter description / keywords を抽出
    fm_text = blocks[0] if blocks else ''
    fm_desc_m = re.search(r'description:\s*"([^"]*)"', fm_text) or \
                re.search(r'description:\s*(.+?)$', fm_text, re.MULTILINE)
    fm_desc = fm_desc_m.group(1).strip() if fm_desc_m else None
    fm_kw_m = re.search(r'content:\s*([^\n]+)', fm_text)
    fm_kw = fm_kw_m.group(1).strip() if fm_kw_m else None

    # テーブルセル
    table_cells: dict[int, list[str]] = {}
    for i, b in enumerate(blocks):
        if not b.startswith('|'):
            continue
        cells: list[str] = []
        for line in b.strip().split('\n'):
            if not line.startswith('|'):
                continue
            if re.match(r'^\|[\s\-:|]+\|$', line.strip()):
                continue
            parts = [p.strip() for p in line.split('|')[1:-1]]
            cells.extend(parts)
        if cells:
            table_cells[i] = cells

    # コードブロック内の日本語フラグメント
    JP_PATTERN = re.compile(r'[぀-ゟ゠-ヿ一-鿿、。〜・「」『』（）]+')
    seen = set()
    code_jp: list[str] = []
    for b in blocks:
        if not b.startswith('```'):
            continue
        for m in JP_PATTERN.finditer(b):
            f = m.group()
            if f not in seen:
                seen.add(f)
                code_jp.append(f)

    return {
        'rel_path': rel_path,
        'body': text,
        'blocks': blocks,
        'fm_desc': fm_desc,
        'fm_kw': fm_kw,
        'table_cells': table_cells,
        'code_jp_fragments': code_jp,
    }


# ============================================================
# Step 2: DeepL 翻訳
# ============================================================

class DeepLTranslator:
    """DeepL API ラッパー（消費文字数カウント、ドライラン対応）"""

    def __init__(self, dry_run: bool = False, use_glossary: bool = True):
        self.dry_run = dry_run
        self.consumed = 0
        self.calls = 0
        self.glossary_ids: dict = {}  # {lang_dir: glossary_id}
        if not dry_run:
            api_key = os.getenv('DEEPL_AUTH_KEY')
            if not api_key:
                raise RuntimeError("環境変数 DEEPL_AUTH_KEY が設定されていません")
            self.client = deepl.Translator(api_key)

        # Glossary IDs を読み込み (存在しなければ glossary 無しで動作)
        if use_glossary and DEEPL_GLOSSARY_IDS_PATH.exists():
            try:
                self.glossary_ids = json.loads(
                    DEEPL_GLOSSARY_IDS_PATH.read_text(encoding='utf-8')
                )
                if self.glossary_ids:
                    print(f"  🔖 Glossary IDs loaded: {list(self.glossary_ids.keys())}")
            except Exception as e:
                print(f"  ⚠️  Glossary IDs 読み込み失敗: {e}")

    def translate(self, text: str, target_lang: str, lang_dir: str = None) -> str:
        """JA → target_lang 翻訳。dry_run なら原文を返す。

        Args:
            text: 翻訳テキスト
            target_lang: DeepL ターゲット言語コード (FR, DE, IT, ES, NL, PT-BR)
            lang_dir: 言語ディレクトリ名 (fr, de, it, es, nl, pt) - glossary 検索用
        """
        if not text or not text.strip():
            return text
        self.calls += 1
        self.consumed += len(text)
        if self.dry_run:
            return text  # ドライラン: 原文をそのまま返す

        kwargs = {}
        if lang_dir and lang_dir in self.glossary_ids:
            kwargs['glossary'] = self.glossary_ids[lang_dir]

        result = self.client.translate_text(
            text, source_lang='JA', target_lang=target_lang, **kwargs
        )
        return result.text


def translate_package(pkg: dict, lang: str, translator: DeepLTranslator) -> dict:
    """パッケージから翻訳対象を抽出し、4-5 call で翻訳"""
    target = LANGS[lang]
    results: dict = {
        'fm_desc': None,
        'body': None,
        'tables': {},
        'code_jp': {},
    }

    # 1. Frontmatter description
    if pkg['fm_desc']:
        results['fm_desc'] = translator.translate(pkg['fm_desc'], target, lang)

    # 2. Body (protected)
    results['body'] = translator.translate(pkg['body'], target, lang)

    # 3. Tables (各テーブルごとに 1 call)
    for idx, cells in pkg['table_cells'].items():
        joined = '\n'.join(cells)
        t = translator.translate(joined, target, lang)
        results['tables'][idx] = t.split('\n')

    # 4. Code 内 JP fragments
    if pkg['code_jp_fragments']:
        joined = '\n'.join(pkg['code_jp_fragments'])
        t = translator.translate(joined, target, lang)
        translated_fragments = t.split('\n')
        if len(translated_fragments) == len(pkg['code_jp_fragments']):
            for jp, tr in zip(pkg['code_jp_fragments'], translated_fragments):
                results['code_jp'][jp] = tr
        else:
            print(f"  ⚠️  fragments の行数が一致しない: JP={len(pkg['code_jp_fragments'])}, T={len(translated_fragments)}")
            # 部分的にでもマッピング
            for jp, tr in zip(pkg['code_jp_fragments'], translated_fragments):
                results['code_jp'][jp] = tr

    return results


# ============================================================
# Step 3: 後処理・組み立て
# ============================================================

def build_table(cells: list[str], n_cols: int) -> str:
    """テーブルセルから Markdown テーブルを再構築"""
    n_rows = len(cells) // n_cols
    lines = ["| " + " | ".join(cells[:n_cols]) + " |"]
    lines.append("|" + "|".join(["---"] * n_cols) + "|")
    for r in range(1, n_rows):
        row = cells[r*n_cols:(r+1)*n_cols]
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines) + "\n"


def assemble(pkg: dict, translations: dict, lang: str, glossary: dict) -> str:
    """翻訳結果を組み立てて完成版 Markdown を生成"""
    blocks = list(pkg['blocks'])
    lang_data = glossary.get(lang, {})

    # Frontmatter
    fm_desc_t = translations.get('fm_desc') or pkg.get('fm_desc', '')
    fm_kw_t = pkg.get('fm_kw', '')  # キーワードは元のまま
    if fm_kw_t:
        # キーワードを翻訳しないが、文字数だけ翻訳することも可能。今回は原文維持。
        pass

    fm_lines = ['---']
    if fm_desc_t:
        fm_lines.append(f'description: "{fm_desc_t}"')
    if fm_kw_t:
        fm_lines.append('head:')
        fm_lines.append('  - - meta')
        fm_lines.append('    - name: keywords')
        fm_lines.append(f'      content: {fm_kw_t}')
    fm_lines.append('---')
    fm_lines.append('')
    blocks[0] = '\n'.join(fm_lines)

    # Tables を Glossary の table_headers でリファイン（ヘッダー部分のみ）
    for idx, cells in translations['tables'].items():
        # ヘッダー (1 行目) を glossary で置換
        headers = glossary.get(lang, {}).get('table_headers', {})
        # 元のヘッダーを取得
        original = pkg['table_cells'].get(int(idx), [])
        if original:
            # まずは翻訳結果を使う。元の JA ヘッダーが glossary にあれば置換
            n_cols = len(original) // (len(original) // 3) if len(original) >= 3 else 3
            # よくあるパターンは 3 列。実際は元のテーブルの形を推定する必要
            # 簡易: テーブルブロック先頭から列数を推定
            block_text = pkg['blocks'][int(idx)]
            first_line = block_text.strip().split('\n')[0]
            n_cols = first_line.count('|') - 1

            blocks[int(idx)] = build_table(cells, n_cols)

    # Code blocks の日本語を置換
    code_jp_map = {**lang_data.get('code_jp', {}), **translations['code_jp']}
    for i in range(len(blocks)):
        if not blocks[i].startswith('```'):
            continue
        code = blocks[i]
        for jp, tr in sorted(code_jp_map.items(), key=lambda x: -len(x[0])):
            code = code.replace(jp, tr)
        blocks[i] = code

    # 本文を組み立て
    text = translations['body'] or pkg['body']

    # プレースホルダー破損修正 (各言語の DeepL バリエーション)
    # 1) ___AUFRUF_13___ / ___oproep_13___ / ___TABELA_10___ 等、既に三重アンダースコアで囲まれた状態
    text = re.sub(
        r'___\s*(Aufruf|oproep|uitroep|chiamata|llamada|chamada)\s*_(\d+)\s*___',
        r'___CALLOUT_\2___', text, flags=re.IGNORECASE
    )
    text = re.sub(
        r'___\s*(TABELA|TABLA|TABEL|TAVOLA|TABELLA)\s*_(\d+)\s*___',
        r'___TABLE_\2___', text, flags=re.IGNORECASE
    )
    text = re.sub(
        r'___\s*(CODICE|CÓDIGO|CODIGO)\s*_(\d+)\s*___',
        r'___CODE_\2___', text, flags=re.IGNORECASE
    )

    # 2) 単語境界版 (古いパターンも維持)
    text = re.sub(r'\b(TAVOLA|TABELLA|TABLA|TABEL|TABELA)_(\d+)(?:___|\b)', r'___TABLE_\2___', text)
    text = re.sub(r'\b(CODICE|CÓDIGO|CODIGO)_(\d+)(?:___|\b)', r'___CODE_\2___', text)
    text = re.sub(r'\b(OPROEP|UITROEP|CHIAMATA|LLAMADA|AUFRUF|CHAMADA)_(\d+)(?:___|\b)',
                  r'___CALLOUT_\2___', text, flags=re.IGNORECASE)

    # 3) DeepL がプレースホルダーを {{NAME...}... 形式に変換するケース（Vue 補間と衝突）
    # バリエーション: {{CODE_2}}, {{CODE_2}_, {{CODE*2}*, {{CODE_2}.,
    #                {{CODE_2} (no end brace), etc.
    text = re.sub(r'\{\{([A-Za-z]+)[*_](\d+)\}[*_.}]{0,2}', r'___\1_\2___', text)
    # 開きカッコだけ {{ で囲まれていないケース
    text = re.sub(r'\{([A-Za-z]+)[*_](\d+)\}[*_.}]{0,2}', r'___\1_\2___', text)

    # 4) 末尾 ___ 欠落のケース (___CODE_5 のような)
    text = re.sub(r'___(FM|CODE|TABLE|CALLOUT)_(\d+)(?!_)', r'___\1_\2___', text)

    # 5) callout や code の前後にハイフン/空白挿入されたケース
    text = re.sub(r'_+\s*(FM|CODE|TABLE|CALLOUT)\s*_+\s*(\d+)\s*_+', r'___\1_\2___', text)

    # 6) 壊れたコードフェンス: ```0___, ```1___ など (DeepL が _CODE_ を消去)
    #    これらを blocks[N] で復元
    def fix_orphan_fence(m):
        idx = int(m.group(1))
        if 0 <= idx < len(blocks):
            return blocks[idx]
        return m.group(0)
    text = re.sub(r'```(\d+)___\.?', fix_orphan_fence, text)

    # 末尾ピリオド除去
    text = re.sub(r'(___[A-Za-z]+_\d+___)\.', r'\1', text)

    # プレースホルダー復元
    def restore(m):
        idx = int(m.group(1))
        if 0 <= idx < len(blocks):
            return blocks[idx]
        return m.group(0)
    text = re.sub(r'___(?:FM|CODE|TABLE|CALLOUT)_(\d+)___', restore, text, flags=re.IGNORECASE)

    # 内部リンクパス変換 (/guide/ → /<lang>/guide/)
    text = re.sub(r'\]\(/guide/', f'](/{lang}/guide/', text)

    # 言語別パス汚染の修正
    if lang == 'it':
        text = re.sub(r'/guida/funzioni di creazione/', r'/it/guide/creation-functions/', text)
        text = re.sub(r'/guida/operatori/', r'/it/guide/operators/', text)
        text = re.sub(r'/guida/creazione-funzioni/', r'/it/guide/creation-functions/', text)
        text = re.sub(r'/guida/', r'/it/guide/', text)
    elif lang == 'es':
        text = re.sub(r'/guía/creación-funciones/', r'/es/guide/creation-functions/', text)
        text = re.sub(r'/guía/operadores/', r'/es/guide/operators/', text)
        text = re.sub(r'/guía/', r'/es/guide/', text)
    elif lang == 'pt':
        text = re.sub(r'/guia/', r'/pt/guide/', text)
    elif lang == 'fr':
        text = re.sub(r'/guide/', r'/fr/guide/', text) if not re.search(r'/fr/guide/', text) else text

    # 余分な空行整理
    text = re.sub(r'\n{3,}', '\n\n', text)

    # ファイル先頭の余分な空行を除去 (frontmatter の前)
    text = text.lstrip('\n')

    # frontmatter 直後の余分な空行 (--- の後に複数空行) を 1 行に
    text = re.sub(r'(\n---\n)\n{2,}', r'\1\n', text, count=1)

    # 言語別の語順問題修正
    text = fix_word_order(text, lang)

    return text


def fix_word_order(text: str, lang: str) -> str:
    """Fragment 連結による語順問題を修正"""
    fixes = {
        'fr': [
            (r'valeur(\d+)émission', r'Émet valeur \1'),
            (r'valeur([A-Z])émission', r'Émet valeur \1'),
            (r'valeur(\d+)émission \(dernière\)', r'Émet valeur \1 (dernière)'),
            (r'\[valeur(\d+), valeur([A-Z])\] émet', r'Émet [valeur \1, valeur \2]'),
            (r'en attente\.{4,}', 'en attente...'),
        ],
        'de': [
            (r'Wert(\d+)Ausgabe', r'Ausgabe Wert \1'),
            (r'Wert([A-Z])Ausgabe', r'Ausgabe Wert \1'),
            (r'\[Wert(\d+), Wert([A-Z])\] ausgeben', r'Ausgabe [Wert \1, Wert \2]'),
            (r'Warten\.{4,}', 'Warten...'),
        ],
        'it': [
            (r'valore(\d+)emissione', r'emette valore \1'),
            (r'valore([A-Z])emissione', r'emette valore \1'),
            (r'\[valore(\d+), valore([A-Z])\] emette', r'emette [valore \1, valore \2]'),
            (r'in attesa\.{4,}', 'in attesa...'),
        ],
        'es': [
            (r'valor(\d+)emisión', r'emite valor \1'),
            (r'valor([A-Z])emisión', r'emite valor \1'),
            (r'\[valor(\d+), valor([A-Z])\] emite', r'emite [valor \1, valor \2]'),
            (r'esperando\.{4,}', 'esperando...'),
        ],
        'nl': [
            (r'waarde(\d+)uitgifte', r'geeft waarde \1 uit'),
            (r'waarde([A-Z])uitgifte', r'geeft waarde \1 uit'),
            (r'\[waarde(\d+), waarde([A-Z])\] geeft uit', r'geeft [waarde \1, waarde \2] uit'),
            (r'wachten\.{4,}', 'wachten...'),
        ],
        'pt': [
            (r'valor(\d+)emissão', r'emite valor \1'),
            (r'valor([A-Z])emissão', r'emite valor \1'),
            (r'\[valor(\d+), valor([A-Z])\] emite', r'emite [valor \1, valor \2]'),
            (r'aguardando\.{4,}', 'aguardando...'),
        ],
    }
    for pat, repl in fixes.get(lang, []):
        text = re.sub(pat, repl, text)
    return text


# ============================================================
# メイン処理
# ============================================================

def process_file(rel_path: str, lang: str, translator: DeepLTranslator,
                 glossary: dict, verbose: bool = True) -> Optional[Path]:
    """1 ファイル × 1 言語を処理"""
    try:
        pkg = generate_package(rel_path)
    except FileNotFoundError as e:
        print(f"  ❌ {e}")
        return None

    if verbose:
        print(f"  📦 {rel_path} [{lang}]: body={len(pkg['body'])} chars, "
              f"tables={len(pkg['table_cells'])}, code_jp={len(pkg['code_jp_fragments'])}")

    # 翻訳
    translations = translate_package(pkg, lang, translator)

    # 組み立て
    text = assemble(pkg, translations, lang, glossary)

    # 残存日本語チェック
    remaining = sorted(set(re.findall(r'[぀-ゟ゠-ヿ一-鿿]+', text)))

    # 書き出し
    out = REPO / 'docs' / lang / 'guide' / rel_path
    out.parent.mkdir(parents=True, exist_ok=True)
    if not translator.dry_run:
        out.write_text(text, encoding='utf-8')

    if verbose:
        status = "✅" if not remaining else f"⚠️  残存JP {len(remaining)}"
        print(f"     → {out.relative_to(REPO)}: {len(text)} chars, {status}")
        if remaining and len(remaining) <= 5:
            print(f"        残: {remaining}")

    return out


def setup_lang_glossary(translator_obj, lang: str) -> str:
    """指定言語の glossary を Free 版 1-slot 制限に合わせて作成する

    既存の全 glossary を削除してから、指定言語のみ作成する。
    Returns: 作成した glossary_id (失敗時は None)
    """
    api_key = os.getenv('DEEPL_AUTH_KEY')
    if not api_key:
        return None
    client = deepl.Translator(api_key)

    # 既存を全削除 (Free 版の 1-slot 制限への対応)
    try:
        for g in client.list_glossaries():
            client.delete_glossary(g)
    except deepl.DeepLException:
        pass

    # glossary.json を読み込み
    if not GLOSSARY_PATH.exists():
        return None
    data = json.loads(GLOSSARY_PATH.read_text(encoding='utf-8'))
    no_translate = data.get('no_translate', [])
    if not no_translate:
        return None

    # identity マッピング
    entries = {term: term for term in no_translate}
    target_map = {'fr': 'FR', 'de': 'DE', 'it': 'IT', 'es': 'ES', 'nl': 'NL', 'pt': 'PT-BR'}
    target_lang = target_map.get(lang)
    if not target_lang:
        return None

    try:
        g = client.create_glossary(
            name=f"rxjs-ja-{lang}",
            source_lang='JA',
            target_lang=target_lang,
            entries=entries,
        )
        return g.glossary_id
    except deepl.DeepLException as e:
        print(f"  ⚠️  Glossary 作成失敗 ({lang}): {e}")
        return None


def cleanup_glossaries():
    """全 rxjs-* 系 glossary を削除 (お掃除)"""
    api_key = os.getenv('DEEPL_AUTH_KEY')
    if not api_key:
        return
    try:
        client = deepl.Translator(api_key)
        for g in client.list_glossaries():
            if g.name.startswith('rxjs-'):
                client.delete_glossary(g)
    except deepl.DeepLException:
        pass


def main():
    parser = argparse.ArgumentParser(description="DeepL で多言語翻訳")
    parser.add_argument('--file', '-f', help="特定のファイル (relative path, e.g. operators/filtering/audit.md)")
    parser.add_argument('--lang', '-l', help="特定の言語 (fr, de, it, es, nl, pt)")
    parser.add_argument('--dry-run', '-n', action='store_true', help="ドライラン (DeepL 呼び出し無し)")
    parser.add_argument('--files', help="複数ファイルをカンマ区切りで指定")
    parser.add_argument('--auto-glossary', action='store_true',
                        help="Free 版の 1-slot 制限対応: 言語ごとに glossary を作成→翻訳→削除")
    parser.add_argument('--targets-file',
                        help="JSON ファイルから (file × language) ペアを読み込み (e.g. scripts/no_translate_targets.json)")
    parser.add_argument('--priority', type=int, choices=[1, 2, 3],
                        help="targets-file 使用時、優先度を指定 (1=critical, 2=major, 3=minor)")
    args = parser.parse_args()

    # ターゲット決定: targets-file モードか通常モードか
    targets_pairs = None  # (file, lang) ペアのリスト
    if args.targets_file:
        with open(args.targets_file) as f:
            targets_data = json.load(f)
        # 優先度フィルタ
        if args.priority:
            keys = [f'priority_{args.priority}_{["critical","major","minor"][args.priority-1]}']
        else:
            keys = list(targets_data.keys())
        pairs = []
        for k in keys:
            for item in targets_data.get(k, []):
                pairs.append((item['file'], item['lang']))
        # 重複除去
        seen = set()
        targets_pairs = []
        for p in pairs:
            if p not in seen:
                seen.add(p)
                targets_pairs.append(p)
        print(f"  Targets file: {args.targets_file}")
        print(f"  Priority filter: {args.priority if args.priority else 'all'}")
        print(f"  Pairs: {len(targets_pairs)}")

    target_files = [args.file] if args.file else (args.files.split(',') if args.files else DEFAULT_FILES)
    target_langs = [args.lang] if args.lang else list(LANGS.keys())

    # 用語集読み込み
    if not GLOSSARY_PATH.exists():
        print(f"⚠️  Glossary が見つからない: {GLOSSARY_PATH}")
        print(f"   .claude/skills/rxjs-glossary/glossary.json を配置してください")
        glossary = {}
    else:
        glossary = json.loads(GLOSSARY_PATH.read_text(encoding='utf-8'))

    # 翻訳器初期化
    translator = DeepLTranslator(dry_run=args.dry_run)

    # 実行
    start = time.time()
    print(f"\n=== DeepL 翻訳開始 ===")
    print(f"  Files: {len(target_files)}")
    print(f"  Langs: {target_langs}")
    print(f"  Dry-run: {args.dry_run}")
    print(f"  Glossary entries: {sum(len(glossary.get(l, {}).get('code_jp', {})) for l in target_langs)} (合計)")
    print()

    # 言語ループ → ファイルループ の順に変更 (auto-glossary 対応)
    # 言語ごとに glossary を切り替えるため、言語を外ループに
    if targets_pairs:
        # targets-file モード: ペアごとに処理
        # 効率のため言語ごとにグループ化
        from collections import defaultdict
        pairs_by_lang = defaultdict(list)
        for file, lang in targets_pairs:
            pairs_by_lang[lang].append(file)

        for lang in sorted(pairs_by_lang.keys()):
            if args.auto_glossary and not args.dry_run:
                print(f"🔖 Glossary 作成中: {lang}...")
                glossary_id = setup_lang_glossary(translator, lang)
                if glossary_id:
                    translator.glossary_ids = {lang: glossary_id}
                    print(f"  ✅ Glossary ID: {glossary_id}")
                else:
                    translator.glossary_ids = {}
            for rel in pairs_by_lang[lang]:
                print(f"📄 {rel} [{lang}]")
                process_file(rel, lang, translator, glossary)
            print()
    else:
        # 通常モード
        for lang in target_langs:
            if args.auto_glossary and not args.dry_run:
                print(f"🔖 Glossary 作成中: {lang}...")
                glossary_id = setup_lang_glossary(translator, lang)
                if glossary_id:
                    translator.glossary_ids = {lang: glossary_id}
                    print(f"  ✅ Glossary ID: {glossary_id}")
                else:
                    translator.glossary_ids = {}
                    print(f"  ⚠️  Glossary 作成失敗、glossary 無しで続行")

            for rel in target_files:
                print(f"📄 {rel} [{lang}]")
                process_file(rel, lang, translator, glossary)
            print()

    # auto-glossary 後始末
    if args.auto_glossary and not args.dry_run:
        print("🧹 Glossary 削除中...")
        cleanup_glossaries()
        print("  ✅ 完了")

    # サマリ
    elapsed = time.time() - start
    print("=" * 50)
    print(f"完了 ({elapsed:.1f}s)")
    print(f"  DeepL API calls: {translator.calls}")
    print(f"  消費文字数(概算): {translator.consumed:,}")
    if args.dry_run:
        print(f"  ⚠️  ドライランのため実ファイルへの書き込みは行われていません")


if __name__ == '__main__':
    main()
