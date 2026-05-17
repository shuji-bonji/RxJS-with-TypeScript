#!/usr/bin/env python3
"""
scripts/fix_broken_translations.py
===================================

translate_files.py 実行後の既存ファイルを修正する緊急パッチ。

修正内容:
1. callout ヘッダーの「本番コードでの注意」を各言語に置換
2. 壊れたプレースホルダー (___Aufruf_N___, ___TABELA_N___ 等) を ___CALLOUT_N___ / ___TABLE_N___ に正規化
3. ファイル内に壊れたプレースホルダーが残っている場合、その場で JA ブロックの内容に復元

注意:
- スクリプト version 1 で実装された「JA ブロック直接コピー」は de/find.md で日本語混入の原因になったため廃止
- 壊れたプレースホルダーが残っているファイルは translate_files.py で再生成することを推奨

使い方:
    python scripts/fix_broken_translations.py
"""

import re
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
LANGS = ['fr', 'de', 'it', 'es', 'nl', 'pt']

# Phase 3 で追加された production warning callout ヘッダー (全言語)
CALLOUT_PROD_WARNING = {
    'fr': 'Attention en code de production',
    'de': 'Hinweis für Produktionscode',
    'it': 'Attenzione in codice di produzione',
    'es': 'Atención en código de producción',
    'nl': 'Let op in productiecode',
    'pt': 'Atenção em código de produção',
}

# Phase 1 で追加された retryWhen 非推奨 callout ヘッダー
CALLOUT_RETRYWHEN = {
    'fr': 'retryWhen est déprécié',
    'de': 'retryWhen ist deprecated',
    'it': 'retryWhen è deprecato',
    'es': 'retryWhen está obsoleto',
    'nl': 'retryWhen is deprecated',
    'pt': 'retryWhen está deprecated',
}

# JA callout ヘッダーフレーズ → 各言語マッピング
COMMON_CALLOUT_HEADERS = {
    '本番コードでの注意': CALLOUT_PROD_WARNING,
    'retryWhen は非推奨': CALLOUT_RETRYWHEN,
    'retryWhenは非推奨': CALLOUT_RETRYWHEN,
}


def normalize_placeholders(text: str) -> tuple[str, int]:
    """壊れたプレースホルダーを正規化 ___CALLOUT_N___ / ___TABLE_N___ などに復元

    DeepL が CALLOUT/TABLE/CODE を各言語に翻訳した形式に対応。
    """
    changes = 0
    original = text

    # ___AUFRUF_13___ / ___oproep_13___ / ___UITROEP_13___ など
    text = re.sub(
        r'___\s*(Aufruf|oproep|uitroep|chiamata|llamada|chamada)\s*_(\d+)\s*___',
        r'___CALLOUT_\2___', text, flags=re.IGNORECASE
    )
    # ___TABELA_10___ / ___TABLA_N___ / ___TAVOLA_N___ など
    text = re.sub(
        r'___\s*(TABELA|TABLA|TABEL|TAVOLA|TABELLA)\s*_(\d+)\s*___',
        r'___TABLE_\2___', text, flags=re.IGNORECASE
    )
    # ___CODICE_N___ / ___CÓDIGO_N___ など
    text = re.sub(
        r'___\s*(CODICE|CÓDIGO|CODIGO)\s*_(\d+)\s*___',
        r'___CODE_\2___', text, flags=re.IGNORECASE
    )
    # 末尾 ___ 欠落 (___CODE_5)
    text = re.sub(r'___(FM|CODE|TABLE|CALLOUT)_(\d+)(?!_)', r'___\1_\2___', text)
    # {{NAME...N...}... のバリエーション (Vue 補間と衝突するケース)
    # {{CODE_2}}, {{CODE_2}_, {{CODE*2}*, {{CODE_2}., {{CODE_2}, など
    text = re.sub(r'\{\{([A-Za-z]+)[*_](\d+)\}[*_.}]{0,2}', r'___\1_\2___', text)
    text = re.sub(r'\{([A-Za-z]+)[*_](\d+)\}[*_.}]{0,2}', r'___\1_\2___', text)

    if text != original:
        changes = 1
    return text, changes


def fix_callout_headers(text: str, lang: str) -> tuple[str, int]:
    """callout ヘッダー内の日本語フレーズを各言語に置換"""
    changes = 0
    for jp_header, lang_map in COMMON_CALLOUT_HEADERS.items():
        if lang in lang_map and jp_header in text:
            text = text.replace(jp_header, lang_map[lang])
            changes += 1
    return text, changes


def fix_leading_whitespace(text: str) -> tuple[str, int]:
    """ファイル冒頭の余分な空行を除去"""
    original = text
    text = text.lstrip('\n')
    # frontmatter 直後の余分な空行 (--- の後に複数空行) を 1 行に
    text = re.sub(r'(\n---\n)\n{2,}', r'\1\n', text, count=1)
    return text, (1 if text != original else 0)


def process_file(lang: str, rel_path: str) -> tuple[bool, list[str]]:
    """1 ファイルを修正"""
    file = REPO / 'docs' / lang / 'guide' / rel_path
    if not file.exists():
        return False, []

    try:
        text = file.read_text(encoding='utf-8')
    except UnicodeDecodeError:
        return False, ['encoding error']

    original = text
    actions = []

    # 1. プレースホルダー正規化
    text, n1 = normalize_placeholders(text)
    if n1:
        actions.append('プレースホルダー正規化')

    # 2. callout ヘッダー翻訳
    text, n2 = fix_callout_headers(text, lang)
    if n2:
        actions.append(f'callout ヘッダー翻訳 ({n2}箇所)')

    # 3. 先頭の余分な空行を除去
    text, n3 = fix_leading_whitespace(text)
    if n3:
        actions.append('先頭空行除去')

    if text != original:
        file.write_text(text, encoding='utf-8')
        return True, actions
    return False, []


def main():
    TARGET_FILES = [
        "operators/filtering/audit.md",
        "operators/filtering/auditTime.md",
        "operators/filtering/elementAt.md",
        "operators/filtering/find.md",
        "operators/filtering/findIndex.md",
        "operators/filtering/ignoreElements.md",
        "operators/filtering/sampleTime.md",
        "operators/filtering/takeLast.md",
    ]

    print("=== 修正開始 ===")
    fixed_count = 0
    for lang in LANGS:
        for rel in TARGET_FILES:
            fixed, actions = process_file(lang, rel)
            if fixed:
                print(f"  ✅ {lang}/{rel}: {', '.join(actions)}")
                fixed_count += 1
    print(f"\n  Total: {fixed_count} ファイル修正")

    # 残存問題チェック
    print("\n=== 残存プレースホルダーチェック ===")
    issues = 0
    for lang in LANGS:
        for rel in TARGET_FILES:
            file = REPO / 'docs' / lang / 'guide' / rel
            if not file.exists():
                continue
            text = file.read_text(encoding='utf-8')
            # 残った ___NAME_N___ パターン
            broken = re.findall(r'___[A-Za-z]+_\d+___', text)
            if broken:
                print(f"  ⚠️  {lang}/{rel}: 残存プレースホルダー: {set(broken)}")
                issues += 1
            # 残った Vue 補間誤認
            vue_break = re.findall(r'\{\{[^}\n]{0,30}', text)
            real_vue = [v for v in vue_break if '}}' not in v]
            if real_vue:
                print(f"  ⚠️  {lang}/{rel}: 開いた {{...}} 検出: {real_vue[:3]}")
                issues += 1

    print("\n=== 残存日本語チェック ===")
    for lang in LANGS:
        for rel in TARGET_FILES:
            file = REPO / 'docs' / lang / 'guide' / rel
            if not file.exists():
                continue
            text = file.read_text(encoding='utf-8')
            remaining = sorted(set(re.findall(r'[぀-ゟ゠-ヿ一-鿿]+', text)))
            if remaining:
                print(f"  ⚠️  {lang}/{rel}: 残存 JP {len(remaining)}: {remaining[:5]}")

    if issues == 0:
        print("  ✅ プレースホルダー問題なし")


if __name__ == '__main__':
    main()
