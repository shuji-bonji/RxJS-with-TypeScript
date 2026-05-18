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


def apply_code_jp_mappings(text: str, lang: str, glossary: dict) -> tuple[str, int]:
    """既存翻訳済みファイルのコードブロック内 JA 残存を glossary[lang].code_jp で置換

    DeepL が翻訳しきれなかった JA フラグメントを後処理で置換する。
    Glossary に新規追加した訳語を既存ファイルにも適用するためのステップ。

    Returns:
        (修正済みテキスト, 置換実行回数)
    """
    code_jp_map = glossary.get(lang, {}).get('code_jp', {})
    if not code_jp_map:
        return text, 0

    # コードブロックを抽出し、各ブロック内でのみ置換
    changes = 0

    def replace_in_block(m):
        nonlocal changes
        block = m.group(0)
        # 長い順にソートして適用 (部分マッチを優先)
        for jp, tr in sorted(code_jp_map.items(), key=lambda x: -len(x[0])):
            if jp and jp in block:
                count = block.count(jp)
                block = block.replace(jp, tr)
                changes += count
        return block

    # ``` fenced code block を 1 単位として置換
    new_text = re.sub(
        r'```[^\n]*\n.*?\n```',
        replace_in_block,
        text,
        flags=re.DOTALL
    )

    return new_text, changes


def fix_duplicate_frontmatter(text: str) -> tuple[str, int]:
    """中盤に残存した JA frontmatter ブロックを除去

    Issue #34 翻訳時のプレースホルダー復元バグで、本来は冒頭にだけあるはずの
    frontmatter (---\\ndescription: ...\\n---) が本文中盤に重複出現するケース。
    JA 原本の description が翻訳されずに残っているため除去する。

    冒頭の正規 frontmatter (最初の --- ... --- ペア) は保護する。

    Returns:
        (修正済みテキスト, 除去ブロック数)
    """
    # まず冒頭の frontmatter を取り分ける
    fm_match = re.match(r'^(---\n.*?\n---\n)', text, re.DOTALL)
    if not fm_match:
        return text, 0

    head = fm_match.group(1)
    body = text[fm_match.end():]

    # body 内の重複 frontmatter ブロックを検索・除去
    # パターン: \n---\ndescription:...\n---\n (改行・空行を含む可能性)
    pattern = re.compile(
        r'\n+---\n+description:.*?\n+---\n+',
        re.DOTALL
    )

    count = 0

    def repl(m):
        nonlocal count
        count += 1
        return '\n\n'  # ブロックを 1 個の空行に置換

    new_body = pattern.sub(repl, body)
    if count == 0:
        return text, 0

    return head + new_body, count


def fix_orphaned_code_fences(text: str, rel_path: str, lang: str,
                              glossary: dict) -> tuple[str, int]:
    """壊れたコードフェンス ```0___ などを JA の対応ブロックで復元

    DeepL が ___CODE_N___ の _CODE_ 部分を消去して、コードブロックフェンスの
    言語指定が ```0___ になってしまうケースに対応。

    Args:
        text: 現在の翻訳ファイル内容
        rel_path: ファイルの相対パス
        lang: 言語コード (fr, de, ...)
        glossary: 用語集

    Returns:
        (修正済みテキスト, 修正箇所数)
    """
    ja_file = REPO / 'docs' / 'guide' / rel_path
    if not ja_file.exists():
        return text, 0
    ja_text = ja_file.read_text(encoding='utf-8')

    # JA の保護パッケージを再構築して blocks を取得
    blocks = []

    def save(content, prefix):
        blocks.append(content)
        return f"\n\n___{prefix}_{len(blocks)-1}___\n\n"

    work_text = ja_text
    # Frontmatter
    fm_match = re.match(r'^---\n.*?\n---\n', work_text, re.DOTALL)
    if fm_match:
        work_text = save(fm_match.group(0), 'FM') + work_text[fm_match.end():]
    # Code blocks
    work_text = re.sub(r'```[^\n]*\n.*?\n```',
                       lambda m: save(m.group(0), 'CODE'),
                       work_text, flags=re.DOTALL)
    # Tables
    work_text = re.sub(r'((?:^\|.*\|\s*$\n?)+)',
                       lambda m: save(m.group(0), 'TABLE'),
                       work_text, flags=re.MULTILINE)
    # Callouts
    work_text = re.sub(r'^> \[![A-Z]+\][^\n]*\n',
                       lambda m: save(m.group(0), 'CALLOUT'),
                       work_text, flags=re.MULTILINE)

    # code_jp マッピングを適用してコードブロックを翻訳
    code_jp_map = glossary.get(lang, {}).get('code_jp', {})
    for i in range(len(blocks)):
        if not blocks[i].startswith('```'):
            continue
        code = blocks[i]
        for jp, tr in sorted(code_jp_map.items(), key=lambda x: -len(x[0])):
            code = code.replace(jp, tr)
        blocks[i] = code

    # ```N___ パターンを blocks[N] で置換
    changes = 0

    def repl(m):
        nonlocal changes
        idx = int(m.group(1))
        if 0 <= idx < len(blocks):
            changes += 1
            return blocks[idx]
        return m.group(0)

    # ```0___ / ```0___. / ```0___\n などすべて捕捉
    text = re.sub(r'```(\d+)___\.?', repl, text)

    return text, changes


def process_file(lang: str, rel_path: str, glossary: dict = None) -> tuple[bool, list[str]]:
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

    # 4. 壊れたコードフェンス ```0___ などを JA ブロックで復元
    if glossary is not None:
        text, n4 = fix_orphaned_code_fences(text, rel_path, lang, glossary)
        if n4:
            actions.append(f'コードフェンス復元 ({n4}箇所)')

    # 5. 中盤に残存した JA frontmatter ブロックを除去
    text, n5 = fix_duplicate_frontmatter(text)
    if n5:
        actions.append(f'重複frontmatter除去 ({n5}箇所)')

    # 6. コードブロック内 JA 残存を glossary code_jp で置換
    if glossary is not None:
        text, n6 = apply_code_jp_mappings(text, lang, glossary)
        if n6:
            actions.append(f'code_jp 置換 ({n6}箇所)')

    if text != original:
        file.write_text(text, encoding='utf-8')
        return True, actions
    return False, []


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='翻訳済みファイルの後処理: プレースホルダー復元・重複 frontmatter 除去・code_jp 適用'
    )
    parser.add_argument('--all-files', action='store_true',
                       help='TARGET_FILES だけでなく全ファイルに適用 (code_jp 残存修正用)')
    parser.add_argument('--code-jp-only', action='store_true',
                       help='code_jp 置換のみ実行 (他のステップはスキップ)')
    args = parser.parse_args()

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

    # Glossary 読み込み (壊れたコードフェンス復元時の code_jp 適用に使用)
    glossary_path = REPO / '.claude' / 'skills' / 'rxjs-glossary' / 'glossary.json'
    glossary = {}
    if glossary_path.exists():
        try:
            import json
            glossary = json.loads(glossary_path.read_text(encoding='utf-8'))
        except Exception as e:
            print(f"  ⚠️  Glossary 読み込み失敗: {e}")

    print("=== 修正開始 ===")
    fixed_count = 0

    if args.all_files:
        # 全ファイル走査モード: code_jp 置換と重複 frontmatter 除去のみを高速適用
        print("  (--all-files モード: 全ファイル走査)")
        for lang in LANGS:
            lang_dir = REPO / 'docs' / lang / 'guide'
            if not lang_dir.exists():
                continue
            for md in lang_dir.rglob('*.md'):
                rel = str(md.relative_to(lang_dir))
                try:
                    text = md.read_text(encoding='utf-8')
                except UnicodeDecodeError:
                    continue
                original = text
                actions = []
                if not args.code_jp_only:
                    text, n5 = fix_duplicate_frontmatter(text)
                    if n5:
                        actions.append(f'重複frontmatter除去 ({n5}箇所)')
                text, n6 = apply_code_jp_mappings(text, lang, glossary)
                if n6:
                    actions.append(f'code_jp 置換 ({n6}箇所)')
                if text != original:
                    md.write_text(text, encoding='utf-8')
                    print(f"  ✅ {lang}/{rel}: {', '.join(actions)}")
                    fixed_count += 1
    else:
        for lang in LANGS:
            for rel in TARGET_FILES:
                fixed, actions = process_file(lang, rel, glossary)
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
