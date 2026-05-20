#!/usr/bin/env python3
"""
scripts/fix_code_jp_schedulers_types.py
========================================

`docs/{lang}/guide/schedulers/types.md` のコードブロック内に残存する
日本語コメント (例: `// 開始`, `console.log('終了')`) を、JA 原本から抽出して
DeepL で翻訳し、各言語のコードブロック内のみで置換する。

repair_schedulers_types.py v1 が code_jp_fragments の処理を省略していたために
glossary-compliance 違反が大量に発生した問題への後処理スクリプト。

処理フロー:
    1. JA から code_jp_fragments を抽出 (コードブロック内の連続日本語)
    2. 言語ごとに DeepL で一括翻訳して翻訳マップを構築
    3. 各言語ファイルのコードブロックを走査し、日本語フラグメントを翻訳結果で置換
    4. glossary[lang].code_jp 辞書も併用 (基礎用語 52 件)
    5. フェンス整合チェック (開閉ペア数が合わない場合は警告)

使い方:
    export DEEPL_AUTH_KEY="..."

    # dry-run: 抽出された日本語フラグメント一覧を表示
    python3 scripts/fix_code_jp_schedulers_types.py --dry-run

    # 本番実行 (全 7 言語)
    python3 scripts/fix_code_jp_schedulers_types.py

    # 単一言語のみ
    python3 scripts/fix_code_jp_schedulers_types.py --lang de
"""

import argparse
import json
import os
import re
import sys
from pathlib import Path

try:
    import deepl
except ImportError:
    print("ERROR: pip install deepl が必要", file=sys.stderr)
    sys.exit(1)


REPO = Path(__file__).resolve().parent.parent
JA_FILE = REPO / 'docs' / 'guide' / 'schedulers' / 'types.md'
IDS_PATH = REPO / '.claude' / 'skills' / 'rxjs-glossary' / 'deepl_glossary_ids.json'
GLOSSARY_PATH = REPO / '.claude' / 'skills' / 'rxjs-glossary' / 'glossary.json'

LANGS = {
    'en': 'EN-US',
    'fr': 'FR',
    'de': 'DE',
    'it': 'IT',
    'es': 'ES',
    'nl': 'NL',
    'pt': 'PT-BR',
}

JP_PATTERN = re.compile(r'[぀-ゟ゠-ヿ一-鿿]+')
CODE_BLOCK_PATTERN = re.compile(r'```[^\n]*\n.*?\n```', re.DOTALL)


def extract_code_jp_fragments(text: str) -> list[str]:
    """テキストのコードブロック内に出現する日本語フラグメントを抽出 (重複除去)"""
    seen = set()
    fragments = []
    for m in CODE_BLOCK_PATTERN.finditer(text):
        block = m.group(0)
        for j in JP_PATTERN.finditer(block):
            f = j.group()
            if f and f not in seen:
                seen.add(f)
                fragments.append(f)
    return fragments


def translate_fragments(translator: deepl.Translator, fragments: list[str],
                        target_lang: str, glossary_id: str = None) -> dict[str, str]:
    """日本語フラグメントを DeepL で一括翻訳して {jp: translated} を返す"""
    if not fragments:
        return {}
    # \n 区切りで結合して 1 call に
    joined = '\n'.join(fragments)
    kwargs = {}
    if glossary_id:
        kwargs['glossary'] = glossary_id
    result = translator.translate_text(
        joined, source_lang='JA', target_lang=target_lang, **kwargs
    )
    translated_lines = result.text.split('\n')
    if len(translated_lines) != len(fragments):
        print(f"  ⚠️  翻訳行数不一致: JA={len(fragments)}, T={len(translated_lines)}")
    mapping = {}
    for jp, tr in zip(fragments, translated_lines):
        mapping[jp] = tr
    return mapping


def apply_to_code_blocks(text: str, jp_map: dict[str, str],
                         glossary_code_jp: dict[str, str]) -> tuple[str, int]:
    """コードブロック内のみで日本語を置換 (長い順に適用、重複置換防止)

    Returns: (置換後テキスト, 置換回数)
    """
    # 長い順にソート (部分マッチ優先)
    combined_map = {**glossary_code_jp, **jp_map}  # jp_map (DeepL 結果) を優先
    sorted_items = sorted(combined_map.items(), key=lambda x: -len(x[0]))

    changes = 0

    def replace_in_block(m):
        nonlocal changes
        block = m.group(0)
        for jp, tr in sorted_items:
            if jp and jp in block:
                count = block.count(jp)
                block = block.replace(jp, tr)
                changes += count
        return block

    new_text = CODE_BLOCK_PATTERN.sub(replace_in_block, text)
    return new_text, changes


def check_fence_integrity(text: str) -> list[str]:
    """コードフェンスの整合性をチェック。問題があれば説明を返す。"""
    issues = []
    fences = [(m.start(), m.group()) for m in re.finditer(r'^```[^\n]*$', text, re.MULTILINE)]
    if len(fences) % 2 != 0:
        issues.append(f"フェンス総数が奇数: {len(fences)} 個 (開閉ペアが合わない)")
    # 開きフェンス (言語マーカー付き) と閉じフェンス (マーカー無し) のパターン
    opens = 0
    for i, (pos, fence) in enumerate(fences):
        stripped = fence.strip()
        if stripped == '```':
            opens -= 1  # 閉じ
        else:
            opens += 1  # 開き (```ts など)
        # 連続する閉じ・開きの異常検出
        if i + 1 < len(fences):
            next_fence = fences[i + 1][1].strip()
            if stripped == '```' and next_fence == '```':
                line_no = text[:pos].count('\n') + 1
                issues.append(f"L{line_no}: 閉じフェンス → 閉じフェンス (空ブロック or 破損)")
    if opens != 0:
        issues.append(f"開閉バランス異常: opens-closes = {opens}")
    return issues


def process_lang(lang: str, target_lang: str, ja_fragments: list[str],
                 glossary: dict, glossary_id: str, translator: deepl.Translator,
                 dry_run: bool):
    """1 言語のファイルを処理"""
    lang_file = REPO / 'docs' / lang / 'guide' / 'schedulers' / 'types.md'
    if not lang_file.exists():
        print(f"  [{lang}] ⏭️  ファイル無し")
        return

    text = lang_file.read_text(encoding='utf-8')

    # フェンス整合チェック (修復前)
    issues_before = check_fence_integrity(text)
    if issues_before:
        print(f"  [{lang}] ⚠️  フェンス問題 (修復前):")
        for issue in issues_before[:5]:
            print(f"        {issue}")

    # 現状コードブロック内の日本語を抽出
    current_jp = extract_code_jp_fragments(text)
    print(f"  [{lang}] 📊 現在のコード内日本語: {len(current_jp)} ユニーク")
    if current_jp and len(current_jp) <= 10:
        print(f"        {current_jp}")

    if not current_jp:
        print(f"  [{lang}] ✅ コード内日本語残存なし")
        return

    if dry_run:
        print(f"  [{lang}] (dry-run)")
        return

    # 翻訳マップを構築 (現状残存フラグメント + JA 抽出フラグメントの和集合)
    # JA 由来のフラグメントを優先 (より体系的)
    all_fragments_set = set(current_jp) | set(ja_fragments)
    all_fragments = list(all_fragments_set)
    jp_map = translate_fragments(translator, all_fragments, target_lang, glossary_id)
    print(f"  [{lang}] 🔤 DeepL 翻訳: {len(all_fragments)} フラグメント")

    # glossary[lang].code_jp も併用
    glossary_code_jp = glossary.get(lang, {}).get('code_jp', {})

    # 置換実行
    new_text, changes = apply_to_code_blocks(text, jp_map, glossary_code_jp)
    print(f"  [{lang}] 🔁 置換: {changes} 箇所")

    # フェンス整合チェック (修復後)
    issues_after = check_fence_integrity(new_text)
    if issues_after:
        print(f"  [{lang}] ⚠️  フェンス問題 (修復後も残存):")
        for issue in issues_after[:5]:
            print(f"        {issue}")

    # 残存確認
    residual = extract_code_jp_fragments(new_text)
    if residual:
        print(f"  [{lang}] ⚠️  残存日本語 ({len(residual)} ユニーク): {residual[:5]}")
    else:
        print(f"  [{lang}] ✅ コード内日本語残存ゼロ")

    # 書き出し
    lang_file.write_text(new_text, encoding='utf-8')
    print(f"  [{lang}] 💾 書き出し完了")


def main():
    parser = argparse.ArgumentParser(description="schedulers/types.md コード内日本語修復")
    parser.add_argument('--lang', help='対象言語 (en/fr/de/it/es/nl/pt)')
    parser.add_argument('--dry-run', action='store_true', help='抽出のみ、翻訳しない')
    args = parser.parse_args()

    if not JA_FILE.exists():
        print(f"ERROR: JA ファイルが無い: {JA_FILE}")
        sys.exit(1)

    # JA からコード内日本語を抽出
    ja_text = JA_FILE.read_text(encoding='utf-8')
    ja_fragments = extract_code_jp_fragments(ja_text)
    print(f"JA: {len(ja_fragments)} コード内日本語フラグメント (ユニーク)")
    print(f"  サンプル(先頭10): {ja_fragments[:10]}")
    print()

    # DeepL クライアント
    translator = None
    glossary_ids = {}
    if not args.dry_run:
        api_key = os.getenv('DEEPL_AUTH_KEY')
        if not api_key:
            print("ERROR: DEEPL_AUTH_KEY が必要")
            sys.exit(1)
        translator = deepl.Translator(api_key)
        if IDS_PATH.exists():
            try:
                glossary_ids = json.loads(IDS_PATH.read_text(encoding='utf-8'))
                print(f"🔖 Glossary IDs: {list(glossary_ids.keys())}")
            except Exception as e:
                print(f"⚠️  Glossary IDs 読み込み失敗: {e}")

    # glossary.json (code_jp 辞書)
    glossary = {}
    if GLOSSARY_PATH.exists():
        try:
            glossary = json.loads(GLOSSARY_PATH.read_text(encoding='utf-8'))
        except Exception as e:
            print(f"⚠️  glossary.json 読み込み失敗: {e}")

    targets = [args.lang] if args.lang else list(LANGS.keys())
    invalid = [l for l in targets if l not in LANGS]
    if invalid:
        print(f"ERROR: 未対応の言語: {invalid}")
        sys.exit(1)

    print(f"\n=== 修復開始 ({len(targets)} 言語) ===\n")
    for lang in targets:
        target_lang = LANGS[lang]
        gid = glossary_ids.get(lang)
        process_lang(lang, target_lang, ja_fragments, glossary, gid, translator, args.dry_run)
        print()

    print("✅ All done!")
    if not args.dry_run:
        print("\n次のステップ:")
        print("  for lang in en fr de it es nl pt; do")
        print("    node scripts/extract-translation-pairs-v2.cjs $lang")
        print("  done")
        print("  ~/.xcomet-venv/bin/python scripts/evaluate_xcomet.py all --force")
        print("  node scripts/generate-evaluation-report.cjs all")
        print("  node scripts/validate-glossary-compliance.cjs all")


if __name__ == '__main__':
    main()
