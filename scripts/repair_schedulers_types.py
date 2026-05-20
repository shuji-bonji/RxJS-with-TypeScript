#!/usr/bin/env python3
"""
scripts/repair_schedulers_types.py
==================================

`docs/guide/schedulers/types.md` の翻訳カバレッジ不足 (全 7 言語で末尾が欠落)
を ad-hoc に補完する。

translate_files.py の長文ファイル末尾欠落バグ (___CODE_N___ プレースホルダー消失)
を回避するため、見出し単位で個別に DeepL へ送る方式を採用。

検出ロジック:
    JA のセクション (見出し N → 見出し N+1 の範囲) にコードフェンスがあるのに、
    同じインデックスの言語版セクションにコードフェンスが無ければ「欠落」とみなす。

保護方式:
    DeepL が ___CODE_N___ を壊すので、代わりに ~~~C0~~~ / ~~~O0~~~ を使う。
    アルファベット 1 文字 + 数字の組み合わせなので、DeepL が改変しにくい。

使い方:
    export DEEPL_AUTH_KEY="..."

    # dry-run: 何が欠落しているか表示
    python3 scripts/repair_schedulers_types.py --dry-run

    # 本番実行
    python3 scripts/repair_schedulers_types.py

    # 単一言語のみ
    python3 scripts/repair_schedulers_types.py --lang de
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

LANGS = {
    'en': 'EN-US',
    'fr': 'FR',
    'de': 'DE',
    'it': 'IT',
    'es': 'ES',
    'nl': 'NL',
    'pt': 'PT-BR',
}


def split_into_sections(text: str) -> list[tuple[str, str]]:
    """frontmatter を除いた本文を、見出し (# / ## / ### / ####) 単位で分割。

    Returns: [(heading_line_or_None, body), ...]
        最初のセクションは見出し無しのプロローグ (heading=None)
    """
    sections = []
    current_heading = None
    current_body_lines = []
    for line in text.split('\n'):
        if re.match(r'^#{1,4}\s', line):
            sections.append((current_heading, '\n'.join(current_body_lines)))
            current_heading = line
            current_body_lines = []
        else:
            current_body_lines.append(line)
    sections.append((current_heading, '\n'.join(current_body_lines)))
    return sections


def strip_frontmatter(text: str) -> tuple[str, str]:
    """frontmatter (---...---) を分離"""
    m = re.match(r'^---\n.*?\n---\n', text, re.DOTALL)
    if m:
        return m.group(0), text[m.end():]
    return '', text


def detect_missing_sections(ja_sections, lang_sections):
    """JA でコードフェンスを含み、言語版で含まないセクションのインデックスを返す"""
    missing = []
    n = min(len(ja_sections), len(lang_sections))
    for i in range(n):
        ja_body = ja_sections[i][1]
        lang_body = lang_sections[i][1]
        ja_fences = ja_body.count('```')
        lang_fences = lang_body.count('```')
        if ja_fences >= 2 and lang_fences < ja_fences:
            missing.append(i)
    if len(ja_sections) != len(lang_sections):
        print(f"  ⚠️  セクション数不一致: JA={len(ja_sections)}, lang={len(lang_sections)}")
    return missing


def protect(text: str) -> tuple[str, list[str]]:
    """コードブロック・callout を ~~~C0~~~ / ~~~O0~~~ 形式で保護"""
    blocks: list[str] = []

    def save(content: str, prefix: str) -> str:
        blocks.append(content)
        return f"~~~{prefix}{len(blocks)-1}~~~"

    text = re.sub(r'```[^\n]*\n.*?\n```',
                  lambda m: save(m.group(0), 'C'),
                  text, flags=re.DOTALL)
    text = re.sub(r'^> \[![A-Z]+\][^\n]*\n',
                  lambda m: save(m.group(0), 'O'),
                  text, flags=re.MULTILINE)
    return text, blocks


def restore(text: str, blocks: list[str]) -> str:
    """~~~C0~~~ / ~~~O0~~~ をブロック内容に戻す"""
    def get(m):
        idx = int(m.group(2))
        if 0 <= idx < len(blocks):
            return blocks[idx]
        return m.group(0)

    # 厳密形式
    text = re.sub(r'~~~([CO])(\d+)~~~', get, text)
    # DeepL バリエーション補正
    text = re.sub(r'~+\s*([CO])\s*(\d+)\s*~+', get, text)
    return text


def translate_section(translator: deepl.Translator, text: str, target_lang: str,
                      glossary_id: str = None) -> str:
    """1 セクションを保護→DeepL→復元の流れで翻訳"""
    protected, blocks = protect(text)
    kwargs = {}
    if glossary_id:
        kwargs['glossary'] = glossary_id
    result = translator.translate_text(
        protected, source_lang='JA', target_lang=target_lang, **kwargs
    )
    return restore(result.text, blocks)


def reassemble(sections: list[tuple[str, str]]) -> str:
    """セクションリストを Markdown 本文に戻す"""
    out_lines = []
    for heading, body in sections:
        if heading is not None:
            out_lines.append(heading)
        out_lines.append(body)
    return '\n'.join(out_lines)


def repair_lang(lang: str, target_lang: str, ja_sections, glossary_id: str,
                translator: deepl.Translator, dry_run: bool):
    """1 言語のファイルを修復"""
    lang_file = REPO / 'docs' / lang / 'guide' / 'schedulers' / 'types.md'
    if not lang_file.exists():
        print(f"  [{lang}] ⏭️  ファイル無し: {lang_file}")
        return

    lang_text = lang_file.read_text(encoding='utf-8')
    lang_fm, lang_body = strip_frontmatter(lang_text)
    lang_sections = split_into_sections(lang_body)

    missing = detect_missing_sections(ja_sections, lang_sections)
    if not missing:
        print(f"  [{lang}] ✅ 欠落なし")
        return

    print(f"  [{lang}] 📝 欠落 {len(missing)} セクション")
    for i in missing:
        ja_h = ja_sections[i][0] or '(prologue)'
        ja_lines = ja_sections[i][1].count('\n') + 1
        lang_lines = lang_sections[i][1].count('\n') + 1
        print(f"        [{i}] {ja_h.strip()[:50]} (JA {ja_lines}行 → lang {lang_lines}行)")

    if dry_run:
        print(f"  [{lang}] (dry-run, 翻訳しない)")
        return

    new_sections = list(lang_sections)
    consumed = 0
    for i in missing:
        ja_heading, ja_body = ja_sections[i]
        lang_heading, _ = lang_sections[i]
        # 見出しは言語版のものを使用 (既に翻訳済みのため再翻訳しない)
        translated_body = translate_section(translator, ja_body, target_lang, glossary_id)
        new_sections[i] = (lang_heading, translated_body)
        consumed += len(ja_body)
        print(f"        ✅ section [{i}] 翻訳完了 ({len(ja_body)} chars)")

    # 再構築して書き出し
    new_text = lang_fm + reassemble(new_sections)
    lang_file.write_text(new_text, encoding='utf-8')
    new_lines = new_text.count('\n') + 1
    print(f"  [{lang}] 💾 書き出し完了 ({new_lines} 行, 消費 {consumed:,} chars)")


def main():
    parser = argparse.ArgumentParser(description="schedulers/types.md 欠落セクション補完")
    parser.add_argument('--lang', help='対象言語 (en/fr/de/it/es/nl/pt)。未指定なら全 7 言語')
    parser.add_argument('--dry-run', action='store_true', help='欠落表示のみ、翻訳しない')
    args = parser.parse_args()

    if not JA_FILE.exists():
        print(f"ERROR: JA ファイルが見つからない: {JA_FILE}")
        sys.exit(1)

    # JA セクション分割
    ja_text = JA_FILE.read_text(encoding='utf-8')
    _, ja_body = strip_frontmatter(ja_text)
    ja_sections = split_into_sections(ja_body)
    print(f"JA: {len(ja_sections)} セクション ({ja_text.count(chr(10))+1} 行)")
    print()

    # DeepL クライアント
    translator = None
    if not args.dry_run:
        api_key = os.getenv('DEEPL_AUTH_KEY')
        if not api_key:
            print("ERROR: DEEPL_AUTH_KEY が必要")
            sys.exit(1)
        translator = deepl.Translator(api_key)

    # Glossary IDs
    glossary_ids = {}
    if IDS_PATH.exists():
        try:
            glossary_ids = json.loads(IDS_PATH.read_text(encoding='utf-8'))
            print(f"🔖 Glossary IDs: {list(glossary_ids.keys())}")
        except Exception as e:
            print(f"⚠️  Glossary IDs 読み込み失敗: {e}")

    # 対象言語
    targets = [args.lang] if args.lang else list(LANGS.keys())
    invalid = [l for l in targets if l not in LANGS]
    if invalid:
        print(f"ERROR: 未対応の言語: {invalid}")
        sys.exit(1)

    print(f"\n=== 修復開始 ({len(targets)} 言語) ===\n")
    for lang in targets:
        target_lang = LANGS[lang]
        gid = glossary_ids.get(lang)
        repair_lang(lang, target_lang, ja_sections, gid, translator, args.dry_run)
        print()

    print("✅ All done!")
    if not args.dry_run:
        print("\n次のステップ:")
        print("  for lang in en fr de it es nl pt; do")
        print("    node scripts/extract-translation-pairs-v2.cjs $lang")
        print("  done")
        print("  ~/.xcomet-venv/bin/python scripts/evaluate_xcomet.py all --force")


if __name__ == '__main__':
    main()
