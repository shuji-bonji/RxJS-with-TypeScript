#!/usr/bin/env python3
"""
scripts/setup_deepl_glossaries.py
==================================

DeepL に Glossary を 6 言語ペア (JA → fr/de/it/es/nl/pt) で作成し、
作成された glossary_id を `.claude/skills/rxjs-glossary/deepl_glossary_ids.json` に保存する。

## 前提
- Python 3.10+
- `pip install deepl`
- 環境変数 `DEEPL_AUTH_KEY` に DeepL API キーを設定
- `.claude/skills/rxjs-glossary/glossary.json` が存在 (no_translate リストを含む)

## 使い方

```bash
# 既存があれば削除して新規作成
python scripts/setup_deepl_glossaries.py

# 既存があるか確認するだけ
python scripts/setup_deepl_glossaries.py --list

# 既存をすべて削除 (お掃除)
python scripts/setup_deepl_glossaries.py --delete-all
```

## Glossary の役割

DeepL Glossary に `forkJoin → forkJoin` のような identity マッピングを登録することで、
DeepL に「この単語は翻訳しないでくれ」と指示できる。

これにより:
- `forkJoin` → `fourcheJoindre` (fr) のような誤訳を防止
- `combineLatest` → `combinaDernier` (es/it) のような語訳混在を防止
- 後処理での修正コードを削減

## 制約 (DeepL Free 版)

- 1 glossary あたり最大 1,000 entries
- 1 アカウントで保持できる glossary 数に制限あり (確認は `--list`)
- 言語ペア (JA → 各言語) ごとに別 glossary が必要

## 関連

- glossary.json: `.claude/skills/rxjs-glossary/glossary.json`
- 使用箇所: `scripts/translate_files.py` の DeepL 呼び出し
- Skill: `.claude/skills/rxjs-glossary/SKILL.md`
"""

import argparse
import json
import os
import sys
from pathlib import Path

try:
    import deepl
except ImportError:
    print("ERROR: deepl パッケージが必要です。`pip install deepl` を実行してください。",
          file=sys.stderr)
    sys.exit(1)


REPO = Path(__file__).resolve().parent.parent
SKILL_DIR = REPO / '.claude' / 'skills' / 'rxjs-glossary'
GLOSSARY_JSON = SKILL_DIR / 'glossary.json'
IDS_PATH = SKILL_DIR / 'deepl_glossary_ids.json'

# 言語ディレクトリ名 → DeepL target_lang コード
# DeepL Glossary 対応言語: JA → EN-US/EN-GB/DE/FR/IT/ES/NL/PT-BR/PT-PT/RU/ZH 等
LANG_MAP = {
    'fr': 'FR',
    'de': 'DE',
    'it': 'IT',
    'es': 'ES',
    'nl': 'NL',
    'pt': 'PT-BR',  # PT-BR を使う (CHANGELOG.md と整合)
}

# glossary 名のプレフィックス
GLOSSARY_NAME_PREFIX = "rxjs-ja-"


def get_translator() -> deepl.Translator:
    api_key = os.getenv('DEEPL_AUTH_KEY')
    if not api_key:
        print("ERROR: 環境変数 DEEPL_AUTH_KEY が設定されていません", file=sys.stderr)
        sys.exit(1)
    return deepl.Translator(api_key)


def list_glossaries(translator: deepl.Translator) -> dict:
    """rxjs-ja-* で始まる既存 glossary をマップで返す ({target_lang: GlossaryInfo})"""
    glossaries = translator.list_glossaries()
    ours = {}
    for g in glossaries:
        if g.name.startswith(GLOSSARY_NAME_PREFIX):
            ours[g.target_lang.upper()] = g
    return ours


def load_glossary_data() -> dict:
    if not GLOSSARY_JSON.exists():
        print(f"ERROR: glossary.json が見つかりません: {GLOSSARY_JSON}", file=sys.stderr)
        print("  rxjs-glossary Skill を .claude/skills/ に配置してください", file=sys.stderr)
        sys.exit(1)
    return json.loads(GLOSSARY_JSON.read_text(encoding='utf-8'))


def build_entries(glossary_data: dict, target_lang: str) -> dict:
    """no_translate リスト + 各言語の callout 翻訳から DeepL entries を構築

    Returns:
        {ja_text: target_text} の dict
    """
    entries = {}

    # 1. no_translate: identity マッピング ("forkJoin" → "forkJoin")
    for term in glossary_data.get('no_translate', []):
        entries[term] = term

    return entries


def create_glossary(translator: deepl.Translator, lang_dir: str,
                    target_lang: str, entries: dict) -> deepl.GlossaryInfo:
    """指定言語ペアの glossary を作成"""
    name = f"{GLOSSARY_NAME_PREFIX}{lang_dir}"
    glossary = translator.create_glossary(
        name=name,
        source_lang='JA',
        target_lang=target_lang,
        entries=entries,
    )
    return glossary


def cmd_list(translator: deepl.Translator):
    """既存 glossary 一覧"""
    existing = list_glossaries(translator)
    if not existing:
        print("  (rxjs-* 系 glossary は登録されていません)")
        return
    print(f"  rxjs-* 系 glossary: {len(existing)} 個")
    for target_lang, g in sorted(existing.items()):
        print(f"    {target_lang}: {g.name}")
        print(f"      ID: {g.glossary_id}")
        print(f"      Entries: {g.entry_count}")
        print(f"      Created: {g.creation_time}")


def cmd_list_all(translator: deepl.Translator):
    """全 glossary 一覧 (rxjs-* に限定しない)"""
    all_g = translator.list_glossaries()
    if not all_g:
        print("  glossary は登録されていません")
        return
    print(f"  DeepL アカウントの全 glossary: {len(all_g)} 個\n")
    # ソース言語ペアでグルーピング
    by_pair: dict[str, list] = {}
    for g in all_g:
        key = f"{g.source_lang}→{g.target_lang}"
        by_pair.setdefault(key, []).append(g)
    for pair, glossaries in sorted(by_pair.items()):
        print(f"  📂 {pair}: {len(glossaries)} 個")
        for g in glossaries:
            print(f"     - {g.name}")
            print(f"       ID: {g.glossary_id}")
            print(f"       Entries: {g.entry_count}")
            print(f"       Created: {g.creation_time}")
    print(f"\n  Total: {len(all_g)} 個")
    print(f"\n  💡 不要なものを削除する場合:")
    print(f"     python scripts/setup_deepl_glossaries.py --delete-id <ID>")
    print(f"     python scripts/setup_deepl_glossaries.py --delete-by-pair <SRC-TGT>")


def cmd_delete_by_id(translator: deepl.Translator, glossary_id: str):
    """指定 ID の glossary を削除"""
    try:
        translator.delete_glossary(glossary_id)
        print(f"  🗑️  Deleted: {glossary_id}")
    except deepl.DeepLException as e:
        print(f"  ❌ Failed: {e}")


def cmd_delete_by_pair(translator: deepl.Translator, pair: str):
    """指定言語ペアの glossary をすべて削除 (例: 'JA-FR', 'EN-DE')"""
    try:
        src, tgt = pair.upper().split('-', 1)
    except ValueError:
        print(f"  ERROR: 'SRC-TGT' 形式で指定してください (例: JA-FR)")
        return
    all_g = translator.list_glossaries()
    deleted = 0
    for g in all_g:
        if g.source_lang.upper() == src and g.target_lang.upper() == tgt:
            translator.delete_glossary(g)
            print(f"  🗑️  Deleted: {g.name} ({g.glossary_id})")
            deleted += 1
    print(f"\n  Total: {deleted} 個削除")


def cmd_delete_all(translator: deepl.Translator):
    """既存 rxjs-* 系 glossary をすべて削除"""
    existing = list_glossaries(translator)
    if not existing:
        print("  削除対象なし")
        return
    for target_lang, g in existing.items():
        translator.delete_glossary(g)
        print(f"  🗑️  Deleted: {g.name} ({g.glossary_id})")
    print(f"\n  Total: {len(existing)} glossary 削除")


def cmd_setup(translator: deepl.Translator, only_lang: str = None):
    """glossary を一括作成 (既存があれば削除してから)

    Args:
        only_lang: 指定された場合、その言語のみ作成 (Free 版の 1 glossary 制限用)
    """
    glossary_data = load_glossary_data()

    # 既存を確認・削除
    existing = list_glossaries(translator)
    for target_lang, g in existing.items():
        translator.delete_glossary(g)
        print(f"  🗑️  Deleted old: {g.name}")

    # 対象言語フィルタ
    targets = LANG_MAP
    if only_lang:
        if only_lang not in LANG_MAP:
            print(f"  ERROR: 未対応の言語 '{only_lang}'. 対応: {list(LANG_MAP.keys())}")
            return
        targets = {only_lang: LANG_MAP[only_lang]}

    # 新規作成
    ids: dict[str, str] = {}
    for lang_dir, target_lang in targets.items():
        entries = build_entries(glossary_data, lang_dir)
        try:
            g = create_glossary(translator, lang_dir, target_lang, entries)
            ids[lang_dir] = g.glossary_id
            print(f"  ✅ {lang_dir} ({target_lang}): {g.glossary_id} "
                  f"({g.entry_count} entries)")
        except deepl.DeepLException as e:
            print(f"  ❌ {lang_dir} ({target_lang}): {e}")

    # 保存 (既存 IDs を保持してマージ)
    SKILL_DIR.mkdir(parents=True, exist_ok=True)
    if only_lang and IDS_PATH.exists():
        try:
            existing_ids = json.loads(IDS_PATH.read_text(encoding='utf-8'))
            existing_ids.update(ids)
            ids = existing_ids
        except Exception:
            pass
    IDS_PATH.write_text(json.dumps(ids, indent=2, ensure_ascii=False), encoding='utf-8')
    print(f"\n  📝 Saved IDs to: {IDS_PATH}")
    if only_lang:
        print(f"\n  ✨ {only_lang} 用の Glossary 作成完了。translate_files.py で利用できます")
    else:
        print(f"\n  使い方:")
        print(f"     scripts/translate_files.py が自動でこの ID を読み込みます")


def main():
    parser = argparse.ArgumentParser(description="DeepL Glossary 整備")
    parser.add_argument('--list', action='store_true', help="rxjs-* 系 glossary を一覧表示")
    parser.add_argument('--list-all', action='store_true',
                        help="DeepL アカウントの全 glossary を一覧表示 (枠確認用)")
    parser.add_argument('--delete-all', action='store_true',
                        help="既存 rxjs-* 系 glossary を削除")
    parser.add_argument('--delete-id', type=str, metavar='ID',
                        help="指定 ID の glossary を削除")
    parser.add_argument('--delete-by-pair', type=str, metavar='SRC-TGT',
                        help="指定言語ペアの glossary を削除 (例: JA-FR)")
    parser.add_argument('--lang', type=str, metavar='LANG',
                        help="指定言語のみ作成 (fr, de, it, es, nl, pt) - Free版の 1 glossary 制限対応")
    args = parser.parse_args()

    translator = get_translator()

    if args.list:
        cmd_list(translator)
    elif args.list_all:
        cmd_list_all(translator)
    elif args.delete_all:
        cmd_delete_all(translator)
    elif args.delete_id:
        cmd_delete_by_id(translator, args.delete_id)
    elif args.delete_by_pair:
        cmd_delete_by_pair(translator, args.delete_by_pair)
    else:
        cmd_setup(translator, only_lang=args.lang)


if __name__ == '__main__':
    main()
