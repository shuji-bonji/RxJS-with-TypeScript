#!/usr/bin/env python3
"""
scripts/evaluate_xcomet.py
============================

xCOMET モデル (Unbabel/XCOMET-XL) を使って翻訳ペアを per-file 平均スコアで評価する。

`scripts/auto-evaluate.cjs` の代替。MCP を介さず unbabel-comet ライブラリを直接利用するため
高速・確実に動作する (auto-evaluate.cjs は xcomet-mcp-server v0.3 の HTTP サーバー前提で
動作しなくなっていた)。

前提条件:
    pip install unbabel-comet
    # HuggingFace 認証 (XCOMET-XL は gated model)
    huggingface-cli login
    # モデルダウンロード (~14GB、初回のみ)
    python -c "from comet import download_model; download_model('Unbabel/XCOMET-XL')"

使い方:
    # 全 7 言語を評価
    python scripts/evaluate_xcomet.py all

    # 単一言語
    python scripts/evaluate_xcomet.py fr

    # GPU 利用 (CUDA 環境のみ)
    python scripts/evaluate_xcomet.py all --gpu

    # 既評価ファイルもスキップせず強制再評価
    python scripts/evaluate_xcomet.py all --force

    # バッチサイズを大きくして高速化 (メモリに余裕がある場合)
    python scripts/evaluate_xcomet.py all --batch-size 16

入力:
    scripts/translation-pairs-{lang}.json  (extract-translation-pairs-v2.cjs の出力)

出力:
    scripts/evaluation-results-{lang}.json  (per-file 平均スコア)
    scripts/evaluation-report-{lang}.md     (generate-evaluation-report.cjs で別途生成)
"""

import argparse
import json
import logging
import os
import sys
import time
import warnings
from collections import defaultdict
from pathlib import Path

# PyTorch Lightning / Transformers の冗長ログを抑制
os.environ.setdefault('TRANSFORMERS_VERBOSITY', 'error')
os.environ.setdefault('PYTHONWARNINGS', 'ignore')
warnings.filterwarnings('ignore')
logging.getLogger('pytorch_lightning').setLevel(logging.ERROR)
logging.getLogger('lightning').setLevel(logging.ERROR)
logging.getLogger('lightning_fabric').setLevel(logging.ERROR)
logging.getLogger('transformers').setLevel(logging.ERROR)

REPO = Path(__file__).resolve().parent.parent
SCRIPTS = REPO / 'scripts'

LANGS = ['en', 'fr', 'de', 'it', 'es', 'nl', 'pt']
DEFAULT_MODEL = 'Unbabel/XCOMET-XL'


def reextract_pairs(lang: str) -> bool:
    """extract-translation-pairs-v2.cjs を呼んでペアを再抽出"""
    import subprocess
    script = SCRIPTS / 'extract-translation-pairs-v2.cjs'
    if not script.exists():
        return False
    print(f"  🔄 Re-extracting pairs for {lang}...")
    result = subprocess.run(['node', str(script), lang], capture_output=True, text=True)
    if result.returncode != 0:
        print(f"  ❌ pair extraction failed: {result.stderr}")
        return False
    print(f"  ✅ Pairs re-extracted")
    return True


def load_model(model_name: str, use_gpu: bool):
    """COMET モデルをロード"""
    print(f"📦 Loading model: {model_name}")
    print(f"   GPU: {use_gpu}")
    print(f"   (初回は ~14GB ダウンロード、その後はキャッシュ利用)")
    t0 = time.time()
    from comet import download_model, load_from_checkpoint
    model_path = download_model(model_name)
    model = load_from_checkpoint(model_path)
    elapsed = time.time() - t0
    print(f"   ✅ Loaded in {elapsed:.1f}s\n")
    return model


def evaluate_lang(lang: str, model, batch_size: int, use_gpu: bool, force: bool, num_workers: int = 1, auto_extract: bool = False) -> bool:
    """単一言語の全ファイルを評価"""
    pairs_file = SCRIPTS / f'translation-pairs-{lang}.json'
    results_file = SCRIPTS / f'evaluation-results-{lang}.json'

    if not pairs_file.exists():
        print(f"❌ {pairs_file} not found. Run extract-translation-pairs-v2.cjs first.")
        return False

    # ペアファイル鮮度チェック: docs/ 内のいずれかの md がペアファイルより新しい場合に警告
    docs_dir = REPO / 'docs' / ('guide' if lang == 'ja' else f'{lang}/guide')
    if docs_dir.exists():
        pairs_mtime = pairs_file.stat().st_mtime
        newer_files = []
        for md in docs_dir.rglob('*.md'):
            if md.stat().st_mtime > pairs_mtime:
                newer_files.append(md.relative_to(docs_dir))
                if len(newer_files) >= 3:
                    break
        if newer_files:
            print(f"⚠️  {pairs_file.name} is OLDER than {lang} docs.")
            print(f"   Newer files (sample): {newer_files[:3]}")
            if auto_extract:
                if reextract_pairs(lang):
                    # 再読み込み
                    with open(pairs_file, encoding='utf-8') as f:
                        pairs_data = json.load(f)
                    pairs = pairs_data['pairs']
                    print()
                else:
                    print(f"   Continuing with stale pairs...\n")
            else:
                print(f"   → Run: node scripts/extract-translation-pairs-v2.cjs {lang}")
                print(f"   Or use --auto-extract to do this automatically.")
                print(f"   Continuing with potentially stale pairs...\n")

    with open(pairs_file, encoding='utf-8') as f:
        pairs_data = json.load(f)
    pairs = pairs_data['pairs']

    # ファイル単位でグループ化
    by_file = defaultdict(list)
    for p in pairs:
        by_file[p['file']].append(p)
    files = sorted(by_file.keys())

    # 既評価結果を読み込み
    if results_file.exists() and not force:
        with open(results_file, encoding='utf-8') as f:
            results = json.load(f)
    else:
        results = {'language': lang, 'evaluatedFiles': [], 'lastUpdated': None}

    evaluated_set = {f['file'] for f in results['evaluatedFiles']}
    pending = [f for f in files if f not in evaluated_set] if not force else files

    if force:
        # 強制再評価モードでは結果リセット
        results['evaluatedFiles'] = []

    print('=' * 60)
    print(f"🌐 [{lang.upper()}] 評価開始 - JA → {lang}")
    print('=' * 60)
    print(f"  Total files: {len(files)}")
    print(f"  Already evaluated: {len(evaluated_set) if not force else 0}")
    print(f"  Pending: {len(pending)}")
    if not pending:
        print(f"  ✅ All evaluated. Use --force to re-evaluate.\n")
        return True

    t0 = time.time()
    completed = 0
    errors = 0

    for file in pending:
        file_pairs = by_file[file]
        # COMET 形式に変換
        data = [{'src': p['source'], 'mt': p['translation']} for p in file_pairs]
        try:
            # num_workers >= 1 を明示: Mac MPS / 一部 PyTorch バージョンで
            # 'multiprocessing_context can only be used with num_workers > 0' エラー回避
            # stderr 抑制: PyTorch Lightning の GPU/Tip ログを抑える
            import contextlib, io, os
            with contextlib.redirect_stderr(io.StringIO()):
                output = model.predict(data, batch_size=batch_size, gpus=1 if use_gpu else 0,
                                       num_workers=num_workers, progress_bar=False)
            scores = output.scores
            avg = sum(scores) / len(scores)
            entry = {
                'file': file,
                'score': round(avg, 3),
                'pairCount': len(file_pairs),
                'evaluatedAt': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            }
            # 既存の評価を置き換え (force / 再評価対応)
            results['evaluatedFiles'] = [e for e in results['evaluatedFiles']
                                         if e['file'] != file]
            results['evaluatedFiles'].append(entry)
            results['lastUpdated'] = entry['evaluatedAt']

            # インクリメンタルに保存 (大規模評価中断対応)
            with open(results_file, 'w', encoding='utf-8') as f:
                json.dump(results, f, indent=2, ensure_ascii=False)

            completed += 1
            mark = '🟢' if avg >= 0.95 else ('🔵' if avg >= 0.85 else ('🟡' if avg >= 0.70 else '🔴'))
            # 言語プレフィックス付きでどの言語処理中か明示
            print(f"  [{lang}][{completed}/{len(pending)}] {mark} {avg:.3f} - {file} ({len(file_pairs)} pairs)")
        except Exception as e:
            errors += 1
            print(f"  ❌ ERROR - {file}: {e}")
            continue

    elapsed = time.time() - t0
    avg_time = elapsed / completed if completed else 0
    print(f"\n  Completed: {completed}/{len(pending)}, Errors: {errors}")
    print(f"  Time: {elapsed:.1f}s (avg {avg_time:.1f}s/file)")
    if completed:
        scores = [e['score'] for e in results['evaluatedFiles']]
        print(f"  Overall avg: {sum(scores)/len(scores):.3f}")
    print()
    return True


def main():
    parser = argparse.ArgumentParser(description='XCOMET 翻訳品質評価')
    parser.add_argument('lang', help="言語コード or 'all' (en/fr/de/it/es/nl/pt)")
    parser.add_argument('--model', default=DEFAULT_MODEL,
                       help=f'XCOMET モデル (default: {DEFAULT_MODEL})')
    parser.add_argument('--gpu', action='store_true',
                       help='GPU 利用 (CUDA 環境のみ)')
    parser.add_argument('--batch-size', type=int, default=8,
                       help='バッチサイズ (default: 8, 大きいほど高速だがメモリ消費増)')
    parser.add_argument('--num-workers', type=int, default=1,
                       help='DataLoader workers (default: 1)')
    parser.add_argument('--force', action='store_true',
                       help='既評価ファイルも強制再評価')
    parser.add_argument('--auto-extract', action='store_true',
                       help='ペアファイルが古い場合に extract-translation-pairs-v2.cjs を自動実行')
    args = parser.parse_args()

    if args.lang == 'all':
        target_langs = LANGS
    elif args.lang in LANGS:
        target_langs = [args.lang]
    else:
        print(f"❌ Invalid lang: {args.lang}. Use one of {LANGS} or 'all'.")
        sys.exit(1)

    # モデルを 1 回だけロード
    try:
        model = load_model(args.model, args.gpu)
    except ImportError:
        print("❌ unbabel-comet not installed. Run:")
        print("   pip install unbabel-comet")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Model load failed: {e}")
        sys.exit(1)

    for lang in target_langs:
        evaluate_lang(lang, model, args.batch_size, args.gpu, args.force,
                     args.num_workers, args.auto_extract)

    print("✅ All done!")
    print("📊 レポート生成:")
    print(f"   node scripts/generate-evaluation-report.cjs all")


if __name__ == '__main__':
    main()
