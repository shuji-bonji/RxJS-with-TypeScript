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
LANGS = ['en', 'fr', 'de', 'it', 'es', 'nl', 'pt']

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


def fix_adjacent_double_close(text: str) -> tuple[str, int]:
    """連続する閉じ ``` のうち余分な方を除去 (見出し直前のみ、厳格)

    安全な検出条件:
        - 第 1 の ``` (閉じ)
        - 空行
        - 第 2 の ``` (この時点では opener の可能性も close 重複の可能性もある)
        - **空行** (これが重要: opener の場合は直後にコンテンツが続く)
        - 見出し (#) で始まる行 ← これが決定打

    Markdown では opener の直後に空行を入れることはほぼ無い。空行 + 見出し
    が来る場合は第 2 の ``` が orphan close の可能性が極めて高い。

    判定が曖昧な場合は触らない (false positive を避ける)。
    """
    pattern = re.compile(
        r'^```\s*$\n+'                # 閉じ ```
        r'^```\s*$'                   # 余分な ``` 候補
        r'\n[ \t]*\n+'                # 必ず空行が後続 (1+ 行の空行)
        r'(?=#)',                     # 直後の文字は # (heading)
        re.MULTILINE
    )
    new_text, n = pattern.subn('```\n\n', text)
    return new_text, n


def fix_double_close_bare_lang(text: str) -> tuple[str, int]:
    """過去のパッチで壊された fence ペアを修復

    パターン:
        ```           <- 閉じ
                      <- 空行
        ```           <- 余計な閉じ (= 開きだった残骸)
                      <- 空行
        ts            <- 裸 lang
        code...
        ```           <- 本来の閉じ

    修正後:
        ```           <- 元のブロックの閉じ
                      <- 空行
        ```ts         <- 新規ブロック開き
        code...
        ```           <- 本来の閉じ

    (連続した ``` ペアと、その後の bare lang を結合)
    """
    LANGS = r'(?:ts|typescript|js|javascript|json|python|py|sh|bash|html|css|scss|mermaid|yaml|yml|sql|rust|go|java|cpp|c|csharp|kotlin|swift|ruby|rb|php|plaintext|text|txt|diff)'
    # ```\n+```\n+lang\n の連続パターン
    pattern = re.compile(
        r'^```\s*$\n+'              # 閉じ ```
        r'^```\s*$\n+'              # 余計な閉じ ```
        r'^(' + LANGS + r')\s*$',   # bare lang
        re.MULTILINE
    )
    changes = 0

    def repl(m):
        nonlocal changes
        changes += 1
        lang = m.group(1)
        return f'```\n\n```{lang}'

    new_text = pattern.sub(repl, text)
    return new_text, changes


def fix_split_fence_pair(text: str) -> tuple[str, int]:
    """DeepL が壊した「閉じフェンス + 開きフェンス」ペアを修復

    パターン:
        ```text       <- 閉じ ``` の後ろに余計な (...) が付加
         (extra)
                      <- 空行
        ts            <- 開き ```ts が ``` を失って lang だけになっている
        import ...
        ```

    修正後:
        ```           <- 正しい閉じ
                      <- 空行
        ```ts         <- 正しい開き
        import ...
        ```
    """
    LANGS = r'(?:ts|typescript|js|javascript|json|python|py|sh|bash|html|css|scss|mermaid|yaml|yml|sql|rust|go|java|cpp|c|csharp|kotlin|swift|ruby|rb|php|plaintext|text|txt|diff)'
    pattern = re.compile(
        r'^```\s+\([^)\n]*\)\s*$'   # 閉じフェンスに余計な (...) が付いている行
        r'(\n\s*)*'                  # 空行
        r'^(' + LANGS + r')\s*$',    # bare lang 行
        re.MULTILINE
    )
    changes = 0

    def repl(m):
        nonlocal changes
        changes += 1
        lang = m.group(2)
        return f'```\n\n```{lang}'

    new_text = pattern.sub(repl, text)
    return new_text, changes


def fix_malformed_fence_languages(text: str) -> tuple[str, int]:
    r"""壊れた言語マーカーを持つコードフェンスを正規化

    Vue ビルドで以下のような警告が出るパターンに対処:
        ```}     <- 言語マーカーが '}' になっている (Vue 補間の破片)
        ```\d+___ <- (別関数で処理)
        ```ts.   <- 末尾にピリオド
        ```A___ etc.

    対策: フェンスの言語マーカーが英数字以外を含む or 妥当でない場合、
    マーカー部分を空にする (just '```')。
    ただし通常の言語 (ts, javascript, python, mermaid 等) は保持。
    """
    VALID_LANGS = {'ts', 'typescript', 'js', 'javascript', 'json', 'python', 'py',
                   'sh', 'bash', 'shell', 'html', 'css', 'scss', 'mermaid', 'yaml',
                   'yml', 'xml', 'md', 'markdown', 'sql', 'rust', 'go', 'java',
                   'cpp', 'c', 'csharp', 'cs', 'kotlin', 'swift', 'ruby', 'rb',
                   'php', 'plaintext', 'text', 'txt', 'diff'}
    changes = 0

    def repl(m):
        nonlocal changes
        lang = m.group(1).strip()
        # 既知の有効な言語ならそのまま
        if lang.lower() in VALID_LANGS:
            return m.group(0)
        # 識別子っぽい (英数字とハイフン、アンダーバー) 場合は許容
        if re.match(r'^[a-zA-Z][a-zA-Z0-9_-]*$', lang):
            return m.group(0)
        # 不正なマーカー (}, .ts., 0___ 等) は剥がす
        changes += 1
        return '```'

    new_text = re.sub(r'^```([^\n]*)$', repl, text, flags=re.MULTILINE)
    return new_text, changes


def fix_residual_placeholder_fences(text: str) -> tuple[str, int]:
    """残存した ```N___ 形式の orphan fence を除去

    Issue #34 翻訳時のプレースホルダー復元バグで、___CODE_N___ プレースホルダーが
    そのまま ```N___ という形でフェンスとして残ることがある。
    対応する閉じ ``` も含めてペアで除去し、間にある内容 (見出し・段落等) を本文に戻す。

    パターン例:
        ```        <- 直前のコードブロックを閉じる
        (空行)
        ```N___    <- orphan opening
        (見出しや段落)
        ```        <- orphan closing (実際は次のセクション開始位置)
        (空行)
        ```ts      <- 次の正常なコードブロック
    """
    lines = text.split('\n')
    result = []
    i = 0
    removed = 0
    while i < len(lines):
        # ```N___ または ```N___. のパターン (orphan placeholder fence)
        if re.match(r'^```\d+___\.?\s*$', lines[i]):
            # 次の ``` 単独 (orphan close) を探す
            j = i + 1
            while j < len(lines):
                if re.match(r'^```\s*$', lines[j]):
                    break
                # 別のフェンスが先に出てきたら相方は見つからず → 単独除去のみ
                if re.match(r'^```\S', lines[j]):
                    j = -1  # 相方なし
                    break
                j += 1

            if j > 0 and j < len(lines):
                # ペアで除去: L[i] と L[j] を削除、間 (L[i+1:j]) は保持
                # ただし result の末尾空行と L[i+1] の空行を整理
                while result and result[-1].strip() == '':
                    result.pop()
                result.append('')  # 区切り空行 1 つ
                # 間の content を追加 (前後の空行をトリム)
                inner = lines[i+1:j]
                # 前後の空行を除去
                while inner and inner[0].strip() == '':
                    inner.pop(0)
                while inner and inner[-1].strip() == '':
                    inner.pop()
                if inner:
                    result.extend(inner)
                    result.append('')
                i = j + 1
                removed += 1
                continue
            else:
                # 相方なし: 単独の ```N___ 行のみ削除
                # 前後空行整理
                while result and result[-1].strip() == '':
                    result.pop()
                result.append('')
                i += 1
                # 次の空行も飛ばす
                while i < len(lines) and lines[i].strip() == '':
                    i += 1
                removed += 1
                continue
        result.append(lines[i])
        i += 1
    return '\n'.join(result), removed


def fix_unescaped_generics(text: str) -> tuple[str, int]:
    """Markdown 本文 (非コード) の <T> 等のジェネリック型を \\<T> にエスケープ

    Vue が HTML タグとして解釈してビルドエラーになる問題への対処。
    JA 原本では `Observable\\<T>` のように記述されているが、DeepL 翻訳時に
    バックスラッシュが脱落するケース。

    対象: 行頭で コードブロック外の <UppercaseIdent> パターン
    """
    changes = 0

    # 行ごとに処理: コードブロック内は変更しない
    lines = text.split('\n')
    in_code = False
    for i, line in enumerate(lines):
        # フェンス境界
        if re.match(r'^```', line):
            in_code = not in_code
            continue
        if in_code:
            continue
        # インラインコード (``` で囲まれた部分) を一時マスク
        masked = re.sub(r'`[^`\n]+`', lambda m: '\x00' * len(m.group(0)), line)
        # <UpperCase> パターンを検出
        new_masked = re.sub(r'<([A-Z][a-zA-Z0-9]*)>', r'\\<\1>', masked)
        if new_masked != masked:
            # 元の行に逆適用 (マスク位置を保ちながら置換)
            # シンプル化: 行内でインラインコード以外を直接 sub
            def replace_outside_code(m):
                nonlocal changes
                changes += 1
                return '\\<' + m.group(1) + '>'
            # インラインコードの位置を把握
            inline_codes = [(m.start(), m.end()) for m in re.finditer(r'`[^`\n]+`', line)]
            def is_inside_code(pos):
                return any(s <= pos < e for s, e in inline_codes)
            new_line = []
            j = 0
            while j < len(line):
                m = re.match(r'<([A-Z][a-zA-Z0-9]*)>', line[j:])
                # 既に \< でエスケープされている場合は重複エスケープしない
                already_escaped = j > 0 and line[j-1] == '\\'
                if m and not is_inside_code(j) and not already_escaped:
                    new_line.append('\\<' + m.group(1) + '>')
                    j += m.end()
                    changes += 1
                else:
                    new_line.append(line[j])
                    j += 1
            lines[i] = ''.join(new_line)

    return '\n'.join(lines), changes


def fix_duplicate_code_blocks(text: str) -> tuple[str, int]:
    """翻訳済みコードブロックの直後に JA 版コードが重複している箇所を除去

    Issue #34 翻訳時に DeepL が翻訳済みコードブロックの後に JA 原文の
    コードブロックを残してしまうバグへの対処。

    パターン:
        ```ts
        <翻訳済みコード>
        ```ts   <-- バグ: 本来 ``` (closing) であるべき
        <JA 原文コード>
        ```

    修正:
        ```ts
        <翻訳済みコード>
        ```      <-- closing
        (JA 重複ブロック全体削除)

    Returns:
        (修正済みテキスト, 除去されたブロック数)
    """
    lines = text.split('\n')
    result = []
    i = 0
    state = 'closed'  # closed | open
    removed_count = 0

    while i < len(lines):
        line = lines[i]
        m = re.match(r'^```(.*)$', line)
        if m:
            lang_marker = m.group(1).strip()
            if state == 'closed':
                # 新規 opening
                state = 'open'
                result.append(line)
                i += 1
            else:  # state == 'open'
                if lang_marker:
                    # BUG パターン: open 状態で言語付きフェンスに遭遇
                    # 次の closing (``` 単独) を探す
                    j = i + 1
                    while j < len(lines):
                        m2 = re.match(r'^```\s*$', lines[j])
                        if m2:
                            break
                        j += 1

                    if j < len(lines):
                        # i+1 から j-1 までの内容を確認
                        block_content = '\n'.join(lines[i+1:j])
                        # 日本語文字をカウント
                        ja_chars = len(re.findall(r'[぀-ゟ゠-ヿ一-鿿]', block_content))
                        if ja_chars >= 10:
                            # JA 重複ブロックとみなして除去
                            result.append('```')  # 翻訳済みブロックの closing
                            i = j + 1  # JA 重複ブロック (i 〜 j) を skip
                            state = 'closed'
                            removed_count += 1
                            continue

                    # JA 文字が少ない場合: 閉じフェンスが欠落している
                    # → 現位置の前に閉じ ``` を挿入し、現フェンスを新規 open とする
                    result.append('```')  # 直前ブロックの close を補う
                    result.append('')      # 空行 (可読性)
                    result.append(line)    # この行が新規 open
                    state = 'open'
                    removed_count += 1  # 「修正されたブロック」としてカウント
                    i += 1
                else:
                    # 通常の closing
                    result.append(line)
                    state = 'closed'
                    i += 1
        else:
            result.append(line)
            i += 1

    return '\n'.join(result), removed_count


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

    # 7. 翻訳済みコードブロック直後の JA 重複ブロックを除去
    text, n7 = fix_duplicate_code_blocks(text)
    if n7:
        actions.append(f'コードブロック重複除去 ({n7}箇所)')

    # 8. orphan placeholder fence (```N___) を除去
    text, n8 = fix_residual_placeholder_fences(text)
    if n8:
        actions.append(f'orphan fence 除去 ({n8}箇所)')

    # 9. 非コードの <T> 等を \\<T> にエスケープ (Vue ビルドエラー回避)
    text, n9 = fix_unescaped_generics(text)
    if n9:
        actions.append(f'ジェネリック型エスケープ ({n9}箇所)')

    # 10. ```text(...) + bare-lang の split-fence-pair を修復 (先に実行: パターンを温存)
    text, n10 = fix_split_fence_pair(text)
    if n10:
        actions.append(f'split fence pair 修復 ({n10}箇所)')

    # 11. 不正な言語マーカー (```} 等) を ``` に正規化 (split fence pair 後に実行)
    text, n11 = fix_malformed_fence_languages(text)
    if n11:
        actions.append(f'不正フェンスマーカー修正 ({n11}箇所)')

    # 12. recovery: 過去のパッチが ``` + ``` + bare lang を作ってしまった分を修復
    text, n12 = fix_double_close_bare_lang(text)
    if n12:
        actions.append(f'double-close 修復 ({n12}箇所)')

    # 13. recovery: 連続する余分な閉じ ``` を除去
    text, n13 = fix_adjacent_double_close(text)
    if n13:
        actions.append(f'隣接 close 重複除去 ({n13}箇所)')

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
                if not args.code_jp_only:
                    text, n7 = fix_duplicate_code_blocks(text)
                    if n7:
                        actions.append(f'コードブロック重複除去 ({n7}箇所)')
                    text, n8 = fix_residual_placeholder_fences(text)
                    if n8:
                        actions.append(f'orphan fence 除去 ({n8}箇所)')
                    text, n9 = fix_unescaped_generics(text)
                    if n9:
                        actions.append(f'ジェネリック型エスケープ ({n9}箇所)')
                    text, n10 = fix_split_fence_pair(text)
                    if n10:
                        actions.append(f'split fence pair 修復 ({n10}箇所)')
                    text, n11 = fix_malformed_fence_languages(text)
                    if n11:
                        actions.append(f'不正フェンスマーカー修正 ({n11}箇所)')
                    text, n12 = fix_double_close_bare_lang(text)
                    if n12:
                        actions.append(f'double-close 修復 ({n12}箇所)')
                    text, n13 = fix_adjacent_double_close(text)
                    if n13:
                        actions.append(f'隣接 close 重複除去 ({n13}箇所)')
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
