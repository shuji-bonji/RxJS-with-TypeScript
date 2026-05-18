#!/usr/bin/env node
/**
 * scripts/validate-glossary-compliance.cjs
 * ==========================================
 *
 * .claude/skills/rxjs-glossary/glossary.json に基づいて、6 言語 (fr/de/it/es/nl/pt) の
 * ドキュメントが用語ルールに準拠しているか検証する。
 *
 * XCOMET (auto-evaluate.cjs) が捕捉できない「用語コンプライアンス」レイヤーを補完する。
 *
 * 検証項目:
 *  1. no_translate 違反: 固有名詞 (forkJoin, Observable 等) が JA に存在するのに、
 *     翻訳版で消失または異なる語に変換されていないか。
 *  2. callouts 遵守: > [!WARNING] / > [!IMPORTANT] の見出しが各言語の標準表現になっているか。
 *     (JA の callout フレーズが残存していないか)
 *  3. code_jp 残存: コードブロック内のコメントに日本語フラグメントが残っていないか。
 *
 * 使い方:
 *   node scripts/validate-glossary-compliance.cjs <lang>   # 単一言語
 *   node scripts/validate-glossary-compliance.cjs all      # 6 言語まとめて
 *   node scripts/validate-glossary-compliance.cjs --file <path> --lang <lang>
 *
 * 出力:
 *   scripts/glossary-compliance-{lang}.json   (機械可読)
 *   scripts/glossary-compliance-{lang}.md     (人間可読)
 */

const fs = require('fs');
const path = require('path');

const REPO = path.join(__dirname, '..');
const GLOSSARY_PATH = path.join(REPO, '.claude', 'skills', 'rxjs-glossary', 'glossary.json');
const LANGS = ['fr', 'de', 'it', 'es', 'nl', 'pt'];

// 言語表示名
const LANG_NAMES = {
  fr: 'French (フランス語)',
  de: 'German (ドイツ語)',
  it: 'Italian (イタリア語)',
  es: 'Spanish (スペイン語)',
  nl: 'Dutch (オランダ語)',
  pt: 'Portuguese (ポルトガル語)',
};

// ---- ユーティリティ ----

function getFilesRecursively(dir, ext = '.md') {
  const files = [];
  if (!fs.existsSync(dir)) return files;
  const items = fs.readdirSync(dir, { withFileTypes: true });
  for (const item of items) {
    const full = path.join(dir, item.name);
    if (item.isDirectory()) files.push(...getFilesRecursively(full, ext));
    else if (item.name.endsWith(ext)) files.push(full);
  }
  return files;
}

// 文字列がコードブロックの内側にある位置を抽出
function extractCodeBlocks(text) {
  // ``` 〜 ``` で囲まれた部分を抽出
  const blocks = [];
  const fenceRegex = /```[^\n]*\n([\s\S]*?)\n```/g;
  let m;
  while ((m = fenceRegex.exec(text)) !== null) {
    blocks.push({
      content: m[1],
      start: m.index,
      end: m.index + m[0].length,
    });
  }
  return blocks;
}

// 日本語フラグメント検出 (ひらがな、カタカナ、漢字)
function findJapaneseFragments(text) {
  const regex = /[぀-ゟ゠-ヿ一-鿿]+[぀-ゟ゠-ヿ一-鿿\s、。「」『』！？]*/g;
  return [...text.matchAll(regex)].map(m => ({
    fragment: m[0].trim(),
    index: m.index,
  })).filter(x => x.fragment.length > 0);
}

// テキストからコード部分のみ抽出 (バッククォート inline + fenced ブロック)
function extractCodeContexts(text) {
  let codeOnly = '';
  // Fenced code blocks
  const fenced = text.match(/```[^\n]*\n([\s\S]*?)\n```/g) || [];
  fenced.forEach(b => { codeOnly += b + '\n'; });
  // Inline code `xxx`
  const inline = text.match(/`[^`\n]+`/g) || [];
  inline.forEach(b => { codeOnly += b + '\n'; });
  return codeOnly;
}

// 文字列の出現を全件カウント (コード内のみ。大文字小文字を区別、word boundary 適用)
function countOccurrencesInCode(text, needle) {
  if (!needle) return 0;
  const codeText = extractCodeContexts(text);
  const escaped = needle.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  // word boundary: 識別子の一部にならないようにする
  // ただし needle に記号が含まれる場合は単純マッチ
  const isIdent = /^[a-zA-Z_$][a-zA-Z0-9_$]*$/.test(needle);
  const pattern = isIdent ? `(?<![a-zA-Z0-9_$])${escaped}(?![a-zA-Z0-9_$])` : escaped;
  const regex = new RegExp(pattern, 'g');
  const matches = codeText.match(regex);
  return matches ? matches.length : 0;
}

// ---- メイン検証 ----

function validateFile(jaPath, transPath, relPath, lang, glossary) {
  const violations = {
    no_translate: [], // [{ term, ja_count, trans_count }]
    callouts_ja_residual: [], // [{ phrase, lineNo }]
    code_jp_residual: [], // [{ fragment, lineNo, blockSnippet }]
  };

  if (!fs.existsSync(jaPath) || !fs.existsSync(transPath)) return violations;

  const ja = fs.readFileSync(jaPath, 'utf8');
  const trans = fs.readFileSync(transPath, 'utf8');

  // --- 1. no_translate 違反 ---
  // 「完全消失」のみを違反とする。翻訳版が部分的に短いケースは Issue #34 (網羅性)
  // で別途検出するため、ここでは扱わない。
  //
  // 違反条件: JA に 3 回以上出現する固有名詞が、翻訳版で完全消失 (count = 0)
  for (const term of glossary.no_translate) {
    const jaCount = countOccurrencesInCode(ja, term);
    if (jaCount < 3) continue; // ノイズ排除: 出現頻度が低い用語はスキップ
    const transCount = countOccurrencesInCode(trans, term);
    if (transCount === 0) {
      violations.no_translate.push({
        term,
        ja_count: jaCount,
        trans_count: transCount,
        ratio: 0,
      });
    }
  }

  // --- 2. callouts JA 残存 ---
  // JA の callout フレーズが翻訳版に残っていないか
  const jaCalloutPhrases = [
    '本番コードでの注意',
    'retryWhen は非推奨',
    'retryWhenは非推奨',
    'shareReplay の使用',
    '注意点',
    '重要',
    'ヒント',
    '警告',
    'メモリリーク',
    'パフォーマンス',
  ];
  const transLines = trans.split('\n');
  jaCalloutPhrases.forEach(phrase => {
    transLines.forEach((line, i) => {
      if (!line.includes(phrase)) return;
      // URL アンカー (#xxx-忘れ-メモリリーク 等) や Markdown リンク内は誤検出として除外
      // [...](.../#anchor-含む-JA) パターンや、(.../#anchor#JA) などをスキップ
      const lineWithoutLinks = line.replace(/\[[^\]]*\]\([^)]*\)/g, ''); // [text](url) を除去
      if (!lineWithoutLinks.includes(phrase)) return;
      // インラインコード ``...`` 内も除外
      const noInlineCode = lineWithoutLinks.replace(/`[^`]+`/g, '');
      if (!noInlineCode.includes(phrase)) return;
      violations.callouts_ja_residual.push({
        phrase,
        lineNo: i + 1,
        line: line.trim().slice(0, 100),
      });
    });
  });

  // --- 3. code_jp 残存 (コードブロック内の日本語) ---
  const blocks = extractCodeBlocks(trans);
  blocks.forEach(block => {
    const fragments = findJapaneseFragments(block.content);
    fragments.forEach(f => {
      // 行番号を逆引き
      const upTo = trans.slice(0, block.start + f.index);
      const lineNo = upTo.split('\n').length;
      violations.code_jp_residual.push({
        fragment: f.fragment.slice(0, 60),
        lineNo,
      });
    });
  });

  return violations;
}

function summarizeFileViolations(v) {
  return v.no_translate.length + v.callouts_ja_residual.length + v.code_jp_residual.length;
}

function run(lang, options = {}) {
  if (!fs.existsSync(GLOSSARY_PATH)) {
    console.error(`Glossary not found: ${GLOSSARY_PATH}`);
    process.exit(1);
  }
  const glossary = JSON.parse(fs.readFileSync(GLOSSARY_PATH, 'utf8'));
  if (!glossary[lang]) {
    console.error(`Language not in glossary: ${lang}`);
    return null;
  }

  const jaDir = path.join(REPO, 'docs', 'guide');
  const transDir = path.join(REPO, 'docs', lang, 'guide');
  if (!fs.existsSync(transDir)) {
    console.error(`Translation dir not found: ${transDir}`);
    return null;
  }

  const jaFiles = getFilesRecursively(jaDir);
  const fileResults = [];
  let totalViolations = 0;
  let filesWithViolations = 0;

  for (const jaPath of jaFiles) {
    const relPath = path.relative(jaDir, jaPath);
    if (options.filter && relPath !== options.filter) continue;

    const transPath = path.join(transDir, relPath);
    if (!fs.existsSync(transPath)) continue;

    const v = validateFile(jaPath, transPath, relPath, lang, glossary);
    const count = summarizeFileViolations(v);

    fileResults.push({
      file: relPath,
      violations: v,
      total: count,
    });

    if (count > 0) {
      filesWithViolations++;
      totalViolations += count;
    }
  }

  // 違反順にソート
  fileResults.sort((a, b) => b.total - a.total);

  return {
    language: lang,
    totalFiles: fileResults.length,
    filesWithViolations,
    totalViolations,
    files: fileResults,
  };
}

// ---- レポート出力 ----

function generateMarkdown(result) {
  let md = `# ${LANG_NAMES[result.language]} 用語遵守チェック結果\n\n`;
  md += `## サマリー\n\n`;
  md += `| 項目 | 値 |\n|------|-----|\n`;
  md += `| 検査ファイル数 | ${result.totalFiles} |\n`;
  md += `| 違反のあるファイル数 | ${result.filesWithViolations} |\n`;
  md += `| 違反総数 | ${result.totalViolations} |\n\n`;

  // 違反タイプ別集計
  const byType = { no_translate: 0, callouts: 0, code_jp: 0 };
  result.files.forEach(f => {
    byType.no_translate += f.violations.no_translate.length;
    byType.callouts += f.violations.callouts_ja_residual.length;
    byType.code_jp += f.violations.code_jp_residual.length;
  });
  md += `### 違反タイプ別\n\n`;
  md += `| タイプ | 件数 |\n|--------|------|\n`;
  md += `| 🔴 no_translate (固有名詞消失) | ${byType.no_translate} |\n`;
  md += `| 🟡 callouts JA 残存 | ${byType.callouts} |\n`;
  md += `| 🟠 code_jp 残存 (コード内日本語) | ${byType.code_jp} |\n\n`;

  // 違反のあるファイルだけ詳細表示
  const withViolations = result.files.filter(f => f.total > 0);
  if (withViolations.length === 0) {
    md += `## ✅ 違反なし\n\nすべてのファイルが用語遵守をクリアしています。\n`;
    return md;
  }

  md += `## 違反詳細 (Top 30)\n\n`;
  withViolations.slice(0, 30).forEach(f => {
    md += `### \`${f.file}\` (違反 ${f.total} 件)\n\n`;
    if (f.violations.no_translate.length > 0) {
      md += `**no_translate 違反:**\n\n`;
      md += `| 用語 | JA出現 | 翻訳出現 | 比率 |\n|------|--------|----------|------|\n`;
      f.violations.no_translate.slice(0, 10).forEach(v => {
        md += `| \`${v.term}\` | ${v.ja_count} | ${v.trans_count} | ${v.ratio} |\n`;
      });
      md += `\n`;
    }
    if (f.violations.callouts_ja_residual.length > 0) {
      md += `**callouts JA 残存:**\n\n`;
      f.violations.callouts_ja_residual.slice(0, 5).forEach(v => {
        md += `- L${v.lineNo}: \`${v.line}\`\n`;
      });
      md += `\n`;
    }
    if (f.violations.code_jp_residual.length > 0) {
      md += `**code_jp 残存 (コードブロック内):**\n\n`;
      f.violations.code_jp_residual.slice(0, 5).forEach(v => {
        md += `- L${v.lineNo}: \`${v.fragment}\`\n`;
      });
      if (f.violations.code_jp_residual.length > 5) {
        md += `- ... 他 ${f.violations.code_jp_residual.length - 5} 件\n`;
      }
      md += `\n`;
    }
  });

  if (withViolations.length > 30) {
    md += `\n*...残り ${withViolations.length - 30} ファイルは JSON レポート参照*\n`;
  }

  md += `\n---\n\n生成日時: ${new Date().toISOString()}\n`;
  return md;
}

function saveReports(result) {
  const lang = result.language;
  const jsonPath = path.join(__dirname, `glossary-compliance-${lang}.json`);
  const mdPath = path.join(__dirname, `glossary-compliance-${lang}.md`);
  fs.writeFileSync(jsonPath, JSON.stringify(result, null, 2));
  fs.writeFileSync(mdPath, generateMarkdown(result));
  console.log(`  ✅ ${jsonPath}`);
  console.log(`  ✅ ${mdPath}`);
}

// ---- CLI ----

function parseArgs(argv) {
  const opts = { lang: null, file: null };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    if (a === '--lang' || a === '-l') opts.lang = argv[++i];
    else if (a === '--file' || a === '-f') opts.file = argv[++i];
    else if (!opts.lang) opts.lang = a;
  }
  return opts;
}

function main() {
  const opts = parseArgs(process.argv);
  if (!opts.lang) {
    console.error('Usage: node scripts/validate-glossary-compliance.cjs <lang|all> [--file <path>]');
    console.error('Example: node scripts/validate-glossary-compliance.cjs fr');
    console.error('         node scripts/validate-glossary-compliance.cjs all');
    console.error('         node scripts/validate-glossary-compliance.cjs fr --file operators/filtering/audit.md');
    process.exit(1);
  }

  const targetLangs = opts.lang === 'all' ? LANGS : [opts.lang];
  for (const lang of targetLangs) {
    console.log(`\n=== ${lang.toUpperCase()} ===`);
    const result = run(lang, { filter: opts.file });
    if (!result) continue;
    console.log(`  Files checked: ${result.totalFiles}`);
    console.log(`  Files with violations: ${result.filesWithViolations}`);
    console.log(`  Total violations: ${result.totalViolations}`);
    if (!opts.file) {
      saveReports(result);
    } else {
      // 単一ファイルの場合は標準出力に詳細
      console.log(JSON.stringify(result.files[0], null, 2));
    }
  }
}

main();
