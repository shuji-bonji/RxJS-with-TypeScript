/**
 * 翻訳ペア抽出スクリプト v2
 * セクションベースのマッチングで精度向上
 */

const fs = require('fs');
const path = require('path');

// 再帰的にファイル一覧を取得
function getFilesRecursively(dir, ext = '.md') {
  const files = [];
  const items = fs.readdirSync(dir, { withFileTypes: true });
  for (const item of items) {
    const fullPath = path.join(dir, item.name);
    if (item.isDirectory()) {
      files.push(...getFilesRecursively(fullPath, ext));
    } else if (item.name.endsWith(ext)) {
      files.push(fullPath);
    }
  }
  return files;
}

// コンテンツをセクション（##見出し）ごとに分割
function splitIntoSections(content) {
  const lines = content.split('\n');
  const sections = [];
  let currentSection = { heading: '_intro', lines: [] };
  let inFrontmatter = false;
  let frontmatterCount = 0;

  for (const line of lines) {
    // Frontmatter処理
    if (line.trim() === '---') {
      frontmatterCount++;
      if (frontmatterCount <= 2) {
        inFrontmatter = frontmatterCount === 1;
        continue;
      }
    }
    if (inFrontmatter) continue;

    // ## 見出しで新セクション開始
    if (line.match(/^##\s+/)) {
      if (currentSection.lines.length > 0) {
        sections.push(currentSection);
      }
      currentSection = { heading: line.trim(), lines: [] };
    } else {
      currentSection.lines.push(line);
    }
  }

  // 最後のセクション
  if (currentSection.lines.length > 0) {
    sections.push(currentSection);
  }

  return sections;
}

// セクション内のテキストセグメントを抽出
function extractSegmentsFromSection(lines) {
  const segments = [];
  let inCodeBlock = false;
  let currentSegment = [];

  for (const line of lines) {
    // コードブロック処理
    if (line.trim().startsWith('```')) {
      inCodeBlock = !inCodeBlock;
      // コードブロック開始時に現在のセグメントをフラッシュ
      if (inCodeBlock && currentSegment.length > 0) {
        const text = currentSegment.join(' ').trim();
        if (isValidSegment(text)) {
          segments.push(text);
        }
        currentSegment = [];
      }
      continue;
    }
    if (inCodeBlock) continue;

    // 空行でセグメント区切り
    if (line.trim() === '') {
      if (currentSegment.length > 0) {
        const text = currentSegment.join(' ').trim();
        if (isValidSegment(text)) {
          segments.push(text);
        }
        currentSegment = [];
      }
      continue;
    }

    currentSegment.push(line.trim());
  }

  // 最後のセグメント
  if (currentSegment.length > 0) {
    const text = currentSegment.join(' ').trim();
    if (isValidSegment(text)) {
      segments.push(text);
    }
  }

  return segments;
}

// 有効なセグメントかどうかを判定（言語適応型）
function isValidSegment(text) {
  // 見出し、テーブル、VitePressカスタムブロックを除外
  if (text.startsWith('#') ||
      text.startsWith('|') ||
      text.startsWith(':::') ||
      text.match(/^>\s*\[!/)) {
    return false;
  }

  // 短すぎるラベルのみのテキストを除外
  // 「**xxx**:」のようなラベルのみの行を除外
  if (text.match(/^\*\*[^*]+\*\*:?$/)) {
    return false;
  }

  // リンクのみの行を除外
  if (text.match(/^-?\s*\[.*\]\(.*\)$/)) {
    return false;
  }

  // 最低文字数（共通）
  if (text.length < 10) {
    return false;
  }

  // 日本語を含むかどうかで判定方法を分ける
  const hasJapanese = /[\u3040-\u309F\u30A0-\u30FF\u4E00-\u9FFF]/.test(text);

  if (hasJapanese) {
    // 日本語: 文字数ベース（15文字以上）
    return text.length >= 15;
  } else {
    // 非日本語: 単語数ベース（3単語以上）
    const words = text.split(/\s+/).filter(w => w.length > 0);
    return words.length >= 3;
  }
}

// セクション単位でペアを生成
function generatePairsBySection(jaSections, transSections) {
  const pairs = [];

  // イントロセクション（見出し前のテキスト）を処理
  const jaIntro = jaSections.find(s => s.heading === '_intro');
  const transIntro = transSections.find(s => s.heading === '_intro');
  if (jaIntro && transIntro) {
    const jaSegs = extractSegmentsFromSection(jaIntro.lines);
    const transSegs = extractSegmentsFromSection(transIntro.lines);
    const minLen = Math.min(jaSegs.length, transSegs.length);
    for (let i = 0; i < minLen; i++) {
      pairs.push({ source: jaSegs[i], translation: transSegs[i], section: '_intro' });
    }
  }

  // 各見出しセクションを順番にマッチング
  const jaHeadingSections = jaSections.filter(s => s.heading !== '_intro');
  const transHeadingSections = transSections.filter(s => s.heading !== '_intro');

  // セクション数が同じ場合は位置ベースでマッチング
  const minSections = Math.min(jaHeadingSections.length, transHeadingSections.length);

  for (let i = 0; i < minSections; i++) {
    const jaSection = jaHeadingSections[i];
    const transSection = transHeadingSections[i];

    const jaSegs = extractSegmentsFromSection(jaSection.lines);
    const transSegs = extractSegmentsFromSection(transSection.lines);

    const minLen = Math.min(jaSegs.length, transSegs.length);
    for (let j = 0; j < minLen; j++) {
      pairs.push({
        source: jaSegs[j],
        translation: transSegs[j],
        section: jaSection.heading
      });
    }
  }

  return pairs;
}

// メイン処理
function main() {
  const targetLang = process.argv[2];
  if (!targetLang) {
    console.error('Usage: node extract-translation-pairs-v2.cjs <lang>');
    console.error('Example: node extract-translation-pairs-v2.cjs en');
    process.exit(1);
  }

  const docsDir = path.join(__dirname, '..', 'docs');
  const jaGuideDir = path.join(docsDir, 'guide');
  const transGuideDir = path.join(docsDir, targetLang, 'guide');

  if (!fs.existsSync(transGuideDir)) {
    console.error(`Translation directory not found: ${transGuideDir}`);
    process.exit(1);
  }

  // 日本語ファイル一覧
  const jaFilesAbs = getFilesRecursively(jaGuideDir);
  const jaFiles = jaFilesAbs.map(f => path.relative(jaGuideDir, f));

  const allPairs = [];
  const fileStats = [];
  let totalSkipped = 0;

  for (const relPath of jaFiles) {
    const jaPath = path.join(jaGuideDir, relPath);
    const transPath = path.join(transGuideDir, relPath);

    if (!fs.existsSync(transPath)) {
      totalSkipped++;
      continue;
    }

    const jaContent = fs.readFileSync(jaPath, 'utf-8');
    const transContent = fs.readFileSync(transPath, 'utf-8');

    const jaSections = splitIntoSections(jaContent);
    const transSections = splitIntoSections(transContent);
    const pairs = generatePairsBySection(jaSections, transSections);

    if (pairs.length > 0) {
      allPairs.push(...pairs.map(p => ({
        source: p.source,
        translation: p.translation,
        file: relPath,
        section: p.section
      })));
      fileStats.push({
        file: relPath,
        jaSections: jaSections.length,
        transSections: transSections.length,
        pairs: pairs.length
      });
    }
  }

  // 結果を出力
  const output = {
    language: targetLang,
    version: 2,
    totalFiles: fileStats.length,
    totalPairs: allPairs.length,
    skippedFiles: totalSkipped,
    pairs: allPairs
  };

  const outputPath = path.join(__dirname, `translation-pairs-${targetLang}.json`);
  fs.writeFileSync(outputPath, JSON.stringify(output, null, 2));

  console.log(`Language: ${targetLang}`);
  console.log(`Files processed: ${fileStats.length}`);
  console.log(`Files skipped: ${totalSkipped}`);
  console.log(`Total pairs: ${allPairs.length}`);
  console.log(`Output: ${outputPath}`);
}

main();
