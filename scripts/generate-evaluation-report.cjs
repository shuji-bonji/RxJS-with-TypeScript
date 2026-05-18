#!/usr/bin/env node
/**
 * scripts/generate-evaluation-report.cjs
 * ========================================
 *
 * evaluation-results-{lang}.json から evaluation-report-{lang}.md を生成。
 *
 * 使い方:
 *   node scripts/generate-evaluation-report.cjs <lang>
 *   node scripts/generate-evaluation-report.cjs all   # 全 7 言語
 *
 * 出力: scripts/evaluation-report-{lang}.md
 */

const fs = require('fs');
const path = require('path');

const LANG_NAMES = {
  en: 'English (英語)',
  fr: 'French (フランス語)',
  de: 'German (ドイツ語)',
  it: 'Italian (イタリア語)',
  es: 'Spanish (スペイン語)',
  nl: 'Dutch (オランダ語)',
  pt: 'Portuguese (ポルトガル語)',
};

function classify(score) {
  if (score >= 0.95) return { mark: '🟢', label: '優秀' };
  if (score >= 0.85) return { mark: '🔵', label: '良好' };
  if (score >= 0.70) return { mark: '🟡', label: '要改善' };
  return { mark: '🔴', label: '要修正' };
}

function categoryOf(file) {
  // file: e.g. "operators/filtering/audit.md"
  const parts = file.split('/');
  return parts[0];
}

function generate(lang) {
  const resultsPath = path.join(__dirname, `evaluation-results-${lang}.json`);
  if (!fs.existsSync(resultsPath)) {
    console.error(`Results file not found: ${resultsPath}`);
    return false;
  }
  const data = JSON.parse(fs.readFileSync(resultsPath, 'utf8'));
  const evaluated = data.evaluatedFiles || [];

  if (evaluated.length === 0) {
    console.error(`No evaluated files in ${resultsPath}`);
    return false;
  }

  const scores = evaluated.map(e => e.score);
  const avg = scores.reduce((a, b) => a + b, 0) / scores.length;
  const min = Math.min(...scores);
  const max = Math.max(...scores);

  // 品質分布
  const dist = { 優秀: 0, 良好: 0, 要改善: 0, 要修正: 0 };
  evaluated.forEach(e => { dist[classify(e.score).label]++; });

  // カテゴリごとに分類してソート
  const byCategory = {};
  evaluated.forEach(e => {
    const cat = categoryOf(e.file);
    if (!byCategory[cat]) byCategory[cat] = [];
    byCategory[cat].push(e);
  });
  Object.values(byCategory).forEach(arr => arr.sort((a, b) => a.file.localeCompare(b.file)));

  // 要修正リスト (score < 0.70)
  const needsFix = evaluated
    .filter(e => e.score < 0.70)
    .sort((a, b) => a.score - b.score);

  // レポート生成
  let md = `# ${LANG_NAMES[lang]} 翻訳評価結果\n\n`;

  md += `## サマリー\n\n`;
  md += `| 項目 | 値 |\n|------|-----|\n`;
  md += `| ファイル数 | ${evaluated.length} |\n`;
  md += `| 平均スコア | ${avg.toFixed(3)} |\n`;
  md += `| 最小スコア | ${min.toFixed(3)} |\n`;
  md += `| 最大スコア | ${max.toFixed(3)} |\n\n`;

  md += `### 品質分布\n\n`;
  md += `| 品質 | 件数 | 割合 |\n|------|------|------|\n`;
  const total = evaluated.length;
  md += `| 🟢 優秀 (≥0.95) | ${dist['優秀']} | ${(dist['優秀'] / total * 100).toFixed(1)}% |\n`;
  md += `| 🔵 良好 (0.85-0.94) | ${dist['良好']} | ${(dist['良好'] / total * 100).toFixed(1)}% |\n`;
  md += `| 🟡 要改善 (0.70-0.84) | ${dist['要改善']} | ${(dist['要改善'] / total * 100).toFixed(1)}% |\n`;
  md += `| 🔴 要修正 (<0.70) | ${dist['要修正']} | ${(dist['要修正'] / total * 100).toFixed(1)}% |\n\n`;

  if (needsFix.length > 0) {
    md += `## 🔴 要修正ファイル (スコア < 0.70)\n\n`;
    md += `| スコア | ファイル | ペア数 |\n|--------|----------|--------|\n`;
    needsFix.forEach(e => {
      md += `| ${classify(e.score).mark} ${e.score.toFixed(3)} | \`${e.file}\` | ${e.pairCount} |\n`;
    });
    md += `\n`;
  }

  md += `## ファイル別一覧\n\n`;
  const categoryOrder = Object.keys(byCategory).sort();
  categoryOrder.forEach(cat => {
    md += `\n### ${cat}\n\n`;
    md += `| スコア | ファイル | ペア数 |\n|--------|----------|--------|\n`;
    byCategory[cat].forEach(e => {
      const c = classify(e.score);
      md += `| ${c.mark} ${e.score.toFixed(3)} | ${e.file} | ${e.pairCount} |\n`;
    });
  });

  md += `\n\n---\n\n`;
  md += `最終更新: ${data.lastUpdated || 'N/A'}\n`;

  const outPath = path.join(__dirname, `evaluation-report-${lang}.md`);
  fs.writeFileSync(outPath, md);
  console.log(`✅ Generated: ${outPath}`);
  console.log(`   Files: ${evaluated.length}, Avg: ${avg.toFixed(3)}, NeedsFix: ${needsFix.length}`);
  return true;
}

const arg = process.argv[2];
if (!arg) {
  console.error('Usage: node scripts/generate-evaluation-report.cjs <lang|all>');
  process.exit(1);
}

if (arg === 'all') {
  const langs = ['en', 'fr', 'de', 'it', 'es', 'nl', 'pt'];
  langs.forEach(generate);
} else {
  generate(arg);
}
