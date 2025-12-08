/**
 * Meta Description Analyzer
 * Analyzes all markdown files for SEO meta description quality
 */

const fs = require('fs');
const path = require('path');

const docsDir = path.join(__dirname, '..', 'docs');

// Results tracking
const results = {
  noDescription: [],
  shortDescription: [], // < 150 chars
  goodDescription: [], // 150-160 chars (ideal)
  longDescription: [], // > 160 chars
};

function extractFrontmatter(content) {
  const match = content.match(/^---\n([\s\S]*?)\n---/);
  if (!match) return null;

  const frontmatter = match[1];
  const descMatch = frontmatter.match(/description:\s*['"]?([\s\S]*?)['"]?\s*(?:\n[a-z]|$)/i);

  if (descMatch) {
    // Handle multiline descriptions
    let desc = descMatch[1].trim();
    // Remove quotes if present
    desc = desc.replace(/^['"]|['"]$/g, '');
    return desc;
  }

  return null;
}

function analyzeFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf-8');
  const relativePath = path.relative(docsDir, filePath);
  const description = extractFrontmatter(content);

  if (!description) {
    results.noDescription.push({ path: relativePath, length: 0 });
  } else {
    const len = description.length;
    const entry = { path: relativePath, length: len, preview: description.substring(0, 80) + (len > 80 ? '...' : '') };

    if (len < 150) {
      results.shortDescription.push(entry);
    } else if (len <= 160) {
      results.goodDescription.push(entry);
    } else {
      results.longDescription.push(entry);
    }
  }
}

function walkDir(dir) {
  const files = fs.readdirSync(dir);

  files.forEach(file => {
    const filePath = path.join(dir, file);
    const stat = fs.statSync(filePath);

    if (stat.isDirectory() && !file.startsWith('.')) {
      walkDir(filePath);
    } else if (file.endsWith('.md')) {
      analyzeFile(filePath);
    }
  });
}

// Run analysis
walkDir(docsDir);

// Output results
console.log('\n========== META DESCRIPTION ANALYSIS ==========\n');

console.log(`Total files analyzed: ${
  results.noDescription.length +
  results.shortDescription.length +
  results.goodDescription.length +
  results.longDescription.length
}\n`);

console.log(`NO DESCRIPTION: ${results.noDescription.length} files`);
if (results.noDescription.length > 0) {
  results.noDescription.slice(0, 10).forEach(f => console.log(`  - ${f.path}`));
  if (results.noDescription.length > 10) console.log(`  ... and ${results.noDescription.length - 10} more`);
}

console.log(`\nSHORT DESCRIPTION (<150 chars): ${results.shortDescription.length} files`);
if (results.shortDescription.length > 0) {
  results.shortDescription.slice(0, 15).forEach(f =>
    console.log(`  - ${f.path} (${f.length} chars)`)
  );
  if (results.shortDescription.length > 15) console.log(`  ... and ${results.shortDescription.length - 15} more`);
}

console.log(`\nGOOD DESCRIPTION (150-160 chars): ${results.goodDescription.length} files`);

console.log(`\nLONG DESCRIPTION (>160 chars): ${results.longDescription.length} files`);

// Write detailed report
const report = {
  summary: {
    noDescription: results.noDescription.length,
    shortDescription: results.shortDescription.length,
    goodDescription: results.goodDescription.length,
    longDescription: results.longDescription.length
  },
  shortDescriptionFiles: results.shortDescription,
  noDescriptionFiles: results.noDescription
};

fs.writeFileSync(
  path.join(__dirname, 'meta-description-report.json'),
  JSON.stringify(report, null, 2)
);

console.log('\n\nDetailed report saved to: scripts/meta-description-report.json');
