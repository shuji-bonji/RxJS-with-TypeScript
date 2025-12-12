/**
 * Translation Helper Script
 *
 * This script helps manage the translation of documentation files.
 * It lists files that need translation and tracks progress.
 *
 * Usage:
 *   node scripts/translate-files.cjs list <lang>      - List files needing translation
 *   node scripts/translate-files.cjs progress <lang> - Show translation progress
 *   node scripts/translate-files.cjs prepare <lang>  - Create directory structure
 */

const fs = require('fs');
const path = require('path');

const DOCS_DIR = path.join(__dirname, '../docs');
const EN_GUIDE_DIR = path.join(DOCS_DIR, 'en/guide');

const LANGUAGES = {
  fr: 'French',
  de: 'German',
  it: 'Italian',
  es: 'Spanish',
  nl: 'Dutch',
  pt: 'Portuguese'
};

/**
 * Get all markdown files in a directory recursively
 */
function getMarkdownFiles(dir, basePath = '') {
  const files = [];

  if (!fs.existsSync(dir)) {
    return files;
  }

  const entries = fs.readdirSync(dir, { withFileTypes: true });

  for (const entry of entries) {
    const relativePath = path.join(basePath, entry.name);
    const fullPath = path.join(dir, entry.name);

    if (entry.isDirectory()) {
      files.push(...getMarkdownFiles(fullPath, relativePath));
    } else if (entry.name.endsWith('.md')) {
      files.push(relativePath);
    }
  }

  return files;
}

/**
 * List files that need translation for a language
 */
function listFilesToTranslate(lang) {
  const enFiles = getMarkdownFiles(EN_GUIDE_DIR);
  const targetDir = path.join(DOCS_DIR, lang, 'guide');
  const targetFiles = getMarkdownFiles(targetDir);

  const targetSet = new Set(targetFiles);
  const needTranslation = enFiles.filter(f => !targetSet.has(f));

  console.log(`\n=== Files needing translation for ${LANGUAGES[lang]} (${lang}) ===\n`);
  console.log(`Total English files: ${enFiles.length}`);
  console.log(`Already translated: ${targetFiles.length}`);
  console.log(`Need translation: ${needTranslation.length}\n`);

  if (needTranslation.length > 0) {
    console.log('Files to translate:');
    needTranslation.forEach((f, i) => {
      console.log(`  ${i + 1}. ${f}`);
    });
  }

  return needTranslation;
}

/**
 * Show translation progress for a language
 */
function showProgress(lang) {
  const enFiles = getMarkdownFiles(EN_GUIDE_DIR);
  const targetDir = path.join(DOCS_DIR, lang, 'guide');
  const targetFiles = getMarkdownFiles(targetDir);

  const total = enFiles.length;
  const done = targetFiles.length;
  const percentage = ((done / total) * 100).toFixed(1);

  console.log(`\n=== Translation Progress for ${LANGUAGES[lang]} (${lang}) ===\n`);
  console.log(`Progress: ${done}/${total} (${percentage}%)`);
  console.log(`${'█'.repeat(Math.floor(percentage / 5))}${'░'.repeat(20 - Math.floor(percentage / 5))} ${percentage}%`);

  // Group by directory
  const byDir = {};
  enFiles.forEach(f => {
    const dir = path.dirname(f) || '.';
    if (!byDir[dir]) byDir[dir] = { total: 0, done: 0 };
    byDir[dir].total++;
  });

  targetFiles.forEach(f => {
    const dir = path.dirname(f) || '.';
    if (byDir[dir]) byDir[dir].done++;
  });

  console.log('\nBy directory:');
  Object.keys(byDir).sort().forEach(dir => {
    const { total, done } = byDir[dir];
    const pct = ((done / total) * 100).toFixed(0);
    console.log(`  ${dir}: ${done}/${total} (${pct}%)`);
  });
}

/**
 * Create directory structure for a language
 */
function prepareDirectories(lang) {
  const enFiles = getMarkdownFiles(EN_GUIDE_DIR);
  const targetBaseDir = path.join(DOCS_DIR, lang, 'guide');

  // Get unique directories
  const dirs = new Set();
  enFiles.forEach(f => {
    const dir = path.dirname(f);
    if (dir && dir !== '.') {
      dirs.add(dir);
    }
  });

  console.log(`\n=== Creating directories for ${LANGUAGES[lang]} (${lang}) ===\n`);

  // Create directories
  dirs.forEach(dir => {
    const fullPath = path.join(targetBaseDir, dir);
    if (!fs.existsSync(fullPath)) {
      fs.mkdirSync(fullPath, { recursive: true });
      console.log(`Created: ${lang}/guide/${dir}`);
    }
  });

  console.log('\nDirectory structure ready.');
}

/**
 * Show overall progress for all languages
 */
function showAllProgress() {
  const enFiles = getMarkdownFiles(EN_GUIDE_DIR);
  const total = enFiles.length;

  console.log('\n=== Translation Progress Overview ===\n');
  console.log(`Source files (English): ${total}\n`);

  Object.keys(LANGUAGES).forEach(lang => {
    const targetDir = path.join(DOCS_DIR, lang, 'guide');
    const targetFiles = getMarkdownFiles(targetDir);
    const done = targetFiles.length;
    const percentage = ((done / total) * 100).toFixed(1);

    console.log(`${LANGUAGES[lang]} (${lang}): ${done}/${total} (${percentage}%)`);
    console.log(`  ${'█'.repeat(Math.floor(percentage / 5))}${'░'.repeat(20 - Math.floor(percentage / 5))}`);
  });
}

// Main
const args = process.argv.slice(2);
const command = args[0];
const lang = args[1];

if (!command) {
  showAllProgress();
} else if (command === 'list' && lang && LANGUAGES[lang]) {
  listFilesToTranslate(lang);
} else if (command === 'progress' && lang && LANGUAGES[lang]) {
  showProgress(lang);
} else if (command === 'progress' && !lang) {
  showAllProgress();
} else if (command === 'prepare' && lang && LANGUAGES[lang]) {
  prepareDirectories(lang);
} else {
  console.log(`
Usage:
  node scripts/translate-files.cjs                    - Show overall progress
  node scripts/translate-files.cjs list <lang>        - List files needing translation
  node scripts/translate-files.cjs progress <lang>    - Show progress for a language
  node scripts/translate-files.cjs prepare <lang>     - Create directory structure

Languages: fr (French), de (German), it (Italian), es (Spanish), nl (Dutch), pt (Portuguese)
`);
}
