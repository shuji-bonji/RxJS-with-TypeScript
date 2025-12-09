/**
 * Batch translation script for operators directory (JA -> DE)
 * Translates all markdown files from Japanese to German
 *
 * Usage: node translate-operators.js
 *
 * Requirements:
 * - DeepL API key (set as DEEPL_API_KEY environment variable)
 * - npm install deepl-node
 */

const fs = require('fs').promises;
const path = require('path');

// File paths
const SOURCE_DIR = './docs/guide/operators';
const TARGET_DIR = './docs/de/guide/operators';

// List of all subdirectories
const SUBDIRS = [
  '', // root level (index.md, pipeline.md)
  'transformation',
  'filtering',
  'combination',
  'utility',
  'conditional',
  'multicasting'
];

/**
 * Get all markdown files in operators directory
 */
async function getAllMarkdownFiles() {
  const files = [];

  for (const subdir of SUBDIRS) {
    const dirPath = path.join(SOURCE_DIR, subdir);
    try {
      const entries = await fs.readdir(dirPath);
      for (const entry of entries) {
        if (entry.endsWith('.md')) {
          files.push({
            subdir,
            filename: entry,
            sourcePath: path.join(dirPath, entry),
            targetPath: path.join(TARGET_DIR, subdir, entry)
          });
        }
      }
    } catch (err) {
      console.error(`Error reading directory ${dirPath}:`, err.message);
    }
  }

  return files;
}

/**
 * Translate markdown content using DeepL
 * This is a placeholder - you need to implement actual DeepL API call
 */
async function translateContent(content, filename) {
  console.log(`Translating ${filename}...`);

  // TODO: Implement DeepL translation
  // You need to:
  // 1. Parse frontmatter (keep it, translate description)
  // 2. Translate markdown text (preserve code blocks)
  // 3. Keep technical terms in English
  // 4. Update internal links from /guide/ to /de/guide/

  // For now, return placeholder
  return `<!-- TODO: Translate this file using DeepL API -->\n${content}`;
}

/**
 * Process a single file
 */
async function processFile(file) {
  try {
    // Read source file
    const content = await fs.readFile(file.sourcePath, 'utf-8');

    // Translate content
    const translatedContent = await translateContent(content, file.filename);

    // Ensure target directory exists
    const targetDir = path.dirname(file.targetPath);
    await fs.mkdir(targetDir, { recursive: true });

    // Write translated file
    await fs.writeFile(file.targetPath, translatedContent, 'utf-8');

    console.log(`✓ Translated: ${file.subdir}/${file.filename}`);
    return { success: true, file: file.filename };
  } catch (err) {
    console.error(`✗ Error translating ${file.filename}:`, err.message);
    return { success: false, file: file.filename, error: err.message };
  }
}

/**
 * Main translation function
 */
async function main() {
  console.log('Starting batch translation of operators directory (JA -> DE)...\n');

  // Get all files
  const files = await getAllMarkdownFiles();
  console.log(`Found ${files.length} markdown files to translate\n`);

  // Process files
  const results = [];
  for (const file of files) {
    const result = await processFile(file);
    results.push(result);

    // Add delay to avoid API rate limits
    await new Promise(resolve => setTimeout(resolve, 100));
  }

  // Summary
  console.log('\n=== Translation Summary ===');
  const successful = results.filter(r => r.success).length;
  const failed = results.filter(r => !r.success).length;
  console.log(`Total: ${results.length}`);
  console.log(`Successful: ${successful}`);
  console.log(`Failed: ${failed}`);

  if (failed > 0) {
    console.log('\nFailed files:');
    results.filter(r => !r.success).forEach(r => {
      console.log(`  - ${r.file}: ${r.error}`);
    });
  }
}

// Run if called directly
if (require.main === module) {
  main().catch(err => {
    console.error('Fatal error:', err);
    process.exit(1);
  });
}

module.exports = { getAllMarkdownFiles, processFile };
