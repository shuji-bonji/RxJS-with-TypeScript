/**
 * Translate French markdown files to German using DeepL API
 *
 * This script reads French markdown files from docs/de/guide (copied from fr/guide),
 * translates the French content to German while preserving:
 * - Code blocks (TypeScript examples)
 * - Technical terms in English
 * - Markdown structure
 * - Internal links (already updated to /de/guide/)
 */

const fs = require('fs');
const path = require('path');

// DeepL API configuration (use environment variable or direct key)
const DEEPL_API_KEY = process.env.DEEPL_API_KEY;
const DEEPL_API_URL = 'https://api-free.deepl.com/v2/translate';

const DOCS_DE_DIR = path.join(__dirname, '../docs/de/guide');

/**
 * Extract code blocks and replace with placeholders
 */
function extractCodeBlocks(content) {
  const codeBlocks = [];
  let index = 0;

  // Match fenced code blocks
  const processed = content.replace(/```[\s\S]*?```/g, (match) => {
    codeBlocks.push(match);
    return `__CODE_BLOCK_${index++}__`;
  });

  return { processed, codeBlocks };
}

/**
 * Restore code blocks from placeholders
 */
function restoreCodeBlocks(content, codeBlocks) {
  let result = content;
  codeBlocks.forEach((block, index) => {
    result = result.replace(`__CODE_BLOCK_${index}__`, block);
  });
  return result;
}

/**
 * Translate text using DeepL API
 */
async function translateWithDeepL(text, sourceLang = 'FR', targetLang = 'DE') {
  if (!DEEPL_API_KEY) {
    throw new Error('DEEPL_API_KEY environment variable is not set');
  }

  const response = await fetch(DEEPL_API_URL, {
    method: 'POST',
    headers: {
      'Authorization': `DeepL-Auth-Key ${DEEPL_API_KEY}`,
      'Content-Type': 'application/x-www-form-urlencoded',
    },
    body: new URLSearchParams({
      text: text,
      source_lang: sourceLang,
      target_lang: targetLang,
      preserve_formatting: '1',
    }),
  });

  if (!response.ok) {
    throw new Error(`DeepL API error: ${response.status} ${response.statusText}`);
  }

  const data = await response.json();
  return data.translations[0].text;
}

/**
 * Process a single markdown file
 */
async function processFile(filePath) {
  console.log(`Processing: ${filePath}`);

  const content = fs.readFileSync(filePath, 'utf-8');

  // Extract code blocks
  const { processed, codeBlocks } = extractCodeBlocks(content);

  // Split into smaller chunks if needed (DeepL has limits)
  const chunks = splitIntoChunks(processed, 4000);

  // Translate each chunk
  const translatedChunks = [];
  for (const chunk of chunks) {
    const translated = await translateWithDeepL(chunk);
    translatedChunks.push(translated);
    // Rate limiting
    await new Promise(resolve => setTimeout(resolve, 100));
  }

  // Combine and restore code blocks
  const translatedContent = restoreCodeBlocks(translatedChunks.join(''), codeBlocks);

  // Write back
  fs.writeFileSync(filePath, translatedContent, 'utf-8');
  console.log(`  Translated: ${filePath}`);
}

/**
 * Split text into chunks respecting paragraph boundaries
 */
function splitIntoChunks(text, maxLength) {
  if (text.length <= maxLength) {
    return [text];
  }

  const chunks = [];
  const paragraphs = text.split('\n\n');
  let currentChunk = '';

  for (const para of paragraphs) {
    if ((currentChunk + '\n\n' + para).length > maxLength && currentChunk) {
      chunks.push(currentChunk);
      currentChunk = para;
    } else {
      currentChunk = currentChunk ? currentChunk + '\n\n' + para : para;
    }
  }

  if (currentChunk) {
    chunks.push(currentChunk);
  }

  return chunks;
}

/**
 * Get all markdown files
 */
function getMarkdownFiles(dir) {
  const files = [];
  const entries = fs.readdirSync(dir, { withFileTypes: true });

  for (const entry of entries) {
    const fullPath = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      files.push(...getMarkdownFiles(fullPath));
    } else if (entry.name.endsWith('.md')) {
      files.push(fullPath);
    }
  }

  return files;
}

/**
 * Main function
 */
async function main() {
  const files = getMarkdownFiles(DOCS_DE_DIR);
  console.log(`Found ${files.length} files to translate`);

  let processed = 0;
  for (const file of files) {
    try {
      await processFile(file);
      processed++;
      console.log(`Progress: ${processed}/${files.length}`);
    } catch (error) {
      console.error(`Error processing ${file}:`, error.message);
    }
  }

  console.log(`\nDone! Translated ${processed}/${files.length} files.`);
}

main().catch(console.error);
