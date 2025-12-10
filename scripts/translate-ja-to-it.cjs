/**
 * Japanese to Italian Translation Script using DeepL API
 *
 * Usage:
 *   DEEPL_API_KEY=your_key node scripts/translate-ja-to-it.cjs [file_path]
 *
 * If no file_path is provided, it will list all files needing translation.
 */

const fs = require('fs');
const path = require('path');
const https = require('https');

const DOCS_DIR = path.join(__dirname, '../docs');
const JA_GUIDE_DIR = path.join(DOCS_DIR, 'guide');
const IT_GUIDE_DIR = path.join(DOCS_DIR, 'it/guide');

const DEEPL_API_KEY = process.env.DEEPL_API_KEY;

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
 * Translate text using DeepL API
 */
async function translateText(text, targetLang = 'IT') {
  return new Promise((resolve, reject) => {
    const postData = new URLSearchParams({
      auth_key: DEEPL_API_KEY,
      text: text,
      target_lang: targetLang,
      source_lang: 'JA'
    }).toString();

    const options = {
      hostname: 'api-free.deepl.com',
      port: 443,
      path: '/v2/translate',
      method: 'POST',
      headers: {
        'Content-Type': 'application/x-www-form-urlencoded',
        'Content-Length': Buffer.byteLength(postData)
      }
    };

    const req = https.request(options, (res) => {
      let data = '';
      res.on('data', chunk => data += chunk);
      res.on('end', () => {
        try {
          const result = JSON.parse(data);
          if (result.translations && result.translations[0]) {
            resolve(result.translations[0].text);
          } else {
            reject(new Error('Invalid response from DeepL'));
          }
        } catch (e) {
          reject(e);
        }
      });
    });

    req.on('error', reject);
    req.write(postData);
    req.end();
  });
}

/**
 * Extract and preserve code blocks
 */
function extractCodeBlocks(content) {
  const codeBlocks = [];
  let index = 0;

  const processed = content.replace(/```[\s\S]*?```/g, (match) => {
    const placeholder = `__CODE_BLOCK_${index}__`;
    codeBlocks.push(match);
    index++;
    return placeholder;
  });

  return { processed, codeBlocks };
}

/**
 * Restore code blocks
 */
function restoreCodeBlocks(content, codeBlocks) {
  let result = content;
  codeBlocks.forEach((block, index) => {
    result = result.replace(`__CODE_BLOCK_${index}__`, block);
  });
  return result;
}

/**
 * Update internal links from /guide/ to /it/guide/
 */
function updateLinks(content) {
  return content
    .replace(/\]\(\/guide\//g, '](/it/guide/')
    .replace(/\]\(\.\.\/guide\//g, '](../it/guide/');
}

/**
 * Translate a single file
 */
async function translateFile(relPath) {
  const sourcePath = path.join(JA_GUIDE_DIR, relPath);
  const targetPath = path.join(IT_GUIDE_DIR, relPath);

  if (!fs.existsSync(sourcePath)) {
    console.error(`Source file not found: ${sourcePath}`);
    return false;
  }

  // Create target directory if needed
  const targetDir = path.dirname(targetPath);
  if (!fs.existsSync(targetDir)) {
    fs.mkdirSync(targetDir, { recursive: true });
  }

  console.log(`Translating: ${relPath}`);

  const content = fs.readFileSync(sourcePath, 'utf-8');

  // Extract code blocks to preserve them
  const { processed, codeBlocks } = extractCodeBlocks(content);

  // Split into chunks for translation (DeepL has limits)
  const chunks = [];
  const lines = processed.split('\n');
  let currentChunk = [];
  let currentLength = 0;

  for (const line of lines) {
    if (currentLength + line.length > 4000) {
      chunks.push(currentChunk.join('\n'));
      currentChunk = [line];
      currentLength = line.length;
    } else {
      currentChunk.push(line);
      currentLength += line.length + 1;
    }
  }
  if (currentChunk.length > 0) {
    chunks.push(currentChunk.join('\n'));
  }

  // Translate each chunk
  const translatedChunks = [];
  for (let i = 0; i < chunks.length; i++) {
    console.log(`  Translating chunk ${i + 1}/${chunks.length}...`);
    try {
      const translated = await translateText(chunks[i]);
      translatedChunks.push(translated);
      // Rate limiting
      await new Promise(resolve => setTimeout(resolve, 500));
    } catch (error) {
      console.error(`  Error translating chunk ${i + 1}:`, error.message);
      translatedChunks.push(chunks[i]); // Keep original on error
    }
  }

  // Reassemble
  let translated = translatedChunks.join('\n');

  // Restore code blocks
  translated = restoreCodeBlocks(translated, codeBlocks);

  // Update links
  translated = updateLinks(translated);

  // Write translated file
  fs.writeFileSync(targetPath, translated, 'utf-8');
  console.log(`  Saved: ${targetPath}`);

  return true;
}

/**
 * List files needing translation
 */
function listFilesToTranslate() {
  const jaFiles = getMarkdownFiles(JA_GUIDE_DIR);
  const itFiles = getMarkdownFiles(IT_GUIDE_DIR);

  const itSet = new Set(itFiles);
  const needTranslation = jaFiles.filter(f => !itSet.has(f));

  console.log('\n=== Files needing Italian translation ===\n');
  console.log(`Total Japanese files: ${jaFiles.length}`);
  console.log(`Already translated: ${itFiles.length}`);
  console.log(`Need translation: ${needTranslation.length}\n`);

  if (needTranslation.length > 0) {
    console.log('Files to translate:');
    needTranslation.forEach((f, i) => {
      console.log(`  ${i + 1}. ${f}`);
    });
  }

  return needTranslation;
}

// Main
async function main() {
  const args = process.argv.slice(2);

  if (!DEEPL_API_KEY) {
    console.log('Note: DEEPL_API_KEY not set. Translation disabled.');
    listFilesToTranslate();
    return;
  }

  if (args.length === 0) {
    listFilesToTranslate();
  } else if (args[0] === 'all') {
    const files = listFilesToTranslate();
    console.log('\n--- Starting batch translation ---\n');
    for (const file of files) {
      await translateFile(file);
    }
    console.log('\n--- Translation complete ---');
  } else {
    await translateFile(args[0]);
  }
}

main().catch(console.error);
