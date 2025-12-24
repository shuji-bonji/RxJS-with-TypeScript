#!/usr/bin/env node
/**
 * Auto-evaluate all translation pairs using xCOMET MCP Server
 * Connects directly to the Python server for batch evaluation
 */

const fs = require('fs');
const path = require('path');

const lang = process.argv[2] || 'en';
const pairsFile = path.join(__dirname, `translation-pairs-${lang}.json`);
const resultsFile = path.join(__dirname, `evaluation-results-${lang}.json`);

// Load data
const pairs = JSON.parse(fs.readFileSync(pairsFile, 'utf8'));
let results;
try {
  results = JSON.parse(fs.readFileSync(resultsFile, 'utf8'));
} catch {
  results = { language: lang, evaluatedFiles: [], lastUpdated: null };
}

// Group by file
const byFile = {};
pairs.pairs.forEach(p => {
  if (!byFile[p.file]) byFile[p.file] = [];
  byFile[p.file].push({ source: p.source, translation: p.translation });
});

const files = Object.keys(byFile).sort();
const evaluatedSet = new Set(results.evaluatedFiles.map(f => f.file));
const pendingFiles = files.filter(f => !evaluatedSet.has(f));

console.log(`Language: ${lang}`);
console.log(`Total files: ${files.length}`);
console.log(`Already evaluated: ${evaluatedSet.size}`);
console.log(`Pending: ${pendingFiles.length}`);
console.log('');

// Find Python server port (check for model_loaded: true)
async function findServerPort() {
  const { execSync } = require('child_process');
  try {
    const output = execSync('lsof -i -P | grep python3 | grep LISTEN', { encoding: 'utf8' });
    const matches = output.matchAll(/localhost:(\d+)/g);
    const ports = [...matches].map(m => parseInt(m[1], 10));

    // Check each port for a working server with model loaded
    for (const port of ports) {
      try {
        const response = await fetch(`http://127.0.0.1:${port}/health`);
        if (response.ok) {
          const health = await response.json();
          if (health.model_loaded) {
            return port;
          }
        }
      } catch {
        // Try next port
      }
    }

    // If no server with model loaded, return the first port and let it load
    return ports[0] || null;
  } catch {
    // No server found
  }
  return null;
}

// Evaluate using direct HTTP call to Python server
async function evaluateBatch(filePairs, port) {
  const response = await fetch(`http://127.0.0.1:${port}/batch_evaluate`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      pairs: filePairs,
      batch_size: 8,
      use_gpu: false
    })
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || `HTTP ${response.status}`);
  }

  return response.json();
}

// Save result
function saveResult(file, score, pairCount) {
  results.evaluatedFiles.push({
    file,
    score: Math.round(score * 1000) / 1000,
    pairCount,
    evaluatedAt: new Date().toISOString()
  });
  results.lastUpdated = new Date().toISOString();
  fs.writeFileSync(resultsFile, JSON.stringify(results, null, 2));
}

// Main
async function main() {
  const port = await findServerPort();
  if (!port) {
    console.error('Error: xCOMET Python server not found.');
    console.error('Please ensure the MCP server is running.');
    process.exit(1);
  }

  console.log(`Found xCOMET server on port ${port}`);
  console.log('Starting evaluation...\n');

  let completed = 0;
  let errors = 0;
  const startTime = Date.now();

  for (const file of pendingFiles) {
    const filePairs = byFile[file];
    const fileIndex = files.indexOf(file) + 1;

    process.stdout.write(`[${fileIndex}/${files.length}] ${file} (${filePairs.length} pairs)... `);

    try {
      const result = await evaluateBatch(filePairs, port);
      saveResult(file, result.average_score, filePairs.length);
      console.log(`Score: ${result.average_score.toFixed(3)}`);
      completed++;
    } catch (error) {
      console.log(`ERROR: ${error.message}`);
      errors++;

      // If server is down, exit
      if (error.message.includes('fetch failed') || error.message.includes('ECONNREFUSED')) {
        console.error('\nServer connection lost. Exiting.');
        break;
      }
    }

    // Small delay to avoid overwhelming the server
    await new Promise(r => setTimeout(r, 100));
  }

  const elapsed = Math.round((Date.now() - startTime) / 1000);
  console.log('\n--- Summary ---');
  console.log(`Completed: ${completed}`);
  console.log(`Errors: ${errors}`);
  console.log(`Time: ${elapsed}s`);
  console.log(`Total evaluated: ${results.evaluatedFiles.length}/${files.length}`);
}

main().catch(err => {
  console.error('Fatal error:', err);
  process.exit(1);
});
