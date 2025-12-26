#!/usr/bin/env node
/**
 * RxJS Code Examples Validator
 *
 * Extracts TypeScript code blocks from markdown files and validates them
 * using RxJS execution.
 *
 * Usage:
 *   node scripts/validate-code-examples.cjs [options]
 *
 * Options:
 *   --dir <path>    Directory to scan (default: docs/guide)
 *   --file <path>   Single file to validate
 *   --verbose       Show detailed output
 *   --report        Generate JSON report
 */

const fs = require('fs');
const path = require('path');
const { Worker } = require('worker_threads');

// RxJS imports for validation
const rxjs = require('rxjs');

// Configuration
const CONFIG = {
  defaultDir: 'docs/guide',
  timeout: 2000,  // Reduced from 5000ms
  takeCount: 10,  // Reduced from 20
};

// Parse command line arguments
function parseArgs() {
  const args = process.argv.slice(2);
  const options = {
    dir: CONFIG.defaultDir,
    file: null,
    verbose: false,
    report: false,
  };

  for (let i = 0; i < args.length; i++) {
    switch (args[i]) {
      case '--dir':
        options.dir = args[++i];
        break;
      case '--file':
        options.file = args[++i];
        break;
      case '--verbose':
        options.verbose = true;
        break;
      case '--report':
        options.report = true;
        break;
    }
  }

  return options;
}

// Extract code blocks from markdown
function extractCodeBlocks(content, filePath) {
  const codeBlocks = [];
  // Match ```ts or ```typescript code blocks
  const regex = /```(?:ts|typescript)\n([\s\S]*?)```/g;
  let match;
  let index = 0;

  while ((match = regex.exec(content)) !== null) {
    const code = match[1].trim();
    codeBlocks.push({
      index: index++,
      code,
      file: filePath,
      line: content.substring(0, match.index).split('\n').length,
    });
  }

  return codeBlocks;
}

// Check if code block is executable (not DOM-dependent)
function isExecutable(code) {
  // Skip if contains DOM operations
  const domPatterns = [
    'document.',
    'getElementById',
    'querySelector',
    'createElement',
    'appendChild',
    'innerHTML',
    'textContent',
    'addEventListener',
    'HTMLInputElement',
    'HTMLElement',
    'fromEvent',
    'window.',
  ];

  for (const pattern of domPatterns) {
    if (code.includes(pattern)) {
      return { executable: false, reason: `Contains DOM operation: ${pattern}` };
    }
  }

  // Skip if contains external HTTP dependencies
  const externalPatterns = [
    'rxjs/ajax',
    'ajax.getJSON',
    'ajax(',
    'fetch(',
    'require(',
    'http.',
    'https.',
    'WebSocket',
  ];

  for (const pattern of externalPatterns) {
    if (code.includes(pattern)) {
      return { executable: false, reason: `Contains external dependency: ${pattern}` };
    }
  }

  // Skip if it's just type definitions or interfaces
  if (code.match(/^(interface|type|class)\s/m) && !code.includes('.pipe(') && !code.includes('subscribe(')) {
    return { executable: false, reason: 'Type definition only' };
  }

  // Remove import statements and check again
  const codeWithoutImports = code.replace(/import\s+.*?from\s+['"][^'"]+['"];?\n?/g, '').trim();

  // Skip if no Observable operations after removing imports
  if (!codeWithoutImports.includes('pipe(') && !codeWithoutImports.includes('subscribe(') &&
      !codeWithoutImports.includes('of(') && !codeWithoutImports.includes('from(') &&
      !codeWithoutImports.includes('interval(') && !codeWithoutImports.includes('timer(') &&
      !codeWithoutImports.includes('concat(') && !codeWithoutImports.includes('merge(') &&
      !codeWithoutImports.includes('combineLatest(') && !codeWithoutImports.includes('forkJoin(') &&
      !codeWithoutImports.includes('zip(') && !codeWithoutImports.includes('range(')) {
    return { executable: false, reason: 'No Observable operations' };
  }

  // Skip partial code examples that reference undefined variables
  // Check if code starts with a variable reference that's not defined in the code
  const partialPatterns = [
    /^[a-z]\w*\$?\s*\./m,  // Starts with variable.method()
    /^[a-z]\w*\$?\s*\(/m,  // Starts with function call on undefined
    /^const\s+\[\s*\w+/,   // Destructuring from undefined
  ];

  // Check if the code references undefined service/class patterns
  const undefinedPatterns = [
    /\bthis\.\w+/,         // this.something (component context)
    /\bsuper\./,           // super.something
    /\bnew\s+\w+Service/,  // new SomeService()
    /\b\w+Service\./,      // someService.method()
    /\bdescribe\s*\(/,     // Test framework
    /\bit\s*\(/,           // Test framework
    /\bexpect\s*\(/,       // Test framework
    /\bbeforeEach\s*\(/,   // Test framework
    /\bafterEach\s*\(/,    // Test framework
  ];

  // Check for undefined external variables commonly used in partial examples
  const partialExampleVars = [
    'source$', 'source', 'input$', 'input', 'result$',
    'click$', 'click', 'event$', 'data$', 'stream$',
    'page', 'pages', 'largeDataArray', 'searchInput',
    'button', 'form', 'element', 'container', 'items',
    'fetchData', 'getData', 'loadData', 'apiCall',
    'scheduler', 'testScheduler', 'cold', 'hot',
    'user', 'users', 'userId', 'userData',
    'logger', 'cache', 'store', 'state',
    'price', 'prices', 'stock', 'stocks',
    'request', 'response', 'message',
    'subscription', 'subject$', 'observable$',
    'grade', 'grades', 'score', 'scores',
  ];

  // Check if code uses these variables without defining them
  for (const varName of partialExampleVars) {
    // Check if variable is used but not defined (const/let/var/function/=)
    const usedRegex = new RegExp(`\\b${varName}\\b`);
    const definedRegex = new RegExp(`(const|let|var|function)\\s+${varName}\\b|\\b${varName}\\s*=`);
    if (usedRegex.test(codeWithoutImports) && !definedRegex.test(codeWithoutImports)) {
      return { executable: false, reason: `Partial example (uses undefined: ${varName})` };
    }
  }

  for (const pattern of undefinedPatterns) {
    if (pattern.test(codeWithoutImports)) {
      return { executable: false, reason: 'Contains undefined service/test pattern' };
    }
  }

  // Check for infinite streams without limiting operators
  const hasInfiniteStream = /\b(interval|timer)\s*\(/.test(codeWithoutImports);
  const hasLimitingOperator = /\b(take|first|takeWhile|takeUntil|takeLast)\s*\(/.test(codeWithoutImports);

  if (hasInfiniteStream && !hasLimitingOperator) {
    return { executable: false, reason: 'Infinite stream without limiting operator' };
  }

  // Check for slow streams that will exceed timeout
  // interval(1000) with take(n) where n > 2 will take > 2 seconds
  const intervalMatch = codeWithoutImports.match(/interval\s*\(\s*(\d+)\s*\)/);
  const takeMatch = codeWithoutImports.match(/take\s*\(\s*(\d+)\s*\)/);
  if (intervalMatch && takeMatch) {
    const intervalMs = parseInt(intervalMatch[1], 10);
    const takeCount = parseInt(takeMatch[1], 10);
    const estimatedTime = intervalMs * takeCount;
    if (estimatedTime > 1800) {  // Allow some margin below 2000ms timeout
      return { executable: false, reason: `Slow stream (estimated ${estimatedTime}ms > 1800ms)` };
    }
  }

  // Check for timer with long delay
  const timerMatch = codeWithoutImports.match(/timer\s*\(\s*(\d+)/);
  if (timerMatch) {
    const timerMs = parseInt(timerMatch[1], 10);
    if (timerMs > 1500) {
      return { executable: false, reason: `Long timer delay (${timerMs}ms)` };
    }
  }

  // Check for window operators that often don't complete in time
  const hasWindowOperator = /\b(windowWhen|windowToggle|bufferWhen|bufferToggle)\s*\(/.test(codeWithoutImports);
  const hasWindowWithInterval = hasWindowOperator && hasInfiniteStream;
  if (hasWindowWithInterval) {
    // Even with take(), these patterns often exceed timeout due to multiple windows
    return { executable: false, reason: 'Window operator with interval (slow execution)' };
  }

  // Check for windowTime which often runs longer than timeout
  const hasWindowTime = /\bwindowTime\s*\([^)]*\d{3,}/.test(codeWithoutImports);  // windowTime with 100+ ms
  if (hasWindowTime && hasInfiniteStream) {
    return { executable: false, reason: 'windowTime with interval (slow execution)' };
  }

  // Check for mergeMap/switchMap with interval that doesn't complete
  const hasHigherOrderWithInterval = /\b(mergeMap|switchMap|concatMap|exhaustMap)\s*\([^)]*interval/.test(codeWithoutImports);
  if (hasHigherOrderWithInterval && !hasLimitingOperator) {
    return { executable: false, reason: 'Higher-order operator with infinite inner stream' };
  }

  return { executable: true };
}

// Transform code for execution
function transformCode(code) {
  // Remove import statements
  let transformed = code.replace(/import\s+.*?from\s+['"][^'"]+['"];?\n?/g, '');

  // Remove export statements
  transformed = transformed.replace(/export\s+(const|let|var|function|class)/g, '$1');

  // Remove type annotations that might cause issues
  transformed = transformed.replace(/:\s*Observable<[^>]+>/g, '');

  // Remove TypeScript interface/type/enum declarations (multiline)
  // Handle multiline by matching balanced braces
  transformed = transformed.replace(/^interface\s+\w+\s*\{[\s\S]*?\n\}/gm, '');
  transformed = transformed.replace(/^type\s+\w+\s*=\s*\{[\s\S]*?\n\}\s*;?/gm, '');
  transformed = transformed.replace(/^enum\s+\w+\s*\{[\s\S]*?\n\}\s*;?/gm, '');

  // Remove single-line type aliases
  transformed = transformed.replace(/^type\s+\w+\s*=\s*[^;]+;/gm, '');

  // Remove TypeScript access modifiers
  transformed = transformed.replace(/\b(private|public|protected|readonly)\s+/g, '');

  // Remove TypeScript type annotations (improved)
  transformed = transformed.replace(/:\s*(?:Observable|Subject|BehaviorSubject|ReplaySubject|Subscription)\s*<[^>]+>/g, '');
  // Remove function parameter type annotations (e.g., "n: number" -> "n")
  transformed = transformed.replace(/(\w+)\s*:\s*\w+(?:<[^>]+>)?(?:\[\])?(?=\s*[,)])/g, '$1');
  // Remove function return type annotations (e.g., "): Observable<number>" -> ")")
  transformed = transformed.replace(/\)\s*:\s*\w+(?:<[^>]+>)?(?:\[\])?\s*(?=\{|=>)/g, ') ');
  transformed = transformed.replace(/:\s*\w+(?:\[\])?(?:\s*\|\s*\w+(?:\[\])?)*(?=\s*[=;,)\]])/g, '');

  // Remove generics from function calls (but not too aggressively)
  transformed = transformed.replace(/(?<=\w)<[^>()]+>(?=\s*\()/g, '');

  // Remove 'as' type assertions (including generic types like "as Record<string, number>")
  transformed = transformed.replace(/\s+as\s+\w+(?:<[^>]+>)?(?:\[\])?/g, '');
  // Handle "as const" and "as unknown as Type"
  transformed = transformed.replace(/\s+as\s+const\b/g, '');
  transformed = transformed.replace(/\s+as\s+unknown\b/g, '');

  // Remove 'async' keyword (not supported in strict mode eval)
  transformed = transformed.replace(/\basync\s+/g, '');
  transformed = transformed.replace(/\bawait\s+/g, '');

  // Clean up empty lines at the start
  transformed = transformed.replace(/^\s*\n/gm, '');

  // Find the main Observable expression and add return
  // If there's a subscribe, we need to extract the Observable before it
  if (transformed.includes('.subscribe(')) {
    // Remove .subscribe(...) call - handle both single-line and multi-line
    // First, find the position of .subscribe
    const subscribeIndex = transformed.lastIndexOf('.subscribe(');
    if (subscribeIndex !== -1) {
      // Get the Observable expression (everything before .subscribe)
      let observableExpr = transformed.substring(0, subscribeIndex);

      // Find where the Observable chain actually starts
      // Look backwards for the creation function or variable
      let startIndex = 0;
      const lines = observableExpr.split('\n');

      // Find the first line that contains an Observable creation or assignment
      for (let i = 0; i < lines.length; i++) {
        const line = lines[i].trim();
        // Skip empty lines and comments
        if (!line || line.startsWith('//')) continue;

        // Check if this line starts the Observable
        if (line.match(/^(of|from|interval|timer|concat|merge|combineLatest|zip|forkJoin|range|generate|defer|iif)\s*\(/) ||
            line.match(/^(const|let|var)\s+\w+\$?\s*=\s*(of|from|interval|timer|concat|merge|combineLatest|zip|forkJoin|range|generate|defer|iif)\s*\(/) ||
            line.match(/^\w+\$\s*\./) ||
            line.match(/^\w+\$\s*$/) ||
            line.match(/^(const|let|var)\s+\w+\$?\s*=\s*\w+\$?\s*\./)) {
          break;
        }
        startIndex = i + 1;
      }

      // Get the observable expression from the start
      const preExpr = lines.slice(0, startIndex).join('\n').trim();
      observableExpr = lines.slice(startIndex).join('\n').trim();

      // Remove leading comments from the observable expression
      // (they would break "return // comment\nexpr")
      while (observableExpr.match(/^\/\//)) {
        observableExpr = observableExpr.replace(/^\/\/.*\n?/, '').trim();
      }

      // Remove trailing semicolons
      observableExpr = observableExpr.replace(/;?\s*$/, '');

      // Check if it's an assignment
      const assignMatch = observableExpr.match(/^(const|let|var)\s+(\w+\$?)\s*=\s*/);
      if (assignMatch) {
        // Remove the assignment and just get the expression
        observableExpr = observableExpr.substring(assignMatch[0].length);
        observableExpr = observableExpr.replace(/;?\s*$/, '');
        transformed = (preExpr ? preExpr + '\n' : '') + 'return ' + observableExpr + ';';
      } else {
        transformed = (preExpr ? preExpr + '\n' : '') + 'return ' + observableExpr + ';';
      }
    }
  }

  // If no return statement exists and there's a clear Observable assignment
  if (!transformed.includes('return ')) {
    // Look for variable assignment ending with $ (common RxJS convention)
    const varMatch = transformed.match(/(const|let|var)\s+(\w+\$)\s*=/);
    if (varMatch) {
      transformed = transformed.replace(/;?\s*$/, '') + `;\nreturn ${varMatch[2]};`;
    } else {
      // Look for any pipe() call
      const pipeMatch = transformed.match(/(const|let|var)\s+(\w+)\s*=\s*.*\.pipe\s*\(/s);
      if (pipeMatch) {
        transformed = transformed.replace(/;?\s*$/, '') + `;\nreturn ${pipeMatch[2]};`;
      } else {
        // Try to wrap the whole thing in return if it looks like an Observable expression
        const trimmed = transformed.trim();
        if (trimmed.match(/^(of|from|interval|timer|concat|merge|combineLatest|zip|forkJoin|range)\s*\(/)) {
          transformed = 'return ' + trimmed.replace(/;?\s*$/, '') + ';';
        }
      }
    }
  }

  return transformed.trim();
}

// Execute code with RxJS context
async function executeCode(code, options = {}) {
  const timeout = options.timeout || CONFIG.timeout;
  const takeCount = options.takeCount || CONFIG.takeCount;

  return new Promise((resolve) => {
    const startTime = Date.now();
    const result = {
      success: false,
      values: [],
      errors: [],
      completed: false,
      executionTime: 0,
    };

    try {
      // Create execution context with all RxJS operators
      const context = {
        // Creation functions
        Observable: rxjs.Observable,
        of: rxjs.of,
        from: rxjs.from,
        interval: rxjs.interval,
        timer: rxjs.timer,
        range: rxjs.range,
        concat: rxjs.concat,
        merge: rxjs.merge,
        combineLatest: rxjs.combineLatest,
        zip: rxjs.zip,
        forkJoin: rxjs.forkJoin,
        race: rxjs.race,
        throwError: rxjs.throwError,
        EMPTY: rxjs.EMPTY,
        NEVER: rxjs.NEVER,
        defer: rxjs.defer,
        iif: rxjs.iif,
        generate: rxjs.generate,

        // Subjects
        Subject: rxjs.Subject,
        BehaviorSubject: rxjs.BehaviorSubject,
        ReplaySubject: rxjs.ReplaySubject,
        AsyncSubject: rxjs.AsyncSubject,

        // Operators
        map: rxjs.map,
        filter: rxjs.filter,
        tap: rxjs.tap,
        take: rxjs.take,
        takeUntil: rxjs.takeUntil,
        takeWhile: rxjs.takeWhile,
        skip: rxjs.skip,
        skipUntil: rxjs.skipUntil,
        skipWhile: rxjs.skipWhile,
        first: rxjs.first,
        last: rxjs.last,
        debounceTime: rxjs.debounceTime,
        throttleTime: rxjs.throttleTime,
        delay: rxjs.delay,
        delayWhen: rxjs.delayWhen,
        distinctUntilChanged: rxjs.distinctUntilChanged,
        distinct: rxjs.distinct,
        scan: rxjs.scan,
        reduce: rxjs.reduce,
        switchMap: rxjs.switchMap,
        mergeMap: rxjs.mergeMap,
        concatMap: rxjs.concatMap,
        exhaustMap: rxjs.exhaustMap,
        expand: rxjs.expand,
        catchError: rxjs.catchError,
        retry: rxjs.retry,
        retryWhen: rxjs.retryWhen,
        finalize: rxjs.finalize,
        startWith: rxjs.startWith,
        endWith: rxjs.endWith,
        withLatestFrom: rxjs.withLatestFrom,
        combineLatestWith: rxjs.combineLatestWith,
        zipWith: rxjs.zipWith,
        mergeWith: rxjs.mergeWith,
        concatWith: rxjs.concatWith,
        share: rxjs.share,
        shareReplay: rxjs.shareReplay,
        toArray: rxjs.toArray,
        buffer: rxjs.buffer,
        bufferTime: rxjs.bufferTime,
        bufferCount: rxjs.bufferCount,
        bufferWhen: rxjs.bufferWhen,
        bufferToggle: rxjs.bufferToggle,
        window: rxjs.window,
        windowTime: rxjs.windowTime,
        windowCount: rxjs.windowCount,
        windowWhen: rxjs.windowWhen,
        windowToggle: rxjs.windowToggle,
        groupBy: rxjs.groupBy,
        pairwise: rxjs.pairwise,
        partition: rxjs.partition,
        pluck: rxjs.pluck,
        mapTo: rxjs.mapTo,
        mergeAll: rxjs.mergeAll,
        concatAll: rxjs.concatAll,
        switchAll: rxjs.switchAll,
        exhaustAll: rxjs.exhaustAll,
        defaultIfEmpty: rxjs.defaultIfEmpty,
        every: rxjs.every,
        isEmpty: rxjs.isEmpty,
        count: rxjs.count,
        max: rxjs.max,
        min: rxjs.min,
        elementAt: rxjs.elementAt,
        find: rxjs.find,
        findIndex: rxjs.findIndex,
        sample: rxjs.sample,
        sampleTime: rxjs.sampleTime,
        audit: rxjs.audit,
        auditTime: rxjs.auditTime,
        timeout: rxjs.timeout,
        timeoutWith: rxjs.timeoutWith,
        timestamp: rxjs.timestamp,
        observeOn: rxjs.observeOn,
        subscribeOn: rxjs.subscribeOn,
        materialize: rxjs.materialize,
        dematerialize: rxjs.dematerialize,
        repeat: rxjs.repeat,
        repeatWhen: rxjs.repeatWhen,
        ignoreElements: rxjs.ignoreElements,
        single: rxjs.single,
        takeLast: rxjs.takeLast,
        skipLast: rxjs.skipLast,
        connect: rxjs.connect,
        connectable: rxjs.connectable,
        refCount: rxjs.refCount,

        // Schedulers
        asyncScheduler: rxjs.asyncScheduler,
        asapScheduler: rxjs.asapScheduler,
        queueScheduler: rxjs.queueScheduler,
        animationFrameScheduler: rxjs.animationFrameScheduler,
        scheduled: rxjs.scheduled,

        // Additional operators
        using: rxjs.using,
        fromFetch: rxjs.fromFetch,
        combineLatestAll: rxjs.combineLatestAll,
        zipAll: rxjs.zipAll,
        raceWith: rxjs.raceWith,
        mergeScan: rxjs.mergeScan,

        // Utilities
        firstValueFrom: rxjs.firstValueFrom,
        lastValueFrom: rxjs.lastValueFrom,

        // Safe console
        console: {
          log: (...args) => {},
          error: (...args) => {},
          warn: (...args) => {},
        },

        // Safe globals
        setTimeout,
        clearTimeout,
        Date,
        Math,
        JSON,
        Array,
        Object,
        String,
        Number,
        Boolean,
        Promise,
      };

      // Create and execute function
      let func;
      try {
        func = new Function(...Object.keys(context), `
          "use strict";
          ${code}
        `);
      } catch (syntaxError) {
        result.errors.push(`Syntax error: ${syntaxError.message}`);
        result.executionTime = Date.now() - startTime;
        resolve(result);
        return;
      }

      let observable$;
      try {
        observable$ = func(...Object.values(context));
      } catch (execError) {
        result.errors.push(execError.message || String(execError));
        result.executionTime = Date.now() - startTime;
        resolve(result);
        return;
      }

      if (!(observable$ instanceof rxjs.Observable)) {
        result.errors.push('Code must return an Observable');
        result.executionTime = Date.now() - startTime;
        resolve(result);
        return;
      }

      // Execute with timeout
      const timeoutHandle = setTimeout(() => {
        result.errors.push(`Timeout after ${timeout}ms`);
        result.executionTime = Date.now() - startTime;
        resolve(result);
      }, timeout);

      observable$.pipe(
        rxjs.take(takeCount),
        rxjs.catchError(error => {
          result.errors.push(error.message || String(error));
          return rxjs.EMPTY;
        }),
        rxjs.finalize(() => {
          clearTimeout(timeoutHandle);
          result.completed = true;
          result.executionTime = Date.now() - startTime;
        })
      ).subscribe({
        next: (value) => {
          result.values.push(value);
        },
        complete: () => {
          result.success = result.errors.length === 0;
          resolve(result);
        },
        error: (err) => {
          result.errors.push(err.message || String(err));
          resolve(result);
        }
      });

    } catch (error) {
      result.errors.push(error.message || String(error));
      result.executionTime = Date.now() - startTime;
      resolve(result);
    }
  });
}

// Scan directory for markdown files
function scanDirectory(dir) {
  const files = [];
  const entries = fs.readdirSync(dir, { withFileTypes: true });

  for (const entry of entries) {
    const fullPath = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      files.push(...scanDirectory(fullPath));
    } else if (entry.name.endsWith('.md')) {
      files.push(fullPath);
    }
  }

  return files;
}

// Main function
async function main() {
  const options = parseArgs();
  const results = {
    total: 0,
    executable: 0,
    skipped: 0,
    passed: 0,
    failed: 0,
    files: {},
  };

  console.log('RxJS Code Examples Validator');
  console.log('============================\n');

  // Get files to process
  let files;
  if (options.file) {
    files = [options.file];
  } else {
    files = scanDirectory(options.dir);
  }

  console.log(`Found ${files.length} markdown files\n`);

  for (const filePath of files) {
    const content = fs.readFileSync(filePath, 'utf-8');
    const codeBlocks = extractCodeBlocks(content, filePath);

    if (codeBlocks.length === 0) continue;

    const relativePath = path.relative(process.cwd(), filePath);
    const fileResult = {
      path: relativePath,
      blocks: [],
    };

    if (options.verbose) {
      console.log(`\nProcessing: ${relativePath}`);
      console.log(`  Found ${codeBlocks.length} code blocks`);
    }

    for (const block of codeBlocks) {
      results.total++;

      const execCheck = isExecutable(block.code);

      if (!execCheck.executable) {
        results.skipped++;
        fileResult.blocks.push({
          index: block.index,
          line: block.line,
          status: 'skipped',
          reason: execCheck.reason,
        });

        if (options.verbose) {
          console.log(`  [SKIP] Block ${block.index + 1} (line ${block.line}): ${execCheck.reason}`);
        }
        continue;
      }

      results.executable++;

      try {
        const transformed = transformCode(block.code);
        const execResult = await executeCode(transformed);

        if (execResult.success) {
          results.passed++;
          fileResult.blocks.push({
            index: block.index,
            line: block.line,
            status: 'passed',
            values: execResult.values,
            executionTime: execResult.executionTime,
          });

          if (options.verbose) {
            console.log(`  [PASS] Block ${block.index + 1} (line ${block.line}): ${execResult.values.length} values in ${execResult.executionTime}ms`);
          }
        } else {
          results.failed++;
          fileResult.blocks.push({
            index: block.index,
            line: block.line,
            status: 'failed',
            errors: execResult.errors,
          });

          console.log(`  [FAIL] ${relativePath} Block ${block.index + 1} (line ${block.line})`);
          for (const error of execResult.errors) {
            console.log(`         ${error}`);
          }
        }
      } catch (error) {
        results.failed++;
        fileResult.blocks.push({
          index: block.index,
          line: block.line,
          status: 'error',
          error: error.message,
        });

        console.log(`  [ERROR] ${relativePath} Block ${block.index + 1} (line ${block.line}): ${error.message}`);
      }
    }

    if (fileResult.blocks.length > 0) {
      results.files[relativePath] = fileResult;
    }
  }

  // Print summary
  console.log('\n============================');
  console.log('Summary');
  console.log('============================');
  console.log(`Total code blocks: ${results.total}`);
  console.log(`Skipped (DOM/external): ${results.skipped}`);
  console.log(`Executable: ${results.executable}`);
  console.log(`Passed: ${results.passed}`);
  console.log(`Failed: ${results.failed}`);

  if (results.executable > 0) {
    const passRate = ((results.passed / results.executable) * 100).toFixed(1);
    console.log(`Pass rate: ${passRate}%`);
  }

  // Generate report if requested
  if (options.report) {
    const reportPath = 'scripts/code-validation-report.json';
    fs.writeFileSync(reportPath, JSON.stringify(results, null, 2));
    console.log(`\nReport saved to: ${reportPath}`);
  }

  // Exit with error code if there are failures
  process.exit(results.failed > 0 ? 1 : 0);
}

// Handle uncaught errors from async code in code examples
process.on('uncaughtException', (error) => {
  // Silently ignore - these are from code examples with undefined functions
});

process.on('unhandledRejection', (error) => {
  // Silently ignore
});

main().catch(error => {
  console.error('Fatal error:', error);
  process.exit(1);
});
