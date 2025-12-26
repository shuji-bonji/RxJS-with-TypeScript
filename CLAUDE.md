# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **multilingual educational documentation site** for learning RxJS with TypeScript, built with VitePress. It's a collaborative project between human engineers (@shuji-bonji) and AI (ChatGPT, Claude) aiming to be a model case for "human-AI co-created educational materials."

**Project Characteristics**:
- Educational material for TypeScript programmers learning RxJS
- Practical learning through code examples and tests
- Model case for human-AI co-created educational content
- VitePress-based static documentation site
- **Multilingual support**: Japanese (primary) and English

**Languages**:
- **Japanese (ja)**: Primary language at `/guide/` (root locale)
- **English (en)**: Secondary language at `/en/guide/`

**Key Technologies**:
- VitePress 1.6.3 (static site generator)
- TypeScript 5.x+
- Mermaid 11.6.0 (diagrams via vitepress-plugin-mermaid)
- markdown-it-footnote 4.0.0 (for footnotes)

**Developer Context**:
- Primary developer (@shuji-bonji) has experience with Angular, RxJS, TypeScript, and Jasmine testing
- Learning focus areas: WebSocket integration, PWA, Web Components

## Development Environment

**Requirements**:
- Node.js 20 or higher
- npm or yarn
- TypeScript 5.x or higher

**Dependency Management**:
```bash
# Check outdated dependencies
npm outdated

# Safe update
npm update

# Major version updates (use with caution)
npm install <package>@latest
```

## Development Commands

### Local Development
```bash
# Install dependencies
npm install

# Start dev server with hot reload
npm run docs:dev
# Starts VitePress dev server at http://localhost:5173
# Use this for live editing of documentation
```

### Building
```bash
npm run docs:build
# Builds the site for production
# Output: docs/.vitepress/dist/
```

### Preview Built Site
```bash
# Option 1: Preview with Vite server (recommended for pre-deploy checks)
npm run docs:preview

# Option 2: Serve static files (lightweight, faster, no hot reload)
npm run docs:serve
```

### Code Example Validation
```bash
# Validate all code examples in the guide
node scripts/validate-code-examples.cjs

# Validate specific directory
node scripts/validate-code-examples.cjs --dir docs/guide/operators/transformation

# Validate with verbose output
node scripts/validate-code-examples.cjs --verbose

# Generate JSON report (saved to scripts/code-validation-report.json)
node scripts/validate-code-examples.cjs --report
```

**Note:** The report file `scripts/code-validation-report.json` is gitignored as it's a generated artifact.

## Architecture

### Directory Structure

```
docs/
├── .vitepress/
│   ├── config/
│   │   ├── index.ts        # Main VitePress configuration
│   │   ├── ja.ts           # Japanese locale config
│   │   └── en.ts           # English locale config
│   ├── theme/
│   │   ├── index.ts        # Theme customization (uses default VitePress theme)
│   │   └── custom.css      # Custom styles
│   └── dist/               # Build output (generated)
├── index.md                # Homepage (Japanese)
├── guide/                  # Japanese documentation (root locale)
│   ├── introduction.md
│   ├── basics/             # RxJS fundamentals
│   ├── observables/        # Observable concepts
│   ├── subjects/           # Subject and multicasting
│   ├── operators/          # Operator categories (transformation, filtering, etc.)
│   ├── error-handling/     # Error handling strategies
│   ├── schedulers/         # Scheduler usage
│   ├── testing/            # Testing techniques
│   └── typescript-advanced/ # Advanced TypeScript integration
└── en/                     # English documentation
    ├── index.md            # Homepage (English)
    └── guide/              # Mirror structure of Japanese content
        ├── introduction.md
        ├── basics/
        ├── observables/
        └── ... (180 total files)
```

### Content Organization

Documentation follows a structured curriculum with 15 chapters (Chapters 12, 14, 15 are planned for future releases):

**✅ Completed Chapters (1-11, 13):**

1. **RxJS入門** (Introduction) - Basics and streams
2. **Observableの基礎** (Observable Basics) - Observable fundamentals, lifecycle, cold/hot
3. **Creation Functions** - Observable creation and combination functions across 7 categories:
   - 基本作成系 (Basic creation): of, from, fromEvent, interval, timer
   - ループ生成系 (Loop generation): range, generate
   - HTTP通信系 (HTTP communication): ajax, fromFetch
   - 結合系 (Combination): concat, merge, combineLatest, zip, forkJoin
   - 選択・分割系 (Selection/Partition): race, partition
   - 条件分岐系 (Conditional): iif, defer
   - 制御系 (Control): scheduled, using
4. **オペレーターの理解** (Operators) - Pipeable operators categorized by type:
   - Transformation operators (map, mergeMap, switchMap, etc.)
   - Filtering operators (filter, debounceTime, throttleTime, etc.)
   - Combination operators (concatWith, mergeWith, withLatestFrom, etc.)
   - Utility operators (tap, delay, retry, etc.)
   - Conditional operators (iif, defer, etc.)
   - Multicasting operators (share, shareReplay, etc.)
5. **Subjectとマルチキャスト** (Subject & Multicasting) - Subject types and usage
6. **エラーハンドリング** (Error Handling) - Error strategies, retry, catchError
7. **スケジューラーの活用** (Schedulers) - Async control and scheduler types
8. **RxJSのデバッグ手法** (Debugging) - Debugging strategies, common scenarios, and tools
9. **テスト手法** (Testing) - Unit tests, TestScheduler, marble testing
10. **RxJSアンチパターン集** (Anti-patterns) - Common mistakes and solutions
11. **RxJS困難点克服** (Overcoming Difficulties) - Conceptual barriers, lifecycle management, operator selection (7 pages)
13. **実践パターン集** (Practical Patterns) - Real-world implementation patterns (9 pages: ui-events, api-calls, form-handling, real-time-data, caching-strategies, error-handling-patterns, subscribe-branching, advanced-form-patterns)

**🔲 Planned Chapters (12, 14, 15):**

12. **TypeScriptとRxJSの高度な連携** (Advanced TypeScript Integration) - Type safety, generics, custom operators (Placeholder)
14. **パフォーマンス最適化** (Performance Optimization) - Subscription management, operator selection (Placeholder)
15. **フレームワークとの統合** (Framework Integration) - Angular, React, Vue, Web APIs (Placeholder)

Each operator/concept page typically includes practical use cases in a `practical-use-cases.md` file.

### Configuration Details

**VitePress Config** (modular structure in `docs/.vitepress/config/`):

**`config/index.ts`** (Main configuration):
- Uses `withMermaid()` wrapper for Mermaid diagram support
- Base path: `/RxJS-with-TypeScript/` (GitHub Pages deployment)
- Multi-language support with `locales` configuration:
  - Root locale: Japanese (ja) at `/`
  - English locale (en) at `/en/`
- Root-level `themeConfig.search` with locale-specific translations
- Configured with Open Graph and Twitter Card metadata
- Footer: CC-BY-4.0 license, Copyright 2025 shuji-bonji

**`config/ja.ts`** (Japanese locale):
- Japanese sidebar structure mirroring the curriculum
- Japanese nav links and labels
- SEO metadata with hreflang tags

**`config/en.ts`** (English locale):
- English sidebar structure (mirror of Japanese)
- English nav links and labels
- SEO metadata with hreflang tags

**Theme**: Uses default VitePress theme with minimal customization in `theme/index.ts` and `custom.css`.

**Markdown Extensions**:
- Footnotes via markdown-it-footnote
- Mermaid diagrams via vitepress-plugin-mermaid

## Deployment

Automated via GitHub Actions (`.github/workflows/deploy.yml`):
- Triggers on push to `main` branch or manual workflow dispatch
- Builds site with `npm run docs:build`
- Deploys to GitHub Pages using peaceiris/actions-gh-pages@v4
- Published to:
  - Japanese: https://shuji-bonji.github.io/RxJS-with-TypeScript/
  - English: https://shuji-bonji.github.io/RxJS-with-TypeScript/en/

## Content Guidelines

### 1. TypeScript-First Approach

**All code examples must be written in TypeScript**:
- Clearly specify type definitions, emphasize type safety
- Provide practical examples using generics and type inference
- Minimize use of `any` - maintain type safety

Example of good TypeScript code:
```typescript
import { Observable, map, filter } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
  isActive: boolean;
}

const users$: Observable<User[]> = getUsersFromAPI();

const activeUsers$ = users$.pipe(
  map(users => users.filter(user => user.isActive)),
  // Use type inference, add type annotations when necessary
  filter((users): users is User[] => users.length > 0)
);
```

### 2. RxJS Version and Latest Developments

**Current RxJS Status (as of October 2025)**:
- **Stable**: RxJS 7.8.2 (released February 2025)
- **In Development**: RxJS v8 (master branch)
- **Important Changes**:
  - Enhanced TypeScript support
  - Optimized bundle size
  - Improved performance
  - More intuitive API

**Import Method** (RxJS 7.2+):
```typescript
// Recommended import method - all from 'rxjs'
import { Observable, of, from, map, filter, catchError } from 'rxjs';

// DEPRECATED (RxJS < 7.2):
// import { Observable, of, from } from 'rxjs';
// import { map, filter, catchError } from 'rxjs/operators';
```

### 3. Angular Integration Considerations

Developer (@shuji-bonji) has Angular experience:
- Include Angular-specific implementation examples where appropriate
- **Framework-agnostic content should be the foundation**
- Appropriately explain comparisons with Angular-specific features (Signals, etc.)
- Angular 19+ integrates Signals and RxJS, allowing interoperability

### 4. Content Structure

Each section should include:
1. **Overview** - Concise explanation of concepts
2. **Basic Usage** - Simple TypeScript code
3. **Practical Examples** - Real-world application examples
4. **Test Code** - Test examples using Jasmine (leveraging developer's experience)
5. **Notes and Best Practices**
6. **Memory Leak Prevention** (where applicable)

### 5. Operator Guidelines

Pay special attention to these operator patterns:
- **Higher-order mapping operators** (mergeMap, switchMap, concatMap, exhaustMap) - usage differences
- **Backpressure control** (throttleTime, debounceTime)
- **Error handling patterns** (catchError, retry, retryWhen)

### 6. Writing Style

**Multi-language Documentation**:
- Primary content in Japanese (root locale at `/guide/`)
- English translations at `/en/guide/`
- Technical terms remain in English in both languages (Observable, Subject, etc.)
- Code examples use English comments in English documentation
- Code examples use appropriate mix of Japanese and English comments in Japanese documentation

**Japanese Documentation Style**:
- Technical terms remain in English (Observable, Subject, etc.)
- Explanations in clear Japanese
- Comments use appropriate mix of Japanese and English

**English Documentation Style**:
- Clear, accessible explanations for TypeScript developers
- Technical terms in English (Observable, Subject, etc.)
- All code comments in English
- Mermaid diagrams with English labels

**Visual Diagrams with Mermaid**:
Visualize complex concepts with diagrams. Example from Japanese documentation:
```mermaid
graph LR
    A[Observable] -->|subscribe| B[Observer]
    B -->|next| C[値の処理]
    B -->|error| D[エラー処理]
    B -->|complete| E[完了処理]
```

In English documentation, diagram text is translated to English.

### 7. Quality Assurance

**Code Verification**:
1. All code examples must be actually tested
2. TypeScript compiler type checking
3. Provide test code wherever possible

**Performance Considerations**:
- Explicitly show patterns to prevent memory leaks
- Proper unsubscription methods
- Efficient operator selection

### 8. Documentation Enhancement Guidelines

**Purpose**: Improve readability and understanding by adding contextual explanations to all documentation pages (140+ pages planned).

**Completed Examples**:
- `docs/guide/anti-patterns/flag-management.md` - Demonstrates comprehensive explanation structure
- `docs/guide/overcoming-difficulties/index.md` - Shows how to enhance index pages

#### 8.1. Article Introduction

Every article should start with a clear introduction that:
- Explains **what the article covers** and **why it matters**
- Provides context for the reader's current situation
- Previews the main topics to be discussed

**Example**:
```markdown
# 状態管理フラグの乱立

RxJSを導入したプロジェクトでも、コンポーネント内に大量のbooleanフラグが乱立する問題がよく見られます。この記事では、17個ものフラグが存在する実際の事例を元に、その原因と改善方法を解説します。
```

#### 8.2. Section Headings

Each major section (##) should include a 1-2 sentence explanation of:
- **What** the section discusses
- **Why** it's important
- **How** it relates to the overall topic

**Example**:
```markdown
## なぜフラグが乱立するのか

フラグが乱立する背景には、技術的な問題だけでなく、開発者の思考パターンや組織の進化過程が関係しています。以下、5つの主要な原因を分析します。
```

#### 8.3. Code Block Context

**Before Code Blocks**:
- Explain what the code demonstrates
- Highlight the key point readers should notice
- Provide context for when this pattern applies

**After Code Blocks**:
- Use VitePress callouts (`> [!TIP]`, `> [!WARNING]`, `> [!IMPORTANT]`) for key points
- Summarize the main takeaway
- List concrete benefits or problems

**Example**:
```markdown
以下の例で、問題のあるフラグと適切なフラグの違いを比較します。

[CODE BLOCK]

> [!WARNING] 問題点
> - 状態が「宣言的」でなく「手続き的」
> - 状態変更のタイミングが散在
> - フラグ間の整合性を開発者が手動保証
```

#### 8.4. Transition Sentences

Connect sections smoothly with transition sentences:
- Between major sections: Explain how topics relate
- Before lists/tables: State what the list contains and how to use it
- Before diagrams: Explain what the diagram illustrates

**Example**:
```markdown
### 改善戦略

フラグ乱立問題を解決するには、以下の3ステップで段階的にリファクタリングを進めます。

### Step 1: 状態の棚卸し

まず、現在のフラグをすべて列挙し、責務ごとに分類します。これにより、どのフラグが統合可能かが見えてきます。
```

#### 8.5. Practical Examples

When showing Before/After comparisons:
- **Before**: Explain the problem concretely
- **After**: Highlight what improved and why
- **Comparison**: Use tables or side-by-side format when helpful

#### 8.6. Conclusion Sections

Every article should end with:
1. **Summary** - Key points recap
2. **Important Principles** - Core takeaways (using callouts)
3. **Related Sections** - Links with brief descriptions
4. **References** - External resources with context

**Example**:
```markdown
## 参考リソース

RxJSの公式ドキュメントや学習リソースで、さらに深く学習できます。

- [RxJS公式ドキュメント](https://rxjs.dev/) - 公式のAPIリファレンスとガイド
- [Learn RxJS](https://www.learnrxjs.io/) - オペレーター別の実践的な例
- [RxJS Marbles](https://rxmarbles.com/) - ビジュアルでオペレーターの動作を理解
```

#### 8.7. Enhancement Checklist

When enhancing any documentation page, verify:
- [ ] Article has clear introduction (what, why, how)
- [ ] Each section has contextual explanation
- [ ] Code blocks have before/after context
- [ ] VitePress callouts used for key points (`> [!TIP]`, `> [!WARNING]`, etc.)
- [ ] Transitions between sections are smooth
- [ ] Tables/lists have introductory sentences
- [ ] Diagrams have explanatory text
- [ ] Conclusion summarizes key points
- [ ] Related links include descriptions
- [ ] Language is accessible to target audience (TypeScript developers learning RxJS)

#### 8.8. Implementation Process

For each page:
1. **Read through** - Understand the existing content
2. **Identify gaps** - Note sections lacking context
3. **Add introductions** - Start with article and section intros
4. **Contextualize code** - Add before/after explanations
5. **Enhance transitions** - Connect sections smoothly
6. **Verify build** - Run `npm run docs:build` to test
7. **Review readability** - Ensure natural flow

**Progress Tracking** (as of 2025-10-20):
- ✅ `docs/guide/anti-patterns/flag-management.md` - Complete
- ✅ `docs/guide/overcoming-difficulties/index.md` - Complete
- 🔲 Remaining: ~140 pages to enhance

#### 8.9. Writing Tips

- **Use concrete examples** - "17個のフラグ" is better than "たくさんのフラグ"
- **Explain the "why"** - Don't just show code, explain reasoning
- **Progressive disclosure** - Start simple, then add complexity
- **Reader empathy** - Anticipate confusion points
- **Consistent terminology** - Use the same terms throughout

### 9. General Guidelines

When editing documentation:
- **Multi-language content**: Maintain both Japanese (root locale) and English (`/en/`) versions
- When editing Japanese content, consider updating English translation if changes are substantial
- When adding new content, create both Japanese and English versions
- Maintain the established curriculum structure across both languages
- Each operator/concept page should include code examples with TypeScript
- Use Mermaid diagrams where helpful for visualizing streams (translate diagram text for English version)
- Include practical use cases for each operator category
- Follow the collaborative human-AI creation approach
- Respect the CC-BY-4.0 license for content
- Reference official RxJS and TypeScript documentation (both Apache 2.0 licensed)

## Content Priorities and TODO

### 🔴 High Priority

1. **Chapter 14: パフォーマンス最適化** (Unimplemented)
   - Subscription lifecycle management and memory leak prevention
   - Operator selection for performance optimization
   - Stream architecture patterns for scalability

2. **Chapter 15: フレームワークとの統合** (Unimplemented)
   - Angular, React, Vue framework integrations
   - State management patterns (NgRX, Signals, Redux Toolkit)
   - Web API integration (WebSocket, SSE, IndexedDB)

3. **RxJS v8 Migration Guide** (Wait for stable release)
   - Research and document new features
   - Create migration guide
   - Update all code examples

### 🟡 Medium Priority

1. **Chapter 12: TypeScriptとRxJSの高度な連携** (Unimplemented)
   - Advanced type safety patterns
   - Custom operator creation with proper typing
   - Conditional types and mapped types

2. **TestScheduler Advanced Guide** (Chapter 9 expansion)
   - Comprehensive TestScheduler techniques
   - Real-world testing scenarios
   - Cross-reference with Chapter 13 patterns

3. **Observable と Signal 統合** (Deep Dive)
   - Angular Signals interoperability (toSignal/toObservable)
   - Framework-agnostic Signal concepts
   - Performance benchmarks and trade-offs

### 🟢 Low Priority

1. Add interactive code examples (CodeSandbox/StackBlitz embedding)
2. Consider video tutorials
3. Add practice exercises
4. Community contribution guidelines
5. JSON Patch Examples Repository (optional supporting infrastructure)

---

## Completed Releases

### ✅ Third Release: New Chapters (Completed)

The following new chapters have been added or are planned for future releases.

#### ✅ Chapter 3: Creation Functions (Added in restructuring)

**Purpose:**
- Clearly separate creation functions from pipeable operators
- Teach Observable creation and combination before manipulation
- Explain the relationship between creation functions (concat, merge) and pipeable operators (concatWith, mergeWith)

**Placement:** After Observable basics, before Pipeable Operators
- Natural progression: Basic creation (of, from) → **Advanced creation/combination** → Manipulation (operators) → Sharing (Subject)

**Structure:**
```
3. Creation Functions
├── Creation Functions とは
│   ├── Pipeable Operator との違い
│   └── 使い分けの基準
├── 基本的な Creation Functions（2章の復習）
│   └── of, from, interval, timer
├── 結合系 Creation Functions
│   ├── concat - 順次結合
│   ├── merge - 並行結合
│   ├── combineLatest - 最新値の組み合わせ
│   ├── zip - 対応する値のペア化
│   ├── race - 最速のストリームを採用
│   ├── forkJoin - すべての完了を待つ
│   └── partition - 条件で分割
└── Pipeable Operator との対応関係
```

#### ✅ Chapter 8: RxJSのデバッグ手法 (Structure created, content TBD)

**Why Critical:**
- RxJS debugging is one of the most challenging aspects for learners
- Essential skill for real-world development
- Complements existing testing and anti-patterns chapters

**Proposed Structure:**
```
8. RxJSのデバッグ手法
├── デバッグの基本戦略
│   ├── tap オペレーターでのログ出力
│   ├── 開発者ツールでの確認
│   └── RxJS DevTools の活用
├── よくあるデバッグシナリオ
│   ├── 値が流れてこない
│   ├── 期待と異なる値が出力される
│   ├── 購読が完了しない（無限ストリーム）
│   ├── メモリリーク（購読解除忘れ）
│   └── エラーが発生しているのに気づかない
├── デバッグツール
│   ├── rxjs-spy の使い方
│   ├── RxJS DevTools（ブラウザ拡張）
│   └── カスタムデバッグオペレーターの作成
└── パフォーマンスデバッグ
    ├── 購読数の確認
    ├── 不要な再評価の検出
    └── メモリ使用量の監視
```

**Placement:** Between Chapter 7 (Schedulers) and Chapter 9 (Testing)
- Natural progression: Implementation → Error Handling → Schedulers → **Debugging** → Testing → Anti-patterns

#### ✅ Chapter 11: RxJS困難点克服 (Completed)

**Purpose:**
- Address common difficulties that experienced developers face when working with RxJS
- Provide actionable guidance to overcome RxJS-specific conceptual and practical barriers
- Bridge the gap between theory (Chapters 1-10) and practice (Chapter 13)

**Structure:**
```
11. RxJS困難点克服
├── index.md                        # なぜRxJSは難しいのか（経験者でも）
├── conceptual-understanding.md     # 概念理解の壁
├── lifecycle-management.md         # ライフサイクル管理の壁
├── operator-selection.md           # オペレーター選択の迷い
├── timing-and-order.md             # タイミングと順序の理解
├── state-and-sharing.md            # 状態管理の難しさ
├── stream-combination.md           # 複数ストリーム組み合わせ
└── debugging-guide.md              # デバッグの壁
```

**Implementation Status:** ✅ All 7 pages completed (Q1 2025)

#### ✅ Chapter 3: Creation Functions - Full Expansion (Completed Q4 2025)

**Purpose:**
- Provide comprehensive, dedicated pages for all major Creation Functions
- Move detailed explanations from Chapter 2 to Chapter 3 where they belong
- Maintain Chapter 2 as a quick reference with links to Chapter 3 detailed pages
- Complete the Creation Functions documentation with consistent depth across all functions

**Final Status:**
- Chapter 3 has been fully expanded to include **7 categories** with comprehensive documentation
- Total of **28 pages** created (1 main index + 7 category indices + 20 detailed function pages)
- Chapter 2 (Observableの作成方法) now serves as quick reference with cross-links to Chapter 3 detailed pages
- All Creation Functions now have consistent documentation depth

**Completed Structure:**

```
3. Creation Functions (COMPLETED)
├── index.md (main index with comprehensive table)
├── 基本作成系 (6 pages)
│   ├── index.md
│   ├── of.md - 指定した値を順番に発行
│   ├── from.md - 配列、Promise等から変換
│   ├── fromEvent.md - イベントをObservableに変換
│   ├── interval.md - 指定間隔で連続発行
│   └── timer.md - 遅延後に発行開始
├── ループ生成系 (3 pages)
│   ├── index.md
│   ├── range.md - 数値の範囲を生成
│   └── generate.md - ループ的な生成（for文のような動作）
├── HTTP通信系 (3 pages) - Renamed from "変換系"
│   ├── index.md
│   ├── ajax.md - Ajax/HTTPリクエスト
│   └── fromFetch.md - Fetch APIのラッパー
├── 結合系 (6 pages) - Previously existing
│   ├── index.md
│   ├── concat.md, merge.md, combineLatest.md
│   ├── zip.md, forkJoin.md
├── 選択・分割系 (3 pages) - Previously existing
│   ├── index.md
│   ├── race.md, partition.md
├── 条件分岐系 (3 pages) - Previously existing
│   ├── index.md
│   ├── iif.md, defer.md
└── 制御系 (3 pages)
    ├── index.md
    ├── scheduled.md - スケジューラーを指定してObservableを生成
    └── using.md - リソース制御付きObservable
```

**Total Pages Created:** 28 pages across 7 categories

**Key Implementation Decisions:**

1. **Category Renaming:**
   - "変換系" renamed to "HTTP通信系" for accuracy
   - Reason: ajax/fromFetch are HTTP-specific, not general conversion functions

2. **Function Placement:**
   - `bindCallback` and `bindNodeCallback` kept in Chapter 2 only
   - Reason: Not classified as Creation operators in Learn RxJS documentation
   - Chapter 2 provides detailed coverage, avoiding duplication

3. **Cross-Referencing:**
   - Chapter 2 (creation.md) updated with cross-links to all Chapter 3 detailed pages
   - Chapter 2 maintained as quick reference guide
   - All functions accessible from both chapters

4. **Documentation Consistency:**
   - All pages follow Section 8 documentation enhancement guidelines
   - Each page includes: introduction, TypeScript examples, practical use cases, marble diagrams
   - Emoji removed from level 1-3 headings for consistency
   - Main index page uses comprehensive table format with clickable links

5. **Navigation Updates:**
   - `docs/.vitepress/config.ts` sidebar fully updated
   - `docs/guide/index.md` updated with all 7 categories
   - Dead links fixed (error-handling, schedulers pages)

**Completed Implementation Steps:**

- ✅ Created directory structure (basic, loop, http-communication, control)
- ✅ Created 7 category index pages with overviews and comparison tables
- ✅ Created 20 detailed function pages with comprehensive examples
- ✅ Updated Chapter 2 with cross-links to Chapter 3
- ✅ Updated main Creation Functions index with table format
- ✅ Updated navigation (config.ts, guide/index.md)
- ✅ Removed emoji from level 1-3 headings across all pages
- ✅ Fixed dead links and verified build
- ✅ Updated terminology (removed "新しい" from Creation Functions description)

**Impact:**
- Complete Creation Functions documentation with 7 categories
- Improved learning experience with dedicated, comprehensive pages
- Clear separation between quick reference (Chapter 2) and detailed guides (Chapter 3)
- Consistent documentation style across all 28 pages

---

#### ✅ Chapter 13: 実践パターン集 (Completed November 2025)

**Purpose:**
- Provide real-world implementation patterns for common use cases
- Show how to apply RxJS knowledge in practical scenarios
- Address the gap between "knowing operators" and "building features"

**Final Status:**
- Chapter 13 has been fully implemented with **9 comprehensive pages** (Japanese and English)
- All code examples follow the **immediately executable** pattern (dynamic DOM creation, no HTML setup required)
- Covers essential practical patterns from UI events to advanced form handling

**Completed Structure:**

```
13. 実践パターン集 (COMPLETED - 9 pages)
├── index.md - 実践パターンの概要
├── ui-events.md - UIイベント処理パターン
├── api-calls.md - API呼び出しパターン
├── form-handling.md - フォーム処理パターン
├── real-time-data.md - リアルタイムデータ処理 (WebSocket, SSE, Polling)
├── caching-strategies.md - キャッシュ戦略
├── error-handling-patterns.md - エラーハンドリング実践パターン
├── subscribe-branching.md - subscribe内の条件分岐パターン
└── advanced-form-patterns.md - JSON Patch高度なフォームパターン
```

**Key Implementation Features:**

1. **Immediately Executable Code Examples:**
   - All DOM elements created dynamically with `document.createElement()`
   - No pre-existing HTML markup required
   - Examples work in browser console, CodeSandbox, StackBlitz immediately
   - Traditional `querySelector` approach shown in comments for educational purposes

2. **Coverage of Developer Focus Areas:**
   - WebSocket integration (`real-time-data.md`)
   - PWA application patterns (`caching-strategies.md`)
   - Web Components integration (`ui-events.md`)

3. **Advanced Patterns:**
   - JSON Patch for large-scale form autosave and Undo/Redo
   - Collaborative editing with operational transforms
   - Offline-first architecture with IndexedDB

4. **Multi-language Support:**
   - All 9 pages fully translated to English
   - Code comments translated appropriately
   - Consistent documentation quality across both languages

**Completed Implementation:**

- ✅ All 9 pages created with comprehensive content
- ✅ Code examples follow executable pattern
- ✅ English translations completed
- ✅ Cross-references to Chapters 6, 10, 11
- ✅ Build verified without errors
- ✅ Sidebar navigation updated

**Developer Impact:**
- Immediate practical value for TypeScript developers
- Bridge between theoretical knowledge and real-world implementation
- Demonstrates best practices with type safety and memory leak prevention

---

## Completed Releases

### ✅ Fourth Release: Multi-language Support (Completed January 2025)

**Purpose:**
- Add English translation to make content accessible to international audience
- Maintain Japanese as primary language with English as secondary
- Establish scalable translation workflow for potential future languages

**Status:** ✅ **COMPLETED** - All 180 pages fully translated and deployed

**Implementation Summary:**

**Directory Structure:**
```
docs/
├── .vitepress/
│   ├── config/
│   │   ├── index.ts        # Main configuration with root-level search
│   │   ├── ja.ts           # Japanese locale config
│   │   └── en.ts           # English locale config
├── guide/                   # Japanese content (root locale)
│   ├── introduction.md
│   ├── observables/
│   └── ... (180 files)
├── en/                      # English content
│   ├── guide/
│   │   ├── introduction.md
│   │   ├── observables/
│   │   └── ... (180 files)
├── public/
└── index.md
```

**Completed Implementation:**

**Phase 0: Investigation & Design** ✅
- ✅ Base path handling (`/RxJS-with-TypeScript/` + i18n) verified
- ✅ VitePress native i18n support implemented
- ✅ DeepL MCP Server API workflow established
- ✅ SEO requirements evaluated (hreflang tags, meta descriptions)
- ✅ VitePress local search selected (built-in multi-language support)
- ✅ Language switcher using VitePress native UI

**Phase 1: Foundation Setup** ✅
- ✅ Directory structure created (`docs/.vitepress/config/`, `docs/en/guide/`)
- ✅ config.ts split into modular structure (index.ts, ja.ts, en.ts)
- ✅ VitePress locales configured with base path
- ✅ Local search setup with multi-language support (root themeConfig)
- ✅ hreflang tags added for SEO
- ✅ Build verified and routing tested

**Phase 2-5: Complete Translation** ✅
- ✅ All 180 markdown files translated using DeepL MCP Server API
- ✅ Code comments and console.log messages translated
- ✅ Mermaid diagrams translated (all text elements)
- ✅ Internal links updated (`/guide/` → `/en/guide/`)
- ✅ URL anchors translated to English
- ✅ Meta descriptions optimized (150-160 chars per Bing Webmaster Tools)
- ✅ All sections completed:
  - Introduction, Basics, Observables
  - Creation Functions (7 categories, 28 pages)
  - Operators (Transformation, Filtering, Combination, Utility, etc.)
  - Subjects and Multicasting
  - Error Handling
  - Schedulers
  - Debugging Techniques
  - Testing Methods
  - Anti-patterns
  - Overcoming Difficulties
  - Appendix (Reactive Architecture, Ecosystem, etc.)

**Technical Implementation:**

1. **Translation Workflow**
   - Used DeepL MCP Server API (`targetLangCode: "en-US"`)
   - Systematic section-by-section translation (100-200 lines)
   - Verification with grep for Japanese characters
   - Final check: Zero Japanese characters in English files

2. **Search Configuration**
   - Root-level themeConfig with locale-specific translations
   - Fixed search button rendering issue
   - Both Japanese and English search fully functional

3. **SEO Optimization**
   - hreflang tags configured in ja.ts and en.ts
   - All meta descriptions 150-160 characters
   - Proper frontmatter with quoted descriptions containing colons

4. **Quality Assurance**
   - All builds successful without errors
   - Internal links verified
   - Mermaid diagrams render correctly
   - Language switcher navigates properly

**Success Metrics Achieved:**
- ✅ English pages build without errors
- ✅ All internal links work correctly
- ✅ Search works in both languages
- ✅ Language switcher navigates correctly
- ✅ SEO tags properly configured
- ✅ Mermaid diagrams render in both languages
- ✅ Zero Japanese characters remaining in English version

**Translation Statistics:**
- **Total files translated:** 180 markdown files
- **Total content:** ~50,000+ lines of documentation
- **Translation method:** DeepL MCP Server API
- **Code examples:** All TypeScript code comments translated
- **Diagrams:** All Mermaid diagrams translated
- **Meta descriptions:** 20 files optimized for SEO

**Deployment:**
- Japanese site: https://shuji-bonji.github.io/RxJS-with-TypeScript/
- English site: https://shuji-bonji.github.io/RxJS-with-TypeScript/en/

---

## Active Development

### 🔴 High Priority: Remaining Chapters

The following three chapters are defined in the sidebar configuration but remain unimplemented (all items are currently commented out):

#### Chapter 12: TypeScriptとRxJSの高度な連携

**Planned Structure** (from `docs/.vitepress/config/ja.ts`):
```typescript
items: [
  { text: 'TypeScriptとRxJSの基本連携', link: '/guide/typescript-advanced/type-safety' },
  { text: 'ジェネリクスの活用', link: '/guide/typescript-advanced/generics' },
  { text: 'カスタムオペレーターと型定義', link: '/guide/typescript-advanced/custom-operators' },
  { text: '条件型とマッピング型の活用', link: '/guide/typescript-advanced/conditional-types' },
]
```

**Purpose:**
- Deep dive into TypeScript integration with RxJS
- Advanced type safety patterns and techniques
- Custom operator creation with proper typing
- Leverage conditional types and mapped types for better type inference

**Implementation Priority:** Medium (foundation for advanced users)

---

#### Chapter 14: パフォーマンス最適化

**Planned Structure** (from `docs/.vitepress/config/ja.ts`):
```typescript
items: [
  { text: '購読の適切な管理', link: '/guide/performance/subscription-management' },
  { text: '効率的なオペレーター選択', link: '/guide/performance/operator-selection' },
  { text: 'ストリームの設計パターン', link: '/guide/performance/stream-design' },
]
```

**Purpose:**
- Subscription lifecycle management and memory leak prevention
- Operator selection for performance optimization
- Stream architecture patterns for scalability

**Implementation Priority:** High (critical for production applications)

---

#### Chapter 15: フレームワークとの統合

**Planned Structure** (from `docs/.vitepress/config/ja.ts`):
```typescript
items: [
  { text: 'Angularとの連携', link: '/guide/frameworks/angular' },
  { text: 'Reactとの連携', link: '/guide/frameworks/react' },
  { text: 'Vueとの連携', link: '/guide/frameworks/vue' },
  { text: 'Web APIとの連携', link: '/guide/frameworks/web-api' },
]
```

**Purpose:**
- Framework-specific RxJS integration patterns
- Angular: NgRX, Signals interoperability (Angular 19+)
- React: React hooks integration, state management
- Vue: Composition API integration
- Web APIs: WebSocket, Server-Sent Events, IndexedDB

**Phased Approach:**

**Phase 1: Basic Framework Integration**
- Quick start guides for each framework (5-10 min read)
- Focus on basic RxJS usage patterns
- Heavy use of external documentation links

**Phase 2: State Management Integration** ⭐ Main Enhancement
- RxJS + NgRX (Angular) - leverage developer's Angular experience
- RxJS + Signals (Angular 19+) - toSignal/toObservable patterns
- RxJS + Redux Toolkit (React)
- RxJS + Zustand, Jotai (lightweight state management)

**Phase 3: Web API Integration**
- WebSocket (developer's focus area)
- Server-Sent Events
- IndexedDB for offline-first applications

**Implementation Priority:** High (connects RxJS to real-world frameworks)

**Considerations:**
- ⚠️ Maintain framework-agnostic foundation
- ⚠️ Avoid scope creep (don't become a state management tutorial)
- ⚠️ Maintenance overhead for framework updates
- ✅ Provides unique value vs other RxJS resources
- ✅ Reflects 2025 ecosystem trends (Signals, Runes)

---

### 🟡 Optional: Supporting Infrastructure

#### JSON Patch Examples Repository (Optional)

> **Note:** This is an optional supporting infrastructure for Chapter 13's `advanced-form-patterns.md` page, which already contains comprehensive documentation on JSON Patch + RxJS patterns.

**Purpose:**
Provide executable code examples for JSON Patch patterns since no public APIs support JSON Patch operations.

**Background:**
- No public testing APIs (like JSONPlaceholder) support JSON Patch (RFC 6902)
- Chapter 13's `advanced-form-patterns.md` already contains comprehensive JSON Patch documentation
- This repository would provide hands-on, runnable examples to complement the documentation

**Proposed Solution:**

**Phase 1: Embedded Demos** (StackBlitz/CodeSandbox with MSW)
- Large Form Autosave demo with JSON Patch generation, auto-save, Undo/Redo
- Collaborative Editing demo with Yjs integration
- Embedded in `advanced-form-patterns.md` page

**Phase 2: Starter Kit Repository** (`https://github.com/shuji-bonji/rxjs-json-patch-examples`)
- 4 runnable examples (Basic Patch, Large Form, Collaborative Editing, Offline Queue)
- Frontend (RxJS + TypeScript + fast-json-patch + MSW) + Backend (Express/Hono)
- Docker Compose setup for easy local development
- Comprehensive README with quick start guide

**Phase 3: Public API** (Optional, long-term)
- Live backend API deployment (Vercel/Railway)
- Not priority - Phase 1 + 2 provide sufficient learning environment

**Decision:** Optional infrastructure. Chapter 13's documentation is already comprehensive. This repository would be a "nice-to-have" for hands-on experimentation but not essential for learning

---

## Future Enhancements

The following enhancements are planned for future releases to keep the documentation current with the latest RxJS developments and ecosystem trends.

#### 🔴 High Priority: RxJS v8 Full Migration

**Purpose:**
- Provide comprehensive migration guide from RxJS v7 to v8
- Document all breaking changes and new features
- Update all code examples to v8 best practices

**Structure:**
```
RxJS v8 Migration Guide
├── Breaking Changes Overview
│   ├── Removed operators and their replacements
│   ├── API signature changes
│   └── Import path updates
├── New Features and Improvements
│   ├── Performance enhancements
│   ├── Bundle size optimizations
│   └── New operator additions
├── Migration Strategy
│   ├── Step-by-step migration process
│   ├── Automated migration tools
│   └── Testing migration results
└── Updated Code Examples
    ├── Before/After comparisons
    └── Best practices for v8
```

**Implementation Tasks:**
1. Monitor RxJS v8 stable release
2. Review official migration guide
3. Update all existing documentation examples
4. Create dedicated migration guide page
5. Add version badges throughout documentation

**Timeline:** Upon RxJS v8 stable release

---

#### 🔴 High Priority: Observable と Signal の統合 (Deep Dive)

**Purpose:**
- Provide comprehensive guide on Observable-Signal interoperability
- Show practical patterns for modern reactive applications
- Cover both Angular Signals and framework-agnostic approaches

**Structure:**
```
Observable と Signal の統合
├── Angular Signals との統合 (詳細版)
│   ├── toSignal / toObservable の詳細
│   ├── Signal based Inputs と RxJS
│   ├── computedとObservableの組み合わせ
│   ├── effectとObservableの連携
│   └── 実践例：リアクティブフォームの最新パターン
├── 他のフレームワークのSignals
│   ├── Solid.js Signals
│   ├── Preact Signals
│   └── Vue 3 Composition API
├── Observable vs Signal の使い分け
│   ├── それぞれの強み・弱み
│   ├── パフォーマンス比較
│   └── 選択のガイドライン
└── 実践パターン集
    ├── フォーム処理
    ├── 状態管理
    └── 非同期データフロー
```

**Implementation Tasks:**
1. Expand Chapter 14.2 content significantly
2. Add standalone chapter option if content grows large
3. Include framework-agnostic Signal concepts
4. Provide migration patterns from pure RxJS to Signal-hybrid approaches
5. Add performance benchmarks and trade-off analysis

**Placement Options:**
- **Option 1**: Expand Chapter 14.2 (State Management Integration)
- **Option 2**: Create standalone Chapter 12.5 (Advanced TypeScript Integration subsection)

**Timeline:** Q2-Q3 2025

---

#### 🟡 Medium Priority: TestScheduler 活用ガイド (拡張)

**Purpose:**
- Expand Chapter 9 testing section with comprehensive TestScheduler guide
- Provide advanced marble testing techniques
- Show real-world testing scenarios

**Structure:**
```
TestScheduler 活用ガイド（拡張）
├── TestScheduler の基礎（復習）
│   ├── 基本的な使い方
│   └── Marble Diagram 記法
├── 高度な TestScheduler テクニック
│   ├── 複雑な時間制御のテスト
│   ├── Higher-order Observable のテスト
│   ├── エラーハンドリングのテスト
│   └── リトライ・タイムアウトのテスト
├── 実践的なテストパターン
│   ├── API呼び出しのテスト
│   ├── フォーム処理のテスト
│   ├── WebSocket 通信のテスト
│   └── キャッシュ戦略のテスト
├── デバッグとトラブルシューティング
│   ├── よくあるテストの失敗原因
│   ├── Marble Diagram のデバッグ方法
│   └── 非同期処理のデバッグ
└── CI/CD との統合
    ├── テスト環境のセットアップ
    ├── カバレッジ測定
    └── パフォーマンステスト
```

**Implementation Tasks:**
1. Expand existing Chapter 9 testing content
2. Add dedicated TestScheduler subsection (Chapter 9.3)
3. Provide downloadable test code examples
4. Include video tutorials for complex scenarios
5. Cross-reference with Chapter 13 practical patterns

**Placement:** Chapter 9.3 (new subsection within existing Testing chapter)

**Timeline:** Q3-Q4 2025

---

**Priority Summary:**
1. **RxJS v8 Full Migration** - Critical for staying current (wait for stable release)
2. **Observable と Signal の統合** - High demand, reflects 2025 ecosystem trends
3. **TestScheduler 活用ガイド** - Important for production quality, less urgent

**Dependencies:**
- **RxJS v8 Migration**: Depends on official v8 stable release
- **Observable-Signal integration**: Can start now with Angular 19+ examples
- **TestScheduler guide**: Can start anytime, complements existing content

## Reference Resources

- [RxJS Official Documentation](https://rxjs.dev)
- [RxJS GitHub](https://github.com/ReactiveX/rxjs)
- [TypeScript Official Documentation](https://www.typescriptlang.org)
- [Learn RxJS](https://www.learnrxjs.io/)
- [RxJS Marbles](https://rxmarbles.com/)

**Track Latest RxJS Information**:
1. Monitor official repository for release notes and breaking changes
2. Follow community trends - RxJS v8 new features, TypeScript integration
3. Update dependencies regularly (see Development Environment section)

## Important Notes

1. **Never sacrifice type safety** - Minimize use of `any`
2. **Thorough subscription management** - Proper use of unsubscribe or takeUntil
3. **Follow latest developments** - RxJS v8, TypeScript 5.x new features
4. **Be practical** - Emphasize real-world examples, not just theory
5. **Prevent memory leaks** - Always include proper cleanup patterns

## Collaboration Guidelines

**Working with AI Assistants**:
- Prioritize technical accuracy
- Provide practical, working code examples
- Reflect latest best practices
- Create content leveraging developer's experience (Angular, RxJS, TypeScript)

**Content Review Perspectives**:
1. **Technical Accuracy** - Compliant with specifications
2. **Practicality** - Usable in real work
3. **Clarity** - Understandable for beginners
4. **Currency** - Compliant with latest RxJS/TypeScript specifications
5. **Completeness** - Includes important notes like memory leak prevention

## License

Content: CC-BY-4.0 (Creative Commons Attribution 4.0 International)
