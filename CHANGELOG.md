# Changelog

このファイルでは RxJS-with-TypeScript の歴代リリースとその主要な変更点を記録します。

形式は [Keep a Changelog](https://keepachangelog.com/) に準拠し、バージョン管理は [Semantic Versioning](https://semver.org/) 風に運用しています。

最新の進行中タスクは [`CLAUDE.md`](./CLAUDE.md) の "Active Development" および "Translation Workflow & i18n Maintenance" セクションを参照してください。

---

## [Fifth Release] - 2026-05 — 6 言語追加 + RxJS MCP 監査適用

### Purpose
6 言語 (fr/de/it/es/nl/pt) を追加し、Issue #32 (RxJS MCP 監査) で検出された問題を全 7 言語に展開。

### Status
✅ **COMPLETED** — 全 6 言語に Phase 1〜3 修正を適用済み

### Implementation Summary

#### Phase 1 (高優先度・error レベル修正)
- `shareReplay(N)` → `shareReplay({ bufferSize: N, refCount: true })`
- `mapTo(value)` → `map(() => value)`
- `retryWhen(...)` → `retry({ count, delay })` (RxJS 7.3+ 推奨形式)
- `takeUntil` の pipe 末尾移動

#### Phase 2 (catchError 型注釈一括強化)
- `catchError(error => ...)` → `catchError((error: unknown) => ...)` (398 箇所)

#### Phase 2.5 (error.message ガード保護)
- 各 catchError ブロック内の `error.message` をインラインガード `(error instanceof Error ? error.message : String(error))` に置換 (172 箇所)

#### Phase 3 (教材表現の調整)
- fromEvent サンプル箇所への production warning callout 追加
- アンチパターン記事への「意図的に悪い例」の callout 補強
- `retryWhen` 非推奨アナウンス callout

### Translation Statistics
- **Total files modified**: 337 ファイル (JA 原本含む)
- **Languages affected**: 7 (ja/en/fr/de/it/es/nl/pt)
- **修正コミット**:
  - Phase 1 JA: `f3ddcf5a`
  - Phase 2 JA: `049490a2`
  - Phase 3 JA: `3f38e94e`
  - JA→EN 同期: `046e0057`
  - 各言語別 commit (fr/de/it/es/nl/pt)

### Deployment
- Japanese: https://shuji-bonji.github.io/RxJS-with-TypeScript/
- English: https://shuji-bonji.github.io/RxJS-with-TypeScript/en/
- French: https://shuji-bonji.github.io/RxJS-with-TypeScript/fr/
- German: https://shuji-bonji.github.io/RxJS-with-TypeScript/de/
- Italian: https://shuji-bonji.github.io/RxJS-with-TypeScript/it/
- Spanish: https://shuji-bonji.github.io/RxJS-with-TypeScript/es/
- Dutch: https://shuji-bonji.github.io/RxJS-with-TypeScript/nl/
- Portuguese: https://shuji-bonji.github.io/RxJS-with-TypeScript/pt/

---

## [Fourth Release] - 2025-01 — Multi-language Support (en)

### Purpose
- Add English translation to make content accessible to international audience
- Maintain Japanese as primary language with English as secondary
- Establish scalable translation workflow for potential future languages

### Status
✅ **COMPLETED** - All 180 pages fully translated and deployed

### Implementation Summary

#### Directory Structure
```
docs/
├── .vitepress/
│   ├── config/
│   │   ├── index.ts        # Main configuration with root-level search
│   │   ├── ja.ts           # Japanese locale config
│   │   └── en.ts           # English locale config
├── guide/                   # Japanese content (root locale)
└── en/guide/                # English content (180 files)
```

#### Completed Phases
- **Phase 0**: Investigation & Design (base path, VitePress i18n, DeepL workflow, SEO with hreflang)
- **Phase 1**: Foundation Setup (modular config, hreflang, search)
- **Phase 2-5**: All 180 markdown files translated using DeepL MCP Server API

### Technical Implementation
1. **Translation Workflow**: DeepL MCP Server API (`targetLangCode: "en-US"`), section-by-section translation
2. **Search Configuration**: Root-level themeConfig with locale-specific translations, both Japanese and English search fully functional
3. **SEO Optimization**: hreflang tags configured in ja.ts and en.ts, all meta descriptions 150-160 characters
4. **Quality Assurance**: All builds successful, internal links verified, Mermaid diagrams render correctly

### Translation Statistics
- **Total files translated**: 180 markdown files
- **Total content**: ~50,000+ lines of documentation
- **Translation method**: DeepL MCP Server API
- **Code examples**: All TypeScript code comments translated
- **Diagrams**: All Mermaid diagrams translated
- **Meta descriptions**: 20 files optimized for SEO

### Deployment
- Japanese site: https://shuji-bonji.github.io/RxJS-with-TypeScript/
- English site: https://shuji-bonji.github.io/RxJS-with-TypeScript/en/

---

## [Third Release] - 2025 — New Chapters

### Chapter 3: Creation Functions - Full Expansion

#### Purpose
- Provide comprehensive, dedicated pages for all major Creation Functions
- Move detailed explanations from Chapter 2 to Chapter 3 where they belong
- Maintain Chapter 2 as a quick reference with links to Chapter 3 detailed pages

#### Final Status
- Chapter 3 fully expanded to include **7 categories** with comprehensive documentation
- Total of **28 pages** created (1 main index + 7 category indices + 20 detailed function pages)
- Chapter 2 (Observableの作成方法) now serves as quick reference with cross-links to Chapter 3 detailed pages

#### Completed Structure
```
3. Creation Functions (COMPLETED)
├── index.md (main index with comprehensive table)
├── 基本作成系 (6 pages: of, from, fromEvent, interval, timer)
├── ループ生成系 (3 pages: range, generate)
├── HTTP通信系 (3 pages: ajax, fromFetch) — Renamed from "変換系"
├── 結合系 (6 pages: concat, merge, combineLatest, zip, forkJoin)
├── 選択・分割系 (3 pages: race, partition)
├── 条件分岐系 (3 pages: iif, defer)
└── 制御系 (3 pages: scheduled, using)
```

#### Key Implementation Decisions
1. **Category Renaming**: "変換系" renamed to "HTTP通信系" (ajax/fromFetch are HTTP-specific)
2. **Function Placement**: `bindCallback` and `bindNodeCallback` kept in Chapter 2 only
3. **Cross-Referencing**: Chapter 2 (creation.md) updated with cross-links to all Chapter 3 detailed pages
4. **Documentation Consistency**: All pages follow Section 8 documentation enhancement guidelines
5. **Navigation Updates**: `docs/.vitepress/config.ts` sidebar fully updated

### Chapter 8: RxJSのデバッグ手法

✅ Structure created with following sections:
- デバッグの基本戦略 (tap, dev tools, RxJS DevTools)
- よくあるデバッグシナリオ
- デバッグツール (rxjs-spy, RxJS DevTools, custom operators)
- パフォーマンスデバッグ

### Chapter 11: RxJS困難点克服

✅ All 7 pages completed (Q1 2025):
- index.md (なぜRxJSは難しいのか)
- conceptual-understanding.md
- lifecycle-management.md
- operator-selection.md
- timing-and-order.md
- state-and-sharing.md
- stream-combination.md
- debugging-guide.md

### Chapter 13: 実践パターン集 (Completed November 2025)

#### Final Status
- Chapter 13 fully implemented with **9 comprehensive pages** (Japanese and English)
- All code examples follow the **immediately executable** pattern (dynamic DOM creation, no HTML setup required)
- Covers essential practical patterns from UI events to advanced form handling

#### Completed Structure
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

#### Key Implementation Features
1. **Immediately Executable Code Examples**: All DOM elements created dynamically, no pre-existing HTML markup required
2. **Coverage of Developer Focus Areas**: WebSocket integration, PWA application patterns, Web Components integration
3. **Advanced Patterns**: JSON Patch for large-scale form autosave and Undo/Redo, collaborative editing with operational transforms, offline-first architecture with IndexedDB
4. **Multi-language Support**: All 9 pages fully translated to English

---

## Earlier Releases

初期リリース（VitePress セットアップ、Chapter 1〜10 の基礎章、初期 Japanese ドキュメント）は git history を参照してください。

```bash
git log --oneline --reverse | head -30
```
