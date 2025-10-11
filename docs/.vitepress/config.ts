// import { defineConfig } from 'vitepress';
import footnote from 'markdown-it-footnote';
import { withMermaid } from 'vitepress-plugin-mermaid';

// export default defineConfig({
export default withMermaid({
  title: 'RxJS with TypeScript',
  description: 'TypeScriptプログラマのためのRxJS入門',
  base: '/RxJS-with-TypeScript/',
  lang: 'ja',
  head: [
    // Open Graph
    ['meta', { property: 'og:title', content: 'RxJS with TypeScript' }],
    ['meta', { property: 'og:description', content: 'TypeScript で RxJS' }],
    [
      'meta',
      {
        property: 'og:image',
        content:
          'https://shuji-bonji.github.io/RxJS-with-TypeScript/images/ts-de-rxjs.webp',
      },
    ],
    [
      'meta',
      {
        property: 'og:url',
        content: 'https://shuji-bonji.github.io/RxJS-with-TypeScript/',
      },
    ],
    // Twitter Card
    ['meta', { name: 'twitter:card', content: 'summary_large_image' }],
    ['meta', { name: 'twitter:title', content: 'RxJS with TypeScript' }],
    ['meta', { name: 'twitter:description', content: 'TypeScript で RxJS' }],
    [
      'meta',
      {
        name: 'twitter:image',
        content:
          'https://shuji-bonji.github.io/RxJS-with-TypeScript/images/ts-de-rxjs.png',
      },
    ],
  ],
  themeConfig: {
    nav: [
      { text: 'ホーム', link: '/' },
      { text: 'ガイド', link: '/guide/' },
    ],
    sidebar: [
      {
        text: '1. RxJS入門',
        items: [
          { text: 'はじめに', link: '/guide/introduction' },
          { text: '学習用の実行環境構築', link: '/guide/starter-kid.md' },
          { text: 'RxJSとは何か', link: '/guide/basics/what-is-rxjs' },
          { text: 'ストリームとは？', link: '/guide/basics/what-is-a-stream' },
        ],
      },
      {
        text: '2. Observableの基礎',
        items: [
          {
            text: 'Observableとは',
            link: '/guide/observables/what-is-observable',
          },
          { text: 'Observableの作成方法', link: '/guide/observables/creation' },
          { text: 'イベントのストリーム化', link: '/guide/observables/events' },
          {
            text: 'fromEvent で利用できないイベント',
            link: '/guide/observables/events#cannot-used-fromEvent'
          },
          { text: 'イベント一覧', link: '/guide/observables/events-list' },
          {
            text: 'Observableのライフサイクル',
            link: '/guide/observables/observable-lifecycle',
          },
          {
            text: 'コールドObservableとホットObservable',
            link: '/guide/observables/cold-and-hot-observables',
          },
        ],
      },
      {
        text: '3. Creation Functions',
        link: '/guide/creation-functions/',
        collapsed: true,
        items: [
          { text: 'concat', link: '/guide/operators/combination/concat' },
          { text: 'merge', link: '/guide/operators/combination/merge' },
          { text: 'combineLatest', link: '/guide/operators/combination/combineLatest' },
          { text: 'zip', link: '/guide/operators/combination/zip' },
          { text: 'race', link: '/guide/operators/combination/race' },
          { text: 'forkJoin', link: '/guide/operators/combination/forkJoin' },
        ],
      },
      {
        text: '4. オペレーターの理解',
        link: '/guide/operators/',
        items: [
          { text: 'パイプラインの概念', link: '/guide/operators/pipeline' },
          {
            text: '変換オペレーター',
            link: '/guide/operators/transformation/',
            collapsed: true,
            items: [
              { text: 'map', link: '/guide/operators/transformation/map' },
              { text: 'scan', link: '/guide/operators/transformation/scan' },
              { text: 'reduce', link: '/guide/operators/transformation/reduce' },
              { text: 'pairwise', link: '/guide/operators/transformation/pairwise' },
              { text: 'groupBy', link: '/guide/operators/transformation/groupBy' },
              { text: 'pluck（非推奨）', link: '/guide/operators/transformation/pluck' },
              { text: 'mapTo（非推奨）', link: '/guide/operators/transformation/mapTo' },
              {
                text: 'mergeMap',
                link: '/guide/operators/transformation/mergeMap',
              },
              {
                text: 'switchMap',
                link: '/guide/operators/transformation/switchMap',
              },
              {
                text: 'concatMap',
                link: '/guide/operators/transformation/concatMap',
              },
              {
                text: 'exhaustMap',
                link: '/guide/operators/transformation/exhaustMap',
              },
              {
                text: 'expand',
                link: '/guide/operators/transformation/expand',
              },
              {
                text: 'buffer',
                link: '/guide/operators/transformation/buffer',
              },
              {
                text: 'bufferTime',
                link: '/guide/operators/transformation/bufferTime',
              },
              {
                text: 'bufferCount',
                link: '/guide/operators/transformation/bufferCount',
              },
              {
                text: 'bufferWhen',
                link: '/guide/operators/transformation/bufferWhen',
              },
              {
                text: 'bufferToggle',
                link: '/guide/operators/transformation/bufferToggle',
              },
              {
                text: 'windowTime',
                link: '/guide/operators/transformation/windowTime',
              },
              {
                text: '実用的なユースケース',
                link: '/guide/operators/transformation/practical-use-cases.md',
              },
            ],
          },
          {
            text: 'フィルタリングオペレーター',
            link: '/guide/operators/filtering/',
            collapsed: true,
            items: [
              { text: 'filter', link: '/guide/operators/filtering/filter' },
              { text: 'take', link: '/guide/operators/filtering/take' },
              { text: 'takeLast', link: '/guide/operators/filtering/takeLast' },
              { text: 'takeWhile', link: '/guide/operators/filtering/takeWhile' },
              { text: 'skip', link: '/guide/operators/filtering/skip' },
              { text: 'skipUntil', link: '/guide/operators/filtering/skipUntil' },
              { text: 'first', link: '/guide/operators/filtering/first' },
              { text: 'last', link: '/guide/operators/filtering/last' },
              { text: 'find', link: '/guide/operators/filtering/find' },
              {
                text: 'debounceTime',
                link: '/guide/operators/filtering/debounceTime',
              },
              {
                text: 'throttleTime',
                link: '/guide/operators/filtering/throttleTime',
              },
              {
                text: 'auditTime',
                link: '/guide/operators/filtering/auditTime',
              },
              {
                text: 'distinct',
                link: '/guide/operators/filtering/distinct',
              },
              {
                text: 'distinctUntilChanged',
                link: '/guide/operators/filtering/distinctUntilChanged',
              },
              {
                text: 'distinctUntilKeyChanged',
                link: '/guide/operators/filtering/distinctUntilKeyChanged',
              },
              {
                text: '実用的なユースケース',
                link: '/guide/operators/filtering/practical-use-cases.md',
              },
            ],
          },
          {
            text: '結合オペレーター（Pipeable）',
            link: '/guide/operators/combination/',
            collapsed: true,
            items: [
              {
                text: 'withLatestFrom',
                link: '/guide/operators/combination/withLatestFrom',
              },
              {
                text: '実用的なユースケース',
                link: '/guide/operators/combination/practical-use-cases.md',
              },
            ],
          },
          {
            text: 'ユーティリティオペレーター',
            link: '/guide/operators/utility/',
            collapsed: true,
            items: [
              { text: 'tap', link: '/guide/operators/utility/tap' },
              { text: 'delay', link: '/guide/operators/utility/delay' },
              { text: 'timeout', link: '/guide/operators/utility/timeout' },
              { text: 'takeUntil', link: '/guide/operators/utility/takeUntil' },
              { text: 'finalize', link: '/guide/operators/utility/finalize' },
              { text: 'repeat', link: '/guide/operators/utility/repeat' },
              { text: 'retry', link: '/guide/operators/utility/retry' },
              { text: 'startWith', link: '/guide/operators/utility/startWith' },
              { text: 'toArray', link: '/guide/operators/utility/toArray' },
              {
                text: '実用的なユースケース',
                link: '/guide/operators/utility/practical-use-cases.md',
              },
            ],
          },
          {
            text: '条件オペレーター',
            link: '/guide/operators/conditional/',
            collapsed: true,
            items: [
              { text: 'iif', link: '/guide/operators/conditional/iif' },
              { text: 'defer', link: '/guide/operators/conditional/defer' },
              {
                text: 'defaultIfEmpty',
                link: '/guide/operators/conditional/defaultIfEmpty',
              },
              { text: 'every', link: '/guide/operators/conditional/every' },
              { text: 'isEmpty', link: '/guide/operators/conditional/isEmpty' },
              {
                text: '実用的なユースケース',
                link: '/guide/operators/conditional/practical-use-cases.md',
              },
            ],
          },
          {
            text: 'マルチキャスティング',
            link: '/guide/operators/multicasting/',
            collapsed: true,
            items: [
              { text: 'share', link: '/guide/operators/multicasting/share' },
              { text: 'shareReplay', link: '/guide/operators/multicasting/shareReplay' },
            ],
          },
        ],
      },
      {
        text: '5. Subjectとマルチキャスト',
        items: [
          { text: 'Subjectとは', link: '/guide/subjects/what-is-subject' },
          { text: 'Subjectの種類', link: '/guide/subjects/types-of-subject' },
          {
            text: 'Multicastingの仕組み',
            link: '/guide/subjects/multicasting',
          },
          { text: 'Subjectのユースケース', link: '/guide/subjects/use-cases' },
        ],
      },
      {
        text: '6. エラーハンドリング',
        items: [
          { text: 'エラー処理戦略', link: '/guide/error-handling/strategies' },
          {
            text: 'retry と catchError',
            link: '/guide/error-handling/retry-catch',
          },
          {
            text: 'finalize と complete',
            link: '/guide/error-handling/finalize',
          },
        ],
      },
      {
        text: '7. スケジューラーの活用',
        items: [
          { text: '非同期処理の制御', link: '/guide/schedulers/async-control' },
          {
            text: 'スケジューラーの種類と使い分け',
            link: '/guide/schedulers/types',
          },
          {
            text: '補足:タスクとスケジューラーの基礎知識',
            link: '/guide/schedulers/task-and-scheduler-basics',
          },
        ],
      },
      {
        text: '8. RxJSのデバッグ手法',
        link: '/guide/debugging/',
      },
      {
        text: '9. テスト手法',
        items: [
          { text: 'RxJSのユニットテスト', link: '/guide/testing/unit-tests' },
          {
            text: 'TestSchedulerの活用',
            link: '/guide/testing/test-scheduler',
          },
          { text: 'マーブルテスト', link: '/guide/testing/marble-testing' },
        ],
      },
      {
        text: '10. RxJSアンチパターン集',
        items: [
          { text: 'アンチパターンとは', link: '/guide/anti-patterns/' },
          { text: 'よくある間違いと対処法', link: '/guide/anti-patterns/common-mistakes' },
          { text: 'アンチパターン回避チェックリスト', link: '/guide/anti-patterns/checklist' },
        ],
      },
      {
        text: '11. TypeScriptとRxJSの高度な連携',
        // items: [
        //   {
        //     text: 'TypeScriptとRxJSの基本連携',
        //     link: '/guide/typescript-advanced/type-safety',
        //   },
        //   {
        //     text: 'ジェネリクスの活用',
        //     link: '/guide/typescript-advanced/generics',
        //   },
        //   {
        //     text: 'カスタムオペレーターと型定義',
        //     link: '/guide/typescript-advanced/custom-operators',
        //   },
        //   {
        //     text: '条件型とマッピング型の活用',
        //     link: '/guide/typescript-advanced/conditional-types',
        //   },
        // ],
      },
      {
        text: '12. 実践パターン',
        // items: [
        //   {
        //     text: '状態管理パターン',
        //     link: '/guide/patterns/state-management',
        //   },
        //   { text: '非同期データフェッチ', link: '/guide/patterns/async-data' },
        //   {
        //     text: 'デバウンスとスロットル',
        //     link: '/guide/patterns/debounce-throttle',
        //   },
        //   { text: 'キャッシュ戦略', link: '/guide/patterns/caching' },
        //   {
        //     text: 'メモリリークの防止',
        //     link: '/guide/patterns/memory-leak-prevention',
        //   },
        // ],
      },
      {
        text: '13. パフォーマンス最適化',
        // items: [
        //   {
        //     text: '購読の適切な管理',
        //     link: '/guide/performance/subscription-management',
        //   },
        //   {
        //     text: '効率的なオペレーター選択',
        //     link: '/guide/performance/operator-selection',
        //   },
        //   {
        //     text: 'ストリームの設計パターン',
        //     link: '/guide/performance/stream-design',
        //   },
        // ],
      },
      {
        text: '14. フレームワークとの統合',
        // items: [
        //   { text: 'Angularとの連携', link: '/guide/frameworks/angular' },
        //   { text: 'Reactとの連携', link: '/guide/frameworks/react' },
        //   { text: 'Vueとの連携', link: '/guide/frameworks/vue' },
        //   { text: 'Web APIとの連携', link: '/guide/frameworks/web-api' },
        // ],
      },
    ],

    socialLinks: [
      {
        icon: 'github',
        link: 'https://github.com/shuji-bonji/RxJS-with-TypeScript',
      },
    ],

    search: {
      provider: 'local',
    },

    footer: {
      message: 'Released under the CC-BY-4.0 license.',
      copyright: 'Copyright © 2025 shuji-bonji',
    },
  },
  markdown: {
    config: (md) => {
      md.use(footnote);
    },
  },
  sitemap: {
    hostname: 'https://shuji-bonji.github.io/RxJS-with-TypeScript/',
  },
});
