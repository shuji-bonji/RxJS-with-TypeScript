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
          'https://shuji-bonji.github.io/RxJS-with-TypeScript/images/ts-de-rxjs.png',
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
        text: '3. Subjectとマルチキャスト',
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
        text: '4. オペレーターの理解',
        items: [
          { text: 'パイプラインの概念', link: '/guide/operators/pipeline' },
          { text: '変換オペレーター', link: '/guide/operators/transformation' },
          {
            text: 'フィルタリングオペレーター',
            link: '/guide/operators/filtering/',
            items: [
              { text: 'filter', link: '/guide/operators/filtering/filter' },
              { text: 'take', link: '/guide/operators/filtering/take' },
              { text: 'first', link: '/guide/operators/filtering/first' },
              { text: 'last', link: '/guide/operators/filtering/last' },
              {
                text: 'debounceTime',
                link: '/guide/operators/filtering/debounceTime',
              },
              {
                text: 'throttleTime',
                link: '/guide/operators/filtering/throttleTime',
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
            text: '結合オペレーター',
            link: '/guide/operators/combination/',
            items: [
              { text: 'concat', link: '/guide/operators/combination/concat' },
              { text: 'merge', link: '/guide/operators/combination/merge' },
              {
                text: 'combineLatest',
                link: '/guide/operators/combination/combineLatest',
              },
              { text: 'zip', link: '/guide/operators/combination/zip' },
              {
                text: 'withLatestFrom',
                link: '/guide/operators/combination/withLatestFrom',
              },
              {
                text: 'forkJoin',
                link: '/guide/operators/combination/forkJoin',
              },
              { text: 'race', link: '/guide/operators/combination/race' },
              {
                text: '実用的なユースケース',
                link: '/guide/operators/combination/practical-use-cases.md',
              },
            ],
          },
          {
            text: 'ユーティリティオペレーター',
            link: '/guide/operators/utility/',
            items: [
              { text: 'tap', link: '/guide/operators/utility/tap' },
              { text: 'delay', link: '/guide/operators/utility/delay' },
              { text: 'timeout', link: '/guide/operators/utility/timeout' },
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
            link: '/guide/operators/multicasting',
          },
        ],
      },
      {
        text: '5. エラーハンドリング',
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
        text: '6. スケジューラーの活用',
        items: [
          { text: '非同期処理の制御', link: '/guide/schedulers/async-control' },
          {
            text: 'スケジューラーの種類と使い分け',
            link: '/guide/schedulers/types',
          },
        ],
      },
      {
        text: '7. テスト手法',
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
        text: '8. TypeScriptとRxJSの高度な連携',
        items: [
          {
            text: 'TypeScriptとRxJSの基本連携',
            link: '/guide/typescript-advanced/type-safety',
          },
          {
            text: 'ジェネリクスの活用',
            link: '/guide/typescript-advanced/generics',
          },
          {
            text: 'カスタムオペレーターと型定義',
            link: '/guide/typescript-advanced/custom-operators',
          },
          {
            text: '条件型とマッピング型の活用',
            link: '/guide/typescript-advanced/conditional-types',
          },
        ],
      },
      {
        text: '9. 実践パターン',
        items: [
          {
            text: '状態管理パターン',
            link: '/guide/patterns/state-management',
          },
          { text: '非同期データフェッチ', link: '/guide/patterns/async-data' },
          {
            text: 'デバウンスとスロットル',
            link: '/guide/patterns/debounce-throttle',
          },
          { text: 'キャッシュ戦略', link: '/guide/patterns/caching' },
          {
            text: 'メモリリークの防止',
            link: '/guide/patterns/memory-leak-prevention',
          },
        ],
      },
      {
        text: '10. パフォーマンス最適化',
        items: [
          {
            text: '購読の適切な管理',
            link: '/guide/performance/subscription-management',
          },
          {
            text: '効率的なオペレーター選択',
            link: '/guide/performance/operator-selection',
          },
          {
            text: 'ストリームの設計パターン',
            link: '/guide/performance/stream-design',
          },
        ],
      },
      {
        text: '11. フレームワークとの統合',
        items: [
          { text: 'Angularとの連携', link: '/guide/frameworks/angular' },
          { text: 'Reactとの連携', link: '/guide/frameworks/react' },
          { text: 'Vueとの連携', link: '/guide/frameworks/vue' },
          { text: 'Web APIとの連携', link: '/guide/frameworks/web-api' },
        ],
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
});
