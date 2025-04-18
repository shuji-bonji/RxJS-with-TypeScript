// docs/.vitepress/config.ts
import { defineConfig } from 'vitepress';

export default defineConfig({
  title: 'RxJS with TypeScript',
  description: 'TypeScriptプログラマのためのRxJS入門',
  themeConfig: {
    nav: [
      { text: 'ホーム', link: '/' },
      { text: 'ガイド', link: '/guide/' },
    ],
    sidebar: [
      {
        text: '導入',
        items: [{ text: 'はじめに', link: '/guide/introduction' }],
      },
      {
        text: '1. 基本概念',
        items: [
          { text: 'RxJSとは何か', link: '/guide/basics/what-is-rxjs' },
          { text: 'RxJSの主要概念', link: '/guide/basics/key-concepts' },
          {
            text: 'TypeScriptとRxJSの基本連携',
            link: '/guide/basics/typescript-and-rxjs-basics',
          },
        ],
      },
      {
        text: '2. Observableの基礎',
        items: [
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
        text: '3. オペレーターの理解',
        items: [
          { text: 'パイプラインの概念', link: '/guide/operators/pipeline' },
          { text: '変換オペレーター', link: '/guide/operators/transformation' },
          {
            text: 'フィルタリングオペレーター',
            link: '/guide/operators/filtering',
          },
          { text: '結合オペレーター', link: '/guide/operators/combination' },
          {
            text: 'ユーティリティオペレーター',
            link: '/guide/operators/utility',
          },
          {
            text: '条件オペレーター',
            link: '/guide/operators/conditional',
          },
          {
            text: 'マルチキャスティング',
            link: '/guide/operators/multicasting',
          },
        ],
      },
      {
        text: '4. エラーハンドリング',
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
        text: '5. スケジューラーの活用',
        items: [
          { text: '非同期処理の制御', link: '/guide/schedulers/async-control' },
          {
            text: 'スケジューラーの種類と使い分け',
            link: '/guide/schedulers/types',
          },
        ],
      },
      {
        text: '6. テスト手法',
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
        text: '7. TypeScriptとRxJSの高度な連携',
        items: [
          {
            text: '型安全なObservableチェーン',
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
        text: '8. 実践パターン',
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
        text: '9. パフォーマンス最適化',
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
        text: '10. フレームワークとの統合',
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
  },
});
