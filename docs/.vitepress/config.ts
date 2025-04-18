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
            text: 'TypeScriptとRxJSの統合',
            link: '/guide/basics/typeScript-and0rxjs-integration',
          },
        ],
      },
      {
        text: '2. Observableの基礎',
        items: [
          { text: 'Observableの作成方法', link: '/guide/observables/creation' },
          { text: 'イベントのストリーム化', link: '/guide/observables/events' },
          { text: '購読と解除', link: '/guide/observables/subscription' },
          {
            text: 'Hot vs Cold Observable',
            link: '/guide/observables/hot-cold',
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
        text: '7. TypeScript特有の利点',
        items: [
          {
            text: '型安全なObservableチェーン',
            link: '/guide/typescript/type-safety',
          },
          { text: 'ジェネリクスの活用', link: '/guide/typescript/generics' },
          {
            text: 'インターフェースとの連携',
            link: '/guide/typescript/interfaces',
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
        ],
      },
      {
        text: '9. パフォーマンス最適化',
        items: [
          {
            text: 'メモリリークの防止',
            link: '/guide/performance/memory-leaks',
          },
          {
            text: '購読の適切な管理',
            link: '/guide/performance/subscription-management',
          },
          {
            text: '効率的なオペレーター選択',
            link: '/guide/performance/operator-selection',
          },
        ],
      },
      {
        text: '10. 応用例',
        items: [
          { text: 'Angularとの連携', link: '/guide/applications/angular' },
          { text: 'TypeScriptでのSPA開発', link: '/guide/applications/spa' },
          { text: 'Web APIとの連携', link: '/guide/applications/web-api' },
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
