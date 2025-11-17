import { defineConfig } from 'vitepress';
import footnote from 'markdown-it-footnote';
import { withMermaid } from 'vitepress-plugin-mermaid';
import { jaThemeConfig } from './ja';
import { enThemeConfig } from './en';

export default withMermaid(
  defineConfig({
    title: 'RxJS with TypeScript',
    titleTemplate: ':title | RxJS+TS',
    description: 'TypeScriptプログラマのためのRxJS入門',
    base: '/RxJS-with-TypeScript/',

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
      ['link', { rel: 'icon', href: '/RxJS-with-TypeScript/favicon.ico' }],
    ],

    locales: {
      root: {
        label: '日本語',
        lang: 'ja',
        themeConfig: jaThemeConfig,
      },
      en: {
        label: 'English',
        lang: 'en',
        link: '/en/',
        themeConfig: enThemeConfig,
      },
    },

    themeConfig: {
      search: {
        provider: 'local',
        options: {
          locales: {
            root: {
              translations: {
                button: {
                  buttonText: '検索',
                  buttonAriaLabel: '検索'
                },
                modal: {
                  noResultsText: '見つかりませんでした',
                  resetButtonTitle: '検索をリセット',
                  footer: {
                    selectText: '選択',
                    navigateText: '移動',
                    closeText: '閉じる'
                  }
                }
              }
            },
            en: {
              translations: {
                button: {
                  buttonText: 'Search',
                  buttonAriaLabel: 'Search'
                },
                modal: {
                  noResultsText: 'No results for',
                  resetButtonTitle: 'Reset search',
                  footer: {
                    selectText: 'to select',
                    navigateText: 'to navigate',
                    closeText: 'to close'
                  }
                }
              }
            }
          }
        }
      }
    },

    markdown: {
      config: (md) => {
        md.use(footnote);
      },
    },

    // Gitコミット履歴から最終更新日を取得
    lastUpdated: true,

    sitemap: {
      hostname: 'https://shuji-bonji.github.io/RxJS-with-TypeScript/',
      lastmodDateOnly: false, // 時刻まで含める（より正確）
    },
  })
);
