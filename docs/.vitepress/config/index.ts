import { defineConfig, HeadConfig } from 'vitepress';
import footnote from 'markdown-it-footnote';
import { withMermaid } from 'vitepress-plugin-mermaid';
import { jaThemeConfig } from './ja';
import { enThemeConfig } from './en';
import { frThemeConfig } from './fr';
import { deThemeConfig } from './de';
import { itThemeConfig } from './it';
import { esThemeConfig } from './es';

export default withMermaid(
  defineConfig({
    title: 'RxJS with TypeScript',
    titleTemplate: ':title | RxJS+TS',
    description: 'TypeScriptプログラマのためのRxJS入門',
    base: '/RxJS-with-TypeScript/',

    // URL正規化: .htmlなしのクリーンURLを使用
    cleanUrls: true,

    // 翻訳作業中は新言語のデッドリンクを無視（翻訳完了後に削除）
    ignoreDeadLinks: [
      /^\/fr\//,
      /^\/de\//,
      /^\/it\//,
      /^\/es\//,
    ],

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
      fr: {
        label: 'Français',
        lang: 'fr',
        link: '/fr/',
        themeConfig: frThemeConfig,
      },
      de: {
        label: 'Deutsch',
        lang: 'de',
        link: '/de/',
        themeConfig: deThemeConfig,
      },
      it: {
        label: 'Italiano',
        lang: 'it',
        link: '/it/',
        themeConfig: itThemeConfig,
      },
      es: {
        label: 'Español',
        lang: 'es',
        link: '/es/',
        themeConfig: esThemeConfig,
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
            },
            fr: {
              translations: {
                button: {
                  buttonText: 'Rechercher',
                  buttonAriaLabel: 'Rechercher'
                },
                modal: {
                  noResultsText: 'Aucun résultat pour',
                  resetButtonTitle: 'Réinitialiser la recherche',
                  footer: {
                    selectText: 'sélectionner',
                    navigateText: 'naviguer',
                    closeText: 'fermer'
                  }
                }
              }
            },
            de: {
              translations: {
                button: {
                  buttonText: 'Suchen',
                  buttonAriaLabel: 'Suchen'
                },
                modal: {
                  noResultsText: 'Keine Ergebnisse für',
                  resetButtonTitle: 'Suche zurücksetzen',
                  footer: {
                    selectText: 'auswählen',
                    navigateText: 'navigieren',
                    closeText: 'schließen'
                  }
                }
              }
            },
            it: {
              translations: {
                button: {
                  buttonText: 'Cerca',
                  buttonAriaLabel: 'Cerca'
                },
                modal: {
                  noResultsText: 'Nessun risultato per',
                  resetButtonTitle: 'Reimposta ricerca',
                  footer: {
                    selectText: 'selezionare',
                    navigateText: 'navigare',
                    closeText: 'chiudere'
                  }
                }
              }
            },
            es: {
              translations: {
                button: {
                  buttonText: 'Buscar',
                  buttonAriaLabel: 'Buscar'
                },
                modal: {
                  noResultsText: 'Sin resultados para',
                  resetButtonTitle: 'Restablecer búsqueda',
                  footer: {
                    selectText: 'seleccionar',
                    navigateText: 'navegar',
                    closeText: 'cerrar'
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

    // hreflang tags for SEO (multilingual support)
    transformHead: ({ pageData }) => {
      const head: HeadConfig[] = []
      const baseUrl = 'https://shuji-bonji.github.io/RxJS-with-TypeScript'
      const pagePath = pageData.relativePath.replace(/((^|\/)index)?\.md$/, '$2')

      // Supported locales with their path prefixes
      const locales = [
        { lang: 'ja', prefix: '' },      // root locale (Japanese)
        { lang: 'en', prefix: 'en/' },
        { lang: 'fr', prefix: 'fr/' },
        { lang: 'de', prefix: 'de/' },
        { lang: 'it', prefix: 'it/' },
        { lang: 'es', prefix: 'es/' },
      ]

      // Determine current locale and extract the content path
      let currentLang = 'ja'
      let contentPath = pagePath

      for (const locale of locales) {
        if (locale.prefix && pagePath.startsWith(locale.prefix)) {
          currentLang = locale.lang
          contentPath = pagePath.replace(new RegExp(`^${locale.prefix}`), '')
          break
        }
      }

      // Generate canonical URL for current page
      const currentPrefix = locales.find(l => l.lang === currentLang)?.prefix || ''
      const canonicalUrl = `${baseUrl}/${currentPrefix}${contentPath}`
      head.push(['link', { rel: 'canonical', href: canonicalUrl }])

      // Generate hreflang tags for all locales
      for (const locale of locales) {
        const url = `${baseUrl}/${locale.prefix}${contentPath}`
        head.push(['link', { rel: 'alternate', hreflang: locale.lang, href: url }])
      }

      // x-default points to Japanese (primary language)
      head.push(['link', { rel: 'alternate', hreflang: 'x-default', href: `${baseUrl}/${contentPath}` }])

      return head
    },
  })
);
