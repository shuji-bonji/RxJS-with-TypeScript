import DefaultTheme from 'vitepress/theme';
import './custom.css';
import { onMounted } from 'vue';
import type { Theme } from 'vitepress';

export default {
  extends: DefaultTheme,
  setup() {
    onMounted(() => {
      // svg-pan-zoom を動的にインポートして Mermaid SVG に適用
      import('svg-pan-zoom').then((module) => {
        const svgPanZoom = module.default;
        let dialogInstance: any = null;

        // ダイアログ要素を作成
        const createDialog = () => {
          const dialog = document.createElement('dialog');
          dialog.className = 'mermaid-fullscreen-dialog';
          dialog.innerHTML = `
            <div class="dialog-content">
              <button class="dialog-close" aria-label="閉じる">×</button>
              <div class="dialog-svg-container"></div>
            </div>
          `;
          document.body.appendChild(dialog);

          // 閉じるボタンのイベント
          const closeBtn = dialog.querySelector('.dialog-close');
          closeBtn?.addEventListener('click', () => {
            if (dialogInstance) {
              dialogInstance.destroy();
              dialogInstance = null;
            }
            dialog.close();
          });

          // バックドロップクリックで閉じる
          dialog.addEventListener('click', (e) => {
            if (e.target === dialog) {
              if (dialogInstance) {
                dialogInstance.destroy();
                dialogInstance = null;
              }
              dialog.close();
            }
          });

          return dialog;
        };

        const dialog = createDialog();

        // ページ内のすべての Mermaid SVG にクリックイベントを追加
        const initializeMermaidSvgs = () => {
          const mermaidSvgs = document.querySelectorAll('.mermaid svg');

          mermaidSvgs.forEach((svg) => {
            // すでに初期化されている場合はスキップ
            if ((svg as any).__mermaidInitialized) {
              return;
            }

            // クリックで全画面表示
            svg.addEventListener('click', () => {
              const container = dialog.querySelector('.dialog-svg-container');
              if (!container) return;

              // SVGをクローン
              const clonedSvg = svg.cloneNode(true) as SVGElement;
              container.innerHTML = '';
              container.appendChild(clonedSvg);

              // ダイアログを表示
              dialog.showModal();

              // クローンしたSVGにsvg-pan-zoomを適用
              try {
                if (dialogInstance) {
                  dialogInstance.destroy();
                }
                dialogInstance = svgPanZoom(clonedSvg, {
                  zoomEnabled: true,
                  controlIconsEnabled: true,
                  fit: true,
                  center: true,
                  minZoom: 0.1,
                  maxZoom: 20,
                  zoomScaleSensitivity: 0.3,
                });
              } catch (error) {
                console.warn('ダイアログ内のsvg-pan-zoom初期化に失敗:', error);
              }
            });

            // カーソルとヒントを表示
            svg.style.cursor = 'pointer';
            svg.setAttribute('title', 'クリックで全画面表示');

            // 初期化済みフラグを設定
            (svg as any).__mermaidInitialized = true;
          });
        };

        // 初回実行
        initializeMermaidSvgs();

        // ページ遷移時に再初期化（VitePress の SPA 対応）
        const observer = new MutationObserver(() => {
          initializeMermaidSvgs();
        });

        observer.observe(document.body, {
          childList: true,
          subtree: true,
        });
      });
    });
  },
} satisfies Theme;
