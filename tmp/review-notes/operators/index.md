# Review: docs/guide/operators/index.md & docs/en/guide/operators/index.md

## Findings
1. オペレーター一覧のリンクが `.html` を付けたまま（例: `./transformation/map.html`）になっており、VitePress dev では 404 になります。`.md` リンクに統一してください（英語版も同様）。
2. カテゴリ一覧の「エラーハンドリングオペレーター」が `../error-handling/strategies` への拡張子なしリンクになっており、開発サーバーでは解決されません。`.md` を付ける必要があります。

## Status
- 技術面: ⚠️
- 翻訳面: ⚠️
