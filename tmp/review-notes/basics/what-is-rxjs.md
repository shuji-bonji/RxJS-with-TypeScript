# Review: docs/guide/basics/what-is-rxjs.md & docs/en/guide/basics/what-is-rxjs.md

## Findings
1. `Subscription` 行のリンクが `../observables/observable-lifecycle.html#subscription` になっており、VitePress dev では 404 になります。`.md` 参照に直す必要あり（英語版も同様）。
2. 「リアルタイム通信」表の `webSocket` リンクが `../operators/index.md` に向いていて、API 解説ページにつながりません。`../observables/creation.md#websocket` など該当箇所へ更新してください（英語版も同じ）。
3. 英語版 Overview の “builds programs in a way that reacts (reactions)” が不自然。日本語のニュアンス（「その流れに対して反応する方式」）に合わせた自然な文章へ修正。
4. 英語版 `Creation Functions` 説明が “creating and binding” になっており、日本語の「作成・結合」を正しく訳せていない。“creating and combining” に修正。

## Status
- 技術面: ⚠️
- 翻訳面: ⚠️
