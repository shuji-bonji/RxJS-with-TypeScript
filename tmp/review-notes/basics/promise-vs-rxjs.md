# Review: docs/guide/basics/promise-vs-rxjs.md & docs/en/guide/basics/promise-vs-rxjs.md

## Findings
1. 英語版には「Conversion by `defer`」節や `exhaustMap` によるフォーム送信例など、複数の節があるのに日本語版には存在せず、内容が非対称になっています（例: EN lines ~318-452）。日本語側へ追加するか、英語版を調整して内容を揃える必要あり。
2. 逆に日本語版の「RxJSが特に活躍する分野」表（JA lines ~470-481）は 6 行構成で詳細ですが、英語版の対応箇所（EN lines ~597-606）は 5 行で内容も異なります。両言語で同じ情報を持つよう統一してください。
3. 英語版末尾には `[!TIP] Next Steps` ブロック（EN lines 618-640）があり、日本語版には見出し自体がないためナビゲーションがずれます。どちらかに合わせる必要があります。

## Status
- 技術面: ⚠️（内容の欠落は参考情報不足につながる）
- 翻訳面: ⚠️（章構造の不一致）
