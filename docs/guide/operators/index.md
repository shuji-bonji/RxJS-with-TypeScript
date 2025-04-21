# オペレーターの理解

## オペレーターの分類と用途

| 種類 | 代表オペレーター | 用途の例 |
|---|---|---|
| 変換 | map, scan | 値を変換したり、累積処理を行う |
| フィルタ | filter, take, debounceTime | 条件に合う値のみを通す、入力制御など |
| 結合 | merge, concat, combineLatest | 複数のObservableを結合・合成 |
| エラー処理 | catchError, retry | エラー発生時のフォールバックや再試行 |
