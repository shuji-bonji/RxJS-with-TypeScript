# 変換オペレーター

変換オペレーターは、RxJSのパイプライン内でデータを変形・加工するために使用されます。  
値を新しい形に変換することで、リアクティブなデータフローをより柔軟かつ強力に制御できるようになります。


## 🧩 主な変換オペレーター

| カテゴリ | オペレーター例 | 説明 |
|:---|:---|:---|
| 単純な値の変換 | `map`, `pluck`, `mapTo` | 各値を変換または抽出 |
| 累積処理 | `scan` | 値を蓄積しながら出力 |
| 非同期変換 | `mergeMap`, `switchMap`, `concatMap`, `exhaustMap` | 非同期処理を展開・制御 |
| バッチ処理 | `bufferTime`, `bufferCount`, `windowTime` | 一定時間・個数でまとめる |


## 📖 各オペレーターの詳細

- [map](./map) - 各値に変換関数を適用
- [scan](./scan) - 累積的に値を生成
- [pluck](./pluck) - オブジェクトからプロパティを抽出
- [mapTo](./mapTo) - 常に固定値を出力
- [mergeMap](./mergeMap) - 各値をObservableに変換し、並列で結合
- [switchMap](./switchMap) - 最新のObservableに切り替え
- [concatMap](./concatMap) - 各Observableを順番に実行
- [exhaustMap](./exhaustMap) - 実行中は新しい入力を無視
- [bufferTime](./bufferTime) - 一定時間ごとに値をまとめる
- [bufferCount](./bufferCount) - 指定個数ごとにまとめる
- [windowTime](./windowTime) - 一定時間ごとにサブObservableに分割


## 💡 実用的な変換パターン

現実のアプリケーションでは、変換オペレーターを組み合わせることで  
次のような処理が可能になります。

- 入力バリデーションとフィードバック
- 非同期APIリクエストの最適制御
- データの整形・集約・正規化
- イベントストリームのバッチ処理やグループ化

👉 詳しくは：[実用的な変換パターン](./practical-use-cases) を参照してください。


## 🧠 ポイントまとめ

- **単純変換**は `map`、**非同期処理**は `mergeMap`・`switchMap`
- **順序保証**には `concatMap`、**重複防止**には `exhaustMap`
- **バッチ処理**には `bufferTime`や`bufferCount`
- 状況に応じて適切な変換オペレーターを選択し、組み合わせましょう！
