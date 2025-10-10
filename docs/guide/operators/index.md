---
description: RxJSのオペレーターをカテゴリごとに分類し、各機能や用途、代表的なオペレーターを網羅的に一覧形式で紹介します。
---

# オペレーターの理解

RxJSのオペレーターは、Observableのデータストリームを変換・合成・制御するための関数群です。

オペレーターは通常、複数を組み合わせて使用することが多く、その際に登場するのが「パイプライン」です。
- [RxJSのパイプラインとは](./pipeline.md)

RxJSでは、オペレーターは以下のカテゴリに分類されます。


## カテゴリ一覧

- [変換オペレーター](./transformation/)
- [フィルタリングオペレーター](./filtering/)
- [結合オペレーター](./combination/)
- [ユーティリティオペレーター](./utility/)
- [条件オペレーター](./conditional/)
- [エラーハンドリングオペレーター](../error-handling/strategies)
- [マルチキャスティングオペレーター](./multicasting/)

それぞれのカテゴリに、多数の便利なオペレーターが含まれています。  
詳細は各カテゴリを参照してください。


## オペレーターの一覧表

各オペレーターの詳細な説明は、リンクをクリックして参照してください。

<table style="overflow: visible;">
  <caption>
   Operatorのカテゴリ一覧
  </caption>
  <thead>
    <tr>
      <th scope="col">カテゴリ</th>
      <th scope="col">Operator</th>
      <th scope="col">説明</th>
    </tr>
  </thead>
  <tbody>
    <!-- 変換オペレーター -->
    <tr>
      <th scope="row" rowspan="14"><a href="./transformation/">変換</a></th>
      <td><a href="./transformation/map.html">map</a></td>
      <td>各値を変換する</td>
    </tr>
    <tr>
      <td><a href="./transformation/pluck.html">pluck</a> ⚠️</td>
      <td>オブジェクトから特定プロパティを抽出する（非推奨）</td>
    </tr>
    <tr>
      <td><a href="./transformation/mapTo.html">mapTo</a> ⚠️</td>
      <td>すべての値を固定値に変換する（非推奨）</td>
    </tr>
    <tr>
      <td><a href="./transformation/scan.html">scan</a></td>
      <td>値を蓄積しながら途中経過も出力する</td>
    </tr>
    <tr>
      <td><a href="./transformation/reduce.html">reduce</a></td>
      <td>すべての値を蓄積して最終結果のみ出力する</td>
    </tr>
    <tr>
      <td><a href="./transformation/pairwise.html">pairwise</a></td>
      <td>連続する2つの値をペアで処理する</td>
    </tr>
    <tr>
      <td><a href="./transformation/groupBy.html">groupBy</a></td>
      <td>キーごとにストリームをグループ化する</td>
    </tr>
    <tr>
      <td><a href="./transformation/mergeMap.html">mergeMap</a></td>
      <td>非同期処理を並列実行する</td>
    </tr>
    <tr>
      <td><a href="./transformation/switchMap.html">switchMap</a></td>
      <td>最新の非同期処理のみ実行（古い処理はキャンセル）</td>
    </tr>
    <tr>
      <td><a href="./transformation/concatMap.html">concatMap</a></td>
      <td>非同期処理を順次実行する</td>
    </tr>
    <tr>
      <td><a href="./transformation/exhaustMap.html">exhaustMap</a></td>
      <td>実行中は新しい処理を無視する</td>
    </tr>
    <tr>
      <td><a href="./transformation/buffer.html">buffer</a></td>
      <td>値を配列にまとめて発行する</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferTime.html">bufferTime</a></td>
      <td>指定時間ごとに値をまとめて発行する</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferCount.html">bufferCount</a></td>
      <td>指定個数ごとに値をまとめて発行する</td>
    </tr>
    <!-- フィルタリングオペレーター -->
    <tr>
      <th scope="row" rowspan="9"><a href="./filtering/">フィルタリング</a></th>
      <td><a href="./filtering/filter.html">filter</a></td>
      <td>条件に合致する値のみ通過させる</td>
    </tr>
    <tr>
      <td><a href="./filtering/take.html">take</a></td>
      <td>最初のN個の値のみ取得する</td>
    </tr>
    <tr>
      <td><a href="./filtering/first.html">first</a></td>
      <td>最初の値または条件を満たす最初の値を取得する</td>
    </tr>
    <tr>
      <td><a href="./filtering/last.html">last</a></td>
      <td>最後の値または条件を満たす最後の値を取得する</td>
    </tr>
    <tr>
      <td><a href="./filtering/debounceTime.html">debounceTime</a></td>
      <td>指定時間入力がない場合に最後の値を発行する</td>
    </tr>
    <tr>
      <td><a href="./filtering/throttleTime.html">throttleTime</a></td>
      <td>最初の値を通し、指定時間は新しい値を無視する</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinct.html">distinct</a></td>
      <td>すべての重複値を除去する（ユニークな値のみ出力）</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilChanged.html">distinctUntilChanged</a></td>
      <td>連続した重複値を除去する</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilKeyChanged.html">distinctUntilKeyChanged</a></td>
      <td>オブジェクトの特定プロパティの変更のみ検出する</td>
    </tr>
    <!-- 結合オペレーター -->
    <tr>
      <th scope="row" rowspan="7"><a href="./combination/">結合</a></th>
      <td><a href="./combination/concat.html">concat</a></td>
      <td>複数のObservableを順番に結合する</td>
    </tr>
    <tr>
      <td><a href="./combination/merge.html">merge</a></td>
      <td>複数のObservableを同時に結合する</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatest.html">combineLatest</a></td>
      <td>各Observableの最新値を組み合わせる</td>
    </tr>
    <tr>
      <td><a href="./combination/withLatestFrom.html">withLatestFrom</a></td>
      <td>メインストリームに他の最新値を付加する</td>
    </tr>
    <tr>
      <td><a href="./combination/zip.html">zip</a></td>
      <td>各Observableから1つずつ値を取ってペアにする</td>
    </tr>
    <tr>
      <td><a href="./combination/forkJoin.html">forkJoin</a></td>
      <td>すべてのObservableの完了を待って最終値を結合する</td>
    </tr>
    <tr>
      <td><a href="./combination/race.html">race</a></td>
      <td>最初に値を発行したObservableのみを使用する</td>
    </tr>
    <!-- ユーティリティオペレーター -->
    <tr>
      <th scope="row" rowspan="9"><a href="./utility/">ユーティリティ</a></th>
      <td><a href="./utility/tap.html">tap</a></td>
      <td>副作用を実行する（ログ出力など）</td>
    </tr>
    <tr>
      <td><a href="./utility/finalize.html">finalize</a></td>
      <td>完了またはエラー時に後処理を実行する</td>
    </tr>
    <tr>
      <td><a href="./utility/delay.html">delay</a></td>
      <td>すべての値を指定時間遅延させる</td>
    </tr>
    <tr>
      <td><a href="./utility/timeout.html">timeout</a></td>
      <td>指定時間内に値が来ない場合エラーを発行する</td>
    </tr>
    <tr>
      <td><a href="./utility/takeUntil.html">takeUntil</a></td>
      <td>別のObservableが値を発行するまで値を取得する</td>
    </tr>
    <tr>
      <td><a href="./utility/retry.html">retry</a></td>
      <td>エラー時に指定回数まで再試行する</td>
    </tr>
    <tr>
      <td><a href="./utility/repeat.html">repeat</a></td>
      <td>完了後に指定回数繰り返す</td>
    </tr>
    <tr>
      <td><a href="./utility/startWith.html">startWith</a></td>
      <td>ストリームの最初に初期値を追加する</td>
    </tr>
    <tr>
      <td><a href="./utility/toArray.html">toArray</a></td>
      <td>すべての値を配列にまとめて発行する</td>
    </tr>
    <!-- 条件オペレーター -->
    <tr>
      <th scope="row" rowspan="5"><a href="./conditional/">条件</a></th>
      <td><a href="./conditional/iif.html">iif</a></td>
      <td>条件により異なるObservableを生成する</td>
    </tr>
    <tr>
      <td><a href="./conditional/defer.html">defer</a></td>
      <td>購読時にObservableを生成する</td>
    </tr>
    <tr>
      <td><a href="./conditional/defaultIfEmpty.html">defaultIfEmpty</a></td>
      <td>値がない場合デフォルト値を発行する</td>
    </tr>
    <tr>
      <td><a href="./conditional/every.html">every</a></td>
      <td>すべての値が条件を満たすか判定する</td>
    </tr>
    <tr>
      <td><a href="./conditional/isEmpty.html">isEmpty</a></td>
      <td>値が発行されなかったか判定する</td>
    </tr>
    <!-- エラーハンドリング -->
    <tr>
      <th scope="row" rowspan="3"><a href="../error-handling/strategies.html">エラーハンドリング</a></th>
      <td><a href="../error-handling/retry-catch.html">catchError</a></td>
      <td>エラーを捕捉してフォールバック処理を実行する</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retry</a></td>
      <td>エラー時に指定回数まで再試行する</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retryWhen</a></td>
      <td>カスタム条件で再試行する</td>
    </tr>
    <!-- マルチキャスト -->
    <tr>
      <th scope="row" rowspan="2"><a href="./multicasting/">マルチキャスト</a></th>
      <td><a href="./multicasting/share.html">share</a></td>
      <td>Observableを複数の購読者間で共有する</td>
    </tr>
    <tr>
      <td><a href="./multicasting/shareReplay.html">shareReplay</a></td>
      <td>最新のN個の値をキャッシュして新規購読者に再生する</td>
    </tr>
  </tbody>
</table>
