---
description: RxJSのオペレーターをカテゴリごとに分類し、各機能や用途、代表的なオペレーターを網羅的に一覧形式で紹介します。変換、フィルタリング、結合、ユーティリティ、条件、エラーハンドリング、マルチキャスティングの7つのカテゴリで整理され、パイプラインの概念とともにTypeScriptでの実践的な使い方を学べます。
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
- [エラーハンドリングオペレーター](../error-handling/strategies.md)
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
      <th scope="row" rowspan="15"><a href="./transformation/">変換</a></th>
      <td><a href="./transformation/map.html">map</a></td>
      <td>各値を変換する</td>
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
      <td><a href="./transformation/expand.html">expand</a></td>
      <td>結果を再帰的に展開する</td>
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
    <tr>
      <td><a href="./transformation/bufferWhen.html">bufferWhen</a></td>
      <td>終了条件を動的に制御してバッファリング</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferToggle.html">bufferToggle</a></td>
      <td>開始と終了を独立制御してバッファリング</td>
    </tr>
    <!-- フィルタリングオペレーター -->
    <tr>
      <th scope="row" rowspan="22"><a href="./filtering/">フィルタリング</a></th>
      <td><a href="./filtering/filter.html">filter</a></td>
      <td>条件に合致する値のみ通過させる</td>
    </tr>
    <tr>
      <td><a href="./filtering/take.html">take</a></td>
      <td>最初のN個の値のみ取得する</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeLast.html">takeLast</a></td>
      <td>最後のN個の値を取得する</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeWhile.html">takeWhile</a></td>
      <td>条件を満たす間値を取得する</td>
    </tr>
    <tr>
      <td><a href="./filtering/skip.html">skip</a></td>
      <td>最初のN個の値をスキップする</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipLast.html">skipLast</a></td>
      <td>最後のN個の値をスキップする</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipWhile.html">skipWhile</a></td>
      <td>条件を満たす間値をスキップする</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipUntil.html">skipUntil</a></td>
      <td>別のObservableが発火するまで値をスキップする</td>
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
      <td><a href="./filtering/elementAt.html">elementAt</a></td>
      <td>指定されたインデックスの値を取得する</td>
    </tr>
    <tr>
      <td><a href="./filtering/find.html">find</a></td>
      <td>条件を満たす最初の値を見つける</td>
    </tr>
    <tr>
      <td><a href="./filtering/findIndex.html">findIndex</a></td>
      <td>条件を満たす最初の値のインデックスを取得する</td>
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
      <td><a href="./filtering/auditTime.html">auditTime</a></td>
      <td>指定時間後に最後の値を発行する</td>
    </tr>
    <tr>
      <td><a href="./filtering/audit.html">audit</a></td>
      <td>カスタムObservableで期間を制御して最後の値を発行する</td>
    </tr>
    <tr>
      <td><a href="./filtering/sampleTime.html">sampleTime</a></td>
      <td>指定時間間隔で最新値をサンプリングする</td>
    </tr>
    <tr>
      <td><a href="./filtering/ignoreElements.html">ignoreElements</a></td>
      <td>すべての値を無視して完了/エラーのみ通す</td>
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
    <!-- 結合オペレーター（Pipeable） -->
    <tr>
      <th scope="row" rowspan="12"><a href="./combination/">結合（Pipeable）</a></th>
      <td><a href="./combination/concatWith.html">concatWith</a></td>
      <td>完了後に他のObservableを順番に結合する</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeWith.html">mergeWith</a></td>
      <td>複数のObservableを同時に結合する</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestWith.html">combineLatestWith</a></td>
      <td>各Observableの最新値を組み合わせる</td>
    </tr>
    <tr>
      <td><a href="./combination/zipWith.html">zipWith</a></td>
      <td>対応する順番の値をペア化する</td>
    </tr>
    <tr>
      <td><a href="./combination/raceWith.html">raceWith</a></td>
      <td>最初に発火したObservableのみを採用する</td>
    </tr>
    <tr>
      <td><a href="./combination/withLatestFrom.html">withLatestFrom</a></td>
      <td>メインストリームに他の最新値を付加する</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeAll.html">mergeAll</a></td>
      <td>Higher-order Observableを並行に平坦化する</td>
    </tr>
    <tr>
      <td><a href="./combination/concatAll.html">concatAll</a></td>
      <td>Higher-order Observableを順番に平坦化する</td>
    </tr>
    <tr>
      <td><a href="./combination/switchAll.html">switchAll</a></td>
      <td>最新のHigher-order Observableに切り替える</td>
    </tr>
    <tr>
      <td><a href="./combination/exhaustAll.html">exhaustAll</a></td>
      <td>実行中は新しいHigher-order Observableを無視する</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestAll.html">combineLatestAll</a></td>
      <td>全ての内部Observableの最新値を組み合わせる</td>
    </tr>
    <tr>
      <td><a href="./combination/zipAll.html">zipAll</a></td>
      <td>各内部Observableの対応する値をペア化する</td>
    </tr>
    <!-- ユーティリティオペレーター -->
    <tr>
      <th scope="row" rowspan="15"><a href="./utility/">ユーティリティ</a></th>
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
      <td><a href="./utility/delayWhen.html">delayWhen</a></td>
      <td>各値を別のObservableで動的に遅延させる</td>
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
    <tr>
      <td><a href="./utility/materialize.html">materialize</a></td>
      <td>通知をNotificationオブジェクトに変換する</td>
    </tr>
    <tr>
      <td><a href="./utility/dematerialize.html">dematerialize</a></td>
      <td>Notificationオブジェクトを通常の通知に戻す</td>
    </tr>
    <tr>
      <td><a href="./utility/observeOn.html">observeOn</a></td>
      <td>値の発行タイミングをスケジューラーで制御する</td>
    </tr>
    <tr>
      <td><a href="./utility/subscribeOn.html">subscribeOn</a></td>
      <td>購読開始タイミングをスケジューラーで制御する</td>
    </tr>
    <tr>
      <td><a href="./utility/timestamp.html">timestamp</a></td>
      <td>各値にタイムスタンプを付与する</td>
    </tr>
    <!-- 条件オペレーター -->
    <tr>
      <th scope="row" rowspan="3"><a href="./conditional/">条件</a></th>
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
