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
- [マルチキャスティングオペレーター](./multicasting)

それぞれのカテゴリに、多数の便利なオペレーターが含まれています。  
詳細は各カテゴリを参照してください。


## オペレーターの一覧表

<table>
  <caption>
   Operatorのカテゴリ一覧
  </caption>
  <thead>
    <tr>
      <th scope="col">カテゴリ</th>
      <th scope="col">サブカテゴリ</th>
      <th scope="col">Operators</th>
      <th scope="col">説明</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th scope="row" rowspan="4"><a href="./transformation/">変換</a></th>
      <td>単純な値の変換</td>
      <td>
        <a href="./transformation/map.html"><code>map</code></a>, <a href="./transformation/pluck.html">pluck</a>, <a href="./transformation/mapTo.html">mapTo</a></td>
      <td>各値を変換または抽出</td>
    </tr>
    <tr>
      <td>累積処理</td>
      <td><a href="./transformation/scan.html">scan</a></td>
      <td>値を蓄積しながら出力</td>
    </tr>
    <tr>
      <td>非同期変換</td>
      <td><a href="./transformation/mergeMap.html">mergeMap</a>, <a href="./transformation/switchMap.html">switchMap</a>, <br><a href="./transformation/concatMap.html">concatMap</a>, <a href="./transformation/exhaustMap.html">exhaustMap</a></td>
      <td>非同期処理を展開・制御</td>
    </tr>
    <tr>
      <td>バッチ処理</td>
      <td style="text-align:left;"><a href="./transformation/bufferTime.html">bufferTime</a>, <a href="./transformation/bufferCount.html">bufferCount</a>, <a href="./transformation/windowTime.html">windowTime</a></td>
      <td>一定時間・個数でまとめる</td>
    </tr>
    <tr>
      <th scope="row" rowspan="3"><a href="./filtering/">ファイルタリング</a></th>
      <td>条件による選別</td>
      <td><a href="./filtering/filter.html">filter</a>, <a href="./filtering/take.html">take</a>, <a href="./filtering/first.html">first</a>, <a href="./filtering/last.html">last</a></td>
      <td>条件に応じて値を選別</td>
    </tr>
    <tr>
      <td>時間による間引き</td>
      <td><a href="./filtering/debounceTime.html">debounceTime</a>, <a href="./filtering/throttleTime.html">throttleTime</a></td>
      <td>一定時間の間隔で発行</td>
    </tr>
    <tr>
      <td>重複の除去</td>
      <td><a href="./filtering/distinctUntilChanged.html">distinctUntilChanged</a>, <a href="./filtering/distinctUntilKeyChanged.html">distinctUntilKeyChanged</a></td>
      <td>重複する値の発行を防ぐ</td>
    </tr>
    <tr>
      <th scope="row" rowspan="3"><a href="./combination/">結合</a></th>
      <td>ストリームの順次結合</td>
      <td><a href="./combination/concat.html">concat</a>, <a href="./combination/merge.html">merge</a></td>
      <td>複数のObservableを順番または同時に結合</td>
    </tr>
    <tr>
      <td>最新値の合成</td>
      <td><a href="./combination/combineLatest.html">combineLatest</a>, <a href="./combination/withLatestFrom.html">withLatestFrom</a></td>
      <td>複数の最新値を組み合わせる</td>
    </tr>
    <tr>
      <td>完了を待つ/競争</td>
      <td><a href="./combination/zip.html">zip</a>, <a href="./combination/forkJoin.html">forkJoin</a>, <a href="./combination/race.html">race</a></td>
      <td>すべて/最初の完了に反応</td>
    </tr>
    <tr>
      <th scope="row" rowspan="4"><a href="./utility/">ユーティリティ</a></th>
      <td>副作用・監視</td>
      <td><a href="./utility/tap.html">tap</a>, <a href="./utility/finalize.html">finalize</a></td>
      <td>ログ出力や後処理などの副作用</td>
    </tr>
    <tr>
      <td>タイミング制御</td>
      <td><a href="./utility/delay.html">delay</a>, <a href="./utility/timeout.html">timeout</a>, <a href="./utility/takeUntil.html">takeUntil</a></td>
      <td>一定時間の遅延や終了制御</td>
    </tr>
    <tr>
      <td>リトライ・繰り返し</td>
      <td><a href="./utility/retry.html">retry</a>, <a href="./utility/repeat.html">repeat</a></td>
      <td>失敗時の再試行や繰り返し</td>
    </tr>
    <tr>
      <td>初期値・配列化</td>
      <td><a href="./utility/startWith.html">startWith</a>, <a href="./utility/toArray.html">toArray</a></td>
      <td>初期値の設定やすべてを配列にまとめる</td>
    </tr>
    <tr>
      <th scope="row" rowspan="2"><a href="./conditional/">条件</a></th>
      <td>条件分岐</td>
      <td><a href="./conditional/iif.html">iif</a>, <a href="./conditional/defer.html">defer</a></td>
      <td>条件により異なるObservableを生成</td>
    </tr>
    <tr>
      <td>存在チェック</td>
      <td><a href="./conditional/defaultIfEmpty.html">defaultIfEmpty</a>, <a href="./conditional/every.html">every</a>, <a href="./conditional/isEmpty.html">isEmpty</a></td>
      <td>値の有無や全件判定など</td>
    </tr>
    <tr>
      <th scope="row"><a href="./multicasting">マルチキャスト</a></th>
      <td>共有化</td>
      <td><code>share</code>, <code>publish</code> など</td>
      <td>Observableのストリームを複数購読者間で共有</td>
    </tr>
  </tbody>
  <tfoot>
    <tr>
    </tr>
  </tfoot>
</table>