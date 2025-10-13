---
description: forkJoin Creation Functionは、複数のObservableがすべて完了した後に、それぞれの最後の値をまとめて配列やオブジェクトとして出力します。複数のAPIリクエストを並列実行し、すべての結果が揃ってから処理したい場合に最適です。
---

# forkJoin - すべての最後の値をまとめて出力する

`forkJoin` は、複数のObservableが**すべて完了した後に、それぞれの最後の値をまとめて配列やオブジェクトとして出力**するCreation Functionです。
「すべて揃ってからまとめて使いたい」場合に非常に便利です。


## 🔰 基本構文と使い方

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

const user$ = of('ユーザーA').pipe(delay(1000));
const posts$ = of('投稿リスト').pipe(delay(1500));

forkJoin([user$, posts$]).subscribe(([user, posts]) => {
  console.log(user, posts);
});

// 出力:
// ユーザーA 投稿リスト
```

- すべてのObservableが`complete`するまで待機します。
- 各Observableの**最後の発行値だけ**がまとめられて出力されます。

[🌐 RxJS公式ドキュメント - `forkJoin`](https://rxjs.dev/api/index/function/forkJoin)


## 💡 典型的な活用パターン

- **複数のAPIリクエストを並列に実行して、すべての結果をまとめる**
- **初期ロード時に必要な複数データセットを一括取得**
- **関連するデータをまとめて取得し、まとめて画面描画**


## 🧠 実践コード例（UI付き）

複数のAPIリクエストをシミュレートして、すべての結果が揃ったらまとめて表示します。

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>forkJoin の実践例:</h3>';
document.body.appendChild(output);

// ダミーデータストリーム
const user$ = of({ id: 1, name: '山田太郎' }).pipe(delay(2000));
const posts$ = of([{ id: 1, title: '投稿1' }, { id: 2, title: '投稿2' }]).pipe(delay(1500));
const weather$ = of({ temp: 22, condition: '晴れ' }).pipe(delay(1000));

// ローディングメッセージ
const loading = document.createElement('div');
loading.textContent = 'データを読み込み中...';
loading.style.color = 'blue';
output.appendChild(loading);

// すべてのリクエストが完了後にまとめて出力
forkJoin({
  user: user$,
  posts: posts$,
  weather: weather$
}).subscribe(result => {
  output.removeChild(loading);
  
  const pre = document.createElement('pre');
  pre.textContent = JSON.stringify(result, null, 2);
  pre.style.background = '#f5f5f5';
  pre.style.padding = '10px';
  pre.style.borderRadius = '5px';
  output.appendChild(pre);
  
  const summary = document.createElement('div');
  summary.textContent = `ユーザー: ${result.user.name}、天気: ${result.weather.condition}、投稿数: ${result.posts.length}`;
  output.appendChild(summary);
});
```

- 最初にローディング表示、
- すべてのデータが揃ったら、まとめて結果を描画します。

