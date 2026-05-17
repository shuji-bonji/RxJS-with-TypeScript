---
description: raceWithは元のObservableと他のObservableのうち最初に値を発行したストリームのみを採用するRxJS結合オペレーターです。タイムアウト実装、フォールバック処理、複数データソースからの最速採用など、競争条件による選択が必要な場面に最適です。pipeable operator版でパイプライン内での使用が便利です。
---

# raceWith - 最速ストリームを採用

`raceWith` オペレーターは、元のObservableと指定された他のObservableのうち、**最初に値を発行したObservableだけを採用**し、
それ以降は他のObservableを無視します。
これは Creation Function の `race` のPipeable Operator版です。

## 🔰 基本構文と使い方

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'ゆっくり (5秒)'));
const medium$ = timer(3000).pipe(map(() => '普通 (3秒)'));
const fast$ = timer(2000).pipe(map(() => '速い (2秒)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// 出力: 速い (2秒)
```

- 最初に値を発行したObservable（この例では`fast$`）だけが勝者となり、その後のストリームを継続します。
- 他のObservableは無視されます。

[🌐 RxJS公式ドキュメント - `raceWith`](https://rxjs.dev/api/operators/raceWith)


## 💡 典型的な活用パターン

- **タイムアウト実装**：メイン処理とタイムアウトタイマーを競争させる
- **フォールバック処理**：複数のデータソースから最速のものを採用
- **ユーザーインタラクション**：クリックと自動進行の早い方を採用


## 🧠 実践コード例（UI付き）

手動クリックと自動進行タイマーを競争させ、早い方を採用する例です。

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

// 出力エリア作成
const output = document.createElement('div');
output.innerHTML = '<h3>raceWith の実践例:</h3>';
document.body.appendChild(output);

// ボタン作成
const button = document.createElement('button');
button.textContent = '手動で進む（5秒以内にクリック）';
document.body.appendChild(button);

// 待機メッセージ
const waiting = document.createElement('div');
waiting.textContent = '5秒以内にボタンをクリックするか、自動で進むのを待ってください...';
waiting.style.marginTop = '10px';
output.appendChild(waiting);

// 手動クリックストリーム
const manualClick$ = fromEvent(button, 'click').pipe(
  take(1),
  map(() => '👆 手動クリックが選択されました！')
);

// 自動進行タイマー（5秒後）
const autoProgress$ = timer(5000).pipe(
  map(() => '⏰ 自動進行が選択されました！')
);

// レース実行
manualClick$
  .pipe(raceWith(autoProgress$))
  .subscribe((winner) => {
    waiting.remove();
    button.disabled = true;

    const result = document.createElement('div');
    result.innerHTML = `<strong>${winner}</strong>`;
    result.style.color = 'green';
    result.style.fontSize = '18px';
    result.style.marginTop = '10px';
    output.appendChild(result);
  });
```

- ボタンを5秒以内にクリックすると手動クリックが採用されます。
- 5秒経過すると自動進行が採用されます。
- **早い方が勝者**となり、遅い方は無視されます。


## 🔄 Creation Function `race` との違い

### 基本的な違い

| | `race` (Creation Function) | `raceWith` (Pipeable Operator) |
|:---|:---|:---|
| **使用場所** | 独立した関数として使用 | `.pipe()` チェーン内で使用 |
| **記述方法** | `race(obs1$, obs2$, obs3$)` | `obs1$.pipe(raceWith(obs2$, obs3$))` |
| **最初のストリーム** | すべて対等に扱う | メインストリームとして扱う |
| **利点** | シンプルで読みやすい | 他のオペレーターと組み合わせやすい |

### 使い分けの具体例

**シンプルなレースだけなら Creation Function がおすすめ**

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const server1$ = timer(3000).pipe(map(() => 'サーバー1からの応答'));
const server2$ = timer(2000).pipe(map(() => 'サーバー2からの応答'));
const server3$ = timer(4000).pipe(map(() => 'サーバー3からの応答'));

// シンプルで読みやすい
race(server1$, server2$, server3$).subscribe(response => {
  console.log('採用:', response);
});
// 出力: 採用: サーバー2からの応答 (最速の2秒)
```

**メインストリームに変換処理を加える場合は Pipeable Operator がおすすめ**

```ts
import { fromEvent, timer, of } from 'rxjs';
import { raceWith, map, switchMap, catchError } from 'rxjs';

const searchButton = document.createElement('button');
searchButton.textContent = '検索';
document.body.appendChild(searchButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// メインストリーム: ユーザーの検索リクエスト
const userSearch$ = fromEvent(searchButton, 'click').pipe(
  switchMap(() => {
    output.textContent = '検索中...';

    // API呼び出しをシミュレート（3秒かかる）
    return timer(3000).pipe(
      map(() => '🔍 検索結果: 100件ヒット'),
      catchError((err: unknown) => of('❌ エラーが発生しました'))
    );
  })
);

// ✅ Pipeable Operator版 - 一つのパイプラインで完結
userSearch$
  .pipe(
    raceWith(
      // タイムアウト（2秒）
      timer(2000).pipe(
        map(() => '⏱️ タイムアウト: 検索に時間がかかっています')
      )
    )
  )
  .subscribe(result => {
    output.textContent = result;
  });

// ❌ Creation Function版 - メインストリームを分けて書く必要がある
import { race } from 'rxjs';
race(
  userSearch$,
  timer(2000).pipe(
    map(() => '⏱️ タイムアウト: 検索に時間がかかっています')
  )
).subscribe(result => {
  output.textContent = result;
});
```

**フォールバック処理の実装**

```ts
import { timer, throwError } from 'rxjs';
import { raceWith, map, mergeMap, catchError, delay } from 'rxjs';
import { of } from 'rxjs';

// UI作成
const output = document.createElement('div');
output.innerHTML = '<h3>データ取得（フォールバック付き）</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'データ取得開始';
document.body.appendChild(button);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
output.appendChild(statusArea);

button.addEventListener('click', () => {
  statusArea.textContent = '取得中...';

  // メインAPI（優先）：時々失敗する
  const mainApi$ = timer(1500).pipe(
    mergeMap(() => {
      const success = Math.random() > 0.5;
      if (success) {
        return of('✅ メインAPIから取得成功');
      } else {
        return throwError(() => new Error('メインAPI失敗'));
      }
    }),
    catchError((err: unknown) => {
      console.log('メインAPI失敗、フォールバックへ...');
      // エラー時は遅延してフォールバックに譲る
      return of('').pipe(delay(10000));
    })
  );

  // ✅ Pipeable Operator版 - メインAPIにフォールバックを追加
  mainApi$
    .pipe(
      raceWith(
        // バックアップAPI（フォールバック）：少し遅いが確実
        timer(2000).pipe(
          map(() => '🔄 バックアップAPIから取得')
        )
      )
    )
    .subscribe(result => {
      if (result) {
        statusArea.textContent = result;
        statusArea.style.color = result.includes('メイン') ? 'green' : 'orange';
      }
    });
});
```

**複数のデータソースから最速を採用**

```ts
import { timer, fromEvent } from 'rxjs';
import { raceWith, map, mergeMap } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>複数CDNから最速ロード</h3>';
document.body.appendChild(output);

const loadButton = document.createElement('button');
loadButton.textContent = 'ライブラリをロード';
document.body.appendChild(loadButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
output.appendChild(result);

fromEvent(loadButton, 'click').pipe(
  mergeMap(() => {
    result.textContent = 'ロード中...';

    // CDN1からロード（シミュレート）
    const cdn1$ = timer(Math.random() * 3000).pipe(
      map(() => ({ source: 'CDN1 (US)', data: 'library.js' }))
    );

    // ✅ Pipeable Operator版 - CDN1をメインに他のCDNを競争相手として追加
    return cdn1$.pipe(
      raceWith(
        // CDN2からロード（シミュレート）
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN2 (EU)', data: 'library.js' }))
        ),
        // CDN3からロード（シミュレート）
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN3 (Asia)', data: 'library.js' }))
        )
      )
    );
  })
).subscribe(response => {
  result.innerHTML = `
    <strong>✅ ロード完了</strong><br>
    取得元: ${response.source}<br>
    ファイル: ${response.data}
  `;
  result.style.color = 'green';
});
```

### まとめ

- **`race`**: 複数のストリームから最速のものをシンプルに採用するだけなら最適
- **`raceWith`**: メインストリームに対して変換や処理を加えながらタイムアウトやフォールバックを実装したい場合に最適


## ⚠️ 注意点

### タイムアウト実装の例

`raceWith` を使ったタイムアウト処理の実装

```ts
import { of, timer, throwError } from 'rxjs';
import { raceWith, delay, mergeMap } from 'rxjs';

// 時間のかかる処理（3秒）
const slowRequest$ = of('データ取得成功').pipe(delay(3000));

// タイムアウト（2秒）
const timeout$ = timer(2000).pipe(
  mergeMap(() => throwError(() => new Error('タイムアウト')))
);

slowRequest$
  .pipe(raceWith(timeout$))
  .subscribe({
    next: console.log,
    error: err => console.error(err.message)
  });
// 出力: タイムアウト
```

### すべてのストリームが購読される

`raceWith` は勝者が決まるまで、すべてのObservableを購読します。
勝者が決まった後、負けたObservableは自動的に購読解除されます。

```ts
import { timer } from 'rxjs';
import { raceWith, tap, map } from 'rxjs';

const slow$ = timer(3000).pipe(
  tap(() => console.log('slow$ 発火')),
  map(() => 'slow')
);

const fast$ = timer(1000).pipe(
  tap(() => console.log('fast$ 発火')),
  map(() => 'fast')
);

slow$.pipe(raceWith(fast$)).subscribe(console.log);
// 出力:
// fast$ 発火
// fast
// (slow$は1秒時点で購読解除され、3秒後には発火しない)
```

### 同期的なObservableの場合

すべてが同期的に発行される場合、最初に登録されたものが勝者になります。

```ts
import { of } from 'rxjs';
import { raceWith } from 'rxjs';

of('A').pipe(
  raceWith(of('B'), of('C'))
).subscribe(console.log);
// 出力: A (最初に登録されたため)
```


## 📚 関連オペレーター

- **[race](/guide/creation-functions/selection/race)** - Creation Function版
- **[timeout](/guide/operators/utility/timeout)** - タイムアウト専用オペレーター
- **[mergeWith](/guide/operators/combination/mergeWith)** - すべてのストリームをマージ
