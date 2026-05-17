---
description: mergeScanは非同期の累積処理を行うRxJS変換オペレーターで、scanとmergeMapを組み合わせた動作をします。APIレスポンスの累積集計、前の結果に基づく次のリクエスト実行、ページネーションでの複数ページデータ累積取得など、非同期処理を伴う累積が必要な場面に最適です。concurrent パラメータで同時実行数の制御も可能です。
---

# mergeScan - 非同期処理を伴う累積

`mergeScan`オペレーターは、ストリームの各値に対して**非同期の累積処理**を実行します。
`scan`と`mergeMap`を組み合わせたような動作で、累積値を保持しながら、各値を新しいObservableに変換し、その結果を次の累積処理に使用します。

## 🔰 基本構文と使い方

```ts
import { interval, of } from 'rxjs';
import { mergeScan, take,  } from 'rxjs';

interval(1000).pipe(
  take(5),
  mergeScan((acc, curr) => {
    // 各値に対して非同期処理（ここでは即座に返す）
    return of(acc + curr);
  }, 0)
).subscribe(console.log);

// 出力: 0, 1, 3, 6, 10
```

- `acc`は累積値、`curr`は現在の値です。
- 累積関数は**Observableを返す**必要があります。
- 各値の処理結果は累積されていきます。

[🌐 RxJS公式ドキュメント - `mergeScan`](https://rxjs.dev/api/operators/mergeScan)

## 💡 典型的な活用パターン

- APIレスポンスを累積して集計する
- 前の結果に基づいて次のAPIリクエストを実行する
- リアルタイムデータの非同期累積処理
- ページネーションで複数ページのデータを累積取得

## 📊 scanとの違い

| オペレーター | 累積関数の戻り値 | ユースケース |
|--------------|------------------|--------------|
| `scan` | 直接値を返す | 同期的な累積処理 |
| `mergeScan` | Observableを返す | 非同期的な累積処理 |

```ts
// scan - 同期処理
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)

// mergeScan - 非同期処理
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr).pipe(delay(100)), 0)
)
```

## 🧠 実践コード例（API累積取得）

ボタンをクリックするたびに、前回の結果に新しいデータを追加していく例です。

```ts
import { fromEvent, of } from 'rxjs';
import { mergeScan, delay, take, map } from 'rxjs';

// ボタン作成
const button = document.createElement('button');
button.textContent = 'データ取得';
document.body.appendChild(button);

// 出力エリア作成
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// ダミーAPI（遅延してデータを返す）
const fetchData = (page: number) => {
  return of(`データ${page}`).pipe(delay(500));
};

// クリックイベントで累積取得
fromEvent(button, 'click').pipe(
  take(5), // 最大5回
  mergeScan((accumulated, _, index) => {
    const page = index + 1;
    console.log(`ページ${page}を取得中...`);

    // 前回までの累積データに新しいデータを追加
    return fetchData(page).pipe(
      map(newData => [...accumulated, newData])
    );
  }, [] as string[])
).subscribe((allData) => {
  output.innerHTML = `
    <div>取得済みデータ:</div>
    <ul>${allData.map(d => `<li>${d}</li>`).join('')}</ul>
  `;
});
```

- 各クリックで非同期にデータを取得します。
- 前回までの結果（`accumulated`）に新しいデータを追加していきます。
- **累積結果がリアルタイムで更新**されます。

## 🎯 実用例：並行制御付きの累積処理

`mergeScan`には`concurrent`パラメーターがあり、同時実行数を制御できます。

```ts
import { interval, of } from 'rxjs';
import { mergeScan, take, delay } from 'rxjs';

interface RequestLog {
  total: number;
  logs: string[];
}

interval(200).pipe(
  take(10),
  mergeScan((acc, curr) => {
    const timestamp = new Date().toLocaleTimeString();
    console.log(`リクエスト${curr}開始: ${timestamp}`);

    // 各リクエストは1秒かかる
    return of({
      total: acc.total + 1,
      logs: [...acc.logs, `リクエスト${curr}完了: ${timestamp}`]
    }).pipe(delay(1000));
  }, { total: 0, logs: [] } as RequestLog, 2) // 同時実行数2
).subscribe((result) => {
  console.log(`累計: ${result.total}件`);
  console.log(result.logs[result.logs.length - 1]);
});
```

- `concurrent: 2`により、最大2つまで同時にリクエストを実行します。
- 3つ目以降のリクエストは、前のリクエストが完了するまで待機します。

## ⚠️ 注意点

### 1. エラーハンドリング

累積関数内でエラーが発生すると、ストリーム全体が停止します。

```ts
source$.pipe(
  mergeScan((acc, curr) => {
    return apiCall(curr).pipe(
      map(result => acc + result),
      catchError((err: unknown) => {
        const message = err instanceof Error ? err.message : String(err);
        console.error('エラー発生:', message);
        // 累積値を維持して続行
        return of(acc);
      })
    );
  }, 0)
)
```

### 2. メモリ管理

累積値が大きくなりすぎないよう注意

```ts
// 悪い例：無制限に累積
mergeScan((acc, curr) => of([...acc, curr]), [])

// 良い例：最新N件のみ保持
mergeScan((acc, curr) => {
  const newAcc = [...acc, curr];
  return of(newAcc.slice(-100)); // 最新100件のみ
}, [])
```

### 3. 同期的な処理ならscanを使う

非同期処理が不要な場合は、シンプルな`scan`を使いましょう。

```ts
// mergeScanは不要
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr), 0)
)

// scanで十分
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)
```

## 🔗 関連オペレーター

- [`scan`](./scan.md) - 同期的な累積処理
- [`reduce`](./reduce.md) - 完了時のみ最終累積値を出力
- [`mergeMap`](./mergeMap.md) - 非同期マッピング（累積なし）
- [`expand`](./expand.md) - 再帰的な展開処理
