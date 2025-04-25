# tap - サイドエフェクトの実行

`tap`オペレーターは、「ストリームを変更せずにサイドエフェクト（副作用）を実行する」ために使用します。
ログの出力、デバッグ、または値に影響を与えないその他の操作に最適です。

## 🔰 基本構文・動作

値の流れを変更せず、副作用だけを加えたい場面で活用されます。

```ts
import { of, tap } from 'rxjs';

of(42).pipe(
  tap(value => console.log('tap:', value))
).subscribe();
// 出力:
// tap: 42
```

この例では、`of(42)`から発行された値が `tap` を通過する際にログが出力されます。  
tapは値を「そのまま通す」ため、ストリームの内容には影響を与えません。


## 💡 典型的な活用例

`tap`は以下のような目的でよく使われます。

- デバッグ・ロギング
- ローディング状態の切り替え
- トースト通知の表示
- UI更新のトリガー

```ts
import { of, tap, map } from 'rxjs';

of(Math.random()).pipe(
  tap(val => console.log('取得した値:', val)),
  map(n => n > 0.5 ? 'High' : 'Low'),
  tap(label => console.log('ラベル:', label))
).subscribe();
// 出力:
// 取得した値: 0.09909888881113504
// ラベル: Low
```


## 🧪 実践コード例（UI付き）

以下は、tapを使ってDOMにログを追加する例です。

```ts
import { of } from 'rxjs';
import { tap, map } from 'rxjs/operators';

// ログ出力用の要素
const logOutput = document.createElement('div');
document.body.appendChild(logOutput);

// 値のシーケンス
of(1, 2, 3, 4, 5)
  .pipe(
    tap((val) => {
      console.log(`元の値: ${val}`);

      // UIにログを追加
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: 値 ${val} が通過`;
      logEntry.style.color = '#666';
      logOutput.appendChild(logEntry);
    }),
    map((val) => val * 10),
    tap((val) => {
      console.log(`変換後の値: ${val}`);

      // UIにログを追加
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: 変換後の値 ${val}`;
      logEntry.style.color = '#090';
      logOutput.appendChild(logEntry);
    })
  )
  .subscribe((val) => {
    // 最終結果をUIに表示
    const resultItem = document.createElement('div');
    resultItem.textContent = `結果: ${val}`;
    resultItem.style.fontWeight = 'bold';
    logOutput.appendChild(resultItem);
  });

```


## ✅ まとめ

- `tap`は**副作用の挿入**に特化した演算子
- 値の流れを変えずに**ログ出力やUI更新**ができる
- `finalize`や`catchError`と組み合わせることで、より実践的な制御が可能