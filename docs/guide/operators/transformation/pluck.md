# pluck - オブジェクトから特定プロパティを抽出する

`pluck`オペレーターは、オブジェクトストリーム内から特定のプロパティだけを取り出すために使用します。  
複雑なオブジェクトの中から必要な値だけを簡単に取り出せる便利な変換オペレーターです。


## 🔰 基本構文と使い方

```ts
import { from } from 'rxjs';
import { pluck } from 'rxjs/operators';

const users$ = from([
  { id: 1, name: '田中', age: 28 },
  { id: 2, name: '佐藤', age: 34 },
  { id: 3, name: '鈴木', age: 22 }
]);

users$.pipe(
  pluck('name')
).subscribe(console.log);
// 出力: 田中, 佐藤, 鈴木
```
各オブジェクトから `name` プロパティのみを抽出して出力します。

[🌐 RxJS公式ドキュメント - `pluck`](https://rxjs.dev/api/operators/pluck)

## 💡 典型的な活用パターン

- APIレスポンスから特定のフィールドだけを抽出
- イベントオブジェクトから必要なプロパティを取得
- 入れ子になったオブジェクト構造からネストされた値を取り出す

## 🧠 実践コード例（UI付き）

マウスクリック位置（x座標）だけをリアルタイムで取得して表示する例です。

```ts
import { fromEvent } from 'rxjs';
import { pluck } from 'rxjs/operators';

// 出力エリア作成
const output = document.createElement('div');
output.style.marginTop = '20px';
document.body.appendChild(output);

// クリックイベントストリーム
fromEvent<MouseEvent>(document, 'click')
  .pipe(pluck('clientX'))
  .subscribe((x) => {
    output.textContent = `クリック位置 (X座標): ${x}`;
  });

```

- クリックされた位置のX座標だけを抽出してリアルタイム表示します。
- オブジェクトから必要な値だけを効率よく取り出す使い方が体験できます。
