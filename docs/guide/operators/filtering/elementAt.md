---
description: elementAtオペレーターは、指定されたインデックス位置の値のみを取得するRxJSフィルタリングオペレーターです。配列のインデックスアクセスに似た動作をします。
---

# elementAt - インデックス指定で取得

`elementAt` オペレーターは、Observable から**指定されたインデックス位置の値のみ**を取得し、即座にストリームを完了させます。配列の `array[index]` に似た動作をします。

## 🔰 基本構文と使い方

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// 出力: 30（インデックス2の値）
```

**動作の流れ**:
1. 10（インデックス0）→ スキップ
2. 20（インデックス1）→ スキップ
3. 30（インデックス2）→ 出力して完了
4. 40, 50 は評価されない

[🌐 RxJS公式ドキュメント - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 典型的な活用パターン

- **ページネーション**：特定ページの最初の項目を取得
- **順序保証データの取得**：N番目のイベントやメッセージを取得
- **テストとデバッグ**：特定位置の値を検証
- **配列的なアクセス**：Observable を配列のように扱う

## 🧠 実践コード例1: イベントのカウントダウン

N回目のクリックでアクションを実行する例です。

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UI作成
const output = document.createElement('div');
output.innerHTML = '<h3>5回クリックでメッセージ表示</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'クリック';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'あと5回クリックしてください';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// クリックイベント
const clicks$ = fromEvent(button, 'click');

// カウント表示用
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `あと${remaining}回クリックしてください`;
  } else {
    counter.textContent = '';
  }
});

// 5回目（インデックス4）のクリックを検出
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 達成しました！';
  result.style.color = 'green';
  button.disabled = true;
});
```

- 5回目（インデックス4）のクリックで完了します。
- 配列のインデックスと同じく0から始まります。

## 🎯 実践コード例2: データストリームからN番目を取得

一定間隔で発行されるデータから特定の順番の値を取得する例です。

```ts
import { interval } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UI作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'データストリームからN番目を取得';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'インデックスを入力（0〜9）';
input.min = '0';
input.max = '9';
input.style.marginRight = '10px';
container.appendChild(input);

const getButton = document.createElement('button');
getButton.textContent = '取得';
container.appendChild(getButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// データストリーム（0.5秒ごとに値を発行、10個まで）
const data$ = interval(500).pipe(
  map(i => ({ index: i, value: Math.floor(Math.random() * 100), timestamp: Date.now() }))
);

getButton.addEventListener('click', () => {
  const index = parseInt(input.value);

  if (isNaN(index) || index < 0 || index > 9) {
    status.textContent = '0〜9の範囲で入力してください';
    status.style.color = 'red';
    return;
  }

  status.textContent = `インデックス ${index} の値を取得中...`;
  status.style.color = 'blue';
  result.style.display = 'none';
  getButton.disabled = true;
  input.disabled = true;

  data$.pipe(
    elementAt(index)
  ).subscribe({
    next: data => {
      status.textContent = '';
      result.style.display = 'block';
      result.innerHTML = `
        <strong>✅ 取得成功</strong><br>
        インデックス: ${data.index}<br>
        値: ${data.value}<br>
        タイムスタンプ: ${new Date(data.timestamp).toLocaleTimeString()}
      `;
      result.style.color = 'green';
      result.style.backgroundColor = '#e8f5e9';
      getButton.disabled = false;
      input.disabled = false;
    },
    error: err => {
      status.textContent = '';
      result.style.display = 'block';
      result.textContent = `❌ エラー: ${err.message}`;
      result.style.color = 'red';
      result.style.backgroundColor = '#ffebee';
      getButton.disabled = false;
      input.disabled = false;
    }
  });
});
```

- 0.5秒ごとに発行されるストリームから指定したインデックスの値を取得します。
- インデックスが範囲外の場合はエラーが発生します。

## 🆚 類似オペレーターとの比較

### elementAt vs take vs first

```ts
import { from } from 'rxjs';
import { elementAt, take, first, skip } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// elementAt: 特定のインデックスの値のみ取得
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// 出力: 30

// take: 最初からN個取得
numbers$.pipe(
  take(3)
).subscribe(console.log);
// 出力: 10, 20, 30

// skip + first: elementAt と同等（冗長）
numbers$.pipe(
  skip(2),
  first()
).subscribe(console.log);
// 出力: 30
```

| オペレーター | 取得する値 | 出力数 | ユースケース |
|:---|:---|:---|:---|
| `elementAt(n)` | インデックスnの値のみ | 1つ | N番目の値を取得 |
| `take(n)` | 最初からn個 | n個 | 最初のN個を取得 |
| `first()` | 最初の値 | 1つ | 最初の1つを取得 |
| `skip(n) + first()` | n個スキップ後の最初 | 1つ | elementAtと同等（非推奨） |

## ⚠️ 注意点

### 1. インデックスが範囲外の場合

ストリームが完了する前に指定したインデックスに到達しない場合、エラーが発生します。

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // 3つしかない

numbers$.pipe(
  elementAt(5) // インデックス5を要求
).subscribe({
  next: console.log,
  error: err => console.error('エラー:', err.message)
});
// 出力: エラー: no elements in sequence
```

### 2. デフォルト値の指定

エラーを防ぐために、デフォルト値を指定できます。

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// デフォルト値を指定
numbers$.pipe(
  elementAt(5, 999) // インデックス5が存在しない場合は999を返す
).subscribe({
  next: console.log,
  error: err => console.error('エラー:', err.message)
});
// 出力: 999
```

### 3. 非同期ストリームでの使用

非同期ストリームでは、インデックス位置に到達するまで待機します。

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// 1秒ごとに値を発行
interval(1000).pipe(
  elementAt(3) // インデックス3（4番目の値）
).subscribe(console.log);
// 3秒後に出力: 3
```

### 4. 負のインデックスは使用不可

負のインデックスは指定できません。

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ❌ 負のインデックスはエラー
numbers$.pipe(
  elementAt(-1)
).subscribe({
  next: console.log,
  error: err => console.error('エラー:', err.message)
});
// エラー: ArgumentOutOfRangeError: index out of range
```

配列の最後から取得したい場合は `takeLast` や `last` を使用してください。

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ 最後の値を取得
numbers$.pipe(
  last()
).subscribe(console.log);
// 出力: 50

// ✅ 最後のN個を取得
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// 出力: 40, 50
```

## 📚 関連オペレーター

- **[take](./take)** - 最初からN個取得
- **[first](./first)** - 最初の値を取得
- **[last](./last)** - 最後の値を取得
- **[skip](./skip)** - 最初のN個をスキップ
- **[takeLast](./takeLast)** - 最後のN個を取得

## まとめ

`elementAt` オペレーターは、指定されたインデックス位置の値のみを取得します。

- ✅ 配列のインデックスアクセスと同じ動作
- ✅ N番目の値を取得するのに最適
- ✅ デフォルト値を指定してエラーを回避可能
- ⚠️ インデックスが範囲外の場合はエラー（デフォルト値なし）
- ⚠️ 負のインデックスは使用不可
- ⚠️ 非同期ストリームでは到達するまで待機
