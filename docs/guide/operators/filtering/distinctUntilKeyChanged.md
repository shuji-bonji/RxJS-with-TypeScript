# distinctUntilKeyChanged - 特定プロパティの変更のみ検出する

`distinctUntilKeyChanged` オペレーターは、オブジェクトの特定のキー（プロパティ）に注目し、その値が前回と異なる場合のみ出力します。  
連続した重複を効率的にスキップする際に便利です。
 

## 🔰 基本構文と使い方

```ts
import { from } from 'rxjs';
import { distinctUntilKeyChanged } from 'rxjs/operators';

const users = [
  { id: 1, name: '田中' },
  { id: 2, name: '田中' }, // nameが同じなのでスキップ
  { id: 3, name: '佐藤' },
  { id: 4, name: '鈴木' },
  { id: 5, name: '鈴木' }, // nameが同じなのでスキップ
  { id: 6, name: '田中' }
];

from(users).pipe(
  distinctUntilKeyChanged('name')
).subscribe(console.log);

// 出力:
// { id: 1, name: '田中' }
// { id: 3, name: '佐藤' }
// { id: 4, name: '鈴木' }
// { id: 6, name: '田中' }
```

- 指定されたプロパティ`name`の値が変化したときだけ出力します。
- その他のプロパティ（例えば`id`など）は比較対象になりません。

[🌐 RxJS公式ドキュメント - `distinctUntilKeyChanged`](https://rxjs.dev/api/operators/distinctUntilKeyChanged)
 

## 💡 典型的な活用パターン

- リスト表示で、特定プロパティが変化したときのみ更新する
- イベントストリームで、特定属性の変化のみ検出する
- 重複排除をキー単位で制御したい場合に利用する
 

## 🧠 実践コード例（UI付き）

テキストボックスに名前を入力し、Enterキーを押すと登録されます。  
**連続して同じ名前が入力された場合は無視**され、異なる名前が入力されたときだけリストに追加されます。

```ts
import { fromEvent } from 'rxjs';
import { map, filter, scan, distinctUntilKeyChanged } from 'rxjs/operators';

// 出力領域作成
const output = document.createElement('div');
document.body.appendChild(output);

const title = document.createElement('h3');
title.textContent = 'distinctUntilKeyChanged の実践例';
output.appendChild(title);

// 入力フォーム
const input = document.createElement('input');
input.placeholder = '名前を入力してEnter';
document.body.appendChild(input);

// 入力イベントストリーム
fromEvent<KeyboardEvent>(input, 'keydown').pipe(
  filter((e) => e.key === 'Enter'),
  map(() => input.value.trim()),
  filter((name) => name.length > 0),
  scan((_, name, index) => ({ id: index + 1, name }), { id: 0, name: '' }),
  distinctUntilKeyChanged('name')
).subscribe((user) => {
  const item = document.createElement('div');
  item.textContent = `ユーザー入力: ID=${user.id}, 名前=${user.name}`;
  output.appendChild(item);
});
```

- 連続して同じ名前が入力された場合はスキップされます。
- 新しい名前が入力されたときのみ表示されます。
