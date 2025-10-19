---
description: RxJSのマーブル図を直感的に理解するための実践的な方法と、インタラクティブな可視化ツールを紹介します
---

# マーブル図を直感的に理解する

## なぜマーブル図は分かりにくい？

RxJSの公式ドキュメントでよく見かけるマーブル図（Marble Diagram）は、慣れないと理解が難しいです。

```
--1--2--3--4--5--|  (source)
----x------------   (trigger)
----[1,2,3]--[4,5]| (output)
```

### 理解を妨げる要因

1. **抽象的すぎる** - 実際のコードとの関連が見えにくい
2. **静的である** - 時間の流れや動きが分からない
3. **横向きの時間軸** - コードは縦に書くのに図は横向き

## 🎯 より直感的な理解方法

### 方法1: 実世界のアナロジーで考える

#### Buffer = 貯金箱 💰
```typescript
// bufferは「貯金箱」のような動き
// 1. コインを入れる = 値が流れてくる
// 2. 貯金箱を開ける = トリガーイベント
// 3. 中身を取り出す = バッファの内容を配列で放出
```

#### 実例：チャットメッセージのバッチ送信
```typescript
import { fromEvent, interval } from 'rxjs';
import { buffer } from 'rxjs/operators';

// ユーザーがメッセージを入力（ストリーム）
const messageInput$ = fromEvent(inputField, 'input');

// 送信ボタンがトリガー
const sendButton$ = fromEvent(sendButton, 'click');

// バッファリングして一括送信
messageInput$.pipe(
  buffer(sendButton$)  // 送信ボタンを押すまで貯める
).subscribe(messages => {
  console.log('一括送信:', messages);
  // APIに配列として送信
});
```

### 方法2: 絵文字で視覚化 

```
📥 入力ストリーム
↓
📦 バッファに貯まる
↓
🔨 トリガー！
↓
📤 [値1, 値2, 値3] まとめて出力
```

### 方法3: 縦型タイムラインで理解

従来の横向きマーブル図の代わりに、縦向きで表現

```
時刻 | ソース | バッファ | トリガー | 出力
-----|--------|----------|----------|--------
0秒  |   1    |   [1]    |          |
1秒  |   2    |  [1,2]   |          |
2秒  |   3    | [1,2,3]  |   🔨     | [1,2,3]
3秒  |   4    |   [4]    |          |
4秒  |   5    |  [4,5]   |   🔨     | [4,5]
```

## 📊 インタラクティブに学ぶ

### Step-by-Step デバッグ法

```typescript
import { interval, Subject } from 'rxjs';
import { buffer, tap } from 'rxjs/operators';

const trigger$ = new Subject();

interval(1000).pipe(
  tap(val => console.log(`📥 ソース値: ${val}`)),
  buffer(trigger$),
  tap(buffered => console.log(`📤 バッファ放出:`, buffered))
).subscribe();

// コンソールで手動トリガー
setTimeout(() => {
  console.log('🔨 トリガー発火！');
  trigger$.next();
}, 3500);
```

実行すると
```
📥 ソース値: 0
📥 ソース値: 1
📥 ソース値: 2
🔨 トリガー発火！
📤 バッファ放出: [0, 1, 2]
📥 ソース値: 3
📥 ソース値: 4
...
```

## 🛠️ 実践演習

### 演習1: クリックイベントをバッファリング

```typescript
import { fromEvent, interval } from 'rxjs';
import { buffer, map } from 'rxjs/operators';

// 1秒ごとのタイマーをトリガーとして使用
const timer$ = interval(1000);

// クリックイベントを1秒ごとにまとめる
fromEvent(document, 'click').pipe(
  buffer(timer$),
  map(clicks => clicks.length)
).subscribe(clickCount => {
  console.log(`過去1秒間のクリック数: ${clickCount}`);
});
```

### 演習2: フォーム入力の自動保存

```typescript
import { fromEvent, interval } from 'rxjs';
import { buffer, filter } from 'rxjs/operators';

const input$ = fromEvent(formElement, 'input');
const autoSave$ = interval(5000); // 5秒ごと

input$.pipe(
  buffer(autoSave$),
  filter(changes => changes.length > 0) // 変更があった場合のみ
).subscribe(changes => {
  console.log('自動保存:', changes.length, '件の変更');
  // APIに送信
});
```

## 💡 理解度チェックリスト

```markdown
- [ ] bufferが「値を貯める箱」であることを理解した
- [ ] トリガーが「箱を開けるタイミング」であることを理解した
- [ ] 出力が常に配列であることを理解した
- [ ] 空のバッファの場合、空配列が出力されることを理解した
- [ ] 実際のコードで動作を確認した
```

## 🎮 インタラクティブツール

以下のツールで実際に動きを確認できます。

- [RxJS Marbles](https://rxmarbles.com/) - インタラクティブなマーブル図
- [RxViz](https://rxviz.com/) - コードを書いて可視化
- [Rx Visualizer](https://rxjs-visualize.netlify.app/) - アニメーション付き

## よくある間違いと解決法

### 間違い1: トリガーなしで値が出力されると思う
```typescript
// ❌ これだけでは何も出力されない
source$.pipe(buffer(trigger$)).subscribe(console.log);

// ✅ トリガーを発火させる必要がある
trigger$.next();
```

### 間違い2: 単一の値が出力されると思う
```typescript
// bufferの出力は常に配列
buffer(trigger$) // 出力: [値1, 値2, ...] （配列）
// 単一の値ではない
```

## まとめ

マーブル図は最初は難しく感じますが、以下の方法で理解しやすくなります。

1. **実世界のアナロジー**で考える（貯金箱、バケツなど）
2. **縦型のタイムライン**で時間の流れを追う
3. **console.log**で各ステップを確認
4. **インタラクティブツール**で視覚的に学ぶ
5. **実践的な例**（チャット、フォームなど）で応用

重要なのは、抽象的な図を具体的なコードと動作に結びつけることです。