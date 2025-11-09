---
description: タスクの分類（同期・マイクロタスク・マクロタスク）とRxJSの各スケジューラーとの対応関係を基本から解説します。JavaScriptのイベントループの仕組み、実行順序の違い、setTimeout、Promise、queueMicrotaskなどの実装と動作を理解し、RxJSのスケジューラー選択に活かせる知識を身につけます。
---

# タスクとスケジューラーの基礎知識

## 同期処理とは
同期処理は、コードが記述された順番に即座に実行され、前の処理が終わるまで次の処理に進みません。

#### 例
```ts
console.log('A');
console.log('B');
console.log('C');

// 出力:
// A
// B
// C
```


## 非同期処理とは
非同期処理は、すぐに実行されず、現在の同期処理が終わった後に実行される処理です。
非同期処理には「マクロタスク」と「マイクロタスク」が存在します。


## マクロタスク
- イベントループの次のサイクルで実行されるタスク。
- 例: `setTimeout`, `setInterval`, ブラウザイベント

#### 実行例
```ts
console.log('Start');
setTimeout(() => console.log('Macro Task'), 0);
console.log('End');

// 出力:
// Start
// End
// Micro Task
```

### RxJSにおける対応
- `asyncScheduler`
  - 内部で`setTimeout`を使用
  - マクロタスクとして動作

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hello')
  .pipe(observeOn(asyncScheduler))
  .subscribe(console.log);

// 出力:
// Hello
```


## マイクロタスク
- 現在のタスクが終了した直後に、次のタスクが始まる前に実行されるタスク。
- 例: `Promise.then`, `queueMicrotask`

#### 実行例
```ts
console.log('Start');
Promise.resolve().then(() => console.log('Micro Task'));
console.log('End');

// 出力:
// Start
// End
// Micro Task
```

### RxJSにおける対応
- `asapScheduler`
  - 内部で`Promise.resolve().then()`を使用
  - マイクロタスクとして動作

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hi')
  .pipe(observeOn(asapScheduler))
  .subscribe(console.log);

// 出力:
// Hi
```


## 同期タスク
- すぐに実行される通常のコード。

### RxJSにおける対応
- `queueScheduler`
  - 同期的に見えるが、タスクキューイングにより細かい制御が可能。

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Now')
  .pipe(observeOn(queueScheduler))
  .subscribe(console.log);

// 出力:
// Now
```


## 実行順序まとめ

#### コード例
```ts
console.log('1');

setTimeout(() => console.log('2 (setTimeout)'), 0);
Promise.resolve().then(() => console.log('3 (Promise)'));

console.log('4');

// 出力:
// 1
// 4
// 3 (Promise) 👈 マイクロタスク
// 2 (setTimeout) 👈 マクロタスク
```


## タスクとRxJSスケジューラー対応表

| 種類         | 例                          | RxJSスケジューラー  |
|--------------|------------------------------|---------------------|
| 同期処理     | 通常のコード                  | `queueScheduler`    |
| マイクロタスク | Promise.then, queueMicrotask | `asapScheduler`     |
| マクロタスク | setTimeout, setInterval      | `asyncScheduler`    |
