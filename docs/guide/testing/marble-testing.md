---
description: "マーブルテストは、RxJSの非同期ストリームを文字列で視覚的に表現しながらテストできる手法です。ColdとHot Observableの違い、マーブル記法のルール、TestSchedulerの活用、複雑な非同期処理の検証方法をTypeScriptコード例で解説します。"
---

# マーブルテスト入門

RxJSでは、非同期ストリームの挙動を**視覚的に表現しながら**テストできる「マーブルテスト」という手法が用意されています。

ここでは、マーブルテストの基礎から、簡単な例を通して学びます。

## マーブル記法とは？

マーブル記法は、**時間経過とイベント発生**を文字列で表現する方法です。

### 基本ルール

| 記号 | 意味 |
|:----|:----|
| `-` | 時間の経過（1フレーム進む） |
| `a`, `b`, `c` | 発行された値（任意の文字） |
| `|` | 完了（complete） |
| `#` | エラー（error） |

例えば

```text
--a-b--c-|
```
これは、
- 2フレーム待って`a`が発行
- 1フレーム後に`b`
- 2フレーム後に`c`
- さらに1フレーム後に完了
を意味します。

## ColdとHotの違い

### Cold Observable

Cold Observableは「購読ごとに最初から再生」されます。

```ts
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('TestSchedulerの基本', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('簡単なテスト', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b--c|');
    
      expectObservable(source$).toBe('--a--b--c|');
    });
  });
});

```

### Hot Observable

Hot Observableは「既に進行中」のストリームです。  
途中から購読した場合、その時点以降の値しか受け取れません。

```ts
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('TestSchedulerの基本', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('簡単なテスト', () => {
    testScheduler.run(({ hot, expectObservable }) => {
      const source$ = hot('--a--b--c|');
    
      expectObservable(source$, '----^').toBe('-----b--c|');
    });
  });
});

```

## マーブルテストの簡単な例

たとえば、`debounceTime`オペレーターをテストする場合

```ts
import { debounceTime } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('TestSchedulerの基本', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('簡単なテスト', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20)
      );
    
      const expected =    '-----------(c|)';
    
      expectObservable(result$).toBe(expected, { c: 'c' });
    });
  });
});

```

ここでは、最後に発行された`c`だけが出力されることを検証しています。

## 注意点

- マーブル記法の1文字はデフォルトで**1フレーム（10ms）**を表します（環境によって設定可）
- `debounceTime`, `delay`, `interval`など、**時間依存のオペレーターはマーブルテストと相性が良い**です
- `expectObservable`を使ってストリームの出力を検証します
- `expectSubscriptions`は、サブスクリプションのタイミングを検証する上級機能ですが、ここでは扱いません

## まとめ

マーブルテストは、RxJSコードのテストを**可視化し直感的**に理解できる非常に強力な手法です。

- **ColdとHotの違いを意識する**
- **時間経過とイベントを文字列で表現する**
- **複雑な非同期ストリームも、明快なテストが可能になる**

まずはシンプルなマーブルテストから練習してみましょう！