---
description: TestSchedulerは仮想時間を使ってRxJSの時間ベースのオペレーターをテストできる強力なツールです。マーブル記法、ColdとHot Observableの扱い、debounceTimeやdelayなど時間依存処理の精密なユニットテスト方法を解説します。
---

# TestSchedulerを活用したテスト

RxJSの`TestScheduler`は、時間ベースのオペレーターを正確にテストするための強力なツールです。この章では、TestSchedulerを活用したテスト方法を体系的に解説します。

## TestSchedulerとは？

通常、Observableは時間に依存して動作します。たとえば、`delay()`や`debounceTime()`は一定時間待つオペレーターです。  
テストにおいて実際に待つのは非効率なため、仮想時間を使って即座にテストを行う仕組みが`TestScheduler`です。

> [!TIP]
> TestSchedulerでは「仮想時間」を使うため、リアル時間の待機が不要になります。

## TestSchedulerの基本構成

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
      const expected =  '--a--b--c|';

      expectObservable(source$).toBe(expected);
    });
  });
});
```

- `cold()`：購読ごとに独立してストリームが始まるCold Observableを作成
- `hot()`：ストリームがすでに進行しているHot Observableを作成
- `expectObservable()`：Observableの出力をマーブル記法で検証


## Cold ObservableとHot Observable

|種類|特徴|用途|
|:---|:---|:---|
|Cold Observable|購読のたびに最初からデータを流す|HTTPリクエストなど|
|Hot Observable|データの流れがすでに始まっていて購読者に共有される|ユーザーイベント、WebSocketなど|


## マーブル記法の基本

マーブル記法とは、Observableの時間経過を文字列で表す手法です。

|記号|意味|
|:---|:---|
|`-`|時間の経過（1フレーム）|
|`a`, `b`, `c`|発行される値|
|`|`|完了|
|`#`|エラー|
|`() `|複数の値を同時に発行（複数イベント）|

例：

```
--a--b--c|    // 2フレーム後にa、その後b、c、完了
```


## 仮想時間を使ったテスト例

### debounceTimeのテスト

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { debounceTime, map } from 'rxjs/operators';
import { TestScheduler } from 'rxjs/testing';

describe('仮想時間を使ったテスト', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('debounceTimeオペレーターのテスト', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20),
        map(x => x.toUpperCase())
      );
      const expected =    '-----------(C|)';  // ←これ！

      expectObservable(result$).toBe(expected, { B: 'B', C: 'C' });
    });
  });
});
```


## エラー処理のテスト

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { catchError} from 'rxjs/operators';
import { TestScheduler } from 'rxjs/testing';
import { of } from 'rxjs';

describe('エラー処理のテスト', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('catchErrorでエラー補足する', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--#');
      const result$ = source$.pipe(
        catchError(() => of('X'))
      );
  
      const expected =    '--a--(X|)';
  
      expectObservable(result$).toBe(expected);
    });
  });
});
```


## まとめ

- TestSchedulerを使うとリアル時間を待たずにテスト可能
- Cold/Hot Observableの違いを理解して使い分ける
- マーブル記法を駆使して時間経過を可視化する
- 複雑な非同期ストリームも精密にテストできる

> [!NEXT]
> 次は、さらに高度なマーブルテスト（マーブル文字列のカスタマイズや複数ストリームの組み合わせ）について学びます。