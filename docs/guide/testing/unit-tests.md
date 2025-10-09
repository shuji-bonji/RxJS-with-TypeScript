---
description: RxJSのユニットテストでは同期・非同期・時間制御の各手法を使い分け、TestSchedulerやマーブルテスト、モック・スタブを用いて堅牢なテスト戦略を構築します。
---

# RxJSのユニットテスト

RxJSを使ったコードは非同期処理が多く、従来のテスト手法とは異なるアプローチが必要です。このガイドでは、RxJSを使ったコードを効果的にテストするための基本的な手法から高度なテクニックまでを解説します。

## 同期的なObservableのテスト

最も単純なケースとして、同期的に完了するObservableのテストから始めましょう。

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs/operators';
import { describe, it, expect } from 'vitest';

// テスト対象の関数
function doubleValues(input$: Observable<number>) : Observable<number>{
  return input$.pipe(
    map(x => x * 2)
  );
}

describe('基本的なObservableのテスト', () => {
  it('値を2倍にする', () => {
    // テスト用のObservable
    const source$ = of(1, 2, 3);
    const result$ = doubleValues(source$);
    
    // 期待する結果
    const expected = [2, 4, 6];
    const actual: number[] = [];
    
    // 実行と検証
    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
      }
    });
  });
});
```

## 非同期Observableをテストする方法

非同期のObservableの場合は、テストフレームワークの非同期サポートを活用します。

```ts
import { Observable, timer } from 'rxjs';
import { map, take } from 'rxjs/operators';
import { describe, it, expect } from 'vitest';

// テスト対象の非同期関数
function getDelayedValues(): Observable<number> {
  return timer(0, 100).pipe(
    map(x => x + 1),
    take(3)
  );
}

describe('非同期Observableのテスト', () => {
  it('非同期値を順番に受け取る', (done: Function) => {
    const result$ = getDelayedValues();
    const expected = [1, 2, 3];
    const actual: number[] = [];
    
    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
        done();
      }
    });
  });
});
```

## Promise変換による非同期テスト

Observableを`toPromise()`や`lastValueFrom()`を使ってPromiseに変換し、modern JS/TSのasync/awaitを活用する方法もあります。

```ts
import { Observable, of } from 'rxjs';
import { map, delay, toArray } from 'rxjs/operators';
import { describe, it, expect } from 'vitest';
import { lastValueFrom } from 'rxjs';

// テスト対象の関数
function processWithDelay(input$: Observable<number>) {
  return input$.pipe(
    map(x => x * 10),
    delay(100),
    toArray()
  );
}

describe('Promise変換を使ったテスト', () => {
  it('遅延処理を待ってから検証', async () => {
    const source$ = of(1, 2, 3);
    const result$ = processWithDelay(source$);
    
    // Observableをpromiseに変換
    const result = await lastValueFrom(result$);
    
    // 期待する結果
    expect(result).toEqual([10, 20, 30]);
  });
});
```

## TestSchedulerの活用

RxJSは`TestScheduler`という特別なスケジューラーを提供しており、これを使って時間ベースのオペレーターのテストを効率的に行えます。

```ts
import { TestScheduler } from 'rxjs/testing';
import { map, debounceTime } from 'rxjs/operators';
import { describe, it, beforeEach } from 'vitest';

describe('TestSchedulerの使用', () => {
  let testScheduler: TestScheduler;
  
  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });
  
  it('debounceTimeのテスト', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('a--b--c--d|', { a: 1, b: 2, c: 3, d: 4 });
      const result = source.pipe(
        debounceTime(20),
        map(x => x * 10)
      );
      
      const expected = '----------(d|)';
      
      expectObservable(result).toBe(expected, { d: 40 });
    });
  });
});
```

> [!NOTE]
> マーブルテスト記法
> `TestScheduler`を使用する際、マーブル図を使用して時間の経過を表現します。

## 時間を操作可能にする

時間に依存したコード（delay, debounceTimeなど）をテストする場合は、`TestScheduler`を使用して時間を制御します。

```ts
import { TestScheduler } from 'rxjs/testing';
import { interval } from 'rxjs';
import { take, map } from 'rxjs/operators';
import { describe, it, beforeEach } from 'vitest';

describe('時間の制御', () => {
  let testScheduler: TestScheduler;
  
  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });
  
  it('時間を早送りしてテスト', () => {
    testScheduler.run(({ expectObservable }) => {
      const source = interval(1000).pipe(
        take(3),
        map(x => x + 1)
      );
      
      // 実際は3秒かかるが、テスト環境では即時実行される
      const expected = '1s a 999ms b 999ms (c|)';
      const values = { a: 1, b: 2, c: 3 };
      
      expectObservable(source).toBe(expected, values);
    });
  });
});
```

## エラー処理のテスト（TestScheduler版）

エラーが発生した場合のObservableの挙動をテストすることも重要です。

```ts
import { TestScheduler } from 'rxjs/testing';
import { throwError, of } from 'rxjs';
import { catchError } from 'rxjs/operators';

describe('エラー処理のテスト', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Observableがエラーを通知する場合', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const expected =     '--a--b--#';

      expectObservable(source).toBe(expected);
    });
  });

  it('catchErrorでエラーを補足して値に置き換える場合', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const handled = source.pipe(
        catchError(() => of('X'))
      );

      const expected =     '--a--b--(X|)';

      expectObservable(handled).toBe(expected);
    });
  });
});
```

## マーブルテスト

複雑なストリームのテストには、マーブル図を使って直感的にテスト期待値を表現します。

### Hot Observable vs Cold Observable

TestSchedulerでは、hotとcoldという2種類のObservableを作成できます。この違いを理解してテストすることが重要です。

```ts
import { TestScheduler } from 'rxjs/testing';
import { Subject } from 'rxjs';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Hot vs Cold Observable テスト', () => {
  let testScheduler: TestScheduler;
  
  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });
  
  it('Cold Observableは購読ごとに独立したストリームを生成する', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      // Cold Observable（各購読者に対して独立）
      const source = cold('--a--b--c|', { a: 1, b: 2, c: 3 });
      
      // 1回目の購読
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
      
      // 2回目の購読（最初から開始される）
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });
  
  it('Hot Observableは購読者間でストリームを共有する', () => {
    testScheduler.run(({ hot, expectObservable }) => {
      // Hot Observable（購読者間で共有）
      const source = hot('--a--b--c|', { a: 1, b: 2, c: 3 });
      
      // 遅れて購読開始（購読開始以降の値のみ受け取る）
      // expectObservable(source, '-----^---').toBe('-----b--c|', { b: 2, c: 3 });
      expectObservable(source, '----^').toBe('-----b--c|', { b: 2, c: 3 });
      
      // 最初から購読（すべての値を受け取る）
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });
  
  it('実際のSubjectを使ったHot Observableのテスト', () => {
    // 非TestScheduler版
    const subject = new Subject<number>();
    const values1: number[] = [];
    const values2: number[] = [];
    
    // 1人目の購読者
    const subscription1 = subject.subscribe(val => values1.push(val));
    
    // 値を発行
    subject.next(1);
    subject.next(2);
    
    // 2人目の購読者（途中から）
    const subscription2 = subject.subscribe(val => values2.push(val));
    
    // さらに値を発行
    subject.next(3);
    subject.complete();
    
    // 検証
    expect(values1).toEqual([1, 2, 3]);
    expect(values2).toEqual([3]); // 購読開始後の値のみ
    
    // クリーンアップ
    subscription1.unsubscribe();
    subscription2.unsubscribe();
  });
});
```

> [!NOTE]
> Cold Observableは購読するたびに独立してデータを生成しますが、Hot Observableはデータを共有して配信します。

## モックとスタブの活用

### 依存サービスのモック化

RxJSを使ったサービスをテストする場合、外部依存をモック化することがよくあります。

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs/operators';
import { describe, it, expect, vi } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
}

// テスト対象のサービス
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}
  
  getUsers(): Observable<User[]> {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('サービスのテスト', () => {
  it('アクティブユーザーのみをフィルタリングする', () => {
    // モックAPIサービス
    const mockApiService = {
      fetchUsers: vi.fn().mockReturnValue(of([
        { id: 1, name: '田中', active: true },
        { id: 2, name: '佐藤', active: false },
        { id: 3, name: '山田', active: true }
      ]))
    };
    
    const userService = new UserService(mockApiService);
    const result$ = userService.getUsers();
    
    // 検証
    result$.subscribe(users => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('田中');
      expect(users[1].name).toBe('山田');
      expect(mockApiService.fetchUsers).toHaveBeenCalledTimes(1);
    });
  });
});
```

### スタブ (Stub) の活用

スタブはテスト対象のコードが依存する外部のデータやAPIを模倣するシンプルなオブジェクトです。  
外部リソースへの依存を排除し、テストが独立して動作するようにします。  
固定値を返すだけで、内部のロジックを持ちません。

```ts
import { of } from 'rxjs';
import { map } from 'rxjs/operators';
import { describe, it, expect } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
};

// テスト対象のサービス
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getActiveUsers() {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('UserService のテスト', () => {
  it('アクティブなユーザーのみを返す', () => {
    // 🔹 スタブの作成
    const stubApiService = {
      fetchUsers: () => of<User[]>([
        { id: 1, name: '田中', active: true },
        { id: 2, name: '佐藤', active: false },
        { id: 3, name: '山田', active: true }
      ])
    };

    // テスト対象のサービス
    const userService = new UserService(stubApiService);

    // 結果を確認
    userService.getActiveUsers().subscribe((users: User[]) => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('田中');
      expect(users[1].name).toBe('山田');
    });
  });
});
```

## サブスクリプションのスパイ

サブスクリプションが正しく行われているか検証するためにスパイを使用できます。

```ts
import { Subject } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

describe('サブスクリプションのテスト', () => {
  it('適切なハンドラーで購読している', () => {
    const subject = new Subject();
    
    // ハンドラーのスパイを作成
    const nextSpy = vi.fn();
    const errorSpy = vi.fn();
    const completeSpy = vi.fn();
    
    // 購読
    subject.subscribe({
      next: nextSpy,
      error: errorSpy,
      complete: completeSpy
    });
    
    // 値を発行
    subject.next('value1');
    subject.next('value2');
    subject.complete();
    
    // 検証
    expect(nextSpy).toHaveBeenCalledTimes(2);
    expect(nextSpy).toHaveBeenCalledWith('value1');
    expect(nextSpy).toHaveBeenCalledWith('value2');
    expect(errorSpy).not.toHaveBeenCalled();
    expect(completeSpy).toHaveBeenCalledTimes(1);
  });
});
```

## ベストプラクティス

|ベストプラクティス|説明|
|---|---|
|単一責任の原則を守る|テスト可能なコードを書くために、各関数やクラスが単一の責任を持つようにします。こうすることで、テストもシンプルになります。|
|外部依存をモック化する|httpリクエストやタイマーなどの外部依存はモック化して、予測可能な環境でテストを行います。|
|非同期コードには適切なテクニックを使う|非同期テストには、TestScheduler、done()コールバック、またはasync/awaitなど、適切な方法を選択します。|
|マーブルテストを活用する|複雑なストリームのテストには、マーブル図を使って直感的にテスト期待値を表現します。

## まとめ

RxJSコードのテストは、同期/非同期の性質や時間に依存する動作など、従来のJavaScriptコードとは異なる側面があります。適切なテスト手法を選択することで、高品質なリアクティブコードを安心して開発することができます。特に以下の点を心がけましょう。

- 同期的なObservableには単純なサブスクリプションテスト
- 非同期処理にはTestSchedulerやPromise変換
- 時間に依存するコードにはマーブルテスト
- 外部依存はモック化して独立したテスト環境を作る
- 単一責任の原則に従い、テスト容易なコードを設計する

## 🔗 関連セクション

- **[よくある間違いと対処法](/guide/anti-patterns/common-mistakes#15-テストの欠如)** - テストに関するアンチパターンを確認
- **[TestSchedulerの活用](/guide/testing/test-scheduler)** - より詳細なTestSchedulerの使い方
- **[マーブルテスト](/guide/testing/marble-testing)** - 高度なマーブルテストの手法