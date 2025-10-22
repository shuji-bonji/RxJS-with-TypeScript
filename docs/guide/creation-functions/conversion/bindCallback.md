---
description: bindCallback() - コールバック関数をObservableに変換するCreation Function。既存のコールバックベースAPIをRxJSで統合できます。
---

# bindCallback() - コールバック関数の変換

`bindCallback()`は、標準的なコールバック関数をObservableに変換するCreation Functionです。

## 概要

`bindCallback()`は、既存のコールバックベースAPIをRxJSのObservableに変換します。jQuery、古いJavaScript APIなど、コールバックパターンを使用するレガシーコードをリアクティブな方法で統合できます。

**シグネチャ**:
```typescript
function bindCallback<T>(
  callbackFunc: Function,
  resultSelector?: Function,
  scheduler?: SchedulerLike
): (...args: any[]) => Observable<T>
```

**公式ドキュメント**: [📘 RxJS公式: bindCallback()](https://rxjs.dev/api/index/function/bindCallback)

## 基本的な使い方

### パターン1: シンプルなコールバックの変換

最も基本的な使用方法です。

```typescript
import { bindCallback } from 'rxjs';

// 既存のコールバックベースAPI
function legacyAPI(value: string, callback: (result: string) => void) {
  setTimeout(() => {
    callback(`処理結果: ${value}`);
  }, 1000);
}

// Observableに変換
const apiAsObservable = bindCallback(legacyAPI);

// Observable として使用
apiAsObservable('テストデータ').subscribe(result => {
  console.log(result); // 出力: 処理結果: テストデータ
});
```

### パターン2: 複数の引数を持つコールバック

コールバックが複数の引数を返す場合です。

```typescript
import { bindCallback } from 'rxjs';

// コールバックが複数の引数を返す
function multiArgCallback(
  value: number,
  callback: (result1: number, result2: number) => void
) {
  setTimeout(() => {
    callback(value * 2, value * 3);
  }, 500);
}

// 変換（結果は配列として発行される）
const wrapped = bindCallback(multiArgCallback);

wrapped(5).subscribe(([result1, result2]) => {
  console.log('結果1:', result1); // 10
  console.log('結果2:', result2); // 15
});
```

### パターン3: resultSelectorで結果を変換

`resultSelector`を使って結果をカスタマイズできます。

```typescript
import { bindCallback } from 'rxjs';

function fetchUserData(
  userId: number,
  callback: (name: string, email: string) => void
) {
  setTimeout(() => {
    callback(`User${userId}`, `user${userId}@example.com`);
  }, 500);
}

// resultSelectorで結果をオブジェクトに変換
const getUserData = bindCallback(
  fetchUserData,
  (name: string, email: string) => ({ name, email })
);

getUserData(123).subscribe(user => {
  console.log('ユーザー:', user);
  // 出力: ユーザー: { name: 'User123', email: 'user123@example.com' }
});
```

## 重要な特徴

### 1. 遅延評価（Lazy Evaluation）

`bindCallback()`は購読されるまで実行されません。

```typescript
import { bindCallback } from 'rxjs';

function heavyOperation(callback: (result: string) => void) {
  console.log('重い処理を開始...');
  setTimeout(() => callback('完了'), 1000);
}

const wrapped = bindCallback(heavyOperation);

console.log('Observable作成（まだ実行されない）');
const obs$ = wrapped();

console.log('購読開始');
obs$.subscribe(result => console.log('結果:', result));

// 出力:
// Observable作成（まだ実行されない）
// 購読開始
// 重い処理を開始...
// 結果: 完了
```

### 2. Cold Observable

`bindCallback()`はCold Observableを生成します。購読するたびに関数が実行されます。

```typescript
import { bindCallback } from 'rxjs';

let callCount = 0;

function countingAPI(callback: (count: number) => void) {
  callCount++;
  callback(callCount);
}

const wrapped = bindCallback(countingAPI);

wrapped().subscribe(count => console.log('購読1:', count)); // 1
wrapped().subscribe(count => console.log('購読2:', count)); // 2
wrapped().subscribe(count => console.log('購読3:', count)); // 3
```

### 3. 引数の受け渡し

元の関数の引数をそのまま渡せます。

```typescript
import { bindCallback } from 'rxjs';

function calculate(
  a: number,
  b: number,
  operation: string,
  callback: (result: number) => void
) {
  setTimeout(() => {
    const result = operation === 'add' ? a + b : a * b;
    callback(result);
  }, 300);
}

const calc = bindCallback(calculate);

// 引数を渡して実行
calc(5, 3, 'add').subscribe(result => {
  console.log('加算:', result); // 8
});

calc(5, 3, 'multiply').subscribe(result => {
  console.log('乗算:', result); // 15
});
```

## 実践的なユースケース

### 1. jQueryのアニメーションAPI

jQueryのコールバックベースAPIをObservableに変換します。

```typescript
import { bindCallback } from 'rxjs';

declare const $: any; // jQuery

// jQueryのanimate()をObservableに変換
function animateElement(selector: string, properties: any) {
  const wrappedAnimate = bindCallback((callback: () => void) => {
    $(selector).animate(properties, 1000, callback);
  });

  return wrappedAnimate();
}

// 使用例：アニメーションを順次実行
animateElement('#box1', { left: '200px' }).pipe(
  concatMap(() => animateElement('#box2', { top: '100px' })),
  concatMap(() => animateElement('#box3', { opacity: 0.5 }))
).subscribe({
  complete: () => console.log('すべてのアニメーション完了')
});
```

### 2. Geolocation API

ブラウザのGeolocation APIをObservableに変換します。

```typescript
import { bindCallback } from 'rxjs';

interface Coordinates {
  latitude: number;
  longitude: number;
}

function getCurrentPosition(): Observable<Coordinates> {
  const wrappedGetCurrentPosition = bindCallback<GeolocationPosition>(
    (callback: (position: GeolocationPosition) => void) => {
      navigator.geolocation.getCurrentPosition(callback);
    }
  );

  return wrappedGetCurrentPosition().pipe(
    map(position => ({
      latitude: position.coords.latitude,
      longitude: position.coords.longitude
    }))
  );
}

// 使用例
getCurrentPosition().subscribe({
  next: coords => console.log('現在地:', coords),
  error: err => console.error('位置情報取得エラー:', err)
});
```

### 3. カスタムタイマー

`setTimeout`をObservableでラップします。

```typescript
import { bindCallback } from 'rxjs';

function delay(ms: number): Observable<void> {
  const wrappedSetTimeout = bindCallback((callback: () => void) => {
    setTimeout(callback, ms);
  });

  return wrappedSetTimeout().pipe(
    map(() => undefined)
  );
}

// 使用例：順次遅延処理
console.log('開始');

delay(1000).pipe(
  tap(() => console.log('1秒経過')),
  concatMap(() => delay(1000)),
  tap(() => console.log('2秒経過')),
  concatMap(() => delay(1000)),
  tap(() => console.log('3秒経過'))
).subscribe({
  complete: () => console.log('完了')
});
```

### 4. ローカルストレージの非同期ラッパー

`localStorage`を非同期APIとしてラップします。

```typescript
import { bindCallback } from 'rxjs';
import { map } from 'rxjs/operators';

class AsyncLocalStorage {
  setItem(key: string, value: string): Observable<void> {
    const wrappedSet = bindCallback((callback: () => void) => {
      try {
        localStorage.setItem(key, value);
        callback();
      } catch (error) {
        console.error('保存エラー:', error);
      }
    });

    return wrappedSet().pipe(
      map(() => undefined)
    );
  }

  getItem(key: string): Observable<string | null> {
    const wrappedGet = bindCallback((callback: (value: string | null) => void) => {
      try {
        const value = localStorage.getItem(key);
        callback(value);
      } catch (error) {
        console.error('取得エラー:', error);
        callback(null);
      }
    });

    return wrappedGet();
  }

  removeItem(key: string): Observable<void> {
    const wrappedRemove = bindCallback((callback: () => void) => {
      try {
        localStorage.removeItem(key);
        callback();
      } catch (error) {
        console.error('削除エラー:', error);
      }
    });

    return wrappedRemove().pipe(
      map(() => undefined)
    );
  }
}

// 使用例
const storage = new AsyncLocalStorage();

storage.setItem('user', 'Alice').pipe(
  concatMap(() => storage.getItem('user')),
  tap(value => console.log('取得:', value)),
  concatMap(() => storage.removeItem('user'))
).subscribe({
  complete: () => console.log('処理完了')
});
```

### 5. Web Worker通信

Web Workerとの通信をObservableでラップします。

```typescript
import { bindCallback } from 'rxjs';
import { Observable } from 'rxjs';

function runWorker(data: any): Observable<any> {
  return new Observable(observer => {
    const worker = new Worker('/worker.js');

    worker.onmessage = (event) => {
      observer.next(event.data);
      observer.complete();
      worker.terminate();
    };

    worker.onerror = (error) => {
      observer.error(error);
      worker.terminate();
    };

    worker.postMessage(data);

    // クリーンアップ
    return () => {
      worker.terminate();
    };
  });
}

// 使用例
runWorker({ task: 'calculate', value: 100 }).subscribe({
  next: result => console.log('Worker結果:', result),
  error: err => console.error('Worker エラー:', err)
});
```

## bindNodeCallback() との比較

`bindCallback()`と`bindNodeCallback()`の違いを理解しましょう。

### bindCallback() - 標準コールバック

```typescript
import { bindCallback } from 'rxjs';

// 標準形式: callback(result)
function standardAPI(value: number, callback: (result: number) => void) {
  callback(value * 2);
}

const wrapped = bindCallback(standardAPI);
wrapped(5).subscribe(result => console.log(result)); // 10
```

### bindNodeCallback() - Node.js形式

```typescript
import { bindNodeCallback } from 'rxjs';

// Node.js形式: callback(error, result)
function nodejsStyleAPI(
  value: number,
  callback: (error: Error | null, result?: number) => void
) {
  if (value < 0) {
    callback(new Error('負の数は処理できません'));
  } else {
    callback(null, value * 2);
  }
}

const wrapped = bindNodeCallback(nodejsStyleAPI);
wrapped(5).subscribe({
  next: result => console.log(result), // 10
  error: err => console.error(err)
});
```

> [!IMPORTANT]
> **使い分けのポイント**:
> - **標準コールバック** (`callback(result)`) → `bindCallback()`
> - **Node.js形式** (`callback(error, result)`) → `bindNodeCallback()`
>
> 間違った関数を使用すると、正しく動作しません。

## エラーハンドリング

`bindCallback()`自体はエラーを処理しないため、明示的なエラーハンドリングが必要です。

```typescript
import { bindCallback } from 'rxjs';
import { catchError } from 'rxjs/operators';
import { of } from 'rxjs';

function unreliableAPI(
  value: number,
  callback: (result: number) => void
) {
  if (value < 0) {
    // エラーをthrowしてもcatchされない
    throw new Error('負の数は処理できません');
  }
  callback(value * 2);
}

const wrapped = bindCallback(unreliableAPI);

wrapped(5).pipe(
  catchError(error => {
    console.error('エラー:', error.message);
    return of(0); // デフォルト値
  })
).subscribe(result => console.log('結果:', result));

// 負の数で実行するとエラー
wrapped(-5).pipe(
  catchError(error => {
    console.error('エラー:', error.message);
    return of(0);
  })
).subscribe(result => console.log('結果:', result));
```

## TypeScriptでの型安全性

`bindCallback()`はTypeScriptの型推論をサポートしています。

```typescript
import { bindCallback } from 'rxjs';

interface User {
  id: number;
  name: string;
}

// 型安全な関数定義
function fetchUser(
  userId: number,
  callback: (user: User) => void
): void {
  setTimeout(() => {
    callback({ id: userId, name: `User${userId}` });
  }, 500);
}

// 型推論が効く
const getUser = bindCallback(fetchUser);

// userはUser型として推論される
getUser(123).subscribe(user => {
  console.log(user.name); // 型安全
  // console.log(user.invalid); // コンパイルエラー
});
```

## まとめ

`bindCallback()`は、レガシーなコールバックベースAPIをリアクティブに統合する強力なツールです。

> [!IMPORTANT]
> **bindCallback()の特徴**:
> - ✅ 標準的なコールバック関数を変換
> - ✅ Cold Observable（購読ごとに実行）
> - ✅ TypeScript型推論サポート
> - ✅ 既存のライブラリとの統合が容易
> - ⚠️ エラーハンドリングは手動実装が必要
> - ⚠️ Node.js形式のコールバックには`bindNodeCallback()`を使用

## 関連項目

- [bindNodeCallback()](/guide/creation-functions/conversion/bindNodeCallback) - Node.js形式のコールバック変換
- [defer()](/guide/creation-functions/conditional/defer) - 購読時に動的生成
- [Observable.create()](/guide/observables/creation) - カスタムObservable作成

## 参考リソース

- [RxJS公式: bindCallback()](https://rxjs.dev/api/index/function/bindCallback)
- [Learn RxJS: bindCallback](https://www.learnrxjs.io/learn-rxjs/operators/creation/bindcallback)
