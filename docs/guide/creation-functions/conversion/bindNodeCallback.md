---
description: bindNodeCallback() - Node.js形式のコールバック関数をObservableに変換するCreation Function。error-firstコールバックをRxJSで扱えます。
---

# bindNodeCallback() - Node.js形式コールバックの変換

`bindNodeCallback()`は、Node.js形式のerror-firstコールバックをObservableに変換するCreation Functionです。

## 概要

`bindNodeCallback()`は、Node.jsの標準的な`callback(error, result)`形式のコールバックをRxJSのObservableに変換します。Node.js標準ライブラリ（fs、http、crypto等）や、error-firstパターンを採用するAPIとの統合に最適です。

**シグネチャ**:
```typescript
function bindNodeCallback<T>(
  callbackFunc: Function,
  resultSelector?: Function,
  scheduler?: SchedulerLike
): (...args: any[]) => Observable<T>
```

**公式ドキュメント**: [📘 RxJS公式: bindNodeCallback()](https://rxjs.dev/api/index/function/bindNodeCallback)

## 基本的な使い方

### パターン1: シンプルなNode.js形式コールバック

最も基本的な使用方法です。

```typescript
import { bindNodeCallback } from 'rxjs';

// Node.js形式のコールバック: callback(error, result)
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

// Observableに変換
const apiAsObservable = bindNodeCallback(nodejsStyleAPI);

// 成功ケース
const result1 = apiAsObservable(5);
result1.subscribe(
  result => console.log('結果:', result), // 10
  err => console.error('エラー:', err)
);

// エラーケース
const result2 = apiAsObservable(-5);
result2.subscribe(
  result => console.log('結果:', result),
  err => console.error('エラー:', err.message) // 負の数は処理できません
);
```

### パターン2: 複数の結果値

コールバックが複数の値を返す場合です。

```typescript
import { bindNodeCallback } from 'rxjs';

function multiResultAPI(
  value: number,
  callback: (error: Error | null, result1?: number, result2?: number) => void
) {
  if (value === 0) {
    callback(new Error('0は処理できません'));
  } else {
    callback(null, value * 2, value * 3);
  }
}

// 複数の結果は配列として発行される
const wrapped = bindNodeCallback(multiResultAPI);

wrapped(5).subscribe({
  next: ([result1, result2]) => {
    console.log('結果1:', result1); // 10
    console.log('結果2:', result2); // 15
  },
  error: err => console.error('エラー:', err)
});
```

### パターン3: resultSelectorで結果を変換

結果をカスタマイズできます。

```typescript
import { bindNodeCallback } from 'rxjs';

function fetchUserAPI(
  userId: number,
  callback: (error: Error | null, name?: string, email?: string) => void
) {
  if (userId <= 0) {
    callback(new Error('無効なユーザーID'));
  } else {
    callback(null, `User${userId}`, `user${userId}@example.com`);
  }
}

// resultSelectorで結果をオブジェクトに変換
const getUser = bindNodeCallback(
  fetchUserAPI,
  (name: string, email: string) => ({ name, email })
);

getUser(123).subscribe({
  next: user => console.log('ユーザー:', user),
  // 出力: ユーザー: { name: 'User123', email: 'user123@example.com' }
  error: err => console.error('エラー:', err)
});
```

## 重要な特徴

### 1. 自動エラーハンドリング

error-firstコールバックのエラーは自動的にObservableのエラーチャンネルに渡されます。

```typescript
import { bindNodeCallback } from 'rxjs';
import { catchError } from 'rxjs/operators';
import { of } from 'rxjs';

function unreliableAPI(
  value: number,
  callback: (error: Error | null, result?: string) => void
) {
  if (Math.random() > 0.5) {
    callback(new Error('ランダムエラー'));
  } else {
    callback(null, `成功: ${value}`);
  }
}

const wrapped = bindNodeCallback(unreliableAPI);

wrapped(10).pipe(
  catchError(error => {
    console.log('エラーをキャッチ:', error.message);
    return of('デフォルト値');
  })
).subscribe(result => console.log('結果:', result));
```

### 2. Cold Observable

購読するたびに関数が実行されます。

```typescript
import { bindNodeCallback } from 'rxjs';

let callCount = 0;

function countingAPI(
  callback: (error: Error | null, count?: number) => void
) {
  callCount++;
  callback(null, callCount);
}

const wrapped = bindNodeCallback(countingAPI);

wrapped().subscribe(count => console.log('購読1:', count)); // 1
wrapped().subscribe(count => console.log('購読2:', count)); // 2
wrapped().subscribe(count => console.log('購読3:', count)); // 3
```

### 3. nullとundefinedの扱い

エラーが`null`または`undefined`の場合のみ、結果が発行されます。

```typescript
import { bindNodeCallback } from 'rxjs';

function testAPI(
  value: number,
  callback: (error: Error | null | undefined, result?: number) => void
) {
  if (value < 0) {
    callback(new Error('負の数'));
  } else if (value === 0) {
    callback(null, 0); // nullでも結果を発行
  } else {
    callback(undefined, value); // undefinedでも結果を発行
  }
}

const wrapped = bindNodeCallback(testAPI);

wrapped(5).subscribe(result => console.log('結果:', result)); // 5
wrapped(0).subscribe(result => console.log('結果:', result)); // 0
wrapped(-1).subscribe({
  next: result => console.log('結果:', result),
  error: err => console.error('エラー:', err) // Error: 負の数
});
```

## 実践的なユースケース

> [!WARNING]
> **Node.js環境について**:
> 以下の実践的なユースケースは**Node.js環境専用**です。ブラウザでは動作しません。
> - `fs`、`crypto`、`dns`、`http`などのNode.js標準モジュールが必要です
> - ブラウザで試す場合は、上記の基本例（シミュレーション）を使用してください

### 1. Node.js fs モジュール

ファイル操作をObservableでラップします。

```typescript
import { bindNodeCallback } from 'rxjs';
import * as fs from 'fs';
import { map, catchError } from 'rxjs/operators';
import { of } from 'rxjs';

// fs.readFileをObservableに変換
const readFileAsObservable = bindNodeCallback(fs.readFile);

const result = readFileAsObservable('./data.txt', 'utf8');
result.pipe(
  map(content => {
    console.log('ファイル内容:', content);
    return content.toUpperCase();
  }),
  catchError(error => {
    console.error('ファイル読み込みエラー:', error.message);
    return of('デフォルトコンテンツ');
  })
).subscribe(data => console.log('結果:', data));

// fs.writeFileをObservableに変換
const writeFileAsObservable = bindNodeCallback(fs.writeFile);

const writeResult = writeFileAsObservable('./output.txt', 'Hello, RxJS!', 'utf8');
writeResult.subscribe(
  () => console.log('ファイル書き込み成功'),
  err => console.error('書き込みエラー:', err)
);

// fs.unlinkをObservableに変換
const deleteFileAsObservable = bindNodeCallback(fs.unlink);

const deleteResult = deleteFileAsObservable('./temp.txt');
deleteResult.subscribe(
  () => console.log('ファイル削除成功'),
  err => console.error('削除エラー:', err)
);
```

### 2. ファイル操作の連鎖

複数のファイル操作を順次実行します。

```typescript
import { bindNodeCallback } from 'rxjs';
import * as fs from 'fs';
import { concatMap, map } from 'rxjs/operators';

const readFileAsObservable = bindNodeCallback(fs.readFile);
const writeFileAsObservable = bindNodeCallback(fs.writeFile);

// ファイルを読み込み→変換→別ファイルに書き込み
const result = readFileAsObservable('./input.txt', 'utf8');
result.pipe(
  map(content => content.toUpperCase()), // 大文字に変換
  concatMap(transformed => {
    const writeResult = writeFileAsObservable('./output.txt', transformed, 'utf8');
    return writeResult.pipe(map(() => transformed));
  })
).subscribe(
  content => console.log('処理完了:', content),
  err => console.error('エラー:', err),
  () => console.log('すべての操作完了')
);
```

### 3. crypto モジュール

暗号化処理をObservableでラップします。

```typescript
import { bindNodeCallback, map } from 'rxjs';
import * as crypto from 'crypto';

// crypto.randomBytesをObservableに変換
const generateRandomBytes = bindNodeCallback(crypto.randomBytes);

generateRandomBytes(32).pipe(
  map(buffer => buffer.toString('hex'))
).subscribe({
  next: token => console.log('ランダムトークン:', token),
  error: err => console.error('生成エラー:', err)
});

// crypto.pbkdf2をObservableに変換（パスワードハッシュ化）
const hashPassword = bindNodeCallback<Buffer>(
  (
    password: string,
    salt: string,
    iterations: number,
    keylen: number,
    digest: string,
    callback: (err: Error | null, derivedKey?: Buffer) => void
  ) => {
    crypto.pbkdf2(password, salt, iterations, keylen, digest, callback);
  }
);

hashPassword('myPassword', 'salt', 100000, 64, 'sha512').pipe(
  map(hash => hash.toString('hex'))
).subscribe({
  next: hashedPassword => console.log('ハッシュ化されたパスワード:', hashedPassword),
  error: err => console.error('ハッシュ化エラー:', err)
});
```

### 4. DNS解決

DNSルックアップをObservableでラップします。

```typescript
import { bindNodeCallback } from 'rxjs';
import * as dns from 'dns';

// dns.lookupをObservableに変換
const dnsLookup = bindNodeCallback<string>(
  (hostname: string, callback: (err: Error | null, address?: string) => void) => {
    dns.lookup(hostname, callback);
  }
);

dnsLookup('google.com').subscribe({
  next: address => console.log('IPアドレス:', address),
  error: err => console.error('DNS解決エラー:', err)
});

// dns.resolveをObservableに変換
const dnsResolve = bindNodeCallback<string[]>(
  (hostname: string, rrtype: string, callback: (err: Error | null, addresses?: string[]) => void) => {
    dns.resolve(hostname, rrtype, callback);
  }
);

dnsResolve('google.com', 'A').subscribe({
  next: addresses => console.log('Aレコード:', addresses),
  error: err => console.error('解決エラー:', err)
});
```

### 5. HTTPリクエスト（Node.js http モジュール）

Node.jsの`http`モジュールをラップします。

```typescript
import { bindNodeCallback, Observable } from 'rxjs';
import * as http from 'http';
import { map, catchError } from 'rxjs/operators';
import { of } from 'rxjs';

function httpGet(url: string): Observable<string> {
  return new Observable(observer => {
    http.get(url, (res) => {
      let data = '';

      res.on('data', (chunk) => {
        data += chunk;
      });

      res.on('end', () => {
        observer.next(data);
        observer.complete();
      });
    }).on('error', (err) => {
      observer.error(err);
    });
  });
}

// 使用例
httpGet('http://jsonplaceholder.typicode.com/posts/1').pipe(
  map(data => JSON.parse(data)),
  catchError(error => {
    console.error('HTTPエラー:', error);
    return of({ error: true });
  })
).subscribe(post => console.log('投稿:', post));
```

### 6. データベース操作（例：sqlite3）

SQLite3のコールバックAPIをラップします。

```typescript
import { bindNodeCallback } from 'rxjs';
import { Observable } from 'rxjs';
import { map } from 'rxjs/operators';

// 疑似コード（実際のsqlite3ライブラリを使用する場合）
declare const sqlite3: any;

class DatabaseService {
  private db: any;

  constructor(dbPath: string) {
    this.db = new sqlite3.Database(dbPath);
  }

  query<T>(sql: string, params: any[] = []): Observable<T[]> {
    const wrappedQuery = bindNodeCallback<T[]>(
      (callback: (err: Error | null, rows?: T[]) => void) => {
        this.db.all(sql, params, callback);
      }
    );

    return wrappedQuery();
  }

  run(sql: string, params: any[] = []): Observable<void> {
    const wrappedRun = bindNodeCallback(
      (callback: (err: Error | null) => void) => {
        this.db.run(sql, params, callback);
      }
    );

    return wrappedRun().pipe(
      map(() => undefined)
    );
  }
}

// 使用例
const db = new DatabaseService('./mydb.sqlite');

db.run('CREATE TABLE IF NOT EXISTS users (id INTEGER, name TEXT)')
  .pipe(
    concatMap(() => db.run('INSERT INTO users VALUES (?, ?)', [1, 'Alice'])),
    concatMap(() => db.query('SELECT * FROM users'))
  )
  .subscribe({
    next: rows => console.log('ユーザー:', rows),
    error: err => console.error('DBエラー:', err)
  });
```

## bindCallback() との比較

`bindNodeCallback()`と`bindCallback()`の違いを理解しましょう。

### bindCallback() - 標準コールバック

```typescript
import { bindCallback } from 'rxjs';

// 標準形式: callback(result)
function standardCallback(
  value: number,
  callback: (result: number) => void
) {
  callback(value * 2);
}

const wrapped = bindCallback(standardCallback);
wrapped(5).subscribe(result => console.log(result)); // 10
```

### bindNodeCallback() - Node.js形式

```typescript
import { bindNodeCallback } from 'rxjs';

// Node.js形式: callback(error, result)
function nodejsCallback(
  value: number,
  callback: (error: Error | null, result?: number) => void
) {
  if (value < 0) {
    callback(new Error('負の数は処理できません'));
  } else {
    callback(null, value * 2);
  }
}

const wrapped = bindNodeCallback(nodejsCallback);
wrapped(5).subscribe({
  next: result => console.log(result), // 10
  error: err => console.error(err)
});
```

> [!IMPORTANT]
> **使い分けのポイント**:
> - **標準コールバック** (`callback(result)`) → `bindCallback()`
> - **error-firstコールバック** (`callback(error, result)`) → `bindNodeCallback()`
> - **Node.js標準ライブラリ** (fs, http, crypto等) → `bindNodeCallback()`
>
> 間違った関数を使用すると、エラーが正しく処理されません。

## エラーハンドリング

`bindNodeCallback()`は自動的にエラーを処理しますが、追加の処理も可能です。

```typescript
import { bindNodeCallback } from 'rxjs';
import { catchError, retry, map } from 'rxjs/operators';
import { of } from 'rxjs';
import * as fs from 'fs';

const readFileAsObservable = bindNodeCallback(fs.readFile);

const result = readFileAsObservable('./config.json', 'utf8');
result.pipe(
  retry(3), // 3回までリトライ
  catchError(error => {
    console.error('ファイル読み込み失敗:', error.message);
    // デフォルト設定を返す
    return of('{"default": true}');
  }),
  map(content => JSON.parse(content))
).subscribe(config => console.log('設定:', config));
```

## TypeScriptでの型安全性

`bindNodeCallback()`はTypeScriptの型推論をサポートしています。

```typescript
import { bindNodeCallback } from 'rxjs';

interface User {
  id: number;
  name: string;
}

// 型安全な関数定義
function fetchUser(
  userId: number,
  callback: (error: Error | null, user?: User) => void
): void {
  if (userId <= 0) {
    callback(new Error('無効なID'));
  } else {
    callback(null, { id: userId, name: `User${userId}` });
  }
}

// 型推論が効く
const getUser = bindNodeCallback(fetchUser);

// userはUser型として推論される
getUser(123).subscribe({
  next: user => {
    console.log(user.name); // 型安全
    // console.log(user.invalid); // コンパイルエラー
  },
  error: err => console.error(err)
});
```

## まとめ

`bindNodeCallback()`は、Node.js標準ライブラリや、error-firstパターンを採用するAPIをリアクティブに統合する強力なツールです。

> [!IMPORTANT]
> **bindNodeCallback()の特徴**:
> - ✅ error-firstコールバックを自動処理
> - ✅ Node.js標準ライブラリとの統合が容易
> - ✅ Cold Observable（購読ごとに実行）
> - ✅ TypeScript型推論サポート
> - ⚠️ error-firstパターン専用（標準コールバックには`bindCallback()`）
> - ⚠️ ブラウザ環境では使用機会が少ない

## 関連項目

- [bindCallback()](/guide/creation-functions/conversion/bindCallback) - 標準コールバックの変換
- [defer()](/guide/creation-functions/conditional/defer) - 購読時に動的生成
- [catchError()](/guide/error-handling/retry-catch) - エラーハンドリング

## 参考リソース

- [RxJS公式: bindNodeCallback()](https://rxjs.dev/api/index/function/bindNodeCallback)
- [Learn RxJS: bindNodeCallback](https://www.learnrxjs.io/learn-rxjs/operators/creation/bindnodecallback)
- [Node.js: Error-first callbacks](https://nodejs.org/api/errors.html#errors_error_first_callbacks)
