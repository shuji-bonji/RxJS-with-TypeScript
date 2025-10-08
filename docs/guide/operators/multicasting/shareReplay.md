---
description: shareReplay()オペレーターは、マルチキャスティングに加えて過去の値をバッファリングし、遅延購読者にも提供します。APIレスポンスのキャッシュや状態管理に最適なオペレーターです。
---

# shareReplay - 過去の値をキャッシュして共有する

`shareReplay()`オペレーターは、`share()`と同様にマルチキャスティングを実現しますが、さらに**指定した数の過去の値を記憶**し、後から参加した購読者にも提供します。

これにより、APIレスポンスのキャッシュや状態の共有など、より高度なユースケースに対応できます。

[📘 RxJS公式ドキュメント - `shareReplay()`](https://rxjs.dev/api/index/function/shareReplay)

## 🔰 基本的な使い方

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs/operators';

// shareReplayを使用（バッファサイズ2）
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`ソース: ${value}`)),
  shareReplay(2) // 直近2つの値をバッファリング
);

// 最初の購読者
console.log('Observer 1 購読開始');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 3.5秒後に2人目の購読者を追加
setTimeout(() => {
  console.log('Observer 2 購読開始 - 最新の2つの値を受け取る');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

### 実行結果

```
Observer 1 購読開始
ソース: 0
Observer 1: 0
ソース: 1
Observer 1: 1
ソース: 2
Observer 1: 2
ソース: 3
Observer 1: 3
Observer 2 購読開始 - 最新の2つの値を受け取る
Observer 2: 2  // ← バッファされた過去の値
Observer 2: 3  // ← バッファされた過去の値
ソース: 4
Observer 1: 4
Observer 2: 4
```

**重要なポイント**:
- 遅延購読者も、バッファされた過去の値を即座に受け取れる
- バッファサイズ分の値が記憶される（この例では2つ）

## 💡 shareReplay() の構文

```typescript
shareReplay(bufferSize?: number, windowTime?: number, scheduler?: SchedulerLike)
shareReplay(config: ShareReplayConfig)
```

### パラメータ

| パラメータ | 型 | 説明 | デフォルト |
|-----------|---|------|----------|
| `bufferSize` | `number` | バッファする値の数 | `Infinity` |
| `windowTime` | `number` | バッファの有効期間（ミリ秒） | `Infinity` |
| `scheduler` | `SchedulerLike` | タイミング制御用スケジューラ | - |

### 設定オブジェクト（RxJS 7+）

```typescript
interface ShareReplayConfig {
  bufferSize?: number;
  windowTime?: number;
  refCount?: boolean;  // 購読者が0になったら解除するか
  scheduler?: SchedulerLike;
}
```

## 📊 shareとshareReplayの違い

### share() の動作

```typescript
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs/operators';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`ソース: ${value}`)),
  share()
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Observer 2 購読開始');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**実行結果**:
```
ソース: 0
Observer 1: 0
ソース: 1
Observer 1: 1
Observer 2 購読開始
ソース: 2
Observer 1: 2
Observer 2: 2  // ← 過去の値（0, 1）は受け取れない
```

### shareReplay() の動作

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs/operators';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`ソース: ${value}`)),
  shareReplay(2) // 直近2つの値をバッファ
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Observer 2 購読開始');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**実行結果**:
```
ソース: 0
Observer 1: 0
ソース: 1
Observer 1: 1
Observer 2 購読開始
Observer 2: 0  // ← バッファされた過去の値
Observer 2: 1  // ← バッファされた過去の値
ソース: 2
Observer 1: 2
Observer 2: 2
```

## 💼 実践的なユースケース

### 1. APIレスポンスのキャッシュ

```typescript
import { Observable } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, shareReplay, tap } from 'rxjs/operators';

class UserService {
  // ユーザー情報をキャッシュする
  private userCache$ = ajax.getJSON<User>('https://api.example.com/user').pipe(
    tap(() => console.log('APIリクエスト実行')),
    shareReplay(1) // 最新の1つの値を永続的にキャッシュ
  );

  getUser(): Observable<User> {
    return this.userCache$;
  }
}

const userService = new UserService();

// 最初のコンポーネント
userService.getUser().subscribe(user => {
  console.log('コンポーネント1:', user);
});

// 2秒後に別のコンポーネント
setTimeout(() => {
  userService.getUser().subscribe(user => {
    console.log('コンポーネント2:', user); // ← キャッシュから取得、APIリクエストなし
  });
}, 2000);
```

**実行結果**:
```
APIリクエスト実行
コンポーネント1: { id: 1, name: "John" }
コンポーネント2: { id: 1, name: "John" }  // ← APIリクエストなし
```

### 2. 設定情報の共有

```typescript
import { of } from 'rxjs';
import { delay, shareReplay, tap } from 'rxjs/operators';

// アプリケーション設定を取得（初回のみ実行）
const appConfig$ = of({
  apiUrl: 'https://api.example.com',
  theme: 'dark',
  language: 'ja'
}).pipe(
  delay(1000), // 読み込みをシミュレート
  tap(() => console.log('設定を読み込みました')),
  shareReplay(1)
);

// 複数のサービスで設定を使用
appConfig$.subscribe(config => console.log('Service A:', config.apiUrl));
appConfig$.subscribe(config => console.log('Service B:', config.theme));
appConfig$.subscribe(config => console.log('Service C:', config.language));
```

**実行結果**:
```
設定を読み込みました
Service A: https://api.example.com
Service B: dark
Service C: ja
```

### 3. 時間制限付きキャッシュ

```typescript
import { ajax } from 'rxjs/ajax';
import { shareReplay, tap } from 'rxjs/operators';

// 5秒間だけキャッシュする
const weatherData$ = ajax.getJSON('https://api.example.com/weather').pipe(
  tap(() => console.log('天気データ取得')),
  shareReplay({
    bufferSize: 1,
    windowTime: 5000, // 5秒間有効
    refCount: true    // 購読者が0になったら解除
  })
);

// 最初の購読
weatherData$.subscribe(data => console.log('取得1:', data));

// 3秒後（キャッシュ有効）
setTimeout(() => {
  weatherData$.subscribe(data => console.log('取得2:', data)); // キャッシュから
}, 3000);

// 6秒後（キャッシュ期限切れ）
setTimeout(() => {
  weatherData$.subscribe(data => console.log('取得3:', data)); // 新規リクエスト
}, 6000);
```

## ⚠️ メモリリークに注意

`shareReplay()`は値をバッファに保持し続けるため、適切に管理しないとメモリリークの原因になります。

### 問題のあるコード

```typescript
// ❌ メモリリークの危険性
const infiniteStream$ = interval(1000).pipe(
  shareReplay() // bufferSize未指定 = Infinity
);

// このストリームは永遠に値を蓄積し続ける
```

### 推奨される対策

```typescript
// ✅ バッファサイズを制限
const safeStream$ = interval(1000).pipe(
  shareReplay(1) // 最新の1つだけ保持
);

// ✅ refCountを使用
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    refCount: true // 購読者が0になったらバッファをクリア
  })
);

// ✅ 時間制限を設定
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    windowTime: 10000 // 10秒で期限切れ
  })
);
```

## 🎯 バッファサイズの選び方

| バッファサイズ | 使用ケース | 例 |
|--------------|-----------|---|
| `1` | 最新の状態のみ必要 | 現在のユーザー情報、設定 |
| `3-5` | 直近数件の履歴が必要 | チャット履歴、通知履歴 |
| `Infinity` | すべての履歴が必要 | ログ、監査証跡（要注意） |

## 🔄 関連オペレーター

- **[share()](/guide/operators/multicasting/share)** - シンプルなマルチキャスト（バッファなし）
- **[publish()](/guide/subjects/multicasting)** - 低レベルなマルチキャスト制御
- **[ReplaySubject](/guide/subjects/types-of-subject)** - shareReplayの基盤となるSubject

## まとめ

`shareReplay()`オペレーターは：
- 過去の値をバッファリングして遅延購読者にも提供
- APIレスポンスのキャッシュに最適
- メモリリークに注意が必要
- `refCount`や`windowTime`で安全に使用可能

状態の共有やキャッシュが必要な場合、`shareReplay()`は非常に強力なツールですが、適切なバッファサイズと期限設定を行うことが重要です。
