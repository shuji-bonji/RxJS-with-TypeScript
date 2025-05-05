# Subjectとは

[📘 RxJS公式: Subject](https://rxjs.dev/api/index/class/Subject)

Subjectは、RxJSにおいて特殊な種類のObservableです。通常のObservableが単方向のデータフローを提供するのに対し、Subjectは「Observable」と「Observer」の両方の性質を持つハイブリッドな存在です。

Subjectは以下の特徴を持ちます。

- データを発行できる（Observable機能）
- データを購読できる（Observer機能）
- 複数の購読者に同じ値を届けられる（マルチキャスト）
- 購読後に発生した値のみを受け取る（Hot Observable的性質）


## Subjectの基本的な使い方

```ts
import { Subject } from 'rxjs';

// Subject（主体）を作成
const subject = new Subject<number>();

// Observerとして購読
subject.subscribe(value => console.log('Observer A:', value));
subject.subscribe(value => console.log('Observer B:', value));

// Observableとして値を発行
subject.next(1); // 両方の購読者に値を発行
subject.next(2); // 両方の購読者に値を発行

// 新たな購読者を追加（遅延購読）
subject.subscribe(value => console.log('Observer C:', value));

subject.next(3); // 全ての購読者に値を発行

// 完了を通知
subject.complete();
```

#### 実行結果
```
Observer A: 1
Observer B: 1
Observer A: 2
Observer B: 2
Observer A: 3
Observer B: 3
Observer C: 3
```

### 通常のObservableとの違い

Subjectは **Hot Observable** であり、通常のCold Observableとは以下の点で異なります。

- 購読の有無にかかわらずデータが発行される
- 複数の購読者に同じ値を共有できる（マルチキャスト）
- `.next()` で外部から値を発行できる
- 過去の値は保持されず、購読後の値のみを受け取る


## Subjectとマルチキャスティング

Subjectの重要な機能の一つが「マルチキャスティング」です。  
これは一つのデータソースを複数の購読者に効率的に配信する機能です。

```ts
import { Subject, interval } from 'rxjs';
import { take } from 'rxjs/operators';

// データソース
const source$ = interval(1000).pipe(take(3));

// マルチキャスト用のSubject
const subject = new Subject<number>();

// ソースをSubjectに接続
source$.subscribe(subject); // Subjectが購読者として機能

// 複数の観測者がSubjectを購読
subject.subscribe(value => console.log('Observer 1:', value));
subject.subscribe(value => console.log('Observer 2:', value));
```

#### 実行結果
```
Observer 1: 0
Observer 2: 0
Observer 1: 1
Observer 2: 1
Observer 1: 2
Observer 2: 2
```

このパターンはシングルソース・マルチキャストとも呼ばれ、一つのデータソースを複数の購読者に効率的に配信する際に使用されます。


## Subjectの2つの使い方

Subjectには主に2つの使い方があります。それぞれ、用途やふるまいが異なります。

### 1. 自分で `.next()` を呼ぶパターン

Subjectが **データ発行の主体（Observable）** として使われます。  
このパターンはイベント通知や状態更新のような「明示的な値の送信」に適しています。

```ts
const subject = new Subject<string>();

subject.subscribe(val => console.log('Observer A:', val));
subject.next('Hello');
subject.next('World');
```

> Observer A: Hello  
> Observer A: World

---

### 2. Observableを中継するパターン（マルチキャスト）

Subjectが **ObserverとしてObservableから値を受け取り、中継する** 役割を果たします。  
この使い方は、**Cold ObservableをHotに変換してマルチキャスト** するのに便利です。

```ts
const source$ = interval(1000).pipe(take(3));
const subject = new Subject<number>();

// Observable → Subject（中継）
source$.subscribe(subject);

// Subject → 複数購読者へ配信
subject.subscribe(val => console.log('Observer 1:', val));
subject.subscribe(val => console.log('Observer 2:', val));
```

> Observer 1: 0  
> Observer 2: 0  
> Observer 1: 1  
> Observer 2: 1  
> Observer 1: 2  
> Observer 2: 2

> [!TIP]
> `.next()`を自分で呼ぶ場合は「自ら話す人」、Observableから受け取って中継する場合は「他人の話をマイクで拡声する人」のようにイメージすると理解しやすくなります。


## Subjectの実践的なユースケース

Subjectは以下のようなシナリオで特に有用です。

1. **ステート管理** - アプリケーションの状態を共有・更新
2. **イベントバス** - コンポーネント間の通信
3. **HTTP応答の共有** - 同一APIコールの結果を複数コンポーネントで共有
4. **UIイベントの集中管理** - 様々なUI操作を一か所で処理

#### 例: イベントバスの実装
```ts
import { Subject } from 'rxjs';
import { filter } from 'rxjs/operators';

interface AppEvent {
  type: string;
  payload: any;
}

// アプリケーション全体のイベントバス
const eventBus = new Subject<AppEvent>();

// 特定のイベントタイプを購読
eventBus.pipe(
  filter(event => event.type === 'USER_LOGGED_IN')
).subscribe(event => {
  console.log('ユーザーログイン:', event.payload);
});

// 別のイベントタイプを購読
eventBus.pipe(
  filter(event => event.type === 'DATA_UPDATED')
).subscribe(event => {
  console.log('データ更新:', event.payload);
});

// イベント発行
eventBus.next({ type: 'USER_LOGGED_IN', payload: { userId: '123', username: 'test_user' } });
eventBus.next({ type: 'DATA_UPDATED', payload: { items: [1, 2, 3] } });
```

## まとめ

Subjectは、RxJSエコシステムにおいて以下の役割を果たす重要な構成要素です。

- Observer（観測者）とObservable（被観測者）の両方の特性を持つ
- コールドなObservableをホットに変換する手段を提供
- 複数の購読者に同じデータストリームを効率的に配信
- コンポーネント間やサービス間の通信を容易にする
- 状態管理やイベント処理のための基盤を提供

Subject系には、この基本Subject以外にも、BehaviorSubject、ReplaySubject、AsyncSubjectなど特化型のSubjectも存在します。それらについては[Subjectの種類](./types-of-subject.md)で詳しく解説します。