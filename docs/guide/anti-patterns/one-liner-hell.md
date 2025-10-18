---
description: RxJSの「ワンライナー地獄」を解消する段階分離構文を詳しく解説します。ストリーム定義・変換・購読を明確に分離することで、デバッグしやすく、テストしやすい、読みやすいコードを実現します。
---

# ワンライナー地獄と段階分離構文

RxJSコードが「ワンライナー地獄」に見える主な原因は、**「ストリームの定義」「変換」「購読（副作用）」がごちゃ混ぜ**になっているからです。これは可読性とデバッグ性を著しく下げます。

## なぜ「ワンライナー地獄」が起きるのか

### ❌ よくある問題コード

```ts
import { fromEvent } from 'rxjs';
import { map, filter, debounceTime, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

fromEvent(document, 'click')
  .pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    debounceTime(300),
    switchMap(x => ajax(`/api?x=${x}`))
  )
  .subscribe(res => {
    if (res.status === 200) {
      console.log('OK');
    } else {
      handleError(res);
    }
  });

function handleError(res: any) {
  console.error('Error:', res);
}
```

### 問題点

| 問題 | 影響 |
|---|---|
| **1行が長い** | 読む人が迷子になる |
| **デバッグ困難** | 途中の状態を確認しにくい |
| **テスト困難** | ストリーム全体をテストするしかない |
| **処理構造がネスト** | subscribe 内で条件分岐が深くなりがち |
| **再利用不可** | パイプライン処理を他で使えない |


## 解決策：段階分離構文（Functional Style）

RxJSコードを「関係が明確な3段構成」に整理します。

1. **ストリーム定義（source）** - データの発生源
2. **ストリーム変換（pipeline）** - データの加工処理
3. **購読と副作用（subscription）** - UI更新やログなどの副作用


## 推奨パターン：段階分離構文

```ts
import { fromEvent } from 'rxjs';
import { map, filter, throttleTime } from 'rxjs';

// 1. Observable 定義（ストリームの発生源）
const clicks$ = fromEvent(document, 'click');

// 2. パイプライン定義（データの変換処理）
const processed$ = clicks$.pipe(
  map(event => (event as MouseEvent).clientX),
  filter(x => x > 100),
  throttleTime(200)
);

// 3. 購読処理（副作用の実行）
const subscription = processed$.subscribe({
  next: x => console.log('クリック位置:', x),
  error: err => console.error(err),
  complete: () => console.log('完了')
});
```

### メリット

| メリット | 詳細 |
|---|---|
| **ステップごとに意味が明確** | 各段階の責務が一目で分かる |
| **デバッグしやすい** | 途中のストリームを `console.log` や `tap` で確認可能 |
| **テストしやすい** | `processed$` などの中間ストリームを単体でテスト可能 |
| **ネストが浅い** | subscribe 内の処理がシンプルに |
| **再利用可能** | パイプライン処理を関数として切り出せる |


## バリエーション：関数分離（モジュール化）

変換処理が長くなる場合は、**パイプラインを関数として分離**します。

```ts
import { Observable } from 'rxjs';
import { map, filter, distinctUntilChanged } from 'rxjs';
import { fromEvent } from 'rxjs';

// パイプライン処理を関数として切り出し
function transformClicks(source$: Observable<Event>): Observable<number> {
  return source$.pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    distinctUntilChanged()
  );
}

// 使用側
const clicks$ = fromEvent(document, 'click');
const xPosition$ = transformClicks(clicks$);
const subscription = xPosition$.subscribe(x => console.log(x));
```

**ポイント:** 「どう変換するか」を純関数として切り出すと、**テスト容易性が爆増**します。


## 命名規則（Naming Rule）

適切な命名で、コードの意図を明確にします。

| 段階 | 命名例 | 意味 |
|---|---|---|
| **ソース** | `clicks$`, `input$`, `routeParams$` | イベントやデータの発生源 |
| **パイプ** | `processed$`, `validInput$`, `apiResponse$` | 加工済みストリーム |
| **サブスクリプション** | `subscription`, `uiSubscription` | 実際に実行される副作用 |

**`$` サフィックス**をつけることで「Observableであること」が一目で分かります。


## より宣言的に書く場合（RxJS 7以降）

`pipe` を関数として切り出し、再利用可能にします。

```ts
import { pipe, fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// パイプラインを関数として定義（再利用可能）
const processClicks = pipe(
  map((ev: MouseEvent) => ev.clientX),
  filter(x => x > 100)
);

const clicks$ = fromEvent(document, 'click');
const processed$ = clicks$.pipe(processClicks);
processed$.subscribe(x => console.log(x));
```

**メリット:** 処理ロジック（`processClicks`）を別のストリームでも再利用可能。


## Before/After：典型パターン別リファクタ

実際のユースケースでの改善例を紹介します。

### A. UIイベント → API → UI更新

#### ❌ Before（ワンライナー地獄）

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, switchMap, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

interface ApiRes {
  items: string[];
  error?: string;
}

const button = document.getElementById('btn') as HTMLButtonElement;
const list = document.getElementById('list') as HTMLElement;

fromEvent(button, 'click').pipe(
  throttleTime(500),
  switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
  catchError(err => of({ items: [], error: err.message }))
).subscribe(res => {
  list.innerHTML = res.items.map(item => `<li>${item}</li>`).join('');
  if (res.error) alert(res.error);
});
```

#### ✅ After（段階分離＋関数化）

```ts
import { fromEvent, pipe, of } from 'rxjs';
import { throttleTime, switchMap, map, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface ApiRes {
  items: string[];
}

interface Result {
  items: string[];
  error: string | null;
}

const button = document.getElementById('btn') as HTMLButtonElement;
const list = document.getElementById('list') as HTMLElement;

// 1) source
const clicks$ = fromEvent(button, 'click');

// 2) pipeline（純関数に抽出）
const loadItems = () =>
  pipe(
    throttleTime(500),
    switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
    map((res: ApiRes) => ({ items: res.items, error: null as string | null })),
    catchError(err => of({ items: [] as string[], error: String(err?.message ?? err) }))
  );

const result$ = clicks$.pipe(loadItems());

// 3) subscription（副作用だけ）
const subscription = result$.subscribe(({ items, error }) => {
  renderList(items);
  if (error) toast(error);
});

function renderList(items: string[]) {
  list.innerHTML = items.map(item => `<li>${item}</li>`).join('');
}

function toast(message: string) {
  alert(message);
}
```

**改善点:**
- パイプライン処理 `loadItems()` を純関数化
- 副作用（`renderList`, `toast`）を subscribe 側に集約
- テストしやすく、デバッグしやすい


### B. フォーム値 → バリデーション → API保存（自動保存）

#### ❌ Before

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

fromEvent(input, 'input')
  .pipe(
    map((e: Event) => (e.target as HTMLInputElement).value),
    debounceTime(400),
    distinctUntilChanged(),
    filter(v => v.length >= 3),
    switchMap(v => ajax.post('/api/save', { v }))
  )
  .subscribe(
    () => console.log('OK'),
    err => alert(err.message)
  );
```

#### ✅ After（責務分離＋名前付け）

```ts
import { fromEvent, pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

// 1) source
const value$ = fromEvent<Event>(input, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value)
);

// 2) pipeline（バリデーション）
const validate = () =>
  pipe(
    debounceTime(400),
    distinctUntilChanged(),
    filter((v: string) => v.length >= 3)
  );

// 2) pipeline（自動保存）
const autosave = () =>
  pipe(
    switchMap((v: string) => ajax.post('/api/save', { v }))
  );

const save$ = value$.pipe(validate(), autosave());

// 3) subscription
const subscription = save$.subscribe({
  next: () => showSuccess(),
  error: (err) => showError(String(err?.message ?? err))
});

function showSuccess() {
  console.log('保存しました');
}

function showError(message: string) {
  alert(message);
}
```

**改善点:**
- バリデーション (`validate`) と保存 (`autosave`) を分離
- 各パイプラインが再利用可能に
- テストが容易（バリデーションと保存を個別にテスト可能）


### C. キャッシュ＋手動リフレッシュ

```ts
import { merge, of, Subject } from 'rxjs';
import { switchMap, shareReplay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Item {
  id: number;
  name: string;
}

const refreshBtn = document.getElementById('refresh-btn') as HTMLButtonElement;

// 1) sources
const refresh$ = new Subject<void>();
const initial$ = of(void 0);

// 2) pipeline
const fetchItems$ = merge(initial$, refresh$).pipe(
  switchMap(() => ajax.getJSON<Item[]>('/api/items')),
  shareReplay({ bufferSize: 1, refCount: true }) // メモ化
);

// 3) subscription
const subscription = fetchItems$.subscribe(items => renderList(items));

// UIから再読込
refreshBtn?.addEventListener('click', () => refresh$.next());

function renderList(items: Item[]) {
  console.log('Items:', items);
}
```

**ポイント:**
- 初回自動ロード (`initial$`) と手動リフレッシュ (`refresh$`) を分離
- `shareReplay` で最新値をキャッシュ
- 複数の購読者が同じ結果を共有


## 上級：中間ログを埋め込みたい場合

`tap()` で各段階を観察できます。

```ts
import { fromEvent } from 'rxjs';
import { map, tap } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

const processed$ = clicks$.pipe(
  tap(() => console.log('クリック発生')),
  map(e => (e as MouseEvent).clientX),
  tap(x => console.log('X座標:', x))
);

processed$.subscribe(x => console.log('最終値:', x));
```

**ポイント:**
- `tap` は副作用専用オペレーター
- デバッグ時に各段階の値を確認できる
- 本番環境では削除すべき


## テスト容易性の実証

段階分離により、**パイプライン処理を単体でテスト**できます。

### 例：入力バリデーションのテスト

```ts
// validate.ts
import { pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter } from 'rxjs';

export const validateQuery = () =>
  pipe(
    map((s: string) => s.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((s) => s.length >= 3)
  );
```

```ts
// validate.spec.ts
import { TestScheduler } from 'rxjs/testing';
import { validateQuery } from './validate';

describe('validateQuery', () => {
  it('trims, debounces, distincts, filters length>=3', () => {
    const scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });

    scheduler.run(({ hot, expectObservable }) => {
      // 入力: " a ", "ab", "abc", "abc ", "abcd"
      const input = hot<string>('-a-b-c--d-e----|', {
        a: ' a ',
        b: 'ab',
        c: 'abc',
        d: 'abc ',
        e: 'abcd'
      });

      const output$ = input.pipe(validateQuery());

      // 期待: 'abc' と 'abcd' のみ通過
      expectObservable(output$).toBe('--------c-----e-|', {
        c: 'abc',
        e: 'abcd'
      });
    });
  });
});
```

**メリット:**
- パイプライン処理を**単独で**テスト可能
- DOM/HTTPに依存しない = **高速・安定**
- マーブルテストで時間軸を制御

詳細は [テスト手法](/guide/testing/unit-tests) を参照してください。


## GitHub Copilot 指示テンプレート

実際のリファクタで使えるプロンプト集です。

### 1. 三段構成への分解

```
このRxJSコードを「source / pipeline / subscription」の3段に分解してリファクタして。
要件:
- Observableは $ サフィックスを付けて命名
- pipelineは pipe(...) を返す関数として抽出（例: validate(), loadItems()）
- 副作用(UI更新, console, toast)は subscribe 内に集約
- 途中状態をtapで観察できるように適所にtap()を入れる（コメント付き）
- 変数名と関数名はドメインが伝わる名前に
```

### 2. オペレータ選択の明確化

```
多発クリックによる多重API呼び出しを防ぎたい。
現在の switchMap/mergeMap/concatMap/exhaustMap のどれを使うべきか提案して、
正しいオペレータに置き換えて。根拠をコメントで書いて。

ガイドライン:
- フォーム保存は順次処理（concatMap）
- 検索候補は古いリクエストを破棄（switchMap）
- ボタン連打は二重実行禁止（exhaustMap）
```

### 3. 自動保存パターン

```
以下のコードを自動保存パターンにリファクタ:
- 入力は debounceTime と distinctUntilChanged
- 保存は concatMap で直列化
- 成功/失敗をUIへ通知する副作用は subscribe 側に寄せる
- テストしやすいように変換を関数化
- 可能なら shareReplay で最新状態をキャッシュ
```

### 4. キャッシュ＋手動リフレッシュ

```
「初回自動ロード＋手動リフレッシュ」パターンに変更して:
- refresh$ Subject を導入
- merge(initial$, refresh$) → switchMap(fetch)
- 最新値を shareReplay({bufferSize:1, refCount:true}) でキャッシュ
- 再利用できるよう fetch パイプを関数抽出
```


## 結論：読みやすく書くための指針まとめ

| 項目 | 推奨内容 |
|---|---|
| ✅ 1 | Observable・pipe・subscribeを**分けて書く** |
| ✅ 2 | 中間ストリームは**変数名で意味を示す** |
| ✅ 3 | 複雑なpipeは**関数化** |
| ✅ 4 | **tap()で途中確認**を可能に |
| ✅ 5 | `processSomething = pipe(...)`で再利用可能に |


## まとめ

- **ワンライナー地獄**は、ストリーム定義・変換・購読が混在することで発生
- **段階分離構文**（Source → Pipeline → Subscription）で責務を明確化
- **パイプラインを関数化**することで、テスト容易性と再利用性が向上
- **適切な命名**（`$`サフィックス、意味のある変数名）で可読性が向上

## 関連セクション

- **[よくある間違いと対処法](/guide/anti-patterns/common-mistakes#13-過度な複雑化)** - 過度な複雑化のアンチパターン
- **[テスト手法](/guide/testing/unit-tests)** - RxJSコードのテスト方法
- **[オペレーターの理解](/guide/operators/)** - 各オペレーターの使い方

## 次のステップ

1. 既存のコードで「ワンライナー地獄」になっている箇所を探す
2. 段階分離構文でリファクタする
3. パイプライン処理を関数化して、単体テストを書く
4. Copilot指示テンプレートを活用して、チーム全体で統一する


> [!NOTE]
> より包括的な「読みやすいRxJSの書き方」は、今後 **Chapter 12: 実践パターン** で扱う予定です。
