---
title: 状態管理フラグの乱立問題
description: RxJSプロジェクトにおける17個のbooleanフラグを、リアクティブ設計に改善する方法
# outline: deep
---

# 状態管理フラグの乱立

RxJSを導入したプロジェクトでも、コンポーネント内に大量のbooleanフラグが乱立する問題がよく見られます。この記事では、17個ものフラグが存在する実際の事例を元に、その原因と改善方法を解説します。

## 問題の実例

まず、実際の現場で遭遇したコードを見てみましょう。以下は、状態管理フラグが乱立している典型的な例です

```typescript
class ProblematicComponent {
  // 17個のフラグが存在
  isLoading = false;
  isSaving = false;
  isDeleting = false;
  isEditing = false;
  hasError = false;
  isFormDirty = false;
  isFormValid = false;
  isDataLoaded = false;
  isUserAuthenticated = false;
  isModalOpen = false;
  isProcessing = false;
  isInitialized = false;
  isUpdating = false;
  isRefreshing = false;
  hasUnsavedChanges = false;
  isSubmitting = false;
  isValidating = false;

  save() {
    // subscribe内で複雑な分岐
    this.apiService.save(this.data).subscribe({
      next: (result) => {
        if (this.isLoading && !this.isSaving) {
          if (this.isFormValid && this.isDataLoaded) {
            if (!this.hasError && !this.isProcessing) {
              // 実際の処理
              this.isSaving = false;
              this.hasUnsavedChanges = false;
            }
          }
        }
      },
      error: (err) => {
        this.isSaving = false;
        this.hasError = true;
        this.isProcessing = false;
      }
    });
  }
}
```

このようなコードは、**RxJSを導入していても**発生します。17個のフラグを手動で管理し、複雑な条件分岐で制御するこのパターンは、保守性・可読性・テスト容易性すべてにおいて問題があります。

## なぜフラグが乱立するのか

フラグが乱立する背景には、技術的な問題だけでなく、開発者の思考パターンや組織の進化過程が関係しています。以下、5つの主要な原因を分析します。

### 原因の構造分析

| 原因カテゴリ | 具体的な症状 | 背景 |
|------------|------------|------|
| **① 命令型思考の残存** | `isLoading`, `isSaving`, `isError` などが10個以上<br>`if (this.isSaving) return;` のようなガードが大量 | RxJSのストリームではなく、**命令型の「状態フラグ」制御**でロジックを分岐。<br>状態と副作用が分離できず、可読性が低下 |
| **② 派生状態の未活用** | コンポーネント側で直接 `this.isLoaded = true;` のように代入して管理 | Observableの`map`や`combineLatest`を活かせば状態の導出を宣言的に定義できるのに、<br>それをせずに手動で状態を合成 |
| **③ 状態設計の責務が曖昧** | 同じ状態に関して複数のフラグ<br>（例：`isLoadingStart`, `isLoadingEnd`）が存在 | **状態変化を命令として扱っている**。<br>「1つの状態」として統合すべきものを複数のフラグに分散 |
| **④ RxJSストリーム分岐が未整理** | 1つの`Observable`内で複数の`if`や`tap`が連なり、<br>副作用と状態更新が混在 | ストリーム設計の責務分離ができていない。<br>`switchMap`や`catchError`の使い方が曖昧 |
| **⑤ ViewModel層の欠如** | UIコンポーネントで `this.isEditing`, `this.isSaved` を直に操作 | 状態をコンポーネント内に持つことで、<br>RxJSの恩恵が断たれている |


## 根本原因：思考モデルの不一致

フラグ乱立の根本原因は、**命令型プログラミングとリアクティブプログラミングの思考モデルの不一致**です。開発者が命令型の発想のままRxJSを使うと、以下のような問題が発生します。

### 過渡期の構造

多くのプロジェクトでは、以下のような進化過程を経てフラグ地獄に陥ります。

```
1. とりあえず動かすために if フラグで制御を追加
   ↓
2. 後から RxJS を導入
   ↓
3. 旧ロジックをストリーム化できず混在
   ↓
4. フラグ地獄が完成
```

### 状態管理の層が混在

アプリケーション内の状態は、本来3つの層に分けて管理すべきです。

```
アプリケーション
 ├── View状態 (isOpen, isLoading, formDirty)     ← コンポーネント内
 ├── Business状態 (entity, filters, errors)      ← 状態管理層
 └── API状態 (pending, success, error)           ← RxJS stream
```

この3層が分離されていないと、同じ「フラグ」でも**責務の異なる3種類**が入り混じります。View状態とAPI状態を同じレベルで管理すると、複雑さが爆発的に増加します。

## 問題の本質：フラグの「性質」

フラグ乱立の真の問題は「個数が多い」ことではなく、**フラグが命令型のmutable変数になっている**ことです。以下で、問題のあるフラグと適切なフラグの違いを比較します。

### ❌ 問題のあるフラグ：命令型のmutable変数

```typescript
class BadComponent {
  // これらは「状態」ではなく「命令」になっている
  isLoading = false;
  isSaving = false;
  hasError = false;

  save() {
    if (this.isSaving) return;        // ガード節が必要
    this.isSaving = true;              // 手動で変更

    this.api.save().subscribe({
      next: () => {
        this.isSaving = false;         // 手動でリセット
        this.hasError = false;         // 他のフラグも手動管理
      },
      error: () => {
        this.isSaving = false;         // 同じ処理を複数箇所に
        this.hasError = true;
      }
    });
  }
}
```

> [!WARNING] 問題点
> - 状態が「宣言的」でなく「手続き的」
> - 状態変更のタイミングが散在
> - フラグ間の整合性を開発者が手動保証

### ✅ 適切なフラグ：リアクティブ変数

```typescript
class GoodComponent {
  // 状態ストリームとして宣言
  private saveAction$ = new Subject<void>();

  readonly saveState$ = this.saveAction$.pipe(
    switchMap(() =>
      this.api.save().pipe(
        map(() => 'success' as const),
        catchError(() => of('error' as const)),
        startWith('loading' as const)
      )
    ),
    startWith('idle' as const),
    shareReplay(1)
  );

  // 派生状態も宣言的に定義
  readonly isLoading$ = this.saveState$.pipe(
    map(state => state === 'loading')
  );

  readonly hasError$ = this.saveState$.pipe(
    map(state => state === 'error')
  );

  save() {
    this.saveAction$.next(); // イベント発火のみ
  }
}
```

> [!TIP] 改善点
> - 状態が「ストリーム」として一元管理
> - 状態遷移がパイプラインで宣言的に定義
> - フラグ間の整合性が自動保証


## フラグ設計の判断基準

自分のコードが問題のあるフラグ設計になっているかを判断するための基準を以下にまとめました。コードレビューや設計時の参考にしてください。

| 観点 | ❌ 問題あり | ✅ 問題なし |
|------|-----------|-----------|
| **型** | `boolean` (mutable) | `Observable<boolean>` / `Signal<boolean>` |
| **変更方法** | 直接代入 `flag = true` | ストリーム/派生 `state$.pipe(map(...))` |
| **依存関係** | 暗黙的（コード順序） | 明示的（combineLatest, computed） |
| **命名** | `xxxFlag`, `isXXX` (boolean) | `xxxState`, `canXXX`, `shouldXXX` |
| **数** | 10個以上の独立したboolean | 1つの状態 + 複数の派生 |


## 改善戦略

フラグ乱立問題を解決するには、以下の3ステップで段階的にリファクタリングを進めます。

### Step 1: 状態の棚卸し

まず、現在のフラグをすべて列挙し、責務ごとに分類します。これにより、どのフラグが統合可能かが見えてきます。

```typescript
// 既存フラグを列挙し、責務を分類
interface StateInventory {
  view: string[];      // UI表示制御 (isModalOpen, isEditing)
  business: string[];  // ビジネスロジック (isFormValid, hasUnsavedChanges)
  api: string[];       // 通信状態 (isLoading, isSaving, hasError)
}
```

### Step 2: 状態のEnum化

次に、関連する複数のbooleanフラグを、1つの状態として統合します。例えば、`isLoading`、`isSaving`、`hasError`は、すべて「リクエストの状態」として統合できます。

```typescript
// 複数のbooleanを1つの状態に統合
enum RequestState {
  Idle = 'idle',
  Loading = 'loading',
  Success = 'success',
  Error = 'error'
}

// 使用例
class Component {
  saveState: RequestState = RequestState.Idle;
  // isLoading, isSaving, hasError が不要に
}
```

### Step 3: リアクティブ化

最後に、状態をObservableまたはSignalで管理し、派生状態を宣言的に定義します。これにより、状態の整合性が自動的に保証されます。

```typescript
// ObservableまたはSignalで管理
class ReactiveComponent {
  private readonly apiState$ = new BehaviorSubject<ApiState>({
    loading: false,
    saving: false,
    error: null
  });

  private readonly formState$ = this.form.valueChanges.pipe(
    map(() => ({
      dirty: this.form.dirty,
      valid: this.form.valid
    })),
    startWith({ dirty: false, valid: false })
  );

  // ViewModelとして統合
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$
  ]).pipe(
    map(([api, form]) => ({
      canSave: !api.saving && form.valid,
      showSpinner: api.loading || api.saving,
      showError: api.error !== null
    }))
  );
}
```


## 実装例：17フラグのリファクタリング

ここでは、冒頭で紹介した17個のフラグを持つコンポーネントを、実際にリアクティブ設計にリファクタリングする過程を示します。Before/Afterを比較することで、改善の効果を実感できるでしょう。

### Before：命令型のフラグ管理

まず、問題のあるコードを再確認します。17個のbooleanフラグが乱立し、複雑な条件分岐で制御されています。

```typescript
class LegacyComponent {
  isLoading = false;
  isSaving = false;
  isDeleting = false;
  isEditing = false;
  hasError = false;
  isFormDirty = false;
  isFormValid = false;
  isDataLoaded = false;
  isUserAuthenticated = false;
  isModalOpen = false;
  isProcessing = false;
  isInitialized = false;
  isUpdating = false;
  isRefreshing = false;
  hasUnsavedChanges = false;
  isSubmitting = false;
  isValidating = false;

  save() {
    if (!this.isLoading &&
        !this.isSaving &&
        this.isFormValid &&
        !this.hasError &&
        this.isDataLoaded) {
      this.isSaving = true;
      this.apiService.save().subscribe({
        next: () => {
          this.isSaving = false;
          this.hasUnsavedChanges = false;
        },
        error: () => {
          this.isSaving = false;
          this.hasError = true;
        }
      });
    }
  }
}
```

### After：リアクティブな状態管理

次に、改善後のコードを見てみましょう。17個のフラグが、3つの基本状態（apiState$、formState$、dataState$）と1つの派生状態（vm$）に整理されています。

```typescript
import { BehaviorSubject, combineLatest, EMPTY } from 'rxjs';
import { map, switchMap, catchError, startWith } from 'rxjs/operators';

interface ApiState {
  loading: boolean;
  saving: boolean;
  deleting: boolean;
  error: string | null;
}

interface DataState {
  loaded: boolean;
  editing: boolean;
}

class RefactoredComponent {
  // 基本状態をObservableで管理
  private readonly apiState$ = new BehaviorSubject<ApiState>({
    loading: false,
    saving: false,
    deleting: false,
    error: null
  });

  private readonly formState$ = this.form.valueChanges.pipe(
    map(() => ({
      dirty: this.form.dirty,
      valid: this.form.valid
    })),
    startWith({ dirty: false, valid: false })
  );

  private readonly dataState$ = new BehaviorSubject<DataState>({
    loaded: false,
    editing: false
  });

  // ViewModelとして統合（派生状態）
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$,
    this.dataState$,
    this.authService.isAuthenticated$
  ]).pipe(
    map(([api, form, data, auth]) => ({
      // UI表示用の派生状態
      canSave: !api.saving && form.valid && data.loaded && auth,
      showSpinner: api.loading || api.saving || api.deleting,
      showError: api.error !== null,
      errorMessage: api.error,
      // 必要に応じて個別の状態も公開
      isEditing: data.editing,
      formDirty: form.dirty
    }))
  );

  save() {
    // 状態チェックはViewModelが自動で行う
    this.apiState$.next({
      ...this.apiState$.value,
      saving: true,
      error: null
    });

    this.apiService.save().pipe(
      catchError(error => {
        this.apiState$.next({
          ...this.apiState$.value,
          saving: false,
          error: error.message
        });
        return EMPTY;
      })
    ).subscribe(() => {
      this.apiState$.next({
        ...this.apiState$.value,
        saving: false
      });
    });
  }
}
```

### UI側での利用

リアクティブな状態管理によって、UI側での利用も大幅に簡潔になります。複数のフラグを個別にチェックする必要がなくなり、ViewModelから必要な情報を取得するだけで済みます。

```typescript
// Before：複数のフラグを直接参照
const isButtonDisabled =
  this.isLoading ||
  this.isSaving ||
  !this.isFormValid ||
  this.hasError ||
  !this.isDataLoaded;

// After：ViewModelから派生状態を取得
this.vm$.subscribe(vm => {
  const isButtonDisabled = !vm.canSave;
  const showSpinner = vm.showSpinner;
  const errorMessage = vm.errorMessage;
});
```


## 命名規則の重要性

フラグ設計において、命名は非常に重要です。適切な命名によって、そのフラグの責務・性質・ライフサイクルが一目で理解できます。逆に曖昧な命名は、混乱の元凶となります。

### ❌ 悪い命名例

以下のような命名は、意図が不明確で保守性を下げます。

```typescript
// 何のフラグ？何をトリガーに変わる？
userFlag: boolean;
dataFlag: boolean;
checkFlag: boolean;

// 状態なのか？アクションなのか？
isProcess: boolean;  // 処理中？処理済み？
```

### ✅ 良い命名例

適切な命名は、状態の意図と性質を明確に表現します。Observable（`$`サフィックス）やSignalを使い、状態の種類（State、can、should）を明確にします。

```typescript
// 状態を明確に表現
readonly userLoadState$: Observable<'idle' | 'loading' | 'loaded' | 'error'>;

// 派生状態も意図が明確
readonly canSubmit$: Observable<boolean>;
readonly shouldShowSpinner$: Observable<boolean>;

// Signalを使った例（Angular、Preact、Solid.js等で利用可能）
readonly userLoadState = signal<LoadState>('idle');
readonly canSubmit = computed(() =>
  this.userLoadState() === 'loaded' && this.formValid()
);
```


## 診断チェックリスト

自分のコードがフラグ乱立問題に陥っていないか、以下のチェックリストで確認してみましょう。コードレビューや設計時の参考にしてください。

```markdown
## 🚨 危険信号

- [ ] boolean変数が5個以上ある
- [ ] `subscribe`内に`if`文が3つ以上ネストしている
- [ ] 同じフラグを複数箇所でセットしている
- [ ] `isXXXing`という命名が3つ以上ある
- [ ] 状態管理層があるのにコンポーネント内で状態を持っている
- [ ] `xxxFlag`という命名が複数ある
- [ ] エラーハンドリングが各`subscribe`に散在している

## ✅ 改善の兆し

- [ ] 状態が`Observable`または`Signal`で管理されている
- [ ] 派生状態が`map`/`computed`で定義されている
- [ ] 状態遷移が宣言的に記述されている
- [ ] ViewModelパターンが適用されている
- [ ] 命名が意図を明確に表現している
```

## まとめ

この記事では、RxJSプロジェクトにおけるフラグ乱立問題の原因と改善方法を解説しました。最後に、重要なポイントを振り返りましょう。

### 問題の本質

1. **フラグが17個あること** ← これは症状
2. **それらが命令型のmutable変数であること** ← これが本質
3. **状態遷移が宣言的でないこと** ← これが原因
4. **命名が曖昧（xxxFlag）であること** ← これが混乱の元

### 改善の方向性

フラグ乱立問題を解決するには、以下の4つの転換が必要です。

- **boolean変数** → **Observable/Signal**
- **直接代入** → **ストリームパイプライン**
- **独立した17個** → **1つの状態 + 派生状態**
- **xxxFlag** → **xxxState$ / canXXX$**

### 最も重要なこと

> [!IMPORTANT] 重要な原則
> 「状態はイベントの結果であり、フラグで直接制御しない」

RxJSの導入は「構文」ではなく「思想」の転換です。命令型思考を引きずると、フラグ地獄は解消されません。状態をストリームとして捉え、宣言的に設計することで、保守性・可読性・テスト容易性のすべてが向上します。


## 関連セクション

この記事で学んだフラグ管理の知識をさらに深めるために、以下の関連記事も参照してください。

- [subscribe内のif文ネスト地獄](./subscribe-if-hell) - 条件分岐の適切な処理方法
- [よくある間違いと対処法](./common-mistakes) - 15のアンチパターン詳細
- [エラーハンドリング](/guide/error-handling/strategies) - 適切なエラー処理戦略
- [Subjectとマルチキャスト](/guide/subjects/what-is-subject) - 状態管理の基礎

## 参考リソース

RxJSの公式ドキュメントや学習リソースで、さらに深く学習できます。

- [RxJS公式ドキュメント](https://rxjs.dev/) - 公式のAPIリファレンスとガイド
- [Learn RxJS](https://www.learnrxjs.io/) - オペレーター別の実践的な例
- [RxJS Marbles](https://rxmarbles.com/) - ビジュアルでオペレーターの動作を理解
