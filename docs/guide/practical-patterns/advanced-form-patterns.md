---
description: "JSON Patchを使った高度なフォームパターン。大規模フォームの自動保存とUndo/Redo、共同編集のリアルタイム同期、オフライン対応、操作履歴の追跡など、エンタープライズレベルのフォーム実装をRxJSとTypeScriptで構築する方法を解説します。"
---

# JSON Patchを使った高度なフォームパターン

大規模なフォームやリアルタイム共同編集を実装する際、従来の「フォーム全体を送信する」アプローチではパフォーマンスやユーザー体験に課題が生じます。

この記事では、**JSON Patch（RFC 6902）** を使った高度なフォームパターンを解説します。差分のみを送信することで、ネットワーク帯域を削減し、Undo/Redoや共同編集を効率的に実装できます。

## この記事で学べること

- JSON Patch/Pointerの基礎（RFC 6902/6901）
- 大規模フォームの自動保存（差分ベース）
- Undo/Redoの実装（逆パッチ）
- 共同編集のリアルタイム同期
- Operational Transform (OT) / CRDTの基礎
- WebSocketとRxJSの統合パターン
- 競合解決とバージョン管理

> [!TIP] 前提知識
> この記事は、[Chapter 4: オペレーター](../operators/index.md)、[フォーム処理パターン](./form-handling.md)、[リアルタイムデータ処理](./real-time-data.md) の知識を前提としています。

> [!NOTE] いつこのパターンが必要か
> - **大規模フォーム**（100フィールド以上）で自動保存が必要
> - **Undo/Redo**機能が必須
> - **リアルタイム共同編集**（Google Docsのような機能）
> - **オフライン対応**で差分キューイングが必要
>
> 小規模フォーム（～20フィールド）なら、[通常のフォーム処理パターン](./form-handling.md) で十分です。

## JSON Patch/Pointerの基礎

### JSON Patchとは

**JSON Patch（RFC 6902）** は、JSON文書の変更を表現するための標準フォーマットです。フォーム全体ではなく、**変更内容だけ**を送信できます。

```typescript
// 変更前のフォームデータ
const before = {
  profile: {
    name: "田中太郎",
    email: "tanaka@example.com",
    age: 30
  }
};

// 変更後のフォームデータ
const after = {
  profile: {
    name: "田中太郎",
    email: "tanaka.updated@example.com", // 変更
    age: 31 // 変更
  }
};

// JSON Patch（差分）
const patch = [
  { op: "replace", path: "/profile/email", value: "tanaka.updated@example.com" },
  { op: "replace", path: "/profile/age", value: 31 }
];
```

> [!NOTE] JSON Patchの6つの操作
> - `add` - 値を追加
> - `remove` - 値を削除
> - `replace` - 値を置換
> - `move` - 値を移動
> - `copy` - 値をコピー
> - `test` - 値をテスト（バリデーション）

### JSON Pointerとは

**JSON Pointer（RFC 6901）** は、JSON文書内の特定の値を指すパス表記法です。

```typescript
const formData = {
  user: {
    profile: {
      name: "田中太郎"
    },
    settings: {
      notifications: true
    }
  }
};

// JSON Pointer examples
"/user/profile/name"           // → "田中太郎"
"/user/settings/notifications" // → true
"/user/profile"                // → { name: "田中太郎" }
```

### RxJSでの差分検出

`pairwise()` と `fast-json-patch` ライブラリを組み合わせて、フォームの変更を自動検出します。

```typescript
import { BehaviorSubject, pairwise, map } from 'rxjs';
import { compare } from 'fast-json-patch';

interface FormData {
  profile: {
    name: string;
    email: string;
    age: number;
  };
}

const initialData: FormData = {
  profile: {
    name: "",
    email: "",
    age: 0
  }
};

const formData$ = new BehaviorSubject<FormData>(initialData);

// 差分を検出
const patches$ = formData$.pipe(
  pairwise(), // [前の値, 現在の値] のペアを取得
  map(([previous, current]) => compare(previous, current))
);

patches$.subscribe(patches => {
  console.log('検出された変更:', patches);
  // 例: [{ op: "replace", path: "/profile/name", value: "田中太郎" }]
});

// フォーム更新をシミュレート
formData$.next({
  profile: {
    name: "田中太郎",
    email: "tanaka@example.com",
    age: 30
  }
});
```

> [!TIP] fast-json-patch ライブラリ
> ```bash
> npm install fast-json-patch
> ```
> - RFC 6902完全準拠
> - 差分生成（`compare`）と適用（`applyPatch`）
> - 逆パッチ生成（Undo用）
> - TypeScript対応

---

## 大規模フォームの自動保存とUndo/Redo

大規模フォーム（例：100フィールドの会員登録、商品管理画面）で、自動保存とUndo/Redo機能を実装します。

### 実装方針

**フロントエンドの責務:**
- 操作（変更）の生成と整列
- 楽観的UI反映（`scan` による即時適用）
- Undo/Redoスタック管理（逆パッチによる履歴）
- 送信キュー管理（`concatMap` で順序保証）
- バッチング（`bufferTime` + 圧縮）

**バックエンドの責務:**
- バージョン管理（Vector Clock / タイムスタンプ）
- 冪等性保証（Request ID による重複検出）
- 永続化と監査ログ

### パターン1: 基本的な自動保存

フォームの変更を検出し、一定時間ごとにバッチでサーバーに送信します。

```typescript
import {
  BehaviorSubject,
  Subject,
  pairwise,
  map,
  bufferTime,
  filter,
  concatMap,
  catchError,
  of
} from 'rxjs';
import { compare, Operation } from 'fast-json-patch';

interface LargeFormData {
  personalInfo: {
    firstName: string;
    lastName: string;
    email: string;
    phone: string;
    dateOfBirth: string;
  };
  address: {
    street: string;
    city: string;
    state: string;
    zipCode: string;
    country: string;
  };
  preferences: {
    newsletter: boolean;
    notifications: boolean;
    theme: 'light' | 'dark';
  };
  // ... 100 フィールド以上を想定
}

const initialFormData: LargeFormData = {
  personalInfo: {
    firstName: "",
    lastName: "",
    email: "",
    phone: "",
    dateOfBirth: ""
  },
  address: {
    street: "",
    city: "",
    state: "",
    zipCode: "",
    country: "Japan"
  },
  preferences: {
    newsletter: false,
    notifications: true,
    theme: 'light'
  }
};

// フォームデータのストリーム
const formData$ = new BehaviorSubject<LargeFormData>(initialFormData);

// 保存結果のストリーム
const saveResult$ = new Subject<{ success: boolean; message: string }>();

// デモ用のステータス表示要素
const statusDiv = document.createElement('div');
statusDiv.style.padding = '10px';
statusDiv.style.margin = '10px';
statusDiv.style.border = '2px solid #ccc';
statusDiv.style.borderRadius = '4px';
statusDiv.style.fontFamily = 'monospace';
statusDiv.style.fontSize = '14px';
document.body.appendChild(statusDiv);

function updateStatus(message: string, color: string = '#333') {
  statusDiv.innerHTML = `<span style="color: ${color}">${message}</span>`;
}

// 自動保存パイプライン
formData$.pipe(
  pairwise(),
  map(([previous, current]) => ({
    patches: compare(previous, current),
    timestamp: Date.now()
  })),
  filter(({ patches }) => patches.length > 0), // 変更がない場合はスキップ
  bufferTime(2000), // 2秒間の変更をバッファリング
  filter(buffer => buffer.length > 0), // 空のバッファはスキップ
  map(buffer => {
    // バッファ内の全パッチを1つの配列に統合
    const allPatches = buffer.flatMap(item => item.patches);
    updateStatus(`📦 ${allPatches.length}個の変更をバッチ処理中...`, '#FF9800');
    return allPatches;
  }),
  concatMap(patches => saveToServer(patches)), // 順序を保証して送信
  catchError(error => {
    console.error('自動保存エラー:', error);
    updateStatus(`❌ 保存失敗: ${error.message}`, '#f44336');
    return of({ success: false, message: error.message });
  })
).subscribe(result => {
  if (result.success) {
    updateStatus(`✅ 自動保存完了 (${new Date().toLocaleTimeString()})`, '#4CAF50');
  }
  saveResult$.next(result);
});

// サーバーへの保存（モック実装）
function saveToServer(patches: Operation[]): Promise<{ success: boolean; message: string }> {
  console.log('サーバーに送信:', patches);

  // 実際の実装例:
  // return fetch('/api/forms/12345/patches', {
  //   method: 'PATCH',
  //   headers: { 'Content-Type': 'application/json-patch+json' },
  //   body: JSON.stringify(patches)
  // }).then(res => res.json());

  // モック: 500ms後に成功を返す
  return new Promise(resolve => {
    setTimeout(() => {
      resolve({
        success: true,
        message: `${patches.length}個の変更を保存しました`
      });
    }, 500);
  });
}

// デモ: フォーム変更をシミュレート
const demoButton = document.createElement('button');
demoButton.textContent = 'フォームを変更（デモ）';
demoButton.style.padding = '10px 20px';
demoButton.style.margin = '10px';
demoButton.style.fontSize = '16px';
demoButton.style.cursor = 'pointer';
document.body.appendChild(demoButton);

demoButton.addEventListener('click', () => {
  // ランダムにフィールドを変更
  const currentData = formData$.getValue();
  const updatedData = {
    ...currentData,
    personalInfo: {
      ...currentData.personalInfo,
      firstName: `太郎_${Math.floor(Math.random() * 100)}`,
      email: `taro${Math.floor(Math.random() * 100)}@example.com`
    },
    preferences: {
      ...currentData.preferences,
      newsletter: !currentData.preferences.newsletter
    }
  };
  formData$.next(updatedData);
  updateStatus('📝 フォームを変更しました...', '#2196F3');
});
```

> [!NOTE] 自動保存のポイント
> - **`bufferTime(2000)`** - 2秒間の変更をまとめて送信（ネットワーク効率化）
> - **`concatMap`** - パッチの順序を保証（`mergeMap`は順序が崩れる可能性あり）
> - **`filter`** - 変更がない場合はスキップ（無駄なリクエスト削減）
> - **冪等性** - 同じパッチを複数回送信しても安全（Request IDを付与）

### パターン2: Undo/Redo実装

逆パッチを使って、Undo/Redo機能を実装します。

```typescript
import { Subject, scan } from 'rxjs';
import { applyPatch, Operation, deepClone } from 'fast-json-patch';

interface HistoryState<T> {
  current: T;
  undoStack: Operation[][];
  redoStack: Operation[][];
}

interface HistoryAction {
  type: 'APPLY_PATCH' | 'UNDO' | 'REDO';
  patches?: Operation[];
}

// 履歴管理のストリーム
const historyAction$ = new Subject<HistoryAction>();

const initialState: HistoryState<LargeFormData> = {
  current: initialFormData,
  undoStack: [],
  redoStack: []
};

// 履歴を管理するReducer
const history$ = historyAction$.pipe(
  scan((state, action) => {
    switch (action.type) {
      case 'APPLY_PATCH':
        if (!action.patches || action.patches.length === 0) return state;

        // パッチを適用
        const cloned = deepClone(state.current);
        const result = applyPatch(cloned, action.patches, true, false);

        return {
          current: result.newDocument,
          undoStack: [...state.undoStack, action.patches],
          redoStack: [] // 新しい操作でRedoスタックをクリア
        };

      case 'UNDO':
        if (state.undoStack.length === 0) return state;

        const patchesToUndo = state.undoStack[state.undoStack.length - 1];
        const beforeUndo = deepClone(state.current);

        // 逆パッチを生成して適用
        const inversePatch = generateInversePatch(state.current, patchesToUndo);
        const undoResult = applyPatch(beforeUndo, inversePatch, true, false);

        return {
          current: undoResult.newDocument,
          undoStack: state.undoStack.slice(0, -1),
          redoStack: [...state.redoStack, patchesToUndo]
        };

      case 'REDO':
        if (state.redoStack.length === 0) return state;

        const patchesToRedo = state.redoStack[state.redoStack.length - 1];
        const beforeRedo = deepClone(state.current);
        const redoResult = applyPatch(beforeRedo, patchesToRedo, true, false);

        return {
          current: redoResult.newDocument,
          undoStack: [...state.undoStack, patchesToRedo],
          redoStack: state.redoStack.slice(0, -1)
        };

      default:
        return state;
    }
  }, initialState)
);

// 逆パッチ生成（簡易実装）
function generateInversePatch(document: any, patches: Operation[]): Operation[] {
  // fast-json-patch の applyPatch は第4引数をtrueにすると逆パッチを返す
  const cloned = deepClone(document);
  const result = applyPatch(cloned, patches, true, true);
  return result[1] || []; // 逆パッチを取得
}

// UI要素
const historyControlDiv = document.createElement('div');
historyControlDiv.style.padding = '10px';
historyControlDiv.style.margin = '10px';
document.body.appendChild(historyControlDiv);

const undoButton = document.createElement('button');
undoButton.textContent = '↶ Undo';
undoButton.style.padding = '10px 20px';
undoButton.style.marginRight = '10px';
undoButton.style.fontSize = '16px';
undoButton.style.cursor = 'pointer';
historyControlDiv.appendChild(undoButton);

const redoButton = document.createElement('button');
redoButton.textContent = '↷ Redo';
redoButton.style.padding = '10px 20px';
redoButton.style.fontSize = '16px';
redoButton.style.cursor = 'pointer';
historyControlDiv.appendChild(redoButton);

const historyInfo = document.createElement('div');
historyInfo.style.marginTop = '10px';
historyInfo.style.fontFamily = 'monospace';
historyInfo.style.fontSize = '14px';
historyControlDiv.appendChild(historyInfo);

// 履歴の状態を表示
history$.subscribe(state => {
  undoButton.disabled = state.undoStack.length === 0;
  redoButton.disabled = state.redoStack.length === 0;

  historyInfo.innerHTML = `
    📚 Undo可能: ${state.undoStack.length}回<br>
    📚 Redo可能: ${state.redoStack.length}回<br>
    📝 現在の値: ${JSON.stringify(state.current.personalInfo.firstName)}
  `;

  // フォームデータを同期
  formData$.next(state.current);
});

// ボタンイベント
undoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'UNDO' });
});

redoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'REDO' });
});

// デモ: パッチ適用ボタン
const applyPatchButton = document.createElement('button');
applyPatchButton.textContent = '変更を適用（Undo/Redoテスト）';
applyPatchButton.style.padding = '10px 20px';
applyPatchButton.style.margin = '10px';
applyPatchButton.style.fontSize = '16px';
applyPatchButton.style.cursor = 'pointer';
document.body.appendChild(applyPatchButton);

applyPatchButton.addEventListener('click', () => {
  const patches: Operation[] = [
    { op: 'replace', path: '/personalInfo/firstName', value: `太郎_${Date.now()}` }
  ];
  historyAction$.next({ type: 'APPLY_PATCH', patches });
});
```

> [!TIP] Undo/Redoのポイント
> - **逆パッチ** - `applyPatch` の第4引数を `true` にすると逆パッチが取得できる
> - **スタック管理** - Undoスタック（過去の操作）とRedoスタック（取り消した操作）
> - **新しい操作でRedoクリア** - 新規変更時はRedoスタックをリセット
> - **`scan` でReducerパターン** - 状態管理をReactのuseReducerのように実装

### パターン3: オフライン対応（IndexedDBキュー）

オフライン時は変更をIndexedDBにキューイングし、オンライン復帰時に同期します。

```typescript
import { fromEvent, merge, map, filter, concatMap, catchError, of } from 'rxjs';

// オンライン/オフライン状態
const online$ = merge(
  fromEvent(window, 'online').pipe(map(() => true)),
  fromEvent(window, 'offline').pipe(map(() => false))
);

// IndexedDB操作（簡易実装）
class PatchQueue {
  private dbName = 'form-patches';
  private storeName = 'patches';
  private db: IDBDatabase | null = null;

  async init(): Promise<void> {
    return new Promise((resolve, reject) => {
      const request = indexedDB.open(this.dbName, 1);

      request.onerror = () => reject(request.error);
      request.onsuccess = () => {
        this.db = request.result;
        resolve();
      };

      request.onupgradeneeded = (event) => {
        const db = (event.target as IDBOpenDBRequest).result;
        if (!db.objectStoreNames.contains(this.storeName)) {
          db.createObjectStore(this.storeName, {
            keyPath: 'id',
            autoIncrement: true
          });
        }
      };
    });
  }

  async enqueue(patches: Operation[]): Promise<void> {
    if (!this.db) throw new Error('DB not initialized');

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction([this.storeName], 'readwrite');
      const store = transaction.objectStore(this.storeName);
      const request = store.add({
        patches,
        timestamp: Date.now()
      });

      request.onsuccess = () => resolve();
      request.onerror = () => reject(request.error);
    });
  }

  async dequeueAll(): Promise<Operation[][]> {
    if (!this.db) throw new Error('DB not initialized');

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction([this.storeName], 'readwrite');
      const store = transaction.objectStore(this.storeName);
      const request = store.getAll();

      request.onsuccess = () => {
        const items = request.result;
        // 取得後にクリア
        store.clear();
        resolve(items.map((item: any) => item.patches));
      };
      request.onerror = () => reject(request.error);
    });
  }
}

const patchQueue = new PatchQueue();
patchQueue.init().then(() => {
  console.log('IndexedDB initialized');
});

// オフライン対応の自動保存
formData$.pipe(
  pairwise(),
  map(([previous, current]) => compare(previous, current)),
  filter(patches => patches.length > 0),
  bufferTime(2000),
  filter(buffer => buffer.length > 0),
  map(buffer => buffer.flatMap(patches => patches)),
  concatMap(async (patches) => {
    const isOnline = navigator.onLine;

    if (isOnline) {
      // オンライン: サーバーに送信
      try {
        return await saveToServer(patches);
      } catch (error) {
        // 送信失敗: キューに追加
        await patchQueue.enqueue(patches);
        return { success: false, message: 'オフラインキューに追加しました' };
      }
    } else {
      // オフライン: キューに追加
      await patchQueue.enqueue(patches);
      console.log('📴 オフライン: キューに追加しました');
      return { success: false, message: 'オフライン' };
    }
  })
).subscribe();

// オンライン復帰時に同期
online$.pipe(
  filter(isOnline => isOnline),
  concatMap(async () => {
    console.log('🌐 オンライン復帰: キューを同期中...');
    const queuedPatches = await patchQueue.dequeueAll();

    for (const patches of queuedPatches) {
      await saveToServer(patches);
    }

    return { synced: queuedPatches.length };
  })
).subscribe(result => {
  console.log(`✅ ${result.synced}件のパッチを同期しました`);
});
```

> [!NOTE] オフライン対応のポイント
> - **IndexedDB** - ブラウザ側の永続ストレージ（LocalStorageより容量大）
> - **キューイング** - オフライン時は送信キューに蓄積
> - **オンライン復帰検知** - `window.addEventListener('online')` でイベント監視
> - **順序保証** - `concatMap` でキュー内のパッチを順番に送信

---

## 共同編集のリアルタイム同期

複数ユーザーが同時に同じフォームを編集する「リアルタイム共同編集」を実装します（Google Docs、Notion、Figmaのような機能）。

### Operational Transform (OT) と CRDT の基礎

リアルタイム共同編集では、**競合解決**が最大の課題です。2つの主要なアプローチがあります。

#### Operational Transform (OT)

変更操作を変換して競合を解決します。

```
ユーザーA: "hello" → "hello world" (末尾に " world" を追加)
ユーザーB: "hello" → "Hi hello"   (先頭に "Hi " を追加)

【変換なし】
結果: "Hi hello world" または "hello world" のいずれか（後勝ち）

【OT適用】
- ユーザーAの操作をユーザーBの操作で変換
- ユーザーBの操作をユーザーAの操作で変換
→ 結果: "Hi hello world"（両方の変更を保持）
```

**メリット:**
- 直感的な結果（両方の変更を保持）
- サーバーが最終的な状態を決定

**デメリット:**
- 実装が複雑
- サーバー必須

#### CRDT (Conflict-free Replicated Data Types)

数学的に競合が発生しないデータ構造を使います。

```
各文字に一意のIDを付与:

ユーザーA: [h1, e2, l3, l4, o5] → [h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
ユーザーB: [h1, e2, l3, l4, o5] → [H12, i13, space14, h1, e2, l3, l4, o5]

マージ時にIDでソート:
→ [H12, i13, space14, h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
→ "Hi hello world"
```

**メリット:**
- サーバー不要（P2P可能）
- オフライン対応が容易

**デメリット:**
- メモリ使用量が多い
- 削除処理が複雑（Tombstone方式）

> [!TIP] ライブラリ選択
> - **OT実装**: [ShareDB](https://github.com/share/sharedb) - Operational Transform
> - **CRDT実装**: [Yjs](https://github.com/yjs/yjs) - 高性能CRDT（推奨）
> - **CRDT実装**: [Automerge](https://github.com/automerge/automerge) - JSON特化

この記事では、**Yjs（CRDT）** を使った実装例を紹介します。

### パターン4: Yjsを使った共同編集

YjsとRxJSを組み合わせて、リアルタイム共同編集を実装します。

```typescript
import { fromEvent, merge, Subject } from 'rxjs';
import * as Y from 'yjs';
import { WebsocketProvider } from 'y-websocket';

// Yjsドキュメント（共有状態）
const ydoc = new Y.Doc();

// 共有するフォームデータ（Yjs Map型）
const yFormData = ydoc.getMap('formData');

// WebSocketプロバイダー（サーバー接続）
// 本番環境では独自のWebSocketサーバーを構築
const wsProvider = new WebsocketProvider(
  'wss://demos.yjs.dev', // デモ用の公開サーバー
  'rxjs-form-demo',      // ルーム名
  ydoc
);

// RxJS Subject for form changes
const formChange$ = new Subject<{
  key: string;
  value: any;
  user: string;
}>();

// Yjsの変更をRxJSストリームに変換
yFormData.observe(event => {
  event.changes.keys.forEach((change, key) => {
    if (change.action === 'add' || change.action === 'update') {
      formChange$.next({
        key: key,
        value: yFormData.get(key),
        user: 'remote'
      });
    }
  });
});

// デモUI
const collaborativeFormDiv = document.createElement('div');
collaborativeFormDiv.style.padding = '20px';
collaborativeFormDiv.style.margin = '10px';
collaborativeFormDiv.style.border = '2px solid #2196F3';
collaborativeFormDiv.style.borderRadius = '8px';
collaborativeFormDiv.style.backgroundColor = '#f5f5f5';
document.body.appendChild(collaborativeFormDiv);

const title = document.createElement('h3');
title.textContent = '🤝 リアルタイム共同編集デモ';
title.style.margin = '0 0 15px 0';
collaborativeFormDiv.appendChild(title);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.placeholder = '名前を入力（他のユーザーと同期されます）';
nameInput.style.padding = '10px';
nameInput.style.width = '100%';
nameInput.style.fontSize = '16px';
nameInput.style.border = '2px solid #ccc';
nameInput.style.borderRadius = '4px';
nameInput.style.boxSizing = 'border-box';
collaborativeFormDiv.appendChild(nameInput);

const syncStatus = document.createElement('div');
syncStatus.style.marginTop = '10px';
syncStatus.style.fontSize = '14px';
syncStatus.style.color = '#666';
collaborativeFormDiv.appendChild(syncStatus);

const activeUsers = document.createElement('div');
activeUsers.style.marginTop = '10px';
activeUsers.style.fontSize = '14px';
activeUsers.style.color = '#666';
collaborativeFormDiv.appendChild(activeUsers);

// 接続状態の監視
wsProvider.on('status', (event: { status: string }) => {
  if (event.status === 'connected') {
    syncStatus.innerHTML = '🟢 <strong>接続中</strong> - 他のユーザーとリアルタイム同期';
    syncStatus.style.color = '#4CAF50';
  } else {
    syncStatus.innerHTML = '🔴 <strong>切断</strong>';
    syncStatus.style.color = '#f44336';
  }
});

// 接続ユーザー数の表示（Awareness API）
wsProvider.awareness.on('change', () => {
  const users = Array.from(wsProvider.awareness.getStates().keys());
  activeUsers.innerHTML = `👥 アクティブユーザー: ${users.length}人`;
});

// ローカル変更をYjsに反映
let isRemoteChange = false;

fromEvent(nameInput, 'input').subscribe(() => {
  if (!isRemoteChange) {
    yFormData.set('name', nameInput.value);
  }
});

// リモート変更をUIに反映
formChange$.subscribe(change => {
  if (change.key === 'name') {
    isRemoteChange = true;
    nameInput.value = change.value || '';
    nameInput.style.borderColor = '#4CAF50';

    setTimeout(() => {
      nameInput.style.borderColor = '#ccc';
      isRemoteChange = false;
    }, 500);
  }
});

// 初期値の設定
const initialName = yFormData.get('name');
if (initialName) {
  nameInput.value = initialName;
}
```

> [!NOTE] Yjsのポイント
> - **Y.Doc** - 共有ドキュメント（CRDT）
> - **Y.Map** - 共有Map型（`{ key: value }`）
> - **WebsocketProvider** - WebSocket経由で同期
> - **Awareness API** - プレゼンス情報（接続ユーザー、カーソル位置）
> - **observe** - 変更を監視してRxJSストリームに変換

### パターン5: プレゼンス管理（カーソル位置の共有）

誰がどこを編集しているかを可視化します。

```typescript
import { throttleTime } from 'rxjs';

interface UserPresence {
  userId: string;
  name: string;
  color: string;
  cursorPosition: number;
  timestamp: number;
}

// ランダムな色を生成
function generateRandomColor(): string {
  const colors = ['#f44336', '#E91E63', '#9C27B0', '#673AB7', '#3F51B5', '#2196F3', '#00BCD4', '#009688'];
  return colors[Math.floor(Math.random() * colors.length)];
}

// 自分のユーザーID（ランダム生成）
const myUserId = `user_${Math.random().toString(36).substr(2, 9)}`;
const myColor = generateRandomColor();

// Awareness（プレゼンス情報）の設定
wsProvider.awareness.setLocalState({
  userId: myUserId,
  name: `ユーザー${myUserId.slice(-4)}`,
  color: myColor
});

// カーソル位置の変更を検出（throttleで送信頻度を制限）
fromEvent(nameInput, 'selectionchange').pipe(
  throttleTime(200)
).subscribe(() => {
  const cursorPosition = nameInput.selectionStart || 0;

  wsProvider.awareness.setLocalStateField('cursorPosition', cursorPosition);
});

// 他のユーザーのカーソル位置を表示
const cursorOverlay = document.createElement('div');
cursorOverlay.style.position = 'relative';
cursorOverlay.style.marginTop = '10px';
cursorOverlay.style.fontSize = '12px';
collaborativeFormDiv.appendChild(cursorOverlay);

wsProvider.awareness.on('change', () => {
  const states = wsProvider.awareness.getStates();
  const cursors: string[] = [];

  states.forEach((state: any, clientId: number) => {
    if (state.userId !== myUserId) {
      cursors.push(
        `<span style="color: ${state.color}">● ${state.name}</span> (位置: ${state.cursorPosition || 0})`
      );
    }
  });

  cursorOverlay.innerHTML = cursors.length > 0
    ? `📍 他のユーザー: ${cursors.join(', ')}`
    : '📍 他のユーザーなし';
});
```

> [!TIP] プレゼンス管理のポイント
> - **`awareness.setLocalState`** - 自分の情報を共有
> - **`awareness.getStates`** - 全ユーザーの情報を取得
> - **`throttleTime`** - カーソル移動の送信頻度を制限（200ms）
> - **色分け** - ユーザーごとに色を割り当てて視認性向上

### パターン6: エラーハンドリングと再接続

WebSocket切断時の再接続とエラーハンドリングを実装します。

```typescript
import { timer, takeUntil, Subject } from 'rxjs';

const disconnect$ = new Subject<void>();

// WebSocket切断検知
wsProvider.on('connection-close', () => {
  console.warn('⚠️ WebSocket切断');
  syncStatus.innerHTML = '🟡 <strong>再接続中...</strong>';
  syncStatus.style.color = '#FF9800';

  // 5秒後に再接続を試みる
  timer(5000).pipe(
    takeUntil(disconnect$)
  ).subscribe(() => {
    console.log('🔄 再接続を試みます');
    wsProvider.connect();
  });
});

// WebSocketエラー処理
wsProvider.on('connection-error', (error: Error) => {
  console.error('❌ WebSocketエラー:', error);
  syncStatus.innerHTML = `❌ <strong>エラー:</strong> ${error.message}`;
  syncStatus.style.color = '#f44336';
});

// クリーンアップ
window.addEventListener('beforeunload', () => {
  disconnect$.next();
  wsProvider.disconnect();
  ydoc.destroy();
});
```

> [!WARNING] 本番環境での注意点
> - **独自WebSocketサーバー** - `wss://demos.yjs.dev` は開発用。本番では [y-websocket-server](https://github.com/yjs/y-websocket) を構築
> - **認証** - WebSocket接続時にトークン認証を実装
> - **スケーリング** - Redis等でWebSocketサーバー間の状態を共有
> - **永続化** - Yjsドキュメントをデータベースに保存（`y-leveldb`, `y-indexeddb`）

---

## まとめ

この記事では、JSON Patchを使った高度なフォームパターンを解説しました。

### 重要なポイント

> [!IMPORTANT] JSON Patch パターンの選択基準
>
> **通常のフォーム処理で十分な場合:**
> - フィールド数: ～20個
> - 自動保存: 不要 or 全体送信でOK
> - Undo/Redo: 不要
> - 共同編集: 不要
> → [通常のフォーム処理パターン](./form-handling.md) を使用
>
> **JSON Patchが必要な場合:**
> - フィールド数: 100個以上
> - 自動保存: 必須（差分のみ送信）
> - Undo/Redo: 必須
> - 共同編集: リアルタイム同期が必要
> → この記事のパターンを使用

### 実装パターンまとめ

| パターン | ユースケース | 主な技術 |
|---------|------------|---------|
| **基本的な自動保存** | 大規模フォームの差分送信 | `pairwise` + `bufferTime` + `concatMap` |
| **Undo/Redo** | 操作履歴の管理 | 逆パッチ + `scan` |
| **オフライン対応** | ネットワーク断時の対応 | IndexedDB + キューイング |
| **共同編集（Yjs）** | リアルタイム同期 | Yjs (CRDT) + WebSocket |
| **プレゼンス管理** | カーソル位置の共有 | Awareness API + `throttleTime` |

### 次のステップ

- **[リアルタイムデータ処理](./real-time-data.md)** - WebSocketの詳細な実装
- **[エラーハンドリング実践](./error-handling-patterns.md)** - API通信のエラー処理
- **[キャッシュ戦略](./caching-strategies.md)** - データのキャッシュ管理

## 参考リソース

### 標準仕様

- [RFC 6902: JSON Patch](https://datatracker.ietf.org/doc/html/rfc6902) - JSON Patch仕様
- [RFC 6901: JSON Pointer](https://datatracker.ietf.org/doc/html/rfc6901) - JSON Pointer仕様

### ライブラリ

- [fast-json-patch](https://github.com/Starcounter-Jack/JSON-Patch) - JSON Patch実装（RFC準拠）
- [Yjs](https://docs.yjs.dev/) - CRDT実装（共同編集）
- [ShareDB](https://share.github.io/sharedb/) - Operational Transform実装
- [Automerge](https://automerge.org/) - JSON特化CRDT

### 学習リソース

- [CRDTs: The Hard Parts](https://www.youtube.com/watch?v=x7drE24geUw) - CRDTの深い理解（動画）
- [Operational Transformation Explained](https://operational-transformation.github.io/) - OTの詳細解説
- [Real-time Collaborative Editing](https://pierrehedkvist.com/posts/1-creating-a-collaborative-editor) - 共同編集の実装ガイド
