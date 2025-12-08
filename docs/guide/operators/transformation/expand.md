---
description: "expandオペレーターは、各値から新しいObservableを生成し、その結果を再帰的に展開するRxJSの演算子です。ツリー構造の走査、APIのページネーション、無限スクロール、依存関係の解決など、再帰的なデータ取得パターンをTypeScriptで型安全に実装します。"
---

# expand - 再帰的な展開

`expand` オペレーターは、**各値から新しいObservableを生成し、その結果も同様に展開する**再帰的な変換を行います。ツリー構造の走査やAPIのページネーション、再帰的計算など、値を次々と展開していく処理に最適です。


## 🔰 基本構文と使い方

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// 2倍にしていく再帰的処理
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // 無限ループ防止
).subscribe(console.log);
// 出力: 1, 2, 4, 8, 16
```

**動作の流れ**:
1. 初期値 `1` が発行される
2. `expand` の関数が `1` を受け取り、`of(2)` を返す
3. `2` が発行され、また `expand` の関数が呼ばれる
4. `expand` の関数が `2` を受け取り、`of(4)` を返す
5. この繰り返し...

> [!WARNING]
> `expand` は終了条件を指定しないと**無限ループ**になります。必ず `take` や条件付きで `EMPTY` を返すなどの終了条件を設定してください。

[🌐 RxJS公式ドキュメント - `expand`](https://rxjs.dev/api/operators/expand)


## 🔄 mergeMap との違い

`expand` は `mergeMap` に似ていますが、**生成されたObservableの結果も再帰的に処理**する点が異なります。

```ts
import { of } from 'rxjs';
import { mergeMap, expand, take } from 'rxjs';

const double = (x: number) => of(x * 2);

// mergeMap: 1回だけ変換
of(1).pipe(
  mergeMap(double),
  take(5)
).subscribe(console.log);
// 出力: 2
// (1つの値のみ、2は再度変換されない)

// expand: 再帰的に変換
of(1).pipe(
  expand(double),
  take(5)
).subscribe(console.log);
// 出力: 1, 2, 4, 8, 16
// (各結果が再度変換される)
```

| オペレーター | 処理 | 再帰 | ユースケース |
|---|---|---|---|
| `mergeMap` | 各値を1回だけ変換 | ❌ | 通常の非同期変換 |
| `expand` | 結果を再帰的に変換 | ✅ | ツリー走査、ページネーション、再帰的計算 |


## 💡 典型的な活用パターン

### 1. 終了条件付きの再帰処理

```ts
import { of, EMPTY } from 'rxjs';
import { expand } from 'rxjs';

// 10未満まで2倍にする
of(1).pipe(
  expand(x => {
    const next = x * 2;
    return next < 10 ? of(next) : EMPTY;
  })
).subscribe(console.log);
// 出力: 1, 2, 4, 8
// (16は10以上なのでEMPTYが返され、終了)
```

### 2. ツリー構造の走査

```ts
import { of, from, EMPTY } from 'rxjs';
import { expand, mergeMap } from 'rxjs';

interface TreeNode {
  id: number;
  name: string;
  children?: TreeNode[];
}

const tree: TreeNode = {
  id: 1,
  name: 'Root',
  children: [
    {
      id: 2,
      name: 'Child 1',
      children: [
        { id: 4, name: 'Grandchild 1' },
        { id: 5, name: 'Grandchild 2' }
      ]
    },
    {
      id: 3,
      name: 'Child 2',
      children: [
        { id: 6, name: 'Grandchild 3' }
      ]
    }
  ]
};

// ツリー全体を走査
of(tree).pipe(
  expand(node =>
    node.children && node.children.length > 0
      ? from(node.children)
      : EMPTY
  )
).subscribe(node => {
  console.log(`ID: ${node.id}, Name: ${node.name}`);
});
// 出力:
// ID: 1, Name: Root
// ID: 2, Name: Child 1
// ID: 3, Name: Child 2
// ID: 4, Name: Grandchild 1
// ID: 5, Name: Grandchild 2
// ID: 6, Name: Grandchild 3
```

### 3. APIのページネーション

```ts
import { of, EMPTY } from 'rxjs';
import { expand, mergeMap } from 'rxjs';

interface PageResponse {
  data: string[];
  nextPage: number | null;
}

function fetchPage(page: number): Promise<PageResponse> {
  // APIリクエストをシミュレート
  return new Promise(resolve => {
    setTimeout(() => {
      if (page > 3) {
        resolve({ data: [], nextPage: null });
      } else {
        resolve({
          data: [`Item ${page}-1`, `Item ${page}-2`, `Item ${page}-3`],
          nextPage: page + 1
        });
      }
    }, 100);
  });
}

// 全ページを順次取得
of(1).pipe(
  expand(page => {
    return page > 0 ? of(page) : EMPTY;
  }),
  mergeMap(page => fetchPage(page)),
  expand(response =>
    response.nextPage
      ? of(response.nextPage).pipe(
          mergeMap(nextPage => fetchPage(nextPage))
        )
      : EMPTY
  )
).subscribe(response => {
  console.log(`ページのデータ:`, response.data);
});
```

#### より実用的なページネーション実装

```ts
import { defer, EMPTY, lastValueFrom } from 'rxjs';
import { expand, map, reduce, tap } from 'rxjs';

interface PaginatedResponse<T> {
  items: T[];
  nextCursor: string | null;
}

function fetchPagedData<T>(
  fetchFn: (cursor: string | null) => Promise<PaginatedResponse<T>>
): Promise<T[]> {
  return lastValueFrom(
    defer(() => fetchFn(null)).pipe(
      expand(response =>
        response.nextCursor
          ? defer(() => fetchFn(response.nextCursor))
          : EMPTY
      ),
      map(response => response.items),
      reduce((acc, items) => [...acc, ...items], [] as T[])
    )
  );
}

// UI要素の作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'ページネーション実装例';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = '全データを取得';
container.appendChild(button);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#f0f0f0';
container.appendChild(status);

const output = document.createElement('pre');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.backgroundColor = '#f9f9f9';
output.style.maxHeight = '300px';
output.style.overflow = 'auto';
container.appendChild(output);

// 使用例：モックAPIでユーザーデータを取得
interface User {
  id: number;
  name: string;
  email: string;
}

// モックAPIをシミュレート
async function fetchUsers(cursor: string | null): Promise<PaginatedResponse<User>> {
  // APIリクエストをシミュレート（100ms遅延）
  await new Promise(resolve => setTimeout(resolve, 100));

  const page = cursor ? parseInt(cursor) : 1;
  const pageSize = 5;
  const totalPages = 4;

  if (page > totalPages) {
    return { items: [], nextCursor: null };
  }

  const items: User[] = Array.from({ length: pageSize }, (_, i) => ({
    id: (page - 1) * pageSize + i + 1,
    name: `User ${(page - 1) * pageSize + i + 1}`,
    email: `user${(page - 1) * pageSize + i + 1}@example.com`
  }));

  return {
    items,
    nextCursor: page < totalPages ? String(page + 1) : null
  };
}

// ボタンクリックで全データを取得
button.addEventListener('click', async () => {
  button.disabled = true;
  status.textContent = 'データ取得中...';
  output.textContent = '';

  try {
    const allUsers = await fetchPagedData(fetchUsers);

    status.textContent = `取得完了: ${allUsers.length}件のユーザーデータ`;
    output.textContent = JSON.stringify(allUsers, null, 2);

    console.log(`全ユーザー数: ${allUsers.length}`);
    console.log('ユーザーデータ:', allUsers);
  } catch (error) {
    status.textContent = `エラー: ${error}`;
  } finally {
    button.disabled = false;
  }
});
```


## 🧠 実践コード例（ディレクトリ階層の表示）

ファイルシステムのディレクトリ構造を再帰的に走査する例です。

```ts
import { of, from, EMPTY } from 'rxjs';
import { expand, tap } from 'rxjs';

interface FileSystemItem {
  name: string;
  type: 'file' | 'directory';
  path: string;
  children?: FileSystemItem[];
  level: number;
}

// サンプルのファイルシステム構造
const fileSystem: FileSystemItem = {
  name: 'root',
  type: 'directory',
  path: '/root',
  level: 0,
  children: [
    {
      name: 'src',
      type: 'directory',
      path: '/root/src',
      level: 1,
      children: [
        { name: 'index.ts', type: 'file', path: '/root/src/index.ts', level: 2 },
        { name: 'utils.ts', type: 'file', path: '/root/src/utils.ts', level: 2 },
        {
          name: 'components',
          type: 'directory',
          path: '/root/src/components',
          level: 2,
          children: [
            { name: 'Button.tsx', type: 'file', path: '/root/src/components/Button.tsx', level: 3 },
            { name: 'Input.tsx', type: 'file', path: '/root/src/components/Input.tsx', level: 3 }
          ]
        }
      ]
    },
    {
      name: 'docs',
      type: 'directory',
      path: '/root/docs',
      level: 1,
      children: [
        { name: 'README.md', type: 'file', path: '/root/docs/README.md', level: 2 }
      ]
    },
    { name: 'package.json', type: 'file', path: '/root/package.json', level: 1 }
  ]
};

// UI要素の作成
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'ディレクトリ階層表示';
container.appendChild(title);

const output = document.createElement('pre');
output.style.padding = '10px';
output.style.backgroundColor = '#f5f5f5';
output.style.fontFamily = 'monospace';
output.style.fontSize = '14px';
container.appendChild(output);

const stats = document.createElement('div');
stats.style.marginTop = '10px';
stats.style.padding = '10px';
stats.style.backgroundColor = '#e3f2fd';
container.appendChild(stats);

let fileCount = 0;
let dirCount = 0;

// ディレクトリ構造を再帰的に展開
of(fileSystem).pipe(
  expand(item => {
    if (item.type === 'directory' && item.children && item.children.length > 0) {
      return from(
        item.children.map(child => ({
          ...child,
          level: item.level + 1
        }))
      );
    }
    return EMPTY;
  }),
  tap(item => {
    if (item.type === 'file') {
      fileCount++;
    } else {
      dirCount++;
    }
  })
).subscribe({
  next: item => {
    const indent = '  '.repeat(item.level);
    const icon = item.type === 'directory' ? '📁' : '📄';
    output.textContent += `${indent}${icon} ${item.name}\n`;
  },
  complete: () => {
    stats.textContent = `ディレクトリ数: ${dirCount}, ファイル数: ${fileCount}`;
  }
});
```


## 📋 型安全な使い方

TypeScript でジェネリクスを活用した型安全な実装例です。

```ts
import { Observable, of, from, EMPTY } from 'rxjs';
import { expand, filter, take, defaultIfEmpty, reduce } from 'rxjs';

interface Node<T> {
  value: T;
  children?: Node<T>[];
}

class TreeTraversal<T> {
  /**
   * ツリー構造を幅優先探索で走査
   */
  traverseBFS(root: Node<T>): Observable<Node<T>> {
    return of(root).pipe(
      expand(node =>
        node.children && node.children.length > 0
          ? from(node.children)
          : EMPTY
      )
    );
  }

  /**
   * 条件に一致する最初のノードを検索
   */
  findNode(
    root: Node<T>,
    predicate: (value: T) => boolean
  ): Observable<Node<T> | undefined> {
    return this.traverseBFS(root).pipe(
      filter(node => predicate(node.value)),
      take(1),
      defaultIfEmpty(undefined as Node<T> | undefined)
    );
  }

  /**
   * ツリーの全ノード数をカウント
   */
  countNodes(root: Node<T>): Observable<number> {
    return this.traverseBFS(root).pipe(
      reduce((count) => count + 1, 0)
    );
  }

  /**
   * 特定の値を持つノードをすべて取得
   */
  findAllNodes(
    root: Node<T>,
    predicate: (value: T) => boolean
  ): Observable<Node<T>[]> {
    return this.traverseBFS(root).pipe(
      filter(node => predicate(node.value)),
      reduce((acc, node) => [...acc, node], [] as Node<T>[])
    );
  }
}

// 使用例
const tree: Node<string> = {
  value: 'A',
  children: [
    {
      value: 'B',
      children: [
        { value: 'D' },
        { value: 'E' }
      ]
    },
    {
      value: 'C',
      children: [
        { value: 'F' }
      ]
    }
  ]
};

const traversal = new TreeTraversal<string>();

// ツリー全体を走査
traversal.traverseBFS(tree).subscribe(node => {
  console.log(`訪問: ${node.value}`);
});
// 出力: 訪問: A, 訪問: B, 訪問: C, 訪問: D, 訪問: E, 訪問: F

// 特定のノードを検索
traversal.findNode(tree, value => value === 'D').subscribe(node => {
  console.log(`見つかったノード: ${node?.value}`);
});
// 出力: 見つかったノード: D

// ノード数をカウント
traversal.countNodes(tree).subscribe(count => {
  console.log(`ツリーのノード数: ${count}`);
});
// 出力: ツリーのノード数: 6

// 条件に一致するノードをすべて取得
traversal.findAllNodes(tree, value => value.length === 1).subscribe(nodes => {
  console.log(`単一文字のノード: ${nodes.map(n => n.value).join(', ')}`);
});
// 出力: 単一文字のノード: A, B, C, D, E, F
```


## 🎯 スケジューラーとの組み合わせ

`expand` はデフォルトで同期的に動作しますが、スケジューラーを使って非同期制御できます。

```ts
import { of, asyncScheduler } from 'rxjs';
import { expand, take } from 'rxjs';

// 同期的（デフォルト）
console.log('同期的expand開始');
of(1).pipe(
  expand(x => of(x * 2)),
  take(5)
).subscribe(x => console.log('同期:', x));
console.log('同期的expand終了');
// 出力:
// 同期的expand開始
// 同期: 1
// 同期: 2
// 同期: 4
// 同期: 8
// 同期: 16
// 同期的expand終了

// 非同期的（asyncScheduler使用）
console.log('非同期expand開始');
of(1, asyncScheduler).pipe(
  expand(x => of(x * 2, asyncScheduler)),
  take(5)
).subscribe(x => console.log('非同期:', x));
console.log('非同期expand終了');
// 出力:
// 非同期expand開始
// 非同期expand終了
// 非同期: 1
// 非同期: 2
// 非同期: 4
// 非同期: 8
// 非同期: 16
```

> [!TIP]
> 大量のデータを処理する場合は、`asyncScheduler` を使用することでメインスレッドをブロックせず、UIの応答性を保つことができます。詳しくは [スケジューラーの種類と使い分け](/guide/schedulers/types) を参照してください。


## 🔄 再帰的計算の例

### フィボナッチ数列

```ts
import { of, EMPTY } from 'rxjs';
import { expand, map, take } from 'rxjs';

interface FibState {
  current: number;
  next: number;
}

// フィボナッチ数列を生成
of({ current: 0, next: 1 } as FibState).pipe(
  expand(state =>
    state.current < 100
      ? of({ current: state.next, next: state.current + state.next })
      : EMPTY
  ),
  map(state => state.current),
  take(10)
).subscribe(n => console.log(n));
// 出力: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### 階乗計算

```ts
import { of, EMPTY, Observable } from 'rxjs';
import { expand, reduce } from 'rxjs';

interface FactorialState {
  n: number;
  result: number;
}

function factorial(n: number): Observable<number> {
  return of({ n, result: 1 } as FactorialState).pipe(
    expand(state =>
      state.n > 1
        ? of({ n: state.n - 1, result: state.result * state.n })
        : EMPTY
    ),
    reduce((acc, state) => state.result, 1)
  );
}

factorial(5).subscribe(result => {
  console.log('5! =', result); // 5! = 120
});
```


## ⚠️ よくある間違い

> [!WARNING]
> `expand` で最も多い間違いは**終了条件を設定し忘れて無限ループ**になることです。

### 誤: 終了条件なし

```ts
import { of } from 'rxjs';
import { expand } from 'rxjs';

// ❌ 悪い例: 無限ループ
of(1).pipe(
  expand(x => of(x + 1))
).subscribe(console.log);
// メモリリークとブラウザのフリーズを引き起こす
```

### 正: 終了条件あり

```ts
import { of, EMPTY } from 'rxjs';
import { expand, take, takeWhile } from 'rxjs';

// ✅ 良い例1: takeで個数制限
of(1).pipe(
  expand(x => of(x + 1)),
  take(10)
).subscribe(console.log);

// ✅ 良い例2: 条件付きでEMPTYを返す
of(1).pipe(
  expand(x => x < 10 ? of(x + 1) : EMPTY)
).subscribe(console.log);

// ✅ 良い例3: takeWhileで条件制限
of(1).pipe(
  expand(x => of(x + 1)),
  takeWhile(x => x <= 10)
).subscribe(console.log);
```

> [!IMPORTANT]
> 再帰処理では常に終了条件を明確にし、`take`、`takeWhile`、または条件に応じた `EMPTY` の返却で無限ループを防いでください。


## 🎓 まとめ

### expand を使うべき場合
- ✅ ツリー構造やグラフを再帰的に走査したい場合
- ✅ APIのページネーションで全データを取得したい場合
- ✅ 再帰的な計算（フィボナッチ、階乗など）を行いたい場合
- ✅ ディレクトリ構造やファイルシステムを走査したい場合
- ✅ 組織図や階層データを探索したい場合

### mergeMap を使うべき場合
- ✅ 各値を1回だけ変換すれば十分な場合
- ✅ 再帰的な処理が不要な通常の非同期変換

### 注意点
- ⚠️ **必ず終了条件を設定**（無限ループ防止）
- ⚠️ メモリ消費に注意（大量のデータを展開する場合）
- ⚠️ 同期的に動作するため、大量データでは `asyncScheduler` の使用を検討
- ⚠️ デバッグが難しいため、`tap` で中間状態をログ出力すると良い


## 🚀 次のステップ

- **[mergeMap](./mergeMap)** - 通常の非同期変換を学ぶ
- **[switchMap](./switchMap)** - 最新の処理に切り替える変換を学ぶ
- **[concatMap](./concatMap)** - 順次実行する変換を学ぶ
- **[スケジューラーの種類と使い分け](/guide/schedulers/types)** - expand とスケジューラーの組み合わせを学ぶ
- **[変換オペレーター実践例](./practical-use-cases)** - 実際のユースケースを学ぶ
