---
description: The expand operator is an RxJS operator that creates a new Observable from each value and recursively expands the result. It can be used for tree structure traversal, API pagination, recursive computation, and more.
---

# expand - Recursive Expansion

The `expand` operator performs a recursive transformation that **generates a new Observable from each value and expands the result as well**. It is best suited for operations that expand values one after another, such as traversing a tree structure, API pagination, or recursive computation.

## 🔰 Basic Syntax and Usage

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// Recursive processing that doubles
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Prevent infinite loop
).subscribe(console.log);
// Output: 1, 2, 4, 8, 16
```

**Flow of operation**:
1. Initial value `1` is issued
2. Function `expand` receives `1` and returns `of(2)`
3. `2` is issued and the function `expand` is called again
4. Function `expand` receives `2` and returns `of(4)`
5. This iteration continues...

> [!WARNING]
> `expand` will **INFINITE LOOP** if you don't specify an exit condition. Be sure to set an exit condition such as `take` or conditionally return `EMPTY`.

[🌐 RxJS Official Documentation - `expand`](https://rxjs.dev/api/operators/expand)

## 🔄 Difference from mergeMap

`expand` is similar to `mergeMap`, except that it also **recursively processes** the results of the generated Observable.

```ts
import { of } from 'rxjs';
import { mergeMap, expand, take } from 'rxjs';

const double = (x: number) => of(x * 2);

// mergeMap: Transform only once
of(1).pipe(
  mergeMap(double),
  take(5)
).subscribe(console.log);
// Output: 2
// (Only one value, 2 is not transformed again)

// expand: Recursive transformation
of(1).pipe(
  expand(double),
  take(5)
).subscribe(console.log);
// Output: 1, 2, 4, 8, 16
// (Each result is transformed again)
```

| Operator | Processing | Recursive | Use Case |
|---|---|---|---|
| `mergeMap` | Transform each value only once | ❌ | Normal asynchronous transform |
| `expand` | Recursively transform the result | ✅ | Tree traversal, pagination, recursive computation |

## 💡 Typical Usage Patterns

### 1. Recursive Processing with Termination Conditions

```ts
import { of, EMPTY } from 'rxjs';
import { expand } from 'rxjs';

// Double until less than 10
of(1).pipe(
  expand(x => {
    const next = x * 2;
    return next < 10 ? of(next) : EMPTY;
  })
).subscribe(console.log);
// Output: 1, 2, 4, 8
// (16 is >= 10, so EMPTY is returned and it ends)
```

### 2. Tree Structure Traversal

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

// Traverse the entire tree
of(tree).pipe(
  expand(node =>
    node.children && node.children.length > 0
      ? from(node.children)
      : EMPTY
  )
).subscribe(node => {
  console.log(`ID: ${node.id}, Name: ${node.name}`);
});
// Output:
// ID: 1, Name: Root
// ID: 2, Name: Child 1
// ID: 3, Name: Child 2
// ID: 4, Name: Grandchild 1
// ID: 5, Name: Grandchild 2
// ID: 6, Name: Grandchild 3
```

### 3. API Pagination

```ts
import { of, EMPTY } from 'rxjs';
import { expand, mergeMap } from 'rxjs';

interface PageResponse {
  data: string[];
  nextPage: number | null;
}

function fetchPage(page: number): Promise<PageResponse> {
  // Simulate API request
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

// Get all pages sequentially
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
  console.log(`Page data:`, response.data);
});
```

#### More Practical Pagination Implementation

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

// Create UI elements
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Pagination Implementation Example';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Fetch All Data';
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

// Usage example: fetch user data with mock API
interface User {
  id: number;
  name: string;
  email: string;
}

// Simulate mock API
async function fetchUsers(cursor: string | null): Promise<PaginatedResponse<User>> {
  // Simulate API request (100ms delay)
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

// Fetch all data on button click
button.addEventListener('click', async () => {
  button.disabled = true;
  status.textContent = 'Fetching data...';
  output.textContent = '';

  try {
    const allUsers = await fetchPagedData(fetchUsers);

    status.textContent = `Fetch complete: ${allUsers.length} user records`;
    output.textContent = JSON.stringify(allUsers, null, 2);

    console.log(`Total users: ${allUsers.length}`);
    console.log('User data:', allUsers);
  } catch (error) {
    status.textContent = `Error: ${error}`;
  } finally {
    button.disabled = false;
  }
});
```

## 🧠 Practical Code Example (Display Directory Hierarchy)

This is an example of recursively traversing the directory structure of a file system.

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

// Sample file system structure
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

// Create UI elements
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Directory Hierarchy Display';
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

// Recursively expand directory structure
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
    stats.textContent = `Directories: ${dirCount}, Files: ${fileCount}`;
  }
});
```

## 📋 Type-Safe Usage

An example of a type-safe implementation utilizing generics in TypeScript.

```ts
import { Observable, of, from, EMPTY } from 'rxjs';
import { expand, filter, take, defaultIfEmpty, reduce } from 'rxjs';

interface Node<T> {
  value: T;
  children?: Node<T>[];
}

class TreeTraversal<T> {
  /**
   * Traverse tree structure in breadth-first search
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
   * Search for the first node that matches the condition
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
   * Count all nodes in the tree
   */
  countNodes(root: Node<T>): Observable<number> {
    return this.traverseBFS(root).pipe(
      reduce((count) => count + 1, 0)
    );
  }

  /**
   * Get all nodes with specific values
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

// Usage example
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

// Traverse entire tree
traversal.traverseBFS(tree).subscribe(node => {
  console.log(`Visit: ${node.value}`);
});
// Output: Visit: A, Visit: B, Visit: C, Visit: D, Visit: E, Visit: F

// Search for specific node
traversal.findNode(tree, value => value === 'D').subscribe(node => {
  console.log(`Found node: ${node?.value}`);
});
// Output: Found node: D

// Count nodes
traversal.countNodes(tree).subscribe(count => {
  console.log(`Tree node count: ${count}`);
});
// Output: Tree node count: 6

// Get all nodes matching condition
traversal.findAllNodes(tree, value => value.length === 1).subscribe(nodes => {
  console.log(`Single character nodes: ${nodes.map(n => n.value).join(', ')}`);
});
// Output: Single character nodes: A, B, C, D, E, F
```

## 🎯 Combination with Scheduler

`expand` works synchronously by default, but can be controlled asynchronously using a scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { expand, take } from 'rxjs';

// Synchronous (default)
console.log('Synchronous expand start');
of(1).pipe(
  expand(x => of(x * 2)),
  take(5)
).subscribe(x => console.log('Sync:', x));
console.log('Synchronous expand end');
// Output:
// Synchronous expand start
// Sync: 1
// Sync: 2
// Sync: 4
// Sync: 8
// Sync: 16
// Synchronous expand end

// Asynchronous (using asyncScheduler)
console.log('Asynchronous expand start');
of(1, asyncScheduler).pipe(
  expand(x => of(x * 2, asyncScheduler)),
  take(5)
).subscribe(x => console.log('Async:', x));
console.log('Asynchronous expand end');
// Output:
// Asynchronous expand start
// Asynchronous expand end
// Async: 1
// Async: 2
// Async: 4
// Async: 8
// Async: 16
```

> [!TIP]
> When processing large amounts of data, you can use `asyncScheduler` to keep the UI responsive without blocking the main thread. For more information, please refer to [Scheduler Types and Usage](/en/guide/schedulers/types).

## 🔄 Examples of Recursive Computation

### Fibonacci Sequence

```ts
import { of, EMPTY } from 'rxjs';
import { expand, map, take } from 'rxjs';

interface FibState {
  current: number;
  next: number;
}

// Generate Fibonacci sequence
of({ current: 0, next: 1 } as FibState).pipe(
  expand(state =>
    state.current < 100
      ? of({ current: state.next, next: state.current + state.next })
      : EMPTY
  ),
  map(state => state.current),
  take(10)
).subscribe(n => console.log(n));
// Output: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### Factorial Calculation

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

## ⚠️ Common Mistakes

> [!WARNING]
> The most common mistake with `expand` is **forgetting to set an exit condition, resulting in an infinite loop**.

### Error: No Exit Condition

```ts
import { of } from 'rxjs';
import { expand } from 'rxjs';

// ❌ Bad example: Infinite loop
of(1).pipe(
  expand(x => of(x + 1))
).subscribe(console.log);
// Causes memory leak and browser freeze
```

### Correct: With Exit Condition

```ts
import { of, EMPTY } from 'rxjs';
import { expand, take, takeWhile } from 'rxjs';

// ✅ Good example 1: Limit count with take
of(1).pipe(
  expand(x => of(x + 1)),
  take(10)
).subscribe(console.log);

// ✅ Good example 2: Return EMPTY conditionally
of(1).pipe(
  expand(x => x < 10 ? of(x + 1) : EMPTY)
).subscribe(console.log);

// ✅ Good example 3: Condition limit with takeWhile
of(1).pipe(
  expand(x => of(x + 1)),
  takeWhile(x => x <= 10)
).subscribe(console.log);
```

> [!IMPORTANT]
> In recursive processing, always make the exit condition explicit and prevent infinite loops by returning `take`, `takeWhile`, or `EMPTY` depending on the condition.

## 🎓 Summary

### When Should expand Be Used?
- ✅ If you want to recursively traverse a tree structure or graph
- ✅ When you want to get all data in API pagination
- ✅ If you want to perform recursive calculations (Fibonacci, factorial, etc.)
- ✅ If you want to traverse a directory structure or file system
- ✅ To explore organizational charts and hierarchical data

### When Should You Use mergeMap?
- ✅ When it is sufficient to convert each value only once
- ✅ Normal asynchronous conversions that do not require recursive processing

### Cautions
- ⚠️ **Always set an exit condition** (to prevent infinite loops)
- ⚠️ Be careful about memory consumption (when extracting large amounts of data)
- ⚠️ Because it works synchronously, consider using `asyncScheduler` for large amounts of data
- ⚠️ Because debugging is difficult, it is good to use `tap` to log out intermediate states

## 🚀 Next Steps

- **[mergeMap](/en/guide/operators/transformation/mergeMap)** - Learn normal asynchronous conversion
- **[switchMap](/en/guide/operators/transformation/switchMap)** - Learn the conversion to switch to the latest process
- **[concatMap](/en/guide/operators/transformation/concatMap)** - Learn conversions that are performed sequentially
- **[Scheduler Types and Usage](/en/guide/schedulers/types)** - Learn to combine expand and schedulers
- **[Transformation Operator Practical Examples](/en/guide/operators/transformation/practical-use-cases)** - Learn real use cases
