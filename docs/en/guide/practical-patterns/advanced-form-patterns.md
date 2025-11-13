---
description: Advanced form patterns using JSON Patch. Describes enterprise-level form implementation patterns, including auto-save and Undo/Redo for large forms, real-time synchronization for collaborative editing, etc.
---

# Advanced Form Patterns with JSON Patch

When implementing large forms and real-time collaborative editing, the traditional "submit the entire form" approach creates performance and user experience challenges.

This article describes an advanced form pattern using **JSON Patch (RFC 6902)**. By submitting only the differences, network bandwidth can be reduced and Undo/Redo and collaborative editing can be implemented more efficiently.

## What you will learn in this article

- Basics of JSON Patch/Pointer (RFC 6902/6901)
- Auto-save large forms (diff-based)
- Undo/Redo implementation (reverse patch)
- Real-time synchronization for collaborative editing
- Operational Transform (OT) / CRDT fundamentals
- WebSocket and RxJS integration patterns
- Conflict Resolution and Version Control

> [!TIP] Prerequisites
> This article is based on [Chapter 4: Operators](../operators/index.md), [Form Handling Patterns](./form-handling.md), [Real-time Data Processing](./real-time-data.md).

> [!NOTE] When this pattern is needed
> - **Large forms** (>100 fields) need auto-save
> - **Undo/Redo** functionality required
> - **Real-time collaborative editing** (like Google Docs)
> - **Offline support** with diff queuing required
>
> For small forms (~20 fields), [normal form-handling pattern](./form-handling.md) is sufficient.

## Basics of JSON Patch/Pointer

### What is JSON Patch?

**JSON Patch (RFC 6902)** is a standard format for representing changes in JSON documents. You can submit **just the changes**, not the entire form.

```typescript
// Form data before change
const before = {
  profile: {
    name: "田中太郎",
    email: "tanaka@example.com",
    age: 30
  }
};

// Form data after change
const after = {
  profile: {
    name: "田中太郎",
    email: "tanaka.updated@example.com", // changed
    age: 31 // changed
  }
};

// JSON Patch (diff only)
const patch = [
  { op: "replace", path: "/profile/email", value: "tanaka.updated@example.com" },
  { op: "replace", path: "/profile/age", value: 31 }
];
```

> [!NOTE] Six operations of JSON Patch
> - `add` - add a value
> - `remove` - remove a value
> - `replace` - replace a value
> - `move` - move a value
> - `copy` - copy a value
> - `test` - test a value (validation)

### What is JSON Pointer?

**JSON Pointer (RFC 6901)** is a path notation that points to a specific value in a JSON document.

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

### Difference detection in RxJS

Combines `pairwise()` and `fast-json-patch` libraries to automatically detect changes in forms.

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

// Detect differences
const patches$ = formData$.pipe(
  pairwise(), // Get pair of [previous value, current value]
  map(([previous, current]) => compare(previous, current))
);

patches$.subscribe(patches => {
  console.log('Detected changes:', patches);
  // Example: [{ op: "replace", path: "/profile/name", value: "田中太郎" }]
});

// Simulate form update
formData$.next({
  profile: {
    name: "田中太郎",
    email: "tanaka@example.com",
    age: 30
  }
});
```

> [!TIP] fast-json-patch library
> ```bash
> npm install fast-json-patch
> ```
> - Fully RFC 6902 compliant
> - Generate (`compare`) and apply (`applyPatch`) differences
> - Reverse patch generation (for Undo)
> - TypeScript support

---

## Autosave and Undo/Redo for large forms

Implement auto-save and Undo/Redo functions on large forms (e.g., member registration and product management screens with 100 fields).

### Implementation Policy

**Front End Responsibilities:**
- Generation and alignment of operations (changes)
- Optimistic UI reflection (immediate application via `scan`)
- Undo/Redo stack management (history via reverse patch)
- Send queue management (order guaranteed with `concatMap`)
- Batching (`bufferTime` + compression)

**Backend responsibilities:**
- Version control (Vector Clock / Timestamp)
- Idempotency guarantee (duplicate detection by Request ID)
- Persistence and audit logging

### Pattern 1: Basic Auto-Save

Detects form changes and sends them to the server in batches at regular intervals.

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
  // ... Assuming 100+ fields
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

// Form data stream
const formData$ = new BehaviorSubject<LargeFormData>(initialFormData);

// Save result stream
const saveResult$ = new Subject<{ success: boolean; message: string }>();

// Demo status display element
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

// Auto-save pipeline
formData$.pipe(
  pairwise(),
  map(([previous, current]) => ({
    patches: compare(previous, current),
    timestamp: Date.now()
  })),
  filter(({ patches }) => patches.length > 0), // Skip if no changes
  bufferTime(2000), // Buffer changes for 2 seconds
  filter(buffer => buffer.length > 0), // Skip empty buffers
  map(buffer => {
    // Consolidate all patches in buffer into one array
    const allPatches = buffer.flatMap(item => item.patches);
    updateStatus(`📦 Batch processing ${allPatches.length} changes...`, '#FF9800');
    return allPatches;
  }),
  concatMap(patches => saveToServer(patches)), // Send with guaranteed order
  catchError(error => {
    console.error('Auto-save error:', error);
    updateStatus(`❌ Save failed: ${error.message}`, '#f44336');
    return of({ success: false, message: error.message });
  })
).subscribe(result => {
  if (result.success) {
    updateStatus(`✅ Auto-save completed (${new Date().toLocaleTimeString()})`, '#4CAF50');
  }
  saveResult$.next(result);
});

// Save to server (mock implementation)
function saveToServer(patches: Operation[]): Promise<{ success: boolean; message: string }> {
  console.log('Sending to server:', patches);

  // Actual implementation example:
  // return fetch('/api/forms/12345/patches', {
  //   method: 'PATCH',
  //   headers: { 'Content-Type': 'application/json-patch+json' },
  //   body: JSON.stringify(patches)
  // }).then(res => res.json());

  // Mock: return success after 500ms
  return new Promise(resolve => {
    setTimeout(() => {
      resolve({
        success: true,
        message: `Saved ${patches.length} changes`
      });
    }, 500);
  });
}

// Demo: Simulate form changes
const demoButton = document.createElement('button');
demoButton.textContent = 'Change form (Demo)';
demoButton.style.padding = '10px 20px';
demoButton.style.margin = '10px';
demoButton.style.fontSize = '16px';
demoButton.style.cursor = 'pointer';
document.body.appendChild(demoButton);

demoButton.addEventListener('click', () => {
  // Randomly change fields
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
  updateStatus('📝 Form changed...', '#2196F3');
});
```

> [!NOTE] Points for automatic saving
> - **`bufferTime(2000)`** - send 2 seconds of changes together (network efficiency)
> - **`concatMap`** - guarantees patch order (`mergeMap` may break order)
> - **`filter`** - skip changes if there are none (reduces wasted requests)
> - **Idempotency** - safe to send the same patch multiple times (assign a Request ID)

### Pattern 2: Undo/Redo implementation

Implement Undo/Redo functionality using the reverse patch.

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

// History management stream
const historyAction$ = new Subject<HistoryAction>();

const initialState: HistoryState<LargeFormData> = {
  current: initialFormData,
  undoStack: [],
  redoStack: []
};

// Reducer to manage history
const history$ = historyAction$.pipe(
  scan((state, action) => {
    switch (action.type) {
      case 'APPLY_PATCH':
        if (!action.patches || action.patches.length === 0) return state;

        // Apply patch
        const cloned = deepClone(state.current);
        const result = applyPatch(cloned, action.patches, true, false);

        return {
          current: result.newDocument,
          undoStack: [...state.undoStack, action.patches],
          redoStack: [] // Clear Redo stack on new operation
        };

      case 'UNDO':
        if (state.undoStack.length === 0) return state;

        const patchesToUndo = state.undoStack[state.undoStack.length - 1];
        const beforeUndo = deepClone(state.current);

        // Generate and apply inverse patch
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

// Inverse patch generation (simple implementation)
function generateInversePatch(document: any, patches: Operation[]): Operation[] {
  // applyPatch returns inverse patch when 4th argument is true
  const cloned = deepClone(document);
  const result = applyPatch(cloned, patches, true, true);
  return result[1] || []; // Get inverse patch
}

// UI elements
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

// Display history state
history$.subscribe(state => {
  undoButton.disabled = state.undoStack.length === 0;
  redoButton.disabled = state.redoStack.length === 0;

  historyInfo.innerHTML = `
    📚 Undo available: ${state.undoStack.length} times<br>
    📚 Redo available: ${state.redoStack.length} times<br>
    📝 Current value: ${JSON.stringify(state.current.personalInfo.firstName)}
  `;

  // Sync form data
  formData$.next(state.current);
});

// Button events
undoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'UNDO' });
});

redoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'REDO' });
});

// Demo: Apply patch button
const applyPatchButton = document.createElement('button');
applyPatchButton.textContent = 'Apply change (Undo/Redo test)';
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

> [!TIP] Undo/Redo points
> - **Reverse Patch** - set `applyPatch`'s 4th argument to `true` to get a reverse patch
> - **Stack management** - Undo stack (past operations) and Redo stack (undone operations)
> - **Clear Redo on new operation** - reset Redo stack on new change
> - **Reducer pattern with `scan`** - Implement state management like useReducer in React

### Pattern 3: Offline support (IndexedDB queue)

Queue changes to IndexedDB when offline and synchronize when back online.

```typescript
import { fromEvent, merge, map, filter, concatMap, catchError, of } from 'rxjs';

// Online/offline status
const online$ = merge(
  fromEvent(window, 'online').pipe(map(() => true)),
  fromEvent(window, 'offline').pipe(map(() => false))
);

// IndexedDB operations (simple implementation)
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
        // Clear after retrieval
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

// Auto-save with offline support
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
      // Online: send to server
      try {
        return await saveToServer(patches);
      } catch (error) {
        // Send failed: add to queue
        await patchQueue.enqueue(patches);
        return { success: false, message: 'Added to offline queue' };
      }
    } else {
      // Offline: add to queue
      await patchQueue.enqueue(patches);
      console.log('📴 Offline: Added to queue');
      return { success: false, message: 'Offline' };
    }
  })
).subscribe();

// Sync on online recovery
online$.pipe(
  filter(isOnline => isOnline),
  concatMap(async () => {
    console.log('🌐 Back online: Syncing queue...');
    const queuedPatches = await patchQueue.dequeueAll();

    for (const patches of queuedPatches) {
      await saveToServer(patches);
    }

    return { synced: queuedPatches.length };
  })
).subscribe(result => {
  console.log(`✅ Synced ${result.synced} patches`);
});
```

> [!NOTE] Points for offline support
> - **IndexedDB** - browser-side persistent storage (larger than LocalStorage)
> - **Queueing** - offline, stored in send queue
> - **Online return detection** - `window.addEventListener('online')` to monitor events
> - **Sequence guarantee** - send patches in queue in order with `concatMap`

---

## Real-time synchronization of collaborative editing

Implement "real-time collaborative editing" where multiple users edit the same form at the same time (features like Google Docs, Notion, Figma).

### Operational Transform (OT) and CRDT Basics

In real-time co-editing, **conflict resolution** is the biggest challenge. There are two main approaches.

#### Operational Transform (OT)

Transform change operations to resolve conflicts.

```
User A: "hello" → "hello world" (add " world" at end)
User B: "hello" → "Hi hello"   (add "Hi " at beginning)

【Without transformation】
Result: Either "Hi hello world" or "hello world" (last write wins)

【With OT】
- Transform User A's operation with User B's operation
- Transform User B's operation with User A's operation
→ Result: "Hi hello world" (both changes preserved)
```

**Benefits:**
- Intuitive results (keeps both changes)
- Server determines final status

**Disadvantages:**
- Complex to implement
- Server required

#### CRDT (Conflict-free Replicated Data Types)

Uses mathematically conflict-free data structures.

```
Assign unique ID to each character:

User A: [h1, e2, l3, l4, o5] → [h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
User B: [h1, e2, l3, l4, o5] → [H12, i13, space14, h1, e2, l3, l4, o5]

Sort by ID when merging:
→ [H12, i13, space14, h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
→ "Hi hello world"
```

**Benefits:**
- No server required (P2P capable)
- Easy offline support

**Disadvantages:**
- High memory usage
- Deletion process is complicated (Tombstone method)

> [!TIP] Library selection
> - **OT Implementation**: [ShareDB](https://github.com/share/sharedb) - Operational Transform
> - **CRDT Implementation**: [Yjs](https://github.com/yjs/yjs) - High performance CRDT (recommended)
> - **CRDT Implementation**: [Automerge](https://github.com/automerge/automerge) - JSON specific

This article presents an example implementation using **Yjs (CRDT)**.

### Pattern 4: Collaborative editing using Yjs

Combine Yjs and RxJS to implement real-time collaborative editing.

```typescript
import { fromEvent, merge, Subject } from 'rxjs';
import * as Y from 'yjs';
import { WebsocketProvider } from 'y-websocket';

// Yjs document (shared state)
const ydoc = new Y.Doc();

// Form data to share (Yjs Map type)
const yFormData = ydoc.getMap('formData');

// WebSocket provider (server connection)
// Build your own WebSocket server for production
const wsProvider = new WebsocketProvider(
  'wss://demos.yjs.dev', // Public demo server
  'rxjs-form-demo',      // Room name
  ydoc
);

// RxJS Subject for form changes
const formChange$ = new Subject<{
  key: string;
  value: any;
  user: string;
}>();

// Convert Yjs changes to RxJS stream
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

// Demo UI
const collaborativeFormDiv = document.createElement('div');
collaborativeFormDiv.style.padding = '20px';
collaborativeFormDiv.style.margin = '10px';
collaborativeFormDiv.style.border = '2px solid #2196F3';
collaborativeFormDiv.style.borderRadius = '8px';
collaborativeFormDiv.style.backgroundColor = '#f5f5f5';
document.body.appendChild(collaborativeFormDiv);

const title = document.createElement('h3');
title.textContent = '🤝 Real-time Collaborative Editing Demo';
title.style.margin = '0 0 15px 0';
collaborativeFormDiv.appendChild(title);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.placeholder = 'Enter name (synced with other users)';
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

// Monitor connection status
wsProvider.on('status', (event: { status: string }) => {
  if (event.status === 'connected') {
    syncStatus.innerHTML = '🟢 <strong>Connected</strong> - Real-time sync with other users';
    syncStatus.style.color = '#4CAF50';
  } else {
    syncStatus.innerHTML = '🔴 <strong>Disconnected</strong>';
    syncStatus.style.color = '#f44336';
  }
});

// Display connected user count (Awareness API)
wsProvider.awareness.on('change', () => {
  const users = Array.from(wsProvider.awareness.getStates().keys());
  activeUsers.innerHTML = `👥 Active users: ${users.length}`;
});

// Reflect local changes to Yjs
let isRemoteChange = false;

fromEvent(nameInput, 'input').subscribe(() => {
  if (!isRemoteChange) {
    yFormData.set('name', nameInput.value);
  }
});

// Reflect remote changes to UI
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

// Set initial value
const initialName = yFormData.get('name');
if (initialName) {
  nameInput.value = initialName;
}
```

> [!NOTE] Yjs points
> - **Y.Doc** - shared document (CRDT)
> - **Y.Map** - Shared Map type (`{ key: value }`)
> - **WebsocketProvider** - Synchronization via WebSocket
> - **Awareness API** - presence information (connected users, cursor position)
> - **observe** - monitor changes and convert to RxJS stream

### Pattern 5: Presence management (sharing cursor position)

Visualize who is editing where.

```typescript
import { throttleTime } from 'rxjs';

interface UserPresence {
  userId: string;
  name: string;
  color: string;
  cursorPosition: number;
  timestamp: number;
}

// Generate random color
function generateRandomColor(): string {
  const colors = ['#f44336', '#E91E63', '#9C27B0', '#673AB7', '#3F51B5', '#2196F3', '#00BCD4', '#009688'];
  return colors[Math.floor(Math.random() * colors.length)];
}

// My user ID (random generation)
const myUserId = `user_${Math.random().toString(36).substr(2, 9)}`;
const myColor = generateRandomColor();

// Set Awareness (presence information)
wsProvider.awareness.setLocalState({
  userId: myUserId,
  name: `User${myUserId.slice(-4)}`,
  color: myColor
});

// Detect cursor position changes (throttle to limit send frequency)
fromEvent(nameInput, 'selectionchange').pipe(
  throttleTime(200)
).subscribe(() => {
  const cursorPosition = nameInput.selectionStart || 0;

  wsProvider.awareness.setLocalStateField('cursorPosition', cursorPosition);
});

// Display other users' cursor positions
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
        `<span style="color: ${state.color}">● ${state.name}</span> (position: ${state.cursorPosition || 0})`
      );
    }
  });

  cursorOverlay.innerHTML = cursors.length > 0
    ? `📍 Other users: ${cursors.join(', ')}`
    : '📍 No other users';
});
```

> [!TIP] Points of Presence Management
> - **`awareness.setLocalState`** - share your information
> - **`awareness.getStates`** - get information about all users
> - **`throttleTime`** - limit how often cursor movement is sent (200ms)
> - **Color coding** - assign a color to each user to improve visibility

### Pattern 6: Error handling and reconnection

Implement reconnection and error handling upon WebSocket disconnection.

```typescript
import { timer, takeUntil, Subject } from 'rxjs';

const disconnect$ = new Subject<void>();

// WebSocket disconnect detection
wsProvider.on('connection-close', () => {
  console.warn('⚠️ WebSocket disconnected');
  syncStatus.innerHTML = '🟡 <strong>Reconnecting...</strong>';
  syncStatus.style.color = '#FF9800';

  // Attempt reconnection after 5 seconds
  timer(5000).pipe(
    takeUntil(disconnect$)
  ).subscribe(() => {
    console.log('🔄 Attempting reconnection');
    wsProvider.connect();
  });
});

// WebSocket error handling
wsProvider.on('connection-error', (error: Error) => {
  console.error('❌ WebSocket error:', error);
  syncStatus.innerHTML = `❌ <strong>Error:</strong> ${error.message}`;
  syncStatus.style.color = '#f44336';
});

// Cleanup
window.addEventListener('beforeunload', () => {
  disconnect$.next();
  wsProvider.disconnect();
  ydoc.destroy();
});
```

> [!WARNING] Notes on the production environment
> - **Your own WebSocket server** - `wss://demos.yjs.dev` is for development. In production, build [y-websocket-server](https://github.com/yjs/y-websocket)
> - **Authentication** - implement token authentication for WebSocket connections
> - **Scaling** - share state between WebSocket servers via Redis, etc.
> - **Persistence** - store Yjs documents in database (`y-leveldb`, `y-indexeddb`)

---

## Summary

This article described an advanced form pattern using JSON Patch.

### Important Points

> [!IMPORTANT] JSON Patch pattern selection criteria
>
> **If normal form processing is sufficient:**
> - Number of fields: ~20
> - Autosave: Not required or whole submission is OK
> - Undo/Redo: Not required
> - Co-editing: Not required
> → [Normal form-handling pattern](./form-handling.md)
>
> **If JSON Patch is required:**
> - Number of fields: 100 or more
> - Auto Save: Required (only send differences)
> - Undo/Redo: required
> - Co-editing: Real-time synchronization required
> → Use the pattern in this article

### Summary of Implementation Patterns

| Pattern | Use Case | Main Technologies |
|---------|----------|-------------------|
| **Basic Auto-Save** | Diff transmission for large forms | `pairwise` + `bufferTime` + `concatMap` |
| **Undo/Redo** | Operation history management | Reverse patch + `scan` |
| **Offline Support** | Handle network disconnection | IndexedDB + Queuing |
| **Collaborative Editing (Yjs)** | Real-time synchronization | Yjs (CRDT) + WebSocket |
| **Presence Management** | Cursor position sharing | Awareness API + `throttleTime` |

### Next Steps

- **[Real-time Data Processing](./real-time-data.md)** - Detailed WebSocket implementation
- **[Error Handling Practices](./error-handling-patterns.md)** - Error handling for API communication
- **[Caching Strategies](./caching-strategies.md)** - Data cache management

## Reference Resources

### Standard Specifications

- [RFC 6902: JSON Patch](https://datatracker.ietf.org/doc/html/rfc6902) - JSON Patch Specification
- [RFC 6901: JSON Pointer](https://datatracker.ietf.org/doc/html/rfc6901) - JSON Pointer Specification

### Libraries

- [fast-json-patch](https://github.com/Starcounter-Jack/JSON-Patch) - JSON Patch implementation (RFC compliant)
- [Yjs](https://docs.yjs.dev/) - CRDT implementation (collaborative editing)
- [ShareDB](https://share.github.io/sharedb/) - Operational Transform implementation
- [Automerge](https://automerge.org/) - JSON-specific CRDT

### Learning Resources

- [CRDTs: The Hard Parts](https://www.youtube.com/watch?v=x7drE24geUw) - Deep understanding of CRDTs (video)
- [Operational Transformation Explained](https://operational-transformation.github.io/) - A detailed explanation of OT
- [Real-time Collaborative Editing](https://pierrehedkvist.com/posts/1-creating-a-collaborative-editor) - Implementation guide to collaborative editing
