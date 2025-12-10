---
description: "Pattern avanzati per form con JSON Patch. Come costruire implementazioni di form di livello enterprise con RxJS e TypeScript, inclusi salvataggio automatico e Undo/Redo per form di grandi dimensioni, sincronizzazione in tempo reale dell'editing collaborativo, supporto offline e tracciamento della cronologia delle operazioni."
---

# Pattern avanzati per form con JSON Patch

Quando si implementano form di grandi dimensioni e editing collaborativo in tempo reale, l'approccio tradizionale "invia l'intero form" crea problemi di prestazioni e di esperienza utente.

Questo articolo descrive pattern avanzati per form utilizzando **JSON Patch (RFC 6902)**. Inviando solo le differenze, è possibile ridurre la larghezza di banda della rete e implementare in modo efficiente Undo/Redo e editing collaborativo.

## Cosa imparerai in questo articolo

- Fondamenti di JSON Patch/Pointer (RFC 6902/6901)
- Salvataggio automatico di form di grandi dimensioni (basato sulle differenze)
- Implementazione di Undo/Redo (patch inversa)
- Sincronizzazione in tempo reale dell'editing collaborativo
- Fondamenti di Operational Transform (OT) / CRDT
- Pattern di integrazione WebSocket e RxJS
- Risoluzione dei conflitti e controllo di versione

> [!TIP] Prerequisiti
> Questo articolo si basa sulla conoscenza di [Capitolo 4: Operatori](../operators/index.md), [Pattern di gestione form](./form-handling.md), e [Elaborazione dati in tempo reale](./real-time-data.md).

> [!NOTE] Quando è necessario questo pattern
> - **Form di grandi dimensioni** (>100 campi) che richiedono salvataggio automatico
> - Funzionalità **Undo/Redo** richiesta
> - **Editing collaborativo in tempo reale** (come Google Docs)
> - **Supporto offline** con accodamento delle differenze richiesto
>
> Per form di piccole dimensioni (~20 campi), il [pattern normale di gestione form](./form-handling.md) è sufficiente.

## Fondamenti di JSON Patch/Pointer

### Cos'è JSON Patch?

**JSON Patch (RFC 6902)** è un formato standard per rappresentare le modifiche nei documenti JSON. È possibile inviare solo le **modifiche**, non l'intero form.

```typescript
// Dati del form prima delle modifiche
const before = {
  profile: {
    name: "Taro Tanaka",
    email: "tanaka@example.com",
    age: 30
  }
};

// Dati del form dopo le modifiche
const after = {
  profile: {
    name: "Taro Tanaka",
    email: "tanaka.updated@example.com", // Modificato
    age: 31 // Modificato
  }
};

// JSON Patch (differenza)
const patch = [
  { op: "replace", path: "/profile/email", value: "tanaka.updated@example.com" },
  { op: "replace", path: "/profile/age", value: 31 }
];
```

> [!NOTE] Le 6 operazioni di JSON Patch
> - `add` - Aggiunge un valore
> - `remove` - Rimuove un valore
> - `replace` - Sostituisce un valore
> - `move` - Sposta un valore
> - `copy` - Copia un valore
> - `test` - Testa un valore (validazione)

### Cos'è JSON Pointer?

**JSON Pointer (RFC 6901)** è una notazione di percorso che punta a un valore specifico in un documento JSON.

```typescript
const formData = {
  user: {
    profile: {
      name: "Taro Tanaka"
    },
    settings: {
      notifications: true
    }
  }
};

// Esempi di JSON Pointer
"/user/profile/name"           // → "Taro Tanaka"
"/user/settings/notifications" // → true
"/user/profile"                // → { name: "Taro Tanaka" }
```

### Rilevamento delle differenze in RxJS

Le librerie `pairwise()` e `fast-json-patch` sono combinate per rilevare automaticamente le modifiche nei form.

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

// Rilevare le differenze
const patches$ = formData$.pipe(
  pairwise(), // Ottiene coppie [valore precedente, valore corrente]
  map(([previous, current]) => compare(previous, current))
);

patches$.subscribe(patches => {
  console.log('Modifiche rilevate:', patches);
  // Esempio: [{ op: "replace", path: "/profile/name", value: "Taro Tanaka" }]
});

// Simula aggiornamento del form
formData$.next({
  profile: {
    name: "Taro Tanaka",
    email: "tanaka@example.com",
    age: 30
  }
});
```

> [!TIP] Libreria fast-json-patch
> ```bash
> npm install fast-json-patch
> ```
> - Completamente conforme a RFC 6902
> - Generazione differenze (`compare`) e applicazione (`applyPatch`)
> - Generazione patch inversa (per Undo)
> - Supporto TypeScript

---

## Salvataggio automatico e Undo/Redo per form di grandi dimensioni

Implementare funzionalità di salvataggio automatico e Undo/Redo in form di grandi dimensioni (es. schermate di registrazione membri e gestione prodotti con 100 campi).

### Politica di implementazione

**Responsabilità frontend:**
- Generazione e allineamento delle operazioni (modifiche)
- Riflessione UI ottimistica (applicazione immediata tramite `scan`)
- Gestione stack Undo/Redo (cronologia tramite patch inversa)
- Gestione coda di invio (ordine garantito con `concatMap`)
- Batching (`bufferTime` + compressione)

**Responsabilità backend:**
- Controllo versione (Vector Clock / timestamp)
- Garanzia idempotenza (rilevamento duplicati tramite Request ID)
- Persistenza e log di audit

### Pattern 1: Salvataggio automatico di base

Le modifiche al form vengono rilevate e inviate al server in batch a intervalli regolari.

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
  // ... assumendo oltre 100 campi
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

// Stream dei dati del form
const formData$ = new BehaviorSubject<LargeFormData>(initialFormData);

// Stream dei risultati di salvataggio
const saveResult$ = new Subject<{ success: boolean; message: string }>();

// Elemento di visualizzazione stato per demo
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

// Pipeline di salvataggio automatico
formData$.pipe(
  pairwise(),
  map(([previous, current]) => ({
    patches: compare(previous, current),
    timestamp: Date.now()
  })),
  filter(({ patches }) => patches.length > 0), // Salta se non ci sono modifiche
  bufferTime(2000), // Buffer delle modifiche per 2 secondi
  filter(buffer => buffer.length > 0), // Salta buffer vuoti
  map(buffer => {
    // Unisce tutte le patch nel buffer in un unico array
    const allPatches = buffer.flatMap(item => item.patches);
    updateStatus(`📦 Elaborazione batch di ${allPatches.length} modifiche...`, '#FF9800');
    return allPatches;
  }),
  concatMap(patches => saveToServer(patches)), // Invia con ordine garantito
  catchError(error => {
    console.error('Errore salvataggio automatico:', error);
    updateStatus(`❌ Salvataggio fallito: ${error.message}`, '#f44336');
    return of({ success: false, message: error.message });
  })
).subscribe(result => {
  if (result.success) {
    updateStatus(`✅ Salvataggio automatico completato (${new Date().toLocaleTimeString()})`, '#4CAF50');
  }
  saveResult$.next(result);
});

// Salva sul server (implementazione mock)
function saveToServer(patches: Operation[]): Promise<{ success: boolean; message: string }> {
  console.log('Invio al server:', patches);

  // Esempio di implementazione reale:
  // return fetch('/api/forms/12345/patches', {
  //   method: 'PATCH',
  //   headers: { 'Content-Type': 'application/json-patch+json' },
  //   body: JSON.stringify(patches)
  // }).then(res => res.json());

  // Mock: restituisce successo dopo 500ms
  return new Promise(resolve => {
    setTimeout(() => {
      resolve({
        success: true,
        message: `${patches.length} modifiche salvate`
      });
    }, 500);
  });
}

// Demo: simula modifiche al form
const demoButton = document.createElement('button');
demoButton.textContent = 'Modifica form (Demo)';
demoButton.style.padding = '10px 20px';
demoButton.style.margin = '10px';
demoButton.style.fontSize = '16px';
demoButton.style.cursor = 'pointer';
document.body.appendChild(demoButton);

demoButton.addEventListener('click', () => {
  // Modifica casuale dei campi
  const currentData = formData$.getValue();
  const updatedData = {
    ...currentData,
    personalInfo: {
      ...currentData.personalInfo,
      firstName: `Taro_${Math.floor(Math.random() * 100)}`,
      email: `taro${Math.floor(Math.random() * 100)}@example.com`
    },
    preferences: {
      ...currentData.preferences,
      newsletter: !currentData.preferences.newsletter
    }
  };
  formData$.next(updatedData);
  updateStatus('📝 Form modificato...', '#2196F3');
});
```

> [!NOTE] Punti chiave del salvataggio automatico
> - **`bufferTime(2000)`** - Invia 2 secondi di modifiche insieme (efficienza di rete)
> - **`concatMap`** - Garantisce l'ordine delle patch (`mergeMap` può rompere l'ordine)
> - **`filter`** - Salta se non ci sono modifiche (riduce richieste inutili)
> - **Idempotenza** - Sicuro inviare la stessa patch più volte (assegna Request ID)

### Pattern 2: Implementazione Undo/Redo

Implementare la funzionalità Undo/Redo usando la patch inversa.

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

// Stream di gestione della cronologia
const historyAction$ = new Subject<HistoryAction>();

const initialState: HistoryState<LargeFormData> = {
  current: initialFormData,
  undoStack: [],
  redoStack: []
};

// Reducer per gestire la cronologia
const history$ = historyAction$.pipe(
  scan((state, action) => {
    switch (action.type) {
      case 'APPLY_PATCH':
        if (!action.patches || action.patches.length === 0) return state;

        // Applica la patch
        const cloned = deepClone(state.current);
        const result = applyPatch(cloned, action.patches, true, false);

        return {
          current: result.newDocument,
          undoStack: [...state.undoStack, action.patches],
          redoStack: [] // Cancella lo stack Redo con una nuova operazione
        };

      case 'UNDO':
        if (state.undoStack.length === 0) return state;

        const patchesToUndo = state.undoStack[state.undoStack.length - 1];
        const beforeUndo = deepClone(state.current);

        // Genera e applica la patch inversa
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

// Generazione patch inversa (implementazione semplice)
function generateInversePatch(document: any, patches: Operation[]): Operation[] {
  // applyPatch di fast-json-patch restituisce patch inversa se il 4° argomento è true
  const cloned = deepClone(document);
  const result = applyPatch(cloned, patches, true, true);
  return result[1] || []; // Ottiene la patch inversa
}

// Elementi UI
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

// Mostra lo stato della cronologia
history$.subscribe(state => {
  undoButton.disabled = state.undoStack.length === 0;
  redoButton.disabled = state.redoStack.length === 0;

  historyInfo.innerHTML = `
    📚 Undo disponibili: ${state.undoStack.length}<br>
    📚 Redo disponibili: ${state.redoStack.length}<br>
    📝 Valore attuale: ${JSON.stringify(state.current.personalInfo.firstName)}
  `;

  // Sincronizza i dati del form
  formData$.next(state.current);
});

// Eventi dei pulsanti
undoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'UNDO' });
});

redoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'REDO' });
});

// Demo: pulsante applica patch
const applyPatchButton = document.createElement('button');
applyPatchButton.textContent = 'Applica modifiche (test Undo/Redo)';
applyPatchButton.style.padding = '10px 20px';
applyPatchButton.style.margin = '10px';
applyPatchButton.style.fontSize = '16px';
applyPatchButton.style.cursor = 'pointer';
document.body.appendChild(applyPatchButton);

applyPatchButton.addEventListener('click', () => {
  const patches: Operation[] = [
    { op: 'replace', path: '/personalInfo/firstName', value: `Taro_${Date.now()}` }
  ];
  historyAction$.next({ type: 'APPLY_PATCH', patches });
});
```

> [!TIP] Punti chiave di Undo/Redo
> - **Patch inversa** - Imposta il quarto argomento di `applyPatch` su `true` per ottenere la patch inversa
> - **Gestione stack** - Stack Undo (operazioni passate) e stack Redo (operazioni annullate)
> - **Cancella stack Redo su nuova operazione** - Resetta lo stack Redo su nuova modifica
> - **Pattern Reducer con `scan`** - Implementa la gestione dello stato come useReducer in React

### Pattern 3: Supporto offline (coda IndexedDB)

Quando offline, le modifiche vengono accodate in IndexedDB e sincronizzate al ritorno online.

```typescript
import { fromEvent, merge, map, filter, concatMap, catchError, of } from 'rxjs';

// Stato online/offline
const online$ = merge(
  fromEvent(window, 'online').pipe(map(() => true)),
  fromEvent(window, 'offline').pipe(map(() => false))
);

// Operazioni IndexedDB (implementazione semplice)
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
        // Cancella dopo il recupero
        store.clear();
        resolve(items.map((item: any) => item.patches));
      };
      request.onerror = () => reject(request.error);
    });
  }
}

const patchQueue = new PatchQueue();
patchQueue.init().then(() => {
  console.log('IndexedDB inizializzato');
});

// Salvataggio automatico con supporto offline
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
      // Online: invia al server
      try {
        return await saveToServer(patches);
      } catch (error) {
        // Invio fallito: aggiungi alla coda
        await patchQueue.enqueue(patches);
        return { success: false, message: 'Aggiunto alla coda offline' };
      }
    } else {
      // Offline: aggiungi alla coda
      await patchQueue.enqueue(patches);
      console.log('📴 Offline: aggiunto alla coda');
      return { success: false, message: 'Offline' };
    }
  })
).subscribe();

// Sincronizza al ritorno online
online$.pipe(
  filter(isOnline => isOnline),
  concatMap(async () => {
    console.log('🌐 Ritorno online: sincronizzazione coda...');
    const queuedPatches = await patchQueue.dequeueAll();

    for (const patches of queuedPatches) {
      await saveToServer(patches);
    }

    return { synced: queuedPatches.length };
  })
).subscribe(result => {
  console.log(`✅ ${result.synced} patch sincronizzate`);
});
```

> [!NOTE] Punti chiave del supporto offline
> - **IndexedDB** - Storage persistente lato browser (più grande di LocalStorage)
> - **Accodamento** - Offline, memorizzato nella coda in uscita
> - **Rilevamento ritorno online** - Monitoraggio eventi con `window.addEventListener('online')`
> - **Garanzia ordine** - Invia le patch in coda in ordine con `concatMap`

---

## Sincronizzazione in tempo reale dell'editing collaborativo

Implementare "editing collaborativo in tempo reale", dove più utenti modificano lo stesso form contemporaneamente (funzionalità come Google Docs, Notion, Figma).

### Fondamenti di Operational Transform (OT) e CRDT

Nell'editing collaborativo in tempo reale, la **risoluzione dei conflitti** è la sfida maggiore. Esistono due approcci principali.

#### Operational Transform (OT)

Trasforma le operazioni di modifica per risolvere i conflitti.

```
Utente A: "hello" → "hello world" (aggiunge " world" alla fine)
Utente B: "hello" → "Hi hello"   (aggiunge "Hi " all'inizio)

【Senza trasformazione】
Risultato: "Hi hello world" o "hello world" (vince l'ultimo)

【Con OT applicato】
- Trasforma l'operazione dell'utente A con l'operazione dell'utente B
- Trasforma l'operazione dell'utente B con l'operazione dell'utente A
→ Risultato: "Hi hello world" (entrambe le modifiche conservate)
```

**Vantaggi:**
- Risultato intuitivo (entrambe le modifiche conservate)
- Il server determina lo stato finale

**Svantaggi:**
- Implementazione complessa
- Server richiesto

#### CRDT (Conflict-free Replicated Data Types)

Utilizza strutture dati matematicamente prive di conflitti.

```
Assegna ID univoco a ogni carattere:

Utente A: [h1, e2, l3, l4, o5] → [h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
Utente B: [h1, e2, l3, l4, o5] → [H12, i13, space14, h1, e2, l3, l4, o5]

Ordina per ID durante il merge:
→ [H12, i13, space14, h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
→ "Hi hello world"
```

**Vantaggi:**
- Server non richiesto (compatibile P2P)
- Facile supporto offline

**Svantaggi:**
- Elevato uso di memoria
- Processo di cancellazione complesso (metodo Tombstone)

> [!TIP] Selezione libreria
> - **Implementazione OT**: [ShareDB](https://github.com/share/sharedb) - Operational Transform
> - **Implementazione CRDT**: [Yjs](https://github.com/yjs/yjs) - CRDT ad alte prestazioni (consigliato)
> - **Implementazione CRDT**: [Automerge](https://github.com/automerge/automerge) - Specifico per JSON

Questo articolo presenta un esempio di implementazione usando **Yjs (CRDT)**.

### Pattern 4: Editing collaborativo con Yjs

Combina Yjs e RxJS per implementare editing collaborativo in tempo reale.

```typescript
import { fromEvent, merge, Subject } from 'rxjs';
import * as Y from 'yjs';
import { WebsocketProvider } from 'y-websocket';

// Documento Yjs (stato condiviso)
const ydoc = new Y.Doc();

// Dati form condivisi (tipo Yjs Map)
const yFormData = ydoc.getMap('formData');

// Provider WebSocket (connessione server)
// In produzione, costruisci il tuo server WebSocket
const wsProvider = new WebsocketProvider(
  'wss://demos.yjs.dev', // Server pubblico per demo
  'rxjs-form-demo',      // Nome stanza
  ydoc
);

// Subject RxJS per le modifiche al form
const formChange$ = new Subject<{
  key: string;
  value: any;
  user: string;
}>();

// Converte le modifiche Yjs in stream RxJS
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

// UI Demo
const collaborativeFormDiv = document.createElement('div');
collaborativeFormDiv.style.padding = '20px';
collaborativeFormDiv.style.margin = '10px';
collaborativeFormDiv.style.border = '2px solid #2196F3';
collaborativeFormDiv.style.borderRadius = '8px';
collaborativeFormDiv.style.backgroundColor = '#f5f5f5';
document.body.appendChild(collaborativeFormDiv);

const title = document.createElement('h3');
title.textContent = '🤝 Demo editing collaborativo in tempo reale';
title.style.margin = '0 0 15px 0';
collaborativeFormDiv.appendChild(title);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.placeholder = 'Inserisci nome (sincronizzato con altri utenti)';
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

// Monitoraggio stato connessione
wsProvider.on('status', (event: { status: string }) => {
  if (event.status === 'connected') {
    syncStatus.innerHTML = '🟢 <strong>Connesso</strong> - sincronizzazione in tempo reale con altri utenti';
    syncStatus.style.color = '#4CAF50';
  } else {
    syncStatus.innerHTML = '🔴 <strong>Disconnesso</strong>';
    syncStatus.style.color = '#f44336';
  }
});

// Mostra numero utenti connessi (Awareness API)
wsProvider.awareness.on('change', () => {
  const users = Array.from(wsProvider.awareness.getStates().keys());
  activeUsers.innerHTML = `👥 Utenti attivi: ${users.length}`;
});

// Riflette modifiche locali in Yjs
let isRemoteChange = false;

fromEvent(nameInput, 'input').subscribe(() => {
  if (!isRemoteChange) {
    yFormData.set('name', nameInput.value);
  }
});

// Riflette modifiche remote nell'UI
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

// Imposta valori iniziali
const initialName = yFormData.get('name');
if (initialName) {
  nameInput.value = initialName;
}
```

> [!NOTE] Punti chiave di Yjs
> - **Y.Doc** - Documento condiviso (CRDT)
> - **Y.Map** - Tipo Map condiviso (`{ key: value }`)
> - **WebsocketProvider** - Sincronizzazione via WebSocket
> - **Awareness API** - Informazioni di presenza (utenti connessi, posizione cursore)
> - **observe** - Monitora modifiche e converte in stream RxJS

### Pattern 5: Gestione presenza (condivisione posizione cursore)

Visualizza chi sta modificando cosa.

```typescript
import { throttleTime } from 'rxjs';

interface UserPresence {
  userId: string;
  name: string;
  color: string;
  cursorPosition: number;
  timestamp: number;
}

// Genera colore casuale
function generateRandomColor(): string {
  const colors = ['#f44336', '#E91E63', '#9C27B0', '#673AB7', '#3F51B5', '#2196F3', '#00BCD4', '#009688'];
  return colors[Math.floor(Math.random() * colors.length)];
}

// Il tuo ID utente (generato casualmente)
const myUserId = `user_${Math.random().toString(36).substr(2, 9)}`;
const myColor = generateRandomColor();

// Configurazione Awareness (informazioni presenza)
wsProvider.awareness.setLocalState({
  userId: myUserId,
  name: `Utente${myUserId.slice(-4)}`,
  color: myColor
});

// Rileva modifiche posizione cursore (throttle limita frequenza invio)
fromEvent(nameInput, 'selectionchange').pipe(
  throttleTime(200)
).subscribe(() => {
  const cursorPosition = nameInput.selectionStart || 0;

  wsProvider.awareness.setLocalStateField('cursorPosition', cursorPosition);
});

// Mostra posizione cursore altri utenti
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
        `<span style="color: ${state.color}">● ${state.name}</span> (posizione: ${state.cursorPosition || 0})`
      );
    }
  });

  cursorOverlay.innerHTML = cursors.length > 0
    ? `📍 Altri utenti: ${cursors.join(', ')}`
    : '📍 Nessun altro utente';
});
```

> [!TIP] Punti chiave della gestione presenza
> - **`awareness.setLocalState`** - Condividi le tue informazioni
> - **`awareness.getStates`** - Ottieni informazioni di tutti gli utenti
> - **`throttleTime`** - Limita frequenza invio movimento cursore (200ms)
> - **Codifica colori** - Assegna un colore a ogni utente per migliorare la visibilità

### Pattern 6: Gestione errori e riconnessione

Implementa riconnessione e gestione errori alla disconnessione WebSocket.

```typescript
import { timer, takeUntil, Subject } from 'rxjs';

const disconnect$ = new Subject<void>();

// Rilevamento disconnessione WebSocket
wsProvider.on('connection-close', () => {
  console.warn('⚠️ Disconnessione WebSocket');
  syncStatus.innerHTML = '🟡 <strong>Riconnessione in corso...</strong>';
  syncStatus.style.color = '#FF9800';

  // Tenta riconnessione dopo 5 secondi
  timer(5000).pipe(
    takeUntil(disconnect$)
  ).subscribe(() => {
    console.log('🔄 Tentativo di riconnessione');
    wsProvider.connect();
  });
});

// Gestione errori WebSocket
wsProvider.on('connection-error', (error: Error) => {
  console.error('❌ Errore WebSocket:', error);
  syncStatus.innerHTML = `❌ <strong>Errore:</strong> ${error.message}`;
  syncStatus.style.color = '#f44336';
});

// Cleanup
window.addEventListener('beforeunload', () => {
  disconnect$.next();
  wsProvider.disconnect();
  ydoc.destroy();
});
```

> [!WARNING] Note per l'ambiente di produzione
> - **Server WebSocket proprio** - `wss://demos.yjs.dev` è per sviluppo. In produzione, costruisci [y-websocket-server](https://github.com/yjs/y-websocket)
> - **Autenticazione** - Implementa autenticazione token per connessioni WebSocket
> - **Scaling** - Condividi stato tra server WebSocket con Redis, ecc.
> - **Persistenza** - Memorizza documenti Yjs nel database (`y-leveldb`, `y-indexeddb`)

---

## Riepilogo

Questo articolo ha descritto pattern avanzati per form usando JSON Patch.

### Punti chiave

> [!IMPORTANT] Criteri di selezione del pattern JSON Patch
>
> **Se la gestione form normale è sufficiente:**
> - Numero campi: ~20
> - Salvataggio automatico: non richiesto o invio completo OK
> - Undo/Redo: non richiesto
> - Editing collaborativo: non richiesto
> → Usa [Pattern normale di gestione form](./form-handling.md)
>
> **Se JSON Patch è richiesto:**
> - Numero campi: > 100
> - Salvataggio automatico: richiesto (invia solo differenze)
> - Undo/Redo: richiesto
> - Editing collaborativo: sincronizzazione in tempo reale richiesta
> → Usa i pattern di questo articolo

### Riepilogo pattern di implementazione

| Pattern | Caso d'uso | Tecnologie chiave |
|---------|------------|---------|
| **Salvataggio automatico base** | Invio differenze per form grandi | `pairwise` + `bufferTime` + `concatMap` |
| **Undo/Redo** | Gestione cronologia operazioni | patch inversa + `scan` |
| **Supporto offline** | Supporto disconnessione rete | IndexedDB + accodamento |
| **Editing collaborativo (Yjs)** | Sincronizzazione in tempo reale | Yjs (CRDT) + WebSocket |
| **Gestione presenza** | Condivisione posizione cursore | Awareness API + `throttleTime` |

### Prossimi passi

- **[Elaborazione dati in tempo reale](./real-time-data.md)** - Implementazione dettagliata WebSocket
- **[Gestione errori pratica](./error-handling-patterns.md)** - Gestione errori comunicazione API
- **[Strategie di caching](./caching-strategies.md)** - Gestione cache dati

## Risorse di riferimento

### Specifiche standard

- [RFC 6902: JSON Patch](https://datatracker.ietf.org/doc/html/rfc6902) - Specifica JSON Patch
- [RFC 6901: JSON Pointer](https://datatracker.ietf.org/doc/html/rfc6901) - Specifica JSON Pointer

### Librerie

- [fast-json-patch](https://github.com/Starcounter-Jack/JSON-Patch) - Implementazione JSON Patch (conforme RFC)
- [Yjs](https://docs.yjs.dev/) - Implementazione CRDT (editing collaborativo)
- [ShareDB](https://share.github.io/sharedb/) - Implementazione Operational Transform
- [Automerge](https://automerge.org/) - CRDT specifico per JSON

### Risorse di apprendimento

- [CRDTs: The Hard Parts](https://www.youtube.com/watch?v=x7drE24geUw) - Comprensione approfondita dei CRDT (video)
- [Operational Transformation Explained](https://operational-transformation.github.io/) - Spiegazione dettagliata OT
- [Real-time Collaborative Editing](https://pierrehedkvist.com/posts/1-creating-a-collaborative-editor) - Guida implementazione editing collaborativo
