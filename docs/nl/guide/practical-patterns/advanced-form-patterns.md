---
description: Geavanceerde formulierpatronen met JSON Patch. Beschrijft formulierimplementatiepatronen op bedrijfsniveau, waaronder automatisch opslaan en Ongedaan maken/Opnieuw uitvoeren voor grote formulieren, realtime synchronisatie voor gezamenlijk bewerken, enz.
---

# Geavanceerde formulierpatronen met JSON Patch

Bij het implementeren van grote formulieren en realtime gezamenlijk bewerken, creëert de traditionele benadering van "het hele formulier indienen" prestatie- en gebruikerservaringsuitdagingen.

Dit artikel beschrijft een geavanceerd formulierpatroon met behulp van **JSON Patch (RFC 6902)**. Door alleen de verschillen in te dienen, kan netwerkbandbreedte worden verminderd en kunnen Ongedaan maken/Opnieuw uitvoeren en gezamenlijk bewerken efficiënter worden geïmplementeerd.

## Wat u in dit artikel leert

- Basisprincipes van JSON Patch/Pointer (RFC 6902/6901)
- Grote formulieren automatisch opslaan (op basis van verschillen)
- Implementatie van Ongedaan maken/Opnieuw uitvoeren (reverse patch)
- Realtime synchronisatie voor gezamenlijk bewerken
- Operational Transform (OT) / CRDT fundamenten
- WebSocket en RxJS integratiepatronen
- Conflictoplossing en versiebeheer

> [!TIP] Vereisten
> Dit artikel is gebaseerd op [Hoofdstuk 4: Operators](../operators/index.md), [Formulierverwerkingspatronen](./form-handling.md), [Realtime gegevensverwerking](./real-time-data.md).

> [!NOTE] Wanneer dit patroon nodig is
> - **Grote formulieren** (>100 velden) hebben automatisch opslaan nodig
> - **Ongedaan maken/Opnieuw uitvoeren** functionaliteit vereist
> - **Realtime gezamenlijk bewerken** (zoals Google Docs)
> - **Offline ondersteuning** met diff wachtrij vereist
>
> Voor kleine formulieren (~20 velden) is het [normale formulierverwerkingspatroon](./form-handling.md) voldoende.

## Basisprincipes van JSON Patch/Pointer

### Wat is JSON Patch?

**JSON Patch (RFC 6902)** is een standaardformaat voor het weergeven van wijzigingen in JSON-documenten. U kunt **alleen de wijzigingen** indienen, niet het hele formulier.

```typescript
// Formuliergegevens voor wijziging
const before = {
  profile: {
    name: "田中太郎",
    email: "tanaka@example.com",
    age: 30
  }
};

// Formuliergegevens na wijziging
const after = {
  profile: {
    name: "田中太郎",
    email: "tanaka.updated@example.com", // gewijzigd
    age: 31 // gewijzigd
  }
};

// JSON Patch (alleen diff)
const patch = [
  { op: "replace", path: "/profile/email", value: "tanaka.updated@example.com" },
  { op: "replace", path: "/profile/age", value: 31 }
];
```

> [!NOTE] Zes operaties van JSON Patch
> - `add` - een waarde toevoegen
> - `remove` - een waarde verwijderen
> - `replace` - een waarde vervangen
> - `move` - een waarde verplaatsen
> - `copy` - een waarde kopiëren
> - `test` - een waarde testen (validatie)

### Wat is JSON Pointer?

**JSON Pointer (RFC 6901)** is een padnotatie die verwijst naar een specifieke waarde in een JSON-document.

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

// JSON Pointer voorbeelden
"/user/profile/name"           // → "田中太郎"
"/user/settings/notifications" // → true
"/user/profile"                // → { name: "田中太郎" }
```

### Verschildetectie in RxJS

Combineert `pairwise()` en `fast-json-patch` bibliotheken om automatisch wijzigingen in formulieren te detecteren.

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

// Verschillen detecteren
const patches$ = formData$.pipe(
  pairwise(), // Ontvang paar van [vorige waarde, huidige waarde]
  map(([previous, current]) => compare(previous, current))
);

patches$.subscribe(patches => {
  console.log('Gedetecteerde wijzigingen:', patches);
  // Voorbeeld: [{ op: "replace", path: "/profile/name", value: "田中太郎" }]
});

// Formulierupdate simuleren
formData$.next({
  profile: {
    name: "田中太郎",
    email: "tanaka@example.com",
    age: 30
  }
});
```

> [!TIP] fast-json-patch bibliotheek
> ```bash
> npm install fast-json-patch
> ```
> - Volledig RFC 6902 conform
> - Genereer (`compare`) en pas (`applyPatch`) verschillen toe
> - Reverse patch generatie (voor Ongedaan maken)
> - TypeScript ondersteuning

---

## Automatisch opslaan en Ongedaan maken/Opnieuw uitvoeren voor grote formulieren

Implementeer automatisch opslaan en Ongedaan maken/Opnieuw uitvoeren functies op grote formulieren (bijv. ledenregistratie en productbeheer schermen met 100 velden).

### Implementatiebeleid

**Front End verantwoordelijkheden:**
- Generatie en uitlijning van operaties (wijzigingen)
- Optimistische UI reflectie (onmiddellijke toepassing via `scan`)
- Ongedaan maken/Opnieuw uitvoeren stack beheer (geschiedenis via reverse patch)
- Verzendwachtrijbeheer (volgorde gegarandeerd met `concatMap`)
- Batching (`bufferTime` + compressie)

**Backend verantwoordelijkheden:**
- Versiebeheer (Vector Clock / Timestamp)
- Idempotentiegarantie (dubbele detectie door Request ID)
- Persistentie en audit logging

### Patroon 1: Basis automatisch opslaan

Detecteert formulierwijzigingen en stuurt ze in batches met regelmatige tussenpozen naar de server.

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
  // ... Veronderstel 100+ velden
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

// Formuliergegevensstroom
const formData$ = new BehaviorSubject<LargeFormData>(initialFormData);

// Opslagresultaatstroom
const saveResult$ = new Subject<{ success: boolean; message: string }>();

// Demo statusweergave-element
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

// Automatisch opslaan pipeline
formData$.pipe(
  pairwise(),
  map(([previous, current]) => ({
    patches: compare(previous, current),
    timestamp: Date.now()
  })),
  filter(({ patches }) => patches.length > 0), // Overslaan als er geen wijzigingen zijn
  bufferTime(2000), // Buffer wijzigingen gedurende 2 seconden
  filter(buffer => buffer.length > 0), // Lege buffers overslaan
  map(buffer => {
    // Consolideer alle patches in buffer in één array
    const allPatches = buffer.flatMap(item => item.patches);
    updateStatus(`📦 Batchverwerking ${allPatches.length} wijzigingen...`, '#FF9800');
    return allPatches;
  }),
  concatMap(patches => saveToServer(patches)), // Verzenden met gegarandeerde volgorde
  catchError(error => {
    console.error('Automatisch opslaan fout:', error);
    updateStatus(`❌ Opslaan mislukt: ${error.message}`, '#f44336');
    return of({ success: false, message: error.message });
  })
).subscribe(result => {
  if (result.success) {
    updateStatus(`✅ Automatisch opslaan voltooid (${new Date().toLocaleTimeString()})`, '#4CAF50');
  }
  saveResult$.next(result);
});

// Opslaan naar server (mock implementatie)
function saveToServer(patches: Operation[]): Promise<{ success: boolean; message: string }> {
  console.log('Verzenden naar server:', patches);

  // Werkelijke implementatie voorbeeld:
  // return fetch('/api/forms/12345/patches', {
  //   method: 'PATCH',
  //   headers: { 'Content-Type': 'application/json-patch+json' },
  //   body: JSON.stringify(patches)
  // }).then(res => res.json());

  // Mock: retourneer succes na 500ms
  return new Promise(resolve => {
    setTimeout(() => {
      resolve({
        success: true,
        message: `${patches.length} wijzigingen opgeslagen`
      });
    }, 500);
  });
}

// Demo: Formulierwijzigingen simuleren
const demoButton = document.createElement('button');
demoButton.textContent = 'Formulier wijzigen (Demo)';
demoButton.style.padding = '10px 20px';
demoButton.style.margin = '10px';
demoButton.style.fontSize = '16px';
demoButton.style.cursor = 'pointer';
document.body.appendChild(demoButton);

demoButton.addEventListener('click', () => {
  // Willekeurig velden wijzigen
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
  updateStatus('📝 Formulier gewijzigd...', '#2196F3');
});
```

> [!NOTE] Punten voor automatisch opslaan
> - **`bufferTime(2000)`** - verzend 2 seconden wijzigingen samen (netwerkefficiëntie)
> - **`concatMap`** - garandeert patch volgorde (`mergeMap` kan volgorde verstoren)
> - **`filter`** - wijzigingen overslaan als er geen zijn (vermindert verspilde verzoeken)
> - **Idempotentie** - veilig om dezelfde patch meerdere keren te verzenden (wijs een Request ID toe)

### Patroon 2: Ongedaan maken/Opnieuw uitvoeren implementatie

Implementeer Ongedaan maken/Opnieuw uitvoeren functionaliteit met behulp van de reverse patch.

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

// Geschiedenisbeheer stroom
const historyAction$ = new Subject<HistoryAction>();

const initialState: HistoryState<LargeFormData> = {
  current: initialFormData,
  undoStack: [],
  redoStack: []
};

// Reducer om geschiedenis te beheren
const history$ = historyAction$.pipe(
  scan((state, action) => {
    switch (action.type) {
      case 'APPLY_PATCH':
        if (!action.patches || action.patches.length === 0) return state;

        // Patch toepassen
        const cloned = deepClone(state.current);
        const result = applyPatch(cloned, action.patches, true, false);

        return {
          current: result.newDocument,
          undoStack: [...state.undoStack, action.patches],
          redoStack: [] // Redo stack wissen bij nieuwe operatie
        };

      case 'UNDO':
        if (state.undoStack.length === 0) return state;

        const patchesToUndo = state.undoStack[state.undoStack.length - 1];
        const beforeUndo = deepClone(state.current);

        // Genereer en pas inverse patch toe
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

// Inverse patch generatie (eenvoudige implementatie)
function generateInversePatch(document: any, patches: Operation[]): Operation[] {
  // applyPatch retourneert inverse patch wanneer 4e argument true is
  const cloned = deepClone(document);
  const result = applyPatch(cloned, patches, true, true);
  return result[1] || []; // Verkrijg inverse patch
}

// UI elementen
const historyControlDiv = document.createElement('div');
historyControlDiv.style.padding = '10px';
historyControlDiv.style.margin = '10px';
document.body.appendChild(historyControlDiv);

const undoButton = document.createElement('button');
undoButton.textContent = '↶ Ongedaan maken';
undoButton.style.padding = '10px 20px';
undoButton.style.marginRight = '10px';
undoButton.style.fontSize = '16px';
undoButton.style.cursor = 'pointer';
historyControlDiv.appendChild(undoButton);

const redoButton = document.createElement('button');
redoButton.textContent = '↷ Opnieuw uitvoeren';
redoButton.style.padding = '10px 20px';
redoButton.style.fontSize = '16px';
redoButton.style.cursor = 'pointer';
historyControlDiv.appendChild(redoButton);

const historyInfo = document.createElement('div');
historyInfo.style.marginTop = '10px';
historyInfo.style.fontFamily = 'monospace';
historyInfo.style.fontSize = '14px';
historyControlDiv.appendChild(historyInfo);

// Geschiedenistoestand weergeven
history$.subscribe(state => {
  undoButton.disabled = state.undoStack.length === 0;
  redoButton.disabled = state.redoStack.length === 0;

  historyInfo.innerHTML = `
    📚 Ongedaan maken beschikbaar: ${state.undoStack.length} keer<br>
    📚 Opnieuw uitvoeren beschikbaar: ${state.redoStack.length} keer<br>
    📝 Huidige waarde: ${JSON.stringify(state.current.personalInfo.firstName)}
  `;

  // Formuliergegevens synchroniseren
  formData$.next(state.current);
});

// Knopgebeurtenissen
undoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'UNDO' });
});

redoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'REDO' });
});

// Demo: Patch toepassen knop
const applyPatchButton = document.createElement('button');
applyPatchButton.textContent = 'Wijziging toepassen (Ongedaan maken/Opnieuw uitvoeren test)';
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

> [!TIP] Ongedaan maken/Opnieuw uitvoeren punten
> - **Reverse Patch** - stel het 4e argument van `applyPatch` in op `true` om een reverse patch te krijgen
> - **Stack beheer** - Ongedaan maken stack (eerdere operaties) en Opnieuw uitvoeren stack (ongedaan gemaakte operaties)
> - **Redo wissen bij nieuwe operatie** - reset Redo stack bij nieuwe wijziging
> - **Reducer patroon met `scan`** - Implementeer statusbeheer zoals useReducer in React

### Patroon 3: Offline ondersteuning (IndexedDB wachtrij)

Wachtrijwijzigingen naar IndexedDB wanneer offline en synchroniseer wanneer weer online.

```typescript
import { fromEvent, merge, map, filter, concatMap, catchError, of } from 'rxjs';

// Online/offline status
const online$ = merge(
  fromEvent(window, 'online').pipe(map(() => true)),
  fromEvent(window, 'offline').pipe(map(() => false))
);

// IndexedDB operaties (eenvoudige implementatie)
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
    if (!this.db) throw new Error('DB niet geïnitialiseerd');

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
    if (!this.db) throw new Error('DB niet geïnitialiseerd');

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction([this.storeName], 'readwrite');
      const store = transaction.objectStore(this.storeName);
      const request = store.getAll();

      request.onsuccess = () => {
        const items = request.result;
        // Wissen na ophalen
        store.clear();
        resolve(items.map((item: any) => item.patches));
      };
      request.onerror = () => reject(request.error);
    });
  }
}

const patchQueue = new PatchQueue();
patchQueue.init().then(() => {
  console.log('IndexedDB geïnitialiseerd');
});

// Automatisch opslaan met offline ondersteuning
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
      // Online: verzenden naar server
      try {
        return await saveToServer(patches);
      } catch (error) {
        // Verzenden mislukt: toevoegen aan wachtrij
        await patchQueue.enqueue(patches);
        return { success: false, message: 'Toegevoegd aan offline wachtrij' };
      }
    } else {
      // Offline: toevoegen aan wachtrij
      await patchQueue.enqueue(patches);
      console.log('📴 Offline: Toegevoegd aan wachtrij');
      return { success: false, message: 'Offline' };
    }
  })
).subscribe();

// Synchroniseren bij online herstel
online$.pipe(
  filter(isOnline => isOnline),
  concatMap(async () => {
    console.log('🌐 Weer online: Wachtrij synchroniseren...');
    const queuedPatches = await patchQueue.dequeueAll();

    for (const patches of queuedPatches) {
      await saveToServer(patches);
    }

    return { synced: queuedPatches.length };
  })
).subscribe(result => {
  console.log(`✅ ${result.synced} patches gesynchroniseerd`);
});
```

> [!NOTE] Punten voor offline ondersteuning
> - **IndexedDB** - browserzijde persistente opslag (groter dan LocalStorage)
> - **Wachtrij** - offline, opgeslagen in verzendwachtrij
> - **Online terugkeerdetectie** - `window.addEventListener('online')` om gebeurtenissen te monitoren
> - **Volgorde garantie** - verzend patches in wachtrij in volgorde met `concatMap`

---

## Realtime synchronisatie van gezamenlijk bewerken

Implementeer "realtime gezamenlijk bewerken" waarbij meerdere gebruikers tegelijkertijd hetzelfde formulier bewerken (functies zoals Google Docs, Notion, Figma).

### Operational Transform (OT) en CRDT basisprincipes

Bij realtime co-bewerking is **conflictoplossing** de grootste uitdaging. Er zijn twee belangrijke benaderingen.

#### Operational Transform (OT)

Transformeer wijzigingsoperaties om conflicten op te lossen.

```
Gebruiker A: "hello" → "hello world" (voeg " world" toe aan einde)
Gebruiker B: "hello" → "Hi hello"   (voeg "Hi " toe aan begin)

【Zonder transformatie】
Resultaat: Ofwel "Hi hello world" of "hello world" (laatst schrijven wint)

【Met OT】
- Transformeer operatie van Gebruiker A met operatie van Gebruiker B
- Transformeer operatie van Gebruiker B met operatie van Gebruiker A
→ Resultaat: "Hi hello world" (beide wijzigingen behouden)
```

**Voordelen:**
- Intuïtieve resultaten (behoudt beide wijzigingen)
- Server bepaalt eindstatus

**Nadelen:**
- Complex om te implementeren
- Server vereist

#### CRDT (Conflict-free Replicated Data Types)

Gebruikt wiskundig conflictvrije gegevensstructuren.

```
Wijs unieke ID toe aan elk teken:

Gebruiker A: [h1, e2, l3, l4, o5] → [h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
Gebruiker B: [h1, e2, l3, l4, o5] → [H12, i13, space14, h1, e2, l3, l4, o5]

Sorteer op ID bij samenvoegen:
→ [H12, i13, space14, h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
→ "Hi hello world"
```

**Voordelen:**
- Geen server vereist (P2P mogelijk)
- Gemakkelijke offline ondersteuning

**Nadelen:**
- Hoog geheugengebruik
- Verwijderingsproces is gecompliceerd (Tombstone methode)

> [!TIP] Bibliotheekselectie
> - **OT Implementatie**: [ShareDB](https://github.com/share/sharedb) - Operational Transform
> - **CRDT Implementatie**: [Yjs](https://github.com/yjs/yjs) - Hoge prestatie CRDT (aanbevolen)
> - **CRDT Implementatie**: [Automerge](https://github.com/automerge/automerge) - JSON specifiek

Dit artikel presenteert een voorbeeldimplementatie met behulp van **Yjs (CRDT)**.

### Patroon 4: Gezamenlijk bewerken met Yjs

Combineer Yjs en RxJS om realtime gezamenlijk bewerken te implementeren.

```typescript
import { fromEvent, merge, Subject } from 'rxjs';
import * as Y from 'yjs';
import { WebsocketProvider } from 'y-websocket';

// Yjs document (gedeelde status)
const ydoc = new Y.Doc();

// Te delen formuliergegevens (Yjs Map type)
const yFormData = ydoc.getMap('formData');

// WebSocket provider (serververbinding)
// Bouw uw eigen WebSocket server voor productie
const wsProvider = new WebsocketProvider(
  'wss://demos.yjs.dev', // Publieke demo server
  'rxjs-form-demo',      // Ruimtenaam
  ydoc
);

// RxJS Subject voor formulierwijzigingen
const formChange$ = new Subject<{
  key: string;
  value: any;
  user: string;
}>();

// Converteer Yjs wijzigingen naar RxJS stroom
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
title.textContent = '🤝 Realtime gezamenlijk bewerken Demo';
title.style.margin = '0 0 15px 0';
collaborativeFormDiv.appendChild(title);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.placeholder = 'Voer naam in (gesynchroniseerd met andere gebruikers)';
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

// Verbindingsstatus monitoren
wsProvider.on('status', (event: { status: string }) => {
  if (event.status === 'connected') {
    syncStatus.innerHTML = '🟢 <strong>Verbonden</strong> - Realtime synchronisatie met andere gebruikers';
    syncStatus.style.color = '#4CAF50';
  } else {
    syncStatus.innerHTML = '🔴 <strong>Verbinding verbroken</strong>';
    syncStatus.style.color = '#f44336';
  }
});

// Weergave verbonden gebruikersaantal (Awareness API)
wsProvider.awareness.on('change', () => {
  const users = Array.from(wsProvider.awareness.getStates().keys());
  activeUsers.innerHTML = `👥 Actieve gebruikers: ${users.length}`;
});

// Lokale wijzigingen reflecteren naar Yjs
let isRemoteChange = false;

fromEvent(nameInput, 'input').subscribe(() => {
  if (!isRemoteChange) {
    yFormData.set('name', nameInput.value);
  }
});

// Externe wijzigingen reflecteren naar UI
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

// Initiële waarde instellen
const initialName = yFormData.get('name');
if (initialName) {
  nameInput.value = initialName;
}
```

> [!NOTE] Yjs punten
> - **Y.Doc** - gedeeld document (CRDT)
> - **Y.Map** - Gedeeld Map type (`{ key: value }`)
> - **WebsocketProvider** - Synchronisatie via WebSocket
> - **Awareness API** - aanwezigheidsinformatie (verbonden gebruikers, cursorpositie)
> - **observe** - wijzigingen monitoren en converteren naar RxJS stroom

### Patroon 5: Aanwezigheidsbeheer (cursorpositie delen)

Visualiseer wie waar bewerkt.

```typescript
import { throttleTime } from 'rxjs';

interface UserPresence {
  userId: string;
  name: string;
  color: string;
  cursorPosition: number;
  timestamp: number;
}

// Genereer willekeurige kleur
function generateRandomColor(): string {
  const colors = ['#f44336', '#E91E63', '#9C27B0', '#673AB7', '#3F51B5', '#2196F3', '#00BCD4', '#009688'];
  return colors[Math.floor(Math.random() * colors.length)];
}

// Mijn gebruikers-ID (willekeurige generatie)
const myUserId = `user_${Math.random().toString(36).substr(2, 9)}`;
const myColor = generateRandomColor();

// Awareness instellen (aanwezigheidsinformatie)
wsProvider.awareness.setLocalState({
  userId: myUserId,
  name: `User${myUserId.slice(-4)}`,
  color: myColor
});

// Cursorpositiewijzigingen detecteren (throttle om verzendfrequentie te beperken)
fromEvent(nameInput, 'selectionchange').pipe(
  throttleTime(200)
).subscribe(() => {
  const cursorPosition = nameInput.selectionStart || 0;

  wsProvider.awareness.setLocalStateField('cursorPosition', cursorPosition);
});

// Cursorposities van andere gebruikers weergeven
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
        `<span style="color: ${state.color}">● ${state.name}</span> (positie: ${state.cursorPosition || 0})`
      );
    }
  });

  cursorOverlay.innerHTML = cursors.length > 0
    ? `📍 Andere gebruikers: ${cursors.join(', ')}`
    : '📍 Geen andere gebruikers';
});
```

> [!TIP] Punten van aanwezigheidsbeheer
> - **`awareness.setLocalState`** - deel uw informatie
> - **`awareness.getStates`** - verkrijg informatie over alle gebruikers
> - **`throttleTime`** - beperk hoe vaak cursorbeweging wordt verzonden (200ms)
> - **Kleurcodering** - wijs een kleur toe aan elke gebruiker om zichtbaarheid te verbeteren

### Patroon 6: Foutafhandeling en herverbinding

Implementeer herverbinding en foutafhandeling bij WebSocket verbreking.

```typescript
import { timer, takeUntil, Subject } from 'rxjs';

const disconnect$ = new Subject<void>();

// WebSocket verbrekingsdetectie
wsProvider.on('connection-close', () => {
  console.warn('⚠️ WebSocket verbinding verbroken');
  syncStatus.innerHTML = '🟡 <strong>Opnieuw verbinden...</strong>';
  syncStatus.style.color = '#FF9800';

  // Poging tot herverbinding na 5 seconden
  timer(5000).pipe(
    takeUntil(disconnect$)
  ).subscribe(() => {
    console.log('🔄 Poging tot herverbinding');
    wsProvider.connect();
  });
});

// WebSocket foutafhandeling
wsProvider.on('connection-error', (error: Error) => {
  console.error('❌ WebSocket fout:', error);
  syncStatus.innerHTML = `❌ <strong>Fout:</strong> ${error.message}`;
  syncStatus.style.color = '#f44336';
});

// Opruimen
window.addEventListener('beforeunload', () => {
  disconnect$.next();
  wsProvider.disconnect();
  ydoc.destroy();
});
```

> [!WARNING] Opmerkingen over de productieomgeving
> - **Uw eigen WebSocket server** - `wss://demos.yjs.dev` is voor ontwikkeling. In productie, bouw [y-websocket-server](https://github.com/yjs/y-websocket)
> - **Authenticatie** - implementeer tokenauthenticatie voor WebSocket verbindingen
> - **Schalen** - deel status tussen WebSocket servers via Redis, enz.
> - **Persistentie** - sla Yjs documenten op in database (`y-leveldb`, `y-indexeddb`)

---

## Samenvatting

Dit artikel beschreef een geavanceerd formulierpatroon met behulp van JSON Patch.

### Belangrijke punten

> [!IMPORTANT] JSON Patch patroon selectiecriteria
>
> **Als normale formulierverwerking voldoende is:**
> - Aantal velden: ~20
> - Automatisch opslaan: Niet vereist of hele indiening is OK
> - Ongedaan maken/Opnieuw uitvoeren: Niet vereist
> - Co-bewerking: Niet vereist
> → [Normale formulierverwerkingspatroon](./form-handling.md)
>
> **Als JSON Patch vereist is:**
> - Aantal velden: 100 of meer
> - Automatisch opslaan: Vereist (alleen verschillen verzenden)
> - Ongedaan maken/Opnieuw uitvoeren: vereist
> - Co-bewerking: Realtime synchronisatie vereist
> → Gebruik het patroon in dit artikel

### Samenvatting van implementatiepatronen

| Patroon | Gebruikssituatie | Belangrijkste technologieën |
|---------|----------|-------------------|
| **Basis automatisch opslaan** | Diff transmissie voor grote formulieren | `pairwise` + `bufferTime` + `concatMap` |
| **Ongedaan maken/Opnieuw uitvoeren** | Operatiegeschiedenisbeheer | Reverse patch + `scan` |
| **Offline ondersteuning** | Netwerkverbreking afhandelen | IndexedDB + Wachtrij |
| **Gezamenlijk bewerken (Yjs)** | Realtime synchronisatie | Yjs (CRDT) + WebSocket |
| **Aanwezigheidsbeheer** | Cursorpositie delen | Awareness API + `throttleTime` |

### Volgende stappen

- **[Realtime gegevensverwerking](./real-time-data.md)** - Gedetailleerde WebSocket implementatie
- **[Foutafhandelingspraktijken](./error-handling-patterns.md)** - Foutafhandeling voor API communicatie
- **[Caching strategieën](./caching-strategies.md)** - Gegevenscachebeheer

## Referentiebronnen

### Standaard specificaties

- [RFC 6902: JSON Patch](https://datatracker.ietf.org/doc/html/rfc6902) - JSON Patch Specificatie
- [RFC 6901: JSON Pointer](https://datatracker.ietf.org/doc/html/rfc6901) - JSON Pointer Specificatie

### Bibliotheken

- [fast-json-patch](https://github.com/Starcounter-Jack/JSON-Patch) - JSON Patch implementatie (RFC conform)
- [Yjs](https://docs.yjs.dev/) - CRDT implementatie (gezamenlijk bewerken)
- [ShareDB](https://share.github.io/sharedb/) - Operational Transform implementatie
- [Automerge](https://automerge.org/) - JSON-specifieke CRDT

### Leermiddelen

- [CRDTs: The Hard Parts](https://www.youtube.com/watch?v=x7drE24geUw) - Diep begrip van CRDTs (video)
- [Operational Transformation Explained](https://operational-transformation.github.io/) - Een gedetailleerde uitleg van OT
- [Real-time Collaborative Editing](https://pierrehedkvist.com/posts/1-creating-a-collaborative-editor) - Implementatiegids voor gezamenlijk bewerken
