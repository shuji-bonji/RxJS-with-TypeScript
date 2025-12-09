---
description: "JSON Patch-Formularmuster mit RxJS: Autosave, Undo/Redo für große Formulare, Echtzeit-Kollaboration, Offline-Unterstützung und Verlaufsverfolgung."
---

# Fortgeschrittene Formularmuster mit JSON Patch

Bei der Implementierung großer Formulare oder kollaborativer Echtzeitbearbeitung führt der traditionelle Ansatz „das gesamte Formular absenden" zu Performance- und Benutzererfahrungsproblemen.

Dieser Artikel erklärt fortgeschrittene Formularmuster unter Verwendung von **JSON Patch (RFC 6902)**. Durch das Senden nur der Unterschiede können Sie die Netzwerkbandbreite reduzieren und Undo/Redo sowie kollaborative Bearbeitung effizient implementieren.

## Was Sie in diesem Artikel lernen werden

- Grundlagen von JSON Patch/Pointer (RFC 6902/6901)
- Automatisches Speichern großer Formulare (differenzbasiert)
- Implementierung von Undo/Redo (inverse Patches)
- Echtzeit-Synchronisation für kollaborative Bearbeitung
- Grundlagen von Operational Transform (OT) / CRDT
- Integrationsmuster von WebSocket und RxJS
- Konfliktlösung und Versionsverwaltung

> [!TIP] Voraussetzungen
> Dieser Artikel setzt Kenntnisse von [Kapitel 4: Operatoren](../operators/index.md), [Formularverarbeitungsmuster](./form-handling.md) und [Echtzeitdatenverarbeitung](./real-time-data.md) voraus.

> [!NOTE] Wann diese Muster erforderlich sind
> - **Große Formulare** (100+ Felder) mit automatischem Speichern
> - **Undo/Redo**-Funktionalität ist erforderlich
> - **Kollaborative Echtzeitbearbeitung** (wie Google Docs)
> - **Offline-Unterstützung** mit Diff-Queueing erforderlich
>
> Für kleine Formulare (~20 Felder) sind [normale Formularverarbeitungsmuster](./form-handling.md) ausreichend.

## Grundlagen von JSON Patch/Pointer

### Was ist JSON Patch

**JSON Patch (RFC 6902)** ist ein Standardformat zur Darstellung von Änderungen an JSON-Dokumenten. Sie können nur die **Änderungen** senden, statt das gesamte Formular.

```typescript
// Formulardaten vor der Änderung
const before = {
  profile: {
    name: "田中太郎",
    email: "tanaka@example.com",
    age: 30
  }
};

// Formulardaten nach der Änderung
const after = {
  profile: {
    name: "田中太郎",
    email: "tanaka.updated@example.com", // geändert
    age: 31 // geändert
  }
};

// JSON Patch (Differenz)
const patch = [
  { op: "replace", path: "/profile/email", value: "tanaka.updated@example.com" },
  { op: "replace", path: "/profile/age", value: 31 }
];
```

> [!NOTE] Die 6 JSON Patch-Operationen
> - `add` - Wert hinzufügen
> - `remove` - Wert entfernen
> - `replace` - Wert ersetzen
> - `move` - Wert verschieben
> - `copy` - Wert kopieren
> - `test` - Wert testen (Validierung)

### Was ist JSON Pointer

**JSON Pointer (RFC 6901)** ist eine Pfadnotation zur Referenzierung spezifischer Werte in JSON-Dokumenten.

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

// JSON Pointer Beispiele
"/user/profile/name"           // → "田中太郎"
"/user/settings/notifications" // → true
"/user/profile"                // → { name: "田中太郎" }
```

### Differenzerkennung mit RxJS

Kombinieren Sie `pairwise()` mit der `fast-json-patch`-Bibliothek, um Formularänderungen automatisch zu erkennen.

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

// Unterschiede erkennen
const patches$ = formData$.pipe(
  pairwise(), // Paare aus [vorheriger Wert, aktueller Wert] abrufen
  map(([previous, current]) => compare(previous, current))
);

patches$.subscribe(patches => {
  console.log('Erkannte Änderungen:', patches);
  // Beispiel: [{ op: "replace", path: "/profile/name", value: "田中太郎" }]
});

// Formularaktualisierung simulieren
formData$.next({
  profile: {
    name: "田中太郎",
    email: "tanaka@example.com",
    age: 30
  }
});
```

> [!TIP] fast-json-patch Bibliothek
> ```bash
> npm install fast-json-patch
> ```
> - Vollständig RFC 6902-konform
> - Diff-Generierung (`compare`) und Anwendung (`applyPatch`)
> - Inverse Patch-Generierung (für Undo)
> - TypeScript-Unterstützung

---

## Automatisches Speichern und Undo/Redo für große Formulare

Implementieren Sie automatisches Speichern und Undo/Redo-Funktionalität für große Formulare (z.B. 100-Felder-Mitgliederregistrierung, Produktverwaltungsbildschirme).

### Implementierungsstrategie

**Frontend-Verantwortlichkeiten:**
- Generierung und Sortierung von Operationen (Änderungen)
- Optimistische UI-Reflexion (sofortige Anwendung mit `scan`)
- Undo/Redo-Stack-Verwaltung (Historie mit inversen Patches)
- Sendewarteschlangenverwaltung (Reihenfolgegarantie mit `concatMap`)
- Batching (`bufferTime` + Komprimierung)

**Backend-Verantwortlichkeiten:**
- Versionsverwaltung (Vector Clock / Zeitstempel)
- Idempotenzgarantie (Duplikaterkennung mit Request-ID)
- Persistierung und Audit-Log

### Muster 1: Grundlegendes automatisches Speichern

Erkennen Sie Formularänderungen und senden Sie sie in regelmäßigen Abständen in Batches an den Server.

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
  // ... 100+ Felder angenommen
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

// Stream der Formulardaten
const formData$ = new BehaviorSubject<LargeFormData>(initialFormData);

// Stream der Speicherergebnisse
const saveResult$ = new Subject<{ success: boolean; message: string }>();

// Demo-Statusanzeigeelement
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

// Automatische Speicher-Pipeline
formData$.pipe(
  pairwise(),
  map(([previous, current]) => ({
    patches: compare(previous, current),
    timestamp: Date.now()
  })),
  filter(({ patches }) => patches.length > 0), // Überspringen, wenn keine Änderungen
  bufferTime(2000), // Änderungen über 2 Sekunden puffern
  filter(buffer => buffer.length > 0), // Leeren Puffer überspringen
  map(buffer => {
    // Alle Patches im Puffer zu einem Array zusammenführen
    const allPatches = buffer.flatMap(item => item.patches);
    updateStatus(`📦 Verarbeite ${allPatches.length} Änderungen als Batch...`, '#FF9800');
    return allPatches;
  }),
  concatMap(patches => saveToServer(patches)), // Mit Reihenfolgegarantie senden
  catchError(error => {
    console.error('Fehler beim automatischen Speichern:', error);
    updateStatus(`❌ Speichern fehlgeschlagen: ${error.message}`, '#f44336');
    return of({ success: false, message: error.message });
  })
).subscribe(result => {
  if (result.success) {
    updateStatus(`✅ Automatisches Speichern abgeschlossen (${new Date().toLocaleTimeString()})`, '#4CAF50');
  }
  saveResult$.next(result);
});

// Speichern auf Server (Mock-Implementierung)
function saveToServer(patches: Operation[]): Promise<{ success: boolean; message: string }> {
  console.log('An Server senden:', patches);

  // Tatsächliche Implementierung Beispiel:
  // return fetch('/api/forms/12345/patches', {
  //   method: 'PATCH',
  //   headers: { 'Content-Type': 'application/json-patch+json' },
  //   body: JSON.stringify(patches)
  // }).then(res => res.json());

  // Mock: Erfolg nach 500ms zurückgeben
  return new Promise(resolve => {
    setTimeout(() => {
      resolve({
        success: true,
        message: `${patches.length} Änderungen gespeichert`
      });
    }, 500);
  });
}

// Demo: Formularänderungen simulieren
const demoButton = document.createElement('button');
demoButton.textContent = 'Formular ändern (Demo)';
demoButton.style.padding = '10px 20px';
demoButton.style.margin = '10px';
demoButton.style.fontSize = '16px';
demoButton.style.cursor = 'pointer';
document.body.appendChild(demoButton);

demoButton.addEventListener('click', () => {
  // Zufällige Feldänderungen
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
  updateStatus('📝 Formular wurde geändert...', '#2196F3');
});
```

> [!NOTE] Wichtige Punkte beim automatischen Speichern
> - **`bufferTime(2000)`** - Änderungen über 2 Sekunden zusammenfassen und senden (Netzwerkeffizienz)
> - **`concatMap`** - Garantiert die Reihenfolge der Patches (`mergeMap` kann die Reihenfolge durcheinanderbringen)
> - **`filter`** - Überspringen, wenn keine Änderungen vorliegen (unnötige Anfragen reduzieren)
> - **Idempotenz** - Sicher, auch wenn derselbe Patch mehrmals gesendet wird (Request-ID hinzufügen)

### Muster 2: Undo/Redo-Implementierung

Implementieren Sie Undo/Redo-Funktionalität mit inversen Patches.

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

// Stream zur Verlaufsverwaltung
const historyAction$ = new Subject<HistoryAction>();

const initialState: HistoryState<LargeFormData> = {
  current: initialFormData,
  undoStack: [],
  redoStack: []
};

// Reducer zur Verlaufsverwaltung
const history$ = historyAction$.pipe(
  scan((state, action) => {
    switch (action.type) {
      case 'APPLY_PATCH':
        if (!action.patches || action.patches.length === 0) return state;

        // Patch anwenden
        const cloned = deepClone(state.current);
        const result = applyPatch(cloned, action.patches, true, false);

        return {
          current: result.newDocument,
          undoStack: [...state.undoStack, action.patches],
          redoStack: [] // Redo-Stack bei neuer Operation löschen
        };

      case 'UNDO':
        if (state.undoStack.length === 0) return state;

        const patchesToUndo = state.undoStack[state.undoStack.length - 1];
        const beforeUndo = deepClone(state.current);

        // Inversen Patch generieren und anwenden
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

// Inverse Patch-Generierung (vereinfachte Implementierung)
function generateInversePatch(document: any, patches: Operation[]): Operation[] {
  // applyPatch von fast-json-patch gibt bei true als 4. Argument den inversen Patch zurück
  const cloned = deepClone(document);
  const result = applyPatch(cloned, patches, true, true);
  return result[1] || []; // Inversen Patch abrufen
}

// UI-Elemente
const historyControlDiv = document.createElement('div');
historyControlDiv.style.padding = '10px';
historyControlDiv.style.margin = '10px';
document.body.appendChild(historyControlDiv);

const undoButton = document.createElement('button');
undoButton.textContent = '↶ Rückgängig';
undoButton.style.padding = '10px 20px';
undoButton.style.marginRight = '10px';
undoButton.style.fontSize = '16px';
undoButton.style.cursor = 'pointer';
historyControlDiv.appendChild(undoButton);

const redoButton = document.createElement('button');
redoButton.textContent = '↷ Wiederholen';
redoButton.style.padding = '10px 20px';
redoButton.style.fontSize = '16px';
redoButton.style.cursor = 'pointer';
historyControlDiv.appendChild(redoButton);

const historyInfo = document.createElement('div');
historyInfo.style.marginTop = '10px';
historyInfo.style.fontFamily = 'monospace';
historyInfo.style.fontSize = '14px';
historyControlDiv.appendChild(historyInfo);

// Verlaufsstatus anzeigen
history$.subscribe(state => {
  undoButton.disabled = state.undoStack.length === 0;
  redoButton.disabled = state.redoStack.length === 0;

  historyInfo.innerHTML = `
    📚 Rückgängig möglich: ${state.undoStack.length} Mal<br>
    📚 Wiederholen möglich: ${state.redoStack.length} Mal<br>
    📝 Aktueller Wert: ${JSON.stringify(state.current.personalInfo.firstName)}
  `;

  // Formulardaten synchronisieren
  formData$.next(state.current);
});

// Button-Events
undoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'UNDO' });
});

redoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'REDO' });
});

// Demo: Patch-Anwendungsbutton
const applyPatchButton = document.createElement('button');
applyPatchButton.textContent = 'Änderung anwenden (Undo/Redo-Test)';
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

> [!TIP] Wichtige Punkte bei Undo/Redo
> - **Inverse Patches** - Das 4. Argument von `applyPatch` auf `true` setzen, um inverse Patches zu erhalten
> - **Stack-Verwaltung** - Undo-Stack (vergangene Operationen) und Redo-Stack (rückgängig gemachte Operationen)
> - **Redo bei neuer Operation löschen** - Redo-Stack bei neuen Änderungen zurücksetzen
> - **Reducer-Muster mit `scan`** - Zustandsverwaltung ähnlich wie useReducer in React

### Muster 3: Offline-Unterstützung (IndexedDB-Warteschlange)

Warteschlange von Änderungen in IndexedDB bei Offline-Betrieb und Synchronisation bei Online-Wiederherstellung.

```typescript
import { fromEvent, merge, map, filter, concatMap, catchError, of } from 'rxjs';

// Online/Offline-Status
const online$ = merge(
  fromEvent(window, 'online').pipe(map(() => true)),
  fromEvent(window, 'offline').pipe(map(() => false))
);

// IndexedDB-Operationen (vereinfachte Implementierung)
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
        // Nach Abruf löschen
        store.clear();
        resolve(items.map((item: any) => item.patches));
      };
      request.onerror = () => reject(request.error);
    });
  }
}

const patchQueue = new PatchQueue();
patchQueue.init().then(() => {
  console.log('IndexedDB initialisiert');
});

// Automatisches Speichern mit Offline-Unterstützung
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
      // Online: An Server senden
      try {
        return await saveToServer(patches);
      } catch (error) {
        // Senden fehlgeschlagen: Zur Warteschlange hinzufügen
        await patchQueue.enqueue(patches);
        return { success: false, message: 'Zur Offline-Warteschlange hinzugefügt' };
      }
    } else {
      // Offline: Zur Warteschlange hinzufügen
      await patchQueue.enqueue(patches);
      console.log('📴 Offline: Zur Warteschlange hinzugefügt');
      return { success: false, message: 'Offline' };
    }
  })
).subscribe();

// Synchronisation bei Online-Wiederherstellung
online$.pipe(
  filter(isOnline => isOnline),
  concatMap(async () => {
    console.log('🌐 Online wiederhergestellt: Warteschlange wird synchronisiert...');
    const queuedPatches = await patchQueue.dequeueAll();

    for (const patches of queuedPatches) {
      await saveToServer(patches);
    }

    return { synced: queuedPatches.length };
  })
).subscribe(result => {
  console.log(`✅ ${result.synced} Patches synchronisiert`);
});
```

> [!NOTE] Wichtige Punkte bei Offline-Unterstützung
> - **IndexedDB** - Browser-seitige persistente Speicherung (größere Kapazität als LocalStorage)
> - **Queueing** - Bei Offline-Betrieb in Sendewarteschlange ansammeln
> - **Online-Wiederherstellungserkennung** - Event-Überwachung mit `window.addEventListener('online')`
> - **Reihenfolgegarantie** - Patches in der Warteschlange nacheinander mit `concatMap` senden

---

## Echtzeit-Synchronisation für kollaborative Bearbeitung

Implementieren Sie „kollaborative Echtzeitbearbeitung", bei der mehrere Benutzer gleichzeitig dasselbe Formular bearbeiten (wie Google Docs, Notion, Figma).

### Grundlagen von Operational Transform (OT) und CRDT

Bei kollaborativer Echtzeitbearbeitung ist die **Konfliktlösung** die größte Herausforderung. Es gibt zwei Hauptansätze.

#### Operational Transform (OT)

Lösen Sie Konflikte durch Transformation von Änderungsoperationen.

```
Benutzer A: "hello" → "hello world" (" world" am Ende hinzufügen)
Benutzer B: "hello" → "Hi hello"   ("Hi " am Anfang hinzufügen)

【Ohne Transformation】
Ergebnis: Entweder "Hi hello world" oder "hello world" (last write wins)

【Mit OT】
- Operation von Benutzer A mit Operation von Benutzer B transformieren
- Operation von Benutzer B mit Operation von Benutzer A transformieren
→ Ergebnis: "Hi hello world" (beide Änderungen beibehalten)
```

**Vorteile:**
- Intuitive Ergebnisse (beide Änderungen beibehalten)
- Server entscheidet über finalen Zustand

**Nachteile:**
- Komplexe Implementierung
- Server erforderlich

#### CRDT (Conflict-free Replicated Data Types)

Verwenden Sie mathematisch konfliktfreie Datenstrukturen.

```
Jedes Zeichen erhält eine eindeutige ID:

Benutzer A: [h1, e2, l3, l4, o5] → [h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
Benutzer B: [h1, e2, l3, l4, o5] → [H12, i13, space14, h1, e2, l3, l4, o5]

Beim Merge nach ID sortieren:
→ [H12, i13, space14, h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
→ "Hi hello world"
```

**Vorteile:**
- Kein Server erforderlich (P2P möglich)
- Einfache Offline-Unterstützung

**Nachteile:**
- Höherer Speicherverbrauch
- Komplexe Löschverarbeitung (Tombstone-Methode)

> [!TIP] Bibliotheksauswahl
> - **OT-Implementierung**: [ShareDB](https://github.com/share/sharedb) - Operational Transform
> - **CRDT-Implementierung**: [Yjs](https://github.com/yjs/yjs) - Hochleistungs-CRDT (empfohlen)
> - **CRDT-Implementierung**: [Automerge](https://github.com/automerge/automerge) - JSON-spezialisiert

Dieser Artikel zeigt Implementierungsbeispiele mit **Yjs (CRDT)**.

### Muster 4: Kollaborative Bearbeitung mit Yjs

Kombinieren Sie Yjs und RxJS für kollaborative Echtzeitbearbeitung.

```typescript
import { fromEvent, merge, Subject } from 'rxjs';
import * as Y from 'yjs';
import { WebsocketProvider } from 'y-websocket';

// Yjs-Dokument (gemeinsamer Zustand)
const ydoc = new Y.Doc();

// Zu teilende Formulardaten (Yjs Map-Typ)
const yFormData = ydoc.getMap('formData');

// WebSocket-Provider (Serververbindung)
// Für Produktionsumgebung eigenen WebSocket-Server aufbauen
const wsProvider = new WebsocketProvider(
  'wss://demos.yjs.dev', // Öffentlicher Server für Demo
  'rxjs-form-demo',      // Raumname
  ydoc
);

// RxJS Subject für Formularänderungen
const formChange$ = new Subject<{
  key: string;
  value: any;
  user: string;
}>();

// Yjs-Änderungen in RxJS-Stream umwandeln
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

// Demo-UI
const collaborativeFormDiv = document.createElement('div');
collaborativeFormDiv.style.padding = '20px';
collaborativeFormDiv.style.margin = '10px';
collaborativeFormDiv.style.border = '2px solid #2196F3';
collaborativeFormDiv.style.borderRadius = '8px';
collaborativeFormDiv.style.backgroundColor = '#f5f5f5';
document.body.appendChild(collaborativeFormDiv);

const title = document.createElement('h3');
title.textContent = '🤝 Kollaborative Echtzeitbearbeitung Demo';
title.style.margin = '0 0 15px 0';
collaborativeFormDiv.appendChild(title);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.placeholder = 'Name eingeben (wird mit anderen Benutzern synchronisiert)';
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

// Verbindungsstatus überwachen
wsProvider.on('status', (event: { status: string }) => {
  if (event.status === 'connected') {
    syncStatus.innerHTML = '🟢 <strong>Verbunden</strong> - Echtzeit-Synchronisation mit anderen Benutzern';
    syncStatus.style.color = '#4CAF50';
  } else {
    syncStatus.innerHTML = '🔴 <strong>Getrennt</strong>';
    syncStatus.style.color = '#f44336';
  }
});

// Anzahl verbundener Benutzer anzeigen (Awareness API)
wsProvider.awareness.on('change', () => {
  const users = Array.from(wsProvider.awareness.getStates().keys());
  activeUsers.innerHTML = `👥 Aktive Benutzer: ${users.length} Person(en)`;
});

// Lokale Änderungen in Yjs übernehmen
let isRemoteChange = false;

fromEvent(nameInput, 'input').subscribe(() => {
  if (!isRemoteChange) {
    yFormData.set('name', nameInput.value);
  }
});

// Remote-Änderungen in UI übernehmen
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

// Initialwert setzen
const initialName = yFormData.get('name');
if (initialName) {
  nameInput.value = initialName;
}
```

> [!NOTE] Yjs-Wichtigkeiten
> - **Y.Doc** - Gemeinsames Dokument (CRDT)
> - **Y.Map** - Gemeinsamer Map-Typ (`{ key: value }`)
> - **WebsocketProvider** - Synchronisation über WebSocket
> - **Awareness API** - Präsenzinformationen (verbundene Benutzer, Cursorposition)
> - **observe** - Änderungen überwachen und in RxJS-Stream umwandeln

### Muster 5: Präsenzverwaltung (Cursorposition teilen)

Visualisieren Sie, wer wo bearbeitet.

```typescript
import { throttleTime } from 'rxjs';

interface UserPresence {
  userId: string;
  name: string;
  color: string;
  cursorPosition: number;
  timestamp: number;
}

// Zufällige Farbe generieren
function generateRandomColor(): string {
  const colors = ['#f44336', '#E91E63', '#9C27B0', '#673AB7', '#3F51B5', '#2196F3', '#00BCD4', '#009688'];
  return colors[Math.floor(Math.random() * colors.length)];
}

// Eigene Benutzer-ID (zufällig generiert)
const myUserId = `user_${Math.random().toString(36).substr(2, 9)}`;
const myColor = generateRandomColor();

// Awareness (Präsenzinformationen) konfigurieren
wsProvider.awareness.setLocalState({
  userId: myUserId,
  name: `Benutzer${myUserId.slice(-4)}`,
  color: myColor
});

// Cursorpositionsänderung erkennen (Sendefrequenz mit throttle begrenzen)
fromEvent(nameInput, 'selectionchange').pipe(
  throttleTime(200)
).subscribe(() => {
  const cursorPosition = nameInput.selectionStart || 0;

  wsProvider.awareness.setLocalStateField('cursorPosition', cursorPosition);
});

// Cursorpositionen anderer Benutzer anzeigen
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
        `<span style="color: ${state.color}">● ${state.name}</span> (Position: ${state.cursorPosition || 0})`
      );
    }
  });

  cursorOverlay.innerHTML = cursors.length > 0
    ? `📍 Andere Benutzer: ${cursors.join(', ')}`
    : '📍 Keine anderen Benutzer';
});
```

> [!TIP] Wichtige Punkte zur Präsenzverwaltung
> - **`awareness.setLocalState`** - Eigene Informationen teilen
> - **`awareness.getStates`** - Informationen aller Benutzer abrufen
> - **`throttleTime`** - Sendefrequenz der Cursorbewegungen begrenzen (200ms)
> - **Farbcodierung** - Farbe pro Benutzer zuweisen für bessere Sichtbarkeit

### Muster 6: Fehlerbehandlung und Wiederverbindung

Implementieren Sie Wiederverbindung und Fehlerbehandlung bei WebSocket-Trennung.

```typescript
import { timer, takeUntil, Subject } from 'rxjs';

const disconnect$ = new Subject<void>();

// WebSocket-Trennung erkennen
wsProvider.on('connection-close', () => {
  console.warn('⚠️ WebSocket getrennt');
  syncStatus.innerHTML = '🟡 <strong>Wiederverbindung läuft...</strong>';
  syncStatus.style.color = '#FF9800';

  // Wiederverbindung nach 5 Sekunden versuchen
  timer(5000).pipe(
    takeUntil(disconnect$)
  ).subscribe(() => {
    console.log('🔄 Versuche Wiederverbindung');
    wsProvider.connect();
  });
});

// WebSocket-Fehlerbehandlung
wsProvider.on('connection-error', (error: Error) => {
  console.error('❌ WebSocket-Fehler:', error);
  syncStatus.innerHTML = `❌ <strong>Fehler:</strong> ${error.message}`;
  syncStatus.style.color = '#f44336';
});

// Bereinigung
window.addEventListener('beforeunload', () => {
  disconnect$.next();
  wsProvider.disconnect();
  ydoc.destroy();
});
```

> [!WARNING] Hinweise für Produktionsumgebung
> - **Eigener WebSocket-Server** - `wss://demos.yjs.dev` ist für Entwicklung. Für Produktion [y-websocket-server](https://github.com/yjs/y-websocket) aufbauen
> - **Authentifizierung** - Token-Authentifizierung bei WebSocket-Verbindung implementieren
> - **Skalierung** - Zustand zwischen WebSocket-Servern über Redis etc. teilen
> - **Persistierung** - Yjs-Dokument in Datenbank speichern (`y-leveldb`, `y-indexeddb`)

---

## Zusammenfassung

Dieser Artikel hat fortgeschrittene Formularmuster mit JSON Patch erklärt.

### Wichtige Punkte

> [!IMPORTANT] Auswahlkriterien für JSON Patch-Muster
>
> **Wenn normale Formularverarbeitung ausreicht:**
> - Feldanzahl: ~20 Felder
> - Automatisches Speichern: Nicht erforderlich oder Gesamtsendung OK
> - Undo/Redo: Nicht erforderlich
> - Kollaborative Bearbeitung: Nicht erforderlich
> → Verwenden Sie [normale Formularverarbeitungsmuster](./form-handling.md)
>
> **Wenn JSON Patch erforderlich ist:**
> - Feldanzahl: 100+ Felder
> - Automatisches Speichern: Erforderlich (nur Differenzen senden)
> - Undo/Redo: Erforderlich
> - Kollaborative Bearbeitung: Echtzeit-Synchronisation erforderlich
> → Verwenden Sie die Muster aus diesem Artikel

### Zusammenfassung der Implementierungsmuster

| Muster | Anwendungsfall | Haupttechnologien |
|--------|----------------|-------------------|
| **Grundlegendes automatisches Speichern** | Differenzsendung für große Formulare | `pairwise` + `bufferTime` + `concatMap` |
| **Undo/Redo** | Verlaufsverwaltung | Inverse Patches + `scan` |
| **Offline-Unterstützung** | Netzwerkausfall-Behandlung | IndexedDB + Queueing |
| **Kollaborative Bearbeitung (Yjs)** | Echtzeit-Synchronisation | Yjs (CRDT) + WebSocket |
| **Präsenzverwaltung** | Cursorposition teilen | Awareness API + `throttleTime` |

### Nächste Schritte

- **[Echtzeitdatenverarbeitung](./real-time-data.md)** - Detaillierte WebSocket-Implementierung
- **[Fehlerbehandlung in der Praxis](./error-handling-patterns.md)** - Fehlerbehandlung bei API-Kommunikation
- **[Cache-Strategien](./caching-strategies.md)** - Datencache-Verwaltung

## Referenzressourcen

### Standardspezifikationen

- [RFC 6902: JSON Patch](https://datatracker.ietf.org/doc/html/rfc6902) - JSON Patch-Spezifikation
- [RFC 6901: JSON Pointer](https://datatracker.ietf.org/doc/html/rfc6901) - JSON Pointer-Spezifikation

### Bibliotheken

- [fast-json-patch](https://github.com/Starcounter-Jack/JSON-Patch) - JSON Patch-Implementierung (RFC-konform)
- [Yjs](https://docs.yjs.dev/) - CRDT-Implementierung (kollaborative Bearbeitung)
- [ShareDB](https://share.github.io/sharedb/) - Operational Transform-Implementierung
- [Automerge](https://automerge.org/) - JSON-spezialisiertes CRDT

### Lernressourcen

- [CRDTs: The Hard Parts](https://www.youtube.com/watch?v=x7drE24geUw) - Tiefes Verständnis von CRDT (Video)
- [Operational Transformation Explained](https://operational-transformation.github.io/) - Detaillierte OT-Erklärung
- [Real-time Collaborative Editing](https://pierrehedkvist.com/posts/1-creating-a-collaborative-editor) - Implementierungshandbuch für kollaborative Bearbeitung
